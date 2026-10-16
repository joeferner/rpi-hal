//! An `embassy-usb-driver` *host* controller over `rpi-hal`'s DWC2
//! host channels, so
//! [`embassy-usb`](https://docs.rs/embassy-usb)'s host stack
//! (`embassy_usb::host`) — its enumeration and its host class drivers —
//! runs on a Pi.
//!
//! Three types, mirroring the three traits:
//!
//! - [`usb::Dwc2HostController`]
//!   ([`UsbHostController`](embassy_usb_driver::host::UsbHostController))
//!   owns the bus-level operations — waiting for a device on the root
//!   port, and resetting it.
//! - [`usb::Dwc2Allocator`]
//!   ([`UsbHostAllocator`](embassy_usb_driver::host::UsbHostAllocator))
//!   hands out pipes, one per endpoint, each taking a host channel from
//!   the controller's pool.
//! - [`usb::Dwc2Pipe`]
//!   ([`UsbPipe`](embassy_usb_driver::host::UsbPipe)) runs the
//!   transfers.
//!
//! # Why this lives here and not in `rpi-hal`
//!
//! `embassy-usb-driver` is trait definitions only, so by the standard
//! this crate's own documentation sets, the adapter would belong in the
//! HAL beside the `embassy-net-driver` one. What moves it here is
//! `embassy-time`: the
//! [`UsbPipe`](embassy_usb_driver::host::UsbPipe) contract is written in
//! wall-clock terms — a control transfer retries a NAK until its
//! [`TimeoutConfig`](embassy_usb_driver::host::TimeoutConfig) expires,
//! an interrupt endpoint re-polls once per `bInterval` *milliseconds* —
//! and `rpi-hal`'s USB layer deliberately has no clock, only bus time
//! (see its `usb::dwc2::asynch`). Somebody has to supply one, and a
//! dependency on `embassy-time` is a dependency on a link-time
//! singleton — exactly the thing this crate exists to keep out of the
//! HAL.
//!
//! So the split is: `rpi-hal` answers "has the hardware finished?", and
//! this module answers "how long are we prepared to wait?".
//!
//! # What an application must provide
//!
//! Beyond [the crate-level requirements](crate), which this inherits:
//!
//! - **The USB interrupt, routed.** `Lic::enable_usb_irq()` at the
//!   interrupt controller, and `rpi_hal::usb::dwc2::on_irq()` called
//!   from the application's `__irq_handler` when `Lic::is_usb_pending()`
//!   reports the line. Nothing here completes without it — every
//!   transfer parks on that interrupt — and since `rpi-hal` supplies
//!   only a weak no-op handler, omitting it looks like a hang at the
//!   first transfer rather than an error.
//! - **The controller and a timer, at `'d`.** Both are borrowed for the
//!   lifetime of the bus. `Dwc2Host` must already be powered up
//!   (`usb::power_on` through the mailbox) and initialised.
//!
//! # Limits
//!
//! - **A control data stage is capped at
//!   [`MAX_TRANSFER_LEN`](rpi_hal::usb::dwc2::MAX_TRANSFER_LEN)** (256
//!   bytes), the size of the DMA buffer each channel stages transfers
//!   through; a longer one fails with
//!   [`PipeError::BufferOverflow`](embassy_usb_driver::host::PipeError::BufferOverflow).
//!   That covers a boot keyboard's descriptors and the on-board hub's
//!   comfortably, but a composite device with a large configuration
//!   descriptor would need a bigger buffer in the HAL.
//! - **A bulk transfer DMAs straight from the caller's buffer**, so the
//!   buffer must be cache-line aligned and a whole number of cache lines
//!   long; anything else is refused rather than risking a neighbouring
//!   line being discarded by the post-transfer invalidate. Control and
//!   interrupt transfers have no such requirement — they go through the
//!   channel's own aligned buffer.
//! - **Isochronous transfers are not implemented**, the HAL having no
//!   isochronous channel support to build them on.
//! - **Overcurrent is never reported.** The root port's overcurrent
//!   status isn't exposed by the HAL, so
//!   [`DeviceEvent::Overcurrent`](embassy_usb_driver::host::DeviceEvent::Overcurrent)
//!   simply never occurs.

use core::marker::PhantomData;

use embassy_time::{Duration, Timer as TimeoutTimer, with_timeout};
use embassy_usb_driver::host::{
    DeviceEvent, HostError, PipeError, SplitInfo, SplitSpeed, TimeoutConfig, UsbHostAllocator,
    UsbHostController, UsbPipe, pipe,
};
use embassy_usb_driver::{EndpointInfo, EndpointType, Speed};
use rpi_hal::timer::Timer;
use rpi_hal::usb::dwc2::{
    Channel, ControlEndpoint, Dwc2Host, MAX_TRANSFER_LEN, SplitTarget, TransferError,
};

/// `HPRT.PSPD` encoding for a low-speed device — see
/// `Dwc2Host::port_speed`.
const PORT_SPEED_LOW: u8 = 2;
/// `HPRT.PSPD` encoding for a full-speed device.
const PORT_SPEED_FULL: u8 = 1;

/// Cache line size the DWC2's DMA engine works against, from the HAL's
/// own bulk-transfer contract: a bulk buffer must start on one of these
/// and occupy whole ones.
const CACHE_LINE: usize = 64;

/// How long to leave between retries of a bulk transfer the device is
/// NAKing.
///
/// The [`UsbPipe`] contract says bulk retries indefinitely and that a
/// caller wanting a deadline wraps the future in one. Retrying flat out
/// would be within the letter of that, and each attempt does yield (it
/// waits on the channel-halt interrupt), but a device that NAKs for a
/// second would spend that second monopolising the bus against every
/// other endpoint. One frame's pause is the usual host behaviour.
const BULK_NAK_RETRY: Duration = Duration::from_millis(1);

/// Maps a HAL transfer failure onto the pipe error that means the same
/// thing.
///
/// [`PipeError::Timeout`] is reserved for something that genuinely ran
/// out of time. In particular [`TransferError::Halted`] — a channel that
/// stopped with no status bit explaining why — is reported as
/// [`PipeError::BadResponse`] instead, even though "the transfer didn't
/// finish" describes both. The two have completely different causes (a
/// deadline this crate imposed, versus the controller giving up on the
/// wire), and folding them together means the one error a caller can
/// actually see says nothing about which happened.
///
/// The NAK variants have no mapping and shouldn't arrive: a NAK is flow
/// control, not a failure, and every path here retries it rather than
/// returning it. If one does escape, the retry budget ran out with the
/// device still not ready, which is a timeout.
fn map_error(error: TransferError) -> PipeError {
    match error {
        TransferError::Stall => PipeError::Stall,
        TransferError::TransactionError => PipeError::BadResponse,
        TransferError::Babble => PipeError::Babble,
        TransferError::DataToggleError => PipeError::DataToggleError,
        TransferError::Halted | TransferError::FrameOverrun => PipeError::BadResponse,
        TransferError::Timeout | TransferError::Nak | TransferError::NakTimeout => {
            PipeError::Timeout
        }
    }
}

/// The SETUP packet of the most recent control transfer a device
/// answered with STALL — see [`last_stalled_setup`].
static LAST_STALLED_SETUP: critical_section::Mutex<core::cell::Cell<[u8; 8]>> =
    critical_section::Mutex::new(core::cell::Cell::new([0; 8]));

/// Which stage of that transfer the device refused — see
/// [`last_stalled_stage`].
static LAST_STALLED_STAGE: critical_section::Mutex<core::cell::Cell<&'static str>> =
    critical_section::Mutex::new(core::cell::Cell::new(""));

/// The 8-byte SETUP packet of the most recent control transfer that a
/// device answered with STALL, or all zeroes if none has.
///
/// A STALL is the device saying "I will not do that", and the only thing
/// that makes it actionable is knowing *what* was asked. By the time the
/// refusal reaches an application it has become
/// [`PipeError::Stall`] on some enumeration step, with the request
/// itself long discarded — and an enumerator issues several, so "a
/// device stalled" narrows things very little. `bRequest` is byte 1:
/// `0x05` is SET_ADDRESS, `0x06` GET_DESCRIPTOR, `0x09`
/// SET_CONFIGURATION.
///
/// Diagnostic only. Nothing here reads it, it is not cleared on success,
/// and with several pipes in use it reports whichever stalled last.
pub fn last_stalled_setup() -> [u8; 8] {
    critical_section::with(|cs| LAST_STALLED_SETUP.borrow(cs).get())
}

/// Which stage of the last refused control transfer the device answered
/// with STALL: `"SETUP"`, `"DATA"`, `"STATUS"`, or `""` if none has.
///
/// The stage matters as much as the request. A device that refuses the
/// SETUP rejects the request outright; one that refuses mid-DATA changed
/// its mind partway; and one that refuses the STATUS stage accepted the
/// request and delivered every byte, then declined to acknowledge — which
/// usually means host and device disagree about how much data the
/// transfer carried, not that the request was unwelcome.
pub fn last_stalled_stage() -> &'static str {
    critical_section::with(|cs| LAST_STALLED_STAGE.borrow(cs).get())
}

/// Records `setup` and `stage` if `error` is a STALL, so
/// [`last_stalled_setup`] and [`last_stalled_stage`] can report what the
/// device refused and at what point.
fn note_stall(error: TransferError, setup: &[u8; 8], stage: &'static str) {
    if error == TransferError::Stall {
        critical_section::with(|cs| {
            LAST_STALLED_SETUP.borrow(cs).set(*setup);
            LAST_STALLED_STAGE.borrow(cs).set(stage);
        });
    }
}

/// Whether a transfer failure is worth simply trying again.
///
/// Two conditions are not really failures, and neither should reach a
/// caller as one:
///
/// - [`TransferError::Nak`] is flow control — "nothing for you yet".
/// - [`TransferError::FrameOverrun`] means the transaction was armed too
///   late in its microframe to finish, so the controller dropped it. It
///   says nothing about the device, and the next interval starts on a
///   fresh microframe.
///
/// Reporting either would turn ordinary bus scheduling into an error the
/// class driver above has to interpret, and class drivers reasonably
/// treat an error as "this device is broken".
fn retryable(error: TransferError) -> bool {
    matches!(error, TransferError::Nak | TransferError::FrameOverrun)
}

/// Whether a *control* transfer failure is worth trying again, which is
/// a slightly wider question than [`retryable`].
///
/// [`TransferError::DataToggleError`] is recoverable here and nowhere
/// else. A control transfer is retried from its SETUP, and a SETUP
/// restarts the sequence with a defined toggle at both ends — so host
/// and device cannot stay out of step. A bulk or interrupt endpoint has
/// no equivalent: its toggle persists across transfers, so retrying a
/// toggle error there repeats it forever, and the caller has to
/// intervene with `reset_data_toggle`.
///
/// Treating it as fatal on a control transfer is what turns a recoverable
/// hiccup into a failed enumeration.
fn retryable_control(error: TransferError) -> bool {
    retryable(error) || error == TransferError::DataToggleError
}

/// Converts a [`TimeoutConfig`] duration (`core::time`) to an
/// `embassy-time` one.
fn timeout_duration(duration: core::time::Duration) -> Duration {
    // Microseconds rather than millis: the tick rate here is 1MHz, so
    // this is lossless for any timeout a device could sensibly need, and
    // saturating covers the theoretical `u128` overflow without a panic
    // in a driver.
    Duration::from_micros(duration.as_micros().min(u64::MAX as u128) as u64)
}

/// True if `buf` satisfies the HAL's bulk-DMA requirement: cache-line
/// aligned, and a whole number of cache lines long.
///
/// Both halves matter. The alignment is what keeps the transfer's own
/// data in lines it owns; the length is what keeps the invalidate that
/// follows a bulk IN from discarding a *neighbouring* dirty line that
/// happens to share the last one.
fn bulk_buffer_ok(buf: &[u8]) -> bool {
    (buf.as_ptr() as usize).is_multiple_of(CACHE_LINE) && buf.len().is_multiple_of(CACHE_LINE)
}

/// A [`UsbHostController`] over a `rpi-hal` DWC2 controller.
///
/// Pair it with `embassy_usb::host::bus` to get the bus controller and
/// handle the host stack works through; see this module's documentation
/// for what the application still has to wire up.
pub struct Dwc2HostController<'d> {
    host: &'d Dwc2Host,
    timer: &'d Timer,
}

impl<'d> Dwc2HostController<'d> {
    /// Wraps an initialised, powered-up [`Dwc2Host`].
    ///
    /// `timer` is used for the root-port reset's spec-mandated settling
    /// times, and to bound the abort of a cancelled transfer; the
    /// transfers themselves are driven by interrupts, not by it.
    pub fn new(host: &'d Dwc2Host, timer: &'d Timer) -> Self {
        Self { host, timer }
    }

    /// Resets the root port and reports the speed the device settled on,
    /// or `None` if the port didn't enable.
    fn reset_and_detect(&self) -> Option<Speed> {
        self.host.reset_port(self.timer);
        // The reset just set the port's own change bits, latching a
        // "port changed" that is entirely this call's own doing.
        // Claiming it here is what keeps the next wait an actual wait —
        // see `Dwc2Host::clear_port_change`.
        self.host.clear_port_change();
        if !self.host.port_enabled() {
            return None;
        }
        Some(match self.host.port_speed() {
            PORT_SPEED_LOW => Speed::Low,
            PORT_SPEED_FULL => Speed::Full,
            _ => Speed::High,
        })
    }
}

impl<'d> UsbHostController<'d> for Dwc2HostController<'d> {
    /// See [`Dwc2Allocator`].
    type Allocator = Dwc2Allocator<'d>;

    /// Hands out an allocator sharing this controller's channel pool.
    /// Cheap and copyable — it borrows, owns nothing.
    fn allocator(&self) -> Self::Allocator {
        Dwc2Allocator {
            host: self.host,
            timer: self.timer,
        }
    }

    /// Waits for a device to attach to or detach from the root port,
    /// driving the bus reset to completion on an attach as the trait
    /// requires.
    ///
    /// The port-change wait is interrupt-driven, but the reset itself is
    /// not: `Dwc2Host::reset_port` holds `PRST` for 50ms and then waits
    /// out the 10ms recovery time, both spec-mandated, and both spent
    /// blocking. That costs one 60ms stall per attach — noticeable, but
    /// it happens once per device and the alternative is reimplementing
    /// the reset sequence here purely to spell its two delays as awaits.
    ///
    /// A port change that reports neither a connection nor a successful
    /// enable is swallowed and the wait resumes, so a device that fails
    /// to come up doesn't surface as a phantom event.
    async fn wait_for_device_event(&mut self) -> DeviceEvent {
        loop {
            self.host.wait_for_port_change().await;

            if !self.host.port_connected() {
                return DeviceEvent::Disconnected;
            }
            if let Some(speed) = self.reset_and_detect() {
                return DeviceEvent::Connected(speed);
            }
        }
    }

    /// Resets the root port, invalidating every device address on the
    /// bus.
    ///
    /// Blocking for the same 60ms, and for the same reason, as
    /// [`Self::wait_for_device_event`]'s reset.
    async fn bus_reset(&mut self) {
        self.reset_and_detect();
    }
}

/// Pipe allocator for [`Dwc2HostController`], drawing on its host
/// channel pool.
///
/// Copyable because it is just a pair of borrows; the pool it allocates
/// from lives in the [`Dwc2Host`].
#[derive(Clone, Copy)]
pub struct Dwc2Allocator<'d> {
    host: &'d Dwc2Host,
    timer: &'d Timer,
}

impl<'d> UsbHostAllocator<'d> for Dwc2Allocator<'d> {
    /// See [`Dwc2Pipe`].
    type Pipe<T: pipe::Type, D: pipe::Direction> = Dwc2Pipe<'d, T, D>;

    /// Takes a host channel from the controller's pool and binds it to
    /// one device endpoint.
    ///
    /// One channel per pipe, held until the pipe is dropped, which is
    /// what makes concurrent endpoints work — the channels are
    /// independent hardware and the core arbitrates between them. It
    /// also makes them finite: this SoC has eight, and running out is
    /// reported as [`HostError::OutOfPipes`] rather than queued behind
    /// an existing pipe. Scope a pipe used for a one-off request so it
    /// gives its channel back.
    ///
    /// `split` is what makes a full- or low-speed device behind the
    /// on-board hub reachable at all: it names the hub's transaction
    /// translator, and every transfer on the pipe is then issued as a
    /// split transaction. For a device the host reaches directly, the
    /// device's speed is the root port's.
    fn alloc_pipe<T: pipe::Type, D: pipe::Direction>(
        &self,
        addr: u8,
        endpoint: &EndpointInfo,
        split: Option<SplitInfo>,
    ) -> Result<Self::Pipe<T, D>, HostError> {
        if T::ep_type() != endpoint.ep_type {
            return Err(HostError::Other(
                "pipe type does not match the endpoint descriptor",
            ));
        }
        if endpoint.ep_type == EndpointType::Isochronous {
            return Err(HostError::Other("isochronous transfers are not supported"));
        }

        let channel = self.host.alloc_channel().ok_or(HostError::OutOfPipes)?;

        let (low_speed, split) = match split {
            Some(info) => (
                info.device_speed() == SplitSpeed::Low,
                Some(SplitTarget {
                    hub_address: info.hub_addr(),
                    port: info.port(),
                }),
            ),
            // Reached directly, so the device runs at whatever the root
            // port negotiated — there is nothing between it and the host
            // to run at a different speed.
            None => (self.host.port_speed() == PORT_SPEED_LOW, None),
        };

        Ok(Dwc2Pipe {
            channel,
            timer: self.timer,
            endpoint: ControlEndpoint {
                address: addr,
                low_speed,
                max_packet_size: endpoint.max_packet_size,
                split,
            },
            endpoint_number: u8::from(endpoint.addr) & 0x0f,
            interval_ms: endpoint.interval_ms.max(1),
            toggle: false,
            timeout: TimeoutConfig::default(),
            _marker: PhantomData,
        })
    }
}

/// A [`UsbPipe`] over one host channel, bound to one device endpoint.
///
/// Created by [`Dwc2Allocator::alloc_pipe`], which is also where
/// everything fixed about the endpoint — the device's address, speed and
/// split route, the endpoint's number and max packet size — is captured.
/// What changes over the pipe's life is the data toggle and the control
/// timeouts.
pub struct Dwc2Pipe<'d, T: pipe::Type, D: pipe::Direction> {
    channel: Channel<'d>,
    timer: &'d Timer,
    /// The device this pipe talks to, as the HAL wants it: address,
    /// speed, this endpoint's max packet size, and the split route.
    endpoint: ControlEndpoint,
    endpoint_number: u8,
    /// `bInterval` in milliseconds, floored at 1 — the pacing between
    /// polls of an interrupt endpoint. A descriptor claiming 0 would
    /// otherwise mean "poll flat out".
    interval_ms: u8,
    /// The endpoint's persistent DATA0/DATA1 toggle. Control transfers
    /// don't use it (each stage's PID is fixed by the transfer's shape);
    /// bulk and interrupt endpoints carry it across transfers, which is
    /// why it lives on the pipe rather than on a call.
    toggle: bool,
    timeout: TimeoutConfig,
    _marker: PhantomData<(T, D)>,
}

impl<T: pipe::Type, D: pipe::Direction> Dwc2Pipe<'_, T, D> {
    /// Runs one complete attempt at a control transfer — SETUP, the
    /// data stage if there is one, then STATUS — giving up on the first
    /// stage that fails.
    ///
    /// No stage is retried in place, and that is the point. A control
    /// transfer is a conversation with state on both ends: the device
    /// tracks how far through the data stage it is, and its data toggle
    /// with it. Re-issuing a data stage that already moved some packets
    /// starts the host again from the first packet and the DATA1 toggle
    /// while the device carries on from where it was, which is not a
    /// retry but a different, malformed transfer — and a device may
    /// answer it with a STALL. That is invisible on a single-packet
    /// stage, which is why it hides until something asks for a
    /// descriptor that spans several: 34 bytes to a low-speed device is
    /// five packets, and the first four go missing on the restart.
    ///
    /// A failed control transfer is therefore retried from its SETUP or
    /// not at all — see [`Self::control`].
    ///
    /// `data` is `None` for a no-data transfer, and its direction
    /// decides the status stage's: opposite the data stage, or IN when
    /// there wasn't one.
    async fn control_attempt(
        &mut self,
        setup: &[u8; 8],
        data: Option<Data<'_>>,
    ) -> Result<usize, TransferError> {
        self.channel
            .control_setup_async(self.endpoint, setup, self.timer)
            .await
            .inspect_err(|e| note_stall(*e, setup, "SETUP"))?;

        // The status stage runs opposite the data stage. A transfer that
        // read from the device acknowledges with an OUT; one that wrote
        // to it, or carried no data at all, with an IN. Decided here
        // because running the data stage consumes `data`.
        let status_in = !matches!(data, Some(Data::In(_)));

        let received = match data {
            Some(Data::In(buf)) => self
                .channel
                .control_data_in_async(self.endpoint, buf, self.timer)
                .await
                .inspect_err(|e| note_stall(*e, setup, "DATA"))?,
            Some(Data::Out(buf)) => {
                self.channel
                    .control_data_out_async(self.endpoint, buf, self.timer)
                    .await
                    .inspect_err(|e| note_stall(*e, setup, "DATA"))?;
                0
            }
            None => 0,
        };

        if status_in {
            self.channel
                .control_status_in_async(self.endpoint, self.timer)
                .await
                .inspect_err(|e| note_stall(*e, setup, "STATUS"))?;
        } else {
            self.channel
                .control_status_out_async(self.endpoint, self.timer)
                .await
                .inspect_err(|e| note_stall(*e, setup, "STATUS"))?;
        }
        Ok(received)
    }

    /// Runs a control transfer, restarting it from the SETUP for as long
    /// as it fails in a way worth retrying.
    ///
    /// Needs no iteration cap of its own: the caller wraps this in a
    /// deadline, and every attempt waits on the channel-halt interrupt,
    /// so the loop yields and the timeout is what ends it.
    async fn control(
        &mut self,
        setup: &[u8; 8],
        data: Option<Data<'_>>,
    ) -> Result<usize, PipeError> {
        // Reborrowed per attempt rather than moved, so a retry gets the
        // caller's buffer back rather than a consumed one.
        let mut data = data;
        loop {
            let attempt = match &mut data {
                Some(Data::In(buf)) => Some(Data::In(buf)),
                Some(Data::Out(buf)) => Some(Data::Out(buf)),
                None => None,
            };
            match self.control_attempt(setup, attempt).await {
                Ok(received) => return Ok(received),
                Err(e) if retryable_control(e) => continue,
                Err(e) => return Err(map_error(e)),
            }
        }
    }

    /// The deadline a control transfer of this shape gets, from the
    /// pipe's [`TimeoutConfig`].
    fn control_timeout(&self, has_data: bool) -> Duration {
        timeout_duration(if has_data {
            self.timeout.data_timeout
        } else {
            self.timeout.no_data_timeout
        })
    }
}

/// A control transfer's data stage, if it has one — which also settles
/// the direction of its status stage.
enum Data<'a> {
    In(&'a mut [u8]),
    Out(&'a [u8]),
}

impl<T: pipe::Type, D: pipe::Direction> UsbPipe<T, D> for Dwc2Pipe<'_, T, D> {
    /// Runs a device-to-host control transfer, reading the data stage
    /// into `buf` and returning how many bytes arrived — which can be
    /// fewer than asked for, a device ending the stage early with a
    /// short packet.
    ///
    /// Bounded by [`TimeoutConfig::data_timeout`] (or `no_data_timeout`
    /// for an empty `buf`); the deadline is what stops a device that
    /// NAKs forever.
    async fn control_in(&mut self, setup: &[u8; 8], buf: &mut [u8]) -> Result<usize, PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsIn,
    {
        if buf.len() > MAX_TRANSFER_LEN {
            return Err(PipeError::BufferOverflow);
        }
        let deadline = self.control_timeout(!buf.is_empty());
        let data = if buf.is_empty() {
            None
        } else {
            Some(Data::In(buf))
        };
        with_timeout(deadline, self.control(setup, data))
            .await
            .map_err(|_| PipeError::Timeout)?
    }

    /// Runs a host-to-device control transfer, sending `buf` as the data
    /// stage. Bounded like [`Self::control_in`].
    async fn control_out(&mut self, setup: &[u8; 8], buf: &[u8]) -> Result<(), PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsOut,
    {
        if buf.len() > MAX_TRANSFER_LEN {
            return Err(PipeError::BufferOverflow);
        }
        let deadline = self.control_timeout(!buf.is_empty());
        let data = if buf.is_empty() {
            None
        } else {
            Some(Data::Out(buf))
        };
        with_timeout(deadline, self.control(setup, data))
            .await
            .map_err(|_| PipeError::Timeout)?
            .map(|_| ())
    }

    /// Reads from a bulk or interrupt IN endpoint, returning when the
    /// device actually has something.
    ///
    /// On an **interrupt** endpoint a NAK means "no report this
    /// interval", so the poll is repeated one `bInterval` later —
    /// yielding to the executor in between, which is the whole point of
    /// the async path.
    ///
    /// On a **bulk** endpoint a NAK means "not ready", and the contract
    /// is to retry indefinitely; wrap this in `embassy_time::with_timeout`
    /// to impose a deadline. Dropping the future aborts the transfer.
    ///
    /// An interrupt transfer is capped at [`MAX_TRANSFER_LEN`]; a
    /// bulk one instead requires a cache-line aligned buffer occupying
    /// whole cache lines, since it DMAs straight into it. Either way a
    /// buffer that can't be used is [`PipeError::BufferOverflow`].
    async fn request_in(&mut self, buf: &mut [u8]) -> Result<usize, PipeError>
    where
        D: pipe::IsIn,
    {
        match T::ep_type() {
            EndpointType::Interrupt => {
                if buf.len() > MAX_TRANSFER_LEN {
                    return Err(PipeError::BufferOverflow);
                }
                loop {
                    match self
                        .channel
                        .interrupt_in_async(
                            self.endpoint,
                            self.endpoint_number,
                            &mut self.toggle,
                            buf,
                            self.timer,
                        )
                        .await
                    {
                        Ok(received) => return Ok(received),
                        Err(e) if retryable(e) => {
                            TimeoutTimer::after(Duration::from_millis(u64::from(self.interval_ms)))
                                .await;
                        }
                        Err(e) => return Err(map_error(e)),
                    }
                }
            }
            EndpointType::Bulk => {
                if !bulk_buffer_ok(buf) {
                    return Err(PipeError::BufferOverflow);
                }
                loop {
                    match self
                        .channel
                        .bulk_in_async(
                            self.endpoint,
                            self.endpoint_number,
                            &mut self.toggle,
                            buf,
                            self.timer,
                        )
                        .await
                    {
                        Ok(received) => return Ok(received),
                        Err(e) if retryable(e) => TimeoutTimer::after(BULK_NAK_RETRY).await,
                        Err(e) => return Err(map_error(e)),
                    }
                }
            }
            // A control pipe's transfers all go through `control_in`,
            // and isochronous is refused at allocation.
            _ => Err(PipeError::BadResponse),
        }
    }

    /// Writes to a bulk or interrupt OUT endpoint, with the same NAK
    /// handling and the same buffer rules as [`Self::request_in`].
    ///
    /// `ensure_transaction_end` appends a zero-length packet when the
    /// data ended exactly on a max-packet boundary — without it the
    /// device cannot tell a completed transfer from one still in
    /// progress, since a short packet is what normally marks the end.
    async fn request_out(
        &mut self,
        buf: &[u8],
        ensure_transaction_end: bool,
    ) -> Result<(), PipeError>
    where
        D: pipe::IsOut,
    {
        let ep_type = T::ep_type();
        if !matches!(ep_type, EndpointType::Interrupt | EndpointType::Bulk) {
            return Err(PipeError::BadResponse);
        }
        if ep_type == EndpointType::Bulk && !bulk_buffer_ok(buf) {
            return Err(PipeError::BufferOverflow);
        }
        if ep_type == EndpointType::Interrupt && buf.len() > MAX_TRANSFER_LEN {
            return Err(PipeError::BufferOverflow);
        }

        loop {
            match self
                .channel
                .bulk_out_async(
                    self.endpoint,
                    self.endpoint_number,
                    &mut self.toggle,
                    buf,
                    self.timer,
                )
                .await
            {
                Ok(_) => break,
                Err(e) if retryable(e) => TimeoutTimer::after(BULK_NAK_RETRY).await,
                Err(e) => return Err(map_error(e)),
            }
        }

        let ends_on_boundary = !buf.is_empty()
            && buf
                .len()
                .is_multiple_of(usize::from(self.endpoint.max_packet_size.max(1)));
        if ensure_transaction_end && ends_on_boundary {
            loop {
                match self
                    .channel
                    .bulk_out_async(
                        self.endpoint,
                        self.endpoint_number,
                        &mut self.toggle,
                        &[],
                        self.timer,
                    )
                    .await
                {
                    Ok(_) => break,
                    Err(e) if retryable(e) => TimeoutTimer::after(BULK_NAK_RETRY).await,
                    Err(e) => return Err(map_error(e)),
                }
            }
        }
        Ok(())
    }

    /// Sets how long this control pipe waits out a NAKing device.
    fn set_timeout(&mut self, timeout: TimeoutConfig)
    where
        T: pipe::IsControl,
    {
        self.timeout = timeout;
    }

    /// Returns the host-side data toggle to DATA0.
    ///
    /// The caller invokes this after anything that resets the device's
    /// own toggle — a successful `CLEAR_FEATURE(ENDPOINT_HALT)`,
    /// `SET_CONFIGURATION`, or `SET_INTERFACE` — since a host and device
    /// disagreeing about the toggle means every packet is discarded as a
    /// retransmission.
    fn reset_data_toggle(&mut self)
    where
        T: pipe::IsBulkOrInterrupt,
    {
        self.toggle = false;
    }
}

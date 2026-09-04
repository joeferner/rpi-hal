#![no_std]
#![no_main]

// How much of an inbound burst the receive path actually delivers.
//
// This exists because a real application made it look like a problem
// somewhere else entirely. A web interface served off a board like this
// stalled for almost exactly one second, on one file out of six, only
// sometimes. It looked like an HTTP problem, then like a TCP timeout, then
// like a socket-pool limit; what it turned out to be was frames arriving
// faster than this crate's receive loop drains them, so a TCP peer's
// acknowledgements went missing and it waited out a retransmission
// timeout. Nothing on the board reported an error, because from the
// driver's point of view nothing went wrong.
//
// So this measures the thing directly, with no TCP anywhere near it. The
// host sends a burst of UDP datagrams, each carrying its own sequence
// number and the burst's total; this counts the ones that arrive and says
// how many went missing. Loss is read off a single line rather than
// inferred from latency.
//
// # Running it
//
// Note the address DHCP prints, then, on the host:
//
// ```
// python3 - <<'EOF'
// import socket, time
// ADDR, PORT = "192.168.1.x", 47913         # the address printed above
// s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
// for depth in (1, 8, 16, 32, 64, 128, 256):
//     for seq in range(depth):
//         s.sendto(b"RPIBURST" + seq.to_bytes(4, "little")
//                  + depth.to_bytes(4, "little") + b"\0" * 48, (ADDR, PORT))
//     time.sleep(1.0)                        # let the board report
// EOF
// ```
//
// Each `sendto` loop is one burst, sent as fast as the host will emit it.
// The board prints one line per burst. Paced traffic is the control: add a
// `time.sleep(0.004)` inside the inner loop and the loss should go to zero
// at any depth, which is what says the problem is burst absorption rather
// than throughput.
//
// The `RPIBURST` prefix is not decoration — a bound UDP port also receives
// broadcast traffic, so datagrams without it are ignored.
//
// # What was measured with it
//
// On a Pi 3, before anything was changed: a *sustained* 4,000 datagrams a
// second arrive with no loss at all, but a burst of 8 back-to-back loses
// about 10% of itself, 64 loses 26%, and a burst big enough to fill the
// link loses 71%. The frames die inside the chip, in the gap between one
// bulk-IN transfer completing and the next being submitted — a gap that
// recurs once per frame, because the chip is left configured to hand over
// exactly one frame per transfer. `rx_stats().batched` staying at zero
// while frames go missing is that configuration showing through.
//
// Pi 2/3 only, like every example here that touches the LAN9514.

use core::fmt::Write as _;

use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Config, StackResources};
use embassy_time::{Duration, with_timeout};
use rpi_hal::mailbox::Mailbox;
use rpi_hal::rng::Rng;
use rpi_hal::usb::dwc2::{Channel, Dwc2Host};
use rpi_hal::usb::lan9514::Lan9514;
use rpi_hal::{halt, irq, lic::Lic, pac, timer::Timer, uart::Uart, usb};
use rpi_hal_embassy::lan9514::{Lan9514Driver, Lan9514Runner, Lan9514State};
use rpi_hal_embassy::{Executor, time_driver};

/// Frames the adapter may hold queued inbound.
///
/// Deliberately generous, and that is the point: the question this example
/// asks is whether frames survive long enough to *reach* this queue, so
/// the queue itself must not be the thing that overflows. Raising it from
/// 4 to 64 on a real application changed the measured loss by almost
/// nothing, which is what first said the queue was downstream of the
/// problem.
const RX_QUEUE: usize = 64;

/// The outbound counterpart to [`RX_QUEUE`]. Nothing here transmits much.
const TX_QUEUE: usize = 8;

/// UDP port the burst is sent to.
///
/// Not a memorable round number, deliberately. The first choice here was
/// 9999, which is TP-Link's device-discovery port: a socket bound to it on
/// a home network receives a steady trickle of other people's broadcasts,
/// and this example dutifully reported each one as a burst of four billion
/// datagrams. Hence [`MAGIC`] as well — the port only has to be quiet, the
/// marker is what makes a datagram ours.
const BURST_PORT: u16 = 47913;

/// Prefix every datagram of a burst carries.
///
/// A socket bound to a UDP port receives broadcast traffic sent to it as
/// well as traffic addressed to this board, so "arrived on the right port"
/// is not the same as "is part of the measurement". Anything without this
/// is ignored rather than counted, which keeps a neighbour's discovery
/// protocol from being reported as catastrophic packet loss.
const MAGIC: &[u8; 8] = b"RPIBURST";

/// Sockets the stack may hold at once: DHCP's, and this example's.
const SOCKETS: usize = 4;

/// How long without a datagram counts as the end of a burst.
///
/// Long enough that a straggler recovered by nothing at all still lands
/// inside its own burst, short enough to report before the next one
/// starts.
const QUIET: Duration = Duration::from_millis(400);

/// Datagram bytes the sender's bookkeeping occupies: [`MAGIC`], then the
/// sequence number and the burst's total as little-endian `u32`s.
const HEADER: usize = MAGIC.len() + 8;

/// Quiet periods between "nothing is arriving" lines — see the idle
/// branch of the receive loop. Twelve of [`QUIET`] is about five seconds.
const IDLE_REPORT: u32 = 12;

/// Whether to periodically hold interrupts off while bursts arrive.
///
/// The reason this knob exists. With it `false`, this example absorbs 256
/// frames back-to-back without losing one, and saturates somewhere above
/// 32,000 frames a second — while a real application on the same driver
/// was losing 10% of a burst of *eight*. The difference between them is
/// not the receive path, it is everything else the application's CPU was
/// doing, so the useful question is what a receive loop that cannot be
/// woken costs.
///
/// That is what this reproduces. The receive is interrupt-driven, so a
/// critical section is not merely a task that will not yield: it stops
/// the USB completion interrupt from being taken at all, no transfer is
/// resubmitted for as long as it lasts, and frames pile into the chip's
/// FIFO until it overruns. Nothing reports the overrun.
///
/// [`STARVE_HOLD_US`] is set to roughly what one line of console output
/// costs an application that logs from inside a critical section at
/// 115200 baud, because that is the specific thing under suspicion.
const STARVE: bool = false;

/// How long each starvation holds interrupts off, in microseconds.
const STARVE_HOLD_US: u64 = 3_000;

/// How often to do it.
const STARVE_EVERY_MS: u64 = 20;

/// Largest burst this will believe a datagram's claim of.
///
/// The total is read out of a packet that arrived over the network, so it
/// is not trustworthy just because it carried the right marker. A bound
/// keeps a corrupt or hostile one from producing arithmetic nobody can
/// read — which is exactly what happened when a stray broadcast's bytes
/// were taken as a burst of 4,154,130,315.
const MAX_BURST: u32 = 1_000_000;

/// Receive buffer for the burst socket, in bytes.
///
/// Large for the same reason [`RX_QUEUE`] is: a socket buffer that
/// overflowed would drop datagrams the driver had already delivered, and
/// this example would report the loss as the driver's.
const SOCKET_RX_BYTES: usize = 96 * 1024;

/// Datagrams the socket can hold at once, for the same reason.
const SOCKET_RX_PACKETS: usize = 512;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// Widens a borrow to `'static`. Sound only where the caller never
/// returns, which is the case for every use below.
unsafe fn make_static<T>(t: &mut T) -> &'static mut T {
    unsafe { core::mem::transmute(t) }
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Lan9514Driver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn lan9514_task(runner: Lan9514Runner<'static, 'static>) -> ! {
    runner.run().await
}

/// Holds interrupts off in bursts, standing in for an application that
/// does blocking work with them masked. Spawned only when [`STARVE`] is
/// set; see it for why this is the experiment.
#[embassy_executor::task]
async fn starve_task(timer: &'static Timer) -> ! {
    loop {
        embassy_time::Timer::after(Duration::from_millis(STARVE_EVERY_MS)).await;
        critical_section::with(|_| {
            // Busy-waiting is the point: the wall clock has to advance
            // with interrupts masked, which is exactly what a blocking
            // UART write inside a critical section does. Awaiting
            // anything here would let the executor run and reproduce
            // nothing.
            let start = timer.now_micros();
            while timer.now_micros().wrapping_sub(start) < STARVE_HOLD_US {}
        });
    }
}

/// One burst's worth of counting.
struct Burst {
    /// Datagrams that arrived.
    received: u32,
    /// What the sender said the burst would be, from the first datagram of
    /// it that arrived. Nothing is inferred from the highest sequence
    /// number seen, because the missing ones are frequently the last:
    /// counting to the highest arrival would quietly shrink the burst to
    /// fit whatever turned up.
    total: u32,
    /// The adapter's own counters when the burst started, so the report
    /// can show what the driver thought was happening at the same time.
    frames: u32,
    batched: u32,
    unusable: u32,
}

impl Burst {
    /// Starts counting, taking a baseline of the adapter's counters.
    fn begin(total: u32) -> Self {
        let rx = rpi_hal_embassy::lan9514::rx_stats();
        Burst {
            received: 0,
            total,
            frames: rx.frames,
            batched: rx.batched,
            unusable: rx.unusable,
        }
    }

    /// Prints the verdict.
    fn report(&self, uart: &mut Uart) {
        let rx = rpi_hal_embassy::lan9514::rx_stats();
        let lost = self.total.saturating_sub(self.received);
        // Integer percent: there is no float formatter in this build, and
        // a loss figure needs no decimal place to be damning. Widened to
        // 64 bits first — `lost * 100` overflows a `u32` for a burst of
        // more than 42 million, and in release that wraps silently into a
        // percentage that looks reassuring.
        let percent = (u64::from(lost) * 100)
            .checked_div(u64::from(self.total))
            .unwrap_or(0);
        let _ = writeln!(
            uart,
            "burst of {}: {} received, {} lost ({}%) | adapter: {} frames, \
             {} batched, {} unusable{}",
            self.total,
            self.received,
            lost,
            percent,
            rx.frames - self.frames,
            rx.batched - self.batched,
            rx.unusable - self.unusable,
            // Stated on the line rather than left to be remembered: two
            // runs of this example differ only by a constant, and a
            // console log of numbers that does not say which is which is
            // worth very little later.
            if STARVE { " | STARVED" } else { "" },
        );
    }
}

#[embassy_executor::task]
async fn burst_task(stack: embassy_net::Stack<'static>, mut uart: Uart) {
    let _ = writeln!(uart, "waiting for DHCP...");
    stack.wait_config_up().await;

    if let Some(config) = stack.config_v4() {
        let _ = writeln!(uart, "DHCP: {}", config.address);
    }
    let _ = writeln!(uart, "counting UDP bursts on port {BURST_PORT}");

    let mut rx_meta = [PacketMetadata::EMPTY; SOCKET_RX_PACKETS];
    let mut rx_buffer = [0u8; SOCKET_RX_BYTES];
    // Nothing is sent from here, but the constructor wants both halves.
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut tx_buffer = [0u8; 64];
    let mut datagram = [0u8; 256];

    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );
    if socket.bind(BURST_PORT).is_err() {
        let _ = writeln!(uart, "could not bind port {BURST_PORT}");
        return;
    }

    // Diagnostics for the case where nothing is reported at all, which is
    // otherwise indistinguishable from the host's packets never arriving.
    // A burst report only happens when marked datagrams turn up, so
    // without these a silent console cannot say whether the socket saw
    // traffic and rejected it, saw none, or errored on every receive.
    let mut ignored = 0u32;
    let mut errors = 0u32;
    let mut idle_ticks = 0u32;

    let mut burst: Option<Burst> = None;
    loop {
        // The timeout is how a burst's end is detected: the sender says
        // nothing when it has finished, so silence is the only signal
        // there is.
        match with_timeout(QUIET, socket.recv_from(&mut datagram)).await {
            Ok(Ok((len, _))) => {
                // Anything that is not one of ours is not a datagram this
                // measurement lost, so it is skipped rather than counted
                // — including the broadcast traffic a bound UDP port
                // receives whether or not anyone meant it for this board.
                if len < HEADER || &datagram[..MAGIC.len()] != MAGIC {
                    ignored += 1;
                    continue;
                }
                let total = u32::from_le_bytes([
                    datagram[MAGIC.len() + 4],
                    datagram[MAGIC.len() + 5],
                    datagram[MAGIC.len() + 6],
                    datagram[MAGIC.len() + 7],
                ]);
                if total == 0 || total > MAX_BURST {
                    continue;
                }
                let burst = burst.get_or_insert_with(|| Burst::begin(total));
                burst.received += 1;
            }
            // A receive error is not loss and must not be counted as
            // either arrival or absence; the socket stays bound and the
            // next datagram is waited for.
            Ok(Err(_)) => errors += 1,
            Err(_) => {
                if let Some(finished) = burst.take() {
                    finished.report(&mut uart);
                    idle_ticks = 0;
                    continue;
                }
                // Nothing arrived. Say so periodically rather than never:
                // a console that has printed the banner and then stayed
                // quiet for a minute looks identical whether the host is
                // sending to the wrong address, the datagrams are being
                // filtered out here, or every receive is failing.
                idle_ticks += 1;
                if idle_ticks.is_multiple_of(IDLE_REPORT) {
                    let rx = rpi_hal_embassy::lan9514::rx_stats();
                    let _ = writeln!(
                        uart,
                        "idle: no marked datagrams yet | {} ignored, {} recv \
                         errors | adapter: {} frames, {} unusable",
                        ignored, errors, rx.frames, rx.unusable
                    );
                }
            }
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "inbound burst loss over the on-board Ethernet");

    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // Power first, then read the MAC: the same order rpi-hal's
    // `usb_ethernet_smoltcp` uses. The controller comes up only partially
    // powered from firmware, and nothing on the bus responds until this
    // has happened.
    if !usb::power_on(&mut mailbox) {
        let _ = writeln!(uart, "USB power-on failed");
        halt();
    }

    // The MAC lives in firmware on this board, not in the chip.
    let mac = match mailbox.mac_address() {
        Ok(mac) => mac,
        Err(e) => {
            let _ = writeln!(uart, "MAC read failed: {e:?}");
            halt();
        }
    };
    let _ = writeln!(
        uart,
        "board MAC: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );

    let mut dwc2 = Dwc2Host::init(
        peripherals.USB_OTG_GLOBAL,
        peripherals.USB_OTG_HOST,
        peripherals.USB_OTG_PWRCLK,
        &timer,
    );

    // Widened here rather than inside `run`, because a `Channel` borrows the
    // controller it was allocated from: the driver holds its channel for the
    // life of the program, so the controller has to be `'static` *before*
    // `alloc_channel` is called, not after. Sound because `kmain` never
    // returns.
    let dwc2: &'static Dwc2Host = unsafe { make_static(&mut dwc2) };

    // Bounded, and noisy about it. An unbounded wait here is
    // indistinguishable from a crash on the console, and this is the one
    // step that depends on the board actually having an on-board hub
    // powered up behind it.
    let _ = writeln!(uart, "waiting for the on-board hub...");
    let mut waited_ms = 0;
    while !dwc2.port_connected() {
        timer.delay_ms(100);
        waited_ms += 100;
        if waited_ms % 1000 == 0 {
            let _ = writeln!(uart, "  still no device on the root port ({waited_ms}ms)");
        }
        if waited_ms >= 10_000 {
            let _ = writeln!(
                uart,
                "root port never reported a device — USB power or the DWC2 \
                 bring-up is the suspect, not anything above it"
            );
            halt();
        }
    }
    let _ = writeln!(uart, "hub detected after {waited_ms}ms");

    let mut uart = Some(uart);
    let result = usb::enumerate(dwc2, &timer, |channel, timer, device| {
        match Lan9514::from_device(channel, timer, device) {
            Ok(Some(lan9514)) => {
                // The stack needs two channels of its own, one per
                // direction — see the adapter's documentation for why it
                // is two. `channel` belongs to enumeration and is gone
                // once this callback returns, while the runner below
                // keeps moving frames forever.
                let (Some(rx_channel), Some(tx_channel)) =
                    (dwc2.alloc_channel(), dwc2.alloc_channel())
                else {
                    let _ = writeln!(
                        uart.as_mut().unwrap(),
                        "no free host channels for the stack"
                    );
                    return core::ops::ControlFlow::Break(());
                };
                // Diverges, so enumeration never resumes — and so the
                // borrows widened inside really do last forever.
                run(
                    uart.take().unwrap(),
                    rx_channel,
                    tx_channel,
                    timer,
                    lan9514,
                    mac,
                )
            }
            _ => core::ops::ControlFlow::Continue(()),
        }
    });

    // Only reachable when the LAN9514 never turned up, since `run`
    // diverges. Say so rather than halting silently.
    if let Some(mut uart) = uart {
        let _ = writeln!(uart, "no LAN9514 on the bus (enumerate: {result:?})");
    }
    halt();
}

/// Services the interrupts the executor and the Ethernet adapter depend
/// on. Mandatory, and silently fatal to omit — see `embassy_net_echo`.
#[unsafe(no_mangle)]
pub extern "C" fn __irq_handler() {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);

    if lic.is_timer1_pending() {
        time_driver::on_timer_irq();
    }
    if lic.is_usb_pending() {
        usb::dwc2::on_irq();
    }
}

/// Brings the chip up and hands it to `embassy-net`. Never returns.
fn run(
    mut uart: Uart,
    mut rx_channel: Channel<'static>,
    tx_channel: Channel<'static>,
    timer: &Timer,
    mut lan9514: Lan9514,
    mac: [u8; 6],
) -> ! {
    if let Err(e) = lan9514.start(&mut rx_channel, timer, mac) {
        let _ = writeln!(uart, "LAN9514 start failed: {e:?}");
        halt();
    }

    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);
    time_driver::init(Timer::new(peripherals.SYSTMR), &lic);

    // After `start`, which ran on the blocking methods and polls its own
    // `HCINT`.
    lic.enable_usb_irq();
    irq::enable_irq();

    let timer: &'static Timer = unsafe { &*(timer as *const Timer) };

    let mut state = Lan9514State::<RX_QUEUE, TX_QUEUE>::new();
    let state = unsafe { make_static(&mut state) };
    let (driver, lan9514_runner) =
        rpi_hal_embassy::lan9514::new(state, lan9514, rx_channel, tx_channel, timer, mac);

    let mut rng = Rng::new();
    let seed = (u64::from(rng.next_u32()) << 32) | u64::from(rng.next_u32());

    let mut resources = StackResources::<SOCKETS>::new();
    let resources = unsafe { make_static(&mut resources) };

    let (stack, runner) =
        embassy_net::new(driver, Config::dhcpv4(Default::default()), resources, seed);

    let mut executor = Executor::new();
    let executor = unsafe { make_static(&mut executor) };

    executor.run(|spawner| {
        spawner.spawn(net_task(runner).unwrap());
        spawner.spawn(lan9514_task(lan9514_runner).unwrap());
        spawner.spawn(burst_task(stack, uart).unwrap());
        if STARVE {
            spawner.spawn(starve_task(timer).unwrap());
        }
    });
}

//! `embassy-net` adapter over `rpi-hal`'s Wi-Fi driver: a queue pair for
//! the stack ([`WifiDriver`](crate::wifi::WifiDriver)) and a task that
//! moves frames between those queues and the radio
//! ([`WifiRunner`](crate::wifi::WifiRunner)). Build both with
//! [`new`](crate::wifi::new).
//!
//! # Why it is split in two
//!
//! The same reason as [`crate::lan9514`]: `embassy_net_driver::Driver`'s
//! `receive`/`transmit` are synchronous, so there is nowhere inside them
//! to `.await` and nowhere to do the chip's SDIO work without holding the
//! executor for the length of every transfer.
//! `embassy-net-driver-channel` is the way out — the `Driver` the stack
//! sees becomes a pair of packet queues, and the bus work moves into a
//! task on the far side of them.
//!
//! # What an application must provide
//!
//! A `rpi_hal::wifi::Wifi` that has already joined a network, and a
//! `rpi_hal::timer::Timer`. That is the whole contract. Unlike
//! [`crate::lan9514`] there is no interrupt to dispatch, because this
//! runner does not wait on one — see below.
//!
//! An application that wants the runner to rejoin the network on its own
//! when the association goes away hands it the credentials as well, through
//! [`WifiRunner::reconnecting`](crate::wifi::WifiRunner::reconnecting).
//!
//! # Why the runner polls
//!
//! The chip does have an interrupt, and `rpi-hal`'s SDIO layer uses it to
//! know a frame is ready — but every call into that driver is blocking, so
//! there is nothing here to await. The runner is therefore a loop on a
//! timer: drain whatever has arrived, hand over whatever is queued, sleep,
//! repeat.
//!
//! That is the same cost an application paid when it polled `smoltcp`
//! directly — one pass per millisecond — and it is bounded work per pass
//! rather than a spin. What it costs in latency is at most one interval.
//! What would remove it is an interrupt-driven SDIO path in the HAL, at
//! which point this module loses its ticker and its `embassy-time`
//! dependency with it.
//!
//! # Receive drains, transmit waits its turn
//!
//! Each pass empties the chip's receive FIFO before it sends anything, and
//! that order is not arbitrary. The firmware gates transmits on a credit
//! window it advances *in the headers of received frames*, so a runner that
//! favoured transmit would run itself out of credit and then have nothing
//! to read the replenishment from. Receiving first is what keeps the window
//! open.
//!
//! A transmit refused for want of credit leaves the frame where it is and
//! tries again next pass, rather than dropping it: the queue is the stack's
//! own, and it is still holding what it asked to send.
//!
//! # Losing the network, and getting it back
//!
//! The runner watches the association and, given a
//! [`Reconnect`](crate::wifi::Reconnect), rejoins
//! by itself. Both halves of that live here rather than in an application
//! because the runner owns the chip from the moment the stack is built:
//! every call into `rpi-hal`'s driver takes `&mut Wifi`, and there is only
//! one of those.
//!
//! What it watches is the firmware's own answer to whether it is on a
//! network ([`rpi_hal::wifi::Wifi::bssid`]), not the association *events* —
//! this firmware does not emit those reliably — and not the traffic, which
//! says nothing: a network that has gone away and one with nothing on it
//! both look like silence.
//!
//! Asking costs a control command, and `rpi-hal`'s control path drops data
//! frames that arrive while it waits for the reply, so the question is put
//! only to a radio that has received nothing for `RX_SILENCE` — a link
//! that is carrying anything is answering the question by carrying it. What
//! that buys is a probe that never runs during a transfer and a loss that
//! is still noticed within about fifteen seconds of the frames stopping.
//!
//! The rejoin itself is [`rpi_hal::wifi::Wifi::start_join`] and then
//! polling, rather than `join_wpa2`, and that is the whole reason it can
//! live in the runner's loop: `join_wpa2` blocks for as long as an
//! association takes — up to fifteen seconds — and this runs on the
//! executor, where fifteen seconds of not returning is every other task on
//! the core stopped. Issued and then looked in on, an association costs a
//! command every quarter of a second and the runner goes on moving frames
//! throughout.
//!
//! The stack is told the link is down for as long as it is, which is what
//! makes DHCP run again on the far side: the network a board comes back on
//! to is not required to be the one it left.
//!
//! Available only with the `wifi` feature enabled.

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use critical_section::Mutex;
use embassy_net_driver::{HardwareAddress, LinkState};
use embassy_net_driver_channel as ch;
use embassy_time::{Duration, Instant, Timer as AsyncTimer};
use rpi_hal::timer::Timer;
use rpi_hal::wifi::{Counters, Error, PowerManagement, Wifi};

/// Largest frame moved in either direction, which is the driver's own
/// limit: a 14-byte Ethernet header plus a 1500-byte payload.
pub const MTU: usize = Wifi::MTU;

/// How long the runner sleeps between passes.
///
/// One millisecond, which is what an application cost when it polled
/// `smoltcp` directly and is far below anything the protocols care about.
/// It bounds both the receive latency and how long a frame the stack has
/// queued waits before it goes out.
const POLL_INTERVAL: Duration = Duration::from_millis(1);

/// Frames drained from the chip in one pass before the loop moves on.
///
/// A bound rather than a target. Draining until the FIFO is empty is what
/// the credit window wants, and an unbounded drain is also how a broadcast
/// storm on a busy network would hold this task forever — so the pass stops
/// here and picks up where it left off a millisecond later.
const RX_BURST: usize = 8;

/// The packet queues [`new`] builds a [`WifiDriver`] and [`WifiRunner`] out
/// of, with room for `N_RX` received and `N_TX` outbound frames.
///
/// An application owns this — typically in a `StaticCell` — because both
/// halves borrow from it for as long as the network stack runs. Depth is
/// the application's call: more buffers absorb a longer burst, at one
/// [`MTU`] each.
pub struct WifiState<const N_RX: usize, const N_TX: usize> {
    inner: ch::State<MTU, N_RX, N_TX>,
}

impl<const N_RX: usize, const N_TX: usize> WifiState<N_RX, N_TX> {
    /// Creates the queues, empty. `const`, so it can initialise a `static`
    /// directly.
    pub const fn new() -> Self {
        Self {
            inner: ch::State::new(),
        }
    }
}

impl<const N_RX: usize, const N_TX: usize> Default for WifiState<N_RX, N_TX> {
    /// The same as [`WifiState::new`].
    fn default() -> Self {
        Self::new()
    }
}

/// The `embassy_net_driver::Driver` half — what goes to `embassy_net::new`.
///
/// Synchronous by design: the frames it hands the stack were fetched by
/// [`WifiRunner`] before the stack ever asked, which is the whole point of
/// the split.
pub type WifiDriver<'d> = ch::Device<'d, MTU>;

/// The half that talks to the chip: an endless loop moving frames between
/// [`WifiDriver`]'s queues and the radio.
///
/// Created by [`new`] and driven by [`run`](WifiRunner::run), which an
/// application spawns as its own task.
pub struct WifiRunner<'d> {
    /// The queues, from the stack's side.
    runner: ch::Runner<'d, MTU>,
    /// The joined chip.
    wifi: Wifi,
    /// What the blocking driver times its SDIO transfers against.
    timer: &'d Timer,
    /// What to rejoin with when the association goes away, or `None` for a
    /// runner that reports the loss and leaves it — see
    /// [`WifiRunner::reconnecting`].
    reconnect: Option<Reconnect<'d>>,
}

/// What a runner needs to put a lost association back by itself, for
/// [`WifiRunner::reconnecting`].
///
/// The credentials are here rather than held by the runner from the start
/// because joining is not what this crate is for: an application brings its
/// own chip up and joins its own network, and this is the narrow case of
/// doing that *again*, which only the runner can do because only the runner
/// still owns the chip.
#[derive(Clone, Copy)]
pub struct Reconnect<'a> {
    /// The network to rejoin. The same one the application joined at
    /// bring-up — this does not roam and does not pick.
    pub ssid: &'a str,
    /// Its WPA2-PSK passphrase.
    pub passphrase: &'a str,
    /// Applied again after each association.
    ///
    /// Not a preference so much as a repair: the firmware resets this on
    /// every re-association (see
    /// [`rpi_hal::wifi::Wifi::set_power_management`]), so a board that
    /// turned power save off at bring-up and then rejoined would come back
    /// with the radio sleeping — a link that works and is slow, which is
    /// the hardest kind of fault to go looking for. Name here what the
    /// application asked for at bring-up; a board that asked for nothing
    /// names [`PowerManagement::Fast`], which is what the chip powers on
    /// in.
    pub power_management: PowerManagement,
}

/// Wraps an already-joined [`Wifi`] as an `embassy-net` device, returning
/// the [`WifiDriver`] to hand to `embassy_net::new` and the [`WifiRunner`]
/// to spawn.
///
/// `mac` is the chip's own address, read during bring-up: `embassy-net`
/// needs it to answer ARP, and the driver does not hold on to it.
pub fn new<'d, const N_RX: usize, const N_TX: usize>(
    state: &'d mut WifiState<N_RX, N_TX>,
    wifi: Wifi,
    timer: &'d Timer,
    mac: [u8; 6],
) -> (WifiDriver<'d>, WifiRunner<'d>) {
    let (runner, device) = ch::new(&mut state.inner, HardwareAddress::Ethernet(mac));
    (device, attach(runner, wifi, timer))
}

/// Puts the radio behind a queue pair somebody else built, returning the
/// runner to spawn.
///
/// [`new`] is this with the queue pair built for you. This exists for the
/// same reason [`crate::ethernet::attach`] does, and the two are meant to
/// be used together: a board that falls back from Ethernet to Wi-Fi owns
/// one `ch::State`, creates the queue pair once, and attaches whichever
/// interface it settled on. Both adapters hand the stack the same
/// `ch::Device<'_, 1514>`, so from above there is nothing to choose
/// between — which is what makes one stack over either interface possible
/// at all.
///
/// `runner` comes from `embassy_net_driver_channel::new` with the radio's
/// own MAC, the one [`new`] would have passed.
pub fn attach<'d>(runner: ch::Runner<'d, MTU>, wifi: Wifi, timer: &'d Timer) -> WifiRunner<'d> {
    WifiRunner {
        runner,
        wifi,
        timer,
        reconnect: None,
    }
}

impl<'d> WifiRunner<'d> {
    /// Gives the runner what it needs to get back onto the network on its
    /// own, and turns it into a runner that does.
    ///
    /// Without this a runner reports the association going away —
    /// [`associated`] goes false and the stack is told the link is down —
    /// and waits for it to come back. With it, the runner rejoins: see the
    /// module documentation for what it does and how long it takes to
    /// notice.
    pub fn reconnecting(mut self, reconnect: Reconnect<'d>) -> Self {
        self.reconnect = Some(reconnect);
        self
    }

    /// Moves frames until the end of time.
    pub async fn run(self) -> ! {
        let Self {
            mut runner,
            mut wifi,
            timer,
            reconnect,
        } = self;

        // Taken as up rather than probed for, because a `Wifi` is joined
        // before it gets here: the application associated it, and asking
        // the chip to confirm what the caller has just been told would only
        // put the first frames of the stack's life behind a control
        // command.
        ASSOCIATED.store(true, Ordering::Relaxed);
        runner.set_link_state(LinkState::Up);

        let mut watch = Watch::new(reconnect);
        // When the next link sample is due, and how long the radio has
        // been quiet — see `sample_link` for why the second one matters.
        let mut due = Instant::now();
        let mut quiet = 0u32;
        // When a frame was last received, which is what `Watch` decides
        // whether to disturb the radio on.
        let mut heard = Instant::now();

        loop {
            PASSES.fetch_add(1, Ordering::Relaxed);
            // Two statements rather than one `||`, and not by accident:
            // both halves of a pass have to run, and short-circuiting
            // would skip the transmit on any pass that received something.
            let received = receive(&mut runner, &mut wifi, timer);
            let sent = transmit(&mut runner, &mut wifi, timer);
            if received {
                heard = Instant::now();
            }
            quiet = if received || sent {
                0
            } else {
                quiet.saturating_add(1)
            };

            sample_link(&mut wifi, timer, &mut due, quiet);
            watch.pass(&mut runner, &mut wifi, timer, heard);
            sample_counters(&mut wifi, timer);
            AsyncTimer::after(POLL_INTERVAL).await;
        }
    }
}

/// How often [`link`] is refreshed, once the radio is quiet enough for
/// it.
const LINK_INTERVAL: Duration = Duration::from_secs(5);

/// Consecutive passes that must have moved nothing before a link sample
/// is taken.
///
/// About a tenth of a second of silence at [`POLL_INTERVAL`]. See
/// [`sample_link`].
const LINK_QUIET_PASSES: u32 = 100;

/// Refreshes [`link`], but only on an idle radio.
///
/// The quiet requirement is the whole design of this. Reading the signal
/// strength is a control command, and `rpi-hal`'s control path discards
/// data frames that arrive while it waits for the reply — so a sampler
/// that ran on a plain timer would drop a frame or two every time it
/// fired, which during a bulk transfer is packet loss injected by the
/// instrument. Waiting for a gap costs nothing: a link with nothing on it
/// is exactly when someone is watching this number, and a link that is
/// busy has better things to report.
///
/// A failed read empties the slot rather than leaving a stale number
/// standing, because a signal reading that has quietly stopped updating
/// is worse than none.
fn sample_link(wifi: &mut Wifi, timer: &Timer, due: &mut Instant, quiet: u32) {
    if quiet < LINK_QUIET_PASSES || Instant::now() < *due {
        return;
    }
    *due = Instant::now() + LINK_INTERVAL;

    let sample = match (wifi.rssi_dbm(timer), wifi.link_rate_kbps(timer)) {
        (Ok(rssi_dbm), Ok(rate_kbps)) => Some(Link {
            rssi_dbm,
            rate_kbps,
        }),
        _ => None,
    };
    critical_section::with(|cs| LINK.borrow(cs).set(sample));
}

/// The most recent link sample. See [`link`].
static LINK: Mutex<Cell<Option<Link>>> = Mutex::new(Cell::new(None));

/// What the radio last reported about the link it is on.
///
/// `None` until the first sample, and again if a read fails.
#[derive(Clone, Copy, Debug)]
pub struct Link {
    /// Received signal strength in dBm — negative, and closer to zero is
    /// stronger. Roughly: -50 is the same room, -70 is workable and
    /// starts costing retransmissions, -80 stays associated and little
    /// else.
    pub rssi_dbm: i32,
    /// The rate the link settled on, in kbit/s.
    ///
    /// The *negotiated* rate, which is a ceiling and not an achievement:
    /// it can read the chip's maximum while almost every frame is being
    /// sent two or three times. Read it alongside
    /// [`rpi_hal::wifi::Counters::txretrans`], which is what says how
    /// much of that rate is reaching anybody.
    pub rate_kbps: u32,
}

/// What the radio last reported about the link — signal strength and
/// rate, refreshed every few seconds whenever the radio is idle enough to
/// be asked without disturbing traffic.
///
/// `None` before the first sample, which needs a gap in traffic to
/// happen, and again if the chip stops answering.
pub fn link() -> Option<Link> {
    critical_section::with(|cs| LINK.borrow(cs).get())
}

/// How long the radio must have received nothing before the association is
/// probed.
///
/// A probe is a control command, and the module documentation says what one
/// costs on a busy radio — so this is what keeps them off a link that is
/// working. Receives are what it watches and not traffic in general,
/// because the case being caught is precisely a stack that is still sending
/// into a network that has gone: the transmit half goes on finding frames
/// queued for as long as anything retries, so a rule that waited for the
/// radio to fall silent altogether would wait forever in exactly the state
/// it exists to notice.
///
/// Ten seconds of it. A link carrying anything at all hears something back
/// inside that — a reply, an ARP, whatever a LAN broadcasts — so a board
/// that has heard nothing for this long is either off the network or on one
/// with nothing to say, and the second costs a control command every
/// [`PROBE_INTERVAL`] to rule out.
const RX_SILENCE: Duration = Duration::from_secs(10);

/// How often the association is probed, once the radio has been quiet for
/// [`RX_SILENCE`].
const PROBE_INTERVAL: Duration = Duration::from_secs(5);

/// How often the chip is asked whether a join it was given has landed.
///
/// A join takes seconds — the firmware scans, authenticates and runs the
/// four-way handshake — and the radio has nothing else to do while it does,
/// so this is paced for the answer arriving promptly rather than against
/// disturbing traffic there is none of.
const JOIN_POLL: Duration = Duration::from_millis(250);

/// How long a join is waited on before it is written off and started again.
///
/// The same budget `rpi_hal::wifi::Wifi::join_wpa2` gives one, and for the
/// same reason: what takes longer than this is not a join that is still
/// coming.
const JOIN_BUDGET: Duration = Duration::from_secs(15);

/// How long the runner waits after a join has failed before issuing
/// another.
///
/// Long enough that a board in a house whose access point is off for the
/// evening is not scanning continuously, short enough that nobody standing
/// in front of it waits for the network to come back after it does.
const RETRY_INTERVAL: Duration = Duration::from_secs(10);

/// Where the runner has got to with the association.
#[derive(Clone, Copy)]
enum Association {
    /// On a network, as far as the last probe could tell.
    Up,
    /// Off it, with a join issued and waited on until `deadline`.
    Joining {
        /// When the join stops being waited on.
        deadline: Instant,
    },
    /// Off it, with nothing outstanding.
    Down,
}

/// Watches the association, and puts it back if it was given the means to.
///
/// The state machine behind [`WifiRunner::reconnecting`]: one step of it
/// runs on each pass of the runner's loop, and every step is either a
/// comparison against the clock or one control command, so a runner that is
/// rejoining still moves frames and a runner on a healthy link pays a
/// comparison.
struct Watch<'a> {
    /// What to rejoin with, or `None` for a runner that only reports.
    reconnect: Option<Reconnect<'a>>,
    /// Where the association has got to.
    state: Association,
    /// When the next probe or join attempt is due.
    due: Instant,
}

impl<'a> Watch<'a> {
    /// Starts out on the association the runner was handed.
    fn new(reconnect: Option<Reconnect<'a>>) -> Self {
        Watch {
            reconnect,
            state: Association::Up,
            due: Instant::now() + PROBE_INTERVAL,
        }
    }

    /// Takes one step, given when a frame was last received.
    fn pass(
        &mut self,
        runner: &mut ch::Runner<'_, MTU>,
        wifi: &mut Wifi,
        timer: &Timer,
        heard: Instant,
    ) {
        if Instant::now() < self.due {
            return;
        }
        match self.state {
            // The only state where the radio may be carrying something, so
            // the only one that waits for a gap before it asks.
            Association::Up => {
                if Instant::now() < heard + RX_SILENCE {
                    return;
                }
                self.due = Instant::now() + PROBE_INTERVAL;
                if probe(wifi, timer) {
                    return;
                }
                DROPS.fetch_add(1, Ordering::Relaxed);
                ASSOCIATED.store(false, Ordering::Relaxed);
                // The last sample described a link that no longer exists,
                // and a signal strength left standing under a symbol that
                // says "no network" is the kind of disagreement somebody
                // reasons from.
                critical_section::with(|cs| LINK.borrow(cs).set(None));
                // Told the stack, so that it stops trying, gives up the
                // lease it has, and asks for a new one when the link comes
                // back — the network on the far side of a rejoin is not
                // required to be the one that went away.
                runner.set_link_state(LinkState::Down);
                self.state = Association::Down;
                self.join(wifi, timer);
            }
            Association::Joining { deadline } => {
                if probe(wifi, timer) {
                    self.associate(runner, wifi, timer);
                    return;
                }
                self.due = Instant::now() + JOIN_POLL;
                if Instant::now() >= deadline {
                    self.state = Association::Down;
                    self.due = Instant::now() + RETRY_INTERVAL;
                }
            }
            // Asked before it is told: the firmware runs its own roaming
            // and can put the association back without being asked to, and
            // a join issued over a link that is already up takes it down
            // again for the length of another association.
            Association::Down => {
                if probe(wifi, timer) {
                    self.associate(runner, wifi, timer);
                    return;
                }
                self.join(wifi, timer);
            }
        }
    }

    /// Issues a join, if there is anything to join with.
    fn join(&mut self, wifi: &mut Wifi, timer: &Timer) {
        let Some(reconnect) = self.reconnect else {
            // Nothing to rejoin with, so this is only a watch: keep asking,
            // so that a link something else puts back is still noticed.
            self.due = Instant::now() + PROBE_INTERVAL;
            return;
        };
        JOINS.fetch_add(1, Ordering::Relaxed);
        match wifi.start_join(reconnect.ssid, reconnect.passphrase, timer) {
            // Issued, not landed: `start_join` returns as soon as the
            // firmware has the request, which is what keeps this off the
            // executor for the seconds an association takes.
            Ok(()) => {
                self.state = Association::Joining {
                    deadline: Instant::now() + JOIN_BUDGET,
                };
                self.due = Instant::now() + JOIN_POLL;
            }
            // The chip would not even take the request. Waiting out the
            // retry rather than hammering it: whatever is wrong with the
            // bus or the firmware is not fixed by asking again immediately.
            Err(_) => {
                self.state = Association::Down;
                self.due = Instant::now() + RETRY_INTERVAL;
            }
        }
    }

    /// Records an association, and puts back what making one resets.
    fn associate(&mut self, runner: &mut ch::Runner<'_, MTU>, wifi: &mut Wifi, timer: &Timer) {
        if let Some(reconnect) = self.reconnect {
            // Refusals ignored: what it costs is latency on a link that
            // works, which is not worth giving up a network over -- and the
            // application cannot be told from here anyway.
            let _ = wifi.set_power_management(reconnect.power_management, timer);
        }
        ASSOCIATED.store(true, Ordering::Relaxed);
        runner.set_link_state(LinkState::Up);
        self.state = Association::Up;
        self.due = Instant::now() + PROBE_INTERVAL;
    }
}

/// Whether the chip says it is on a network.
///
/// A command that did not get through counts as "no". It is not the same
/// fact — one is a radio off the network and the other a chip that has
/// stopped answering — but the two want the same thing done about them
/// here: the stack told the link is down, and a join issued, which is also
/// the cheapest thing that would find a chip answering again.
fn probe(wifi: &mut Wifi, timer: &Timer) -> bool {
    matches!(wifi.bssid(timer), Ok(Some(_)))
}

/// Whether the runner last saw the chip on a network. See [`associated`].
static ASSOCIATED: AtomicBool = AtomicBool::new(false);

/// Whether the chip is on a network, as of the runner's last look.
///
/// True from the moment a runner starts — it is handed a joined chip — and
/// false once a probe finds the association gone, until one finds it back.
/// A board that has not started its runner yet reads false, which is the
/// honest answer for one that is still downloading firmware.
///
/// How stale it can be is set by `RX_SILENCE` and `PROBE_INTERVAL`: a
/// link that dies while nothing is being received is noticed within about
/// fifteen seconds, and one that dies mid-transfer as soon as the receives
/// stop.
pub fn associated() -> bool {
    ASSOCIATED.load(Ordering::Relaxed)
}

/// Times the association has been found gone. See [`LinkStats::drops`].
static DROPS: AtomicU32 = AtomicU32::new(0);

/// Joins issued to get it back. See [`LinkStats::joins`].
static JOINS: AtomicU32 = AtomicU32::new(0);

/// What the runner has seen of the association since boot.
///
/// Returned by [`link_stats`].
#[derive(Clone, Copy, Debug)]
pub struct LinkStats {
    /// Times the association has gone away.
    pub drops: u32,
    /// Joins issued to get it back, which is zero on a runner that was
    /// never given anything to rejoin with.
    pub joins: u32,
}

/// What the runner has seen of the association since boot.
///
/// Worth reporting for the same reason [`rx_stats`] is: a board that has
/// been on the network all week and one that has rejoined four hundred
/// times look identical to anything that only asks whether it is on the
/// network now. [`LinkStats::joins`] over [`LinkStats::drops`] is how many
/// attempts each recovery took, which is what separates an access point
/// that reboots nightly from a radio that cannot hold a link.
pub fn link_stats() -> LinkStats {
    LinkStats {
        drops: DROPS.load(Ordering::Relaxed),
        joins: JOINS.load(Ordering::Relaxed),
    }
}

/// Times [`WifiRunner::run`] has been round its loop since boot.
static PASSES: AtomicU32 = AtomicU32::new(0);

/// Takes a counters reading if one has been asked for.
///
/// Only on request, because a reading is a control command: it goes out
/// on the bus and then waits for its reply, and `rpi-hal`'s control path
/// discards any data frames that arrive before it. Sampling on a timer
/// would therefore drop received frames in proportion to how often it
/// sampled, which on a link being measured for packet loss is a
/// spectacularly bad trade.
///
/// A failure is stored rather than discarded: a caller that asked for a
/// reading and gets nothing back cannot tell a chip that refused from a
/// runner that never ran, and those want looking at in different places.
fn sample_counters(wifi: &mut Wifi, timer: &Timer) {
    if !COUNTERS_WANTED.swap(false, Ordering::Relaxed) {
        return;
    }
    let result = wifi.counters(timer);
    critical_section::with(|cs| COUNTERS.borrow(cs).set(Some(result)));
}

/// Set when something has asked for a fresh counters reading.
static COUNTERS_WANTED: AtomicBool = AtomicBool::new(false);

/// The most recent counters reading, empty until one has been attempted.
static COUNTERS: Mutex<Cell<Option<Result<Counters, Error>>>> = Mutex::new(Cell::new(None));

/// Asks the runner to read the firmware's MAC-layer counters on its next
/// pass, and clears whatever was there.
///
/// Asynchronous because the runner owns the chip and every call into it
/// blocks — there is nowhere else the read can happen. Clearing first is
/// what makes the result unambiguous: [`counters`] returning `Some` means
/// a reading taken since this call, so a caller polling for one cannot
/// mistake the previous answer for a fresh one.
///
/// # When not to call it
///
/// During a transfer whose throughput or loss is being measured. The
/// reading costs a control round trip, and `rpi-hal`'s control path drops
/// data frames that arrive while it waits — which is the very thing these
/// counters are usually being read to investigate. Bracket the transfer
/// instead: one reading before, one after, and subtract.
pub fn request_counters() {
    critical_section::with(|cs| COUNTERS.borrow(cs).set(None));
    COUNTERS_WANTED.store(true, Ordering::Relaxed);
}

/// What came of the reading [`request_counters`] asked for.
///
/// `None` until the runner has attempted it, then whatever the chip said
/// — the two are worth distinguishing, since a runner that is not running
/// and a firmware that will not answer produce the same absence of a
/// number and are entirely different problems.
///
/// See [`rpi_hal::wifi::Wifi::counters`] for what the fields mean. They
/// are cumulative since the firmware started, so what a caller wants is
/// the difference between two of these.
pub fn counters() -> Option<Result<Counters, Error>> {
    critical_section::with(|cs| COUNTERS.borrow(cs).get())
}

/// Times the runner has been round its loop since boot.
///
/// The denominator for everything else here. The runner's poll interval
/// sets what this *should* be — a little under a thousand a second — and
/// the gap between that and what it is is the cost of the work a pass does,
/// which nothing else reports. Divided into [`RxStats::frames`] it also
/// gives frames per pass, which is what separates a receive that is
/// keeping up from one that is starved: a pass that finds the chip's FIFO
/// empty and a pass that stops at its burst limit both look like a number
/// of frames, and only this says which.
pub fn passes() -> u32 {
    PASSES.load(Ordering::Relaxed)
}

/// Drains up to [`RX_BURST`] frames from the chip into the stack's queue,
/// reporting whether it found any.
///
/// Stops early on a full queue, which is the stack falling behind: the
/// frame stays in the chip's FIFO and is read on a later pass, which is the
/// right place for it to wait. It is also, with a deep enough queue, a case
/// that does not arise.
fn receive(runner: &mut ch::Runner<'_, MTU>, wifi: &mut Wifi, timer: &Timer) -> bool {
    let mut received = false;
    for _ in 0..RX_BURST {
        let Some(buffer) = runner.try_rx_buf() else {
            return received;
        };
        match wifi.recv_ethernet(buffer, timer) {
            // Nothing waiting, or a control frame the driver consumed and
            // dropped -- either way there is no frame for the stack, and
            // the receive that produced it has already done its real job of
            // advancing the transmit credit window.
            Ok(None) => return received,
            Ok(Some(len)) => {
                RX_FRAMES.fetch_add(1, Ordering::Relaxed);
                runner.rx_done(len);
                received = true;
            }
            // The driver has already put the stream back in step by the
            // time this arrives -- see `rpi_hal::wifi::Error::BadFrame` --
            // so this is a lost frame and a counter, not a reason to stop.
            // TCP treats a dropped frame as a dropped frame.
            Err(error) => {
                RX_ERRORS.fetch_add(1, Ordering::Relaxed);
                store(&RX_LAST_ERROR, error);
                // A frame arrived, even though nothing came of it: the
                // radio is busy, which is what the caller is asking.
                return true;
            }
        }
    }
    received
}

/// Frames [`receive`] has delivered to the stack since boot.
static RX_FRAMES: AtomicU32 = AtomicU32::new(0);

/// Receive frames lost to a malformed SDPCM header since boot.
static RX_ERRORS: AtomicU32 = AtomicU32::new(0);

/// The most recent receive error, or `None` if none has occurred.
static RX_LAST_ERROR: Mutex<Cell<Option<Error>>> = Mutex::new(Cell::new(None));

/// What the receive half has managed since boot.
///
/// Returned by [`rx_stats`].
#[derive(Clone, Copy, Debug)]
pub struct RxStats {
    /// Frames handed to the stack.
    pub frames: u32,
    /// Frames lost to a malformed SDPCM header.
    ///
    /// A handful over a long transfer is the stream recovering as designed
    /// — `rpi-hal` resynchronizes before it returns the error — while a
    /// number that climbs with every transfer is a receive path that needs
    /// looking at rather than a board that is working.
    pub errors: u32,
    /// Why the most recent receive failed, or `None` if none has.
    pub last_error: Option<Error>,
}

/// What the receive half has managed since boot.
///
/// Counters rather than log lines, and the reason is what the first
/// occurrence looked like: printing each one filled the console with
/// hundreds of identical lines a millisecond apart, every one of which cost
/// more time than the receive it was reporting on. An application that
/// wants them logged reads this on its own schedule and decides how loudly
/// to say so.
pub fn rx_stats() -> RxStats {
    RxStats {
        frames: RX_FRAMES.load(Ordering::Relaxed),
        errors: RX_ERRORS.load(Ordering::Relaxed),
        last_error: load(&RX_LAST_ERROR),
    }
}

/// Hands the chip one queued frame, if there is one and the firmware will
/// take it. Reports whether there was anything to send, which is not the
/// same as whether it went: a frame waiting on credit still means the
/// radio is busy.
fn transmit(runner: &mut ch::Runner<'_, MTU>, wifi: &mut Wifi, timer: &Timer) -> bool {
    let Some(frame) = runner.try_tx_buf() else {
        return false;
    };
    match wifi.send_ethernet(frame, timer) {
        Ok(()) => {
            TX_FRAMES.fetch_add(1, Ordering::Relaxed);
            runner.tx_done();
        }
        // Out of credit, or the firmware has paused the data channel. The
        // frame stays queued and goes out on a later pass, once a received
        // frame has advanced the window -- see the module documentation.
        Err(Error::TxBusy) => {
            TX_BUSY_PASSES.fetch_add(1, Ordering::Relaxed);
        }
        Err(error) => {
            // Dropped, and counted. Anything else from here is the frame
            // being unsendable rather than the moment being wrong, so
            // retrying it forever would wedge the queue behind it.
            TX_FAILURES.fetch_add(1, Ordering::Relaxed);
            store(&TX_LAST_ERROR, error);
            runner.tx_done();
        }
    }
    true
}

/// Frames [`transmit`] has handed to the chip since boot.
static TX_FRAMES: AtomicU32 = AtomicU32::new(0);

/// Passes that found a frame queued and no credit to send it with.
static TX_BUSY_PASSES: AtomicU32 = AtomicU32::new(0);

/// Frames dropped because the chip refused them for some other reason.
static TX_FAILURES: AtomicU32 = AtomicU32::new(0);

/// The most recent transmit error, or `None` if none has occurred.
static TX_LAST_ERROR: Mutex<Cell<Option<Error>>> = Mutex::new(Cell::new(None));

/// What the transmit half has managed since boot.
///
/// Returned by [`tx_stats`].
#[derive(Clone, Copy, Debug)]
pub struct TxStats {
    /// Frames the chip accepted.
    pub frames: u32,
    /// Passes that found a frame queued and the firmware unwilling to take
    /// it.
    ///
    /// **Passes, not frames.** One frame waiting out a long credit stall is
    /// counted once per poll interval until it goes out, so this is a
    /// measure of how long transmits spend waiting rather than of how many
    /// waited. Nothing is lost when it climbs — the frame is still queued —
    /// but a number that grows without [`Self::frames`] growing is a credit
    /// window that is never being replenished, which means receives have
    /// stopped.
    pub busy_passes: u32,
    /// Frames dropped because sending them failed for a reason retrying
    /// would not fix.
    ///
    /// **Not the same as frames lost on the network.** A drop here is a
    /// frame that never reached the air at all, and nothing below TCP will
    /// notice: the peer waits out a retransmission timeout, so the symptom
    /// presents as latency with no corresponding error anywhere.
    pub failures: u32,
    /// Why the most recent failure failed, or `None` if none has.
    pub last_error: Option<Error>,
}

/// What the transmit half has managed since boot.
pub fn tx_stats() -> TxStats {
    TxStats {
        frames: TX_FRAMES.load(Ordering::Relaxed),
        busy_passes: TX_BUSY_PASSES.load(Ordering::Relaxed),
        failures: TX_FAILURES.load(Ordering::Relaxed),
        last_error: load(&TX_LAST_ERROR),
    }
}

/// Records the most recent error of one direction.
///
/// A `critical-section` cell rather than an atomic, which is what
/// [`crate::lan9514`] uses for the same job: [`Error`] carries fields — a
/// length and its failed complement, a firmware status code — and those
/// fields are most of the value. Packing the variant into a byte would
/// throw away exactly the part worth reading.
fn store(slot: &Mutex<Cell<Option<Error>>>, error: Error) {
    critical_section::with(|cs| slot.borrow(cs).set(Some(error)));
}

/// Reads back what [`store`] last recorded.
fn load(slot: &Mutex<Cell<Option<Error>>>) -> Option<Error> {
    critical_section::with(|cs| slot.borrow(cs).get())
}

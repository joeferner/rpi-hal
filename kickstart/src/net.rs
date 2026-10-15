//! Getting a board onto the network, whichever way it can.
//!
//! A board states what it is willing to use and in what order:
//!
//! ```toml
//! connection = ["ethernet", "wifi"]
//! ```
//!
//! and [`discover`](crate::net::discover) takes the first that is actually
//! there. Nothing above
//! this line learns which it got — both interfaces hand `embassy-net` the
//! same queue pair, so the stack, the sockets and everything built on them
//! are identical either way.
//!
//! # Presence is two questions
//!
//! "Ethernet is not available" turns out to mean two different things, and
//! a board in a cupboard hits both:
//!
//! - **No chip.** A Pi Zero or a 3A+ has no Ethernet at all. Worse, a 3B+
//!   *has* one that attaches seconds after power-on, later than any
//!   settling delay — so "absent" and "not here yet" look identical until
//!   a timeout says otherwise. That is what
//!   [`Config::chip_timeout_ms`](crate::net::Config::chip_timeout_ms) is.
//! - **No cable.** The chip is there and nothing is plugged into it.
//!   Auto-negotiation takes a second or three from a standing start even
//!   when a cable *is* in, so this is a timeout too — see
//!   [`Config::link_timeout_ms`](crate::net::Config::link_timeout_ms).
//!
//! Both are deadlines rather than answers, and both cost real time on a
//! board that ends up falling through to Wi-Fi. They are separate settings
//! because they fail differently: a short chip timeout misses a 3B+
//! entirely, while a short link timeout misses a slow switch port.
//!
//! Wi-Fi's version of the same question is answered before this module
//! sees it — see [`Hardware::wifi`](crate::net::Hardware::wifi).
//!
//! # Policy here, hardware from the board
//!
//! [`Config`](crate::net::Config) is what the board wants;
//! [`Hardware`](crate::net::Hardware) is what it has. They
//! are separate because naming an interface the board cannot supply is a
//! mistake worth a line in the log, and folding them together would make
//! it unsayable.
//!
//! # What this module does not do
//!
//! **It picks once, at startup.** If the cable is pulled an hour later,
//! nothing here moves to Wi-Fi. That is a larger piece of work —
//! `embassy_net::Stack` is built around one driver and there is no routing
//! between interfaces at any level — and the groundwork for it is the
//! reason `rpi-hal-embassy` lets a board own the queue pair rather than
//! having each adapter build its own. Swapping which runner is attached to
//! a live pair is the shape that would work; it is not built.
//!
//! # What it costs to compile both
//!
//! Less than it looks. The board owns one `embassy-net-driver-channel`
//! queue pair — about 12 KB at four buffers each way — and attaches
//! whichever interface won, rather than one pair per interface, because
//! only one is ever attached. What does double is the runner task pools,
//! which are allocated whether or not they are spawned, and the per-chip
//! bring-up code. Worth knowing, because it is the sort of cost that is
//! invisible until an image stops fitting.

// A walk with nothing to find is a build that cannot produce a network,
// and it would fail at run time with a log line rather than here. The
// alternative to this is a module whose every arm reports that the
// interface was not compiled in.
#[cfg(not(any(feature = "ethernet", feature = "wifi")))]
compile_error!(
    "the `net` feature on its own compiles a priority walk with no interface to find: \
     enable `ethernet`, `wifi`, or both alongside it"
);

#[cfg(feature = "ethernet")]
use core::ops::ControlFlow;

use rpi_hal::timer::Timer;
#[cfg(feature = "ethernet")]
use rpi_hal::usb::dwc2::{Channel, Dwc2Host};
#[cfg(feature = "ethernet")]
use rpi_hal::usb::ethernet::Ethernet;
#[cfg(feature = "ethernet")]
use rpi_hal::usb::lan7800::Lan7800;
#[cfg(feature = "ethernet")]
use rpi_hal::usb::lan9514::Lan9514;
#[cfg(feature = "ethernet")]
use rpi_hal::usb::{Bus, Event};
#[cfg(feature = "wifi")]
use rpi_hal::wifi::Wifi;

use crate::logln;

/// One kind of network interface a board is willing to use.
///
/// Every variant exists in every build, whichever interface features are
/// on. That is deliberate: a configuration file is written once and read
/// by images built several different ways, and a name that stops parsing
/// because of how the image was compiled is a much worse diagnosis than
/// one that parses and is reported as absent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Connection {
    /// USB Ethernet — a LAN9514 on a Pi 2B/3B, a LAN7800 on a 3B+.
    Ethernet,
    /// The on-board wireless radio.
    Wifi,
}

impl Connection {
    /// Parses one entry of a `connection` list, as it is spelled in
    /// configuration. `None` for anything else, which a caller should
    /// report rather than skip: a misspelling that is quietly dropped
    /// looks exactly like an interface that was tried and failed.
    pub fn parse(name: &str) -> Option<Self> {
        match name {
            "ethernet" => Some(Connection::Ethernet),
            "wifi" => Some(Connection::Wifi),
            _ => None,
        }
    }

    /// The spelling [`Self::parse`] accepts.
    pub fn as_str(self) -> &'static str {
        match self {
            Connection::Ethernet => "ethernet",
            Connection::Wifi => "wifi",
        }
    }
}

/// How many interfaces a board may list. Two is every combination a
/// Raspberry Pi offers; the extra room is so a third does not need this
/// changed.
pub const MAX_CONNECTIONS: usize = 4;

/// What to try, in what order, and how long to give each.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// The interfaces to try, best first. An empty list brings up nothing,
    /// which is a legitimate choice for a board with no network at all.
    pub order: [Option<Connection>; MAX_CONNECTIONS],

    /// How long to wait for an Ethernet chip to appear on the USB bus.
    ///
    /// Covers both halves of that: the root port reporting a device at
    /// all, and then an Ethernet function turning up behind it. Neither is
    /// instant — the root port is a hub coming up, and a Pi 3B+'s LAN7800
    /// then attaches seconds after power-on. Too short and that board is
    /// treated as having no Ethernet; too long and every board without one
    /// pays the whole of it before falling through.
    pub chip_timeout_ms: u32,

    /// How long to wait for the link to come up once a chip is found.
    ///
    /// This is the "is a cable plugged in" question, and it is a deadline
    /// because auto-negotiation takes a second or three even when the
    /// answer is yes. Too short and a slow switch port loses to Wi-Fi.
    ///
    /// Ethernet only. Wi-Fi's equivalent is spent inside the board's own
    /// bring-up, which does not return until it has joined — see
    /// [`Hardware::wifi`].
    pub link_timeout_ms: u32,
}

impl Default for Config {
    /// Ethernet then Wi-Fi, with deadlines sized for the slowest board
    /// that works: ten seconds for the root port to come up and a 3B+'s
    /// Ethernet to attach behind it, then five for auto-negotiation.
    ///
    /// Generous on purpose. These are the numbers a board that *does* have
    /// Ethernet has to fit inside, and getting them wrong that way is a
    /// board with no network at all; getting them wrong the other way
    /// costs a slow boot on a board that was going to use Wi-Fi anyway.
    ///
    /// Naming Wi-Fi costs a board that has no radio nothing: the walk only
    /// reaches it if Ethernet failed, and it is reported and passed over
    /// unless the board supplied a [`Hardware::wifi`] bring-up.
    fn default() -> Self {
        Self {
            order: [
                Some(Connection::Ethernet),
                Some(Connection::Wifi),
                None,
                None,
            ],
            chip_timeout_ms: 10_000,
            link_timeout_ms: 5_000,
        }
    }
}

/// The hardware a board is offering, one entry per [`Connection`].
///
/// Separate from [`Config`] because the two answer different questions:
/// the order is what the board *wants*, and this is what it *has*. An
/// interface named in the order with nothing here to carry it out is
/// reported rather than silently skipped — that combination is almost
/// always a board that meant to supply something and did not.
///
/// Built with [`Hardware::new`] and one call per interface, so that a
/// board naming only what it has compiles the same whichever interface
/// features are on.
pub struct Hardware<'a> {
    /// The USB host, walked to find an Ethernet chip.
    #[cfg(feature = "ethernet")]
    usb: Option<&'a Dwc2Host>,

    /// How the board brings its radio up, called at most once.
    #[cfg(feature = "wifi")]
    wifi: Option<&'a mut dyn FnMut() -> Option<Wifi>>,
}

impl<'a> Hardware<'a> {
    /// A board offering nothing, which is where every one starts.
    pub fn new() -> Self {
        Self {
            #[cfg(feature = "ethernet")]
            usb: None,
            #[cfg(feature = "wifi")]
            wifi: None,
        }
    }

    /// Offers the USB host controller, which is what
    /// [`Connection::Ethernet`] is found on.
    ///
    /// The caller must have powered it (`rpi_hal::usb::power_on`) and
    /// initialised `dwc2` already.
    #[cfg(feature = "ethernet")]
    pub fn usb(mut self, dwc2: &'a Dwc2Host) -> Self {
        self.usb = Some(dwc2);
        self
    }

    /// Offers a way to bring the radio up, for [`Connection::Wifi`].
    ///
    /// **It must return a `Wifi` that has already joined a network**, or
    /// `None`. That is the whole contract, and it is what makes Wi-Fi's
    /// presence one question here rather than the two Ethernet's is: by
    /// the time this returns `Some`, the firmware is running, the
    /// regulatory blob is loaded and the association has happened.
    ///
    /// # Why this is a closure and not something this module does
    ///
    /// Bringing up a Pi's radio means reading a firmware image, an nvram
    /// file and a regulatory blob from somewhere, handing the EMMC
    /// controller from the SD driver to the SDIO one — which gives up the
    /// card slot for good — and joining with credentials from somewhere
    /// else again. Every one of those is a board's own decision, and this
    /// crate has no storage module yet to have an opinion with.
    ///
    /// What it does own is *when*. The closure is called at most once, and
    /// only if the walk reaches Wi-Fi — so a board that lists Ethernet
    /// first and finds it never pays the several hundred kilobytes of card
    /// reading that a radio costs.
    #[cfg(feature = "wifi")]
    pub fn wifi(mut self, bring_up: &'a mut dyn FnMut() -> Option<Wifi>) -> Self {
        self.wifi = Some(bring_up);
        self
    }
}

impl Default for Hardware<'_> {
    /// The same as [`Hardware::new`].
    fn default() -> Self {
        Self::new()
    }
}

/// An interface that was found, with the address it answers on.
///
/// Each variant carries its own MAC because they genuinely differ: the
/// Ethernet chips are programmed with the board address the firmware
/// mailbox reports, while the radio has one of its own that the firmware
/// supplies. A stack handed the wrong one associates and then answers no
/// ARP.
///
/// The Ethernet variants are deliberately **not** started for the network
/// stack: `rpi-hal-embassy`'s adapter brings the chip up itself, because
/// the blocking and awaited bring-ups configure the chip differently and
/// having one owner is what stops them disagreeing. [`discover`] does
/// start it — that is how it answers the cable question — and the
/// adapter's reset undoes that.
//
// The variants are wildly uneven -- an Ethernet chip is about 4 KB and a
// joined radio about 66, the difference being the SDPCM reassembly
// buffers the Wi-Fi driver carries -- so every one of these is the size of
// the largest. Allowed rather than boxed, for two reasons. There is no
// heap to box into: `heap` is an optional feature of this crate and a
// board is entitled to have no allocator at all, which is the usual answer
// to this lint and not available here. And the cost is a one-off: exactly
// one of these exists, it is moved from `discover` into the adapter during
// bring-up, and nothing keeps a collection of them. What it spends is a
// 66 KB memcpy or two against a 1 MiB stack, at a point in the boot where
// the board is otherwise waiting on hardware.
#[allow(clippy::large_enum_variant)]
pub enum Interface {
    /// A Pi 2B/3B's LAN9514, and the address it was given.
    #[cfg(feature = "ethernet")]
    Lan9514(Lan9514, [u8; 6]),
    /// A Pi 3B+'s LAN7800, and the address it was given.
    #[cfg(feature = "ethernet")]
    Lan7800(Lan7800, [u8; 6]),
    /// A joined radio, and the address its firmware reports.
    #[cfg(feature = "wifi")]
    Wifi(Wifi, [u8; 6]),
}

impl Interface {
    /// Which [`Connection`] this is, for a caller reporting what it got.
    pub fn connection(&self) -> Connection {
        match self {
            #[cfg(feature = "ethernet")]
            Interface::Lan9514(..) | Interface::Lan7800(..) => Connection::Ethernet,
            #[cfg(feature = "wifi")]
            Interface::Wifi(..) => Connection::Wifi,
        }
    }

    /// What to call the interface in a log line.
    pub fn name(&self) -> &'static str {
        match self {
            #[cfg(feature = "ethernet")]
            Interface::Lan9514(..) => "LAN9514",
            #[cfg(feature = "ethernet")]
            Interface::Lan7800(..) => "LAN7800",
            #[cfg(feature = "wifi")]
            Interface::Wifi(..) => "Wi-Fi",
        }
    }

    /// The address this interface answers on, which is what the network
    /// stack must be built with.
    pub fn mac(&self) -> [u8; 6] {
        match self {
            #[cfg(feature = "ethernet")]
            Interface::Lan9514(_, mac) | Interface::Lan7800(_, mac) => *mac,
            #[cfg(feature = "wifi")]
            Interface::Wifi(_, mac) => *mac,
        }
    }
}

/// Walks `config.order` and returns the first interface that is present
/// and usable, or `None` if none is.
///
/// Blocking, and meant to run during bring-up before there is an executor:
/// every deadline here is spent waiting on hardware that has nothing to
/// interleave with yet.
///
/// `mac` is the board's address from the firmware mailbox, which is what
/// gets programmed into whichever Ethernet chip is found. A radio ignores
/// it and reports its own — see [`Interface::mac`], which is what a caller
/// should build the stack from rather than this.
// `mac` is Ethernet's alone -- a radio reports its own -- so a build with
// only `wifi` has nothing to do with the argument. Kept in the signature
// rather than feature-gated away, because a function whose parameters
// change shape with the features is one every board has to call twice.
#[cfg_attr(not(feature = "ethernet"), allow(unused_variables))]
pub fn discover(
    hardware: &mut Hardware<'_>,
    timer: &Timer,
    mac: [u8; 6],
    config: &Config,
) -> Option<Interface> {
    for entry in config.order.iter().flatten() {
        match entry {
            Connection::Ethernet => {
                #[cfg(feature = "ethernet")]
                if let Some(interface) = discover_ethernet(hardware, timer, mac, config) {
                    return Some(interface);
                }
                #[cfg(not(feature = "ethernet"))]
                logln!(
                    "net: `ethernet` is in the connection order, but this image was \
                     built without the `ethernet` feature"
                );
            }
            Connection::Wifi => {
                #[cfg(feature = "wifi")]
                if let Some(interface) = discover_wifi(hardware, timer) {
                    return Some(interface);
                }
                #[cfg(not(feature = "wifi"))]
                logln!(
                    "net: `wifi` is in the connection order, but this image was built \
                     without the `wifi` feature"
                );
            }
        }
    }
    logln!("net: no interface came up");
    None
}

/// Brings the radio up through the board's own bring-up and reads the
/// address its firmware reports.
///
/// Short, and that is the point: every expensive part of getting a Pi onto
/// Wi-Fi — firmware, regulatory blob, association — happens inside the
/// closure, which does not return until it has joined. What is left here
/// is the one thing this module needs and the board does not necessarily
/// have to hand.
#[cfg(feature = "wifi")]
fn discover_wifi(hardware: &mut Hardware<'_>, timer: &Timer) -> Option<Interface> {
    let Some(bring_up) = hardware.wifi.as_mut() else {
        logln!(
            "net: `wifi` is in the connection order, but the board offered no way to \
             bring the radio up"
        );
        return None;
    };

    // A failure is the board's to explain: it is the only side that knows
    // whether the firmware was missing, the card unreadable or the
    // passphrase wrong, and those are three different things to go and do.
    let mut wifi = bring_up()?;

    let mut mac = [0u8; 6];
    match wifi.get_iovar("cur_etheraddr", &mut mac, timer) {
        Ok(6) => Some(Interface::Wifi(wifi, mac)),
        // Joined, and then unable to say what address it joined as. Worth
        // its own line rather than folding into the error below: a radio
        // that associates and answers a malformed address is a firmware
        // problem, not a network one.
        Ok(n) => {
            logln!("net: the radio joined but reported a {n}-byte address");
            None
        }
        Err(e) => {
            logln!("net: the radio joined but would not report its address: {e:?}");
            None
        }
    }
}

/// Finds an Ethernet chip and waits for its link, within the two deadlines
/// [`Config`] carries.
#[cfg(feature = "ethernet")]
fn discover_ethernet(
    hardware: &mut Hardware<'_>,
    timer: &Timer,
    mac: [u8; 6],
    config: &Config,
) -> Option<Interface> {
    let Some(dwc2) = hardware.usb else {
        logln!("net: `ethernet` is in the connection order, but the board offered no USB host");
        return None;
    };

    // One deadline covering both halves of "is a chip reachable": the root
    // port reporting a device at all, and then an Ethernet function
    // turning up behind it. They are the same question asked at two
    // depths, and splitting the budget would only invite one half to be
    // tuned without the other.
    let mut waited_ms = 0;

    // The root port does not report instantly — it is a hub coming up, not
    // a register read. Checking once and giving up is a mistake that
    // presents as a board with no Ethernet at all, on a board that has it.
    while !dwc2.port_connected() {
        if waited_ms >= config.chip_timeout_ms {
            logln!(
                "net: nothing on the USB root port within {}ms — USB power \
                 or the DWC2 bring-up is the suspect, not the chip",
                config.chip_timeout_ms
            );
            return None;
        }
        timer.delay_ms(POLL_INTERVAL_MS);
        waited_ms += POLL_INTERVAL_MS;
    }
    logln!("net: root port reported a device after {waited_ms}ms");

    // `Bus` rather than a one-shot walk, and it has to be: a 3B+'s LAN7800
    // sits behind two cascaded hubs and attaches seconds after power-on,
    // so a single pass finishes before it exists.
    let mut bus = Bus::new(dwc2);
    let mut found = None;
    if let Err(e) = bus.enumerate(timer, |channel, timer, event| {
        found = claim(channel, timer, event, mac);
        stop_when(&found)
    }) {
        logln!("net: enumeration failed: {e:?}");
        return None;
    }

    while found.is_none() {
        if waited_ms >= config.chip_timeout_ms {
            logln!(
                "net: no ethernet chip within {}ms — a board without one, \
                 or one slower than that deadline",
                config.chip_timeout_ms
            );
            return None;
        }
        timer.delay_ms(POLL_INTERVAL_MS);
        waited_ms += POLL_INTERVAL_MS;
        if let Err(e) = bus.poll(timer, |channel, timer, event| {
            found = claim(channel, timer, event, mac);
            stop_when(&found)
        }) {
            logln!("net: bus poll failed: {e:?}");
        }
    }

    let mut interface = found?;
    logln!("net: found {}", interface.name());

    // A channel of its own for the probe below, given back when this
    // returns: the adapter allocates its own two later.
    let Some(mut channel) = dwc2.alloc_channel() else {
        logln!("net: no free host channel to probe the link with");
        return None;
    };

    if !link_up_within(&mut interface, &mut channel, timer, mac, config) {
        return None;
    }
    Some(interface)
}

/// Brings the chip up far enough to ask the PHY whether a cable is in, and
/// waits out [`Config::link_timeout_ms`] for the answer.
///
/// This starts the chip with the *blocking* bring-up, which is not the one
/// the network stack's adapter wants — and that is fine, because the
/// adapter resets the chip and starts it again its own way. The cost is
/// one extra reset, a fraction of a second; the alternative is having no
/// way to answer the cable question at all, since the PHY does not report
/// anything until it has been powered and released.
#[cfg(feature = "ethernet")]
fn link_up_within(
    interface: &mut Interface,
    channel: &mut Channel,
    timer: &Timer,
    mac: [u8; 6],
    config: &Config,
) -> bool {
    // One `match`, then generic: the two chips share no types but do share
    // `Ethernet`, so the probe below is written once. Without the trait
    // every step of it would be a two-arm match.
    let name = interface.name();
    match interface {
        Interface::Lan9514(dev, _) => probe_link(dev, channel, timer, mac, config, name),
        Interface::Lan7800(dev, _) => probe_link(dev, channel, timer, mac, config, name),
        // Not reachable: nothing puts a radio through the Ethernet walk.
        #[cfg(feature = "wifi")]
        Interface::Wifi(..) => false,
    }
}

/// [`link_up_within`] once the chip's type is known.
#[cfg(feature = "ethernet")]
fn probe_link<E: Ethernet>(
    dev: &mut E,
    channel: &mut Channel,
    timer: &Timer,
    mac: [u8; 6],
    config: &Config,
    name: &str,
) -> bool {
    if let Err(e) = dev.start(channel, timer, mac) {
        logln!("net: {name} would not start: {e:?}");
        return false;
    }

    let mut waited_ms = 0;
    loop {
        match dev.is_link_up(channel, timer) {
            Ok(true) => {
                logln!("net: link up after {waited_ms}ms");
                return true;
            }
            Ok(false) => {}
            Err(e) => {
                logln!("net: link check failed: {e:?}");
                return false;
            }
        }
        if waited_ms >= config.link_timeout_ms {
            logln!(
                "net: no link within {}ms — the chip is there and nothing \
                 is plugged into it",
                config.link_timeout_ms
            );
            return false;
        }
        timer.delay_ms(POLL_INTERVAL_MS);
        waited_ms += POLL_INTERVAL_MS;
    }
}

/// How often the two waits above look again, in milliseconds.
#[cfg(feature = "ethernet")]
const POLL_INTERVAL_MS: u32 = 250;

/// Stops a bus walk once something has been claimed.
#[cfg(feature = "ethernet")]
fn stop_when(found: &Option<Interface>) -> ControlFlow<()> {
    if found.is_some() {
        ControlFlow::Break(())
    } else {
        ControlFlow::Continue(())
    }
}

/// Takes `event`'s device as whichever Ethernet chip it is.
///
/// Each driver declines a device that is not its own by vendor/product ID,
/// so offering it to both in turn is how the board identifies itself.
#[cfg(feature = "ethernet")]
fn claim(channel: &mut Channel, timer: &Timer, event: Event, mac: [u8; 6]) -> Option<Interface> {
    let Event::Attached(device) = event else {
        return None;
    };

    match Lan9514::from_device(channel, timer, device) {
        Ok(Some(dev)) => return Some(Interface::Lan9514(dev, mac)),
        Ok(None) => {}
        Err(e) => {
            logln!("net: LAN9514 setup failed: {e:?}");
            return None;
        }
    }

    match Lan7800::from_device(channel, timer, device) {
        Ok(Some(dev)) => Some(Interface::Lan7800(dev, mac)),
        Ok(None) => None,
        Err(e) => {
            logln!("net: LAN7800 setup failed: {e:?}");
            None
        }
    }
}

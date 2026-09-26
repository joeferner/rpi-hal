//! Getting a board onto the network, whichever way it can.
//!
//! A board states what it is willing to use and in what order:
//!
//! ```toml
//! connection = ["ethernet", "wifi"]
//! ```
//!
//! and [`discover`] takes the first that is actually there. Nothing above
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
//!   [`Config::chip_timeout_ms`] is.
//! - **No cable.** The chip is there and nothing is plugged into it.
//!   Auto-negotiation takes a second or three from a standing start even
//!   when a cable *is* in, so this is a timeout too — see
//!   [`Config::link_timeout_ms`].
//!
//! Both are deadlines rather than answers, and both cost real time on a
//! board that ends up falling through to Wi-Fi. They are separate settings
//! because they fail differently: a short chip timeout misses a 3B+
//! entirely, while a short link timeout misses a slow switch port.
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
//! **Wi-Fi is not implemented yet.** The [`Connection::Wifi`] variant
//! exists so the configuration shape is settled, and naming it is accepted
//! and reported rather than silently skipped — an interface that is quietly
//! passed over looks exactly like one that was tried and failed.

use core::ops::ControlFlow;

use rpi_hal::timer::Timer;
use rpi_hal::usb::dwc2::{Channel, Dwc2Host};
use rpi_hal::usb::ethernet::Ethernet;
use rpi_hal::usb::lan7800::Lan7800;
use rpi_hal::usb::lan9514::Lan9514;
use rpi_hal::usb::{Bus, Event};

use crate::logln;

/// One kind of network interface a board is willing to use.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Connection {
    /// USB Ethernet — a LAN9514 on a Pi 2B/3B, a LAN7800 on a 3B+.
    Ethernet,
    /// The on-board wireless radio. **Not implemented yet**; naming it is
    /// accepted and reported rather than passed over in silence.
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
    pub link_timeout_ms: u32,
}

impl Default for Config {
    /// Ethernet only, with deadlines sized for the slowest board that
    /// works: ten seconds for the root port to come up and a 3B+'s
    /// Ethernet to attach behind it, then five for auto-negotiation.
    ///
    /// Generous on purpose. These are the numbers a board that *does* have
    /// Ethernet has to fit inside, and getting them wrong that way is a
    /// board with no network at all; getting them wrong the other way
    /// costs a slow boot on a board that was going to use Wi-Fi anyway.
    fn default() -> Self {
        Self {
            order: [Some(Connection::Ethernet), None, None, None],
            chip_timeout_ms: 10_000,
            link_timeout_ms: 5_000,
        }
    }
}

/// An interface that was found, and whose link is up.
///
/// Deliberately **not** started for the network stack: `rpi-hal-embassy`'s
/// adapter brings the chip up itself, because the blocking and awaited
/// bring-ups configure the chip differently and having one owner is what
/// stops them disagreeing. [`discover`] does start it — that is how it
/// answers the cable question — and the adapter's reset undoes that.
pub enum Interface {
    /// A Pi 2B/3B's LAN9514.
    Lan9514(Lan9514),
    /// A Pi 3B+'s LAN7800.
    Lan7800(Lan7800),
}

impl Interface {
    /// Which [`Connection`] this is, for a caller reporting what it got.
    pub fn connection(&self) -> Connection {
        match self {
            Interface::Lan9514(_) | Interface::Lan7800(_) => Connection::Ethernet,
        }
    }

    /// What to call the chip in a log line.
    pub fn name(&self) -> &'static str {
        match self {
            Interface::Lan9514(_) => "LAN9514",
            Interface::Lan7800(_) => "LAN7800",
        }
    }
}

/// Walks `config.order` and returns the first interface that is present
/// with its link up, or `None` if none is.
///
/// Blocking, and meant to run during bring-up before there is an executor:
/// every deadline here is spent waiting on hardware that has nothing to
/// interleave with yet.
///
/// The caller must have powered the USB controller
/// (`rpi_hal::usb::power_on`) and initialised `dwc2`. `mac` is the board's
/// address from the firmware mailbox, which is what gets programmed into
/// whichever chip is found.
pub fn discover(
    dwc2: &Dwc2Host,
    timer: &Timer,
    mac: [u8; 6],
    config: &Config,
) -> Option<Interface> {
    for entry in config.order.iter().flatten() {
        match entry {
            Connection::Ethernet => {
                if let Some(interface) = discover_ethernet(dwc2, timer, mac, config) {
                    return Some(interface);
                }
            }
            Connection::Wifi => {
                logln!("net: wifi is not implemented yet, skipping");
            }
        }
    }
    logln!("net: no interface came up");
    None
}

/// Finds an Ethernet chip and waits for its link, within the two deadlines
/// [`Config`] carries.
fn discover_ethernet(
    dwc2: &Dwc2Host,
    timer: &Timer,
    mac: [u8; 6],
    config: &Config,
) -> Option<Interface> {
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
        found = claim(channel, timer, event);
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
            found = claim(channel, timer, event);
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
        Interface::Lan9514(dev) => probe_link(dev, channel, timer, mac, config, name),
        Interface::Lan7800(dev) => probe_link(dev, channel, timer, mac, config, name),
    }
}

/// [`link_up_within`] once the chip's type is known.
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
const POLL_INTERVAL_MS: u32 = 250;

/// Stops a bus walk once something has been claimed.
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
fn claim(channel: &mut Channel, timer: &Timer, event: Event) -> Option<Interface> {
    let Event::Attached(device) = event else {
        return None;
    };

    match Lan9514::from_device(channel, timer, device) {
        Ok(Some(dev)) => return Some(Interface::Lan9514(dev)),
        Ok(None) => {}
        Err(e) => {
            logln!("net: LAN9514 setup failed: {e:?}");
            return None;
        }
    }

    match Lan7800::from_device(channel, timer, device) {
        Ok(Some(dev)) => Some(Interface::Lan7800(dev)),
        Ok(None) => None,
        Err(e) => {
            logln!("net: LAN7800 setup failed: {e:?}");
            None
        }
    }
}

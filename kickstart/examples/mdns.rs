#![no_std]
#![no_main]

// The mDNS responder on a real link: bring up the on-board Ethernet, take
// a DHCP lease, and answer to `kickstart.local`.
//
// Most of this file is the stack, not the responder. That is the honest
// shape of the thing — `mdns::run` is three arguments and a spawn, and
// everything above it is what a board has to have working first. It is
// also why this example exists at all: the responder cannot be tested
// without a link, and its failure mode is a name that silently does not
// resolve rather than anything that reports.
//
// Two pieces of plumbing are easy to miss and each produces that same
// silence:
//
//   * `Lan9514::set_all_multicast`, below. The chip drops multicast
//     before the host sees it, and every mDNS query and announcement is
//     multicast. Nothing else a board does notices, because DHCP is
//     broadcast — which is why it takes a responder to surface it.
//   * `Stack::join_multicast_group`, which `mdns::run` does itself.
//
// Verify it from another machine on the same link:
//
//     ping kickstart.local
//     dig +short @224.0.0.251 -p 5353 kickstart.local
//
// The second is the legacy-query path (RFC 6762 §6.7) and is the one that
// works from a shell with no mDNS client in the way.
//
// Build it with `scripts/build-example.sh mdns` (kernel7.img) or
// `scripts/build-example64.sh mdns` (kernel8.img).

use embassy_executor::Spawner;
use embassy_net::{Config, StackResources};
use rpi_hal::mailbox::Mailbox;
use rpi_hal::rng::Rng;
use rpi_hal::usb::dwc2::{Channel, Dwc2Host};
use rpi_hal::usb::ethernet::EthernetAsync;
use rpi_hal::usb::lan7800::Lan7800;
use rpi_hal::usb::lan9514::Lan9514;
use rpi_hal::{halt, irq, lic::Lic, pac, timer::Timer, uart::Uart, usb};
use rpi_hal_embassy::ethernet::{EthernetConfig, EthernetDriver, EthernetRunner, EthernetState};
use rpi_hal_embassy::{Executor, time_driver};
use rpi_kickstart::net::{self, Interface};
use rpi_kickstart::{console, logln, mdns};
use static_cell::StaticCell;

/// The name the board answers to. A real board reads this from its
/// settings and passes a function that re-reads it, which is why
/// `mdns::run` takes `fn() -> &'static str` rather than a `&str` — see
/// that module. Here it is fixed, which is the other end of the same
/// seam.
const NAME: &str = "kickstart";

/// Frames queued in each direction between the driver and the stack.
const RX_QUEUE: usize = 4;
const TX_QUEUE: usize = 4;

/// Sockets the stack has room for: the responder's UDP socket, and one
/// spare so a failure here is not the first thing suspected.
const SOCKETS: usize = 2;

static UART: StaticCell<Uart> = StaticCell::new();
static DWC2: StaticCell<Dwc2Host> = StaticCell::new();
static TIMER: StaticCell<Timer> = StaticCell::new();
static STATE: StaticCell<EthernetState<RX_QUEUE, TX_QUEUE>> = StaticCell::new();
static RESOURCES: StaticCell<StackResources<SOCKETS>> = StaticCell::new();
static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // Not `logln!`: the console may be mid-write, and re-entering its
    // `RefCell` would turn one panic into an endless pair of them.
    use core::fmt::Write as _;
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

// No `__irq_handler` here: `rpi-hal-embassy`'s `irq-dispatch` feature
// defines one that services every source this program has — the System
// Timer that `embassy-time` runs on, and the USB controller the driver's
// transfers are parked on. Without a handler the first deadline
// livelocks the core, which looks exactly like a hang somewhere else.

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, EthernetDriver<'static>>) -> ! {
    runner.run().await
}

// One per chip, and not because they differ -- the bodies are identical.
// `#[embassy_executor::task]` allocates a pool of the future's concrete
// type, so a generic task has no size to allocate.
#[embassy_executor::task]
async fn lan9514_task(runner: EthernetRunner<'static, 'static, Lan9514>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn lan7800_task(runner: EthernetRunner<'static, 'static, Lan7800>) -> ! {
    runner.run().await
}

// The crate hands back an `async fn` and the application wraps it, which
// is the whole reason `mdns::run` is not a task itself:
// `#[embassy_executor::task]` cannot be generic, so the concrete types
// have to be named somewhere the application owns.
#[embassy_executor::task]
async fn mdns_task(stack: embassy_net::Stack<'static>) -> ! {
    mdns::run(stack, || NAME).await
}

/// Reports the lease once, so there is an address to compare what
/// `kickstart.local` resolves to against.
#[embassy_executor::task]
async fn report_task(stack: embassy_net::Stack<'static>) {
    logln!("waiting for DHCP...");
    stack.wait_config_up().await;
    if let Some(config) = stack.config_v4() {
        logln!(
            "DHCP: {} — this is what {NAME}.local should resolve to",
            config.address
        );
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    console::init(
        UART.init(Uart::init(&peripherals.GPIO, peripherals.UART0)),
        // The realistic form of this seam: `Instant::now()` captures
        // nothing, so it coerces to the `fn` pointer `console::init`
        // takes. It reads correctly before `time_driver::init` because
        // the System Timer has been running since boot.
        || embassy_time::Instant::now().as_micros(),
    );
    logln!("rpi-kickstart mdns");

    let timer = TIMER.init(Timer::new(peripherals.SYSTMR));
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    if !usb::power_on(&mut mailbox) {
        logln!("USB power-on failed");
        halt();
    }

    let mac = match mailbox.mac_address() {
        Ok(mac) => mac,
        Err(e) => {
            logln!("MAC read failed: {e:?}");
            halt();
        }
    };
    logln!(
        "board MAC: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5]
    );

    let dwc2: &'static Dwc2Host = DWC2.init(Dwc2Host::init(
        peripherals.USB_OTG_GLOBAL,
        peripherals.USB_OTG_HOST,
        peripherals.USB_OTG_PWRCLK,
        timer,
    ));

    // Everything from "is there a hub" to "is a cable in" is `net`'s, and
    // that is the point of the module: the deadlines involved are the
    // whole diagnosis when a board comes up with no network, and no board
    // should have to write them out again.
    let Some(interface) = net::discover(dwc2, timer, mac, &net::Config::default()) else {
        halt();
    };
    logln!("net: using {}", interface.name());

    // Two host channels for the stack, one per direction. The walk's own
    // is gone with it; these last as long as the interface does.
    let (Some(rx_channel), Some(tx_channel)) = (dwc2.alloc_channel(), dwc2.alloc_channel()) else {
        logln!("no free host channels for the stack");
        halt();
    };

    // The only place the chip is named. `run` is generic; all this picks
    // is which task function gets spawned, since that is the one thing
    // that cannot be.
    match interface {
        Interface::Lan9514(dev) => run(rx_channel, tx_channel, timer, dev, mac, |s, r| {
            s.spawn(lan9514_task(r).unwrap())
        }),
        Interface::Lan7800(dev) => run(rx_channel, tx_channel, timer, dev, mac, |s, r| {
            s.spawn(lan7800_task(r).unwrap())
        }),
    }
}

/// Hands the interface to `embassy-net` and runs the executor. Never
/// returns.
///
/// Generic over `EthernetAsync`, so a LAN9514 and a LAN7800 take the same
/// path. `spawn_eth` is the one chip-specific line, passed in because
/// `#[embassy_executor::task]` cannot be generic and this function has no
/// way to name the opaque `SpawnToken` a task returns.
fn run<E, F>(
    rx_channel: Channel<'static>,
    tx_channel: Channel<'static>,
    timer: &Timer,
    ethernet: E,
    mac: [u8; 6],
    spawn_eth: F,
) -> !
where
    E: EthernetAsync + 'static,
    F: FnOnce(Spawner, EthernetRunner<'static, 'static, E>),
{
    // Not started here. `net::discover` started it to ask the PHY whether
    // a cable was in, and the adapter resets it and starts it again its
    // own way -- the blocking and awaited bring-ups configure the chip
    // differently, and one owner is what keeps them from disagreeing.
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);
    time_driver::init(Timer::new(peripherals.SYSTMR), &lic);
    lic.enable_usb_irq();
    irq::enable_irq();

    // The `Timer` outlives this frame because it came from a `StaticCell`
    // in `kmain`; the borrow is just narrower than its lifetime.
    let timer: &'static Timer = unsafe { &*(timer as *const Timer) };

    let state = STATE.init(EthernetState::new());
    let (driver, eth_runner) = rpi_hal_embassy::ethernet::new(
        state,
        ethernet,
        rx_channel,
        tx_channel,
        timer,
        // `all_multicast` is the half of the plumbing `mdns::run` cannot do
        // for itself. Without it the chip filters multicast out before the
        // host sees it and the responder binds a socket nothing ever
        // arrives on -- while still announcing, since announcements are
        // transmitted. The symptom is a name that resolves once and then
        // never answers again.
        EthernetConfig {
            mac,
            all_multicast: true,
        },
    );

    // A random seed keeps TCP initial sequence numbers and the DHCP
    // transaction ID from repeating across boots.
    let mut rng = Rng::new();
    let seed = (u64::from(rng.next_u32()) << 32) | u64::from(rng.next_u32());

    let resources = RESOURCES.init(StackResources::new());
    let (stack, runner) =
        embassy_net::new(driver, Config::dhcpv4(Default::default()), resources, seed);

    EXECUTOR.init(Executor::new()).run(|spawner| {
        // In embassy-executor 0.10 it is the *task function* that returns
        // the `Result` — a token, or `SpawnError` if its pool is already
        // full — and `spawn` takes the token. Every task here has a pool
        // of one and is spawned once, so the unwrap is unreachable.
        spawner.spawn(net_task(runner).unwrap());
        spawn_eth(spawner, eth_runner);
        spawner.spawn(report_task(stack).unwrap());
        spawner.spawn(mdns_task(stack).unwrap());
    });
}

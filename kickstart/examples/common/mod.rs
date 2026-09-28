// The bring-up every network example shares, so that each example file is
// the part that example is about.
//
// Not an example of its own: Cargo takes `examples/*.rs` and
// `examples/*/main.rs` as examples, and this is `examples/common/mod.rs`,
// pulled in with `mod common;`. It is compiled once per example with that
// example's features, which is why everything optional below is behind
// the feature it needs -- and why `dead_code` is allowed: each example
// uses a different subset.
//
// Order of use, in every example:
//
//   1. `boot`     -- console, then (with `heap`) the heap.
//   2. `usb`      -- power the USB side and read the board's MAC.
//   3. `discover` -- `net::discover`, over whatever hardware the example
//                    offers it; logs which interface won.
//   4. `start`    -- interrupts, the queue pair, the adapter, the stack,
//                    and the executor, spawning the example's own tasks.

#![allow(dead_code)]

use embassy_executor::Spawner;
use embassy_net::{Config, Stack, StackResources};
use rpi_hal::mailbox::Mailbox;
use rpi_hal::usb::dwc2::Dwc2Host;
use rpi_hal::usb::ethernet::EthernetAsync;
use rpi_hal::usb::lan7800::Lan7800;
use rpi_hal::usb::lan9514::Lan9514;
use rpi_hal::{halt, irq, lic::Lic, pac, timer::Timer, uart::Uart, usb};
use rpi_hal_embassy::channel as ch;
use rpi_hal_embassy::ethernet::{EthernetConfig, EthernetRunner};
use rpi_hal_embassy::{Executor, time_driver};
use rpi_kickstart::net::{self, Interface};
use rpi_kickstart::{console, logln};
use static_cell::StaticCell;

#[cfg(all(feature = "storage", feature = "config"))]
pub mod settings;

/// Largest frame the queue pair carries, and the reason there is one pair
/// rather than one per interface.
///
/// With `wifi`, the check below is load-bearing. Both adapters hand the
/// stack a `ch::Device<'_, MTU>`, and they are the *same type* only because
/// both MTUs are 1514 — `ch::Device` differs by const generic, so an
/// interface that carried a different frame would make them unrelated
/// types and the single-stack arrangement would stop compiling. Better to
/// say so here than as a wall of type errors.
pub const MTU: usize = rpi_hal_embassy::ethernet::MTU;
#[cfg(feature = "wifi")]
const _: () = assert!(
    MTU == rpi_hal_embassy::wifi::MTU,
    "the Ethernet and Wi-Fi adapters must share an MTU for one stack to run over either"
);

/// Frames queued in each direction between the driver and the stack.
const RX_QUEUE: usize = 4;
const TX_QUEUE: usize = 4;

/// Sockets the stack has room for — enough for the busiest example: the
/// responder's UDP socket, the SNTP client's, the DNS one `embassy-net`
/// opens to resolve names, the TLS check's TCP socket, and one spare so a
/// failure here is not the first thing suspected. Too few is a request
/// that fails with "could not be sent", which looks like a network problem
/// and is not.
const SOCKETS: usize = 5;

static UART: StaticCell<Uart> = StaticCell::new();
static TIMER: StaticCell<Timer> = StaticCell::new();
static DWC2: StaticCell<Dwc2Host> = StaticCell::new();
static STATE: StaticCell<ch::State<MTU, RX_QUEUE, TX_QUEUE>> = StaticCell::new();
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
// defines one that services every source these programs have — the System
// Timer that `embassy-time` runs on, and the USB controller the driver's
// transfers are parked on. Without a handler the first deadline livelocks
// the core, which looks exactly like a hang somewhere else.

/// What `boot` hands every later step.
pub struct Board {
    pub timer: &'static Timer,
    pub mailbox: Mailbox,
}

/// The console, then the heap if the example has one. `title` is the
/// first line on the console, so a log says which image it came from.
pub fn boot(title: &str) -> Board {
    let peripherals = unsafe { pac::Peripherals::steal() };
    console::init(
        UART.init(Uart::init(&peripherals.GPIO, peripherals.UART0)),
        // `Instant::now()` captures nothing, so it coerces to the `fn`
        // pointer `console::init` takes. It reads correctly before
        // `time_driver::init` because the System Timer has been running
        // since boot.
        || embassy_time::Instant::now().as_micros(),
    );
    logln!("rpi-kickstart {title}");

    let timer = TIMER.init(Timer::new(peripherals.SYSTMR));
    #[allow(unused_mut)]
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // Before anything allocates: mounting the card reads its allocation
    // table onto the heap, and parsing settings builds strings.
    #[cfg(feature = "heap")]
    match rpi_kickstart::heap::init(&mut mailbox) {
        Ok(bytes) => logln!("heap: {} MiB", bytes / (1024 * 1024)),
        Err(e) => {
            logln!("heap: {e:?}");
            halt();
        }
    }

    Board { timer, mailbox }
}

/// The USB side, powered, and the board's own MAC address.
pub struct Usb {
    pub dwc2: &'static Dwc2Host,
    pub mac: [u8; 6],
}

/// Powers the USB controller and reads the MAC the firmware reports — the
/// address the Ethernet chips are programmed with.
pub fn usb(board: &mut Board) -> Usb {
    if !usb::power_on(&mut board.mailbox) {
        logln!("USB power-on failed");
        halt();
    }
    let mac = match board.mailbox.mac_address() {
        Ok(mac) => mac,
        Err(e) => {
            logln!("MAC read failed: {e:?}");
            halt();
        }
    };
    logln!("board MAC: {}", Mac(mac));

    let peripherals = unsafe { pac::Peripherals::steal() };
    let dwc2 = DWC2.init(Dwc2Host::init(
        peripherals.USB_OTG_GLOBAL,
        peripherals.USB_OTG_HOST,
        peripherals.USB_OTG_PWRCLK,
        board.timer,
    ));
    Usb { dwc2, mac }
}

/// Runs `net::discover` over `hardware`, logging which interface won and
/// its address; halts if none did.
///
/// Everything from "is there a hub" to "is a cable in" is `net`'s: the
/// deadlines involved are the whole diagnosis when a board comes up with no
/// network, and `net` logs each one that expires.
pub fn discover(hardware: &mut net::Hardware<'_>, board: &Board, usb: &Usb) -> Interface {
    let Some(interface) = net::discover(hardware, board.timer, usb.mac, &net::Config::default())
    else {
        halt();
    };
    // The address as well as the name, because they differ per interface
    // and the stack is built from this one: the Ethernet chips take the
    // board address, the radio has its own. A stack built with the wrong
    // one takes a lease, announces itself — and then answers nothing sent
    // to it. Worth a line to compare against a peer's ARP table.
    logln!(
        "net: using {} at {}",
        interface.name(),
        Mac(interface.mac())
    );
    interface
}

/// Brings the chosen interface up behind a stack and runs the executor,
/// spawning the network tasks and then `spawn`, the example's own. Never
/// returns.
pub fn start(
    board: &Board,
    usb: &Usb,
    interface: Interface,
    spawn: impl FnOnce(Spawner, Stack<'static>),
) -> ! {
    // From here on the interrupts are live, so nothing before this may
    // still be blocking on the USB controller or the SDIO bus: both
    // bring-ups poll, and an interrupt arriving mid-poll is serviced by a
    // handler that knows nothing about them.
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);
    time_driver::init(Timer::new(peripherals.SYSTMR), &lic);
    irq::enable_irq();

    // The queue pair, built once and here rather than inside an adapter,
    // which is what lets either interface attach to it. The address is the
    // interface's own, not the board's.
    let state = STATE.init(ch::State::new());
    let (ch_runner, driver) = ch::new(
        state,
        ch::driver::HardwareAddress::Ethernet(interface.mac()),
    );

    // The only place an interface is named. Each arm attaches its runner
    // and spawns it, which is the one thing that cannot be written once:
    // `#[embassy_executor::task]` needs a concrete future type.
    let timer = board.timer;
    match interface {
        Interface::Lan9514(dev, _) => {
            let runner = attach_ethernet(ch_runner, dev, usb, timer, &lic);
            run(driver, |s| s.spawn(lan9514_task(runner).unwrap()), spawn)
        }
        Interface::Lan7800(dev, _) => {
            let runner = attach_ethernet(ch_runner, dev, usb, timer, &lic);
            run(driver, |s| s.spawn(lan7800_task(runner).unwrap()), spawn)
        }
        #[cfg(feature = "wifi")]
        Interface::Wifi(wifi, _) => {
            // No USB interrupt on this path: the radio is driven over SDIO
            // by a runner that polls, and the USB controller -- walked a
            // moment ago and found wanting -- has nothing outstanding.
            let runner = rpi_hal_embassy::wifi::attach(ch_runner, wifi, timer);
            run(driver, |s| s.spawn(wifi_task(runner).unwrap()), spawn)
        }
    }
}

/// Puts a USB Ethernet chip behind the queue pair, and turns on the
/// interrupt its transfers park on.
///
/// Generic over `EthernetAsync`, so a LAN9514 and a LAN7800 take the same
/// path. The chip is **not** started here: `net::discover` started it to
/// ask the PHY whether a cable was in, and the adapter resets and starts it
/// again its own way, since the blocking and awaited bring-ups configure it
/// differently.
fn attach_ethernet<E: EthernetAsync + 'static>(
    ch_runner: ch::Runner<'static, MTU>,
    ethernet: E,
    usb: &Usb,
    timer: &'static Timer,
    lic: &Lic,
) -> EthernetRunner<'static, 'static, E> {
    // Two host channels, one per direction, held for as long as the
    // interface is. The walk's own went back when it returned.
    let (Some(rx_channel), Some(tx_channel)) = (usb.dwc2.alloc_channel(), usb.dwc2.alloc_channel())
    else {
        logln!("no free host channels for the stack");
        halt();
    };
    lic.enable_usb_irq();

    rpi_hal_embassy::ethernet::attach(
        ch_runner,
        ethernet,
        rx_channel,
        tx_channel,
        timer,
        // `all_multicast` is the half of the mDNS plumbing `mdns::run`
        // cannot do for itself: without it the chip filters multicast out
        // before the host sees it, and a responder binds a socket nothing
        // ever arrives on while still announcing. On regardless of whether
        // the example runs a responder -- it costs nothing, and an example
        // that later gains one should not have to rediscover it.
        EthernetConfig {
            mac: usb.mac,
            all_multicast: true,
        },
    )
}

/// Builds the stack and runs the executor. `spawn_interface` is the one
/// line that cannot be written once, passed in because this function has
/// no way to name the opaque token a task returns.
fn run(
    driver: ch::Device<'static, MTU>,
    spawn_interface: impl FnOnce(Spawner),
    spawn: impl FnOnce(Spawner, Stack<'static>),
) -> ! {
    let resources = RESOURCES.init(StackResources::new());
    let (stack, runner) = embassy_net::new(
        driver,
        Config::dhcpv4(Default::default()),
        resources,
        seed(),
    );

    EXECUTOR.init(Executor::new()).run(|spawner| {
        // In embassy-executor 0.10 the *task function* returns the
        // `Result` -- a token, or `SpawnError` if its pool is full -- and
        // `spawn` takes the token. Every task here has a pool of one and
        // is spawned once, so the unwrap is unreachable.
        spawner.spawn(net_task(runner).unwrap());
        spawn_interface(spawner);
        spawner.spawn(lease_task(stack).unwrap());
        spawn(spawner, stack);
    });
}

/// A random seed for the stack, which keeps TCP initial sequence numbers,
/// the DHCP transaction ID and the first dynamic port from repeating across
/// boots.
///
/// From `entropy` when the example has it: that module holds the one
/// generator TLS also draws from, and a second instance would arm a second
/// warm-up on the same hardware.
fn seed() -> u64 {
    let mut seed = [0u8; 8];
    #[cfg(feature = "entropy")]
    rpi_kickstart::entropy::fill(&mut seed);
    #[cfg(not(feature = "entropy"))]
    {
        let mut rng = rpi_hal::rng::Rng::new();
        seed[..4].copy_from_slice(&rng.next_u32().to_le_bytes());
        seed[4..].copy_from_slice(&rng.next_u32().to_le_bytes());
    }
    u64::from_le_bytes(seed)
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, ch::Device<'static, MTU>>) -> ! {
    runner.run().await
}

// One per interface, and for the Ethernet pair not because they differ --
// the bodies are identical. `#[embassy_executor::task]` allocates a pool
// of the future's concrete type, so a generic task has no size to allocate.
#[embassy_executor::task]
async fn lan9514_task(runner: EthernetRunner<'static, 'static, Lan9514>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn lan7800_task(runner: EthernetRunner<'static, 'static, Lan7800>) -> ! {
    runner.run().await
}

#[cfg(feature = "wifi")]
#[embassy_executor::task]
async fn wifi_task(runner: rpi_hal_embassy::wifi::WifiRunner<'static>) -> ! {
    runner.run().await
}

/// Reports the lease once, and any Ethernet bring-up failure.
#[embassy_executor::task]
async fn lease_task(stack: Stack<'static>) {
    logln!("waiting for DHCP...");
    stack.wait_config_up().await;
    if let Some(config) = stack.config_v4() {
        logln!("DHCP: {}", config.address);
    }

    // The adapter records a bring-up failure rather than logging it. That
    // matters most for the multicast enable: it is not fatal, so a chip
    // that refused it carries every other frame and looks healthy while a
    // responder answers nothing it is asked.
    if let Some(error) = rpi_hal_embassy::ethernet::start_error() {
        logln!("ethernet: something in bring-up failed: {error:?}");
    }
}

/// A MAC address, printed the usual way.
pub struct Mac(pub [u8; 6]);

impl core::fmt::Display for Mac {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let [a, b, c, d, e, g] = self.0;
        write!(f, "{a:02x}:{b:02x}:{c:02x}:{d:02x}:{e:02x}:{g:02x}")
    }
}

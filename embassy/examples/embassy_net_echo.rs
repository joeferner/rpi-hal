#![no_std]
#![no_main]

// A TCP/IP stack on the on-board Ethernet, driven by `embassy-net` — the
// async counterpart to rpi-hal's `usb_ethernet_smoltcp.rs`, and the thing
// this crate's `ethernet` adapter exists for.
//
// Whichever Ethernet chip the board has. A Pi 2B/3B has an SMSC LAN9514
// (hub and Ethernet in one); a 3B+ has a Microchip LAN7515, which is two
// cascaded hubs with a LAN7800 Ethernet behind the second. Each driver
// declines a device that is not its own, so offering every enumerated
// device to both is how the board identifies itself, and everything past
// that point is written against `EthernetAsync` and never learns which it
// got.
//
// Bring-up: power the USB controller, start DWC2, walk the bus, claim the
// Ethernet function, then hand it to `embassy-net`. The chip is *not*
// started here — the adapter's runner does that, for the reason
// `rpi_hal_embassy::ethernet::new` gives.
//
// Three tasks:
//
// - `net_task` runs `embassy-net`'s own runner, which owns the stack and
//   serves it from the adapter's queues.
// - `lan9514_task`/`lan7800_task` run the adapter's runner, which brings
//   the chip up and then moves frames between those queues and its two
//   bulk endpoints. It awaits the USB controller's interrupt rather than
//   polling, so there is no ticker here and no poll interval for the
//   application to pick — which is why `__irq_handler` below has to
//   dispatch that interrupt. There are two of them only because
//   `#[embassy_executor::task]` cannot be generic; the code is identical.
// - `echo_task` waits for DHCP, prints the lease, then serves TCP echo on
//   port 7 (RFC 862).
//
// What that proves, end to end: ARP, the driver adapter in both
// directions, a DHCP exchange, ICMP (the stack answers pings by itself
// with `auto-icmp-echo-reply`), and a real TCP connection. Try
// `ping <address>` and `nc <address> 7`.

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config, StackResources};
use rpi_hal::mailbox::Mailbox;
use rpi_hal::rng::Rng;
use rpi_hal::usb::dwc2::{Channel, Dwc2Host};
use rpi_hal::usb::ethernet::EthernetAsync;
use rpi_hal::usb::lan7800::Lan7800;
use rpi_hal::usb::lan9514::Lan9514;
use rpi_hal::usb::{Bus, Event};
use rpi_hal::{halt, irq, lic::Lic, pac, timer::Timer, uart::Uart, usb};
use rpi_hal_embassy::ethernet::{EthernetConfig, EthernetDriver, EthernetRunner, EthernetState};
use rpi_hal_embassy::{Executor, time_driver};

/// Frames the adapter may hold queued inbound. Four absorbs a small burst
/// without the receive loop having to wait on the stack; each costs one
/// MTU of RAM.
const RX_QUEUE: usize = 4;

/// The outbound counterpart to [`RX_QUEUE`].
const TX_QUEUE: usize = 4;

/// TCP port the echo server listens on — the Echo Protocol's conventional
/// port (RFC 862).
const ECHO_PORT: u16 = 7;

/// Sockets the stack may hold at once.
const SOCKETS: usize = 2;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// Widens a borrow to `'static`. Sound only where the caller never
/// returns, which is the case for every use below — the same lifetime
/// widening `#[embassy_executor::main]` performs, and the reason it
/// requires a diverging entry point.
unsafe fn make_static<T>(t: &mut T) -> &'static mut T {
    unsafe { core::mem::transmute(t) }
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, EthernetDriver<'static>>) -> ! {
    runner.run().await
}

// One task per chip, and not because the code differs — it is identical.
// `#[embassy_executor::task]` allocates a pool of the future's concrete
// type, so a generic task has no size to allocate and the attribute will
// not take one. The generic part is everything else; only the spawn has to
// know which it got, and `run` takes a closure to do it (see there).
#[embassy_executor::task]
async fn lan9514_task(runner: EthernetRunner<'static, 'static, Lan9514>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn lan7800_task(runner: EthernetRunner<'static, 'static, Lan7800>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn echo_task(stack: embassy_net::Stack<'static>, mut uart: Uart) {
    let _ = writeln!(uart, "waiting for DHCP...");
    stack.wait_config_up().await;

    if let Some(config) = stack.config_v4() {
        let _ = writeln!(uart, "DHCP: {}", config.address);
        if let Some(gateway) = config.gateway {
            let _ = writeln!(uart, "gateway: {gateway}");
        }
    }
    let _ = writeln!(uart, "echo server on port {ECHO_PORT}");

    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 1024];
    let mut buf = [0u8; 256];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

        if socket.accept(ECHO_PORT).await.is_err() {
            continue;
        }
        let _ = writeln!(uart, "connected");

        loop {
            match socket.read(&mut buf).await {
                // A zero-length read is the peer closing, not an idle
                // connection: the await only returns once there is
                // something to report.
                Ok(0) | Err(_) => break,
                Ok(n) => {
                    if socket.write(&buf[..n]).await.is_err() {
                        break;
                    }
                }
            }
        }

        // Discard rather than close: `abort` frees the socket without
        // waiting out TIME_WAIT, so the next connection can be accepted
        // straight away.
        socket.abort();
        let _ = writeln!(uart, "closed");
    }
}

// `unsafe(no_mangle)` rather than the bare `no_mangle` rpi-hal's own
// examples use: this crate is edition 2024, which requires the wrapper.
#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "embassy-net over the on-board Ethernet");

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

    // `Bus` rather than `usb::enumerate`, and it has to be. A Pi 3B+'s
    // LAN7800 sits behind two cascaded hubs and attaches *seconds* after
    // power-on, so a one-shot walk finishes before it exists and reports
    // an empty bus. On a 2B/3B the LAN9514 is there from the start and the
    // first walk finds it, so the poll loop below never runs.
    let mut bus = Bus::new(dwc2);
    let mut uart = Some(uart);
    let mut found = None;

    let result = bus.enumerate(&timer, |channel, timer, event| {
        found = claim(uart.as_mut().unwrap(), channel, timer, event);
        break_when_found(&found)
    });
    if let Err(e) = result {
        let _ = writeln!(uart.as_mut().unwrap(), "enumeration failed: {e:?}");
        halt();
    }
    if found.is_none() {
        let _ = writeln!(
            uart.as_mut().unwrap(),
            "waiting for an Ethernet function to attach..."
        );
    }
    while found.is_none() {
        let result = bus.poll(&timer, |channel, timer, event| {
            found = claim(uart.as_mut().unwrap(), channel, timer, event);
            break_when_found(&found)
        });
        if let Err(e) = result {
            let _ = writeln!(uart.as_mut().unwrap(), "poll failed: {e:?}");
        }
        timer.delay_ms(250);
    }

    // The stack needs two channels of its own, one per direction — see the
    // adapter's documentation for why it is two. The walk's own channel is
    // gone with it, while the runner below keeps moving frames forever.
    let (Some(rx_channel), Some(tx_channel)) = (dwc2.alloc_channel(), dwc2.alloc_channel()) else {
        let _ = writeln!(
            uart.as_mut().unwrap(),
            "no free host channels for the stack"
        );
        halt();
    };
    let uart = uart.take().unwrap();

    // The only place either chip is named. `run` is generic over
    // `EthernetAsync`; all this decides is which task function gets
    // spawned, because that is the one thing that cannot be.
    match found.expect("the loop above only exits once it is set") {
        Board::Lan9514(dev) => run(uart, rx_channel, tx_channel, &timer, dev, mac, |s, r| {
            s.spawn(lan9514_task(r).unwrap())
        }),
        Board::Lan7800(dev) => run(uart, rx_channel, tx_channel, &timer, dev, mac, |s, r| {
            s.spawn(lan7800_task(r).unwrap())
        }),
    }
}

/// Whichever Ethernet chip this board turned out to have.
enum Board {
    /// A Pi 2B/3B's LAN9514 — hub and Ethernet in one chip.
    Lan9514(Lan9514),
    /// A Pi 3B+'s LAN7800, behind the LAN7515's two hubs.
    Lan7800(Lan7800),
}

/// Stops the walk once something has been claimed.
fn break_when_found(found: &Option<Board>) -> core::ops::ControlFlow<()> {
    if found.is_some() {
        core::ops::ControlFlow::Break(())
    } else {
        core::ops::ControlFlow::Continue(())
    }
}

/// Takes `event`'s device as whichever Ethernet chip it is.
///
/// Each driver declines a device that is not its own by vendor/product ID,
/// so offering it to both in turn is how the board identifies itself —
/// nothing here has to know which Pi it is running on.
fn claim(uart: &mut Uart, channel: &mut Channel, timer: &Timer, event: Event) -> Option<Board> {
    let Event::Attached(device) = event else {
        return None;
    };

    match Lan9514::from_device(channel, timer, device) {
        Ok(Some(dev)) => {
            let _ = writeln!(
                uart,
                "LAN9514 on hub {} port {}",
                device.hub_address, device.port
            );
            return Some(Board::Lan9514(dev));
        }
        Ok(None) => {}
        Err(e) => {
            let _ = writeln!(uart, "LAN9514 setup failed: {e:?}");
            return None;
        }
    }

    match Lan7800::from_device(channel, timer, device) {
        Ok(Some(dev)) => {
            let _ = writeln!(
                uart,
                "LAN7800 on hub {} port {}",
                device.hub_address, device.port
            );
            Some(Board::Lan7800(dev))
        }
        Ok(None) => None,
        Err(e) => {
            let _ = writeln!(uart, "LAN7800 setup failed: {e:?}");
            None
        }
    }
}

/// Services the interrupts the executor and the Ethernet adapter depend
/// on.
///
/// Mandatory, and silently fatal to omit: `rpi-hal` provides only a *weak*
/// no-op `__irq_handler`, so without this the first `embassy-time`
/// deadline fires, nothing acknowledges the Compare 1 match, the interrupt
/// controller keeps asserting, and the core re-enters the handler
/// forever. Every task stops making progress at that instant, which on the
/// console looks like a hang immediately after bring-up.
///
/// The USB line is what completes the adapter's transfers. Omitting *that*
/// half fails differently and more quietly: bring-up prints normally, then
/// no frame is ever received and DHCP never completes, because the
/// runner's receive is parked on a channel nothing will ever report the
/// halt of.
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

/// Hands the chip to `embassy-net` and runs forever, generic over
/// [`EthernetAsync`] so a LAN9514 and a LAN7800 take the same path.
///
/// `spawn_eth` exists because that genericity stops at the spawn:
/// `#[embassy_executor::task]` needs a concrete future type for its pool,
/// so the runner's task cannot be generic. A closure is what carries the
/// one chip-specific line in without this function having to name the
/// opaque `SpawnToken` type it returns.
fn run<E, F>(
    mut uart: Uart,
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
    // Not started here: the adapter's runner does it, on the receive
    // channel and before either channel takes up its frame duties. The
    // bring-up and the awaited frame path have to agree about how the chip
    // answers an empty receive, and having one owner is what makes that
    // impossible to get wrong — see `rpi_hal_embassy::ethernet::new`.
    let _ = writeln!(uart, "handing the interface to embassy-net");

    // The time driver needs the System Timer, but `timer` here is only
    // borrowed from `kmain`; steal a second handle rather than restructure
    // the bring-up around it. Both refer to the same free-running counter,
    // and only this one touches Compare 1.
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);
    time_driver::init(Timer::new(peripherals.SYSTMR), &lic);

    // The adapter's transfers complete on this interrupt and on nothing
    // else — including the bring-up, which now happens inside the runner
    // and is awaited like everything after it. Enabled before the executor
    // starts rather than after the chip is up, which is the order that
    // changed when the adapter took bring-up over: there are no blocking
    // transfers left here to race the handler for `HCINT`.
    lic.enable_usb_irq();
    irq::enable_irq();

    // Everything below outlives `run`, which never returns.
    let timer: &'static Timer = unsafe { &*(timer as *const Timer) };

    let mut state = EthernetState::<RX_QUEUE, TX_QUEUE>::new();
    let state = unsafe { make_static(&mut state) };
    let (driver, eth_runner) = rpi_hal_embassy::ethernet::new(
        state,
        ethernet,
        rx_channel,
        tx_channel,
        timer,
        EthernetConfig::new(mac),
    );

    // A random seed keeps TCP initial sequence numbers and the DHCP
    // transaction ID from repeating across boots. The hardware RNG is
    // right here, so there is no reason to use a fixed one.
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
        spawn_eth(spawner, eth_runner);
        spawner.spawn(echo_task(stack, uart).unwrap());
    });
}

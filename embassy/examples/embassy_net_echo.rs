#![no_std]
#![no_main]

// A TCP/IP stack on the on-board Ethernet, driven by `embassy-net` — the
// async counterpart to rpi-hal's `usb_ethernet_smoltcp.rs`, and the thing
// this crate's `lan9514` adapter exists for.
//
// Bring-up is identical to that example: power the USB controller, start
// DWC2, enumerate the bus, find the LAN9514, program the firmware MAC and
// enable RX/TX. From there this hands the chip to `embassy-net` instead of
// running a `smoltcp` poll loop by hand.
//
// Three tasks:
//
// - `net_task` runs `embassy-net`'s own runner, which owns the stack and
//   serves it from the adapter's queues.
// - `lan9514_task` runs the adapter's runner, which moves frames between
//   those queues and the chip's two bulk endpoints. It awaits the USB
//   controller's interrupt rather than polling, so there is no ticker
//   here and no poll interval for the application to pick — which is why
//   `__irq_handler` below has to dispatch that interrupt.
// - `echo_task` waits for DHCP, prints the lease, then serves TCP echo on
//   port 7 (RFC 862).
//
// What that proves, end to end: ARP, the driver adapter in both
// directions, a DHCP exchange, ICMP (the stack answers pings by itself
// with `auto-icmp-echo-reply`), and a real TCP connection. Try
// `ping <address>` and `nc <address> 7`.

use core::fmt::Write as _;

use embassy_net::tcp::TcpSocket;
use embassy_net::{Config, StackResources};
use rpi_hal::mailbox::Mailbox;
use rpi_hal::rng::Rng;
use rpi_hal::usb::dwc2::{Channel, Dwc2Host};
use rpi_hal::usb::lan9514::Lan9514;
use rpi_hal::{halt, irq, lic::Lic, pac, timer::Timer, uart::Uart, usb};
use rpi_hal_embassy::lan9514::{Lan9514Driver, Lan9514Runner, Lan9514State};
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
async fn net_task(mut runner: embassy_net::Runner<'static, Lan9514Driver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn lan9514_task(runner: Lan9514Runner<'static, 'static>) -> ! {
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

/// Brings the chip up and hands it to `embassy-net`. Never returns.
fn run(
    mut uart: Uart,
    mut rx_channel: Channel<'static>,
    tx_channel: Channel<'static>,
    timer: &Timer,
    mut lan9514: Lan9514,
    mac: [u8; 6],
) -> ! {
    // Started over the receive channel, on endpoint 0 — bring-up is
    // control transfers, and happens before either channel takes up its
    // frame duties.
    if let Err(e) = lan9514.start(&mut rx_channel, timer, mac) {
        let _ = writeln!(uart, "LAN9514 start failed: {e:?}");
        halt();
    }
    let _ = writeln!(uart, "LAN9514 started, handing it to embassy-net");

    // The time driver needs the System Timer, but `timer` here is only
    // borrowed from `kmain`; steal a second handle rather than restructure
    // the bring-up around it. Both refer to the same free-running counter,
    // and only this one touches Compare 1.
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);
    time_driver::init(Timer::new(peripherals.SYSTMR), &lic);

    // The adapter's transfers complete on this interrupt and on nothing
    // else. Enabled after `start`, deliberately: bring-up above ran on
    // the blocking methods, which poll their own `HCINT` and would race
    // the handler for it.
    lic.enable_usb_irq();
    irq::enable_irq();

    // Everything below outlives `run`, which never returns.
    let timer: &'static Timer = unsafe { &*(timer as *const Timer) };

    let mut state = Lan9514State::<RX_QUEUE, TX_QUEUE>::new();
    let state = unsafe { make_static(&mut state) };
    let (driver, lan9514_runner) =
        rpi_hal_embassy::lan9514::new(state, lan9514, rx_channel, tx_channel, timer, mac);

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
        spawner.spawn(lan9514_task(lan9514_runner).unwrap());
        spawner.spawn(echo_task(stack, uart).unwrap());
    });
}

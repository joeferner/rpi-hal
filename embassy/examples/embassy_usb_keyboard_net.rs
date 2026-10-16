#![no_std]
#![no_main]

// A USB keyboard and a TCP/IP stack running at the same time, over the
// same USB controller.
//
// `embassy_usb_keyboard.rs` reads a keyboard and `embassy_net_echo.rs`
// serves TCP; this runs both at once, which is a different problem. Both
// devices are behind the board's on-board LAN9514 — the keyboard in a
// physical port, the Ethernet function on an internal one — so every
// frame and every key report crosses the same DWC2 host controller.
//
// What makes that possible is that a host channel is owned, not shared.
// `Dwc2Host::alloc_channel` hands each endpoint its own: two for the
// keyboard path (the hub's status endpoint and the keyboard's reports),
// and three for the LAN9514 — one for the control transfers that bring it
// up, then one per direction so a parked receive never has to be
// cancelled to send. The controller arbitrates between them in hardware,
// so neither side has to know the other exists, and running out of
// channels is an error rather than a silent queue.
//
// The two halves reach the controller through different stacks, and this
// example is where that gets tested:
//
// - The **keyboard** goes through `embassy-usb`'s host stack, whose transfers are
//   interrupt-driven and await.
// - The **Ethernet** goes through this crate's `lan9514` adapter: a
//   `Driver` of packet queues for `embassy-net`, and a runner task moving
//   frames with rpi-hal's `_async` methods.
//
// Both therefore park on `usb::dwc2::on_irq` and neither holds the
// executor while the bus works — which is what lets a keystroke be
// serviced during a frame transfer and vice versa. Nothing polls: the
// receive waits on the bulk endpoint rather than asking a thousand times
// a second whether a frame has arrived.
//
// Enumeration has a single owner: `embassy_usb::host` walks the bus, and
// the LAN9514 is built from the address and route it reports
// (`Lan9514::from_endpoint`) rather than being enumerated a second time.
// Two enumerators on one bus would hand out conflicting addresses.
//
// Expect to type into the console and, at the same time, `ping` the
// board and `nc <address> 7`.

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config, StackResources};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_usb::class::hid::host::kbd::{KbdEvent, KbdHandler};
use embassy_usb::class::hub::host::{HubEvent, HubHandler};
use embassy_usb::host::handler::{BusRoute, EnumerationInfo, HandlerEvent, RegisterError};
use embassy_usb::host::{BusController, BusHandle, BusState};
use embassy_usb_driver::Speed;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::rng::Rng;
use rpi_hal::usb::dwc2::{ControlEndpoint, Dwc2Host, SplitTarget};
use rpi_hal::usb::hid::keyboard::usage_to_ascii;
use rpi_hal::usb::lan9514::{self, Lan9514};
use rpi_hal::{halt, irq, lic::Lic, pac, timer::Timer, uart::Uart, usb};
use rpi_hal_embassy::lan9514::{EthernetConfig, Lan9514Driver, Lan9514Runner, Lan9514State};
use rpi_hal_embassy::usb::{Dwc2Allocator, Dwc2HostController};
use rpi_hal_embassy::{Executor, time_driver};

/// Downstream ports the hub driver tracks — see
/// `embassy_usb_keyboard.rs`.
const MAX_PORTS: usize = 8;

/// Scratch for a configuration descriptor during enumeration, and also
/// the largest control data stage this controller can carry.
const CONFIG_BUFFER: usize = 256;

/// `KeyStatusUpdate::modifiers` bit for either shift key.
const MODIFIER_SHIFT: u8 = (1 << 1) | (1 << 5);

/// Consecutive hub status-endpoint failures tolerated before the bus is
/// restarted.
const HUB_ERROR_TOLERANCE: u32 = 5;

/// Sockets the stack may hold at once.
const SOCKETS: usize = 2;

/// TCP port the echo server listens on — the Echo Protocol's
/// conventional port (RFC 862).
const ECHO_PORT: u16 = 7;

/// Frames the adapter may hold in each direction — see
/// `embassy_net_echo.rs`, which explains the sizing.
const RX_QUEUE: usize = 4;
/// The outbound counterpart to [`RX_QUEUE`].
const TX_QUEUE: usize = 4;

type Bus = BusHandle<'static, Dwc2Allocator<'static>>;
type Hub = HubHandler<'static, Dwc2Allocator<'static>, MAX_PORTS>;
type Keyboard = KbdHandler<'static, Bus>;
type Console = Mutex<CriticalSectionRawMutex, Uart>;

/// Everything the network half needs, parked until the LAN9514 turns up
/// during enumeration.
///
/// Built in `kmain` rather than in the task that uses it because all of
/// it has to be `'static`, and `kmain` is where a borrow can be widened
/// soundly — it never returns.
struct NetKit {
    state: &'static mut Lan9514State<RX_QUEUE, TX_QUEUE>,
    resources: &'static mut StackResources<SOCKETS>,
    mac: [u8; 6],
    seed: u64,
}

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

/// Services the interrupts this example depends on — the System Timer
/// for `embassy-time`, and the USB controller for every async transfer.
/// Omitting either wedges the whole program; see
/// `embassy_usb_keyboard.rs`.
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

#[embassy_executor::task]
async fn kbd_task(mut keyboard: Keyboard, console: &'static Console) {
    loop {
        match keyboard.wait_for_event().await {
            Ok(HandlerEvent::HandlerEvent(KbdEvent::KeyStatusUpdate(update))) => {
                let shift = update.modifiers & MODIFIER_SHIFT != 0;
                let mut console = console.lock().await;
                for usage in update.keypress.into_iter().flatten() {
                    match usage_to_ascii(usage.get(), shift) {
                        Some(c) => {
                            let _ = write!(console, "{c}");
                        }
                        None => {
                            let _ = write!(console, "{{{}}}", usage.get());
                        }
                    }
                }
            }
            Ok(HandlerEvent::NoChange) => {}
            Ok(HandlerEvent::HandlerDisconnected) => {
                let _ = writeln!(console.lock().await, "\nkeyboard disconnected");
                return;
            }
            Err(e) => {
                let _ = writeln!(console.lock().await, "\nkeyboard error: {e:?}");
                return;
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Lan9514Driver<'static>>) -> ! {
    runner.run().await
}

/// Moves frames between the chip and the stack's queues.
///
/// What used to be a ticker calling `lan9514::wake_rx` a thousand times a
/// second, mostly to find nothing: the receive now parks on the bulk
/// endpoint and wakes on the USB interrupt this example already
/// dispatches for the keyboard.
#[embassy_executor::task]
async fn lan9514_task(runner: Lan9514Runner<'static, 'static>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn echo_task(stack: embassy_net::Stack<'static>, console: &'static Console) {
    let _ = writeln!(console.lock().await, "waiting for DHCP...");
    stack.wait_config_up().await;

    if let Some(config) = stack.config_v4() {
        let _ = writeln!(console.lock().await, "DHCP: {}", config.address);
    }
    let _ = writeln!(
        console.lock().await,
        "echo server on port {ECHO_PORT} -- keyboard still live"
    );

    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 1024];
    let mut buf = [0u8; 256];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        if socket.accept(ECHO_PORT).await.is_err() {
            continue;
        }
        let _ = writeln!(console.lock().await, "\n[net] connected");

        loop {
            match socket.read(&mut buf).await {
                Ok(0) | Err(_) => break,
                Ok(n) => {
                    // Reported so a silent `nc` can be told apart from a
                    // `nc` nobody typed into: this line appearing means
                    // the bytes reached the board, and the only thing
                    // left in question is the way back.
                    let _ = writeln!(console.lock().await, "\n[net] echoing {n} bytes");
                    if socket.write(&buf[..n]).await.is_err() {
                        break;
                    }
                    // Without this the bytes sit in the send buffer
                    // until something else prompts the stack to flush
                    // them — which, on an idle echo connection, may be
                    // the next thing the peer sends. That looks exactly
                    // like an echo that never arrives.
                    if socket.flush().await.is_err() {
                        break;
                    }
                }
            }
        }
        socket.abort();
        let _ = writeln!(console.lock().await, "\n[net] closed");
    }
}

#[embassy_executor::task]
async fn hub_task(
    mut controller: BusController<'static, Dwc2HostController<'static>>,
    bus: Bus,
    console: &'static Console,
    spawner: Spawner,
    host: &'static Dwc2Host,
    timer: &'static Timer,
    mut net: Option<NetKit>,
) -> ! {
    let mut config = [0u8; CONFIG_BUFFER];

    loop {
        let _ = writeln!(console.lock().await, "waiting for the root port...");
        let speed = controller.wait_for_connection().await;

        let root = match bus.enumerate(BusRoute::Direct(speed), &mut config).await {
            Ok((info, _)) => info,
            Err(e) => {
                let _ = writeln!(console.lock().await, "root enumeration failed: {e:?}");
                continue;
            }
        };

        let mut hub = match HubHandler::<_, MAX_PORTS>::try_register(&bus, &root).await {
            Ok(hub) => hub,
            Err(e) => {
                let _ = writeln!(console.lock().await, "hub registration failed: {e:?}");
                bus.free_address(root.device_address);
                continue;
            }
        };
        let _ = writeln!(console.lock().await, "hub ready");

        let mut errors = 0;
        loop {
            match hub.wait_for_event().await {
                Ok(HandlerEvent::HandlerEvent(event)) => {
                    errors = 0;
                    on_hub_event(
                        &mut hub,
                        &bus,
                        &mut config,
                        console,
                        spawner,
                        host,
                        timer,
                        &mut net,
                        event,
                    )
                    .await;
                }
                Ok(HandlerEvent::NoChange) => errors = 0,
                Ok(HandlerEvent::HandlerDisconnected) => break,
                Err(e) => {
                    errors += 1;
                    let _ = writeln!(console.lock().await, "hub error ({errors}): {e:?}");
                    if errors >= HUB_ERROR_TOLERANCE {
                        break;
                    }
                }
            }
        }
    }
}

/// Prints the last control request a device answered with STALL, and at
/// which stage.
///
/// Worth having here rather than only in `embassy_usb_keyboard.rs`,
/// because this example is where enumeration competes with the network
/// stack for the bus: knowing whether a refusal moved to a different
/// request under that load is the difference between a device quirk and
/// a scheduling problem.
async fn dump_last_stall(console: &'static Console) {
    let setup = rpi_hal_embassy::usb::last_stalled_setup();
    if setup == [0; 8] {
        return;
    }
    let request = match setup[1] {
        0x05 => "SET_ADDRESS",
        0x06 => "GET_DESCRIPTOR",
        0x08 => "GET_CONFIGURATION",
        0x09 => "SET_CONFIGURATION",
        _ => "?",
    };
    let _ = writeln!(
        console.lock().await,
        "  stalled on {request} at the {} stage (wValue=0x{:04x} wLength={})",
        rpi_hal_embassy::usb::last_stalled_stage(),
        u16::from_le_bytes([setup[2], setup[3]]),
        u16::from_le_bytes([setup[6], setup[7]]),
    );
}

/// Enumerates a newly-attached device and hands it to whichever half of
/// this example wants it — the network stack for the LAN9514, a keyboard
/// task for a keyboard, nothing for anything else.
#[allow(clippy::too_many_arguments)]
async fn on_hub_event(
    hub: &mut Hub,
    bus: &Bus,
    config: &mut [u8],
    console: &'static Console,
    spawner: Spawner,
    host: &'static Dwc2Host,
    timer: &'static Timer,
    net: &mut Option<NetKit>,
    event: HubEvent,
) {
    let HubEvent::DeviceDetected { port, speed } = event else {
        return;
    };

    let device = match hub.enumerate_port(config, port, speed).await {
        Ok((info, _)) => info,
        Err(e) => {
            let _ = writeln!(
                console.lock().await,
                "port {port}: enumeration failed: {e:?}"
            );
            dump_last_stall(console).await;
            return;
        }
    };
    let (vid, pid) = (device.device_desc.vendor_id, device.device_desc.product_id);
    let _ = writeln!(
        console.lock().await,
        "port {port}: {vid:04x}:{pid:04x} at address {}",
        device.device_address
    );

    if vid == lan9514::VENDOR_ID && pid == lan9514::PRODUCT_ID {
        match net.take() {
            Some(kit) => start_ethernet(&device, console, spawner, host, timer, kit).await,
            // Already running: the LAN9514 appears once, so a second
            // sighting means the bus restarted under us.
            None => {
                let _ = writeln!(console.lock().await, "port {port}: LAN9514 already running");
            }
        }
        return;
    }

    match KbdHandler::try_register(bus, &device).await {
        Ok(keyboard) => match kbd_task(keyboard, console) {
            Ok(task) => {
                let _ = writeln!(console.lock().await, "port {port}: keyboard -- type away");
                spawner.spawn(task);
            }
            Err(_) => {
                let _ = writeln!(
                    console.lock().await,
                    "port {port}: already driving a keyboard"
                );
            }
        },
        Err(RegisterError::NoSupportedInterface) => {
            let _ = writeln!(console.lock().await, "port {port}: not a keyboard");
        }
        Err(e) => {
            let _ = writeln!(
                console.lock().await,
                "port {port}: keyboard registration failed ({e:?})"
            );
        }
    }
}

/// Brings the LAN9514 up on a channel of its own and starts
/// `embassy-net` over it.
///
/// The device has already been addressed and configured by
/// `embassy_usb::host`, so this reconstructs the `ControlEndpoint` from
/// what enumeration reported rather than enumerating it again —
/// `Lan9514::from_endpoint` exists for exactly that. Two enumerators on
/// one bus would hand out conflicting addresses.
async fn start_ethernet(
    device: &EnumerationInfo,
    console: &'static Console,
    spawner: Spawner,
    host: &'static Dwc2Host,
    timer: &'static Timer,
    kit: NetKit,
) {
    // Three channels, not one: bring-up below is control transfers on the
    // first, and the runner then keeps a parked receive on one bulk
    // endpoint while transmits go out on the other. Sharing one would mean
    // cancelling a receive to send, losing whatever frame the chip was
    // part-way through handing over.
    let (Some(mut channel), Some(rx_channel), Some(tx_channel)) = (
        host.alloc_channel(),
        host.alloc_channel(),
        host.alloc_channel(),
    ) else {
        let _ = writeln!(console.lock().await, "no host channel for the LAN9514");
        return;
    };

    // The route `embassy_usb::host` settled on, translated into what the
    // HAL's transfer primitives want. On this board the Ethernet
    // function is high-speed behind a high-speed hub, so `split` is
    // `None` — nothing between it and the host re-clocks its traffic.
    let endpoint = ControlEndpoint {
        address: device.device_address,
        low_speed: matches!(device.speed(), Speed::Low),
        max_packet_size: u16::from(device.device_desc.max_packet_size0),
        split: device.split().map(|s| SplitTarget {
            hub_address: s.hub_addr(),
            port: s.port(),
        }),
    };

    // Blocking calls, on this channel only. They busy-wait, which is
    // acceptable here — it is bring-up, it happens once, and `on_irq`
    // keeps servicing the other channels throughout because it ignores
    // any channel without an async transfer waiting on it.
    let lan9514 = match Lan9514::from_endpoint(&mut channel, timer, endpoint) {
        Ok(Some(lan9514)) => lan9514,
        Ok(None) => {
            let _ = writeln!(console.lock().await, "LAN9514: no bulk endpoint pair");
            return;
        }
        Err(e) => {
            let _ = writeln!(console.lock().await, "LAN9514 setup failed: {e:?}");
            return;
        }
    };
    // Not started here: the adapter's runner brings the chip up itself, so
    // the bring-up and the awaited frame path agree about how the chip
    // answers an empty receive. The MAC goes in through the config instead
    // of being programmed beforehand.
    let (driver, lan9514_runner) = rpi_hal_embassy::lan9514::new(
        kit.state,
        lan9514,
        rx_channel,
        tx_channel,
        timer,
        EthernetConfig::new(kit.mac),
    );
    let (stack, runner) = embassy_net::new(
        driver,
        Config::dhcpv4(Default::default()),
        kit.resources,
        kit.seed,
    );

    let _ = writeln!(console.lock().await, "LAN9514 up, starting the stack");
    if let (Ok(net), Ok(frames), Ok(echo)) = (
        net_task(runner),
        lan9514_task(lan9514_runner),
        echo_task(stack, console),
    ) {
        spawner.spawn(net);
        spawner.spawn(frames);
        spawner.spawn(echo);
    } else {
        let _ = writeln!(console.lock().await, "network tasks already spawned");
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "USB keyboard + Ethernet, one controller");

    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

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

    let dwc2 = Dwc2Host::init(
        peripherals.USB_OTG_GLOBAL,
        peripherals.USB_OTG_HOST,
        peripherals.USB_OTG_PWRCLK,
        &timer,
    );

    let stolen = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(stolen.LIC);
    time_driver::init(Timer::new(stolen.SYSTMR), &lic);
    lic.enable_usb_irq();
    irq::enable_irq();

    // A random seed keeps TCP initial sequence numbers and the DHCP
    // transaction ID from repeating across boots.
    let mut rng = Rng::new();
    let seed = (u64::from(rng.next_u32()) << 32) | u64::from(rng.next_u32());

    // Everything below outlives `kmain`, which never returns.
    let mut timer = timer;
    let timer: &'static Timer = unsafe { make_static(&mut timer) };
    let mut dwc2 = dwc2;
    let dwc2: &'static Dwc2Host = unsafe { make_static(&mut dwc2) };

    let mut bus_state = BusState::new();
    let bus_state: &'static BusState = unsafe { make_static(&mut bus_state) };

    let mut console = Mutex::new(uart);
    let console: &'static Console = unsafe { make_static(&mut console) };

    let mut state = Lan9514State::<RX_QUEUE, TX_QUEUE>::new();
    let state = unsafe { make_static(&mut state) };

    let mut resources = StackResources::<SOCKETS>::new();
    let resources = unsafe { make_static(&mut resources) };

    let (controller, bus) = embassy_usb::host::bus(Dwc2HostController::new(dwc2, timer), bus_state);

    let mut executor = Executor::new();
    let executor = unsafe { make_static(&mut executor) };

    executor.run(|spawner| {
        spawner.spawn(
            hub_task(
                controller,
                bus,
                console,
                spawner,
                dwc2,
                timer,
                Some(NetKit {
                    state,
                    resources,
                    mac,
                    seed,
                }),
            )
            .unwrap(),
        );
    });
}

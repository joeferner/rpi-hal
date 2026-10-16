#![no_std]
#![no_main]

// Reads a USB keyboard through `embassy-usb`'s host stack, running on
// `rpi-hal-embassy`'s `embassy-usb-driver` host controller.
//
// Where rpi-hal's own `usb_hid_keyboard.rs` walks the bus with its
// blocking enumerator and its own HID driver, everything above the
// controller here comes from `embassy_usb::host`: it does the
// enumeration, drives the on-board hub through `class::hub::host`, and
// decodes reports through `class::hid::host::kbd`. This crate supplies only the
// bottom layer — `Dwc2HostController` and the pipes it allocates over
// rpi-hal's DWC2 host channels.
//
// Two tasks, and the split is the point:
//
// - `hub_task` owns the bus. It waits for the root port, enumerates
//   whatever is there (on this board the on-board SMSC LAN9514 hub),
//   registers the hub driver, and then services the hub's status-change
//   endpoint forever — so a keyboard plugged in later is found, and one
//   unplugged is noticed.
// - `kbd_task` polls the keyboard's interrupt endpoint and prints what
//   it gets. It is spawned once a keyboard turns up.
//
// Both are live at the same time, each with its own host channel, which
// is what the channel pool exists for: the hub's status endpoint being
// polled cannot stall a key press, or the other way round. Everything
// they wait on is an interrupt — no part of this busy-waits on a USB
// register.
//
// A keyboard in one of the board's physical ports is a full-speed device
// behind a high-speed hub, so its transfers are split transactions
// through the hub's transaction translator, scheduled against the
// start-of-frame interrupt. That is the most demanding path in the
// driver, and it is the one this example runs by default.

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_usb::class::hid::host::kbd::{KbdEvent, KbdHandler};
use embassy_usb::class::hub::host::{HubEvent, HubHandler};
use embassy_usb::host::handler::{BusRoute, HandlerEvent, RegisterError};
use embassy_usb::host::{BusController, BusHandle, BusState};
use rpi_hal::mailbox::Mailbox;
use rpi_hal::usb::descriptor::{Descriptors, InterfaceDescriptor};
use rpi_hal::usb::dwc2::Dwc2Host;
use rpi_hal::usb::hid::keyboard::usage_to_ascii;
use rpi_hal::{halt, irq, lic::Lic, pac, timer::Timer, uart::Uart, usb};
use rpi_hal_embassy::usb::{Dwc2Allocator, Dwc2HostController};
use rpi_hal_embassy::{Executor, time_driver};

/// Downstream ports the hub driver tracks. The on-board LAN9514 has
/// five (four sockets plus the internal one its Ethernet function sits
/// on); eight leaves room for a different board without costing more
/// than a few bytes of lookup table.
const MAX_PORTS: usize = 8;

/// Scratch for a configuration descriptor during enumeration. 256 bytes
/// is also the largest control data stage this controller can carry —
/// see `rpi_hal_embassy::usb`'s documented limits — so asking for more
/// here would only move the failure.
const CONFIG_BUFFER: usize = 256;

/// `KeyStatusUpdate::modifiers` bit for either shift key (left is bit 1,
/// right is bit 5), which is all the decoding below needs.
const MODIFIER_SHIFT: u8 = (1 << 1) | (1 << 5);

/// Consecutive failures of the hub's status-change endpoint before the
/// hub is treated as gone and the bus restarted.
const HUB_ERROR_TOLERANCE: u32 = 5;

/// The bus handle's concrete type, spelled once so the task signatures
/// below stay readable. `BusHandle` is itself an allocator (it forwards
/// to the one inside), which is why the class drivers take it directly.
type Bus = BusHandle<'static, Dwc2Allocator<'static>>;

/// The hub driver's concrete type. Its allocator parameter is the plain
/// [`Dwc2Allocator`], because `HubHandler::try_register` takes the
/// `BusHandle` itself (it needs to enumerate through it) and so names
/// the allocator inside.
type Hub = HubHandler<'static, Dwc2Allocator<'static>, MAX_PORTS>;

/// The keyboard driver's concrete type. Unlike the hub it takes only an
/// allocator, and a `BusHandle` is one — which is why the parameter here
/// is [`Bus`] where the hub's is the allocator it wraps.
type Keyboard = KbdHandler<'static, Bus>;

/// The console, shared because both tasks report to it. An async mutex
/// rather than a blocking one: a task waiting for the UART should yield,
/// not spin, and nothing here holds it across a USB transfer.
type Console = Mutex<CriticalSectionRawMutex, Uart>;

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

/// Services the interrupts this example depends on.
///
/// Mandatory, and silently fatal to omit: `rpi-hal` provides only a
/// *weak* no-op `__irq_handler`, so without this the first source to
/// fire is never acknowledged, the interrupt controller keeps asserting,
/// and the core re-enters the handler forever. On the console that looks
/// like a hang immediately after bring-up.
///
/// Two sources here, not one. The System Timer is what `embassy-time`
/// runs on; the USB line is what every transfer in this example waits
/// for. Dropping either wedges the whole program, since both feed the
/// same executor.
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
                    // `usage_to_ascii` comes from rpi-hal's own HID
                    // driver: the usage-to-character table is the same
                    // whichever stack delivered the report, so there is
                    // no reason to write a second one here.
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
            // The report was identical to the last one, or the device
            // went away. Neither is an error; the second ends the task.
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
async fn hub_task(
    mut controller: BusController<'static, Dwc2HostController<'static>>,
    bus: Bus,
    console: &'static Console,
    spawner: Spawner,
    // Only for diagnostics: the number of channels to dump, and the
    // right to read their last interrupt status.
    host: &'static Dwc2Host,
) -> ! {
    let mut config = [0u8; CONFIG_BUFFER];

    loop {
        let _ = writeln!(console.lock().await, "waiting for the root port...");
        let speed = controller.wait_for_connection().await;
        let _ = writeln!(console.lock().await, "root device at {speed:?} speed");

        // The root device is reached directly — there is nothing between
        // it and the host to re-clock its traffic.
        let root = match bus.enumerate(BusRoute::Direct(speed), &mut config).await {
            Ok((info, _)) => info,
            Err(e) => {
                let _ = writeln!(console.lock().await, "root enumeration failed: {e:?}");
                continue;
            }
        };
        let _ = writeln!(
            console.lock().await,
            "root device {:04x}:{:04x} at address {}",
            root.device_desc.vendor_id,
            root.device_desc.product_id,
            root.device_address
        );

        let mut hub = match HubHandler::<_, MAX_PORTS>::try_register(&bus, &root).await {
            Ok(hub) => hub,
            Err(e) => {
                // On this board the root device *is* a hub, so this is a
                // real failure rather than "not the device we wanted" —
                // and the error alone doesn't say which interface was
                // rejected, so dump what the device actually offered.
                let _ = writeln!(console.lock().await, "hub registration failed: {e:?}");
                dump_interfaces(&config, console).await;
                // Hand the address back. Without this every failed
                // attempt burns one, and the loop below walks up through
                // all 127 before enumeration starts failing outright.
                bus.free_address(root.device_address);
                continue;
            }
        };
        let _ = writeln!(console.lock().await, "hub ready -- plug in a keyboard");

        // Service the hub's status-change endpoint until it goes away.
        // Returning to the outer loop then starts over from the root
        // port, which is what a hub being unplugged actually means.
        // Consecutive failures of the hub's status endpoint. A single one
        // is not worth tearing the bus down for — every device already
        // enumerated, the keyboard included, is still working, and
        // restarting from the root port would take them all with it.
        // Persistent failure is different, and means the hub really is
        // gone.
        let mut errors = 0;
        loop {
            match hub.wait_for_event().await {
                Ok(HandlerEvent::HandlerEvent(event)) => {
                    errors = 0;
                    on_hub_event(&mut hub, &bus, &mut config, console, spawner, host, event).await;
                }
                Ok(HandlerEvent::NoChange) => errors = 0,
                Ok(HandlerEvent::HandlerDisconnected) => break,
                Err(e) => {
                    errors += 1;
                    let _ = writeln!(console.lock().await, "hub error ({errors}): {e:?}");
                    // A `PipeError` says which category the failure fell
                    // into, never what the controller reported — and
                    // several very different hardware conditions collapse
                    // into one variant. Dump the raw per-channel `HCINT`
                    // so the difference is visible.
                    dump_channel_interrupts(host, console).await;
                    if errors >= HUB_ERROR_TOLERANCE {
                        let _ = writeln!(console.lock().await, "hub gone -- restarting the bus");
                        break;
                    }
                }
            }
        }
    }
}

/// Prints the raw `HCINT` of the last interrupt-driven halt on every
/// host channel, decoded.
///
/// The error a class driver reports has been through two lossy
/// translations by the time it reaches here — the HAL collapses `HCINT`
/// into a `TransferError`, and this crate maps that onto one of
/// `PipeError`'s eight variants. Several genuinely different hardware
/// conditions arrive as the same word. These are the bits the controller
/// actually set, which is the only account of what happened that hasn't
/// been summarised.
///
/// `XFRC` with nothing else is a completed transfer; `NAK` is ordinary
/// flow control; `TXERR` is a real error on the wire; and `CHH` alone —
/// a channel that stopped with no reason given — is the signature of the
/// controller abandoning a transaction, which is a scheduling problem
/// rather than a device problem.
async fn dump_channel_interrupts(host: &'static Dwc2Host, console: &'static Console) {
    let mut console = console.lock().await;
    for channel in 0..host.num_channels() {
        let hcint = usb::dwc2::asynch::last_interrupt(channel);
        let seen = usb::dwc2::asynch::seen_interrupts(channel);
        if hcint == 0 {
            continue;
        }
        // `seen` carries every bit that ever occurred on this channel,
        // which is what shows a cause that the final halt has already
        // overwritten.
        let _ = write!(
            console,
            "  ch{channel} hcint=0x{hcint:08x} seen=0x{seen:08x}"
        );
        for (bit, name) in [
            (0, "XFRC"),
            (1, "CHH"),
            (3, "STALL"),
            (4, "NAK"),
            (5, "ACK"),
            (6, "NYET"),
            (7, "TXERR"),
            (8, "BBERR"),
            (9, "FRMOR"),
            (10, "DTERR"),
        ] {
            if hcint & (1 << bit) != 0 {
                let _ = write!(console, " {name}");
            } else if seen & (1 << bit) != 0 {
                // Seen at some point, but not on the halt that ended
                // the transfer — parenthesised so the two can't be
                // confused.
                let _ = write!(console, " ({name})");
            }
        }
        let _ = writeln!(console);
    }
}

/// Prints the last control request a device answered with STALL.
///
/// An enumeration failure reports only that *something* was refused, but
/// enumeration issues half a dozen requests and which one was rejected
/// is the whole diagnosis: a device refusing SET_ADDRESS is a very
/// different problem from one refusing an optional descriptor it simply
/// doesn't implement.
async fn dump_last_stall(console: &'static Console) {
    let setup = rpi_hal_embassy::usb::last_stalled_setup();
    if setup == [0; 8] {
        return;
    }
    let request = match setup[1] {
        0x00 => "GET_STATUS",
        0x01 => "CLEAR_FEATURE",
        0x03 => "SET_FEATURE",
        0x05 => "SET_ADDRESS",
        0x06 => "GET_DESCRIPTOR",
        0x08 => "GET_CONFIGURATION",
        0x09 => "SET_CONFIGURATION",
        0x0a => "GET_INTERFACE",
        0x0b => "SET_INTERFACE",
        _ => "?",
    };
    let _ = writeln!(
        console.lock().await,
        "  stalled on {request} at the {} stage (bmRequestType=0x{:02x} bRequest=0x{:02x} \
         wValue=0x{:04x} wIndex=0x{:04x} wLength={})",
        rpi_hal_embassy::usb::last_stalled_stage(),
        setup[0],
        setup[1],
        u16::from_le_bytes([setup[2], setup[3]]),
        u16::from_le_bytes([setup[4], setup[5]]),
        u16::from_le_bytes([setup[6], setup[7]]),
    );
}

/// Prints the class/subclass/protocol of every interface in a
/// configuration descriptor.
///
/// A class driver that declines a device reports only that nothing
/// matched, never what it saw — and the two are very different when a
/// device you *know* is a hub or a keyboard is turned away. The triple
/// is what every class driver matches on, so printing it turns "no
/// supported interface" into a fact you can check against the spec.
async fn dump_interfaces(config: &[u8], console: &'static Console) {
    let mut console = console.lock().await;
    for descriptor in Descriptors::new(config) {
        if let Some(interface) = InterfaceDescriptor::parse(descriptor) {
            let _ = writeln!(
                console,
                "  interface {}: class 0x{:02x} subclass 0x{:02x} protocol 0x{:02x}",
                interface.number(),
                interface.class(),
                interface.subclass(),
                interface.protocol()
            );
        }
    }
}

/// Enumerates a newly-attached downstream device and, if it is a
/// keyboard, hands it to its own task.
///
/// A device that isn't a keyboard is enumerated and then left alone —
/// addressed and configured, but with nothing driving it. That includes
/// the hub's own Ethernet function, which on this board sits on an
/// internal port and so shows up here like anything else.
async fn on_hub_event(
    hub: &mut Hub,
    bus: &Bus,
    config: &mut [u8],
    console: &'static Console,
    spawner: Spawner,
    host: &'static Dwc2Host,
    event: HubEvent,
) {
    let HubEvent::DeviceDetected { port, speed } = event else {
        if let HubEvent::DeviceRemoved { port, .. } = event {
            let _ = writeln!(console.lock().await, "port {port}: device removed");
        }
        return;
    };

    let device = match hub.enumerate_port(config, port, speed).await {
        Ok((info, _)) => info,
        Err(e) => {
            // The speed belongs on the failure path, not just the success
            // one: it decides whether the device is addressed directly or
            // through the hub's transaction translator, and getting that
            // wrong surfaces as a transaction error that names nothing
            // useful. Note this is the speed *before* the port reset — the
            // hub driver re-reads it afterwards, which is the whole point.
            let _ = writeln!(
                console.lock().await,
                "port {port}: enumeration failed (detected at {speed:?} speed): {e:?}"
            );
            dump_channel_interrupts(host, console).await;
            dump_last_stall(console).await;
            return;
        }
    };
    let _ = writeln!(
        console.lock().await,
        "port {port}: {:04x}:{:04x} at {speed:?} speed, address {}",
        device.device_desc.vendor_id,
        device.device_desc.product_id,
        device.device_address
    );

    match KbdHandler::try_register(bus, &device).await {
        Ok(keyboard) => match kbd_task(keyboard, console) {
            Ok(task) => {
                let _ = writeln!(console.lock().await, "port {port}: keyboard -- type away");
                spawner.spawn(task);
            }
            // One keyboard at a time: the task pool holds one, and
            // saying so beats a silently ignored second keyboard.
            Err(_) => {
                let _ = writeln!(
                    console.lock().await,
                    "port {port}: already driving a keyboard, ignoring this one"
                );
            }
        },
        // Not a boot-protocol keyboard. Expected for most of what is on
        // this bus, so it is reported at all only to make a keyboard
        // that *should* have matched debuggable.
        // `NoSupportedInterface` is the ordinary answer — most of what is
        // on this bus is not a keyboard. Anything else means registration
        // failed before it got as far as looking at an interface, so say
        // what the device refused.
        Err(RegisterError::NoSupportedInterface) => {
            let _ = writeln!(console.lock().await, "port {port}: not a keyboard");
        }
        Err(e) => {
            let _ = writeln!(
                console.lock().await,
                "port {port}: keyboard registration failed ({e:?})"
            );
            dump_last_stall(console).await;
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "embassy-usb host keyboard");

    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // Power first: the controller comes up only partially powered from
    // firmware, and until this has happened its registers respond
    // normally while no transaction ever runs.
    if !usb::power_on(&mut mailbox) {
        let _ = writeln!(uart, "USB power-on failed");
        halt();
    }

    let dwc2 = Dwc2Host::init(
        peripherals.USB_OTG_GLOBAL,
        peripherals.USB_OTG_HOST,
        peripherals.USB_OTG_PWRCLK,
        &timer,
    );

    // The time driver needs the System Timer, which `timer` above
    // already owns; steal a second handle rather than restructure the
    // bring-up around it. Both refer to the same free-running counter,
    // and only the driver touches Compare 1.
    let stolen = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(stolen.LIC);
    time_driver::init(Timer::new(stolen.SYSTMR), &lic);
    lic.enable_usb_irq();
    irq::enable_irq();

    // Everything below outlives `kmain`, which never returns.
    let mut timer = timer;
    let timer: &'static Timer = unsafe { make_static(&mut timer) };
    let mut dwc2 = dwc2;
    let dwc2: &'static Dwc2Host = unsafe { make_static(&mut dwc2) };

    let mut bus_state = BusState::new();
    let bus_state: &'static BusState = unsafe { make_static(&mut bus_state) };

    let mut console = Mutex::new(uart);
    let console: &'static Console = unsafe { make_static(&mut console) };

    let (controller, bus) = embassy_usb::host::bus(Dwc2HostController::new(dwc2, timer), bus_state);

    let mut executor = Executor::new();
    let executor = unsafe { make_static(&mut executor) };

    executor.run(|spawner| {
        spawner.spawn(hub_task(controller, bus, console, spawner, dwc2).unwrap());
    });
}

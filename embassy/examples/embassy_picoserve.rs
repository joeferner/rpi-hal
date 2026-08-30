#![no_std]
#![no_main]

// An HTTP server on the on-board Ethernet, using `picoserve` over
// `embassy-net`.
//
// This is `embassy_net_echo` with the hand-written socket loop replaced by
// a router. The bring-up is identical — USB power, DWC2, enumerate the
// LAN9514, DHCP — and everything above the socket is picoserve's:
// request parsing, method and path routing, content types, and the
// per-connection timeouts that a hand-rolled server has to remember to
// implement.
//
// Two routes, chosen to show both halves of the response story:
//
//   GET /        a static page, served with a real `text/html` content
//                type via `response::File::html`
//   GET /uptime  a body built at request time into a `heapless::String`
//
// Two server tasks run concurrently. `listen_and_serve` handles one
// connection at a time by design, so the pool size is how many requests
// can be in flight at once — with only one, a browser opening a second
// connection waits for the first to finish.
//
// No extra wiring beyond an Ethernet cable. Point a browser at the
// address it prints, or `curl http://<address>/uptime`.

use core::fmt::Write as _;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_net::{Config, StackResources};
use embassy_time::{Duration, Instant, Ticker};
use picoserve::response::File;
use picoserve::routing::{get, get_service};
use rpi_hal::mailbox::Mailbox;
use rpi_hal::rng::Rng;
use rpi_hal::usb::dwc2::{Channel, Dwc2Host};
use rpi_hal::usb::lan9514::Lan9514;
use rpi_hal::{halt, irq, lic::Lic, pac, timer::Timer, uart::Uart, usb};
use rpi_hal_embassy::lan9514::{Lan9514Driver, Lan9514Runner, Lan9514State};
use rpi_hal_embassy::{Executor, time_driver};

/// Frames the adapter may hold queued in each direction, inbound and
/// outbound. Each costs one MTU of RAM.
const RX_QUEUE: usize = 4;
/// The outbound counterpart to [`RX_QUEUE`].
const TX_QUEUE: usize = 4;

/// Concurrent connections the server can hold, one per task.
const WEB_TASKS: usize = 2;

/// Sockets the stack may hold at once: one per server task, plus DHCP,
/// plus margin.
const SOCKETS: usize = 4;

/// Requests served, so the console shows the server working without
/// anyone watching a browser.
static REQUESTS: AtomicU32 = AtomicU32::new(0);

/// The index page. Static, so it can be served straight from flash with
/// the correct content type and no formatting at request time.
const INDEX_HTML: &str = "<!DOCTYPE html>\
<html><head><title>rpi-hal</title></head><body>\
<h1>rpi-hal + embassy-net + picoserve</h1>\
<p>Served from a Raspberry Pi running bare metal.</p>\
<p><a href=\"/uptime\">/uptime</a></p>\
</body></html>";

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// Widens a borrow to `'static`. Sound only where the caller never
/// returns, which holds for every use below.
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

#[embassy_executor::task(pool_size = WEB_TASKS)]
async fn web_task(id: usize, stack: embassy_net::Stack<'static>) -> ! {
    stack.wait_config_up().await;

    // Built per task instead of shared behind a `&'static`: the router is
    // a handful of closures, and keeping it local means no lifetime
    // plumbing and no `AppBuilder`/TAIT dance just to name its type.
    let app = picoserve::Router::new()
        // `File` is a *service*, not a value a handler returns: it
        // implements `RequestHandlerService`, so it goes through
        // `get_service` rather than `get`. That is what carries the
        // `text/html` content type with it.
        .route("/", get_service(File::html(INDEX_HTML)))
        .route(
            "/uptime",
            get(async || {
                REQUESTS.fetch_add(1, Ordering::Relaxed);
                let mut body = heapless::String::<96>::new();
                let _ = writeln!(
                    body,
                    "up {}s, {} requests served",
                    Instant::now().as_secs(),
                    REQUESTS.load(Ordering::Relaxed)
                );
                body
            }),
        );

    // The defaults are deliberate values, not placeholders: they bound how
    // long a half-open or stalled connection can occupy this task, which
    // with a pool this small is the difference between one rude client and
    // a server that stops answering.
    let config = picoserve::Config::new(picoserve::Timeouts::const_default());

    let mut http_buffer = [0u8; 1024];
    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 1024];

    // Never returns: with no shutdown signal configured, the reason type
    // is `NoGracefulShutdown`, an uninhabited enum — so matching it with
    // no arms is how the compiler is told this is unreachable, and what
    // lets the task be `-> !`.
    match picoserve::Server::new(&app, &config, &mut http_buffer)
        .listen_and_serve(id, stack, 80, &mut rx_buffer, &mut tx_buffer)
        .await {}
}

#[embassy_executor::task]
async fn report_task(stack: embassy_net::Stack<'static>, mut uart: Uart) {
    let _ = writeln!(uart, "waiting for DHCP...");
    stack.wait_config_up().await;

    if let Some(config) = stack.config_v4() {
        let _ = writeln!(uart, "serving on http://{}/", config.address.address());
    }

    let mut ticker = Ticker::every(Duration::from_secs(10));
    let mut last = 0;
    loop {
        ticker.next().await;
        let served = REQUESTS.load(Ordering::Relaxed);
        if served != last {
            let _ = writeln!(uart, "{served} requests served");
            last = served;
        }
    }
}

// `unsafe(no_mangle)` rather than the bare `no_mangle` rpi-hal's own
// examples use: this crate is edition 2024, which requires the wrapper.
#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "picoserve over embassy-net");

    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // Power first, then read the MAC — nothing on the bus responds until
    // the controller is fully powered.
    if !usb::power_on(&mut mailbox) {
        let _ = writeln!(uart, "USB power-on failed");
        halt();
    }

    let mac = match mailbox.mac_address() {
        Ok(mac) => mac,
        Err(e) => {
            let _ = writeln!(uart, "MAC read failed: {e:?}");
            halt();
        }
    };

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

    let _ = writeln!(uart, "waiting for the on-board hub...");
    let mut waited_ms = 0;
    while !dwc2.port_connected() {
        timer.delay_ms(100);
        waited_ms += 100;
        if waited_ms >= 10_000 {
            let _ = writeln!(uart, "root port never reported a device");
            halt();
        }
    }

    let mut uart = Some(uart);
    let result = usb::enumerate(dwc2, &timer, |channel, timer, device| {
        match Lan9514::from_device(channel, timer, device) {
            Ok(Some(lan9514)) => {
                // The stack needs two channels of its own, one per
                // direction: `channel` belongs to enumeration and is gone
                // once this callback returns, while the runner keeps
                // moving frames forever.
                let (Some(rx_channel), Some(tx_channel)) =
                    (dwc2.alloc_channel(), dwc2.alloc_channel())
                else {
                    let _ = writeln!(
                        uart.as_mut().unwrap(),
                        "no free host channels for the stack"
                    );
                    return core::ops::ControlFlow::Break(());
                };
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

    if let Some(mut uart) = uart {
        let _ = writeln!(uart, "no LAN9514 on the bus (enumerate: {result:?})");
    }
    halt();
}

/// Services the interrupts the executor and the Ethernet adapter depend
/// on.
///
/// Mandatory, and silently fatal to omit: `rpi-hal`'s `__irq_handler` is
/// weak and a no-op, so without this the first `embassy-time` deadline
/// fires, nothing acknowledges the Compare 1 match, and the core livelocks
/// re-entering the handler. The USB line is what completes the adapter's
/// transfers; leaving that half out is quieter still — bring-up prints
/// normally and then no frame ever arrives.
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

/// Brings the chip up and starts the stack. Never returns.
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
        spawner.spawn(report_task(stack, uart).unwrap());
        for id in 0..WEB_TASKS {
            spawner.spawn(web_task(id, stack).unwrap());
        }
    });
}

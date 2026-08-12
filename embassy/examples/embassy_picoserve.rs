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
use rpi_hal::usb::dwc2::Dwc2Host;
use rpi_hal::usb::lan9514::{self, Lan9514, Lan9514Driver};
use rpi_hal::{halt, irq, lic::Lic, pac, timer::Timer, uart::Uart, usb};
use rpi_hal_embassy::{Executor, time_driver};

/// How often the application nudges the driver to look for a frame.
/// There is no USB interrupt, so nothing else will.
const RX_POLL_INTERVAL: Duration = Duration::from_millis(1);

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
async fn poll_task() {
    let mut ticker = Ticker::every(RX_POLL_INTERVAL);
    loop {
        ticker.next().await;
        lan9514::wake_rx();
    }
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
    let result = usb::enumerate(
        &mut dwc2,
        &timer,
        |dwc2, timer, device| match Lan9514::from_device(dwc2, timer, device) {
            Ok(Some(lan9514)) => run(uart.take().unwrap(), dwc2, timer, lan9514, mac),
            _ => core::ops::ControlFlow::Continue(()),
        },
    );

    if let Some(mut uart) = uart {
        let _ = writeln!(uart, "no LAN9514 on the bus (enumerate: {result:?})");
    }
    halt();
}

/// Services the interrupts the executor depends on.
///
/// Mandatory, and silently fatal to omit: `rpi-hal`'s `__irq_handler` is
/// weak and a no-op, so without this the first `embassy-time` deadline
/// fires, nothing acknowledges the Compare 1 match, and the core livelocks
/// re-entering the handler.
#[unsafe(no_mangle)]
pub extern "C" fn __irq_handler() {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);

    if lic.is_timer1_pending() {
        time_driver::on_timer_irq();
    }
}

/// Brings the chip up and starts the stack. Never returns.
fn run(
    mut uart: Uart,
    dwc2: &mut Dwc2Host,
    timer: &Timer,
    mut lan9514: Lan9514,
    mac: [u8; 6],
) -> ! {
    if let Err(e) = lan9514.start(dwc2, timer, mac) {
        let _ = writeln!(uart, "LAN9514 start failed: {e:?}");
        halt();
    }

    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);
    time_driver::init(Timer::new(peripherals.SYSTMR), &lic);
    irq::enable_irq();

    // Everything below outlives `run`, which never returns.
    let dwc2: &'static mut Dwc2Host = unsafe { make_static(dwc2) };
    let timer: &'static Timer = unsafe { &*(timer as *const Timer) };

    let driver = Lan9514Driver::new(lan9514, dwc2, timer, mac);

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
        spawner.spawn(poll_task().unwrap());
        spawner.spawn(report_task(stack, uart).unwrap());
        for id in 0..WEB_TASKS {
            spawner.spawn(web_task(id, stack).unwrap());
        }
    });
}

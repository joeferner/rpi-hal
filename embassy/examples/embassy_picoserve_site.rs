#![no_std]
#![no_main]

// Serves a whole site's worth of files at once, to reproduce what a
// browser actually does to this stack.
//
// `embassy_picoserve` shows the router; this one exists to be *measured*.
// A real application built on this crate served a web interface split
// across several files, and loading one page occasionally stalled for
// almost exactly a second on one file out of six. Every attempt to find
// that from underneath — frame counters, burst probes, socket pools — either
// exonerated the layer being measured or turned out to be measuring the
// probe rather than the board. So this reproduces the symptom instead: the
// same file count, the same file sizes, the same server configuration, on
// a board doing nothing else.
//
// # What it serves
//
// Bodies of the same sizes as that application's, because size is what
// decides how many TCP segments a response takes and therefore what the
// acknowledgement traffic looks like coming back:
//
//   GET /                4173 bytes   the document
//   GET /style.css       8276 bytes
//   GET /app.js         11932 bytes   the largest, 9 segments
//   GET /dashboard.js    4701 bytes
//   GET /icon.svg         740 bytes   one segment
//   GET /metrics         4944 bytes   stands in for the scrape endpoint
//   GET /api/v1/session    47 bytes   the two calls the page makes on load
//   GET /api/v1/status   1024 bytes
//
// The contents are filler. Nothing here depends on what the bytes say,
// only on how many of them there are.
//
// # Running it
//
// ```
// for f in / /style.css /app.js /dashboard.js /icon.svg /metrics \
//          /api/v1/session /api/v1/status; do
//   ( curl -s -o /dev/null -w "$f %{http_code} ttfb=%{time_starttransfer} \
//     total=%{time_total} %{size_download}B\n" "http://<address>$f" ) &
// done; wait
// ```
//
// All eight at once, which is what a browser does. Repeat it a few dozen
// times: the failure being hunted is intermittent, showing up on a
// minority of requests, and a single clean run says nothing. What it looks
// like is `ttfb` in the tens of milliseconds and `total` about a second
// larger — the response starting promptly and then stopping mid-body until
// a retransmission timeout expires.
//
// Pi 2/3 only, like every example here that touches the LAN9514.

use core::fmt::Write as _;

use embassy_net::{Config, StackResources};
use embassy_time::{Duration, Ticker};
use picoserve::response::File;
use picoserve::routing::get_service;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::rng::Rng;
use rpi_hal::usb::dwc2::{Channel, Dwc2Host};
use rpi_hal::usb::lan9514::Lan9514;
use rpi_hal::{halt, irq, lic::Lic, pac, timer::Timer, uart::Uart, usb};
use rpi_hal_embassy::lan9514::{Lan9514Driver, Lan9514Runner, Lan9514State};
use rpi_hal_embassy::{Executor, time_driver};

/// Frames the adapter may hold queued inbound.
///
/// Sixty-four rather than the four the other examples use, matching the
/// application this reproduces. It made no measurable difference there and
/// is not expected to here; it is set the same way so that a difference in
/// behaviour cannot be attributed to it.
const RX_QUEUE: usize = 64;

/// The outbound counterpart to [`RX_QUEUE`].
const TX_QUEUE: usize = 64;

/// Concurrent connections the server can hold, one per task.
///
/// Eight, because a browser opens six to one origin and this page is
/// eight requests. `embassy-net` has no listen backlog: a connection
/// arriving when every task is busy matches no socket and smoltcp answers
/// it with a reset, which the browser reports as a failed request rather
/// than retrying. Two was enough for a single-document page and produced
/// exactly that failure once the page became eight files.
const WEB_TASKS: usize = 8;

/// Sockets the stack may hold at once: one per web task, plus DHCP, plus
/// margin.
const SOCKETS: usize = WEB_TASKS + 4;

/// Per-connection TCP receive buffer. Only headers arrive on these.
const TCP_RX_BUFFER_SIZE: usize = 16 * 1024;

/// Per-connection TCP transmit buffer.
///
/// **Sized to hold the largest response whole**, which is what stops a
/// response having to wait mid-body for the client's acknowledgements. At
/// 4 KB — enough for none of the files above — every one of them stopped
/// part-way, and under a concurrent burst that wait crossed `write` below
/// and the connection was aborted mid-response.
const TCP_TX_BUFFER_SIZE: usize = 32 * 1024;

/// Buffer `picoserve` parses the request line and headers in.
const HTTP_BUFFER_SIZE: usize = 8 * 1024;

/// Filler for a response body of a given size.
///
/// One array, sliced to each length, rather than one per file: the bytes
/// are irrelevant and only the count matters, so there is no reason to
/// spend `.rodata` on eight copies of the same thing.
static FILLER: [u8; 12 * 1024] = [b'x'; 12 * 1024];

/// How many routes [`web_task`] registers, for the banner.
///
/// Written down rather than derived: `picoserve`'s router is typed by its
/// routes — every `route` call returns a different type — so the table
/// cannot be a slice the router is built from in a loop, and the count
/// has nowhere else to come from.
const FILE_COUNT: usize = 8;

/// A body of `size` filler bytes with the given content type.
///
/// The bytes are irrelevant and only the count matters, so every route
/// slices one shared array rather than carrying its own copy.
fn file(content_type: &'static str, size: usize) -> File {
    File::with_content_type(content_type, &FILLER[..size])
}

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

    // The sizes are the point of this example — see the module comment.
    // Built per task rather than shared, as in `embassy_picoserve`: the
    // router is cheap and keeping it local avoids naming its type.
    let app = picoserve::Router::new()
        .route("/", get_service(file(File::MIME_HTML, 4173)))
        .route("/style.css", get_service(file(File::MIME_CSS, 8276)))
        .route("/app.js", get_service(file(File::MIME_JS, 11932)))
        .route("/dashboard.js", get_service(file(File::MIME_JS, 4701)))
        .route("/icon.svg", get_service(file("image/svg+xml", 740)))
        .route(
            "/metrics",
            get_service(file("text/plain; charset=utf-8", 4944)),
        )
        .route("/api/v1/session", get_service(file("application/json", 47)))
        .route(
            "/api/v1/status",
            get_service(file("application/json", 1024)),
        );

    // The application's timeouts, not `picoserve`'s defaults, because one
    // of those defaults is the fault this reproduction has to avoid
    // re-introducing: `write` at one second is not a service target but
    // the point at which a response is *aborted* mid-body, and a
    // concurrent burst of these files crosses it.
    let config = picoserve::Config::new(picoserve::Timeouts {
        start_read_request: Duration::from_secs(5),
        // Long enough that a browser's next request lands inside the
        // window it believes the connection is still good for. Shorter
        // hands the socket back sooner but closes connections the client
        // is about to reuse, and a request sent onto one already dropped
        // is answered with a reset rather than a refusal.
        persistent_start_read_request: Duration::from_secs(1),
        read_request: Duration::from_secs(3),
        write: Duration::from_secs(10),
    })
    // Off by default in `picoserve`. One page here is eight requests, and
    // closing after each makes that eight connections against a pool of
    // [`WEB_TASKS`], each ending in a shutdown that waits for the client's
    // own close before the socket frees.
    .keep_connection_alive();

    let mut http_buffer = [0u8; HTTP_BUFFER_SIZE];
    let mut rx_buffer = [0u8; TCP_RX_BUFFER_SIZE];
    let mut tx_buffer = [0u8; TCP_TX_BUFFER_SIZE];

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
        let _ = writeln!(uart, "{} files, {} web tasks", FILE_COUNT, WEB_TASKS);
    }

    // The adapter's counters, printed whenever they move. Requests are not
    // counted: every route here is a static file service with no handler
    // to hook, and adding a wrapper purely to increment a number would be
    // work in the request path this example exists to measure.
    //
    // These are the numbers worth having beside a stall anyway. If one
    // ever coincides with a frame going missing on the way in or a send
    // being dropped, this is where it shows.
    let mut ticker = Ticker::every(Duration::from_secs(10));
    let mut last = 0;
    loop {
        ticker.next().await;
        let rx = rpi_hal_embassy::lan9514::rx_stats();
        let tx = rpi_hal_embassy::lan9514::tx_stats();
        if rx.frames != last {
            let _ = writeln!(
                uart,
                "rx {} frames, {} unusable | tx {} frames, {} dropped | \
                 no-transfer gap: max {} us, {} over 100us",
                rx.frames, rx.unusable, tx.frames, tx.failures, rx.gap_max_us, rx.gaps_long
            );
            last = rx.frames;
        }
    }
}

// `unsafe(no_mangle)` rather than the bare `no_mangle` rpi-hal's own
// examples use: this crate is edition 2024, which requires the wrapper.
#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "a whole site over picoserve, for measuring");

    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

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
    let _ = writeln!(uart, "hub detected after {waited_ms}ms");

    let mut uart = Some(uart);
    let result = usb::enumerate(
        dwc2,
        &timer,
        |channel, timer, device| match Lan9514::from_device(channel, timer, device) {
            Ok(Some(lan9514)) => {
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
        },
    );

    if let Some(mut uart) = uart {
        let _ = writeln!(uart, "no LAN9514 on the bus (enumerate: {result:?})");
    }
    halt();
}

/// Services the interrupts the executor and the Ethernet adapter depend
/// on. Mandatory, and silently fatal to omit — see `embassy_net_echo`.
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
    if let Err(e) = lan9514.start(&mut rx_channel, timer, mac) {
        let _ = writeln!(uart, "LAN9514 start failed: {e:?}");
        halt();
    }

    // Wait for auto-negotiation, then program the MAC's duplex to match
    // what it settled on. `start` assumes full duplex because it runs
    // before the link is up and cannot know; this is where it is made
    // true. A mismatch here is frames discarded inside the MAC whenever
    // traffic runs both ways at once, reported by nothing — see
    // `Lan9514::set_duplex`.
    let _ = writeln!(uart, "waiting for link...");
    loop {
        match lan9514.is_link_up(&mut rx_channel, timer) {
            Ok(true) => break,
            Ok(false) => timer.delay_ms(100),
            Err(e) => {
                let _ = writeln!(uart, "link check failed: {e:?}");
                halt();
            }
        }
    }
    let full = match lan9514.is_full_duplex(&mut rx_channel, timer) {
        Ok(full) => full,
        Err(e) => {
            let _ = writeln!(uart, "duplex read failed: {e:?}");
            halt();
        }
    };
    if let Err(e) = lan9514.set_duplex(&mut rx_channel, timer, full) {
        let _ = writeln!(uart, "duplex set failed: {e:?}");
        halt();
    }
    let _ = writeln!(
        uart,
        "link up, {} duplex",
        if full { "full" } else { "half" }
    );

    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);
    time_driver::init(Timer::new(peripherals.SYSTMR), &lic);

    lic.enable_usb_irq();
    irq::enable_irq();

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

//! Over-the-air updates: the upload route, and the reboot after it.
//!
//! A bundle — the format and the install are `rpi-loader-ota`'s, which the
//! tool that packs one links too — is `POST`ed to a route this serves:
//!
//! ```ignore
//! static REBOOT: ota::Reboot = ota::Reboot::new();
//!
//! let app = Router::from_service(web::SiteFiles(site))
//!     .route("/api/v1/ota", post_service(ota::OtaUpload::new(&CARD, &FORMAT, &REBOOT)));
//!
//! #[embassy_executor::task]
//! async fn reboot_task() -> ! {
//!     REBOOT.wait().await;
//!     rpi_hal::power::reboot()
//! }
//! ```
//!
//! The upload is read whole, installed while the connection is still open,
//! and answered with what the install did — so `ok` in the response means
//! **installed**, not accepted, and the figures in it are what the card now
//! holds. Then the board reboots into it, once the answer has had time to
//! leave.
//!
//! # The response
//!
//! One shape for every board, so the tools that send bundles — the
//! `rpi-loader` CLI, a board's own tools page — read any of them:
//!
//! ```json
//! { "ok": true, "kernel": 2293760, "written": 3, "skipped": 18,
//!   "elapsed_ms": 970, "rebooting_in_seconds": 1,
//!   "kernel_timing": { "write_ms": 612, "verify_ms": 140,
//!                      "write_kib_s": 3659, "verify_kib_s": 16000 } }
//! { "ok": false, "error": "upload incomplete" }
//! ```
//!
//! A field with nothing to say is left out rather than sent as `null`:
//! `kernel` and `kernel_timing` when the bundle carried no kernel or the
//! card already held it, and the timing's command counts unless the board
//! counts its card's commands (see
//! [`OtaUpload::counting`](crate::ota::OtaUpload::counting)).
//!
//! # What stays the board's
//!
//! The route's path, the bundle's `rpi_loader_ota::Format` — whose magic is what stops
//! another board's update being installed — the size it accepts, and the
//! reboot itself, which is the board's task. And through
//! [`Hooks`](crate::ota::Hooks),
//! anything it wants done around the transfer or the install: measuring
//! the radio while the bundle arrives, holding off a watchdog while the
//! card is written, tidying a file after.

use alloc::string::String;
use alloc::vec;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use picoserve::ResponseSent;
use picoserve::io::Read;
use picoserve::request::Request;
use picoserve::response::{IntoResponse, Json, ResponseWriter, StatusCode};
use picoserve::routing::RequestHandlerService;
use resident_fat::BlockDevice;
// Ungated, though `resident-fat` builds it only where there are atomics:
// every core a Pi has, and every host this crate's tests run on, has them.
use resident_fat::counted::Counters;
use rpi_loader_ota::Format;
use rpi_loader_ota::apply::{self, Report};
use rpi_loader_ota::measure::{Measure, Phase, rate_kib_s};

use crate::logln;
use crate::storage::{Shared, Volume};

/// How long [`Reboot::wait`] lets the answer to an installed update drain
/// before it returns.
///
/// Long enough for the response to have left, since the reboot takes the
/// network with it: a second is around a thousand passes of a frame runner
/// for a response of a few hundred bytes. Nothing can do better by
/// watching, because the handler is gone by the time the bytes are on the
/// wire and the stack will not say whether the far end acknowledged them.
pub const DRAIN: Duration = Duration::from_secs(1);

/// The largest bundle accepted unless a board says otherwise — see
/// [`OtaUpload::max_bundle`].
///
/// The whole bundle is held in memory while it is written, so this is what
/// stops an upload exhausting the heap. Sixteen mebibytes against a bundle
/// of a few: the thing that grows is Raspberry Pi's firmware, when a
/// pinned release is bumped, rather than anything a board writes.
pub const MAX_BUNDLE: usize = 16 * 1024 * 1024;

/// The handshake between an installed update and the reboot into it.
///
/// The reboot cannot happen in the handler: the response is owed to
/// whoever sent the bundle, and it has not left the board when the handler
/// returns. So the handler signals, and a task of the board's — which is
/// where the reboot, and anything to do just before it, belong — waits.
pub struct Reboot {
    installed: Signal<CriticalSectionRawMutex, Report>,
}

impl Reboot {
    /// Not yet requested. `const`, for a `static`.
    pub const fn new() -> Self {
        Reboot {
            installed: Signal::new(),
        }
    }

    /// Asks for the reboot, carrying what was installed.
    pub fn request(&self, installed: Report) {
        self.installed.signal(installed);
    }

    /// Waits for an installed update, then for its answer to drain, and
    /// returns what it installed — for a board that announces an update
    /// before rebooting into it. The board reboots after this returns.
    pub async fn wait(&self) -> Report {
        let installed = self.installed.wait().await;
        Timer::after(DRAIN).await;
        logln!("ota: rebooting into the new image");
        installed
    }
}

impl Default for Reboot {
    fn default() -> Self {
        Reboot::new()
    }
}

/// What a board does around an upload, beyond the upload itself. `()` is
/// the board with nothing to add.
///
/// [`install`](Self::install) has a default that only runs the install. The
/// two around the transfer do not, since stable Rust cannot default
/// [`Receiving`](Self::Receiving): a board with nothing to measure writes
/// them as empty `async fn`s with `type Receiving = ()`.
// `async fn` without a `Send` bound, as picoserve's own service traits are:
// the route runs on a single-core executor, and nothing here moves a future
// between threads.
#[allow(async_fn_in_trait)]
pub trait Hooks<D: BlockDevice> {
    /// What [`receiving`](Self::receiving) hands
    /// [`received`](Self::received).
    type Receiving;

    /// Before the bundle is read — to take a reading to compare against
    /// once it has arrived.
    async fn receiving(&self) -> Self::Receiving;

    /// After the bundle was read, whether or not all of it arrived, with
    /// what [`receiving`](Self::receiving) returned.
    async fn received(&self, before: Self::Receiving);

    /// Runs `install` on the volume, which writes the bundle to it. Wraps
    /// the install, so it can hold something for its length — a watchdog
    /// held off, say — or touch the volume after it.
    fn install<R>(&self, volume: &mut Volume<D>, install: impl FnOnce(&mut Volume<D>) -> R) -> R {
        install(volume)
    }
}

impl<D: BlockDevice> Hooks<D> for () {
    type Receiving = ();

    async fn receiving(&self) {}

    async fn received(&self, _before: ()) {}
}

/// `POST` a bundle here: it is read, installed onto the card, and answered
/// with what the install did, and then [`Reboot`] is requested.
///
/// A service rather than a handler because the body is read in chunks: a
/// bundle is megabytes, and an extractor wants a body small enough to have
/// been buffered already. The request's timeout bounds the whole upload,
/// so a board taking one raises `read_request` in its
/// [`ServeConfig`](crate::web::ServeConfig) to cover a transfer of the
/// largest bundle over its link.
pub struct OtaUpload<'a, D: BlockDevice, H = ()> {
    card: &'a Shared<D>,
    format: &'a Format,
    reboot: &'a Reboot,
    max_bundle: usize,
    counters: Option<&'a Counters>,
    hooks: H,
}

impl<'a, D: BlockDevice> OtaUpload<'a, D> {
    /// Installs bundles in `format` onto `card`, and requests `reboot` once
    /// one is in.
    pub fn new(card: &'a Shared<D>, format: &'a Format, reboot: &'a Reboot) -> Self {
        OtaUpload {
            card,
            format,
            reboot,
            max_bundle: MAX_BUNDLE,
            counters: None,
            hooks: (),
        }
    }
}

impl<'a, D: BlockDevice, H> OtaUpload<'a, D, H> {
    /// The largest bundle this accepts, in bytes; [`MAX_BUNDLE`] otherwise.
    pub fn max_bundle(mut self, bytes: usize) -> Self {
        self.max_bundle = bytes;
        self
    }

    /// Counts the card's commands through `counters` — the ones the card's
    /// `resident_fat::counted::Counted` device counts into — and reports
    /// them with each phase and in the response.
    pub fn counting(mut self, counters: &'a Counters) -> Self {
        self.counters = Some(counters);
        self
    }

    /// Runs `hooks` around the transfer and the install.
    pub fn hooks<H2: Hooks<D>>(self, hooks: H2) -> OtaUpload<'a, D, H2> {
        OtaUpload {
            card: self.card,
            format: self.format,
            reboot: self.reboot,
            max_bundle: self.max_bundle,
            counters: self.counters,
            hooks,
        }
    }

    /// A [`Measure`] over the clock, counting when counters were given.
    fn measure(&self) -> Measure<'a, impl FnMut() -> u64 + use<'a, D, H>> {
        let now = || Instant::now().as_millis();
        match self.counters {
            Some(counters) => Measure::counting(now, counters),
            None => Measure::new(now),
        }
    }
}

impl<D, H, State, PathParameters> RequestHandlerService<State, PathParameters>
    for OtaUpload<'_, D, H>
where
    D: BlockDevice,
    H: Hooks<D>,
{
    async fn call_request_handler_service<R: Read, W: ResponseWriter<Error = R::Error>>(
        &self,
        _state: &State,
        _path_parameters: PathParameters,
        mut request: Request<'_, R>,
        response_writer: W,
    ) -> Result<ResponseSent, W::Error> {
        let content_length = request.body_connection.body().content_length();
        if content_length > self.max_bundle {
            return refuse(
                request,
                response_writer,
                413,
                "bundle is larger than this board accepts",
            )
            .await;
        }

        // On the heap rather than a static with a busy flag: an owned buffer
        // makes two uploads at once a non-issue rather than something to
        // guard against.
        let mut bundle = vec![0u8; content_length];

        let before = self.hooks.receiving().await;
        let started = Instant::now();
        // The two ways a read stops short kept apart: a client that hung up
        // and a read that ran out of the request's time budget give the
        // same short count, and want looking at in different places.
        let mut stopped = "";
        // Scoped, so the reader's borrow of `request` ends and the response
        // below can consume it.
        let received = {
            let mut reader = request.body_connection.body().reader();
            let mut received = 0;
            while received < content_length {
                match reader.read(&mut bundle[received..]).await {
                    // The reader reports 0 only once it has handed over the
                    // whole body, so a short read here is the peer leaving.
                    Ok(0) => {
                        stopped = "the client stopped sending";
                        break;
                    }
                    Err(_) => {
                        stopped = "the read failed or ran out of time";
                        break;
                    }
                    Ok(n) => received += n,
                }
            }
            received
        };
        let elapsed_ms = Instant::now()
            .saturating_duration_since(started)
            .as_millis();
        self.hooks.received(before).await;

        if received != content_length {
            logln!(
                "ota: received {received} of {content_length} bytes in {elapsed_ms} ms -- {stopped}"
            );
            return refuse(request, response_writer, 400, "upload incomplete").await;
        }
        // The rate as well as the count: an update that feels slow is the
        // transfer or the write, and this is the line that tells them apart.
        logln!(
            "ota: received {received} bytes in {elapsed_ms} ms ({} KiB/s)",
            rate_kib_s(received, elapsed_ms)
        );

        let mut measure = self.measure();
        let format = self.format;
        let installed = self
            .card
            .with(|volume| {
                self.hooks.install(volume, |volume| {
                    apply::apply(volume, format, &bundle, &mut measure)
                })
            })
            .await;
        // After the install rather than as it goes -- it blocks the executor
        // for its length, so nothing would print sooner -- and before its
        // result is looked at, so a failure still says how far it got.
        for entry in measure.entries() {
            logln!("ota: {entry}");
        }

        let connection = request.body_connection.finalize().await?;
        let (status, outcome) = match installed {
            None => {
                logln!("ota: refused -- no card mounted");
                (503, Outcome::refused("this board has no card mounted"))
            }
            Some(Err(error)) => {
                logln!("ota: failed -- {error}");
                // Asked rather than assumed: a bundle refused is the
                // sender's problem and a card failing mid-write is not, and
                // one status for both sends whoever is updating to look in
                // the wrong place.
                (
                    error.http_status(),
                    Outcome::refused(&alloc::format!("{error}")),
                )
            }
            Some(Ok(report)) => {
                let elapsed_ms = measure.elapsed_ms();
                logln!(
                    "ota: {} written, {} unchanged in {elapsed_ms} ms",
                    report.written,
                    report.skipped
                );
                // Before the response is written rather than after: the
                // drain covers the write, and nothing here can watch the
                // socket once the handler has returned.
                self.reboot.request(report);
                (
                    200,
                    Outcome::installed(&report, elapsed_ms, measure.kernel()),
                )
            }
        };
        Json(outcome)
            .into_response()
            .with_status_code(StatusCode::new(status))
            .write_to(connection, response_writer)
            .await
    }
}

/// Answers a refused upload, reading what is left of the request first.
async fn refuse<R: Read, W: ResponseWriter<Error = R::Error>>(
    request: Request<'_, R>,
    response_writer: W,
    status: u16,
    reason: &str,
) -> Result<ResponseSent, W::Error> {
    logln!("ota: refused -- {reason}");
    Json(Outcome::refused(reason))
        .into_response()
        .with_status_code(StatusCode::new(status))
        .write_to(request.body_connection.finalize().await?, response_writer)
        .await
}

/// The body of the response, whichever way it went — see the module
/// documentation for its shape.
#[derive(serde::Serialize)]
struct Outcome {
    /// Installed, rather than accepted.
    ok: bool,
    /// The kernel's size, when the bundle carried one.
    #[serde(skip_serializing_if = "Option::is_none")]
    kernel: Option<u32>,
    /// Entries written.
    #[serde(skip_serializing_if = "Option::is_none")]
    written: Option<u32>,
    /// Entries the card already held, so read and not written. Worth its
    /// own field: a bundle carrying the Raspberry Pi firmware carries
    /// megabytes of it, and this says whether an update paid for that.
    #[serde(skip_serializing_if = "Option::is_none")]
    skipped: Option<u32>,
    /// The whole install, in milliseconds.
    #[serde(skip_serializing_if = "Option::is_none")]
    elapsed_ms: Option<u64>,
    /// The kernel's write and read-back, when it was written — the figure
    /// that stays comparable between updates.
    #[serde(skip_serializing_if = "Option::is_none")]
    kernel_timing: Option<KernelTiming>,
    /// How long until the board reboots into it.
    #[serde(skip_serializing_if = "Option::is_none")]
    rebooting_in_seconds: Option<u64>,
    /// Why it was refused or failed.
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<String>,
}

impl Outcome {
    fn refused(reason: &str) -> Self {
        Outcome {
            ok: false,
            kernel: None,
            written: None,
            skipped: None,
            elapsed_ms: None,
            kernel_timing: None,
            rebooting_in_seconds: None,
            error: Some(String::from(reason)),
        }
    }

    fn installed(report: &Report, elapsed_ms: u64, kernel: Option<(Phase, Phase)>) -> Self {
        Outcome {
            ok: true,
            kernel: report.kernel_len,
            written: Some(report.written),
            skipped: Some(report.skipped),
            elapsed_ms: Some(elapsed_ms),
            kernel_timing: kernel.map(|(write, verify)| KernelTiming::of(&write, &verify)),
            rebooting_in_seconds: Some(DRAIN.as_secs()),
            error: None,
        }
    }
}

/// What writing and reading back the kernel cost.
///
/// The commands beside the times, when the board counts them, because the
/// times alone cannot say *why* an update is slow: the same milliseconds at
/// a hundred blocks per command and at one say different things went wrong.
#[derive(serde::Serialize)]
struct KernelTiming {
    write_ms: u64,
    verify_ms: u64,
    write_kib_s: u64,
    verify_kib_s: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    write_calls: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    write_blocks: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    verify_calls: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    verify_blocks: Option<u32>,
}

impl KernelTiming {
    fn of(write: &Phase, verify: &Phase) -> Self {
        let (write_counts, verify_counts) = (write.counts, verify.counts);
        KernelTiming {
            write_ms: write.ms,
            verify_ms: verify.ms,
            write_kib_s: write.rate_kib_s(),
            verify_kib_s: verify.rate_kib_s(),
            write_calls: write_counts.map(|c| c.write_calls),
            write_blocks: write_counts.map(|c| c.write_blocks),
            verify_calls: verify_counts.map(|c| c.read_calls),
            verify_blocks: verify_counts.map(|c| c.read_blocks),
        }
    }
}

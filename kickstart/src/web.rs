//! Serving HTTP: the socket loop, and the pieces a board builds its router
//! from.
//!
//! Not a web task. Routing *is* the application — which API a board has,
//! behind what, and in what order is the whole of what makes one board's
//! page different from another's — so the router is the board's own
//! `picoserve::Router`, and this module is what every board otherwise writes
//! identically around it:
//!
//! ```ignore
//! #[embassy_executor::task(pool_size = 4)]
//! async fn web_task(id: usize, stack: Stack<'static>, site: &'static Site) -> ! {
//!     let app = Router::from_service(web::SiteFiles(site))
//!         .route("/api/v1/status", get(status));
//!     web::serve(id, stack, &app, &web::ServeConfig::DEFAULT).await
//! }
//! ```
//!
//! # A pool of tasks, and no backlog
//!
//! Each task holds one socket for the life of the program, and `embassy-net`
//! has no listen backlog: a SYN arriving while every socket is busy matches
//! nothing, and smoltcp answers it with a reset rather than queueing it. So
//! the pool size is a floor on how many requests can be in flight at once,
//! and it has to cover a page load — which is a burst, not a request: the
//! document, a stylesheet, a script or two, the icon, the first API call. A
//! browser opens several connections in parallel before it knows it could
//! have reused one.
//!
//! The stack's socket table is sized by the board, and has to have room for
//! the whole pool beside every other socket the board opens. Too small is a
//! panic inside `embassy-net`, not a refused connection.
//!
//! # Buffers are per task, and allocated
//!
//! Every task in the pool has its own request buffer and its own TCP
//! buffers, so the memory is
//! [`ServeConfig`](crate::web::ServeConfig)'s three sizes times the pool.
//! They come off the heap, once, when the task starts: they are the board's
//! numbers rather than this crate's — see
//! [`ServeConfig`](crate::web::ServeConfig) — and a size that
//! is a value rather than a constant has nowhere else to live.

use alloc::string::String;
use alloc::vec;

use embassy_net::Stack;
use embassy_time::Duration;
use picoserve::io::Write;
use picoserve::response::Content;
use picoserve::routing::PathRouter;
use picoserve::{Router, Timeouts};

/// How [`serve`] listens: the port, the buffers, and `picoserve`'s own
/// settings.
///
/// A value rather than constants, because boards already disagree and are
/// both right: one serving a large stylesheet needs a transmit buffer that
/// holds it whole, and one taking multi-megabyte uploads needs a request
/// timeout measured in minutes. Start from [`ServeConfig::DEFAULT`] and
/// change what differs.
#[derive(Debug)]
pub struct ServeConfig {
    /// The TCP port. Default 80.
    pub port: u16,
    /// The buffer `picoserve` parses a request's head in, in bytes. Default
    /// 2 KiB.
    ///
    /// Bounds the request line plus headers, not the body — a body is read
    /// through the connection. A browser's headers, cookies included, fit
    /// in this with room to spare.
    pub http_buffer: usize,
    /// Per-connection TCP receive buffer, in bytes: the receive window.
    /// Default 16 KiB.
    ///
    /// Larger does not make an upload faster over a radio link. A sender
    /// achieves the window divided by the round trip, and a larger window
    /// in front of a link with a fixed rate only grows the round trip by
    /// the same factor: the extra is a queue, and everything else on the
    /// link waits behind it. Size it for the headers, and to keep that
    /// queue short.
    pub tcp_rx_buffer: usize,
    /// Per-connection TCP transmit buffer, in bytes. Default 16 KiB.
    ///
    /// **Size it so every response fits whole.** A larger body is written
    /// in instalments, each waiting on the client's acknowledgements while
    /// holding one of the pool — and a wait that outlives the write
    /// timeout makes `picoserve` abort a body the client has already
    /// started reading, which a browser reports as a connection reset on a
    /// file it was halfway through.
    pub tcp_tx_buffer: usize,
    /// `picoserve`'s timeouts and connection handling. Default [`TIMEOUTS`]
    /// with connections kept alive.
    ///
    /// Kept alive because a page is a burst of requests, and closing after
    /// each makes every one a new connection — each ending in a shutdown
    /// that waits on the client before the socket is free again, from a
    /// pool that has no backlog.
    pub server: picoserve::Config,
}

impl ServeConfig {
    /// The defaults, as a constant, so a board can start a `static` from
    /// them.
    pub const DEFAULT: Self = ServeConfig {
        port: 80,
        http_buffer: 2 * 1024,
        tcp_rx_buffer: 16 * 1024,
        tcp_tx_buffer: 16 * 1024,
        server: picoserve::Config::new(TIMEOUTS).keep_connection_alive(),
    };
}

/// Timeouts for a board serving a small site out of RAM.
///
/// The timeouts are the point of a server over a hand-rolled loop: they
/// bound how long a stalled connection can hold one of the pool, which with
/// a pool this small is the difference between one rude client and a board
/// that stops answering.
pub const TIMEOUTS: Timeouts = Timeouts {
    // Waiting for the first request on a fresh connection. A browser sends
    // one immediately; this only bounds a client that connects and then
    // says nothing.
    start_read_request: Duration::from_secs(5),
    // How long a kept-alive connection waits for a further request. Long
    // enough to cover a page load's burst, which a browser spreads over the
    // connections it has; short enough not to hold a socket through the
    // gaps between a page's polls, which would cost a viewer one of the
    // pool for as long as their tab is open.
    persistent_start_read_request: Duration::from_secs(1),
    // **The whole request, not one read of it** — `picoserve` keeps this
    // running while the handler reads the body. Three seconds is right for
    // requests that are all headers and a small form. A board that takes an
    // upload raises it to cover the whole transfer, since otherwise it is
    // this, and not the link, that decides how large an upload may be.
    read_request: Duration::from_secs(3),
    // Writing a response. Ten seconds rather than `picoserve`'s one: this is
    // where it *aborts* a response mid-body, and its job is to catch a
    // client that stopped reading, not one that is merely slow behind a
    // burst of other connections on a shared executor.
    write: Duration::from_secs(10),
};

/// Serves `app` on one socket, forever. Run one per task in the pool.
///
/// Waits for the stack to have an address first, so the pool can be
/// spawned with everything else rather than after DHCP.
///
/// `id` only distinguishes the tasks in `picoserve`'s own logging.
pub async fn serve<P: PathRouter>(
    id: usize,
    stack: Stack<'_>,
    app: &Router<P>,
    config: &ServeConfig,
) -> ! {
    stack.wait_config_up().await;

    let mut http_buffer = vec![0u8; config.http_buffer];
    let mut rx_buffer = vec![0u8; config.tcp_rx_buffer];
    let mut tx_buffer = vec![0u8; config.tcp_tx_buffer];

    // With no shutdown signal the reason type is uninhabited; matching it
    // with no arms is how the compiler is told this is unreachable.
    match picoserve::Server::new(app, &config.server, &mut http_buffer)
        .listen_and_serve(id, stack, config.port, &mut rx_buffer, &mut tx_buffer)
        .await {}
}

/// A body of text built for the response, with its own content type.
///
/// `picoserve` serves an owned `String` already, but as `text/plain;
/// charset=utf-8` — and some formats have a type of their own that a client
/// is entitled to see: Prometheus's exposition format above all.
#[derive(Debug)]
pub struct TextBody {
    /// The text.
    pub body: String,
    /// What to serve it as.
    pub content_type: &'static str,
}

impl Content for TextBody {
    fn content_type(&self) -> &'static str {
        self.content_type
    }

    fn content_length(&self) -> usize {
        self.body.len()
    }

    async fn write_content<W: Write>(self, mut writer: W) -> Result<(), W::Error> {
        writer.write_all(self.body.as_bytes()).await
    }
}

/// A body already in memory, with its own content type.
///
/// Borrowed rather than owned, so serving a file held for the life of the
/// program copies it into the socket's buffer and nowhere else.
#[derive(Debug, Clone, Copy)]
pub struct StaticFile<'a> {
    /// The bytes.
    pub body: &'a [u8],
    /// What to serve them as.
    pub content_type: &'static str,
}

impl Content for StaticFile<'_> {
    fn content_type(&self) -> &'static str {
        self.content_type
    }

    fn content_length(&self) -> usize {
        self.body.len()
    }

    async fn write_content<W: Write>(self, mut writer: W) -> Result<(), W::Error> {
        writer.write_all(self.body).await
    }
}

#[cfg(feature = "site")]
pub use site_files::SiteFiles;

#[cfg(feature = "site")]
mod site_files {
    use picoserve::ResponseSent;
    use picoserve::io::Read;
    use picoserve::request::{Path, Request};
    use picoserve::response::{IntoResponse, ResponseWriter, StatusCode};
    use picoserve::routing::PathRouterService;

    use super::StaticFile;
    use crate::site::Site;

    /// Serves a [`Site`]: a file by request path, or a 404.
    ///
    /// Made to be the router's innermost service —
    /// `Router::from_service(SiteFiles(site))` — so every path no route
    /// claims falls through to the files. A service rather than a route
    /// per file, because the site's paths are whatever the card holds: a
    /// page added there should not also have to be added to the router.
    ///
    /// Anything but `GET` is a 405. The site is files and nothing here is
    /// written to, so a `POST` to one is a client with the wrong address
    /// for an API route, and a 404 would send it looking for a file that
    /// was never missing.
    #[derive(Debug, Clone, Copy)]
    pub struct SiteFiles<'s>(pub &'s Site);

    impl<State, PathParameters> PathRouterService<State, PathParameters> for SiteFiles<'_> {
        async fn call_path_router_service<R: Read, W: ResponseWriter<Error = R::Error>>(
            &self,
            _state: &State,
            _path_parameters: PathParameters,
            path: Path<'_>,
            request: Request<'_, R>,
            response_writer: W,
        ) -> Result<ResponseSent, W::Error> {
            if request.parts.method() != "GET" {
                return (StatusCode::new(405), "")
                    .write_to(request.body_connection.finalize().await?, response_writer)
                    .await;
            }

            match self.0.get(path.encoded()) {
                Some(asset) => {
                    (
                        StatusCode::OK,
                        StaticFile {
                            body: &asset.body,
                            content_type: asset.content_type,
                        },
                    )
                        .write_to(request.body_connection.finalize().await?, response_writer)
                        .await
                }
                // Including `/` on a board whose card has no site at all.
                // `site::load` said so on the console at boot; there is no
                // page to say it with here.
                None => {
                    (StatusCode::new(404), "not found")
                        .write_to(request.body_connection.finalize().await?, response_writer)
                        .await
                }
            }
        }
    }
}

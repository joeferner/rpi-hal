//! A minimal HTTPS client: one request, one response, connection closed.
//!
//! What a board says to the outside world is a handful of small requests to
//! known endpoints — a forecast, a push notification. Those need no
//! connection reuse, no redirects and no content negotiation, and each of
//! those is surface that can go wrong on a device nobody logs into. So this
//! is deliberately the smallest client that does the job:
//!
//! ```ignore
//! let client = https::Client { user_agent: "rpi-weather-station", ..https::Client::DEFAULT };
//! let response = client
//!     .send(stack, tls_config.clone(), &https::Request::get("api.open-meteo.com", "/v1/forecast?…"))
//!     .await?;
//! if response.status != 200 {
//!     logln!("forecast: {} -- {}", response.status_line, response.preview());
//! }
//! ```
//!
//! It does need **chunked framing**, which is the one thing on that list
//! that is not optional: a server may answer a small body with
//! `Transfer-Encoding: chunked` rather than a `Content-Length`, and a client
//! that took what followed the headers as the body would hand its parser a
//! hexadecimal length and two CRLFs along with the content.
//!
//! [`Client::send`](crate::https::Client::send) is where the stack meets:
//! DNS and TCP from `embassy-net`, [`TlsStream`](crate::tls::TlsStream) over
//! the socket, `rustls` verifying the chain, and the clock making that
//! verification mean something — so every request fails closed until
//! something has set the time.
//!
//! # The body comes back whole, and so does a refusal's
//!
//! A server that refuses a request explains itself in the body, and a client
//! that discards it turns a precise diagnosis into "400". So the body is
//! returned whatever the status, and
//! [`Response::preview`](crate::https::Response::preview) is the excerpt
//! made safe to put on a console.

use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt::{self, Write as _};

use embassy_net::dns::DnsQueryType;
use embassy_net::tcp::TcpSocket;
use embassy_net::{IpEndpoint, Stack};
use embassy_time::{Duration, with_timeout};
use embedded_io_async::{Read, Write};
use rustls::ClientConfig;

use crate::tls::{self, TlsStream};

/// Longest body excerpt [`Response::preview`] returns.
const PREVIEW: usize = 256;

/// How a board's requests are made: what it calls itself, and its limits.
/// Start from [`Client::DEFAULT`].
#[derive(Debug, Clone, Copy)]
pub struct Client {
    /// The `User-Agent` sent — the board's name, so the far end's logs say
    /// who asked.
    pub user_agent: &'static str,
    /// How long a whole exchange may take, DNS through the last byte.
    /// Default 30 seconds.
    ///
    /// One deadline around all of it rather than one per step, because the
    /// caller cares how long it can be kept waiting, not which stage was
    /// slow — and the failure it bounds is a server that accepts a
    /// connection and then says nothing.
    pub timeout: Duration,
    /// Bytes of response kept: status line, headers and body. Default 4 KiB.
    ///
    /// A response larger than this is cut short. A chunked body cut short
    /// fails to parse, and a plain one comes back as a prefix — which is
    /// why it is a limit to size against the largest answer expected,
    /// generously, rather than a buffer to economize on.
    pub response_max: usize,
    /// TCP buffer, in bytes, per direction. Default 4 KiB.
    pub tcp_buffer: usize,
}

impl Client {
    /// The defaults, as a constant.
    pub const DEFAULT: Self = Client {
        user_agent: "rpi-kickstart",
        timeout: Duration::from_secs(30),
        response_max: 4 * 1024,
        tcp_buffer: 4 * 1024,
    };

    /// Performs `request` and returns what came back, whatever its status.
    ///
    /// The TCP and response buffers come off the heap for the length of the
    /// exchange.
    pub async fn send(
        &self,
        stack: Stack<'_>,
        config: Arc<ClientConfig>,
        request: &Request<'_>,
    ) -> Result<Response, Error> {
        match with_timeout(self.timeout, self.exchange(stack, config, request)).await {
            Ok(result) => result,
            Err(_) => Err(Error::Timeout),
        }
    }

    async fn exchange(
        &self,
        stack: Stack<'_>,
        config: Arc<ClientConfig>,
        request: &Request<'_>,
    ) -> Result<Response, Error> {
        let addresses = stack
            .dns_query(request.host, DnsQueryType::A)
            .await
            .map_err(|_| Error::Dns)?;
        let address = *addresses.first().ok_or(Error::NoAddress)?;

        let mut rx_buffer = vec![0u8; self.tcp_buffer];
        let mut tx_buffer = vec![0u8; self.tcp_buffer];
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket
            .connect(IpEndpoint::new(address, request.port))
            .await
            .map_err(|_| Error::Connect)?;

        let mut tls = TlsStream::connect(socket, config, request.host)
            .await
            .map_err(Error::Tls)?;

        tls.write_all(head(request, self.user_agent).as_bytes())
            .await
            .map_err(|_| Error::Io)?;
        if !request.body.is_empty() {
            tls.write_all(request.body).await.map_err(|_| Error::Io)?;
        }
        tls.flush().await.map_err(|_| Error::Io)?;

        // Read until the peer closes: `Connection: close` bounds it, and the
        // buffer bounds it again for an answer larger than expected.
        let mut buffer = vec![0u8; self.response_max];
        let mut filled = 0;
        while filled < buffer.len() {
            match tls.read(&mut buffer[filled..]).await {
                Ok(0) => break,
                Ok(n) => filled += n,
                // What arrived before the failure is still worth parsing: a
                // server that answers and then drops the connection has
                // already said what it had to.
                Err(_) if filled > 0 => break,
                Err(_) => return Err(Error::Io),
            }
        }
        parse(&buffer[..filled])
    }
}

/// One request.
///
/// Headers are the caller's: [`Client::send`] adds the three every request
/// needs (`Host`, `Connection: close`, `User-Agent`) and `Content-Length`
/// for a body, and has no opinion about the rest.
#[derive(Debug, Clone, Copy)]
pub struct Request<'a> {
    /// HTTP method, uppercase.
    pub method: &'a str,
    /// Host name, used for DNS, the `Host` header and TLS server-name
    /// verification alike — the same name in all three, which is what makes
    /// the certificate check mean anything.
    pub host: &'a str,
    /// TCP port; 443 unless the endpoint is somewhere unusual.
    pub port: u16,
    /// Request target, beginning with `/`.
    pub path: &'a str,
    /// Extra headers, as name/value pairs. Values must not contain CR or
    /// LF — see `config::value::word`, which refuses any value that could
    /// end one header and begin another.
    pub headers: &'a [(&'a str, &'a str)],
    /// The body. `Content-Length` is sent for anything but a `GET`.
    pub body: &'a [u8],
}

impl<'a> Request<'a> {
    /// A `GET` of `path` from `host` on 443, with no extra headers.
    pub fn get(host: &'a str, path: &'a str) -> Self {
        Request {
            method: "GET",
            host,
            port: 443,
            path,
            headers: &[],
            body: &[],
        }
    }
}

/// As much of a response as was read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Response {
    /// The status code from the status line.
    pub status: u16,
    /// The whole status line, for logging a failure usefully.
    pub status_line: String,
    /// The body, unchunked if it was chunked. Empty if there was none.
    pub body: Vec<u8>,
}

impl Response {
    /// The start of the body, safe to print: control characters become
    /// spaces, so a server's error text cannot rearrange the console it is
    /// logged to, and it is cut to 256 bytes, so a server answering with a
    /// web page cannot scroll it either.
    pub fn preview(&self) -> String {
        let body = &self.body[..self.body.len().min(PREVIEW)];
        let cleaned: String = String::from_utf8_lossy(body)
            .chars()
            .map(|c| if c.is_control() { ' ' } else { c })
            .collect();
        String::from(cleaned.trim())
    }
}

/// Why a request failed.
#[derive(Debug)]
pub enum Error {
    /// The host name would not resolve — usually a lease with no usable
    /// resolver, worth telling apart from a server that is down.
    Dns,
    /// The name resolved to no addresses.
    NoAddress,
    /// The TCP connection could not be made.
    Connect,
    /// The TLS layer failed — including, deliberately, certificate
    /// rejection, and a clock nothing has set yet.
    Tls(tls::Error),
    /// Writing the request or reading the response failed.
    Io,
    /// What came back was not an HTTP response this could take apart.
    BadResponse,
    /// The exchange took longer than [`Client::timeout`].
    Timeout,
}

/// One line, for the console.
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Dns => f.write_str("name would not resolve"),
            Error::NoAddress => f.write_str("name resolved to nothing"),
            Error::Connect => f.write_str("could not connect"),
            Error::Tls(e) => write!(f, "{e}"),
            Error::Io => f.write_str("connection failed mid-exchange"),
            Error::BadResponse => f.write_str("not an HTTP response"),
            Error::Timeout => f.write_str("timed out"),
        }
    }
}

impl core::error::Error for Error {}

/// The request line and headers.
///
/// `Connection: close`, so the server ends the response by closing — which
/// is what lets the reader run to the end without tracking a length.
fn head(request: &Request<'_>, user_agent: &str) -> String {
    let mut text = String::new();
    let _ = write!(
        text,
        "{} {} HTTP/1.1\r\nHost: {}\r\nConnection: close\r\nUser-Agent: {user_agent}\r\n",
        request.method, request.path, request.host
    );
    if request.method != "GET" {
        let _ = write!(text, "Content-Length: {}\r\n", request.body.len());
    }
    for (name, value) in request.headers {
        let _ = write!(text, "{name}: {value}\r\n");
    }
    text.push_str("\r\n");
    text
}

/// Takes a response apart: status line, then the body after the blank line,
/// unchunked when the headers say it is chunked.
fn parse(response: &[u8]) -> Result<Response, Error> {
    let line_end = response
        .iter()
        .position(|&b| b == b'\n')
        .ok_or(Error::BadResponse)?;
    let line = core::str::from_utf8(&response[..line_end])
        .map_err(|_| Error::BadResponse)?
        .trim_end();

    // `HTTP/1.1 200 OK` — the code is the second field. Anything that does
    // not look like that is a failure, not something to guess at.
    if !line.starts_with("HTTP/") {
        return Err(Error::BadResponse);
    }
    let status: u16 = line
        .split_whitespace()
        .nth(1)
        .ok_or(Error::BadResponse)?
        .parse()
        .map_err(|_| Error::BadResponse)?;

    // A response with no blank line has no body, which for most requests is
    // a malformed answer -- left empty, so it fails at the caller's parse
    // rather than being guessed at here.
    let body = match response.windows(4).position(|window| window == b"\r\n\r\n") {
        Some(at) => {
            let (head, body) = (&response[..at], &response[at + 4..]);
            if is_chunked(head) {
                dechunk(body).ok_or(Error::BadResponse)?
            } else {
                Vec::from(body)
            }
        }
        None => Vec::new(),
    };

    Ok(Response {
        status,
        status_line: String::from(line),
        body,
    })
}

/// Whether the headers say the body is chunked.
///
/// Only `Transfer-Encoding`, and only for the word. A list — `gzip,
/// chunked` — would need the *last* coding to be this, but this client
/// sends no `Accept-Encoding`, so a server layering a content coding on top
/// is answering something never asked; what it produces is a body that
/// does not parse, the same visible failure as any other surprise.
fn is_chunked(head: &[u8]) -> bool {
    head.split(|&b| b == b'\n').any(|line| {
        let Some(colon) = line.iter().position(|&b| b == b':') else {
            return false;
        };
        line[..colon]
            .trim_ascii()
            .eq_ignore_ascii_case(b"transfer-encoding")
            && line[colon + 1..]
                .windows(7)
                .any(|word| word.eq_ignore_ascii_case(b"chunked"))
    })
}

/// Reassembles a chunked body.
///
/// A chunk is a hexadecimal length on its own line, that many bytes and a
/// line ending; a zero length ends the body. Trailers after it are not read:
/// nothing here asked for one, and the body is complete without them.
///
/// `None` for a body this cannot walk, which is deliberately also what a
/// *truncated* one gives: a chunk claiming more bytes than arrived is what a
/// response over [`Client::response_max`] looks like from here, and handing
/// back the part that fit would be a prefix of the truth with nothing to say
/// so.
fn dechunk(mut body: &[u8]) -> Option<Vec<u8>> {
    let mut whole = Vec::new();
    loop {
        let line_end = body.iter().position(|&b| b == b'\n')?;
        let line = body[..line_end].trim_ascii();
        // A chunk extension -- `1c4;name=value` -- is framing, not length.
        let size = line.split(|&b| b == b';').next()?;
        let size = usize::from_str_radix(core::str::from_utf8(size).ok()?, 16).ok()?;
        body = body.get(line_end + 1..)?;

        if size == 0 {
            return Some(whole);
        }
        whole.extend_from_slice(body.get(..size)?);
        // The line ending that closes the chunk, taken as up to two bytes
        // rather than exactly `\r\n`, which keeps a server sending a bare LF
        // from failing the whole request.
        body = &body[size..];
        while let Some((&first, rest)) = body.split_first() {
            if first != b'\r' && first != b'\n' {
                break;
            }
            body = rest;
            if first == b'\n' {
                break;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_head_names_the_host_and_closes() {
        let get = Request::get("api.example.com", "/v1/x?y=1");
        assert_eq!(
            head(&get, "board"),
            "GET /v1/x?y=1 HTTP/1.1\r\nHost: api.example.com\r\nConnection: close\r\n\
             User-Agent: board\r\n\r\n"
        );
        let post = Request {
            method: "POST",
            headers: &[("Authorization", "Bearer t")],
            body: b"hello",
            ..Request::get("ntfy.example.com", "/topic")
        };
        let text = head(&post, "board");
        assert!(text.contains("Content-Length: 5\r\n"), "{text}");
        assert!(text.ends_with("Authorization: Bearer t\r\n\r\n"), "{text}");
    }

    #[test]
    fn a_plain_body_comes_back_as_it_arrived() {
        let response = parse(b"HTTP/1.1 200 OK\r\nContent-Length: 5\r\n\r\nhello").unwrap();
        assert_eq!(response.status, 200);
        assert_eq!(response.status_line, "HTTP/1.1 200 OK");
        assert_eq!(response.body, b"hello");
    }

    #[test]
    fn a_chunked_body_is_reassembled() {
        let wire = b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n\
                     5\r\nhello\r\n7;ext=1\r\n, world\r\n0\r\nTrailer: x\r\n\r\n";
        assert_eq!(parse(wire).unwrap().body, b"hello, world");
        // A bare LF closing a chunk, and a header in another case.
        let wire = b"HTTP/1.1 200 OK\r\ntransfer-encoding: Chunked\r\n\r\n3\nabc\n0\n\n";
        assert_eq!(parse(wire).unwrap().body, b"abc");
    }

    #[test]
    fn a_chunked_body_cut_short_is_refused_not_shortened() {
        let wire = b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n10\r\nonly six";
        assert!(matches!(parse(wire), Err(Error::BadResponse)));
    }

    #[test]
    fn what_is_not_a_response() {
        for wire in [
            &b""[..],
            b"garbage\r\n",
            b"HTTP/1.1 abc OK\r\n\r\n",
            b"SSH-2.0-x\r\n",
        ] {
            assert!(matches!(parse(wire), Err(Error::BadResponse)), "{wire:?}");
        }
        // No blank line: a status, and no body to speak of.
        let response = parse(b"HTTP/1.1 204 No Content\r\n").unwrap();
        assert_eq!((response.status, response.body.len()), (204, 0));
    }

    #[test]
    fn a_refusal_previews_safely() {
        let mut body = b"bad\r\n\x1b[2Jtoken".to_vec();
        body.extend(core::iter::repeat_n(b'x', 1000));
        let response = Response {
            status: 401,
            status_line: String::from("HTTP/1.1 401 Unauthorized"),
            body,
        };
        let preview = response.preview();
        assert!(preview.starts_with("bad   [2Jtoken"), "{preview}");
        assert!(preview.len() <= PREVIEW);
    }
}

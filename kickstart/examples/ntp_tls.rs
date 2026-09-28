#![no_std]
#![no_main]

// The wall clock from the network, then TLS against it: come up on
// Ethernet, set the clock with SNTP, and once it is set, make two TLS 1.3
// connections -- one that must verify and carry a request, and one that
// must be refused.
//
// The order is the point. TLS checks certificate validity against
// `clock`, which reads `None` until something sets it, so every handshake
// fails closed until SNTP has answered. The TLS task waits for the clock
// and says so, rather than logging a handshake failure with no clue
// attached.
//
// On the card, beside `config.txt`, optionally:
//
//   kickstart.toml -- an `[ntp]` table (`server`, `retry_interval`,
//                     `resync_interval`) overrides the defaults:
//                     `pool.ntp.org`, 30 s, 6 h. See
//                     `kickstart.toml.example`.
//
// A failed sync names the address a pool name resolved to, and every
// reply it ignores is logged with why -- a kiss of death with its code --
// so "no reply" and "replies, none usable" read differently.
//
// Build it with `scripts/build-example.sh ntp_tls`. The image is ~1.2 MB
// larger than the others, almost all of it `rustls`, its crypto and the
// trust anchors.

extern crate alloc;

use alloc::format;
use alloc::string::String;
use core::cell::Cell;

use common::settings;
use critical_section::Mutex;
use embassy_net::Stack;
use rpi_kickstart::sntp::{self, NtpConfig};
use rpi_kickstart::tls::{self, TlsStream};
use rpi_kickstart::{clock, logln, net};

mod common;

/// Where the clock comes from: the defaults unless `[ntp]` says otherwise.
/// A static because it is decided during bring-up and read by a task
/// spawned much later.
static NTP: Mutex<Cell<NtpConfig<'static>>> = Mutex::new(Cell::new(NtpConfig::DEFAULT));

#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let mut board = common::boot("ntp_tls");

    // Only `[ntp]` is wanted from the card, and the card is not kept.
    if let Some((settings, text)) = settings::read(&mut board).settings {
        match settings.ntp() {
            Ok(ntp) => critical_section::with(|cs| NTP.borrow(cs).set(ntp)),
            Err(problem) => logln!(
                "settings: {}; using the NTP defaults",
                problem.report(settings::FILE, &text)
            ),
        }
    }

    let usb = common::usb(&mut board);
    let mut hardware = net::Hardware::new().usb(usb.dwc2);
    let interface = common::discover(&mut hardware, &board, &usb);

    common::start(&board, &usb, interface, |spawner, stack| {
        spawner.spawn(sntp_task(stack).unwrap());
        spawner.spawn(tls_task(stack).unwrap());
    })
}

/// Keeps the wall clock set, which `sntp::run` does by calling
/// `clock::set` on every answer.
#[embassy_executor::task]
async fn sntp_task(stack: Stack<'static>) -> ! {
    let config = critical_section::with(|cs| NTP.borrow(cs).get());
    sntp::run(stack, config).await
}

/// One half of the TLS check: connect to `host`'s address, verify the
/// certificate against `name`, and whether that must succeed.
struct TlsCheck {
    host: &'static str,
    name: &'static str,
    must_verify: bool,
}

/// The TLS check.
///
/// The first case is any public server speaking TLS 1.3, by its own name;
/// this one exists for being used in examples.
///
/// The second is the one that matters — a TLS layer that accepts
/// everything also passes the first — and it has to be built with care,
/// because two obvious versions of it prove nothing:
///
/// * **A made-up name under the first host's domain** passes, correctly:
///   `example.com`'s certificate also names `*.example.com`. That was this
///   check's first form, and it reported a wrong name accepted when the
///   name was right.
/// * **A test site's deliberately bad certificate** — `badssl.com` and its
///   kin — is refused for the wrong reason: those servers do not speak TLS
///   1.3, and this configuration speaks nothing else.
///
/// So: a large front-end that completes a 1.3 handshake with its own valid
/// certificate for a name it does not host, asked for a name under
/// `.example`, which RFC 2606 reserves and no publicly trusted certificate
/// can cover. The chain verifies, the name does not, and only the client's
/// check can refuse it.
const TLS_CHECKS: [TlsCheck; 2] = [
    TlsCheck {
        host: "example.com",
        name: "example.com",
        must_verify: true,
    },
    TlsCheck {
        host: "www.google.com",
        name: "kickstart-wrong-name.example",
        must_verify: false,
    },
];

/// Runs [`TLS_CHECKS`] once the clock is set, and says how each went.
#[embassy_executor::task]
async fn tls_task(stack: Stack<'static>) {
    stack.wait_config_up().await;
    if clock::now_unix().is_none() {
        logln!("tls: waiting for the clock");
        while clock::now_unix().is_none() {
            embassy_time::Timer::after_millis(500).await;
        }
    }

    let config = tls::client_config();
    logln!("tls: {} trust anchors", tls::trust_anchor_count());

    for TlsCheck {
        host,
        name,
        must_verify,
    } in TLS_CHECKS
    {
        let address = match stack
            .dns_query(host, embassy_net::dns::DnsQueryType::A)
            .await
        {
            Ok(addresses) if !addresses.is_empty() => addresses[0],
            other => {
                logln!("tls: {host} did not resolve ({other:?})");
                continue;
            }
        };
        logln!("tls: {name}: connecting to {host} at {address}");

        let mut rx = [0u8; 4096];
        let mut tx = [0u8; 1024];
        let mut socket = embassy_net::tcp::TcpSocket::new(stack, &mut rx, &mut tx);
        socket.set_timeout(Some(embassy_time::Duration::from_secs(15)));
        if let Err(e) = socket.connect((address, 443)).await {
            logln!("tls: {name}: TCP connect failed: {e:?}");
            continue;
        }

        let started = embassy_time::Instant::now();
        match TlsStream::connect(socket, config.clone(), name).await {
            Ok(mut stream) => {
                let took = started.elapsed().as_millis();
                logln!(
                    "tls: {name}: handshake in {took} ms, {:?}, {:?}{}",
                    stream.protocol_version(),
                    stream.cipher_suite().map(|suite| suite.suite()),
                    if must_verify {
                        ""
                    } else {
                        " -- ACCEPTED A WRONG NAME"
                    }
                );
                logln!("tls: {name}: {}", head(&mut stream, name).await);
            }
            Err(e) => logln!(
                "tls: {name}: refused: {e}{}",
                if must_verify {
                    " -- A GOOD HOST WAS REFUSED"
                } else {
                    ", as it must be"
                }
            ),
        }
    }
}

/// Sends `HEAD /` over `stream` and returns the response's status line, or
/// what went wrong. Enough to show application data both ways, which a
/// handshake alone does not.
async fn head(stream: &mut TlsStream<'_>, host: &str) -> String {
    use embedded_io_async::{Read, Write};

    let request = format!("HEAD / HTTP/1.1\r\nHost: {host}\r\nConnection: close\r\n\r\n");
    if let Err(e) = stream.write_all(request.as_bytes()).await {
        return format!("request failed: {e}");
    }
    let mut response = [0u8; 256];
    let mut len = 0;
    while len < response.len() {
        match stream.read(&mut response[len..]).await {
            Ok(0) => break,
            Ok(n) => len += n,
            Err(e) => return format!("response failed: {e}"),
        }
        if response[..len].contains(&b'\n') {
            break;
        }
    }
    let line = response[..len]
        .split(|&b| b == b'\r' || b == b'\n')
        .next()
        .unwrap_or(&[]);
    String::from_utf8_lossy(line).into_owned()
}

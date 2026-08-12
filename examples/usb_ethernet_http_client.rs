#![no_std]
#![no_main]

// An HTTP client over the on-board Ethernet, using `smoltcp` TCP + DNS.
//
// Bring-up is identical to `usb_ethernet_smoltcp`: power the USB
// controller on, start DWC2, enumerate the bus, bring the LAN9514 up
// (program the firmware MAC, enable RX/TX, wait for link), wrap it in the
// `smoltcp` `phy::Device` adapter, and get an address over DHCP. Where
// that example stands up a UDP echo *server*, this one:
//
//   1. resolves a hostname via DNS, using the DNS servers DHCP handed out,
//   2. opens an outbound TCP connection to the resolved address, and
//   3. performs a single HTTP/1.0 `GET`, printing the raw response
//      (status line, headers, and body) to the UART.
//
// This is the first thing in the crate to exercise smoltcp **TCP** and
// **DNS** sockets -- the library's `smoltcp` feature only pulls in
// `socket-udp`, so this example's own dev-dependency entry adds
// `socket-tcp` and `socket-dns` (see `Cargo.toml`). The `Lan9514Phy`
// adapter is protocol-agnostic, so the same phy carries all of them.
//
// The default target is `http://httpforever.com/`, a site that
// intentionally stays plain HTTP (no HTTPS redirect) -- ideal for a stack
// with no TLS. Reaching it needs the LAN's DHCP to provide a working
// gateway + DNS server and a route to the internet. Point it elsewhere by
// editing the `SERVER_HOST` / `SERVER_PORT` / `REQUEST_PATH` constants; a
// local `python3 -m http.server 80` works too (its hostname must resolve,
// or use its dotted-quad address as `SERVER_HOST`).

use core::fmt::Write;
use core::ops::ControlFlow;
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::usb;
use rpi_hal::usb::dwc2::Dwc2Host;
use rpi_hal::usb::lan9514::{Lan9514, Lan9514Phy};
use smoltcp::iface::{Config, Interface, SocketSet, SocketStorage};
use smoltcp::socket::{dhcpv4, dns, tcp};
use smoltcp::time::Instant;
use smoltcp::wire::{DnsQueryType, EthernetAddress, HardwareAddress, IpAddress, IpCidr};

/// Hostname to fetch. Also sent verbatim in the HTTP `Host:` header.
/// **Edit for your target.** May be a dotted-quad address, which resolves
/// to itself.
const SERVER_HOST: &str = "httpforever.com";
/// TCP port to connect to on [`SERVER_HOST`].
const SERVER_PORT: u16 = 80;
/// Request target (path) for the `GET`.
const REQUEST_PATH: &str = "/";
/// Local (source) TCP port for the outbound connection -- any free
/// ephemeral port; nothing else in this example binds it.
const LOCAL_PORT: u16 = 49500;

/// Capacity of the TCP receive buffer, in bytes. The response is streamed
/// out of it as it arrives, so this only bounds how much can be in flight
/// unread at once, not the total response size.
const TCP_RX_BUFFER_SIZE: usize = 4096;
/// Capacity of the TCP transmit buffer, in bytes -- only ever holds the
/// small request line + headers.
const TCP_TX_BUFFER_SIZE: usize = 512;
/// Maximum size of the formatted request, in bytes.
const REQUEST_BUFFER_SIZE: usize = 256;
/// Maximum number of DHCP-provided DNS servers to register.
const MAX_DNS_SERVERS: usize = 3;
/// How long to wait, in microseconds, from starting name resolution to
/// the response completing before giving up (15 s -- covers DNS + connect
/// + transfer).
const REQUEST_TIMEOUT_US: u64 = 15_000_000;

/// One-shot progress: resolve the name, connect, send, receive.
#[derive(PartialEq)]
enum HttpState {
    /// Have an address + DNS servers but haven't started resolving yet.
    Idle,
    /// DNS query in flight.
    Resolving,
    /// Connection initiated; waiting for the handshake to complete.
    Connecting,
    /// Request sent; streaming the response until the peer closes.
    Receiving,
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);

    // The USB controller comes up only partially powered from firmware;
    // power it fully via the mailbox before touching DWC2.
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
    if !usb::power_on(&mut mailbox) {
        let _ = writeln!(uart, "USB power-on failed");
        halt();
    }

    // The MAC lives in firmware on this board, not the chip -- read it
    // here so we can program it into the LAN9514 and hand it to smoltcp.
    let board_mac = match mailbox.mac_address() {
        Ok(mac) => {
            let _ = writeln!(uart, "board MAC: {}", FormatMac(mac));
            mac
        }
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
    while !dwc2.port_connected() {
        timer.delay_ms(100);
    }

    // Enumerate the bus; when the LAN9514 turns up, run the client on it.
    let result = usb::enumerate(&mut dwc2, &timer, |dwc2, timer, device| {
        let lan9514 = match Lan9514::from_device(dwc2, timer, device) {
            Ok(Some(lan9514)) => lan9514,
            Ok(None) => return ControlFlow::Continue(()),
            Err(e) => {
                let _ = writeln!(uart, "LAN9514 setup failed: {e:?}");
                return ControlFlow::Break(());
            }
        };
        run_client(&mut uart, dwc2, timer, lan9514, board_mac);
        ControlFlow::Break(())
    });

    if let Err(e) = result {
        let _ = writeln!(uart, "enumeration failed: {e:?}");
    }
    halt();
}

/// Brings the LAN9514 up, wraps it in a smoltcp `phy::Device`, gets an
/// address over DHCP, resolves [`SERVER_HOST`], performs one HTTP/1.0
/// `GET`, and prints the response. Returns once the request completes,
/// times out, or fails.
fn run_client(
    uart: &mut Uart,
    dwc2: &mut Dwc2Host,
    timer: &Timer,
    mut lan9514: Lan9514,
    mac: [u8; 6],
) {
    if let Err(e) = lan9514.start(dwc2, timer, mac) {
        let _ = writeln!(uart, "LAN9514 start failed: {e:?}");
        return;
    }

    let _ = writeln!(uart, "waiting for link...");
    loop {
        match lan9514.is_link_up(dwc2, timer) {
            Ok(true) => break,
            Ok(false) => timer.delay_ms(100),
            Err(e) => {
                let _ = writeln!(uart, "link check failed: {e:?}");
                return;
            }
        }
    }
    let _ = writeln!(uart, "link up");

    let mut phy = Lan9514Phy::new(lan9514, dwc2, timer);

    // Configure the interface with the board MAC and a fresh clock seed.
    // No IP is assigned here -- DHCP fills that in once it gets a lease.
    let mut config = Config::new(HardwareAddress::Ethernet(EthernetAddress(mac)));
    config.random_seed = timer.now_micros();
    let mut iface = Interface::new(config, &mut phy, instant(timer));

    // Three sockets: DHCP to lease an address + learn DNS servers, DNS to
    // resolve the hostname, and TCP for the request itself.
    let mut socket_storage: [SocketStorage; 3] = [SocketStorage::EMPTY; 3];
    let mut sockets = SocketSet::new(&mut socket_storage[..]);
    let dhcp_handle = sockets.add(dhcpv4::Socket::new());

    // DNS servers get filled in from the DHCP lease; start with none.
    let mut dns_queries: [Option<dns::DnsQuery>; 1] = [None];
    let dns_handle = sockets.add(dns::Socket::new(&[], &mut dns_queries[..]));

    let mut tcp_rx_payload = [0u8; TCP_RX_BUFFER_SIZE];
    let mut tcp_tx_payload = [0u8; TCP_TX_BUFFER_SIZE];
    let tcp_handle = sockets.add(tcp::Socket::new(
        tcp::SocketBuffer::new(&mut tcp_rx_payload[..]),
        tcp::SocketBuffer::new(&mut tcp_tx_payload[..]),
    ));

    // Build the request once. `Connection: close` tells the server to
    // close the socket once the response is complete -- that close is our
    // EOF signal. Note we do NOT half-close our own transmit side after
    // sending (a common shortcut): CDNs like Cloudflare treat a client
    // half-close before the response as an abort and hang up with no
    // reply, so we leave the connection fully open until the server closes
    // it. HTTP/1.1 (with an explicit `Host`) is handled more predictably
    // by CDNs than 1.0.
    let mut request_buf = [0u8; REQUEST_BUFFER_SIZE];
    let request_len = {
        let mut w = SliceWriter::new(&mut request_buf);
        let _ = write!(
            w,
            "GET {REQUEST_PATH} HTTP/1.1\r\nHost: {SERVER_HOST}\r\nConnection: close\r\n\r\n"
        );
        w.len()
    };
    let request = &request_buf[..request_len];

    let _ = writeln!(uart, "requesting an address over DHCP...");
    // Whether we currently hold a lease -- so the initial `Deconfigured`
    // event smoltcp emits before the first lease isn't reported as a loss.
    let mut configured = false;
    let mut http = HttpState::Idle;
    let mut query_handle: Option<dns::QueryHandle> = None;
    let mut deadline: Option<u64> = None;
    loop {
        iface.poll(instant(timer), &mut phy, &mut sockets);

        // Apply whatever the DHCP client just learned to the interface.
        match sockets.get_mut::<dhcpv4::Socket>(dhcp_handle).poll() {
            None => {}
            Some(dhcpv4::Event::Configured(dhcp)) => {
                configured = true;
                let _ = writeln!(uart, "DHCP: address {}", dhcp.address);
                iface.update_ip_addrs(|addrs| {
                    addrs.clear();
                    let _ = addrs.push(IpCidr::Ipv4(dhcp.address));
                });
                match dhcp.router {
                    Some(router) => {
                        let _ = writeln!(uart, "DHCP: gateway {router}");
                        let _ = iface.routes_mut().add_default_ipv4_route(router);
                    }
                    None => {
                        iface.routes_mut().remove_default_ipv4_route();
                    }
                }
                // Register the leased DNS servers so the DNS socket can
                // resolve names.
                let mut servers = [IpAddress::v4(0, 0, 0, 0); MAX_DNS_SERVERS];
                let mut n = 0;
                for dns_server in dhcp.dns_servers.iter() {
                    if n < servers.len() {
                        let _ = writeln!(uart, "DHCP: dns {dns_server}");
                        servers[n] = IpAddress::Ipv4(*dns_server);
                        n += 1;
                    }
                }
                sockets
                    .get_mut::<dns::Socket>(dns_handle)
                    .update_servers(&servers[..n]);
            }
            Some(dhcpv4::Event::Deconfigured) => {
                if configured {
                    let _ = writeln!(uart, "DHCP: lease lost");
                    configured = false;
                }
                iface.update_ip_addrs(|addrs| addrs.clear());
                iface.routes_mut().remove_default_ipv4_route();
            }
        }

        // Drive the one-shot request once we have an address.
        if configured {
            match http {
                HttpState::Idle => {
                    let dns_socket = sockets.get_mut::<dns::Socket>(dns_handle);
                    match dns_socket.start_query(iface.context(), SERVER_HOST, DnsQueryType::A) {
                        Ok(handle) => {
                            let _ = writeln!(uart, "resolving {SERVER_HOST}...");
                            query_handle = Some(handle);
                            deadline = Some(timer.now_micros() + REQUEST_TIMEOUT_US);
                            http = HttpState::Resolving;
                        }
                        Err(e) => {
                            let _ = writeln!(uart, "DNS start_query failed: {e:?}");
                            return;
                        }
                    }
                }
                HttpState::Resolving => {
                    let dns_socket = sockets.get_mut::<dns::Socket>(dns_handle);
                    // `get_query_result` returns an owned address list, so
                    // the DNS-socket borrow ends here -- freeing us to take
                    // the TCP socket below to initiate the connection.
                    match dns_socket.get_query_result(query_handle.unwrap()) {
                        Ok(addresses) => match addresses.first() {
                            Some(&ip) => {
                                let _ = writeln!(uart, "resolved {SERVER_HOST} -> {ip}");
                                let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
                                match socket.connect(iface.context(), (ip, SERVER_PORT), LOCAL_PORT)
                                {
                                    Ok(()) => {
                                        let _ =
                                            writeln!(uart, "connecting to {ip}:{SERVER_PORT}...");
                                        http = HttpState::Connecting;
                                    }
                                    Err(e) => {
                                        let _ = writeln!(uart, "connect failed: {e:?}");
                                        return;
                                    }
                                }
                            }
                            None => {
                                let _ = writeln!(uart, "DNS returned no addresses");
                                return;
                            }
                        },
                        Err(dns::GetQueryResultError::Pending) => {}
                        Err(dns::GetQueryResultError::Failed) => {
                            let _ = writeln!(uart, "DNS query failed");
                            return;
                        }
                    }
                }
                HttpState::Connecting => {
                    // `may_send` first goes true when the handshake reaches
                    // ESTABLISHED and the TX half is open.
                    let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
                    if socket.may_send() {
                        match socket.send_slice(request) {
                            Ok(_) => {
                                let _ = writeln!(uart, "connected; sent GET {REQUEST_PATH}");
                                let _ = writeln!(uart, "-- response --");
                                // Leave our transmit half open: the server
                                // closes when the response is done (we asked
                                // for `Connection: close`), and that close is
                                // our EOF.
                                http = HttpState::Receiving;
                            }
                            Err(e) => {
                                let _ = writeln!(uart, "send failed: {e:?}");
                                return;
                            }
                        }
                    }
                }
                HttpState::Receiving => {
                    let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
                    while socket.can_recv() {
                        let r = socket.recv(|data| {
                            match core::str::from_utf8(data) {
                                Ok(s) => {
                                    let _ = write!(uart, "{s}");
                                }
                                Err(_) => {
                                    for &b in data.iter() {
                                        uart.write_byte(b);
                                    }
                                }
                            }
                            (data.len(), ())
                        });
                        if let Err(e) = r {
                            let _ = writeln!(uart, "\nrecv failed: {e:?}");
                            return;
                        }
                    }
                    // Done when the server has closed its send half (so no
                    // more response bytes can arrive) and we've drained
                    // what it sent. `may_recv` goes false once that FIN is
                    // processed; the `while` above has already emptied the
                    // RX buffer this poll. Close our half to finish cleanly.
                    if !socket.may_recv() {
                        socket.close();
                        let _ = writeln!(uart, "\n-- request complete --");
                        return;
                    }
                }
            }

            // Bound the whole exchange so an unreachable server, a failed
            // resolve, or a half-open connection doesn't wedge the loop.
            if let Some(d) = deadline {
                if timer.now_micros() > d {
                    let _ = writeln!(uart, "\ntimed out");
                    return;
                }
            }
        }

        // Pace the loop -- an idle poll is a cheap register read (the phy
        // skips the bulk-IN when the chip's RX FIFO is empty), so 1ms is
        // plenty responsive without hammering the bus.
        timer.delay_ms(1);
    }
}

/// The current time as a smoltcp [`Instant`], from the system timer.
fn instant(timer: &Timer) -> Instant {
    Instant::from_micros(timer.now_micros() as i64)
}

/// A `core::fmt::Write` sink that formats into a fixed byte slice, used to
/// build the request without a heap. Formatting past the end of the slice
/// is truncated (and reported as a `fmt` error, which the caller ignores).
struct SliceWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> SliceWriter<'a> {
    /// Wraps `buf` as an empty writer positioned at its start.
    fn new(buf: &'a mut [u8]) -> Self {
        SliceWriter { buf, pos: 0 }
    }

    /// Number of bytes written so far.
    fn len(&self) -> usize {
        self.pos
    }
}

impl core::fmt::Write for SliceWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let end = (self.pos + bytes.len()).min(self.buf.len());
        let n = end - self.pos;
        self.buf[self.pos..end].copy_from_slice(&bytes[..n]);
        self.pos = end;
        if n < bytes.len() {
            Err(core::fmt::Error)
        } else {
            Ok(())
        }
    }
}

/// Formats a MAC address as colon-separated hex (`aa:bb:cc:dd:ee:ff`).
struct FormatMac([u8; 6]);

impl core::fmt::Display for FormatMac {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let m = self.0;
        write!(
            f,
            "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            m[0], m[1], m[2], m[3], m[4], m[5]
        )
    }
}

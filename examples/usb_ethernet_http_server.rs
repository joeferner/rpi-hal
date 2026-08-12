#![no_std]
#![no_main]

// An HTTP server over the on-board Ethernet, using `smoltcp` TCP.
//
// Bring-up is identical to `usb_ethernet_smoltcp`: power the USB
// controller on, start DWC2, enumerate the bus, bring the LAN9514 up
// (program the firmware MAC, enable RX/TX, wait for link), wrap it in the
// `smoltcp` `phy::Device` adapter, and get an address over DHCP. Where
// `usb_ethernet_http_client` makes an *outbound* TCP connection, this
// stands up an *inbound* HTTP server: it listens on port 80, reads each
// request's headers, logs the request line to the UART, and serves a
// small fixed HTML page with `Connection: close`.
//
// Together with the client example this covers both directions of smoltcp
// TCP -- the library's `smoltcp` feature only pulls in `socket-udp`, so
// the example's own dev-dependency entry adds `socket-tcp` (see
// `Cargo.toml`). The `Lan9514Phy` adapter is protocol-agnostic and knows
// nothing about TCP vs UDP.
//
// A small pool of listening sockets accepts several connections rather
// than serializing everything behind one -- a browser opens multiple
// connections at once, and a socket that has just served a request sits
// in TCP TIME-WAIT for a while before it can listen again. Test it with
// `curl http://<address the board printed>/` or a browser.

use core::fmt::Write;
use core::ops::ControlFlow;
use core::ptr::addr_of_mut;
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::usb;
use rpi_hal::usb::dwc2::Dwc2Host;
use rpi_hal::usb::lan9514::{Lan9514, Lan9514Phy};
use smoltcp::iface::{Config, Interface, SocketHandle, SocketSet, SocketStorage};
use smoltcp::socket::{dhcpv4, tcp};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, HardwareAddress, IpCidr};

/// TCP port the server listens on.
const HTTP_PORT: u16 = 80;
/// Number of listening sockets in the pool -- how many connections can be
/// in flight (being served, or lingering in TIME-WAIT) at once.
const POOL_SIZE: usize = 4;
/// Per-socket TCP receive buffer, in bytes -- holds one request's headers.
const TCP_RX_BUFFER_SIZE: usize = 1536;
/// Per-socket TCP transmit buffer, in bytes. Must be at least the full
/// response length so the whole response is queued in one `send_slice`.
const TCP_TX_BUFFER_SIZE: usize = 2048;
/// Per-socket buffer for accumulating a request's header bytes across
/// however many segments they arrive in.
const REQUEST_BUFFER_SIZE: usize = 1024;
/// Scratch buffer for formatting the HTTP response.
const RESPONSE_BUFFER_SIZE: usize = 2048;

/// The page served for every request.
const BODY: &str = "<!doctype html>\
<html><head><title>rpi-hal</title></head>\
<body><h1>Hello from rpi-hal</h1>\
<p>Bare-metal HTTP server over the LAN9514 Ethernet, via smoltcp TCP.</p>\
</body></html>\n";

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

    // Enumerate the bus; when the LAN9514 turns up, run the server on it.
    let result = usb::enumerate(&mut dwc2, &timer, |dwc2, timer, device| {
        let lan9514 = match Lan9514::from_device(dwc2, timer, device) {
            Ok(Some(lan9514)) => lan9514,
            Ok(None) => return ControlFlow::Continue(()),
            Err(e) => {
                let _ = writeln!(uart, "LAN9514 setup failed: {e:?}");
                return ControlFlow::Break(());
            }
        };
        run_server(&mut uart, dwc2, timer, lan9514, board_mac);
        ControlFlow::Break(())
    });

    if let Err(e) = result {
        let _ = writeln!(uart, "enumeration failed: {e:?}");
    }
    halt();
}

/// Brings the LAN9514 up, wraps it in a smoltcp `phy::Device`, gets an
/// address over DHCP, and serves HTTP on [`HTTP_PORT`] forever.
fn run_server(
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

    // A DHCP client socket plus the pool of HTTP server sockets.
    let mut socket_storage: [SocketStorage; POOL_SIZE + 1] = [SocketStorage::EMPTY; POOL_SIZE + 1];
    let mut sockets = SocketSet::new(&mut socket_storage[..]);
    let dhcp_handle = sockets.add(dhcpv4::Socket::new());

    // The per-connection TCP + request buffers live in `static` storage
    // (.bss), NOT on the stack. The main stack grows down from the load
    // address `_start` (0x8000), so it's only ~32 KiB (see `boot64.s`);
    // this pool's buffers are ~20 KiB, and a stack frame that large blows
    // past the bottom of the stack once smoltcp's poll + the USB transfers
    // nest on top -- the board hangs mid-bring-up. .bss has the whole ARM
    // RAM region to itself, so that's where anything big belongs.
    //
    // SAFETY: `run_server` is called at most once (the enumerate callback
    // breaks after the first LAN9514 and `kmain` halts afterwards) and this
    // is single-threaded, so each `static mut` below is borrowed exactly
    // once here for the rest of the program -- no aliasing.
    static mut RX_BUFS: [[u8; TCP_RX_BUFFER_SIZE]; POOL_SIZE] =
        [[0; TCP_RX_BUFFER_SIZE]; POOL_SIZE];
    static mut TX_BUFS: [[u8; TCP_TX_BUFFER_SIZE]; POOL_SIZE] =
        [[0; TCP_TX_BUFFER_SIZE]; POOL_SIZE];
    static mut REQ_BUFS: [[u8; REQUEST_BUFFER_SIZE]; POOL_SIZE] =
        [[0; REQUEST_BUFFER_SIZE]; POOL_SIZE];
    let rx_bufs = unsafe { &mut *addr_of_mut!(RX_BUFS) };
    let tx_bufs = unsafe { &mut *addr_of_mut!(TX_BUFS) };
    let req_bufs = unsafe { &mut *addr_of_mut!(REQ_BUFS) };

    // Give each pool socket its own buffer pair. `iter_mut` hands out
    // disjoint mutable borrows, one per socket, held for the socket set's
    // lifetime.
    let mut handles: [Option<SocketHandle>; POOL_SIZE] = [None; POOL_SIZE];
    for ((rx, tx), handle) in rx_bufs
        .iter_mut()
        .zip(tx_bufs.iter_mut())
        .zip(handles.iter_mut())
    {
        let socket = tcp::Socket::new(
            tcp::SocketBuffer::new(&mut rx[..]),
            tcp::SocketBuffer::new(&mut tx[..]),
        );
        *handle = Some(sockets.add(socket));
    }

    // Per-socket request-parsing progress (indexed in lockstep with
    // `handles`); small enough to stay on the stack.
    let mut req_lens = [0usize; POOL_SIZE];
    let mut responded = [false; POOL_SIZE];

    // The response is identical for every request, so format it once.
    let mut response_buf = [0u8; RESPONSE_BUFFER_SIZE];
    let response_len = {
        let mut w = SliceWriter::new(&mut response_buf);
        let _ = write!(
            w,
            "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
            BODY.len(),
            BODY
        );
        w.len()
    };
    let response = &response_buf[..response_len];

    let _ = writeln!(uart, "requesting an address over DHCP...");
    // Whether we currently hold a lease -- so the initial `Deconfigured`
    // event smoltcp emits before the first lease isn't reported as a loss.
    let mut configured = false;
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
                let _ = writeln!(
                    uart,
                    "ready -- try `curl http://{}/`",
                    dhcp.address.address()
                );
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

        // Service each socket in the pool.
        for i in 0..POOL_SIZE {
            let handle = handles[i].unwrap();
            let socket = sockets.get_mut::<tcp::Socket>(handle);

            // A closed socket is idle -- (re)arm it to accept the next
            // connection, resetting its per-connection state.
            if !socket.is_open() {
                req_lens[i] = 0;
                responded[i] = false;
                if let Err(e) = socket.listen(HTTP_PORT) {
                    let _ = writeln!(uart, "listen failed: {e:?}");
                }
                continue;
            }

            // Already served this connection; just let it close.
            if responded[i] {
                continue;
            }

            // Accumulate request bytes into this socket's header buffer.
            if socket.can_recv() {
                let buf = &mut req_bufs[i];
                let start = req_lens[i];
                let n = socket
                    .recv(|data| {
                        let n = data.len().min(buf.len() - start);
                        buf[start..start + n].copy_from_slice(&data[..n]);
                        // Consume only what we stored; anything past our
                        // buffer stays queued (and, for a well-formed GET,
                        // there is nothing past the headers we care about).
                        (n, n)
                    })
                    .unwrap_or(0);
                req_lens[i] = start + n;
            }

            let received = &req_bufs[i][..req_lens[i]];
            if find_subslice(received, b"\r\n\r\n").is_some() {
                // Full request headers in hand -- respond once we can, then
                // close (`Connection: close`), leaving the socket to reset
                // to idle and listen again.
                if socket.can_send() {
                    log_request_line(uart, received);
                    match socket.send_slice(response) {
                        Ok(_) => {}
                        Err(e) => {
                            let _ = writeln!(uart, "send failed: {e:?}");
                        }
                    }
                    socket.close();
                    responded[i] = true;
                }
            } else if req_lens[i] == req_bufs[i].len() {
                // Header buffer full without an end-of-headers marker: give
                // up on this connection rather than stall forever.
                let _ = writeln!(uart, "request headers too large; dropping");
                socket.abort();
                responded[i] = true;
            }
        }

        // Pace the loop -- an idle poll is a cheap register read (the phy
        // skips the bulk-IN when the chip's RX FIFO is empty), so 1ms is
        // plenty responsive without hammering the bus.
        timer.delay_ms(1);
    }
}

/// Logs the request line (the first CRLF-terminated line) of `request` to
/// the UART, e.g. `request: GET / HTTP/1.1`.
fn log_request_line(uart: &mut Uart, request: &[u8]) {
    let line_end = find_subslice(request, b"\r\n").unwrap_or(request.len());
    match core::str::from_utf8(&request[..line_end]) {
        Ok(line) => {
            let _ = writeln!(uart, "request: {line}");
        }
        Err(_) => {
            let _ = writeln!(uart, "request: <non-utf8 request line>");
        }
    }
}

/// Returns the index of the first occurrence of `needle` in `haystack`, or
/// `None`.
fn find_subslice(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    if needle.is_empty() || haystack.len() < needle.len() {
        return None;
    }
    haystack.windows(needle.len()).position(|w| w == needle)
}

/// The current time as a smoltcp [`Instant`], from the system timer.
fn instant(timer: &Timer) -> Instant {
    Instant::from_micros(timer.now_micros() as i64)
}

/// A `core::fmt::Write` sink that formats into a fixed byte slice, used to
/// build the response without a heap. Formatting past the end of the slice
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

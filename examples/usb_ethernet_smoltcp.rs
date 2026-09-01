#![no_std]
#![no_main]

// A TCP/IP stack over the on-board Ethernet, using `smoltcp`.
//
// Bring-up is the same as `usb_ethernet`: power the USB controller on,
// start DWC2, enumerate the bus, and bring the LAN9514 up (program the
// firmware MAC, enable RX/TX, wait for link). From there this example
// wraps the LAN9514 in a `smoltcp` `phy::Device` -- the adapter that lets
// the stack move Ethernet frames through `Lan9514::send_frame` /
// `receive_frame` -- gets an address over DHCP, and runs the poll loop.
//
// With `auto-icmp-echo-reply` on, the interface answers pings on its own.
// A UDP socket on top runs a line-level echo server on port 7 (the
// conventional Echo Protocol port, RFC 862) -- send it a datagram (e.g.
// `nc -u <address the board printed> 7`) and it sends the same bytes
// back. Between the two, this exercises the whole path -- ARP, the phy
// adapter, the DHCP exchange, ICMP, and a real UDP socket -- end to end.
//
// The `phy::Device` adapter is `rpi_hal::usb::lan9514::Lan9514Phy`, behind
// the crate's `smoltcp` feature. The stack itself (IP config, sockets,
// poll loop) is application policy and stays here.
//
// Pi 2/3 only. Both the USB hub and the Ethernet are one soldered-on
// LAN9514 there, behind the DWC2 controller. A Pi 4 has neither half: its
// USB host is a VL805 xHCI behind PCIe and its Ethernet is a native GENET
// MAC on RGMII pins, neither of which this crate drives yet -- so this
// builds for `bcm2711` and then finds an empty root port.

use core::fmt::Write;
use core::ops::ControlFlow;
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::usb;
use rpi_hal::usb::dwc2::{Channel, Dwc2Host};
use rpi_hal::usb::lan9514::{Lan9514, Lan9514Phy};
use smoltcp::iface::{Config, Interface, SocketSet, SocketStorage};
use smoltcp::socket::{dhcpv4, udp};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, HardwareAddress, IpCidr};

/// UDP port the echo server listens on -- the Echo Protocol's conventional
/// port (RFC 862).
const ECHO_PORT: u16 = 7;
/// Payload capacity of the echo socket's RX/TX buffers, in bytes -- more
/// than enough for a single UDP datagram exercising the demo.
const ECHO_BUFFER_SIZE: usize = 1024;
/// How many datagrams the echo socket can queue in each direction before
/// a sender must wait for the previous one to be read/sent.
const ECHO_QUEUE_DEPTH: usize = 4;

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

    let dwc2 = Dwc2Host::init(
        peripherals.USB_OTG_GLOBAL,
        peripherals.USB_OTG_HOST,
        peripherals.USB_OTG_PWRCLK,
        &timer,
    );

    // Bounded, because the hub is soldered on: a root port that hasn't
    // reported in five seconds has nothing behind it at all, which is
    // what a Pi 4 looks like (see the header). Falling through rather
    // than halting here -- `usb::enumerate` reports that as
    // `EnumerationError::NotConnected` through the same error path
    // everything else goes through, where an unbounded wait would sit
    // here silently and look like a lock-up.
    let _ = writeln!(uart, "waiting for the on-board hub...");
    let deadline_us = timer.now_micros() + 5_000_000;
    while !dwc2.port_connected() && timer.now_micros() < deadline_us {
        timer.delay_ms(100);
    }

    // Enumerate the bus; when the LAN9514 turns up, run the stack on it.
    let result = usb::enumerate(&dwc2, &timer, |channel, timer, device| {
        let lan9514 = match Lan9514::from_device(channel, timer, device) {
            Ok(Some(lan9514)) => lan9514,
            Ok(None) => return ControlFlow::Continue(()),
            Err(e) => {
                let _ = writeln!(uart, "LAN9514 setup failed: {e:?}");
                return ControlFlow::Break(());
            }
        };
        // The stack needs a channel of its own: `channel` is
        // enumeration's and goes away when the callback returns, while
        // the `phy::Device` below keeps moving frames forever.
        let Some(net_channel) = dwc2.alloc_channel() else {
            let _ = writeln!(uart, "no free host channel for the stack");
            return ControlFlow::Break(());
        };
        run_stack(&mut uart, net_channel, timer, lan9514, board_mac);
        ControlFlow::Break(())
    });

    if let Err(e) = result {
        let _ = writeln!(uart, "enumeration failed: {e:?}");
    }
    halt();
}

/// Brings the LAN9514 up, wraps it in a smoltcp `phy::Device`, gets an
/// address over DHCP, and runs the poll loop forever, echoing back any
/// UDP datagram sent to [`ECHO_PORT`].
fn run_stack(
    uart: &mut Uart,
    mut channel: Channel,
    timer: &Timer,
    mut lan9514: Lan9514,
    mac: [u8; 6],
) {
    if let Err(e) = lan9514.start(&mut channel, timer, mac) {
        let _ = writeln!(uart, "LAN9514 start failed: {e:?}");
        return;
    }

    let _ = writeln!(uart, "waiting for link...");
    loop {
        match lan9514.is_link_up(&mut channel, timer) {
            Ok(true) => break,
            Ok(false) => timer.delay_ms(100),
            Err(e) => {
                let _ = writeln!(uart, "link check failed: {e:?}");
                return;
            }
        }
    }
    let _ = writeln!(uart, "link up");

    let mut phy = Lan9514Phy::new(lan9514, channel, timer);

    // Configure the interface with the board MAC and a fresh clock seed.
    // No IP is assigned here -- DHCP fills that in once it gets a lease.
    let mut config = Config::new(HardwareAddress::Ethernet(EthernetAddress(mac)));
    config.random_seed = timer.now_micros();
    let mut iface = Interface::new(config, &mut phy, instant(timer));

    // A DHCP client socket drives the address lease; the interface answers
    // pings on its own once an address is assigned (`auto-icmp-echo-reply`).
    let mut socket_storage: [SocketStorage; 2] = [SocketStorage::EMPTY; 2];
    let mut sockets = SocketSet::new(&mut socket_storage[..]);
    let dhcp_handle = sockets.add(dhcpv4::Socket::new());

    // The echo server: bound up front since a fresh socket can only fail
    // to bind on port 0, and `ECHO_PORT` isn't that.
    let mut echo_rx_metadata = [udp::PacketMetadata::EMPTY; ECHO_QUEUE_DEPTH];
    let mut echo_rx_payload = [0u8; ECHO_BUFFER_SIZE];
    let mut echo_tx_metadata = [udp::PacketMetadata::EMPTY; ECHO_QUEUE_DEPTH];
    let mut echo_tx_payload = [0u8; ECHO_BUFFER_SIZE];
    let mut echo_socket = udp::Socket::new(
        udp::PacketBuffer::new(&mut echo_rx_metadata[..], &mut echo_rx_payload[..]),
        udp::PacketBuffer::new(&mut echo_tx_metadata[..], &mut echo_tx_payload[..]),
    );
    let _ = echo_socket.bind(ECHO_PORT);
    let echo_handle = sockets.add(echo_socket);
    let mut echo_buffer = [0u8; ECHO_BUFFER_SIZE];

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
                for dns in dhcp.dns_servers.iter() {
                    let _ = writeln!(uart, "DHCP: dns {dns}");
                }
                let _ = writeln!(uart, "ready -- try `ping {}`", dhcp.address.address());
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

        // Echo back every datagram sent to `ECHO_PORT`, one at a time --
        // `recv_slice` returning `Exhausted` (nothing left queued) ends
        // the loop each poll.
        let echo_socket = sockets.get_mut::<udp::Socket>(echo_handle);
        while let Ok((len, meta)) = echo_socket.recv_slice(&mut echo_buffer) {
            let _ = writeln!(uart, "echo: {len} bytes from {}", meta.endpoint);
            let _ = echo_socket.send_slice(&echo_buffer[..len], meta);
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

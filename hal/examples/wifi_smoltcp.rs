#![no_std]
#![no_main]

// A TCP/IP stack over Wi-Fi (Pi 3 only): reads the BCM43430 firmware and
// network credentials off the SD card's FAT boot partition, downloads the
// firmware into the wireless chip and starts its CPU, loads the
// regulatory blob, joins the WPA2-PSK network in WIFI.CFG, then wraps the
// chip in a `smoltcp` `phy::Device`, gets an address over DHCP, and runs
// the poll loop.
//
// With `auto-icmp-echo-reply` on, the interface answers pings on its own.
// A UDP socket on top runs a line-level echo server on port 7 (RFC 862) --
// send it a datagram (e.g. `nc -u <address the board printed> 7`) and it
// sends the same bytes back. Between the two, this exercises the whole
// data path -- the SDPCM/BDC framing, the phy adapter, DHCP, ARP, ICMP,
// and a real UDP socket -- end to end.
//
// See `wifi_scan.rs` for scanning without joining.
//
// Because the SD card and the Wi-Fi chip share the one EMMC controller,
// the files are read into RAM *first* (over the SD driver), and only
// then is the controller handed to the SDIO/Wi-Fi driver -- driving
// Wi-Fi gives up the SD slot.
//
// The blobs go in a `wifi` directory on the boot partition, under a
// subdirectory named for the radio and 8.3 filenames -- so one card boots
// any Pi, and the chip picks its own set. A 3B/Zero W wants `wifi/43430`:
//
//   FW.BIN    -- Broadcom's brcmfmac43430-sdio.bin
//   NVRAM.TXT -- the matching nvram (brcmfmac43430-sdio.txt)
//   CLM.DAT   -- the CLM regulatory blob (cyfmac43430-sdio.clm_blob)
//
// and a 3B+/Pi 4 `wifi/43455`, with the 43455 files of the same names --
// including the board-specific nvram
// (brcmfmac43455-sdio.raspberrypi,3-model-b-plus.txt for a 3B+).
//
// The credentials are not per-radio and live at the root instead:
//
//   WIFI.CFG  -- two lines: the SSID, then the WPA2 passphrase

use core::fmt::Write;
use core::ptr::{addr_of, addr_of_mut};
use embedded_sdmmc::{Mode, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::sd::{Sd, SdCard, SdCardError};
use rpi_hal::sdio::Sdio;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::wifi::{Wifi, WifiPhy};
use smoltcp::iface::{Config, Interface, SocketSet, SocketStorage};
use smoltcp::socket::{dhcpv4, udp};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, HardwareAddress, IpCidr};

/// UDP port the echo server listens on -- the Echo Protocol's conventional
/// port (RFC 862).
const ECHO_PORT: u16 = 7;
/// Payload capacity of the echo socket's RX/TX buffers, in bytes.
const ECHO_BUFFER_SIZE: usize = 1024;
/// How many datagrams the echo socket can queue in each direction.
const ECHO_QUEUE_DEPTH: usize = 4;

/// Directory on the FAT boot partition holding the firmware files.
const WIFI_DIR: &str = "WIFI";
/// Firmware image, within a [`WIFI_DIR`] subdirectory (8.3 name).
const FIRMWARE_FILE: &str = "FW.BIN";
/// Raw nvram config, within a [`WIFI_DIR`] subdirectory (8.3 name).
const NVRAM_FILE: &str = "NVRAM.TXT";
/// CLM (regulatory) blob, within a [`WIFI_DIR`] subdirectory (8.3 name).
const CLM_FILE: &str = "CLM.DAT";
/// Network credentials: the SSID on the first line and the WPA2
/// passphrase on the second (8.3 name).
///
/// At the **root** of the boot partition, not under [`WIFI_DIR`]. The
/// files there are vendor blobs, copied once and never read by a person,
/// and they are per-radio; this is the one file here somebody writes, and
/// the network a board joins does not change when the card moves to a
/// board with different silicon. It sits beside `config.txt` with the
/// board's other settings.
const CONFIG_FILE: &str = "WIFI.CFG";

/// Which subdirectory of [`WIFI_DIR`] this board's blobs are in, and the
/// chip id they are for. `None` for a board with no radio this drives.
///
/// A directory per radio rather than one set of files, so that one card
/// boots any Pi: a 3B and a 3B+ carry different silicon and each refuses
/// the other's image, and swapping three files by hand every time the
/// card moves is the sort of step that gets forgotten once and then
/// debugged for an hour. The names are the part numbers the firmware
/// files are published under, not the chip ids — a directory somebody
/// has to copy files into should be named the way the files are.
///
/// # Why the board and not the chip
///
/// Asking the radio what it is would need no table and never go stale,
/// and it does not work. The chip id is only readable over the
/// backplane, the backplane only once the one EMMC controller has been
/// muxed off the card, and the card is where the firmware is — so it
/// would mean bringing SDIO up, asking, reading the card, and bringing
/// SDIO up a second time. The radio does not answer `CMD5` on that
/// second pass: re-asserting an already-high `WL_ON` is not the power
/// cycle it needs to enumerate again.
///
/// So this guesses from the board and the caller *verifies* against the
/// chip id once SDIO is up — before any firmware is written. A wrong
/// entry below is then one clear line naming both numbers rather than a
/// download that fails several steps later for no visible reason.
fn radio(board_revision: u32) -> Option<(&'static str, u32)> {
    // Old-style revision codes are Pi 1s and have no radio at all. Worth
    // rejecting rather than shifting: the fields below do not exist in
    // them, so the bits would decode to a board at random.
    if board_revision & (1 << 23) == 0 {
        return None;
    }
    // Bits 4..11 of a new-style code are the board type.
    match (board_revision >> 4) & 0xff {
        // 3B, Zero W.
        0x08 | 0x0c => Some(("43430", rpi_hal::sdio::BCM43438_CHIP_ID)),
        // 3B+, 3A+, 4B.
        0x0d | 0x0e | 0x11 => Some(("43455", rpi_hal::sdio::BCM43455_CHIP_ID)),
        _ => None,
    }
}

/// Buffer for the firmware image; zeroed BSS.
///
/// Sized for the largest image this drives rather than for the 43430's
/// ~400KB, because a file that does not fit is read as far as the buffer
/// goes and its truncated length reported — so an image half a megabyte
/// short downloads, starts, and simply never answers, with nothing
/// reported anywhere. A 43455's is over 600KB, so a megabyte is the size
/// that keeps this from being a trap.
static mut FW_BUF: [u8; 1024 * 1024] = [0; 1024 * 1024];
/// Buffer for the raw nvram text.
static mut NV_BUF: [u8; 4096] = [0; 4096];
/// Buffer for the CLM regulatory blob (~5KB).
static mut CLM_BUF: [u8; 8192] = [0; 8192];
/// Buffer for the network-credentials file.
static mut CFG_BUF: [u8; 256] = [0; 256];

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// A fixed timestamp for `embedded-sdmmc` (only used for file mtimes on
/// writes, which this read-only path never does).
struct FixedTime;

impl TimeSource for FixedTime {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 56,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

/// Mounts the boot partition and reads the firmware and nvram files into
/// the static buffers, returning their lengths. Consumes the SD driver
/// (and with it the EMMC controller), which the caller reclaims for
/// Wi-Fi once this returns.
fn load_files(
    sd: Sd,
    subdir: &str,
    timer: &Timer,
) -> Result<(usize, usize, usize, usize), embedded_sdmmc::Error<SdCardError>> {
    let volume_mgr = VolumeManager::new(SdCard::new(sd, timer), FixedTime);
    let volume = volume_mgr.open_volume(VolumeIdx(0))?;
    let root = volume.open_root_dir()?;
    // Two bindings rather than one chained expression: the intermediate
    // `Directory` borrows the volume manager, so a temporary would be
    // dropped at the end of the statement while `wifi` still holds it.
    let wifi_root = root.open_dir(WIFI_DIR)?;
    let wifi = wifi_root.open_dir(subdir)?;

    // Safety: single-threaded bare-metal; these buffers are touched only
    // here and, after this returns, read-only in `kmain`.
    let fw_len = read_file(&wifi, FIRMWARE_FILE, unsafe { &mut *addr_of_mut!(FW_BUF) })?;
    let nv_len = read_file(&wifi, NVRAM_FILE, unsafe { &mut *addr_of_mut!(NV_BUF) })?;
    let clm_len = read_file(&wifi, CLM_FILE, unsafe { &mut *addr_of_mut!(CLM_BUF) })?;
    let cfg_len = read_file(&root, CONFIG_FILE, unsafe { &mut *addr_of_mut!(CFG_BUF) })?;
    Ok((fw_len, nv_len, clm_len, cfg_len))
}

/// Splits the credentials file (SSID on line 1, passphrase on line 2)
/// into `(ssid, passphrase)`, trimming end-of-line whitespace. Returns
/// `None` if the two lines aren't both present and valid UTF-8.
fn parse_config(bytes: &[u8]) -> Option<(&str, &str)> {
    let text = core::str::from_utf8(bytes).ok()?;
    let mut lines = text.lines();
    let ssid = lines.next()?.trim_end();
    let passphrase = lines.next()?.trim_end();
    if ssid.is_empty() || passphrase.is_empty() {
        return None;
    }
    Some((ssid, passphrase))
}

/// Reads the whole of `name` into `buf`, returning the byte count (or
/// `buf.len()` if the file is larger).
fn read_file<D, T, const A: usize, const B: usize, const C: usize>(
    dir: &embedded_sdmmc::Directory<D, T, A, B, C>,
    name: &str,
    buf: &mut [u8],
) -> Result<usize, embedded_sdmmc::Error<D::Error>>
where
    D: embedded_sdmmc::BlockDevice,
    T: TimeSource,
{
    let file = dir.open_file_in_dir(name, Mode::ReadOnly)?;
    let mut total = 0;
    while !file.is_eof() && total < buf.len() {
        let n = file.read(&mut buf[total..])?;
        if n == 0 {
            break;
        }
        total += n;
    }
    Ok(total)
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // Which firmware this board needs, from the revision code the
    // firmware mailbox reports -- which costs nothing and, unlike asking
    // the radio, does not need the card given up first. Checked against
    // the chip id further down, once SDIO is up and before anything is
    // written.
    let board_revision = match mailbox.board_revision() {
        Ok(revision) => revision,
        Err(e) => {
            let _ = writeln!(uart, "board revision read failed: {e:?}");
            halt();
        }
    };
    let Some((subdir, expected_chip_id)) = radio(board_revision) else {
        let _ = writeln!(
            uart,
            "board revision {board_revision:#010x} has no radio this drives"
        );
        halt();
    };
    let _ = writeln!(
        uart,
        "board revision {board_revision:#010x} -> {WIFI_DIR}/{subdir}/"
    );

    // Read the firmware + nvram off the SD card first (this owns EMMC).
    let _ = writeln!(uart, "reading firmware from SD card...");
    let sd = match Sd::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, &timer) {
        Ok(sd) => sd,
        Err(e) => {
            let _ = writeln!(uart, "SD init failed: {e:?}");
            halt();
        }
    };
    let (fw_len, nv_len, clm_len, cfg_len) = match load_files(sd, subdir, &timer) {
        Ok(lengths) => lengths,
        Err(e) => {
            let _ = writeln!(uart, "reading Wi-Fi files failed: {e:?}");
            halt();
        }
    };
    let _ = writeln!(
        uart,
        "  {FIRMWARE_FILE}: {fw_len} bytes, {NVRAM_FILE}: {nv_len} bytes, {CLM_FILE}: {clm_len} bytes"
    );

    // Reclaim the EMMC controller for Wi-Fi (the SD driver is dropped, so
    // the slot is now free to be re-muxed to the wireless pins).
    let peripherals = unsafe { pac::Peripherals::steal() };
    let _ = writeln!(uart, "bringing up Wi-Fi chip over SDIO...");
    let mut sdio = match Sdio::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, &timer) {
        Ok(sdio) => sdio,
        Err(e) => {
            let _ = writeln!(uart, "SDIO init failed: {e:?}");
            halt();
        }
    };
    // The check on the guess `radio` made from the board revision, and a
    // bus liveness check besides. Halting on a mismatch rather than
    // warning: the blobs in hand are for another chip, and the download
    // would either fail obscurely or -- worse -- appear to work.
    match sdio.chip_id(&timer) {
        Ok(id) if id == expected_chip_id => {}
        Ok(id) => {
            let _ = writeln!(
                uart,
                "chip id {id:#06x}, but board revision {board_revision:#010x} \
                 said to load {WIFI_DIR}/{subdir}/ (for {expected_chip_id:#06x}) \
                 -- the board table in `radio` is wrong for this Pi"
            );
            halt();
        }
        Err(e) => {
            let _ = writeln!(uart, "chip id read failed: {e:?}");
            halt();
        }
    }
    let _ = writeln!(uart, "SDIO link up; downloading firmware...");

    // Safety: `load_files` has finished writing these; read-only now.
    let firmware = &unsafe { &*addr_of!(FW_BUF) }[..fw_len];
    let nvram = &unsafe { &*addr_of!(NV_BUF) }[..nv_len];
    if let Err(e) = sdio.load_firmware(firmware, nvram, &timer) {
        let _ = writeln!(uart, "firmware load failed: {e:?}");
        halt();
    }
    let _ = writeln!(uart, "firmware running: WLAN function ready");

    // Talk to the running firmware over SDPCM/CDC: read its version
    // string and MAC address -- proof the control path round-trips.
    let mut wifi = match Wifi::new(sdio, &timer) {
        Ok(wifi) => wifi,
        Err(e) => {
            let _ = writeln!(uart, "wifi protocol init failed: {e:?}");
            halt();
        }
    };

    let mut version = [0u8; 128];
    match wifi.get_iovar("ver", &mut version, &timer) {
        Ok(n) => {
            // The version is an ASCII string, NUL-terminated within n.
            let end = version[..n].iter().position(|&b| b == 0).unwrap_or(n);
            let _ = write!(uart, "firmware version: ");
            for &b in &version[..end] {
                let c = if (0x20..=0x7e).contains(&b) {
                    b as char
                } else {
                    '.'
                };
                let _ = uart.write_char(c);
            }
            let _ = writeln!(uart);
        }
        Err(e) => {
            let _ = writeln!(uart, "get 'ver' failed: {e:?}");
        }
    }

    let mut mac = [0u8; 6];
    match wifi.get_iovar("cur_etheraddr", &mut mac, &timer) {
        Ok(6) => {
            let _ = writeln!(
                uart,
                "MAC address: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
            );
        }
        Ok(n) => {
            let _ = writeln!(uart, "MAC address: unexpected length {n}");
        }
        Err(e) => {
            let _ = writeln!(uart, "get 'cur_etheraddr' failed: {e:?}");
        }
    }

    // Load the CLM regulatory blob -- the Cypress firmware needs it
    // before the radio can scan or join.
    let clm = &unsafe { &*addr_of!(CLM_BUF) }[..clm_len];
    match wifi.load_clm(clm, &timer) {
        Ok(()) => {
            let _ = writeln!(uart, "CLM loaded ({clm_len} bytes)");
        }
        Err(e) => {
            let _ = writeln!(uart, "CLM load failed: {e:?}");
        }
    }
    let mut country = [0u8; 12];
    if let Ok(n) = wifi.get_iovar("country", &mut country, &timer) {
        let _ = writeln!(uart, "country = {:02x?}", &country[..n.min(12)]);
    }

    // Join the WPA2 network named in wifi/WIFI.CFG.
    let config = &unsafe { &*addr_of!(CFG_BUF) }[..cfg_len];
    let Some((ssid, passphrase)) = parse_config(config) else {
        let _ = writeln!(uart, "{WIFI_DIR}/{CONFIG_FILE} missing/invalid; can't join");
        halt();
    };
    let _ = writeln!(uart, "joining network {ssid:?}...");
    match wifi.join_wpa2(ssid, passphrase, &timer) {
        Ok(bssid) => {
            let _ = writeln!(
                uart,
                "associated! BSSID {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]
            );
        }
        Err(e) => {
            let _ = writeln!(uart, "join failed: {e:?}");
            halt();
        }
    }

    // Run the TCP/IP stack over the data channel.
    run_stack(&mut uart, wifi, &timer, mac);
    halt();
}

/// Wraps the joined [`Wifi`] in a smoltcp `phy::Device`, gets an address
/// over DHCP, and runs the poll loop forever, echoing back any UDP
/// datagram sent to [`ECHO_PORT`].
fn run_stack(uart: &mut Uart, wifi: Wifi, timer: &Timer, mac: [u8; 6]) {
    let mut phy = WifiPhy::new(wifi, timer);

    // Configure the interface with the chip's MAC and a fresh clock seed.
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

        // Echo back every datagram sent to `ECHO_PORT`, one at a time.
        let echo_socket = sockets.get_mut::<udp::Socket>(echo_handle);
        while let Ok((len, meta)) = echo_socket.recv_slice(&mut echo_buffer) {
            let _ = writeln!(uart, "echo: {len} bytes from {}", meta.endpoint);
            let _ = echo_socket.send_slice(&echo_buffer[..len], meta);
        }

        timer.delay_ms(1);
    }
}

/// The current time as a smoltcp [`Instant`], from the system timer.
fn instant(timer: &Timer) -> Instant {
    Instant::from_micros(timer.now_micros() as i64)
}

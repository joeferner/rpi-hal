#![no_std]
#![no_main]

// A TCP/IP stack over the on-board Wi-Fi, driven by `embassy-net` — the
// async counterpart to rpi-hal's `wifi_smoltcp.rs`, and the thing this
// crate's `wifi` adapter exists for.
//
// Bring-up is identical to that example, and blocking: read the BCM43430
// firmware and the network credentials off the card, download the firmware
// into the wireless chip, load the regulatory blob, join the WPA2-PSK
// network in WIFI.CFG. From there this hands the joined chip to
// `embassy-net` instead of running a `smoltcp` poll loop by hand.
//
// Three tasks:
//
// - `net_task` runs `embassy-net`'s own runner, which owns the stack and
//   serves it from the adapter's queues.
// - `wifi_task` runs the adapter's runner, which moves frames between those
//   queues and the radio. Unlike the LAN9514 adapter's, it polls — see that
//   module's documentation — so there is nothing to dispatch for it in
//   `__irq_handler` below.
// - `echo_task` waits for DHCP, prints the lease, then serves TCP echo on
//   port 7 (RFC 862). Try `ping <address>` and `nc <address> 7`.
//
// Because the SD card and the Wi-Fi chip share the one EMMC controller on
// these boards, the files are read into RAM *first* (over the SD driver),
// and only then is the controller handed to the SDIO/Wi-Fi driver —
// driving Wi-Fi gives up the SD slot. An application that needs both at
// once puts the card on the *other* controller instead, which is what
// rpi-hal's `sdhost` module is for.
//
// In a `wifi` directory on the boot partition, under 8.3 names:
//   FW.BIN    -- Broadcom's brcmfmac43430-sdio.bin
//   NVRAM.TXT -- the matching nvram (brcmfmac43430-sdio.txt)
//   CLM.DAT   -- the CLM regulatory blob (cyfmac43430-sdio.clm_blob)
//   WIFI.CFG  -- two lines: the SSID, then the WPA2 passphrase

use core::fmt::Write as _;
use core::ptr::{addr_of, addr_of_mut};

use embassy_net::tcp::TcpSocket;
use embassy_net::{Config, StackResources};
use embedded_sdmmc::{Mode, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use rpi_hal::mailbox::Mailbox;
use rpi_hal::rng::Rng;
use rpi_hal::sd::{Sd, SdCard, SdCardError};
use rpi_hal::sdio::Sdio;
use rpi_hal::wifi::{PowerManagement, Wifi};
use rpi_hal::{halt, irq, lic::Lic, pac, timer::Timer, uart::Uart};
use rpi_hal_embassy::wifi::{Reconnect, WifiDriver, WifiRunner, WifiState};
use rpi_hal_embassy::{Executor, time_driver};

/// Frames the adapter may hold queued inbound. Four absorbs a small burst
/// without the receive loop having to wait on the stack; each costs one
/// MTU of RAM.
const RX_QUEUE: usize = 4;

/// The outbound counterpart to [`RX_QUEUE`].
const TX_QUEUE: usize = 4;

/// TCP port the echo server listens on — the Echo Protocol's conventional
/// port (RFC 862).
const ECHO_PORT: u16 = 7;

/// Sockets the stack may hold at once.
const SOCKETS: usize = 2;

/// Directory on the FAT boot partition holding the firmware files.
const WIFI_DIR: &str = "WIFI";
/// Firmware image, within [`WIFI_DIR`] (8.3 name).
const FIRMWARE_FILE: &str = "FW.BIN";
/// Raw nvram config, within [`WIFI_DIR`] (8.3 name).
const NVRAM_FILE: &str = "NVRAM.TXT";
/// CLM (regulatory) blob, within [`WIFI_DIR`] (8.3 name).
const CLM_FILE: &str = "CLM.DAT";
/// Network credentials, within [`WIFI_DIR`] (8.3 name): the SSID on the
/// first line and the WPA2 passphrase on the second.
const CONFIG_FILE: &str = "WIFI.CFG";

/// Buffer for the firmware image (the 43430's is ~420KB); zeroed BSS.
static mut FW_BUF: [u8; 512 * 1024] = [0; 512 * 1024];
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

/// Widens a borrow to `'static`. Sound only where the caller never
/// returns, which is the case for every use below.
unsafe fn make_static<T>(t: &mut T) -> &'static mut T {
    unsafe { core::mem::transmute(t) }
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

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, WifiDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn wifi_task(runner: WifiRunner<'static>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn echo_task(stack: embassy_net::Stack<'static>, mut uart: Uart) {
    let _ = writeln!(uart, "waiting for DHCP...");
    stack.wait_config_up().await;

    if let Some(config) = stack.config_v4() {
        let _ = writeln!(uart, "DHCP: {}", config.address);
        if let Some(gateway) = config.gateway {
            let _ = writeln!(uart, "gateway: {gateway}");
        }
    }
    let _ = writeln!(uart, "echo server on port {ECHO_PORT}");

    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 1024];
    let mut buf = [0u8; 256];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

        if socket.accept(ECHO_PORT).await.is_err() {
            continue;
        }
        let _ = writeln!(uart, "connected");

        loop {
            match socket.read(&mut buf).await {
                // A zero-length read is the peer closing, not an idle
                // connection: the await only returns once there is
                // something to report.
                Ok(0) | Err(_) => break,
                Ok(n) => {
                    if socket.write(&buf[..n]).await.is_err() {
                        break;
                    }
                }
            }
        }

        // Discard rather than close: `abort` frees the socket without
        // waiting out TIME_WAIT, so the next connection can be accepted
        // straight away.
        socket.abort();
        let _ = writeln!(uart, "closed");
    }
}

/// Mounts the boot partition and reads the firmware, nvram, CLM blob and
/// credentials into the static buffers, returning their lengths. Consumes
/// the SD driver (and with it the EMMC controller), which the caller
/// reclaims for Wi-Fi once this returns.
fn load_files(
    sd: Sd,
    timer: &Timer,
) -> Result<(usize, usize, usize, usize), embedded_sdmmc::Error<SdCardError>> {
    let volume_mgr = VolumeManager::new(SdCard::new(sd, timer), FixedTime);
    let volume = volume_mgr.open_volume(VolumeIdx(0))?;
    let root = volume.open_root_dir()?;
    let wifi = root.open_dir(WIFI_DIR)?;

    // Safety: single-threaded bare-metal; these buffers are touched only
    // here and, after this returns, read-only in `kmain`.
    let fw_len = read_file(&wifi, FIRMWARE_FILE, unsafe { &mut *addr_of_mut!(FW_BUF) })?;
    let nv_len = read_file(&wifi, NVRAM_FILE, unsafe { &mut *addr_of_mut!(NV_BUF) })?;
    let clm_len = read_file(&wifi, CLM_FILE, unsafe { &mut *addr_of_mut!(CLM_BUF) })?;
    let cfg_len = read_file(&wifi, CONFIG_FILE, unsafe { &mut *addr_of_mut!(CFG_BUF) })?;
    Ok((fw_len, nv_len, clm_len, cfg_len))
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

// `unsafe(no_mangle)` rather than the bare `no_mangle` rpi-hal's own
// examples use: this crate is edition 2024, which requires the wrapper.
#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "embassy-net over the on-board Wi-Fi");

    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // Read the firmware + nvram off the SD card first (this owns EMMC).
    let _ = writeln!(uart, "reading firmware from SD card...");
    let sd = match Sd::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, &timer) {
        Ok(sd) => sd,
        Err(e) => {
            let _ = writeln!(uart, "SD init failed: {e:?}");
            halt();
        }
    };
    let (fw_len, nv_len, clm_len, cfg_len) = match load_files(sd, &timer) {
        Ok(lengths) => lengths,
        Err(e) => {
            let _ = writeln!(uart, "reading Wi-Fi files failed: {e:?}");
            halt();
        }
    };

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

    // Safety: `load_files` has finished writing these; read-only now.
    let firmware = &unsafe { &*addr_of!(FW_BUF) }[..fw_len];
    let nvram = &unsafe { &*addr_of!(NV_BUF) }[..nv_len];
    if let Err(e) = sdio.load_firmware(firmware, nvram, &timer) {
        let _ = writeln!(uart, "firmware load failed: {e:?}");
        halt();
    }
    let _ = writeln!(uart, "firmware running: WLAN function ready");

    let mut wifi = match Wifi::new(sdio, &timer) {
        Ok(wifi) => wifi,
        Err(e) => {
            let _ = writeln!(uart, "wifi protocol init failed: {e:?}");
            halt();
        }
    };

    // `embassy-net` needs the chip's own address to answer ARP, and the
    // adapter does not read it back — so it is read here and passed in.
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
            halt();
        }
        Err(e) => {
            let _ = writeln!(uart, "get 'cur_etheraddr' failed: {e:?}");
            halt();
        }
    }

    // The Cypress firmware needs the regulatory blob before the radio can
    // scan or join.
    let clm = &unsafe { &*addr_of!(CLM_BUF) }[..clm_len];
    if let Err(e) = wifi.load_clm(clm, &timer) {
        let _ = writeln!(uart, "CLM load failed: {e:?}");
        halt();
    }
    let _ = writeln!(uart, "CLM loaded ({clm_len} bytes)");

    // Join the WPA2 network named in wifi/WIFI.CFG. The credentials are
    // borrowed from a `static` and named `'static` here on purpose: the
    // runner is handed them below and keeps them for the life of the
    // program, so that it can rejoin this network on its own.
    let config: &'static [u8] = &unsafe { &*addr_of!(CFG_BUF) }[..cfg_len];
    let Some((ssid, passphrase)) = parse_config(config) else {
        let _ = writeln!(uart, "{WIFI_DIR}/{CONFIG_FILE} missing/invalid; can't join");
        halt();
    };
    let _ = writeln!(uart, "joining network {ssid:?}...");
    if let Err(e) = wifi.join_wpa2(ssid, passphrase, &timer) {
        let _ = writeln!(uart, "join failed: {e:?}");
        halt();
    }
    let _ = writeln!(uart, "associated; handing the chip to embassy-net");
    let _ = writeln!(uart, "  ({nv_len} bytes nvram, {fw_len} bytes firmware)");

    run(uart, wifi, timer, mac, ssid, passphrase);
}

/// Services the interrupt the executor depends on.
///
/// Mandatory, and silently fatal to omit: `rpi-hal` provides only a *weak*
/// no-op `__irq_handler`, so without this the first `embassy-time` deadline
/// fires, nothing acknowledges the Compare 1 match, the interrupt
/// controller keeps asserting, and the core re-enters the handler forever.
/// Every task stops making progress at that instant, which on the console
/// looks like a hang immediately after bring-up.
///
/// Only the one line, unlike the LAN9514 example: the Wi-Fi adapter's
/// runner drives its transfers to completion itself rather than awaiting an
/// interrupt.
#[unsafe(no_mangle)]
pub extern "C" fn __irq_handler() {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);

    if lic.is_timer1_pending() {
        time_driver::on_timer_irq();
    }
}

/// Hands the joined chip to `embassy-net` and starts the executor. Never
/// returns.
///
/// `ssid` and `passphrase` are the same credentials the join above used, and
/// are handed on so the runner can put the association back by itself — see
/// `Reconnect` below.
fn run(
    uart: Uart,
    wifi: Wifi,
    timer: Timer,
    mac: [u8; 6],
    ssid: &'static str,
    passphrase: &'static str,
) -> ! {
    // The time driver needs the System Timer too. Both handles refer to the
    // same free-running counter, and only the driver's touches Compare 1.
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);
    time_driver::init(Timer::new(peripherals.SYSTMR), &lic);
    irq::enable_irq();

    // Everything below outlives `run`, which never returns.
    let mut timer = timer;
    let timer: &'static Timer = unsafe { make_static(&mut timer) };

    let mut state = WifiState::<RX_QUEUE, TX_QUEUE>::new();
    let state = unsafe { make_static(&mut state) };
    let (driver, wifi_runner) = rpi_hal_embassy::wifi::new(state, wifi, timer, mac);
    // Without this the runner reports the association going away and waits
    // for it to come back, which on a board with no console and no way in is
    // a reboot. With it, an access point that reboots overnight costs a
    // minute of downtime and nothing else.
    //
    // `Fast` is what the chip powers on in, and this example never changed
    // it. A program that turned power save off before joining names that
    // here instead: the firmware resets the setting on every association, so
    // what is not named here is what the board silently loses on its first
    // rejoin.
    let wifi_runner = wifi_runner.reconnecting(Reconnect {
        ssid,
        passphrase,
        power_management: PowerManagement::Fast,
    });

    // A random seed keeps TCP initial sequence numbers and the DHCP
    // transaction ID from repeating across boots. The hardware RNG is right
    // here, so there is no reason to use a fixed one.
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
        spawner.spawn(wifi_task(wifi_runner).unwrap());
        spawner.spawn(echo_task(stack, uart).unwrap());
    });
}

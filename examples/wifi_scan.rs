#![no_std]
#![no_main]

// Wi-Fi scan (Pi 3 only): brings the on-board BCM43430 wireless chip up
// over SDIO, reads back its ChipCommon ID as a bus liveness check, then
// downloads the firmware, loads the regulatory blob, and scans for
// nearby access points -- printing each one's SSID, BSSID, channel, and
// signal strength. See `wifi_smoltcp.rs` for joining a network.
//
// Because the SD card and the Wi-Fi chip share the one EMMC controller,
// the firmware blobs are read into RAM *first* (over the SD driver), and
// only then is the controller handed to the SDIO/Wi-Fi driver -- driving
// Wi-Fi gives up the SD slot.
//
// In a `wifi` directory on the boot partition, under 8.3 names:
//   FW.BIN    -- Broadcom's brcmfmac43430-sdio.bin
//   NVRAM.TXT -- the matching nvram (brcmfmac43430-sdio.txt)
//   CLM.DAT   -- the CLM regulatory blob (cyfmac43430-sdio.clm_blob)

use core::fmt::Write;
use core::ptr::{addr_of, addr_of_mut};
use embedded_sdmmc::{Mode, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::sd::{Sd, SdCard, SdCardError};
use rpi_hal::sdio::{Sdio, BCM43438_CHIP_ID};
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::wifi::Wifi;

/// Directory on the FAT boot partition holding the firmware files.
const WIFI_DIR: &str = "WIFI";
/// Firmware image, within [`WIFI_DIR`] (8.3 name).
const FIRMWARE_FILE: &str = "FW.BIN";
/// Raw nvram config, within [`WIFI_DIR`] (8.3 name).
const NVRAM_FILE: &str = "NVRAM.TXT";
/// CLM (regulatory) blob, within [`WIFI_DIR`] (8.3 name).
const CLM_FILE: &str = "CLM.DAT";

/// Buffer for the firmware image (the 43430's is ~420KB); zeroed BSS.
static mut FW_BUF: [u8; 512 * 1024] = [0; 512 * 1024];
/// Buffer for the raw nvram text.
static mut NV_BUF: [u8; 4096] = [0; 4096];
/// Buffer for the CLM regulatory blob (~5KB).
static mut CLM_BUF: [u8; 8192] = [0; 8192];

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

/// Mounts the boot partition and reads the firmware blobs into the static
/// buffers, returning their lengths. Consumes the SD driver (and with it
/// the EMMC controller), which the caller reclaims for Wi-Fi once this
/// returns.
fn load_files(
    sd: Sd,
    timer: &Timer,
) -> Result<(usize, usize, usize), embedded_sdmmc::Error<SdCardError>> {
    let volume_mgr = VolumeManager::new(SdCard::new(sd, timer), FixedTime);
    let volume = volume_mgr.open_volume(VolumeIdx(0))?;
    let root = volume.open_root_dir()?;
    let wifi = root.open_dir(WIFI_DIR)?;

    // Safety: single-threaded bare-metal; these buffers are touched only
    // here and, after this returns, read-only in `kmain`.
    let fw_len = read_file(&wifi, FIRMWARE_FILE, unsafe { &mut *addr_of_mut!(FW_BUF) })?;
    let nv_len = read_file(&wifi, NVRAM_FILE, unsafe { &mut *addr_of_mut!(NV_BUF) })?;
    let clm_len = read_file(&wifi, CLM_FILE, unsafe { &mut *addr_of_mut!(CLM_BUF) })?;
    Ok((fw_len, nv_len, clm_len))
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

    // Read the firmware blobs off the SD card first (this owns EMMC).
    let _ = writeln!(uart, "reading firmware from SD card...");
    let sd = match Sd::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, &timer) {
        Ok(sd) => sd,
        Err(e) => {
            let _ = writeln!(uart, "SD init failed: {e:?}");
            halt();
        }
    };
    let (fw_len, nv_len, clm_len) = match load_files(sd, &timer) {
        Ok(lengths) => lengths,
        Err(e) => {
            let _ = writeln!(uart, "reading Wi-Fi files failed: {e:?}");
            halt();
        }
    };
    let _ = writeln!(
        uart,
        "  {WIFI_DIR}/{FIRMWARE_FILE}: {fw_len} bytes, {WIFI_DIR}/{NVRAM_FILE}: {nv_len} bytes"
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

    // Bus liveness check: read the ChipCommon ID over the backplane before
    // loading firmware. 0xa9a6 is a Pi 3 B's BCM43438 answering.
    match sdio.chip_id(&timer) {
        Ok(id) if id == BCM43438_CHIP_ID => {
            let _ = writeln!(uart, "SDIO link up; chip id {id:#06x} (BCM43438)");
        }
        Ok(id) => {
            let _ = writeln!(uart, "SDIO link up; unexpected chip id {id:#06x}");
        }
        Err(e) => {
            let _ = writeln!(uart, "chip id read failed: {e:?}");
            halt();
        }
    }

    let _ = writeln!(uart, "downloading firmware...");
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
    // before the radio can scan.
    let clm = &unsafe { &*addr_of!(CLM_BUF) }[..clm_len];
    match wifi.load_clm(clm, &timer) {
        Ok(()) => {
            let _ = writeln!(uart, "CLM loaded ({clm_len} bytes)");
        }
        Err(e) => {
            let _ = writeln!(uart, "CLM load failed: {e:?}");
        }
    }

    // Scan for access points, printing each as it arrives.
    let _ = writeln!(uart, "scanning for access points...");
    let mut count = 0u32;
    let result = wifi.scan(&timer, |ap| {
        count += 1;
        let _ = write!(uart, "  \"");
        for &b in ap.ssid() {
            let c = if (0x20..=0x7e).contains(&b) {
                b as char
            } else {
                '.'
            };
            let _ = uart.write_char(c);
        }
        let _ = writeln!(
            uart,
            "\" {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} ch{} {}dBm",
            ap.bssid[0],
            ap.bssid[1],
            ap.bssid[2],
            ap.bssid[3],
            ap.bssid[4],
            ap.bssid[5],
            ap.channel,
            ap.rssi
        );
    });
    match result {
        Ok(()) => {
            let _ = writeln!(uart, "scan done: {count} result(s)");
        }
        Err(e) => {
            let _ = writeln!(uart, "scan failed: {e:?}");
        }
    }

    halt();
}

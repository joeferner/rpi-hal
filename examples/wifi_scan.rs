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
use rpi_hal::wifi::Wifi;

/// Directory on the FAT boot partition holding the firmware files, one
/// subdirectory per radio — see [`firmware_subdir`].
const WIFI_DIR: &str = "WIFI";
/// Firmware image, within a [`WIFI_DIR`] subdirectory (8.3 name).
const FIRMWARE_FILE: &str = "FW.BIN";
/// Raw nvram config, within a [`WIFI_DIR`] subdirectory (8.3 name).
const NVRAM_FILE: &str = "NVRAM.TXT";
/// CLM (regulatory) blob, within a [`WIFI_DIR`] subdirectory (8.3 name).
const CLM_FILE: &str = "CLM.DAT";

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
    subdir: &str,
    timer: &Timer,
) -> Result<(usize, usize, usize), embedded_sdmmc::Error<SdCardError>> {
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

    // Read the firmware blobs off the SD card first (this owns EMMC).
    let _ = writeln!(uart, "reading firmware from SD card...");
    let sd = match Sd::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, &timer) {
        Ok(sd) => sd,
        Err(e) => {
            let _ = writeln!(uart, "SD init failed: {e:?}");
            halt();
        }
    };
    let (fw_len, nv_len, clm_len) = match load_files(sd, subdir, &timer) {
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

    // Bus liveness check, and the check on the guess `radio` made from
    // the board revision. Halting on a mismatch rather than warning: the
    // blobs in hand are for another chip, and the download would either
    // fail obscurely or -- worse -- appear to work.
    match sdio.chip_id(&timer) {
        Ok(id) if id == expected_chip_id => {
            let _ = writeln!(uart, "SDIO link up; chip id {id:#06x}");
        }
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

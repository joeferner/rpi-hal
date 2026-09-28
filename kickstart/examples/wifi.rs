#![no_std]
#![no_main]

// Ethernet or Wi-Fi, whichever the board turns out to have: read the
// settings and the radio's firmware off the card, let `net::discover` try
// Ethernet first and fall back to the radio, and take a lease -- reached
// at the address on the `DHCP:` line.
//
// One image, three boards. A 3B and a 3B+ have different Ethernet chips
// and neither is named here; a Zero W has no Ethernet at all and falls
// through to the radio; and a 3B with the cable pulled does the same thing
// the Zero W does. `net::discover` is the whole of that decision, and
// nothing after it knows which way it went -- both interfaces hand the
// stack the same queue pair, which is what this example is demonstrating.
//
// The radio's bring-up is a closure, called at most once and only if
// Ethernet did not answer: where its firmware lives is a board's decision,
// not the crate's. A board with a cable in never reads the firmware -- and
// keeps its card, which the radio would otherwise take the controller
// from.
//
// On the card, beside `config.txt`:
//
//   kickstart.toml -- a `[wifi]` table with `ssid` and `passphrase`. See
//                     `kickstart.toml.example`.
//
// and three vendor blobs in a `wifi` directory under a subdirectory named
// for the radio -- copied once and never looked at again. A 3B or a Zero W
// wants `wifi/43430`:
//
//   FW.BIN    -- Broadcom's brcmfmac43430-sdio.bin
//   NVRAM.TXT -- the matching nvram (brcmfmac43430-sdio.txt)
//   CLM.DAT   -- the CLM regulatory blob (cyfmac43430-sdio.clm_blob)
//
// and a 3B+ or a Pi 4 `wifi/43455`, with the 43455 files of the same names
// -- including the board-specific nvram
// (brcmfmac43455-sdio.raspberrypi,3-model-b-plus.txt for a 3B+). A
// directory per radio is what lets one card boot any of them, since each
// chip refuses the other's image. Names are matched case-insensitively,
// long or 8.3.
//
// A board missing any of them, or the credentials, simply has no Wi-Fi to
// fall back to, and the walk reports that rather than failing. A Zero 2 W's
// 43436 is a third radio again and `rpi-hal` does not drive it, so that
// board reaches Wi-Fi and stops -- which is the honest outcome.
//
// Verify it with `ping <the DHCP address>`, and pull the cable to see the
// same image come up on the radio instead.
//
// Build it with `scripts/build-example.sh wifi`.

extern crate alloc;

use alloc::format;
use alloc::vec::Vec;

use common::settings::{self, Card};
use rpi_hal::mailbox::Mailbox;
use rpi_hal::sdio::Sdio;
use rpi_hal::wifi::Wifi;
use rpi_hal::{pac, timer::Timer};
use rpi_kickstart::config::WifiNetwork;
use rpi_kickstart::{logln, net};

mod common;

/// Directory on the FAT boot partition holding the Wi-Fi firmware files,
/// one subdirectory per radio — see [`radio`].
///
/// The credentials are *not* in here but in `kickstart.toml` at the root,
/// which is a distinction worth keeping: these are vendor blobs, copied
/// once, never looked at again and per-radio, while the network a board
/// joins is something a person edits and does not change with its silicon.
const WIFI_DIR: &str = "WIFI";
/// Firmware image, within a [`WIFI_DIR`] subdirectory.
const FIRMWARE_FILE: &str = "FW.BIN";
/// Raw nvram config, within a [`WIFI_DIR`] subdirectory.
const NVRAM_FILE: &str = "NVRAM.TXT";
/// CLM (regulatory) blob, within a [`WIFI_DIR`] subdirectory.
const CLM_FILE: &str = "CLM.DAT";

#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let mut board = common::boot("wifi");

    let loaded = settings::read(&mut board);
    let mut card = loaded.card;
    let mut network = None;
    if let Some((settings, text)) = loaded.settings {
        network = settings.wifi().unwrap_or_else(|problem| {
            logln!(
                "settings: {}; no Wi-Fi",
                problem.report(settings::FILE, &text)
            );
            None
        });
    }

    let usb = common::usb(&mut board);
    let timer = board.timer;
    let mut bring_up_wifi = || join(timer, card.take(), network);
    let mut hardware = net::Hardware::new().usb(usb.dwc2).wifi(&mut bring_up_wifi);
    let interface = common::discover(&mut hardware, &board, &usb);

    common::start(&board, &usb, interface, |_, _| {})
}

/// Which subdirectory of [`WIFI_DIR`] this board's blobs are in, and the
/// chip id they are for. `None` for a board with no radio `rpi-hal`
/// drives.
///
/// A directory per radio rather than one set of files, so that one card
/// boots any Pi: a 3B and a 3B+ carry different silicon and each refuses
/// the other's image. The names are the part numbers the firmware files
/// are published under, since a directory somebody has to copy files into
/// should be named the way the files are.
///
/// # Why the board and not the chip
///
/// Asking the radio what it is would need no table and never go stale,
/// and it does not work. The chip id is only readable over the backplane,
/// the backplane only once the one EMMC controller has been muxed off the
/// card, and the card is where the firmware is — so it would mean
/// bringing SDIO up, asking, reading the card, and bringing SDIO up a
/// second time. The radio does not answer `CMD5` on that second pass:
/// re-asserting an already-high `WL_ON` is not the power cycle it needs
/// to enumerate again.
///
/// So this guesses from the board and [`join`] *verifies* against the
/// chip id once SDIO is up, before any firmware is written. A wrong entry
/// below is then one clear line naming both numbers rather than a
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

/// Brings the radio up the way a Pi has to: the firmware, nvram and
/// regulatory blob off the card, then a WPA2 join with the credentials
/// from the settings. `None` at the first step that does not work, having
/// said which.
///
/// This is what `net::Hardware::wifi` takes, and the reason it takes a
/// closure rather than doing it: every line below is a board's own choice
/// — where the files live, what they are called, how the credentials are
/// spelled — and a crate that decided them would be deciding for boards
/// that keep their firmware somewhere else entirely.
fn join(timer: &Timer, card: Option<Card>, network: Option<WifiNetwork>) -> Option<Wifi> {
    // Checked first, because without one nothing below is worth doing --
    // and doing it gives the card slot away for nothing.
    let Some(network) = network else {
        logln!(
            "wifi: no [wifi] table in {}; nothing to join",
            settings::FILE
        );
        return None;
    };
    let Some(mut card) = card else {
        logln!("wifi: no card to read firmware from");
        return None;
    };

    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // Which blobs this board needs, before anything touches the card —
    // the mailbox needs no controller, which is exactly why the board and
    // not the chip answers this. See `radio`.
    let board_revision = match mailbox.board_revision() {
        Ok(revision) => revision,
        Err(e) => {
            logln!("wifi: board revision read failed: {e:?}");
            return None;
        }
    };
    let Some((subdir, expected_chip_id)) = radio(board_revision) else {
        logln!("wifi: board revision {board_revision:#010x} has no radio rpi-hal drives");
        return None;
    };

    // Every file the radio needs, into RAM, and then the card goes: the Pi
    // has one EMMC controller and `Sdio::init` re-muxes it onto the
    // wireless pins, so the card slot is gone for the rest of the boot.
    let mut read = |name: &str| -> Option<Vec<u8>> {
        let path = format!("{WIFI_DIR}/{subdir}/{name}");
        let result = card.open(&path).and_then(|file| card.read_all(&file));
        result
            .inspect_err(|e| logln!("wifi: reading {path} off the card failed: {e}"))
            .ok()
    };
    let firmware = read(FIRMWARE_FILE)?;
    let nvram = read(NVRAM_FILE)?;
    let clm = read(CLM_FILE)?;
    logln!(
        "wifi: {WIFI_DIR}/{subdir}/ — firmware {} bytes, nvram {}, clm {}",
        firmware.len(),
        nvram.len(),
        clm.len()
    );
    // Dropped rather than unmounted: nothing was written, so there is
    // nothing to sync, and what matters is that the SD driver inside it is
    // gone before the controller is handed to SDIO.
    drop(card);

    let mut sdio = match Sdio::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, timer) {
        Ok(sdio) => sdio,
        Err(e) => {
            logln!("wifi: SDIO init failed: {e:?}");
            return None;
        }
    };

    // The check on the guess `radio` made, and a bus liveness check
    // besides. Giving up rather than warning: the blobs in hand are for
    // another chip, and the download would either fail obscurely or --
    // worse -- appear to work.
    match sdio.chip_id(timer) {
        Ok(id) if id == expected_chip_id => {}
        Ok(id) => {
            logln!(
                "wifi: chip id {id:#06x}, but board revision {board_revision:#010x} said to \
                 load {WIFI_DIR}/{subdir}/ (for {expected_chip_id:#06x}) — the board table \
                 in `radio` is wrong for this Pi"
            );
            return None;
        }
        Err(e) => {
            logln!("wifi: chip id read failed: {e:?}");
            return None;
        }
    }

    if let Err(e) = sdio.load_firmware(&firmware, &nvram, timer) {
        logln!("wifi: firmware load failed: {e:?}");
        return None;
    }

    let mut wifi = match Wifi::new(sdio, timer) {
        Ok(wifi) => wifi,
        Err(e) => {
            logln!("wifi: protocol init failed: {e:?}");
            return None;
        }
    };

    // The Cypress firmware will not scan or join until the regulatory
    // blob is loaded.
    if let Err(e) = wifi.load_clm(&clm, timer) {
        logln!("wifi: CLM load failed: {e:?}");
        return None;
    }

    let ssid = network.ssid;
    logln!("wifi: joining {ssid:?}...");
    match wifi.join_wpa2(ssid, network.passphrase, timer) {
        Ok(bssid) => {
            logln!("wifi: associated with {}", common::Mac(bssid));

            // The radio's counterpart to `EthernetConfig::all_multicast`,
            // and needed for the same reason: every mDNS query arrives at
            // 224.0.0.251, and a firmware that has not been asked for
            // multicast drops it before the host sees it. The board still
            // announces, takes a lease and answers a ping, so nothing looks
            // wrong -- it simply answers no question anyone asks it.
            //
            // After the join, because the firmware resets this on every
            // association. Not fatal if it is refused: what is lost is
            // being queryable, not the network.
            if let Err(e) = wifi.set_all_multicast(true, timer) {
                logln!("wifi: multicast not enabled ({e:?}); queries will go unanswered");
            }

            // Power save is left at the firmware's default, `Fast`, which
            // is what `rpi-hal`'s own Wi-Fi examples run at. Turning it off
            // keeps the receiver on and costs current; it also has to be
            // re-applied after every association, since the firmware resets
            // it — so it belongs with the runner's rejoin (see
            // `rpi_hal_embassy::wifi::Reconnect`) rather than being set once
            // here and silently lost on the first reconnect.
            //
            // What it costs to leave alone is wake-up latency on an idle
            // board: the first packet after a quiet spell can be dropped and
            // the second take a couple of hundred milliseconds.
            Some(wifi)
        }
        Err(e) => {
            logln!("wifi: join failed: {e:?}");
            None
        }
    }
}

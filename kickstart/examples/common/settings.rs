// The card, and `kickstart.toml` on it.
//
// One schema for every example, because it is one file: whichever example
// is booted, the card carries the same `kickstart.toml`, and an example
// with a narrower schema would report the other examples' tables as
// unknown keys. Each example takes the parts it uses.

// Here rather than relying on the example's root: an example that uses
// none of this still compiles it when every feature is on, and it may have
// no `extern crate alloc` of its own.
extern crate alloc;

use alloc::string::String;
use alloc::vec::Vec;

use rpi_hal::pac;
use rpi_hal::sd::{Sd, SdBlockDevice};
use rpi_kickstart::config::{self, At, Problem, Spanned, value};
use rpi_kickstart::{logln, storage};

use super::Board;

/// The settings file, at the root of the card's FAT partition.
pub const FILE: &str = "kickstart.toml";

/// The card, mounted.
pub type Card = storage::Volume<SdBlockDevice<'static>>;

/// Everything the examples read from [`FILE`]. Every key is optional, and
/// so is the file.
///
/// `Spanned` on the strings a check in `config::value` runs on, so a bad
/// one is reported at its line and column. `Serialize` as well, so the
/// same struct is what `config::save` writes back.
#[derive(serde::Serialize, serde::Deserialize)]
pub struct Settings {
    /// The mDNS name, without `.local` (which is accepted and dropped).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hostname: Option<Spanned<String>>,
    /// Where the clock comes from, if not the defaults.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ntp: Option<config::NtpSettings>,
    /// The network to join if Ethernet does not answer.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wifi: Option<config::WifiSettings>,
}

impl Settings {
    /// The semantic half, which TOML cannot do: it typed `hostname` as a
    /// string, not as a name a resolver can ask for.
    pub fn hostname(&self) -> Result<Option<&str>, Problem> {
        self.hostname
            .as_ref()
            .map(|name| value::label(name.as_ref()).at(name))
            .transpose()
    }

    /// The time server and intervals, over the defaults. Leaked once, at
    /// boot, since `sntp::run` borrows the server for as long as the
    /// program runs.
    #[cfg(feature = "sntp")]
    pub fn ntp(&self) -> Result<rpi_kickstart::sntp::NtpConfig<'static>, Problem> {
        match &self.ntp {
            Some(ntp) => Ok(ntp.check()?.leak()),
            None => Ok(rpi_kickstart::sntp::NtpConfig::DEFAULT),
        }
    }

    /// The `[wifi]` network, checked and leaked for the radio to hold.
    pub fn wifi(&self) -> Result<Option<config::WifiNetwork<'static>>, Problem> {
        self.wifi
            .as_ref()
            .map(|wifi| Ok(wifi.check()?.leak()))
            .transpose()
    }
}

/// What [`read`] found.
pub struct Loaded {
    /// The card, mounted, if there is one.
    pub card: Option<Card>,
    /// The settings and the file's text, if there is a file and it parsed.
    /// The text is what a [`Problem`] found in them is reported against.
    pub settings: Option<(Settings, Vec<u8>)>,
}

/// Mounts the card and reads [`FILE`].
///
/// Nothing here is fatal. No card, no file or a malformed one each leave
/// the example on its defaults with a line saying why: the policy for a
/// bad settings file is the application's, and for an example whose point
/// is something else, booting on defaults is the useful one.
pub fn read(board: &mut Board) -> Loaded {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let sd = match Sd::init(
        &peripherals.GPIO,
        peripherals.EMMC,
        &mut board.mailbox,
        board.timer,
    ) {
        Ok(sd) => sd,
        Err(e) => {
            logln!("settings: no SD card ({e:?}); using defaults");
            return Loaded {
                card: None,
                settings: None,
            };
        }
    };
    let mut card = match storage::mount(SdBlockDevice::new(sd, board.timer)) {
        Ok(card) => card,
        Err(e) => {
            logln!("settings: {e}; using defaults");
            return Loaded {
                card: None,
                settings: None,
            };
        }
    };

    let loaded = match config::load::<Settings, _>(&mut card, FILE) {
        Ok(loaded) => loaded,
        Err(e) => {
            logln!("settings: {e}; using defaults");
            return Loaded {
                card: Some(card),
                settings: None,
            };
        }
    };
    let Some(text) = loaded.text else {
        logln!("settings: no {FILE} on the card; using defaults");
        return Loaded {
            card: Some(card),
            settings: None,
        };
    };
    for key in &loaded.parsed.unknown {
        logln!("settings: {FILE}: ignoring unknown key `{key}`");
    }
    Loaded {
        card: Some(card),
        settings: Some((loaded.parsed.settings, text)),
    }
}

/// The checked `hostname`, leaked for the life of the program — or `None`,
/// with a located line on the console if the value was bad.
pub fn hostname(settings: &Settings, text: &[u8]) -> Option<&'static str> {
    match settings.hostname() {
        Ok(name) => name.map(|name| &*String::from(name).leak()),
        Err(problem) => {
            logln!(
                "settings: {}; using the default",
                problem.report(FILE, text)
            );
            None
        }
    }
}

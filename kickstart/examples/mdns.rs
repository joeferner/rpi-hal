#![no_std]
#![no_main]

// The mDNS responder on a real link, over whichever interface the board
// turns out to have: read `kickstart.toml` off the card, bring up Ethernet
// or Wi-Fi, take a DHCP lease, and answer to the name the file gives --
// `kickstart.local` if it gives none.
//
// One image, three boards. A Pi 3B and a 3B+ have different Ethernet
// chips and neither is named here; a Zero W has no Ethernet at all and
// falls through to the radio; and a 3B with the cable pulled does the same
// thing the Zero W does. `net::discover` is the whole of that decision,
// and nothing below it knows which way it went -- both interfaces hand the
// stack the same queue pair, which is what this example is really
// demonstrating.
//
// Most of the file is still bring-up rather than the responder. That is
// the honest shape: `mdns::run` is three arguments and a spawn, and
// everything above it is what a board has to have working first.
//
// Two pieces of plumbing are easy to miss and each produces the same
// silence -- a name that does not resolve, with nothing reported:
//
//   * `EthernetConfig::all_multicast`, below. The chip drops multicast
//     before the host sees it, and every mDNS query and announcement is
//     multicast. Nothing else a board does notices, because DHCP is
//     broadcast -- which is why it takes a responder to surface it.
//   * `Stack::join_multicast_group`, which `mdns::run` does itself.
//
// The radio has no counterpart to the first of those here, and `join`
// says why not. Queries over Wi-Fi may therefore go unanswered even
// though the board announces itself and is reachable; that belongs in
// `rpi-hal`'s Wi-Fi driver rather than in this file.
//
// The card is mounted once, at boot, through the crate's `storage`, and
// its settings read through `config::load`. The file is optional: with no
// card, or no `kickstart.toml` on it, the board answers to `kickstart` and
// has no Wi-Fi to fall back to. A bad value is reported with its line and
// column and that one setting is ignored, rather than halting -- an
// example that can still come up on Ethernet should.
//
//   kickstart.toml -- at the root, beside `config.txt`. Copy
//                     `kickstart.toml.example` from this repository and
//                     fill it in: `hostname`, and a `[wifi]` table with
//                     `ssid` and `passphrase`.
//
// A `hostname` written as `name.local` is accepted, and the file is then
// saved back through `config::save` with the bare `name` -- which is how
// this example exercises writing as well as reading. The saved file is
// regenerated whole, so it loses the template's comments.
//
//   www/           -- optional: the web assets, loaded into RAM at boot
//                     and listed on the console. Nothing serves them
//                     yet; they are what a board with a web server will.
//
// Wi-Fi needs three more files, all vendor blobs, in a `wifi` directory
// under a subdirectory named for the radio -- copied once and never looked
// at again. A 3B or a Zero W wants `wifi/43430`:
//
//   FW.BIN    -- Broadcom's brcmfmac43430-sdio.bin
//   NVRAM.TXT -- the matching nvram (brcmfmac43430-sdio.txt)
//   CLM.DAT   -- the CLM regulatory blob (cyfmac43430-sdio.clm_blob)
//
// and a 3B+ or a Pi 4 `wifi/43455`, with the 43455 files of the same
// names -- including the board-specific nvram
// (brcmfmac43455-sdio.raspberrypi,3-model-b-plus.txt for a 3B+). A
// directory per radio is what lets one card boot any of them, since each
// chip refuses the other's image. Names are matched case-insensitively,
// long or 8.3.
//
// A board missing any of them, or the credentials, simply has no Wi-Fi to
// fall back to, and the walk reports that rather than failing.
//
// A Zero 2 W's 43436 is a third radio again and `rpi-hal` does not drive
// it, so that board reaches Wi-Fi and stops -- which is the honest
// outcome: the fallback works and there is no driver on the far side.
//
// Verify it from another machine on the same link:
//
//     ping kickstart.local
//     avahi-resolve -n kickstart.local
//     dig +short @<the board's address> -p 5353 kickstart.local
//
// (or whatever `hostname` the file sets).
//
// **Not `dig @224.0.0.251`.** That looks like the obvious test and it
// cannot work: `dig` checks that a reply comes from the server it asked,
// and a responder answers a legacy query from its own address rather than
// from the group -- so `dig` discards a perfectly good answer and reports
// a timeout. Asking the board's own address is the same legacy path (RFC
// 6762 §6.7) with a source `dig` will accept.
//
// `avahi-resolve` can answer out of its cache, which a responder fills by
// announcing. It therefore proves the announcement arrived, *not* that a
// query was answered; only the `dig` line above proves that.
//
// Build it with `scripts/build-example.sh mdns` (kernel7.img) or
// `scripts/build-example64.sh mdns` (kernel8.img).

extern crate alloc;

use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use core::cell::Cell;

use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_net::{Config, StackResources};
use rpi_hal::mailbox::Mailbox;
use rpi_hal::rng::Rng;
use rpi_hal::sd::{Sd, SdBlockDevice};
use rpi_hal::sdio::Sdio;
use rpi_hal::usb::dwc2::Dwc2Host;
use rpi_hal::usb::ethernet::EthernetAsync;
use rpi_hal::usb::lan7800::Lan7800;
use rpi_hal::usb::lan9514::Lan9514;
use rpi_hal::wifi::Wifi;
use rpi_hal::{halt, irq, lic::Lic, pac, timer::Timer, uart::Uart, usb};
use rpi_hal_embassy::channel as ch;
use rpi_hal_embassy::ethernet::{EthernetConfig, EthernetRunner};
use rpi_hal_embassy::wifi::WifiRunner;
use rpi_hal_embassy::{Executor, time_driver};
use rpi_kickstart::config::{self, At, Problem, Spanned, value};
use rpi_kickstart::net::{self, Interface};
use rpi_kickstart::site::{self, Site};
use rpi_kickstart::{console, heap, logln, mdns, storage};
use static_cell::StaticCell;

/// The settings file, at the root of the card's FAT partition.
const SETTINGS_FILE: &str = "kickstart.toml";

/// The name the board answers to when the settings do not give one.
const DEFAULT_HOSTNAME: &str = "kickstart";

/// The name the board answers to, once the settings have been read.
///
/// A static behind a function rather than a value passed down, because
/// `mdns::run` takes `fn() -> &'static str` — so a board that changes its
/// name at run time, from a web form say, is answering to the new one on
/// the next query. This example only ever sets it once, at boot, which is
/// the simple end of the same seam.
static HOSTNAME: Mutex<Cell<&'static str>> = Mutex::new(Cell::new(DEFAULT_HOSTNAME));

fn hostname() -> &'static str {
    critical_section::with(|cs| HOSTNAME.borrow(cs).get())
}

/// Everything this example reads from [`SETTINGS_FILE`]. Every key is
/// optional, and so is the file.
///
/// `Spanned` on the strings a check in `config::value` runs on, so a bad
/// one is reported at its line and column. `Serialize` as well, so the
/// same struct is what `config::save` writes back.
#[derive(serde::Serialize, serde::Deserialize)]
struct Settings {
    /// The mDNS name, without `.local` (which is accepted and dropped).
    #[serde(skip_serializing_if = "Option::is_none")]
    hostname: Option<Spanned<String>>,
    /// The network to join if Ethernet does not answer.
    #[serde(skip_serializing_if = "Option::is_none")]
    wifi: Option<WifiSettings>,
}

/// The `[wifi]` table.
#[derive(serde::Serialize, serde::Deserialize)]
struct WifiSettings {
    ssid: Spanned<String>,
    passphrase: Spanned<String>,
}

/// The network credentials, once checked.
struct Credentials {
    ssid: String,
    passphrase: String,
}

impl Settings {
    /// The semantic half, which TOML cannot do: it typed `hostname` as a
    /// string, not as a name a resolver can ask for.
    fn hostname(&self) -> Result<Option<&str>, Problem> {
        self.hostname
            .as_ref()
            .map(|name| value::label(name.as_ref()).at(name))
            .transpose()
    }

    fn credentials(&self) -> Result<Option<Credentials>, Problem> {
        let Some(wifi) = &self.wifi else {
            return Ok(None);
        };
        Ok(Some(Credentials {
            ssid: value::ssid(wifi.ssid.as_ref()).at(&wifi.ssid)?.into(),
            passphrase: value::passphrase(wifi.passphrase.as_ref())
                .at(&wifi.passphrase)?
                .into(),
        }))
    }
}

/// The card, mounted. Kept for as long as the Wi-Fi bring-up might still
/// want its firmware off it.
type Card = storage::Volume<SdBlockDevice<'static>>;

/// Largest frame the queue pair carries, and the reason there is one pair
/// rather than one per interface.
///
/// The check below is the load-bearing part. Both adapters hand the stack
/// a `ch::Device<'_, MTU>`, and they are the *same type* only because both
/// MTUs are 1514 — `ch::Device` differs by const generic, so an interface
/// that carried a different frame would make these two unrelated types and
/// the single-stack arrangement here would stop compiling. Better to say
/// so here than to discover it as a wall of type errors.
const MTU: usize = rpi_hal_embassy::ethernet::MTU;
const _: () = assert!(
    MTU == rpi_hal_embassy::wifi::MTU,
    "the Ethernet and Wi-Fi adapters must share an MTU for one stack to run over either"
);

/// Frames queued in each direction between the driver and the stack.
const RX_QUEUE: usize = 4;
const TX_QUEUE: usize = 4;

/// Sockets the stack has room for: the responder's UDP socket, and one
/// spare so a failure here is not the first thing suspected.
const SOCKETS: usize = 2;

/// Directory on the FAT boot partition holding the Wi-Fi firmware files,
/// one subdirectory per radio — see [`radio`].
///
/// The credentials are *not* in here but in [`SETTINGS_FILE`] at the root,
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

static UART: StaticCell<Uart> = StaticCell::new();
static DWC2: StaticCell<Dwc2Host> = StaticCell::new();
static TIMER: StaticCell<Timer> = StaticCell::new();
static STATE: StaticCell<ch::State<MTU, RX_QUEUE, TX_QUEUE>> = StaticCell::new();
static RESOURCES: StaticCell<StackResources<SOCKETS>> = StaticCell::new();
static EXECUTOR: StaticCell<Executor> = StaticCell::new();
static SITE: StaticCell<Site> = StaticCell::new();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // Not `logln!`: the console may be mid-write, and re-entering its
    // `RefCell` would turn one panic into an endless pair of them.
    use core::fmt::Write as _;
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

// No `__irq_handler` here: `rpi-hal-embassy`'s `irq-dispatch` feature
// defines one that services every source this program has — the System
// Timer that `embassy-time` runs on, and the USB controller the driver's
// transfers are parked on. Without a handler the first deadline
// livelocks the core, which looks exactly like a hang somewhere else.

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, ch::Device<'static, MTU>>) -> ! {
    runner.run().await
}

// One per interface, and for the Ethernet pair not because they differ --
// those two bodies are identical. `#[embassy_executor::task]` allocates a
// pool of the future's concrete type, so a generic task has no size to
// allocate.
#[embassy_executor::task]
async fn lan9514_task(runner: EthernetRunner<'static, 'static, Lan9514>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn lan7800_task(runner: EthernetRunner<'static, 'static, Lan7800>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn wifi_task(runner: WifiRunner<'static>) -> ! {
    runner.run().await
}

// The crate hands back an `async fn` and the application wraps it, which
// is the whole reason `mdns::run` is not a task itself:
// `#[embassy_executor::task]` cannot be generic, so the concrete types
// have to be named somewhere the application owns.
#[embassy_executor::task]
async fn mdns_task(stack: embassy_net::Stack<'static>) -> ! {
    mdns::run(stack, hostname).await
}

/// Reports the lease once, so there is an address to compare what
/// `kickstart.local` resolves to against.
#[embassy_executor::task]
async fn report_task(stack: embassy_net::Stack<'static>) {
    logln!("waiting for DHCP...");
    stack.wait_config_up().await;
    if let Some(config) = stack.config_v4() {
        logln!(
            "DHCP: {} — this is what {}.local should resolve to",
            config.address,
            hostname()
        );
    }

    // The adapter records a bring-up failure rather than logging it, and
    // nothing was reading it. That matters most for the multicast enable:
    // it is not fatal, so a chip that refused it carries every other
    // frame and looks entirely healthy — while the responder answers
    // nothing it is asked, which is the exact failure this example exists
    // to surface.
    if let Some(error) = rpi_hal_embassy::ethernet::start_error() {
        logln!("ethernet: something in bring-up failed: {error:?}");
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    console::init(
        UART.init(Uart::init(&peripherals.GPIO, peripherals.UART0)),
        // The realistic form of this seam: `Instant::now()` captures
        // nothing, so it coerces to the `fn` pointer `console::init`
        // takes. It reads correctly before `time_driver::init` because
        // the System Timer has been running since boot.
        || embassy_time::Instant::now().as_micros(),
    );
    logln!("rpi-kickstart mdns");

    let timer = TIMER.init(Timer::new(peripherals.SYSTMR));
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // Before anything allocates: mounting the card reads its allocation
    // table onto the heap, and parsing the settings builds strings.
    match heap::init(&mut mailbox) {
        Ok(bytes) => logln!("heap: {} MiB", bytes / (1024 * 1024)),
        Err(e) => {
            logln!("heap: {e:?}");
            halt();
        }
    }

    let (mut card, credentials) = read_settings(&mut mailbox, timer);
    logln!("mdns: answering to {}.local", hostname());

    // The web assets, off the same mount and before the radio might take
    // the card. Nothing serves them yet -- that is the web module's, which
    // does not exist -- so this is here to show what a board will serve:
    // one line per file, and a count.
    let site = SITE.init(match card.as_mut() {
        Some(card) => site::load(card).unwrap_or_else(|e| {
            logln!(
                "site: reading /{} failed: {e}; serving no pages",
                site::WWW_DIR
            );
            Site::default()
        }),
        None => Site::default(),
    });
    logln!(
        "site: {} {}",
        site.len(),
        if site.len() == 1 { "file" } else { "files" }
    );

    if !usb::power_on(&mut mailbox) {
        logln!("USB power-on failed");
        halt();
    }

    let mac = match mailbox.mac_address() {
        Ok(mac) => mac,
        Err(e) => {
            logln!("MAC read failed: {e:?}");
            halt();
        }
    };
    logln!(
        "board MAC: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5]
    );

    let dwc2: &'static Dwc2Host = DWC2.init(Dwc2Host::init(
        peripherals.USB_OTG_GLOBAL,
        peripherals.USB_OTG_HOST,
        peripherals.USB_OTG_PWRCLK,
        timer,
    ));

    // Everything from "is there a hub" to "is a cable in" is `net`'s, and
    // that is the point of the module: the deadlines involved are the
    // whole diagnosis when a board comes up with no network, and no board
    // should have to write them out again.
    //
    // The radio's bring-up is a closure because where its firmware lives
    // is a board's decision and not this crate's -- see `net::Hardware`.
    // It is called at most once, and only if Ethernet did not answer, so a
    // board with a cable in never reads the firmware -- and keeps its card,
    // which the radio would otherwise take the controller from.
    let mut bring_up_wifi = || join(timer, card.take(), credentials.as_ref());
    let mut hardware = net::Hardware::new().usb(dwc2).wifi(&mut bring_up_wifi);
    let Some(interface) = net::discover(&mut hardware, timer, mac, &net::Config::default()) else {
        halt();
    };
    // The address as well as the name, because they differ per interface
    // and the stack is built from this one: the Ethernet chips take the
    // board address the firmware mailbox reports, while the radio has its
    // own. A stack built with the wrong one associates, takes a lease,
    // announces itself — and then answers nothing sent to it, because
    // every frame addressed to the chip is discarded a layer above as not
    // ours. Worth one line to be able to compare against what a peer's
    // ARP table says.
    let interface_mac = interface.mac();
    logln!(
        "net: using {} at {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        interface.name(),
        interface_mac[0],
        interface_mac[1],
        interface_mac[2],
        interface_mac[3],
        interface_mac[4],
        interface_mac[5]
    );

    // From here on the interrupts are live, so nothing above may be
    // blocking on the USB controller or the SDIO bus any more: both
    // bring-ups poll, and an interrupt arriving mid-poll is serviced by a
    // handler that knows nothing about them.
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);
    time_driver::init(Timer::new(peripherals.SYSTMR), &lic);
    irq::enable_irq();

    // The queue pair, built once and here rather than inside an adapter.
    // That is what lets either interface attach to it, and it is the
    // groundwork for swapping one for the other on a live stack -- see
    // `rpi_hal_embassy::ethernet::attach`.
    //
    // The address is the interface's own, not the board's: the Ethernet
    // chips are programmed with what the firmware mailbox reports, while
    // the radio has one of its own. A stack built with the wrong one
    // associates and then answers no ARP.
    let state = STATE.init(ch::State::new());
    let (ch_runner, driver) = ch::new(
        state,
        ch::driver::HardwareAddress::Ethernet(interface.mac()),
    );

    // The only place an interface is named. Each arm attaches its runner
    // to the queue pair above and hands `run` a closure that spawns it,
    // which is the one thing that cannot be written once:
    // `#[embassy_executor::task]` needs a concrete future type.
    match interface {
        Interface::Lan9514(dev, _) => {
            let runner = attach_ethernet(ch_runner, dev, dwc2, timer, mac, &lic);
            run(driver, |spawner| {
                spawner.spawn(lan9514_task(runner).unwrap())
            })
        }
        Interface::Lan7800(dev, _) => {
            let runner = attach_ethernet(ch_runner, dev, dwc2, timer, mac, &lic);
            run(driver, |spawner| {
                spawner.spawn(lan7800_task(runner).unwrap())
            })
        }
        Interface::Wifi(wifi, _) => {
            // No USB interrupt enabled on this path: the radio is driven
            // over SDIO by a runner that polls, and the USB controller --
            // powered and walked a moment ago, and found wanting -- has
            // nothing outstanding to interrupt about.
            let runner = rpi_hal_embassy::wifi::attach(ch_runner, wifi, timer);
            run(driver, |spawner| spawner.spawn(wifi_task(runner).unwrap()))
        }
    }
}

/// Puts a USB Ethernet chip behind the queue pair, and turns on the
/// interrupt its transfers park on.
///
/// Generic over `EthernetAsync`, so a LAN9514 and a LAN7800 take the same
/// path — the two chips share no types but do share the trait.
///
/// The chip is **not** started here. `net::discover` started it to ask the
/// PHY whether a cable was in, and the adapter resets it and starts it
/// again its own way: the blocking and awaited bring-ups configure the
/// chip differently, and one owner is what keeps them from disagreeing.
fn attach_ethernet<E: EthernetAsync + 'static>(
    ch_runner: ch::Runner<'static, MTU>,
    ethernet: E,
    dwc2: &'static Dwc2Host,
    timer: &'static Timer,
    mac: [u8; 6],
    lic: &Lic,
) -> EthernetRunner<'static, 'static, E> {
    // Two host channels, one per direction, held for as long as the
    // interface is. The walk's own went back when it returned.
    let (Some(rx_channel), Some(tx_channel)) = (dwc2.alloc_channel(), dwc2.alloc_channel()) else {
        logln!("no free host channels for the stack");
        halt();
    };
    lic.enable_usb_irq();

    rpi_hal_embassy::ethernet::attach(
        ch_runner,
        ethernet,
        rx_channel,
        tx_channel,
        timer,
        // `all_multicast` is the half of the plumbing `mdns::run` cannot do
        // for itself. Without it the chip filters multicast out before the
        // host sees it and the responder binds a socket nothing ever
        // arrives on -- while still announcing, since announcements are
        // transmitted. The symptom is a name that resolves once and then
        // never answers again.
        EthernetConfig {
            mac,
            all_multicast: true,
        },
    )
}

/// Builds the stack over whatever attached to the queue pair and runs the
/// executor. Never returns.
///
/// `spawn_interface` is the one line that cannot be written once, passed
/// in because this function has no way to name the opaque `SpawnToken` a
/// task returns.
fn run<S: FnOnce(Spawner)>(driver: ch::Device<'static, MTU>, spawn_interface: S) -> ! {
    // A random seed keeps TCP initial sequence numbers and the DHCP
    // transaction ID from repeating across boots.
    let mut rng = Rng::new();
    let seed = (u64::from(rng.next_u32()) << 32) | u64::from(rng.next_u32());

    let resources = RESOURCES.init(StackResources::new());
    let (stack, runner) =
        embassy_net::new(driver, Config::dhcpv4(Default::default()), resources, seed);

    EXECUTOR.init(Executor::new()).run(|spawner| {
        // In embassy-executor 0.10 it is the *task function* that returns
        // the `Result` — a token, or `SpawnError` if its pool is already
        // full — and `spawn` takes the token. Every task here has a pool
        // of one and is spawned once, so the unwrap is unreachable.
        spawner.spawn(net_task(runner).unwrap());
        spawn_interface(spawner);
        spawner.spawn(report_task(stack).unwrap());
        spawner.spawn(mdns_task(stack).unwrap());
    });
}

/// Mounts the card and reads [`SETTINGS_FILE`], setting [`HOSTNAME`] and
/// returning the mounted card and the Wi-Fi credentials, if any.
///
/// Nothing here is fatal. No card, no file or a bad value each leave the
/// board on its defaults with a line saying why, because the policy for a
/// malformed settings file is the application's, and for an example whose
/// point is the responder, booting on defaults is the useful one.
fn read_settings(
    mailbox: &mut Mailbox,
    timer: &'static Timer,
) -> (Option<Card>, Option<Credentials>) {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let sd = match Sd::init(&peripherals.GPIO, peripherals.EMMC, mailbox, timer) {
        Ok(sd) => sd,
        Err(e) => {
            logln!("settings: no SD card ({e:?}); using defaults");
            return (None, None);
        }
    };
    let mut card = match storage::mount(SdBlockDevice::new(sd, timer)) {
        Ok(card) => card,
        Err(e) => {
            logln!("settings: {e}; using defaults");
            return (None, None);
        }
    };

    let loaded = match config::load::<Settings, _>(&mut card, SETTINGS_FILE) {
        Ok(loaded) => loaded,
        Err(e) => {
            logln!("settings: {e}; using defaults");
            return (Some(card), None);
        }
    };
    let Some(text) = loaded.text else {
        logln!("settings: no {SETTINGS_FILE} on the card; using defaults");
        return (Some(card), None);
    };
    for key in &loaded.parsed.unknown {
        logln!("settings: {SETTINGS_FILE}: ignoring unknown key `{key}`");
    }

    let mut settings = loaded.parsed.settings;
    match settings.hostname() {
        Ok(Some(name)) => {
            // Leaked once, at boot: the name lives as long as the program,
            // and `mdns::run` wants a `&'static str`.
            let name: &'static str = String::from(name).leak();
            critical_section::with(|cs| HOSTNAME.borrow(cs).set(name));
            normalize_hostname(&mut card, &mut settings, name);
        }
        Ok(None) => {}
        Err(problem) => logln!(
            "settings: {}; answering to {DEFAULT_HOSTNAME}",
            problem.report(SETTINGS_FILE, &text)
        ),
    }
    let credentials = settings.credentials().unwrap_or_else(|problem| {
        logln!(
            "settings: {}; no Wi-Fi",
            problem.report(SETTINGS_FILE, &text)
        );
        None
    });
    (Some(card), credentials)
}

/// Writes the settings back with `hostname` as the label it resolved to,
/// if the file spelled it some other way — `kickstart.local` for
/// `kickstart`, which `value::label` accepts and drops.
///
/// This is the example's use of `config::save`, and a deliberate trigger
/// for it: put `.local` on the name, boot, and the file on the card comes
/// back without it. The whole file is regenerated, so the template's
/// comments go with it — the trade `config::save` makes, and the reason
/// the annotated copy lives in the repository.
fn normalize_hostname(card: &mut Card, settings: &mut Settings, name: &str) {
    let Some(written) = &settings.hostname else {
        return;
    };
    if written.as_ref() == name {
        return;
    }
    // The span is carried over for form's sake; nothing reads it on the
    // way out, and the next load gives the new file's own.
    settings.hostname = Some(Spanned::new(written.span(), name.into()));
    match config::save(card, SETTINGS_FILE, settings) {
        Ok(text) => logln!(
            "settings: wrote {SETTINGS_FILE} back with hostname = {name:?} ({} bytes)",
            text.len()
        ),
        Err(e) => logln!("settings: saving {SETTINGS_FILE} failed: {e}"),
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
fn join(timer: &Timer, card: Option<Card>, credentials: Option<&Credentials>) -> Option<Wifi> {
    // Checked first, because without them nothing below is worth doing --
    // and doing it gives the card slot away for nothing.
    let Some(credentials) = credentials else {
        logln!("wifi: no [wifi] table in {SETTINGS_FILE}; nothing to join");
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

    // No `allmulti` iovar here, deliberately, and it is worth saying why
    // rather than leaving its absence to be rediscovered.
    //
    // The radio does need something like `EthernetConfig::all_multicast`:
    // the firmware filters multicast the host has not asked for, and
    // every mDNS query arrives at 224.0.0.251. But setting
    // `allmulti` here -- before the join, which is not where Linux sets
    // it -- left the board receiving broadcast and *no unicast at all*:
    // 100 pings produced no rise in the driver's receive counter and one
    // transmit. Whatever it did, it was not what it was reaching for, and
    // a filter that costs the board every unicast frame is far worse than
    // a responder that cannot be queried.
    //
    // The right home for this is `rpi-hal`'s Wi-Fi driver, alongside its
    // Ethernet counterpart and applied where Linux applies it, rather
    // than an iovar poked in from an example on the strength of a name.

    let ssid = credentials.ssid.as_str();
    logln!("wifi: joining {ssid:?}...");
    match wifi.join_wpa2(ssid, &credentials.passphrase, timer) {
        Ok(bssid) => {
            logln!(
                "wifi: associated with {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                bssid[0],
                bssid[1],
                bssid[2],
                bssid[3],
                bssid[4],
                bssid[5]
            );

            // Keep the receiver on, and set it *after* associating
            // because the firmware resets this on every association.
            //
            // The default is `Fast`, which sleeps once a link has been
            // idle a while and wakes for beacons. A board that answers
            // rather than asks — a responder, a server — is idle by
            // definition, and a dozing station gets its broadcast
            // delivered after the DTIM beacon while the access point
            // *buffers its unicast*. That reads as a board which takes a
            // DHCP lease, announces itself, and then cannot be pinged,
            // with the driver's receive counter still climbing on
            // broadcast the whole time.
            //
            // The giveaway is the latency when it does answer: the first
            // packet lost, the second a couple of hundred milliseconds,
            // the third back to normal.
            // The radio's counterpart to `EthernetConfig::all_multicast`,
            // and needed for the same reason: every mDNS query arrives at
            // 224.0.0.251, and a firmware that has not been asked for
            // multicast drops it before the host sees it. The board still
            // announces, still takes a lease and still answers a ping, so
            // nothing looks wrong -- it simply answers no question anyone
            // asks it.
            //
            // After the join, because the firmware resets this on every
            // association. Not fatal if it is refused: what is lost is
            // being queryable, not the network.
            if let Err(e) = wifi.set_all_multicast(true, timer) {
                logln!("wifi: multicast not enabled ({e:?}); queries will go unanswered");
            }

            // Power save is left at the firmware's default, `Fast`, which
            // is what `rpi-hal`'s own Wi-Fi examples run at. Turning it
            // off keeps the receiver on and costs current; it also has to
            // be re-applied after every association, since the firmware
            // resets it — so it belongs with the runner's rejoin (see
            // `rpi_hal_embassy::wifi::Reconnect`) rather than being set
            // once here and silently lost on the first reconnect.
            //
            // What it costs to leave alone is wake-up latency on an idle
            // board: the first packet after a quiet spell can be dropped
            // and the second take a couple of hundred milliseconds.
            Some(wifi)
        }
        Err(e) => {
            logln!("wifi: join failed: {e:?}");
            None
        }
    }
}

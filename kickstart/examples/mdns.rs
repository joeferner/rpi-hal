#![no_std]
#![no_main]

// The mDNS responder on a real link, over whichever interface the board
// turns out to have: bring up Ethernet or Wi-Fi, take a DHCP lease, and
// answer to `kickstart.local`.
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
// Three pieces of plumbing are easy to miss and each produces the same
// silence -- a name that does not resolve, with nothing reported:
//
//   * `EthernetConfig::all_multicast`, below. The chip drops multicast
//     before the host sees it, and every mDNS query and announcement is
//     multicast. Nothing else a board does notices, because DHCP is
//     broadcast -- which is why it takes a responder to surface it.
//   * the `allmulti` iovar, which is the radio's version of the same
//     thing. See `join`.
//   * `Stack::join_multicast_group`, which `mdns::run` does itself.
//
// Wi-Fi needs four files on the boot partition, under 8.3 names. Three
// are vendor blobs, in a `wifi` directory -- copied once and never looked
// at again:
//
//   wifi/FW.BIN    -- Broadcom's brcmfmac43430-sdio.bin
//   wifi/NVRAM.TXT -- the matching nvram (brcmfmac43430-sdio.txt)
//   wifi/CLM.DAT   -- the CLM regulatory blob (cyfmac43430-sdio.clm_blob)
//
// and the fourth is the one a person edits, so it sits at the root beside
// `config.txt` with the board's other settings:
//
//   WIFI.CFG       -- two lines: the SSID, then the WPA2 passphrase
//
// A board missing any of them simply has no Wi-Fi to fall back to, and the
// walk reports that rather than failing.
//
// **The radio has to be a BCM43430** -- a Pi 3B or a Zero W. `rpi-hal`'s
// SDIO driver walks the backplane and remaps the RAM banks in a way that
// is specific to that chip, and a Pi 3B+ or Zero 2 W carries a 43455/43436
// instead. On one of those this falls through to the radio and fails
// there, which is the honest outcome: the fallback works and there is no
// driver on the other side of it.
//
// Verify it from another machine on the same link:
//
//     ping kickstart.local
//     dig +short @224.0.0.251 -p 5353 kickstart.local
//
// The second is the legacy-query path (RFC 6762 §6.7) and is the one that
// works from a shell with no mDNS client in the way.
//
// Build it with `scripts/build-example.sh mdns` (kernel7.img) or
// `scripts/build-example64.sh mdns` (kernel8.img).

use core::ptr::{addr_of, addr_of_mut};

use embassy_executor::Spawner;
use embassy_net::{Config, StackResources};
use embedded_sdmmc::{Mode, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use rpi_hal::mailbox::Mailbox;
use rpi_hal::rng::Rng;
use rpi_hal::sd::{Sd, SdCard, SdCardError};
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
use rpi_kickstart::net::{self, Interface};
use rpi_kickstart::{console, logln, mdns};
use static_cell::StaticCell;

/// The name the board answers to. A real board reads this from its
/// settings and passes a function that re-reads it, which is why
/// `mdns::run` takes `fn() -> &'static str` rather than a `&str` — see
/// that module. Here it is fixed, which is the other end of the same
/// seam.
const NAME: &str = "kickstart";

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

/// Directory on the FAT boot partition holding the Wi-Fi firmware files.
const WIFI_DIR: &str = "WIFI";
/// Firmware image, within [`WIFI_DIR`] (8.3 name).
const FIRMWARE_FILE: &str = "FW.BIN";
/// Raw nvram config, within [`WIFI_DIR`] (8.3 name).
const NVRAM_FILE: &str = "NVRAM.TXT";
/// CLM (regulatory) blob, within [`WIFI_DIR`] (8.3 name).
const CLM_FILE: &str = "CLM.DAT";
/// Network credentials: the SSID on the first line and the WPA2
/// passphrase on the second (8.3 name).
///
/// At the **root** of the boot partition rather than in [`WIFI_DIR`],
/// which is a distinction worth keeping: the three files above are vendor
/// blobs that are copied once and never looked at again, while this is the
/// one a person edits. It sits beside `config.txt`, where a board's other
/// settings already live.
const CONFIG_FILE: &str = "WIFI.CFG";

/// Buffer for the firmware image (the 43430's is ~420KB); zeroed BSS.
static mut FW_BUF: [u8; 512 * 1024] = [0; 512 * 1024];
/// Buffer for the raw nvram text.
static mut NV_BUF: [u8; 4096] = [0; 4096];
/// Buffer for the CLM regulatory blob (~5KB).
static mut CLM_BUF: [u8; 8192] = [0; 8192];
/// Buffer for the network-credentials file.
static mut CFG_BUF: [u8; 256] = [0; 256];

static UART: StaticCell<Uart> = StaticCell::new();
static DWC2: StaticCell<Dwc2Host> = StaticCell::new();
static TIMER: StaticCell<Timer> = StaticCell::new();
static STATE: StaticCell<ch::State<MTU, RX_QUEUE, TX_QUEUE>> = StaticCell::new();
static RESOURCES: StaticCell<StackResources<SOCKETS>> = StaticCell::new();
static EXECUTOR: StaticCell<Executor> = StaticCell::new();

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
    mdns::run(stack, || NAME).await
}

/// Reports the lease once, so there is an address to compare what
/// `kickstart.local` resolves to against.
#[embassy_executor::task]
async fn report_task(stack: embassy_net::Stack<'static>) {
    logln!("waiting for DHCP...");
    stack.wait_config_up().await;
    if let Some(config) = stack.config_v4() {
        logln!(
            "DHCP: {} — this is what {NAME}.local should resolve to",
            config.address
        );
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
    // board with a cable in never pays the card reading below.
    let mut bring_up_wifi = || join(timer);
    let mut hardware = net::Hardware::new().usb(dwc2).wifi(&mut bring_up_wifi);
    let Some(interface) = net::discover(&mut hardware, timer, mac, &net::Config::default()) else {
        halt();
    };
    logln!("net: using {}", interface.name());

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

/// Brings the radio up the way a Pi has to: the firmware, nvram and
/// regulatory blob off the card, then a WPA2 join with the credentials
/// beside them. `None` at the first step that does not work, having said
/// which.
///
/// This is what `net::Hardware::wifi` takes, and the reason it takes a
/// closure rather than doing it: every line below is a board's own choice
/// — where the files live, what they are called, how the credentials are
/// spelled — and a crate that decided them would be deciding for boards
/// that keep their firmware somewhere else entirely.
fn join(timer: &Timer) -> Option<Wifi> {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // The card first, and only once: the Pi has one EMMC controller and
    // `Sdio::init` re-muxes it onto the wireless pins, so every file the
    // radio needs has to be in RAM before it starts — and the card slot is
    // gone for the rest of the boot once it has.
    let sd = match Sd::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, timer) {
        Ok(sd) => sd,
        Err(e) => {
            logln!("wifi: no SD card to read firmware from: {e:?}");
            return None;
        }
    };
    let (fw_len, nv_len, clm_len, cfg_len) = match load_files(sd, timer) {
        Ok(lengths) => lengths,
        Err(e) => {
            logln!("wifi: reading {WIFI_DIR}/ off the card failed: {e:?}");
            return None;
        }
    };
    logln!("wifi: firmware {fw_len} bytes, nvram {nv_len}, clm {clm_len}");

    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut sdio = match Sdio::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, timer) {
        Ok(sdio) => sdio,
        Err(e) => {
            logln!("wifi: SDIO init failed: {e:?}");
            return None;
        }
    };

    // Safety: `load_files` has finished writing these; read-only now.
    let firmware = &unsafe { &*addr_of!(FW_BUF) }[..fw_len];
    let nvram = &unsafe { &*addr_of!(NV_BUF) }[..nv_len];
    if let Err(e) = sdio.load_firmware(firmware, nvram, timer) {
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
    let clm = &unsafe { &*addr_of!(CLM_BUF) }[..clm_len];
    if let Err(e) = wifi.load_clm(clm, timer) {
        logln!("wifi: CLM load failed: {e:?}");
        return None;
    }

    // The radio's version of `EthernetConfig::all_multicast`, and needed
    // for exactly the same reason: the firmware filters multicast frames
    // that are not addressed to a group it has been told about, and every
    // mDNS query arrives at 224.0.0.251. Without it the responder
    // announces, is heard, and then answers nothing it is asked.
    //
    // Set here rather than in the driver because `rpi-hal`'s Wi-Fi driver
    // has no multicast surface yet -- an iovar is what there is. A rejoin
    // does not re-apply it, so a radio that reassociates may come back
    // deaf to multicast while the link looks healthy.
    if let Err(e) = wifi.set_iovar_u32("allmulti", 1, timer) {
        logln!("wifi: could not ask the firmware for all multicast: {e:?}");
    }

    let config = &unsafe { &*addr_of!(CFG_BUF) }[..cfg_len];
    let Some((ssid, passphrase)) = parse_config(config) else {
        logln!("wifi: {WIFI_DIR}/{CONFIG_FILE} is missing or malformed; nothing to join");
        return None;
    };
    logln!("wifi: joining {ssid:?}...");
    match wifi.join_wpa2(ssid, passphrase, timer) {
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
            Some(wifi)
        }
        Err(e) => {
            logln!("wifi: join failed: {e:?}");
            None
        }
    }
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

/// Mounts the boot partition and reads the firmware, nvram, CLM blob and
/// credentials into the static buffers, returning their lengths. Consumes
/// the SD driver, and with it the EMMC controller the radio is about to
/// want.
fn load_files(
    sd: Sd,
    timer: &Timer,
) -> Result<(usize, usize, usize, usize), embedded_sdmmc::Error<SdCardError>> {
    let volume_mgr = VolumeManager::new(SdCard::new(sd, timer), FixedTime);
    let volume = volume_mgr.open_volume(VolumeIdx(0))?;
    let root = volume.open_root_dir()?;
    let wifi = root.open_dir(WIFI_DIR)?;

    // Safety: single-threaded bare-metal; these buffers are touched only
    // here and, after this returns, read-only in `join`.
    let fw_len = read_file(&wifi, FIRMWARE_FILE, unsafe { &mut *addr_of_mut!(FW_BUF) })?;
    let nv_len = read_file(&wifi, NVRAM_FILE, unsafe { &mut *addr_of_mut!(NV_BUF) })?;
    let clm_len = read_file(&wifi, CLM_FILE, unsafe { &mut *addr_of_mut!(CLM_BUF) })?;
    let cfg_len = read_file(&root, CONFIG_FILE, unsafe { &mut *addr_of_mut!(CFG_BUF) })?;
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

/// Splits the credentials file — SSID on the first line, passphrase on the
/// second — trimming end-of-line whitespace. `None` unless both lines are
/// present, non-empty and valid UTF-8.
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

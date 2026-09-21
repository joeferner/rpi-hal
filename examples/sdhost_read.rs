#![no_std]
#![no_main]

//! Drives the SD card and the wireless chip **at the same time**: the
//! card over the SDHOST controller (`rpi_hal::sdhost`), the radio over the
//! Arasan/EMMC one (`rpi_hal::sdio`).
//!
//! That is the whole point of `sdhost`, and this is the test of it. Every
//! other example that touches Wi-Fi has to read what it needs off the card
//! first and then give the slot up, because one controller cannot be muxed
//! to two pin groups — see `wifi_smoltcp.rs`, which says so at the top.
//!
//! So the sequence here is deliberately interleaved:
//!
//! 1. Mount the card on SDHOST and read `config.txt`.
//! 2. Bring up the wireless chip on the Arasan controller, and read its
//!    chip id over the backplane — proof that the SDIO bus enumerated.
//! 3. Read `config.txt` **again**, over SDHOST, with the radio's
//!    controller still up.
//!
//! If the third step works, the two controllers are genuinely independent
//! on this board. If it fails where the first succeeded, they are not, and
//! the failure is worth more than the rest of the output.
//!
//! No firmware blob is needed: enumerating the chip and reading its id
//! happens before any firmware is downloaded into it. What this does not
//! show is the radio *working*, which needs those files — that is what the
//! application built on this does.
//!
//! Pi 2/3 and Pi Zero W. Not a Pi 4: its card is on EMMC2, a separate
//! controller again, and GPIO48-53 there belong to the Ethernet PHY.
//!
//! Output goes to UART0 (PL011) at 115200 8N1.

extern crate alloc;

use core::fmt::Write;

use embedded_alloc::LlffHeap as Heap;
use resident_fat::{Error, FileSystem};
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::sdhost::{Sdhost, SdhostBlockDevice, SdhostBlockDeviceError};
use rpi_hal::sdio::{Sdio, BCM43438_CHIP_ID};
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;

/// The global heap, which the filesystem needs: `resident-fat` keeps the
/// allocation table and the directories it has walked in RAM. See
/// `examples/heap_alloc.rs` for the full explanation.
#[global_allocator]
static HEAP: Heap = Heap::empty();

extern "C" {
    /// End of the `.bss` section, defined by the linker script. Only its
    /// address is meaningful — the byte itself is never read.
    static __bss_end: u8;
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// The card's volume, on this example's one block device.
type Volume<'t> = FileSystem<SdhostBlockDevice<'t>>;

/// Mounts the boot partition and lists it. Split out from `kmain` so the
/// filesystem steps can use `?` against one error type.
fn mount<'t>(
    sdhost: Sdhost,
    timer: &'t Timer,
    uart: &mut Uart,
) -> Result<Volume<'t>, Error<SdhostBlockDeviceError>> {
    // Partition 0 is the FAT boot partition on a stock Raspberry Pi card.
    // `mount` would be the call for a card formatted as one bare volume.
    let fs = FileSystem::mount_partition(SdhostBlockDevice::new(sdhost, timer), 0)?;

    let clusters = fs.fat().cluster_count();
    let _ = writeln!(
        uart,
        "mounted: {clusters} clusters of {} KiB, allocation table {} KiB in RAM{}",
        fs.boot_sector().cluster_bytes() / 1024,
        (clusters + 2) * 4 / 1024,
        if fs.is_dirty() {
            ", NOT CLEANLY UNMOUNTED"
        } else {
            ""
        }
    );
    Ok(fs)
}

/// Reads `config.txt` and reports its length and first line.
///
/// Called twice — before the radio's controller is up and after it — so
/// what it prints is meant to be compared with itself. A length that
/// changes, or a read that fails the second time, is the two controllers
/// interfering.
fn read_config(fs: &mut Volume<'_>, uart: &mut Uart, when: &str) {
    match fs.open("config.txt") {
        Ok(file) => match fs.read_all(&file) {
            Ok(contents) => {
                let first_line = contents
                    .split(|&b| b == b'\n')
                    .next()
                    .unwrap_or(&[])
                    .iter()
                    .map(|&b| {
                        if (0x20..=0x7e).contains(&b) {
                            b as char
                        } else {
                            '.'
                        }
                    })
                    .take(40)
                    .collect::<alloc::string::String>();
                let _ = writeln!(
                    uart,
                    "{when}: config.txt is {} bytes, starts \"{first_line}\"",
                    contents.len()
                );
            }
            Err(e) => {
                let _ = writeln!(uart, "{when}: config.txt could not be read: {e:?}");
            }
        },
        Err(Error::NotFound { .. }) => {
            let _ = writeln!(uart, "{when}: no config.txt on the card");
        }
        Err(e) => {
            let _ = writeln!(uart, "{when}: config.txt could not be opened: {e:?}");
        }
    }
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // The heap has to exist before the filesystem: mounting allocates the
    // resident allocation table as its first act.
    let heap_start = &raw const __bss_end as usize;
    let region = match mailbox.arm_memory() {
        Ok(region) => region,
        Err(e) => {
            let _ = writeln!(uart, "could not read ARM memory size: {e:?}");
            halt();
        }
    };
    let heap_size = (region.base_address + region.size_bytes) as usize - heap_start;
    let _ = writeln!(uart, "heap: {} KiB at 0x{heap_start:08x}", heap_size / 1024);
    // SAFETY: called once, before any allocation, on a region above `.bss`
    // and below the peripheral base that nothing else claims.
    unsafe { HEAP.init(heap_start, heap_size) };

    let _ = writeln!(uart, "\ninitializing the card on SDHOST...");
    let sdhost = match Sdhost::init(&peripherals.GPIO, &mut mailbox, &timer) {
        Ok(sdhost) => sdhost,
        Err(e) => {
            let _ = writeln!(uart, "SDHOST init failed: {e:?}");
            halt();
        }
    };
    let _ = writeln!(
        uart,
        "SDHOST ready: {} card, {} bus",
        if sdhost.high_capacity() {
            "high-capacity"
        } else {
            "standard-capacity"
        },
        if sdhost.four_bit_bus() {
            "4-bit"
        } else {
            "1-bit"
        }
    );

    let mut fs = match mount(sdhost, &timer, &mut uart) {
        Ok(fs) => fs,
        Err(e) => {
            let _ = writeln!(uart, "mount failed: {e:?}");
            halt();
        }
    };
    read_config(&mut fs, &mut uart, "before Wi-Fi");

    // The other controller, on the other pins, while the volume above
    // stays mounted. Nothing is unmounted and nothing is dropped: that is
    // the claim being tested.
    let _ = writeln!(uart, "\nbringing up the wireless chip on EMMC...");
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut sdio = match Sdio::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, &timer) {
        Ok(sdio) => sdio,
        Err(e) => {
            let _ = writeln!(uart, "SDIO init failed: {e:?}");
            halt();
        }
    };
    match sdio.chip_id(&timer) {
        Ok(id) => {
            let _ = writeln!(
                uart,
                "SDIO ready: chip id {id:#06x}{}",
                if id == BCM43438_CHIP_ID {
                    " (BCM43438)"
                } else {
                    " -- not the chip this expects"
                }
            );
        }
        Err(e) => {
            let _ = writeln!(uart, "chip id unreadable: {e:?}");
        }
    }

    // The point of the whole example.
    let _ = writeln!(
        uart,
        "\nreading the card again, with both controllers up..."
    );
    read_config(&mut fs, &mut uart, "after Wi-Fi");

    let _ = writeln!(uart, "\ndone");
    halt();
}

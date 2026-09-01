#![no_std]
#![no_main]

//! Reads files from the FAT partition on the SD card using `resident-fat` as
//! the FAT layer, on top of `rpi_hal::sd::SdBlockDevice`. It lists the root
//! directory of the first partition (the Pi's boot FAT partition on a stock
//! card) and prints the contents of `config.txt` if present -- a
//! human-verifiable end-to-end check that a real filesystem parses off a
//! real card.
//!
//! The sibling of `sd_fat_read.rs`, which does the same thing over
//! `embedded-sdmmc` and `rpi_hal::sd::SdCard`. Reading the two side by side
//! is the clearest statement of what each adapter costs:
//!
//! * There is no `TimeSource` to supply. `resident-fat` defaults to the FAT
//!   epoch and takes a clock through `set_clock` when the application has
//!   one, so a read-only mount needs nothing.
//! * `mount_partition` finds the volume behind the MBR, so the partition
//!   table isn't this example's problem either.
//! * The block device is handed byte slices spanning whole runs, and passes
//!   them to the driver's multi-block path unchanged. No staging buffer, and
//!   no copy.
//!
//! What it does need that the other doesn't is a heap: `resident-fat` keeps
//! the allocation table and the directories it has walked in RAM, and uses
//! `alloc` to do it. So this example registers a `#[global_allocator]` --
//! `rpi-hal` cannot, since a program may have only one and that choice
//! belongs to the final binary. `examples/heap_alloc.rs` covers that setup
//! on its own; the same three steps are repeated here.
//!
//! The mount cost is printed, because it is the resource this approach
//! spends and the number is worth seeing on a real card: four bytes per
//! cluster, for the life of the volume.
//!
//! Output goes to UART0 (PL011) at 115200 8N1.

extern crate alloc;

use core::fmt::Write;

use embedded_alloc::LlffHeap as Heap;
use resident_fat::{Error, FileSystem};
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::sd::{Sd, SdBlockDevice, SdBlockDeviceError};
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;

/// The global heap. See `examples/heap_alloc.rs` for the full explanation:
/// `empty()` is a `const` constructor so this can be a `static`, but it
/// hands out nothing until `init` gives it a region.
#[global_allocator]
static HEAP: Heap = Heap::empty();

extern "C" {
    /// End of the `.bss` section, defined by `linker.ld`. Only its address
    /// is meaningful -- the byte itself is never read.
    static __bss_end: u8;
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// Lists the root directory of the first partition and prints `config.txt`
/// if present. Split out from `kmain` so the filesystem steps can use `?`
/// against one error type.
fn run(sd: Sd, timer: &Timer, uart: &mut Uart) -> Result<(), Error<SdBlockDeviceError>> {
    // Partition 0 is the FAT boot partition on a stock Raspberry Pi card.
    // `mount` would be the call for a card formatted as one bare volume,
    // with no partition table at all.
    let mut fs = FileSystem::mount_partition(SdBlockDevice::new(sd, timer), 0)?;

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

    let _ = writeln!(uart, "\nroot directory:");
    for entry in fs.root_dir()?.iter() {
        let _ = writeln!(
            uart,
            "  {:<12} {:>10} bytes{}",
            entry.name(),
            entry.len(),
            if entry.is_directory() { " <DIR>" } else { "" }
        );
    }

    match fs.open("config.txt") {
        Ok(file) => {
            let _ = writeln!(uart, "\nconfig.txt ({} bytes):", file.len());
            // One call, one run: a contiguous `config.txt` reaches the card
            // as a single `CMD18` rather than a command per block, which is
            // the whole point of the layer under this.
            let contents = fs.read_all(&file)?;
            // config.txt is ASCII text; print it as-is, substituting '.' for
            // any non-printable byte so a stray value can't scramble the
            // terminal.
            for &b in &contents {
                let c = if (0x20..=0x7e).contains(&b) || b == b'\n' || b == b'\r' {
                    b as char
                } else {
                    '.'
                };
                let _ = uart.write_char(c);
            }
            let _ = writeln!(uart);
        }
        Err(Error::NotFound { .. }) => {
            let _ = writeln!(uart, "\nconfig.txt not found");
        }
        Err(e) => return Err(e),
    }

    Ok(())
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // The heap has to exist before the filesystem: mounting allocates the
    // resident allocation table as its first act. See `heap_alloc.rs` on
    // why the region is `.bss`'s end up to the firmware-reported top of the
    // ARM side of the memory split.
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

    let _ = writeln!(uart, "initializing SD card...");
    let sd = match Sd::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, &timer) {
        Ok(sd) => sd,
        Err(e) => {
            let _ = writeln!(uart, "SD init failed: {e:?}");
            halt();
        }
    };
    let _ = writeln!(uart, "SD card ready, mounting FAT filesystem...");

    match run(sd, &timer, &mut uart) {
        Ok(()) => {
            let _ = writeln!(uart, "\ndone");
        }
        Err(e) => {
            let _ = writeln!(uart, "\nFAT read failed: {e:?}");
        }
    }

    halt();
}

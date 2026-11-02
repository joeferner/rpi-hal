#![no_std]
#![no_main]

//! Pi 5 only: brings up the external PCIe connector, finds the NVMe SSD on
//! it, and reads and writes it.
//!
//! In order, printing as it goes:
//!
//! 1. [`Pcie::init`] trains the link (Gen 2, the connector's rating) and
//!    the example lists every function that answered on bus 1.
//! 2. The NVMe controller among them gets its BAR from [`Pcie::enable`],
//!    and [`Nvme::new`] brings it up: model, serial, firmware, block size,
//!    capacity, and the largest transfer one command moves.
//! 3. Block 0, the first 64 bytes in hex, and whether it holds an MBR
//!    (with the protective entry a GPT disk has).
//! 4. Read speed: 256 MiB from block 0 onward, in 2 MiB reads.
//! 5. A write round trip on one block halfway through the drive: read the
//!    block, write a pattern over it, flush, read it back and compare, then
//!    write the original back, flush, and check that too. The drive ends
//!    up as it started, unless power is lost in between — **don't run this
//!    on an SSD whose data matters without a backup.**
//! 6. A normal controller shutdown, so the drive can be powered off.
//!
//! Output goes to the console UART (RP1's UART0) at 115200 8N1. Needs no
//! `config.txt` changes: `dtparam=pciex1` only tells Linux the port is in
//! use, and this sets the port up itself.
//!
//! ```sh
//! ./scripts/build-example64.sh nvme bcm2712
//! ```

use core::fmt::Write;
use core::ptr::addr_of_mut;

use rpi_hal::mailbox::{ClockId, Mailbox};
use rpi_hal::nvme::{self, Nvme};
use rpi_hal::pcie::{LinkSpeed, Pcie};
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::{halt, pac};

/// Bytes the speed test reads.
const SPEED_TEST_BYTES: usize = 256 << 20;

/// The speed test's buffer: one 2 MiB read's worth, page aligned.
#[repr(C, align(4096))]
struct Buffer([u8; 2 << 20]);

static mut BUFFER: Buffer = Buffer([0; 2 << 20]);
static mut MEMORY: nvme::Memory = nvme::Memory::new();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let p = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&p.GPIO, p.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let p = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&p.GPIO, p.UART0);
    let timer = Timer::new(p.SYSTMR);
    let mut mailbox = Mailbox::new(p.VCMAILBOX);
    if let Ok(max) = mailbox.max_clock_rate_hz(ClockId::Arm) {
        let _ = mailbox.set_clock_rate_hz(ClockId::Arm, max);
    }

    let start = timer.now_micros();
    let mut pcie = match Pcie::init(p.PCIE1, LinkSpeed::Gen2, &timer) {
        Ok(pcie) => pcie,
        Err(e) => {
            let _ = writeln!(uart, "nvme: PCIe bring-up failed: {e:?}");
            halt();
        }
    };
    let link = pcie.link();
    let _ = writeln!(
        uart,
        "nvme: PCIe link up in {} ms: Gen {} x{}",
        (timer.now_micros() - start) / 1000,
        link.generation,
        link.width,
    );

    let mut controller = None;
    for function in pcie.functions() {
        let _ = writeln!(
            uart,
            "nvme: {:02x}:{:02x}.{} {:04x}:{:04x} class {:06x}",
            function.address.bus,
            function.address.device,
            function.address.function,
            function.vendor_id,
            function.device_id,
            function.class,
        );
        if function.class == nvme::CLASS && controller.is_none() {
            controller = Some(function);
        }
    }
    let Some(controller) = controller else {
        let _ = writeln!(uart, "nvme: no NVMe controller on the bus");
        halt();
    };
    let function = match pcie.enable(controller) {
        Ok(function) => function,
        Err(e) => {
            let _ = writeln!(uart, "nvme: enabling it failed: {e:?}");
            halt();
        }
    };
    if let Some(bar) = function.bars[0] {
        let _ = writeln!(
            uart,
            "nvme: BAR0 at {:#x}, {} KiB",
            bar.address,
            bar.len / 1024
        );
    }

    // SAFETY: the only reference to either static, made once.
    let memory = unsafe { &mut *addr_of_mut!(MEMORY) };
    let buffer = unsafe { &mut (*addr_of_mut!(BUFFER)).0 };

    let start = timer.now_micros();
    let mut nvme = match Nvme::new(&function, memory, &timer) {
        Ok(nvme) => nvme,
        Err(e) => {
            let _ = writeln!(uart, "nvme: controller bring-up failed: {e:?}");
            halt();
        }
    };
    let block_size = nvme.block_size();
    let _ = writeln!(
        uart,
        "nvme: controller up in {} ms: {:?}, serial {:?}, firmware {:?}",
        (timer.now_micros() - start) / 1000,
        nvme.model(),
        nvme.serial(),
        nvme.firmware(),
    );
    let _ = writeln!(
        uart,
        "nvme: {} blocks of {} bytes ({} GB), up to {} KiB per command",
        nvme.block_count(),
        block_size,
        nvme.block_count() * block_size as u64 / 1_000_000_000,
        nvme.max_transfer() / 1024,
    );

    // Block 0.
    let block = &mut buffer[..block_size];
    if let Err(e) = nvme.read_blocks(0, block, &timer) {
        let _ = writeln!(uart, "nvme: reading block 0 failed: {e:?}");
        halt();
    }
    let _ = write!(uart, "nvme: block 0:");
    for (i, byte) in block[..64].iter().enumerate() {
        if i % 16 == 0 {
            let _ = write!(uart, "\r\n  ");
        }
        let _ = write!(uart, "{byte:02x} ");
    }
    let _ = writeln!(uart);
    let mbr = block[510] == 0x55 && block[511] == 0xaa;
    // The first partition entry's type byte; 0xee is GPT's protective one.
    let _ = writeln!(
        uart,
        "nvme: MBR signature {}, first partition type {:#04x}{}",
        if mbr { "present" } else { "absent" },
        block[446 + 4],
        if mbr && block[446 + 4] == 0xee {
            " (GPT)"
        } else {
            ""
        },
    );

    // Read speed.
    let blocks = (SPEED_TEST_BYTES / block_size) as u64;
    let per_read = (buffer.len() / block_size) as u64;
    let start = timer.now_micros();
    let mut lba = 0;
    while lba < blocks.min(nvme.block_count()) {
        let count = per_read.min(nvme.block_count() - lba) as usize;
        if let Err(e) = nvme.read_blocks(lba, &mut buffer[..count * block_size], &timer) {
            let _ = writeln!(uart, "nvme: read at block {lba} failed: {e:?}");
            halt();
        }
        lba += count as u64;
    }
    let us = (timer.now_micros() - start).max(1);
    let bytes = lba * block_size as u64;
    let _ = writeln!(
        uart,
        "nvme: read {} MiB in {} ms: {} MB/s",
        bytes >> 20,
        us / 1000,
        bytes / us,
    );

    // Write round trip, restoring the block afterwards.
    let lba = nvme.block_count() / 2;
    let (original, rest) = buffer.split_at_mut(block_size);
    let (pattern, readback) = rest.split_at_mut(block_size);
    let readback = &mut readback[..block_size];
    let result = (|| {
        nvme.read_blocks(lba, original, &timer)?;
        for (i, byte) in pattern.iter_mut().enumerate() {
            *byte = (i as u8) ^ 0xa5;
        }
        nvme.write_blocks(lba, pattern, &timer)?;
        nvme.flush(&timer)?;
        nvme.read_blocks(lba, readback, &timer)?;
        let wrote = readback == pattern;
        nvme.write_blocks(lba, original, &timer)?;
        nvme.flush(&timer)?;
        nvme.read_blocks(lba, readback, &timer)?;
        Ok::<_, nvme::Error>((wrote, readback == original))
    })();
    match result {
        Ok((wrote, restored)) => {
            let _ = writeln!(
                uart,
                "nvme: write round trip at block {lba}: pattern {}, original {}",
                if wrote { "read back" } else { "MISMATCH" },
                if restored { "restored" } else { "NOT RESTORED" },
            );
        }
        Err(e) => {
            let _ = writeln!(uart, "nvme: write round trip at block {lba} failed: {e:?}");
        }
    }

    match nvme.shutdown(&timer) {
        Ok(()) => {
            let _ = writeln!(uart, "nvme: controller shut down; done");
        }
        Err(e) => {
            let _ = writeln!(uart, "nvme: shutdown failed: {e:?}");
        }
    }
    halt();
}

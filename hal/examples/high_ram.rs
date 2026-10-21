#![no_std]
#![no_main]

//! The Pi 5's RAM above the first gigabyte, mapped and tested.
//!
//! The firmware reports only the first gigabyte to the ARM, whatever the
//! board's size, and the boot-time map stops at 4 GiB. This asks how much
//! RAM the board has (`mem::ram_bytes`, from its revision code), takes the
//! rest with `mem::high_ram_region` — which maps whatever lies past 4 GiB
//! — and then proves it is RAM: every 8-byte word gets a value made from
//! its own address, and a second pass reads them all back.
//!
//! A value made from the address is what makes the check mean something.
//! A region with nothing behind it faults (an external abort, reported by
//! nothing here, so it stops), and one whose high addresses wrapped onto
//! lower ones — RAM smaller than the revision code says — reads back the
//! value of whichever address was written last, which is the wrong one.
//!
//! Prints the sizes, then each pass's time and rate. Output goes to the
//! console UART (RP1's UART0) at 115200 8N1.

use core::fmt::Write;

use rpi_hal::mailbox::{ClockId, Mailbox};
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::{halt, mem, pac};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// The value the word at `address` should hold: the address scrambled, so
/// that neighbouring words differ in every byte.
fn pattern(address: usize) -> u64 {
    (address as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15)
}

/// MB/s for `bytes` in `us`.
fn rate(bytes: usize, us: u64) -> u64 {
    bytes as u64 / us.max(1)
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
    if let Ok(max) = mailbox.max_clock_rate_hz(ClockId::Arm) {
        let _ = mailbox.set_clock_rate_hz(ClockId::Arm, max);
    }

    let revision = mailbox.board_revision().unwrap_or(0);
    let total = match mem::ram_bytes(&mut mailbox) {
        Ok(total) => total,
        Err(e) => {
            let _ = writeln!(uart, "high_ram: no RAM size: {e:?}");
            halt();
        }
    };
    let _ = writeln!(
        uart,
        "high_ram: revision {revision:#08x}, {} MiB of RAM",
        total >> 20
    );
    if let Ok(low) = mem::heap_region(&mut mailbox) {
        let _ = writeln!(
            uart,
            "high_ram: the firmware's share: {:#x}..{:#x} ({} MiB free above the image)",
            low.start,
            low.end,
            low.len() >> 20
        );
    }

    let high = match mem::high_ram_region(&mut mailbox) {
        Ok(high) => high,
        Err(e) => {
            let _ = writeln!(uart, "high_ram: {e:?}");
            halt();
        }
    };
    let _ = writeln!(
        uart,
        "high_ram: above it: {:#x}..{:#x} ({} MiB)",
        high.start,
        high.end,
        high.len() >> 20
    );
    if high.is_empty() {
        let _ = writeln!(uart, "high_ram: nothing above the first gigabyte; done");
        halt();
    }

    // One GiB at a time, so a board that stops says where.
    const STEP: usize = 1 << 30;
    let words = |base: usize, len: usize| {
        // SAFETY: inside `high`, which `high_ram_region` mapped as RAM and
        // which nothing else in this program uses.
        unsafe { core::slice::from_raw_parts_mut(base as *mut u64, len / 8) }
    };

    let started = timer.now_micros();
    for base in high.clone().step_by(STEP) {
        let len = STEP.min(high.end - base);
        for (i, word) in words(base, len).iter_mut().enumerate() {
            *word = pattern(base + i * 8);
        }
        let _ = writeln!(uart, "high_ram: wrote {:#x}..{:#x}", base, base + len);
    }
    let write_us = timer.now_micros() - started;

    let started = timer.now_micros();
    let mut wrong = 0usize;
    let mut first_wrong = None;
    for base in high.clone().step_by(STEP) {
        let len = STEP.min(high.end - base);
        for (i, word) in words(base, len).iter().enumerate() {
            let address = base + i * 8;
            if *word != pattern(address) {
                wrong += 1;
                first_wrong.get_or_insert(address);
            }
        }
    }
    let read_us = timer.now_micros() - started;

    let _ = writeln!(
        uart,
        "high_ram: write {} ms ({} MB/s), read back {} ms ({} MB/s)",
        write_us / 1000,
        rate(high.len(), write_us),
        read_us / 1000,
        rate(high.len(), read_us)
    );
    match first_wrong {
        None => {
            let _ = writeln!(
                uart,
                "high_ram: PASS: all {} MiB read back as written",
                high.len() >> 20
            );
        }
        Some(address) => {
            let _ = writeln!(
                uart,
                "high_ram: FAIL: {wrong} words wrong, the first at {address:#x}"
            );
        }
    }
    halt();
}

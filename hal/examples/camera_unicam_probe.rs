//! Unicam CSI-2 receiver liveness probe (first bring-up step).
//!
//! Before building the full capture path, this answers the one risky
//! unknown: is the Unicam peripheral's power/clock domain live enough that
//! its MMIO registers can be read at all? Per the rpi-open-firmware notes,
//! touching Unicam registers while its digital domain is unpowered can
//! fault or hang the bus — so this enables the peripheral clock first, then
//! reads a few registers and prints them. If it prints plausible values,
//! MMIO works and the full receiver bring-up (`src/unicam.rs`, later) is
//! just a matter of following the documented sequence. If the board hangs
//! right after "reading Unicam registers", the domain isn't powered and
//! that has to be solved first.
//!
//! Scope: this does NOT configure or start the receiver. The clock is
//! brought up from the 19.2 MHz oscillator (a deterministic, known rate)
//! purely to make register access valid — real capture needs a faster
//! parent (Linux uses 100 MHz), handled when the capture path is written.
//!
//! Register facts (offsets, clock manager layout, lane-gate address) are
//! from Linux's `bcm2835-unicam.c` / `vc4-regs-unicam.h` and the
//! clock-manager driver `clk-bcm2835.c`; Unicam is not in the public
//! BCM2835 datasheet.

#![no_std]
#![no_main]

use core::fmt::Write;
use core::ptr::{read_volatile, write_volatile};
use rpi_hal::halt;
use rpi_hal::timer::Timer;
use rpi_hal::{pac, uart::Uart};

/// Clock manager (CPRMAN) base: peripheral base + 0x10_1000.
const CPRMAN_BASE: usize = 0x3f10_1000;
/// CM_CAM1CTL — control for the Unicam1 clock (`BCM2835_CLOCK_CAM1`).
const CM_CAM1CTL: *mut u32 = (CPRMAN_BASE + 0x48) as *mut u32;
/// CM_CAM1DIV — divider for the Unicam1 clock.
const CM_CAM1DIV: *mut u32 = (CPRMAN_BASE + 0x4c) as *mut u32;
/// Password OR'd into the top byte of every clock-manager register write;
/// a write without it is silently ignored.
const CM_PASSWORD: u32 = 0x5a00_0000;
/// CM control: clock enable.
const CM_ENABLE: u32 = 1 << 4;
/// CM control: clock generator busy (running). Poll after enable/disable.
const CM_BUSY: u32 = 1 << 7;
/// Clock source 1 = the 19.2 MHz crystal oscillator.
const CM_SRC_OSC: u32 = 1;

/// The per-lane clock-gate register (a separate MMIO word from the main
/// Unicam block), shared across the Unicam instances. Also password-gated.
const UNICAM_LANE_GATE: *mut u32 = 0x3f80_2004 as *mut u32;
/// Lane-gate pattern for a 2-data-lane setup: clock lane + two data lanes,
/// two enable bits each (`1`, then `<<2|1` per data lane → 0b010101).
const LANE_GATE_2LANE: u32 = 0x15;

/// Unicam1 register block base: peripheral base + 0x80_1000.
const UNICAM1_BASE: usize = 0x3f80_1000;
/// Control register offset.
const UNICAM_CTRL: usize = 0x000;
/// Status register offset.
const UNICAM_STA: usize = 0x004;
/// Analog/D-PHY control register offset.
const UNICAM_ANA: usize = 0x008;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);

    // Enable the Unicam1 clock from the oscillator. Follow the clock
    // manager's change protocol: gate the clock and wait for it to stop
    // (BUSY low), set the divider, then set source + enable.
    let _ = writeln!(uart, "enabling CM_CAM1 clock...");
    unsafe {
        // Gate (clear enable, keep password) and wait until not busy.
        write_volatile(CM_CAM1CTL, CM_PASSWORD);
        while read_volatile(CM_CAM1CTL) & CM_BUSY != 0 {
            core::hint::spin_loop();
        }
        // Integer divider 1 (DIVI in bits [23:12]) → straight 19.2 MHz.
        write_volatile(CM_CAM1DIV, CM_PASSWORD | (1 << 12));
        // Source = oscillator, then enable.
        write_volatile(CM_CAM1CTL, CM_PASSWORD | CM_SRC_OSC);
        write_volatile(CM_CAM1CTL, CM_PASSWORD | CM_SRC_OSC | CM_ENABLE);
        while read_volatile(CM_CAM1CTL) & CM_BUSY == 0 {
            core::hint::spin_loop();
        }
        // Enable the clock lanes' gate.
        write_volatile(UNICAM_LANE_GATE, CM_PASSWORD | LANE_GATE_2LANE);
    }
    timer.delay_ms(1);
    let _ = writeln!(uart, "clock enabled (CM_CAM1CTL = {:#010x})", unsafe {
        read_volatile(CM_CAM1CTL)
    });

    // The moment of truth: read Unicam MMIO. If the digital domain is
    // unpowered this may hang or fault — hence the announce line first, so
    // a hang localises here.
    let _ = writeln!(
        uart,
        "reading Unicam registers (may hang if domain unpowered)..."
    );
    unsafe {
        let ctrl = read_volatile((UNICAM1_BASE + UNICAM_CTRL) as *const u32);
        let sta = read_volatile((UNICAM1_BASE + UNICAM_STA) as *const u32);
        let ana = read_volatile((UNICAM1_BASE + UNICAM_ANA) as *const u32);
        let _ = writeln!(uart, "  UNICAM_CTRL (0x000) = {ctrl:#010x}");
        let _ = writeln!(uart, "  UNICAM_STA  (0x004) = {sta:#010x}");
        let _ = writeln!(uart, "  UNICAM_ANA  (0x008) = {ana:#010x}");

        // A crude write/read-back check that the block is truly live: the
        // reset bit (CTRL.CPR, bit 2) is safe to pulse. Set it, read it
        // back, then clear it.
        write_volatile((UNICAM1_BASE + UNICAM_CTRL) as *mut u32, 1 << 2);
        let after = read_volatile((UNICAM1_BASE + UNICAM_CTRL) as *const u32);
        write_volatile((UNICAM1_BASE + UNICAM_CTRL) as *mut u32, 0);
        let _ = writeln!(uart, "  CTRL after pulsing reset bit = {after:#010x}");
    }

    let _ = writeln!(uart, "Unicam MMIO readable — power/clock domain is live.");
    halt();
}

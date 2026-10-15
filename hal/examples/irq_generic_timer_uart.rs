#![no_std]
#![no_main]

//! ARM generic-timer interrupt with UART progress output -- the per-core
//! architected timer (`generic_timer.rs`) in place of the shared BCM
//! System Timer used by `irq_timer_uart`. Proves the second interrupt
//! path end to end: generic-timer comparator -> ARM-local interrupt
//! controller routing (`GenericTimer::route_irq`) -> vector dispatch ->
//! return-from-exception. Unlike `irq_timer_uart`, the interrupt is routed
//! through the *per-core* controller rather than the legacy LIC.
//!
//! Scheduling uses the drift-free absolute-deadline primitive
//! (`set_deadline` against a 64-bit `now()` count) rather than a relative
//! reload: each tick advances the deadline by a fixed period, pinning the
//! cadence to the counter instead of to when the handler happened to run
//! -- the shape an interrupt-driven executor (Embassy) will build on. Each
//! tick prints the live `now()` count so the counter can be seen
//! advancing.

use core::fmt::Write;
use core::ptr::{addr_of, addr_of_mut};
use rpi_hal::{generic_timer::GenericTimer, irq, pac, uart::Uart};

const PERIOD_US: u32 = 500_000;

// Plain volatile, not atomics: written only by the IRQ handler (which
// can't re-enter itself), so no exclusive-access read-modify-write is
// needed -- see irq_timer_blink.rs for the full reasoning. PERIOD_TICKS is
// written once in kmain before the IRQ is enabled, then only read.
static mut TICKS: u32 = 0;
static mut PERIOD_TICKS: u64 = 0;
// Absolute count the next tick fires at; advanced by PERIOD_TICKS each
// interrupt so the cadence never drifts with handler latency.
static mut NEXT_DEADLINE: u64 = 0;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let p = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&p.GPIO, p.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    loop {
        unsafe { core::arch::asm!("wfe") };
    }
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let p = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&p.GPIO, p.UART0);
    let _ = writeln!(uart, "irq_generic_timer_uart: start");

    let gt = GenericTimer::new();
    let freq = gt.frequency();
    let _ = writeln!(uart, "counter frequency: {freq} Hz");

    // Anchor the first deadline to the live count, then hand both it and
    // the fixed period to the handler through statics.
    let period_ticks = (PERIOD_US as u64 * freq as u64) / 1_000_000;
    let first_deadline = gt.now() + period_ticks;
    unsafe {
        core::ptr::write_volatile(addr_of_mut!(PERIOD_TICKS), period_ticks);
        core::ptr::write_volatile(addr_of_mut!(NEXT_DEADLINE), first_deadline);
    }
    gt.set_deadline(first_deadline);
    let _ = writeln!(uart, "timer armed at count {first_deadline}");

    gt.route_irq();
    let _ = writeln!(uart, "irq routed to this core");

    irq::enable_irq();
    let _ = writeln!(uart, "irq enabled; waiting for ticks");

    let mut last = 0u32;
    loop {
        let now = unsafe { core::ptr::read_volatile(addr_of!(TICKS)) };
        if now != last {
            last = now;
            let _ = writeln!(uart, "tick {now} @ count {}", gt.now());
        }
        unsafe { core::arch::asm!("wfe") };
    }
}

#[no_mangle]
pub extern "C" fn __irq_handler() {
    let gt = GenericTimer::new();
    // Advance the deadline by one whole period and re-arm. Setting a
    // comparator ahead of the current count both acks the level-sensitive
    // tick just taken (deasserts ISTATUS) and schedules the next one.
    let next = unsafe {
        let period = core::ptr::read_volatile(addr_of!(PERIOD_TICKS));
        let next = core::ptr::read_volatile(addr_of!(NEXT_DEADLINE)).wrapping_add(period);
        core::ptr::write_volatile(addr_of_mut!(NEXT_DEADLINE), next);
        next
    };
    gt.set_deadline(next);
    unsafe {
        let ticks = core::ptr::read_volatile(addr_of!(TICKS)).wrapping_add(1);
        core::ptr::write_volatile(addr_of_mut!(TICKS), ticks);
    }
}

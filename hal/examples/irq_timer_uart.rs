#![no_std]
#![no_main]

//! Timer interrupt with UART progress output -- the same interrupt chain
//! as `irq_timer_blink` (timer -> LIC routing -> vector dispatch ->
//! return-from-exception) but narrating each step over UART instead of
//! toggling an LED, so the path can be diagnosed without a scope or a
//! wired LED. Useful for bringing the interrupt path up on a new
//! architecture (e.g. AArch64's vectors64.s trampoline + DAIF unmask).

use core::fmt::Write;
use core::ptr::{addr_of, addr_of_mut};
use rpi_hal::{irq, lic::Lic, pac, timer::Timer, uart::Uart};

const PERIOD_US: u32 = 500_000;

// Plain volatile, not AtomicU32: only the IRQ handler writes it (it can't
// re-enter itself), so no exclusive-access read-modify-write is needed --
// see irq_timer_blink.rs for the full reasoning.
static mut TICKS: u32 = 0;

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
    let _ = writeln!(uart, "irq_timer_uart: start");

    let timer = Timer::new(p.SYSTMR);
    timer.arm_periodic_c1(PERIOD_US);
    let _ = writeln!(uart, "timer armed");

    let lic = Lic::new(p.LIC);
    lic.enable_timer1_irq();
    let _ = writeln!(uart, "lic routed");

    irq::enable_irq();
    let _ = writeln!(uart, "irq enabled; waiting for ticks");

    let mut last = 0u32;
    loop {
        let now = unsafe { core::ptr::read_volatile(addr_of!(TICKS)) };
        if now != last {
            last = now;
            let _ = writeln!(uart, "tick {now}");
        }
        unsafe { core::arch::asm!("wfe") };
    }
}

#[no_mangle]
pub extern "C" fn __irq_handler() {
    let p = unsafe { pac::Peripherals::steal() };
    let timer = Timer::new(p.SYSTMR);
    timer.ack_c1(PERIOD_US);
    unsafe {
        let next = core::ptr::read_volatile(addr_of!(TICKS)).wrapping_add(1);
        core::ptr::write_volatile(addr_of_mut!(TICKS), next);
    }
}

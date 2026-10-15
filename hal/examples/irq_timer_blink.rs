#![no_std]
#![no_main]

// Same LED as blink.rs, but toggled entirely from the IRQ handler —
// proves the whole interrupt chain end-to-end: vector table dispatch
// (vectors.s) -> LIC routing (lic.rs) -> peripheral-level ack
// (Timer::ack_c1) -> correct return-from-exception. kmain's own loop
// does nothing but sleep.

use core::ptr::{addr_of, addr_of_mut};
use embedded_hal::digital::OutputPin;
use rpi_hal::gpio::{Input, Output, Pin};
use rpi_hal::{irq, lic::Lic, pac, timer::Timer};

const PERIOD_US: u32 = 500_000;

// Plain volatile static, not `AtomicBool`: only ever touched from the
// IRQ handler (which can't re-enter itself — IRQ auto-masks during
// handling), so there's no concurrent-writer case needing true atomic
// read-modify-write. That matters here because `AtomicBool`'s
// load/store compile to ldrex/strex, and exclusive-access instructions
// are architecturally UNPREDICTABLE on Strongly-Ordered/Device memory —
// which is what memory defaults to with the MMU disabled (this hung
// for real on hardware before switching to plain volatile access). A
// plain volatile load/store of an aligned word is already atomic on
// ARM at the instruction level, no exclusive monitor needed.
static mut LED_ON: bool = false;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        unsafe { core::arch::asm!("wfe") };
    }
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    // Only configuring the pin here; `__irq_handler` re-wraps it via
    // `assume_mode` each time rather than holding onto this value,
    // same as it already does for `Timer`/`Lic`.
    Pin::<4, Input>::new(peripherals.GPIO).into_output();

    let timer = Timer::new(peripherals.SYSTMR);
    timer.arm_periodic_c1(PERIOD_US);

    let lic = Lic::new(peripherals.LIC);
    lic.enable_timer1_irq();

    irq::enable_irq();

    loop {
        unsafe { core::arch::asm!("wfe") };
    }
}

#[no_mangle]
pub extern "C" fn __irq_handler() {
    let peripherals = unsafe { pac::Peripherals::steal() };

    let timer = Timer::new(peripherals.SYSTMR);
    timer.ack_c1(PERIOD_US);

    let on = unsafe {
        let on = !core::ptr::read_volatile(addr_of!(LED_ON));
        core::ptr::write_volatile(addr_of_mut!(LED_ON), on);
        on
    };

    // Safe: `kmain` already configured pin 4 as an output before
    // enabling the IRQ that leads here.
    let mut led = unsafe { Pin::<4, Output>::assume_mode(peripherals.GPIO) };
    let _ = if on { led.set_high() } else { led.set_low() };
}

#![no_std]
#![no_main]

// Two cores, two LEDs: core 0 blinks GPIO4 (this crate's usual "LED"
// pin, see blink.rs) at 1Hz from its own loop, while core 1 -- spawned
// via `multicore::Cores::steal`/`Core1::spawn` -- independently blinks
// GPIO27 at a different rate out of its own copy of the loop. The two
// visibly-different rates confirm core 1 is actually alive and running
// on its own, not just linked in.
//
// Both cores also narrate their own progress over UART0. Core 0 brings
// it up (`Uart::init`) before spawning core 1, which only ever wraps
// the already-initialized peripheral (`Uart::from_initialized`) --
// otherwise both cores would race to reconfigure the same GPIO
// mux/baud-rate registers.
//
// Both delays are off the shared 1MHz System Timer (`Timer`), a real
// wall-clock source, not a spin of N `nop`s: the CPU clock is
// ~900MHz-1.2GHz and unknown at build time, so a fixed instruction
// count blinks at an unpredictable (and, human-visibly, far too fast)
// rate. The timer is a free-running counter both cores only *read*, so
// each core independently steals its own `SYSTMR` handle -- no shared
// mutable state between them.

use core::fmt::Write;
use core::ptr::addr_of_mut;
use embedded_hal::digital::OutputPin;
use rpi_hal::gpio::{Input, Pin};
use rpi_hal::multicore::{Cores, Stack};
use rpi_hal::pac;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    loop {
        unsafe { core::arch::asm!("wfe") };
    }
}

// 8KB: half for this example's own (trivial) main-mode stack use, half
// for the IRQ-mode region `spawn` carves off the top (unused here,
// since this example never enables IRQ on core 1, but `Stack::new`'s
// compile-time check requires room for it regardless).
static mut CORE1_STACK: Stack<8192> = Stack::new();

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "core0: kmain entered, uart up");

    let _ = writeln!(uart, "core0: spawning core1");
    // Safe: this is the only place `Cores::steal` is called, and
    // `core1` is handed off to `spawn` immediately, never reused.
    let cores = unsafe { Cores::steal() };
    unsafe {
        cores
            .core1
            .spawn(&mut *addr_of_mut!(CORE1_STACK), core1_main);
    }
    let _ = writeln!(uart, "core0: core1 spawned, blinking");

    // `peripherals.GPIO` was moved into `Uart::init`'s borrow above but
    // not consumed; re-steal a fresh token for the `Pin`, same as
    // `spi_loopback.rs`/`uart_debug.rs` do.
    let mut led = Pin::<4, Input>::new(unsafe { pac::GPIO::steal() }).into_output();
    let timer = Timer::new(peripherals.SYSTMR);

    let mut count: u32 = 0;
    loop {
        let _ = led.set_high();
        timer.delay_ms(500);
        let _ = led.set_low();
        timer.delay_ms(500);
        let _ = writeln!(uart, "core0: alive ({count})");
        count += 1;
    }
}

extern "C" fn core1_main() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    // Not `Uart::init` -- see this file's module doc.
    let mut uart = Uart::from_initialized(peripherals.UART0);
    let _ = writeln!(uart, "core1: alive");

    let mut led = Pin::<27, Input>::new(peripherals.GPIO).into_output();
    let timer = Timer::new(peripherals.SYSTMR);

    let mut count: u32 = 0;
    loop {
        let _ = led.set_high();
        timer.delay_ms(300);
        let _ = led.set_low();
        timer.delay_ms(300);
        let _ = writeln!(uart, "core1: alive ({count})");
        count += 1;
    }
}

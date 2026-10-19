#![no_std]
#![no_main]

//! Pi 5 only: drives the fan connector from RP1's PWM1 and reads its speed
//! back.
//!
//! The fan runs flat out from power-on, because the firmware leaves its PWM
//! pin (GPIO45) with its output disabled, and the pad's pull-down reads to
//! the fan as full speed.
//! This takes the pin over and steps the fan through off, 25%, 50% and
//! 100%, five seconds each, printing once a second:
//!
//! - the speed asked for,
//! - [`Pwm::fan_rpm`], the RPM value the board's device tree tells Linux to
//!   read (see that method for where it comes from),
//! - the SoC temperature, from the firmware.
//!
//! The fan takes the signal inverted (low is fast), so the channel is
//! inverted and a duty fraction is a speed fraction. The period is Linux's,
//! 41,566 ns (about 24kHz): 2,078 ticks of the undivided 50MHz clock.
//!
//! Before taking the pin over it prints GPIO45's function select and pad
//! register as the firmware left them, which is what says whether anything
//! was driving the fan before this ran. On a Pi 5 that reads `FUNCSEL=0x5`
//! (software GPIO) and `pad=0xd6`: bit 7 set, the output buffer disabled,
//! and bit 2, the pull-down.
//!
//! Expect the 25% step to read 0 RPM for a few seconds: the fan is slow to
//! start from a standstill at low duty (see [`rpi_hal::pwm`]).
//!
//! ```sh
//! ./scripts/build-example64.sh pwm_fan bcm2712
//! ```

use core::fmt::Write;

use embedded_hal::pwm::SetDutyCycle;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::pwm::{Pwm, Pwm1Pin};
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;

/// PWM clock ticks per period: Linux's 41,566 ns at 50MHz.
const FAN_RANGE: u16 = 2078;

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
    let timer = Timer::new(p.SYSTMR);
    let mut mailbox = Mailbox::new(p.VCMAILBOX);

    // GPIO45 is index 11 of RP1's bank 2, whose `IO_BANK` and `PADS_BANK`
    // blocks sit 0x8000 above bank 0's. `CTRL` is the second word of the
    // pin's 8-byte slot; its low five bits are `FUNCSEL` (0x1f = none).
    // The pad register's bit 2 is the pull-down, bit 3 the pull-up.
    let (ctrl, pad) = unsafe {
        let ctrl = (pac::GPIO::IO_BANK0 + 0x8000 + 8 * 11 + 4) as *const u32;
        let pad = (pac::GPIO::PADS_BANK0 + 0x8000 + 4 + 4 * 11) as *const u32;
        (ctrl.read_volatile(), pad.read_volatile())
    };
    let _ = writeln!(
        uart,
        "pwm_fan: GPIO45 as found: FUNCSEL={:#x} pad={:#x} (pull-down {}, pull-up {})",
        ctrl & 0x1f,
        pad,
        pad & (1 << 2) != 0,
        pad & (1 << 3) != 0,
    );

    let pwm = Pwm::init(p.PWM1, 1);
    let _ = writeln!(
        uart,
        "pwm_fan: PWM1 clock {} Hz, period {} ticks",
        pwm.clock_rate_hz(),
        FAN_RANGE
    );
    let mut fan = pwm.channel(&p.GPIO, Pwm1Pin::Gpio45, FAN_RANGE);
    fan.set_inverted();

    loop {
        for percent in [0u8, 25, 50, 100] {
            let _ = fan.set_duty_cycle_percent(percent);
            for _ in 0..5 {
                timer.delay_ms(1000);
                let temp = mailbox.temperature_millicelsius().unwrap_or(0);
                let _ = writeln!(
                    uart,
                    "fan {percent:>3}%  rpm {:>5}  soc {}.{} C",
                    pwm.fan_rpm(),
                    temp / 1000,
                    temp % 1000 / 100,
                );
            }
        }
    }
}

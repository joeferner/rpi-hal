//! Prints the SoC's die temperature, the ARM clock, and the firmware's
//! throttling status once a second.
//!
//! The three belong together, which is the point of the example. The
//! firmware caps the ARM clock as the die heats up, so a program that
//! watches only its own progress sees thermal throttling as its code
//! inexplicably getting slower — a frame budget that was comfortable at
//! minute one and is not at minute ten, with nothing in the program
//! having changed. Watching all three at once is what makes that visible:
//! load the board and the clock line follows the temperature line down.
//!
//! The throttling word has two halves, and only one of them is about
//! right now. Bits 0-3 say what is happening at this instant; bits 16-19
//! are sticky since boot, and they are the only way to see an
//! under-voltage dip that has already passed — a marginal supply or a
//! long USB cable shows up there long after the board has recovered and
//! looks healthy.
//!
//! All three come from the VideoCore firmware over the mailbox rather
//! than from a register: the thermal sensor and the clock manager are the
//! firmware's, and it is the thing doing the capping.
#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::halt;
use rpi_hal::mailbox::{
    ClockId, Mailbox, THROTTLED_ARM_FREQ_CAPPED, THROTTLED_ARM_FREQ_CAPPED_EVER, THROTTLED_EVER,
    THROTTLED_NOW, THROTTLED_SOFT_TEMP_LIMIT, THROTTLED_SOFT_TEMP_LIMIT_EVER,
    THROTTLED_UNDER_VOLTAGE, THROTTLED_UNDER_VOLTAGE_EVER,
};
use rpi_hal::{pac, timer::Timer, uart::Uart};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// The "happening now" half of the throttling word, in bit order.
const NOW: [(u32, &str); 4] = [
    (THROTTLED_UNDER_VOLTAGE, "under-voltage"),
    (THROTTLED_ARM_FREQ_CAPPED, "ARM clock capped"),
    (THROTTLED_NOW, "throttled"),
    (THROTTLED_SOFT_TEMP_LIMIT, "soft temperature limit"),
];

/// The sticky half: the same four conditions, latched since boot.
const EVER: [(u32, &str); 4] = [
    (THROTTLED_UNDER_VOLTAGE_EVER, "under-voltage"),
    (THROTTLED_ARM_FREQ_CAPPED_EVER, "ARM clock capped"),
    (THROTTLED_EVER, "throttled"),
    (THROTTLED_SOFT_TEMP_LIMIT_EVER, "soft temperature limit"),
];

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    let _ = writeln!(uart, "-- SoC temperature, ARM clock, throttling --");

    loop {
        match mailbox.temperature_millicelsius() {
            // Thousandths of a degree, printed to one decimal place --
            // the sensor's own resolution is coarser than that, and a
            // temperature quoted to three decimals invites reading
            // precision into it that isn't there.
            Ok(milli) => {
                let _ = write!(uart, "temp {}.{} C", milli / 1000, (milli % 1000) / 100);
            }
            Err(e) => {
                let _ = write!(uart, "temp error {e:?}");
            }
        }

        match mailbox.clock_rate_hz(ClockId::Arm) {
            Ok(hz) => {
                let _ = write!(uart, "   ARM {} MHz", hz / 1_000_000);
            }
            Err(e) => {
                let _ = write!(uart, "   ARM error {e:?}");
            }
        }

        match mailbox.throttled() {
            Ok(word) => {
                let _ = write!(uart, "   now: ");
                write_flags(&mut uart, word, &NOW);
                let _ = write!(uart, "   since boot: ");
                write_flags(&mut uart, word, &EVER);
                let _ = writeln!(uart);
            }
            Err(e) => {
                let _ = writeln!(uart, "   throttled error {e:?}");
            }
        }

        timer.delay_ms(1000);
    }
}

/// Writes the names of whichever `flags` are set in `word`, or `none`.
fn write_flags(uart: &mut Uart, word: u32, flags: &[(u32, &str)]) {
    let mut first = true;
    for (bit, name) in flags {
        if word & bit != 0 {
            let _ = write!(uart, "{}{name}", if first { "" } else { ", " });
            first = false;
        }
    }
    if first {
        let _ = write!(uart, "none");
    }
}

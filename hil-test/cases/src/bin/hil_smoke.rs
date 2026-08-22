//! The smoke tier: what a board can prove about itself with nothing attached
//! but a serial cable.
//!
//! Every case here is self-checking, so this binary is the whole `hil-smoke`
//! target — a drive-by contributor with one Pi and a USB-serial adapter gets
//! real signal from it without owning any of the bench.

#![no_std]
#![no_main]

use hil_cases::{hil_panic_handler, Session};
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::timer::Timer;

hil_panic_handler!();

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let mut session = Session::start(4);

    let peripherals = unsafe { pac::Peripherals::steal() };
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // The System Timer must advance. Reading the same value twice across a
    // delay means the counter is dead, which would silently break every
    // timing assertion the rest of the suite makes.
    let first = timer.now_micros();
    timer.delay_us(1_000);
    let second = timer.now_micros();
    session.check(
        "timer_advances",
        second > first,
        "counter did not move across a 1ms delay",
    );

    // And it must advance at roughly the right rate. A counter ticking at
    // the wrong frequency still passes the test above while making every
    // measured duration wrong, so the rate needs its own check. The window is
    // deliberately loose: this is catching a broken clock source, not
    // measuring jitter, and the delay itself is not calibrated.
    let start = timer.now_micros();
    timer.delay_us(50_000);
    let elapsed = timer.now_micros().wrapping_sub(start);
    session.check(
        "timer_rate",
        (40_000..=60_000).contains(&elapsed),
        "50ms delay measured outside the 40-60ms window",
    );

    // The mailbox round-trips. This is the channel the banner's board
    // revision came from, so if it were broken the runner could not trust
    // which board it is talking to in the first place.
    match mailbox.firmware_revision() {
        Ok(rev) if rev != 0 => session.check("mailbox_firmware_revision", true, ""),
        Ok(_) => session.check(
            "mailbox_firmware_revision",
            false,
            "firmware reported revision 0",
        ),
        Err(_) => session.check("mailbox_firmware_revision", false, "mailbox call failed"),
    }

    // A second, different mailbox property, because one working call can be a
    // stale buffer rather than a working channel.
    match mailbox.board_revision() {
        Ok(rev) if rev != 0 => session.check("mailbox_board_revision", true, ""),
        Ok(_) => session.check("mailbox_board_revision", false, "board revision read as 0"),
        Err(_) => session.check("mailbox_board_revision", false, "mailbox call failed"),
    }

    session.finish()
}

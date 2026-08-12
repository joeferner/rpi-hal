//! GPIO pin wrapper with compile-time input/output mode enforcement
//! (typestate) and `embedded-hal` `digital` trait implementations.

use crate::pac::GPIO;
use core::marker::PhantomData;

#[cfg(feature = "async")]
mod asynch;
#[cfg(feature = "async")]
pub use asynch::on_irq;

/// Marker type: pin configured as output. See [`Pin`].
pub struct Output;

/// Marker type: pin configured as input. See [`Pin`].
pub struct Input;

/// Which pin condition the BCM2836/2837 event-detect hardware latches
/// into GPEDS — and, once the pin's bank IRQ is routed through the
/// interrupt controller, raises a CPU interrupt for. Selected per pin
/// via [`Pin::enable_interrupt`]; only one trigger is active at a time
/// (enabling a new one replaces the previous).
///
/// The `Rising`/`Falling`/`Any` edge variants use the *synchronous*
/// detectors (GPREN/GPFEN): the pin is sampled through a two-cycle
/// debounce synchronizer before an edge is recognized, so glitches
/// shorter than that are filtered out. The `Async*` variants use the
/// asynchronous detectors (GPAREN/GPAFEN), which bypass the synchronizer
/// and so can catch pulses too short for the synchronous path — at the
/// cost of that debounce. The level variants (GPHEN/GPLEN) hold the
/// interrupt asserted for as long as the pin sits at the given level, so
/// a handler must either change the condition or mask the source, not
/// just ack GPEDS, to stop it re-firing.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Trigger {
    /// A low→high transition (synchronous, GPREN).
    RisingEdge,
    /// A high→low transition (synchronous, GPFEN).
    FallingEdge,
    /// Either transition (synchronous, GPREN + GPFEN).
    AnyEdge,
    /// The pin sitting high (level-sensitive, GPHEN).
    HighLevel,
    /// The pin sitting low (level-sensitive, GPLEN).
    LowLevel,
    /// A low→high transition, detected asynchronously (GPAREN).
    AsyncRisingEdge,
    /// A high→low transition, detected asynchronously (GPAFEN).
    AsyncFallingEdge,
    /// Either transition, detected asynchronously (GPAREN + GPAFEN).
    AsyncAnyEdge,
}

/// A single GPIO pin (0-53), typestated on whether it's currently
/// `Input` or `Output` — `into_output`/`into_input` are the only way
/// to change mode, and `embedded_hal::digital`'s traits are only
/// implemented for the mode they apply to, so e.g. `set_high` doesn't
/// compile against a pin still in `Input` mode.
pub struct Pin<const N: u8, MODE> {
    gpio: GPIO,
    _mode: PhantomData<MODE>,
}

impl<const N: u8> Pin<N, Input> {
    /// Wraps an already-stolen `GPIO` token as pin `N`. Returned in
    /// `Input` mode — this hardware's actual GPFSEL reset-default
    /// state, not an assumption this type imposes; call `into_output`
    /// if you need it configured as an output.
    pub fn new(gpio: GPIO) -> Self {
        const { Self::VALID_PIN };
        Self {
            gpio,
            _mode: PhantomData,
        }
    }
}

impl<const N: u8, MODE> Pin<N, MODE> {
    /// Compile-time check that `N` is a real pin on this SoC —
    /// evaluated (forced) from `Pin::new`, so an out-of-range `N`
    /// fails to build rather than misbehaving at runtime.
    const VALID_PIN: () = assert!(N <= 53, "BCM2836/2837 only has GPIO pins 0..=53");

    /// Reconfigures this pin as an output (GPFSEL = 001).
    pub fn into_output(self) -> Pin<N, Output> {
        set_fsel(&self.gpio, N, FSEL_OUTPUT);
        Pin {
            gpio: self.gpio,
            _mode: PhantomData,
        }
    }

    /// Reconfigures this pin as an input (GPFSEL = 000).
    pub fn into_input(self) -> Pin<N, Input> {
        set_fsel(&self.gpio, N, FSEL_INPUT);
        Pin {
            gpio: self.gpio,
            _mode: PhantomData,
        }
    }

    /// Wraps an already-stolen `GPIO` token as pin `N` in mode `MODE`,
    /// without touching hardware — unlike `into_output`/`into_input`,
    /// this doesn't reconfigure GPFSEL. Needed for contexts (like an
    /// IRQ handler) that only have a freshly-stolen `GPIO` token, not
    /// the original `Pin` `kmain` already configured; mirrors
    /// `Timer::new`/`Uart::from_initialized`'s existing shape.
    ///
    /// # Safety
    ///
    /// `MODE` must correctly describe pin `N`'s actual current GPFSEL
    /// configuration — the compiler can't check this the way it can
    /// for `into_output`/`into_input`, since there's no register access
    /// here to attach that guarantee to.
    pub unsafe fn assume_mode(gpio: GPIO) -> Self {
        const { Self::VALID_PIN };
        Self {
            gpio,
            _mode: PhantomData,
        }
    }
}

impl<const N: u8> embedded_hal::digital::ErrorType for Pin<N, Output> {
    /// Infallible — every operation here is a direct, always-succeeding
    /// register access.
    type Error = core::convert::Infallible;
}

impl<const N: u8> embedded_hal::digital::OutputPin for Pin<N, Output> {
    /// Drives the pin low.
    fn set_low(&mut self) -> Result<(), Self::Error> {
        set_level(&self.gpio, N, false);
        Ok(())
    }

    /// Drives the pin high.
    fn set_high(&mut self) -> Result<(), Self::Error> {
        set_level(&self.gpio, N, true);
        Ok(())
    }
}

impl<const N: u8> embedded_hal::digital::StatefulOutputPin for Pin<N, Output> {
    /// Reads back the pin's current *output* level (GPLEV reflects the
    /// driven level for output-configured pins, not just input signals).
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(read_level(&self.gpio, N))
    }

    /// The inverse of `is_set_high`.
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!read_level(&self.gpio, N))
    }
}

impl<const N: u8> embedded_hal::digital::ErrorType for Pin<N, Input> {
    /// Infallible — every operation here is a direct, always-succeeding
    /// register access.
    type Error = core::convert::Infallible;
}

impl<const N: u8> embedded_hal::digital::InputPin for Pin<N, Input> {
    /// True if the pin currently reads high.
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(read_level(&self.gpio, N))
    }

    /// The inverse of `is_high`.
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!read_level(&self.gpio, N))
    }
}

impl<const N: u8> Pin<N, Input> {
    /// Enables event detection on this pin for `trigger`, replacing any
    /// trigger previously set here. A matching event latches into GPEDS
    /// and — once the pin's bank IRQ is routed through the interrupt
    /// controller (`Lic::enable_gpio_irq`)
    /// and the CPU IRQ mask is open ([`enable_irq`](crate::irq::enable_irq))
    /// — raises an interrupt. Those are the same three independent gates
    /// every other source in this crate goes through; this method only
    /// opens the peripheral one, exactly as `Uart::enable_rx_irq` does.
    ///
    /// The handler must ack the event with [`clear_interrupt`] (a
    /// write-1-to-clear on GPEDS). For the synchronous/asynchronous edge
    /// triggers that's enough; for a level trigger ([`Trigger::HighLevel`]/
    /// [`Trigger::LowLevel`]) GPEDS re-latches immediately while the pin
    /// stays at that level, so a level handler has to also remove the
    /// condition or mask the source to make progress.
    ///
    /// [`clear_interrupt`]: Self::clear_interrupt
    pub fn enable_interrupt(&self, trigger: Trigger) {
        set_trigger(&self.gpio, N, Some(trigger));
    }

    /// Disables all event detection on this pin — the inverse of
    /// [`enable_interrupt`](Self::enable_interrupt), clearing every
    /// edge/level detector (sync and async) for it. Does not touch the
    /// bank routing in the interrupt controller, which is shared with
    /// other pins in the same range.
    pub fn disable_interrupt(&self) {
        set_trigger(&self.gpio, N, None);
    }

    /// Acknowledges a latched event by clearing this pin's GPEDS bit
    /// (write-1-to-clear). Call this from the IRQ handler after servicing
    /// the pin, or the synchronous-edge/level line stays asserted and the
    /// handler re-fires immediately.
    pub fn clear_interrupt(&self) {
        clear_event(&self.gpio, N);
    }

    /// True if this pin currently has an event latched in GPEDS. In a
    /// handler this disambiguates which pin fired, since all pins in the
    /// same range share one interrupt-controller line (see
    /// `Lic::enable_gpio_irq`).
    pub fn is_interrupt_pending(&self) -> bool {
        event_detected(&self.gpio, N)
    }

    /// Blocks until the pin reads high, busy-polling GPLEV. Returns
    /// immediately if it is already high. This is a plain level poll, not
    /// the event-detect hardware — no interrupt setup, and a high pulse
    /// shorter than the loop's sampling can be missed. Use the interrupt
    /// API ([`enable_interrupt`](Self::enable_interrupt)) when a transient
    /// edge must not be lost.
    pub fn wait_for_high(&self) {
        while !read_level(&self.gpio, N) {
            core::hint::spin_loop();
        }
    }

    /// Blocks until the pin reads low, busy-polling GPLEV — the inverse of
    /// [`wait_for_high`](Self::wait_for_high).
    pub fn wait_for_low(&self) {
        while read_level(&self.gpio, N) {
            core::hint::spin_loop();
        }
    }
}

const FSEL_INPUT: u32 = 0b000;
const FSEL_OUTPUT: u32 = 0b001;

/// Read-modify-write of the 3-bit GPFSEL field for `pin`, across
/// whichever of GPFSEL0-5 actually holds it (10 pins per register).
/// Uses each register's raw `bits()` accessor rather than the named
/// per-pin fields `bcm2837-lpa` also exposes (e.g. `fsel4()`), since a
/// single generic implementation over any `pin: u8` can't name a
/// specific field at compile time the way the existing hand-written
/// examples do.
fn set_fsel(gpio: &GPIO, pin: u8, value: u32) {
    let shift = ((pin % 10) as u32) * 3;
    let mask = !(0b111u32 << shift);

    macro_rules! read_modify_write {
        ($reg:ident) => {{
            let current = gpio.$reg().read().bits();
            let new = (current & mask) | (value << shift);
            // `write_with_zero`, not `write`: these registers aren't
            // `Resettable` (no documented SVD reset value), but that
            // doesn't matter here since `.bits(new)` always supplies
            // the complete value anyway, not relying on any default.
            unsafe { gpio.$reg().write_with_zero(|w| w.bits(new)) };
        }};
    }

    match pin / 10 {
        0 => read_modify_write!(gpfsel0),
        1 => read_modify_write!(gpfsel1),
        2 => read_modify_write!(gpfsel2),
        3 => read_modify_write!(gpfsel3),
        4 => read_modify_write!(gpfsel4),
        _ => read_modify_write!(gpfsel5),
    }
}

/// Sets or clears `pin` via GPSET0/1 or GPCLR0/1 (write-1-to-set/clear;
/// selects the bank holding pins 0-31 vs 32-53).
fn set_level(gpio: &GPIO, pin: u8, high: bool) {
    let bit = 1u32 << (pin % 32);
    if pin < 32 {
        if high {
            unsafe { gpio.gpset0().write_with_zero(|w| w.bits(bit)) };
        } else {
            unsafe { gpio.gpclr0().write_with_zero(|w| w.bits(bit)) };
        }
    } else if high {
        unsafe { gpio.gpset1().write_with_zero(|w| w.bits(bit)) };
    } else {
        unsafe { gpio.gpclr1().write_with_zero(|w| w.bits(bit)) };
    }
}

/// Reads `pin`'s current level via GPLEV0/1 (selects the bank holding
/// pins 0-31 vs 32-53).
fn read_level(gpio: &GPIO, pin: u8) -> bool {
    let bit = 1u32 << (pin % 32);
    let bits = if pin < 32 {
        gpio.gplev0().read().bits()
    } else {
        gpio.gplev1().read().bits()
    };
    bits & bit != 0
}

/// Configures `pin`'s event detection to exactly `trigger`: sets the
/// detect-enable bits the trigger calls for and clears the other five,
/// so switching triggers (or `None` to disable) never leaves a stale
/// detector armed. Each of the six detect registers is banked like the
/// rest of the GPIO block (bank 0 = pins 0-31, bank 1 = 32-53), and each
/// write is a read-modify-write so other pins' enables in the same
/// register survive — the same `bits()`/`write_with_zero` idiom `set_fsel`
/// uses, and for the same reason (no SVD reset value, full value supplied).
fn set_trigger(gpio: &GPIO, pin: u8, trigger: Option<Trigger>) {
    let bit = 1u32 << (pin % 32);

    // Set or clear `bit` in whichever bank register (`$reg0` for pins
    // 0-31, `$reg1` for 32-53) holds this pin.
    macro_rules! set_bit_in {
        ($reg0:ident, $reg1:ident, $on:expr) => {{
            macro_rules! rmw {
                ($reg:ident) => {{
                    let current = gpio.$reg().read().bits();
                    let new = if $on { current | bit } else { current & !bit };
                    unsafe { gpio.$reg().write_with_zero(|w| w.bits(new)) };
                }};
            }
            if pin < 32 {
                rmw!($reg0)
            } else {
                rmw!($reg1)
            }
        }};
    }

    use Trigger::*;
    set_bit_in!(
        gpren0,
        gpren1,
        matches!(trigger, Some(RisingEdge | AnyEdge))
    );
    set_bit_in!(
        gpfen0,
        gpfen1,
        matches!(trigger, Some(FallingEdge | AnyEdge))
    );
    set_bit_in!(gphen0, gphen1, matches!(trigger, Some(HighLevel)));
    set_bit_in!(gplen0, gplen1, matches!(trigger, Some(LowLevel)));
    set_bit_in!(
        gparen0,
        gparen1,
        matches!(trigger, Some(AsyncRisingEdge | AsyncAnyEdge))
    );
    set_bit_in!(
        gpafen0,
        gpafen1,
        matches!(trigger, Some(AsyncFallingEdge | AsyncAnyEdge))
    );
}

/// Acknowledges `pin`'s latched event via GPEDS0/1. These are
/// write-1-to-clear, so writing just this pin's bit (and zeros
/// elsewhere) clears it without disturbing other pins' latched events —
/// no read-modify-write needed, unlike the enable registers above.
fn clear_event(gpio: &GPIO, pin: u8) {
    let bit = 1u32 << (pin % 32);
    if pin < 32 {
        unsafe { gpio.gpeds0().write_with_zero(|w| w.bits(bit)) };
    } else {
        unsafe { gpio.gpeds1().write_with_zero(|w| w.bits(bit)) };
    }
}

/// Reads `pin`'s latched event bit via GPEDS0/1 (selects the bank holding
/// pins 0-31 vs 32-53).
fn event_detected(gpio: &GPIO, pin: u8) -> bool {
    let bit = 1u32 << (pin % 32);
    let bits = if pin < 32 {
        gpio.gpeds0().read().bits()
    } else {
        gpio.gpeds1().read().bits()
    };
    bits & bit != 0
}

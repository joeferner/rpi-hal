use crate::pac::LIC;

/// Safe wrapper around the BCM2836/2837 legacy interrupt controller —
/// routes GPU-side IRQ sources (System Timer, UART0, etc.) to the ARM
/// core. Despite the PAC's name for it, this is the legacy
/// VideoCore-shared controller, not the separate per-core "local"
/// controller (mailboxes, ARM generic timer) that `bcm2837-lpa` doesn't
/// model at all.
///
/// Distinct from the CPU-level IRQ mask (see
/// [`crate::irq::enable_irq`]/[`crate::irq::disable_irq`]) — both need
/// to be open for an interrupt to fire.
pub struct Lic {
    lic: LIC,
}

impl Lic {
    /// Wraps an already-configured `LIC` peripheral. The controller
    /// itself needs no setup — routing is per-source, via the
    /// `enable_*`/`disable_*` methods below.
    pub fn new(lic: LIC) -> Self {
        Self { lic }
    }

    /// Routes System Timer Compare 1's IRQ to the ARM core. Compare 1
    /// (not 0 or 2) is used throughout this crate because GPU firmware
    /// reserves those two — see `Timer`'s module doc comment.
    pub fn enable_timer1_irq(&self) {
        unsafe {
            self.lic
                .enable_1()
                .write_with_zero(|w| w.timer_1().set_bit());
        }
    }

    /// Masks System Timer Compare 1's IRQ at the interrupt controller —
    /// the inverse of `enable_timer1_irq`.
    pub fn disable_timer1_irq(&self) {
        unsafe {
            self.lic
                .disable_1()
                .write_with_zero(|w| w.timer_1().clear_bit_by_one());
        }
    }

    /// True if Compare 1's IRQ is currently pending at the interrupt
    /// controller.
    pub fn is_timer1_pending(&self) -> bool {
        self.lic.pending_1().read().timer_1().bit_is_set()
    }

    /// Routes UART0's IRQ (shared by all four status conditions PL011
    /// can raise — see `Uart::enable_rx_irq`) to the ARM core.
    pub fn enable_uart_irq(&self) {
        unsafe {
            self.lic.enable_2().write_with_zero(|w| w.uart().set_bit());
        }
    }

    /// Masks UART0's IRQ at the interrupt controller — the inverse of
    /// `enable_uart_irq`.
    pub fn disable_uart_irq(&self) {
        unsafe {
            self.lic
                .disable_2()
                .write_with_zero(|w| w.uart().clear_bit_by_one());
        }
    }

    /// True if UART0's IRQ is currently pending at the interrupt
    /// controller.
    pub fn is_uart_pending(&self) -> bool {
        self.lic.pending_2().read().uart().bit_is_set()
    }

    /// Routes the I2C IRQ to the ARM core. Like the AUX line below, this
    /// one is shared: BSC0 and BSC1 raise the same interrupt, so a
    /// handler cannot tell from the controller which of them fired and
    /// has to read each one's `S` register — which is what
    /// `i2c::on_irq` does (behind the `async` feature, which is what
    /// arms those conditions in the first place).
    ///
    /// The interrupt conditions themselves (`C.INTD`/`INTT`/`INTR`) are
    /// opened by the async transfer that needs them, not here.
    pub fn enable_i2c_irq(&self) {
        unsafe {
            self.lic.enable_2().write_with_zero(|w| w.i2c().set_bit());
        }
    }

    /// Masks the I2C IRQ at the interrupt controller — the inverse of
    /// [`enable_i2c_irq`](Self::enable_i2c_irq), and it masks it for both
    /// BSC0 and BSC1, the two sharing one line.
    pub fn disable_i2c_irq(&self) {
        unsafe {
            self.lic
                .disable_2()
                .write_with_zero(|w| w.i2c().clear_bit_by_one());
        }
    }

    /// True if the I2C IRQ is currently pending at the interrupt
    /// controller — for either controller, the line being shared.
    pub fn is_i2c_pending(&self) -> bool {
        self.lic.pending_2().read().i2c().bit_is_set()
    }

    /// Routes the AUX IRQ to the ARM core. This one line is shared by all
    /// three AUX sub-peripherals — the mini UART (UART1), SPI1, and SPI2
    /// (the controller ORs their interrupts together), so a handler must
    /// read the AUX `IRQ` status register to tell which one fired. The
    /// mini UART's RX interrupt is enabled at the peripheral with
    /// [`MiniUart::enable_rx_irq`](crate::mini_uart::MiniUart::enable_rx_irq).
    pub fn enable_aux_irq(&self) {
        unsafe {
            self.lic.enable_1().write_with_zero(|w| w.aux().set_bit());
        }
    }

    /// Masks the AUX IRQ at the interrupt controller — the inverse of
    /// `enable_aux_irq`. Because the line is shared across all three AUX
    /// sub-peripherals, this masks every one of them.
    pub fn disable_aux_irq(&self) {
        unsafe {
            self.lic
                .disable_1()
                .write_with_zero(|w| w.aux().clear_bit_by_one());
        }
    }

    /// True if the AUX IRQ is currently pending at the interrupt
    /// controller. This only says *some* AUX sub-peripheral fired; read
    /// the AUX `IRQ` status register to find which.
    pub fn is_aux_pending(&self) -> bool {
        self.lic.pending_1().read().aux().bit_is_set()
    }

    /// Routes the USB controller's IRQ to the ARM core. Like the AUX
    /// line this is one interrupt covering everything the DWC2 core can
    /// report — channel halts, start-of-frame, root-port changes — so a
    /// handler dispatches on the core's own `GINTSTS`/`HAINT` rather
    /// than on which line fired;
    /// [`usb::dwc2::on_irq`](crate::usb::dwc2::on_irq) does exactly
    /// that.
    ///
    /// Which of those conditions actually reach this line is decided at
    /// the core by `GINTMSK`, which
    /// [`Dwc2Host::init`](crate::usb::dwc2::Dwc2Host::init) sets up. In
    /// particular start-of-frame is masked there and only unmasked
    /// while something is waiting on it — at high speed it fires every
    /// 125µs, and it is a level source, so an unserviced SOF is not a
    /// wasted interrupt but a hang.
    pub fn enable_usb_irq(&self) {
        unsafe {
            self.lic.enable_1().write_with_zero(|w| w.usb().set_bit());
        }
    }

    /// Masks the USB controller's IRQ at the interrupt controller — the
    /// inverse of `enable_usb_irq`.
    pub fn disable_usb_irq(&self) {
        unsafe {
            self.lic
                .disable_1()
                .write_with_zero(|w| w.usb().clear_bit_by_one());
        }
    }

    /// True if the USB controller's IRQ is currently pending at the
    /// interrupt controller. This only says the DWC2 core asserted its
    /// line; `GINTSTS`/`HAINT` say why.
    pub fn is_usb_pending(&self) -> bool {
        self.lic.pending_1().read().usb().bit_is_set()
    }

    /// Routes the EMMC (Arasan SD host controller) IRQ to the ARM core —
    /// the line [`sd::on_irq`](crate::sd::on_irq) services, and what an
    /// `async` SD transfer parks on instead of spinning on `INTERRUPT`.
    ///
    /// Which of the controller's conditions actually reach this line is
    /// decided at the peripheral by its `IRPT_EN` register, and that one
    /// is opened by the transfer that needs it rather than by
    /// [`Sd::init`](crate::sd::Sd::init) — so a program driving the card
    /// through the blocking API raises nothing here even with this line
    /// routed. The controller's other register, `IRPT_MASK`, gates only
    /// whether a condition becomes visible in `INTERRUPT` at all; `init`
    /// opens that one wide for the polling loop's benefit, which is why
    /// the two must not be confused.
    ///
    /// One line covers everything the controller can report, so a handler
    /// dispatches on `INTERRUPT` rather than on which line fired. It is
    /// also shared with the same controller's SDIO use
    /// ([`crate::sdio`], which drives this same hardware for the on-board
    /// WiFi chip): only one of the two can own the controller at a time,
    /// so the sharing costs nothing, but it is the reason `sd::on_irq`
    /// checks that an async transfer armed `IRPT_EN` before touching
    /// anything.
    pub fn enable_emmc_irq(&self) {
        unsafe {
            self.lic.enable_2().write_with_zero(|w| w.emmc().set_bit());
        }
    }

    /// Masks the EMMC IRQ at the interrupt controller — the inverse of
    /// [`enable_emmc_irq`](Self::enable_emmc_irq).
    pub fn disable_emmc_irq(&self) {
        unsafe {
            self.lic
                .disable_2()
                .write_with_zero(|w| w.emmc().clear_bit_by_one());
        }
    }

    /// True if the EMMC IRQ is currently pending at the interrupt
    /// controller. This only says the controller asserted its line; its
    /// `INTERRUPT` register says why.
    pub fn is_emmc_pending(&self) -> bool {
        self.lic.pending_2().read().emmc().bit_is_set()
    }

    /// Routes the GPIO bank IRQ that covers `pin` to the ARM core.
    ///
    /// The BCM2836/2837 splits GPIO event detection across three
    /// interrupt lines by pin range — 0-27, 28-45, 46-53 — so this maps
    /// the pin to the right one. Those ranges are the hardware's, matching
    /// Linux's `pinctrl-bcm2835` bank split; note they are deliberately
    /// *not* the 0-31/32-53 boundary the GPEDS registers use, which is
    /// different. (A fourth line ORs all GPIO events together, but the
    /// per-range lines are used instead so unrelated pins can be masked
    /// independently.)
    ///
    /// All pins in the same range share one line: enabling it for one pin
    /// enables it for every pin in that range, so a handler must read
    /// GPEDS — [`Pin::is_interrupt_pending`](crate::gpio::Pin::is_interrupt_pending)
    /// — to tell which pin actually fired.
    pub fn enable_gpio_irq(&self, pin: u8) {
        unsafe {
            match gpio_line(pin) {
                0 => self
                    .lic
                    .enable_2()
                    .write_with_zero(|w| w.gpio_0().set_bit()),
                1 => self
                    .lic
                    .enable_2()
                    .write_with_zero(|w| w.gpio_1().set_bit()),
                _ => self
                    .lic
                    .enable_2()
                    .write_with_zero(|w| w.gpio_2().set_bit()),
            }
        }
    }

    /// Masks the GPIO bank IRQ covering `pin` — the inverse of
    /// `enable_gpio_irq`. Because the line is shared across the whole pin
    /// range, this also masks every other pin in that range.
    pub fn disable_gpio_irq(&self, pin: u8) {
        unsafe {
            match gpio_line(pin) {
                0 => self
                    .lic
                    .disable_2()
                    .write_with_zero(|w| w.gpio_0().clear_bit_by_one()),
                1 => self
                    .lic
                    .disable_2()
                    .write_with_zero(|w| w.gpio_1().clear_bit_by_one()),
                _ => self
                    .lic
                    .disable_2()
                    .write_with_zero(|w| w.gpio_2().clear_bit_by_one()),
            }
        }
    }

    /// True if the GPIO bank IRQ covering `pin` is currently pending. This
    /// only says *some* pin in the range fired; read GPEDS via
    /// [`Pin::is_interrupt_pending`](crate::gpio::Pin::is_interrupt_pending)
    /// to find which.
    pub fn is_gpio_pending(&self, pin: u8) -> bool {
        match gpio_line(pin) {
            0 => self.lic.pending_2().read().gpio_0().bit_is_set(),
            1 => self.lic.pending_2().read().gpio_1().bit_is_set(),
            _ => self.lic.pending_2().read().gpio_2().bit_is_set(),
        }
    }
}

/// Maps a GPIO pin to which of the three GPIO interrupt lines carries its
/// events: 0 for pins 0-27, 1 for 28-45, 2 for 46-53. See
/// [`Lic::enable_gpio_irq`] for why these ranges (not the GPEDS register
/// banks) are the right split.
const fn gpio_line(pin: u8) -> u8 {
    if pin <= 27 {
        0
    } else if pin <= 45 {
        1
    } else {
        2
    }
}

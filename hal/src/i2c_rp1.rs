//! Blocking driver for the Pi 5's header I2C buses, on the RP1 south
//! bridge.
//!
//! The other chips' `i2c` module drives the BCM2835 BSC controller. The
//! Pi 5's header pins route to RP1's I2C controllers instead, a Synopsys
//! DesignWare I2C (`snps,designware-i2c` in `rp1.dtsi`) with nothing in
//! common with the BSC but the bus it drives. This module keeps the same
//! shape so code written against the header bus ports across:
//! [`I2c<BSC1>`] is I2C1 on GPIO2 (SDA1)/GPIO3 (SCL1), [`I2c<BSC0>`] is
//! I2C0 on GPIO0/1 (`ID_SD`/`ID_SC`, the HAT ID EEPROM bus), the same
//! [`Error`] variants come back, and `embedded_hal::i2c::I2c` is
//! implemented on both. The `BSC0`/`BSC1` names are the earlier chips';
//! see [`pac::BSC1`](crate::pac::BSC1).
//!
//! Register layout and programming sequence from Linux's
//! `i2c-designware-core.h`, `-common.c` and `-master.c`.
//!
//! # Clock
//!
//! The controller counts SCL's high and low phases in cycles of RP1's
//! `clk_sys`, which `clk-rp1.c` caps at 200MHz. Nothing pins what the
//! firmware set it to, so [`I2c::init_with_frequency`] computes the counts
//! against that 200MHz ceiling: on a slower `clk_sys` the bus runs slower
//! than asked, never faster, which is the safe direction for a part with a
//! maximum bus rate.
//!
//! # Transactions
//!
//! Unlike the BSC, this controller does real repeated starts: every
//! operation of an `embedded_hal::i2c::I2c::transaction` after the first
//! begins with a repeated START rather than a STOP and a fresh START, and
//! only the last one ends with a STOP. A `write_read` against a device
//! that resets its register pointer on STOP therefore works here.
//!
//! Every transfer is bounded against the System Timer, as on the other
//! chips: a device that stops driving the bus mid-transfer becomes an
//! [`Error::Timeout`] instead of a hang.

use crate::gpio::Pull;
use crate::mailbox::Mailbox;
use crate::pac::{BSC0, BSC1, GPIO};
use crate::timer::Timer;
use embedded_hal::i2c::Operation;

/// Fixed allowance for START, the address byte and the controller's own
/// setup, on top of [`TIMEOUT_PER_BYTE_US`].
const TIMEOUT_BASE_US: u64 = 5_000;

/// Allowance per byte of the transfer: ~90us at 100kHz, so more than
/// tenfold margin, and still enough for a slave that stretches the clock.
const TIMEOUT_PER_BYTE_US: u64 = 1_000;

/// How long disabling the controller, or aborting a transfer, may take.
/// Either completes within a few bus clock periods.
const CONTROL_TIMEOUT_US: u64 = 10_000;

/// The `clk_sys` rate SCL is timed against: the most `clk-rp1.c` lets it
/// run at (see this module's doc comment).
const IC_CLK_HZ: u64 = 200_000_000;

/// Fastest SCL this driver programs: fast mode. Fast mode plus needs a
/// different pad configuration, and nothing here has been tried there.
const MAX_FREQUENCY_HZ: u32 = 400_000;

// Register offsets (`DW_IC_*` in `i2c-designware-core.h`).
const IC_CON: usize = 0x00;
const IC_TAR: usize = 0x04;
const IC_DATA_CMD: usize = 0x10;
const IC_SS_SCL_HCNT: usize = 0x14;
const IC_SS_SCL_LCNT: usize = 0x18;
const IC_FS_SCL_HCNT: usize = 0x1c;
const IC_FS_SCL_LCNT: usize = 0x20;
const IC_INTR_MASK: usize = 0x30;
const IC_RAW_INTR_STAT: usize = 0x34;
const IC_CLR_INTR: usize = 0x40;
const IC_CLR_TX_ABRT: usize = 0x54;
const IC_CLR_STOP_DET: usize = 0x60;
const IC_ENABLE: usize = 0x6c;
const IC_STATUS: usize = 0x70;
const IC_TXFLR: usize = 0x74;
const IC_TX_ABRT_SOURCE: usize = 0x80;
const IC_ENABLE_STATUS: usize = 0x9c;
const IC_COMP_PARAM_1: usize = 0xf4;

// `IC_CON` bits.
const CON_MASTER: u32 = 1 << 0;
const CON_SPEED_STD: u32 = 1 << 1;
const CON_SPEED_FAST: u32 = 2 << 1;
const CON_RESTART_EN: u32 = 1 << 5;
const CON_SLAVE_DISABLE: u32 = 1 << 6;

// `IC_DATA_CMD` bits above the data byte.
const CMD_READ: u32 = 1 << 8;
const CMD_STOP: u32 = 1 << 9;
const CMD_RESTART: u32 = 1 << 10;

// `IC_ENABLE` bits.
const ENABLE: u32 = 1 << 0;
const ENABLE_ABORT: u32 = 1 << 1;

// `IC_STATUS` bits.
const STATUS_TFE: u32 = 1 << 2;
const STATUS_RFNE: u32 = 1 << 3;
const STATUS_MASTER_ACTIVITY: u32 = 1 << 5;

// `IC_RAW_INTR_STAT` bits.
const INTR_TX_ABRT: u32 = 1 << 6;
const INTR_STOP_DET: u32 = 1 << 9;

/// `IC_TX_ABRT_SOURCE` bits meaning a byte was not acknowledged: the
/// 7-bit address, either half of a 10-bit one, a data byte, or a general
/// call (`DW_IC_TX_ABRT_NOACK`).
const ABRT_NOACK: u32 = 0b1_1111;

/// GPIO function (`CTRL.FUNCSEL`) routing GPIO0-3 to RP1's I2C0 (0/1) and
/// I2C1 (2/3): column 3 of RP1's pin function table in `pinctrl-rp1.c`.
const FUNCSEL_I2C: u32 = 3;

/// Errors surfaced by [`I2c`]'s `embedded_hal::i2c::I2c` methods. The same
/// variants as the other chips' `i2c::Error`, so code matching on them
/// ports across.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The controller aborted the transfer because a byte went
    /// unacknowledged: the address (nothing listening there) or a data
    /// byte.
    NoAcknowledge,
    /// An operation in the transaction had zero length. The controller
    /// can only send an address as part of a byte, so there is no way to
    /// issue an address-only probe through this API; a 1-byte read is the
    /// usual substitute.
    ZeroLengthUnsupported,
    /// The transfer didn't complete within its deadline: the bus is being
    /// held, or a slave stopped driving mid-transfer. The transfer has
    /// been aborted and the controller disabled, so the next one starts
    /// from a clean state; a bus a slave is still holding will time out
    /// again.
    ///
    /// Also reported when the controller aborted for a reason other than
    /// a missing acknowledge (lost arbitration to another master, chiefly):
    /// the same meaning, someone else had the bus.
    Timeout,
    /// The transfer ended with fewer bytes read than were asked for.
    /// Carries both counts; `buffer[received..]` is untouched.
    ///
    /// The master clocks every byte of a read itself, so a slave can't cut
    /// one short on this controller the way it can on the BSC; this is
    /// reported only if the read ran out of time after the bus went idle.
    Incomplete {
        /// Bytes actually read into the buffer.
        received: usize,
        /// Bytes the transfer asked for.
        requested: usize,
    },
}

impl embedded_hal::i2c::Error for Error {
    /// [`Error::NoAcknowledge`] maps to `NoAcknowledge` with an unknown
    /// source, everything else to `Other`.
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Self::NoAcknowledge => embedded_hal::i2c::ErrorKind::NoAcknowledge(
                embedded_hal::i2c::NoAcknowledgeSource::Unknown,
            ),
            Self::ZeroLengthUnsupported | Self::Timeout | Self::Incomplete { .. } => {
                embedded_hal::i2c::ErrorKind::Other
            }
        }
    }
}

/// Blocking driver for one of RP1's I2C controllers, generic over the
/// instance `I` (defaulting to [`BSC1`] so `I2c` alone means the header
/// bus). Construct with [`I2c::<BSC1>::init_with_frequency`] for GPIO2/3
/// or [`I2c::<BSC0>::init_id_with_frequency`] for GPIO0/1.
pub struct I2c<'a, I = BSC1> {
    _bsc: I,
    /// The controller's register block.
    base: usize,
    timer: &'a Timer,
    /// TX FIFO depth, which bounds commands queued ahead of the bus.
    tx_depth: u32,
    /// RX FIFO depth, which bounds reads queued ahead of being drained.
    rx_depth: u32,
}

impl<'a> I2c<'a, BSC1> {
    /// Routes GPIO2/3 to I2C1 and sets SCL to `frequency_hz` or the
    /// nearest rate below it this controller can produce, at most 400kHz
    /// (fast mode).
    ///
    /// The same signature as on the other chips, where `mailbox` supplies
    /// the core clock the BSC divides down. Here it goes unused: SCL is
    /// timed against RP1's `clk_sys` (see this module's doc comment), so
    /// this never fails, and returns `Result` only to match.
    ///
    /// `timer` bounds every transfer this driver performs.
    pub fn init_with_frequency(
        gpio: &GPIO,
        bsc1: BSC1,
        frequency_hz: u32,
        mailbox: &mut Mailbox,
        timer: &'a Timer,
    ) -> Result<Self, crate::mailbox::Error> {
        let _ = (gpio, mailbox);
        route_pins([2, 3]);
        Ok(Self::configure(bsc1, BSC1::ADDR, frequency_hz, timer))
    }
}

impl<'a> I2c<'a, BSC0> {
    /// Routes GPIO0/1 (`ID_SD`/`ID_SC`, the HAT ID EEPROM bus on header
    /// pins 27/28) to I2C0 and sets SCL as
    /// [`I2c::<BSC1>::init_with_frequency`] does.
    ///
    /// The firmware reads a HAT's ID EEPROM during boot, before this image
    /// runs, and leaves the pins alone afterwards, so a bare-metal program
    /// is free to use them.
    pub fn init_id_with_frequency(
        gpio: &GPIO,
        bsc0: BSC0,
        frequency_hz: u32,
        mailbox: &mut Mailbox,
        timer: &'a Timer,
    ) -> Result<Self, crate::mailbox::Error> {
        let _ = (gpio, mailbox);
        route_pins([0, 1]);
        Ok(Self::configure(bsc0, BSC0::ADDR, frequency_hz, timer))
    }
}

/// Hands `pins` to the I2C function the way `rp1.dtsi` configures them:
/// pulled up, at 12mA drive. The board's own pull-ups on GPIO0-3 are what
/// actually hold the bus up; the pad's are a backstop.
fn route_pins(pins: [u8; 2]) {
    for pin in pins {
        crate::gpio::set_pull(pin, Pull::Up);
        crate::gpio::set_drive_12ma(pin);
        crate::gpio::select_function(pin, FUNCSEL_I2C);
    }
}

/// `IC_CON` speed bits and the SCL high/low counts for `frequency_hz`.
///
/// The period is `IC_CLK_HZ / frequency_hz` cycles, split between high and
/// low in the ratio of the mode's minimum phase times (4.0us/4.7us for
/// standard mode, 0.6us/1.3us for fast mode), and never below those
/// minimums. The controller adds about 8 cycles to the programmed high
/// count and 1 to the low (`i2c_dw_scl_hcnt`/`_lcnt` in Linux), so those
/// come off. The bus's rise time adds on top, so the real rate lands a
/// little below the one asked for.
fn scl_counts(frequency_hz: u32) -> (u32, u32, u32) {
    let frequency_hz = u64::from(frequency_hz.clamp(1, MAX_FREQUENCY_HZ));
    let (speed, high_ns, low_ns) = if frequency_hz > 100_000 {
        (CON_SPEED_FAST, 600, 1300)
    } else {
        (CON_SPEED_STD, 4000, 4700)
    };

    let period = IC_CLK_HZ.div_ceil(frequency_hz);
    let min_high = (IC_CLK_HZ * high_ns).div_ceil(1_000_000_000);
    let min_low = (IC_CLK_HZ * low_ns).div_ceil(1_000_000_000);
    let high = (period * high_ns / (high_ns + low_ns)).max(min_high);
    let low = period.saturating_sub(high).max(min_low);

    let hcnt = (high.saturating_sub(8)).clamp(6, 0xffff) as u32;
    let lcnt = (low.saturating_sub(1)).clamp(8, 0xffff) as u32;
    (speed, hcnt, lcnt)
}

impl<'a, I> I2c<'a, I> {
    /// Shared tail of the constructors: program the bus speed with the
    /// controller disabled, and read its FIFO depths.
    fn configure(bsc: I, base: usize, frequency_hz: u32, timer: &'a Timer) -> Self {
        let mut i2c = Self {
            _bsc: bsc,
            base,
            timer,
            tx_depth: 1,
            rx_depth: 1,
        };
        i2c.disable();

        let (speed, hcnt, lcnt) = scl_counts(frequency_hz);
        let (hcnt_reg, lcnt_reg) = if speed == CON_SPEED_FAST {
            (IC_FS_SCL_HCNT, IC_FS_SCL_LCNT)
        } else {
            (IC_SS_SCL_HCNT, IC_SS_SCL_LCNT)
        };
        i2c.write(
            IC_CON,
            CON_MASTER | speed | CON_RESTART_EN | CON_SLAVE_DISABLE,
        );
        i2c.write(hcnt_reg, hcnt);
        i2c.write(lcnt_reg, lcnt);
        // Polled: no interrupt reaches anything on this chip yet.
        i2c.write(IC_INTR_MASK, 0);

        // `TX_BUFFER_DEPTH - 1` in bits 23:16, `RX_BUFFER_DEPTH - 1` in
        // 15:8. A core built without the register reads zero, which gives
        // depths of 1: correct, only slower.
        let params = i2c.read(IC_COMP_PARAM_1);
        i2c.tx_depth = ((params >> 16) & 0xff) + 1;
        i2c.rx_depth = ((params >> 8) & 0xff) + 1;
        i2c
    }

    /// Reads the register at `offset`.
    fn read(&self, offset: usize) -> u32 {
        unsafe { ((self.base + offset) as *const u32).read_volatile() }
    }

    /// Writes `value` to the register at `offset`.
    fn write(&self, offset: usize, value: u32) {
        unsafe { ((self.base + offset) as *mut u32).write_volatile(value) };
    }

    /// Disables the controller and waits until it reports itself disabled,
    /// which `IC_TAR` and the timing registers require before they can be
    /// written. Bounded: returns `false` if it never settles.
    fn disable(&mut self) -> bool {
        self.write(IC_ENABLE, 0);
        let deadline = self.timer.now_micros() + CONTROL_TIMEOUT_US;
        while self.read(IC_ENABLE_STATUS) & 1 != 0 {
            if self.timer.now_micros() > deadline {
                return false;
            }
        }
        true
    }

    /// Recovers from a transfer that ran out of time: asks the controller
    /// to abort it (`IC_ENABLE.ABORT`, which flushes the FIFOs and puts a
    /// STOP on the bus), waits for that to take, then clears the latched
    /// status and disables the controller.
    ///
    /// Best-effort, as on the other chips: nothing here can make a slave
    /// that is holding SDA let go, and a transfer against such a bus will
    /// time out again.
    fn abandon(&mut self) {
        self.write(IC_ENABLE, ENABLE | ENABLE_ABORT);
        let deadline = self.timer.now_micros() + CONTROL_TIMEOUT_US;
        while self.read(IC_ENABLE) & ENABLE_ABORT != 0 && self.timer.now_micros() <= deadline {}
        let _ = self.read(IC_CLR_INTR);
        self.disable();
    }

    /// Reports and clears an abort the controller latched, if any:
    /// [`Error::NoAcknowledge`] for a byte nobody acknowledged,
    /// [`Error::Timeout`] for anything else. The source register has to be
    /// read before the clear, which resets it.
    fn take_abort(&mut self) -> Option<Error> {
        if self.read(IC_RAW_INTR_STAT) & INTR_TX_ABRT == 0 {
            return None;
        }
        let source = self.read(IC_TX_ABRT_SOURCE);
        let _ = self.read(IC_CLR_TX_ABRT);
        self.disable();
        Some(if source & ABRT_NOACK != 0 {
            Error::NoAcknowledge
        } else {
            Error::Timeout
        })
    }

    /// Runs `operations` against `address` as one transaction: START, the
    /// operations separated by repeated STARTs, then STOP.
    ///
    /// Commands go into the TX FIFO as fast as it has room, reads limited
    /// to what the RX FIFO can hold until drained, while a separate cursor
    /// drains received bytes into the read buffers in order. Done is the
    /// STOP having gone out with the controller idle and nothing left to
    /// send or receive.
    fn run(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        let mut total = 0;
        for operation in operations.iter() {
            let len = match operation {
                Operation::Read(buffer) => buffer.len(),
                Operation::Write(bytes) => bytes.len(),
            };
            if len == 0 {
                return Err(Error::ZeroLengthUnsupported);
            }
            total += len;
        }
        let count = operations.len();
        if count == 0 {
            return Ok(());
        }

        if !self.disable() {
            return Err(Error::Timeout);
        }
        self.write(IC_TAR, u32::from(address & 0x7f));
        let _ = self.read(IC_CLR_INTR);
        self.write(IC_ENABLE, ENABLE);

        let deadline =
            self.timer.now_micros() + TIMEOUT_BASE_US + TIMEOUT_PER_BYTE_US * total as u64;
        // Push cursor: the next command to queue.
        let (mut push_op, mut push_byte) = (0, 0);
        // Drain cursor: where the next received byte goes.
        let (mut drain_op, mut drain_byte) = (0, 0);
        let mut outstanding = 0;

        loop {
            if let Some(error) = self.take_abort() {
                return Err(error);
            }

            // Drain whatever has arrived, moving the cursor past writes and
            // finished reads as it goes.
            loop {
                while drain_op < count {
                    match &operations[drain_op] {
                        Operation::Read(buffer) if drain_byte < buffer.len() => break,
                        _ => {
                            drain_op += 1;
                            drain_byte = 0;
                        }
                    }
                }
                if drain_op == count || self.read(IC_STATUS) & STATUS_RFNE == 0 {
                    break;
                }
                let byte = self.read(IC_DATA_CMD) as u8;
                if let Operation::Read(buffer) = &mut operations[drain_op] {
                    buffer[drain_byte] = byte;
                }
                drain_byte += 1;
                outstanding -= 1;
            }

            // Queue the next command, if there is one and room for it.
            if push_op < count && self.read(IC_TXFLR) < self.tx_depth {
                let (len, is_read) = match &operations[push_op] {
                    Operation::Read(buffer) => (buffer.len(), true),
                    Operation::Write(bytes) => (bytes.len(), false),
                };
                if !is_read || outstanding < self.rx_depth {
                    let mut command = 0;
                    if push_op > 0 && push_byte == 0 {
                        command |= CMD_RESTART;
                    }
                    if push_op == count - 1 && push_byte == len - 1 {
                        command |= CMD_STOP;
                    }
                    match &operations[push_op] {
                        Operation::Read(_) => {
                            self.write(IC_DATA_CMD, command | CMD_READ);
                            outstanding += 1;
                        }
                        Operation::Write(bytes) => {
                            self.write(IC_DATA_CMD, command | u32::from(bytes[push_byte]));
                        }
                    }
                    push_byte += 1;
                    if push_byte == len {
                        push_op += 1;
                        push_byte = 0;
                    }
                }
            }

            if push_op == count && drain_op == count {
                let status = self.read(IC_STATUS);
                let idle = status & STATUS_TFE != 0 && status & STATUS_MASTER_ACTIVITY == 0;
                if idle && self.read(IC_RAW_INTR_STAT) & INTR_STOP_DET != 0 {
                    let _ = self.read(IC_CLR_STOP_DET);
                    // A NACK on the final byte can land with the STOP.
                    if let Some(error) = self.take_abort() {
                        return Err(error);
                    }
                    return Ok(());
                }
            }

            if self.timer.now_micros() > deadline {
                let idle = self.read(IC_STATUS) & STATUS_MASTER_ACTIVITY == 0;
                self.abandon();
                return Err(match &operations.get(drain_op) {
                    Some(Operation::Read(buffer)) if idle => Error::Incomplete {
                        received: drain_byte,
                        requested: buffer.len(),
                    },
                    _ => Error::Timeout,
                });
            }
        }
    }
}

impl<I> embedded_hal::i2c::ErrorType for I2c<'_, I> {
    /// See [`Error`].
    type Error = Error;
}

impl<I> embedded_hal::i2c::I2c for I2c<'_, I> {
    /// `read`/`write`/`write_read` all forward here. The operations run as
    /// one transaction with repeated STARTs between them — see this
    /// module's doc comment.
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.run(address, operations)
    }
}

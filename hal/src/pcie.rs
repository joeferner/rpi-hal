//! Root complex bring-up for the Pi 5's external PCIe connector: the
//! BCM2712's `pcie1`, a one-lane Broadcom set-top-box controller.
//!
//! [`Pcie::init`] resets the controller, programs its address windows,
//! trains the link and sets the root port up as the bridge to bus 1, where
//! whatever is plugged in appears. [`Pcie::functions`] lists what answered
//! there, and [`Pcie::enable`] gives one function its memory BARs and turns
//! on its memory decoding and bus mastering, which is everything a driver
//! for it (e.g. [`crate::nvme`]) needs from the bus.
//!
//! The sequence is Linux's `pcie-brcmstb.c` for the `brcm,bcm2712-pcie`
//! compatible, with the addresses and windows from `bcm2712.dtsi`'s `pcie1`
//! node, plus the pieces of the Linux PCI core's enumeration a single
//! device behind a single root port needs: bus numbers, the bridge's
//! memory window, BAR sizing and assignment.
//!
//! ## The firmware leaves it in reset
//!
//! Unlike `pcie2` (RP1, which `pciex4_reset=0` keeps alive into the
//! kernel), the Pi 5's firmware does not hand `pcie1` over trained, and
//! `dtparam=pciex1` only edits the device tree for Linux, which a bare-metal
//! kernel never reads. So this does the whole bring-up itself, and needs no
//! `config.txt` line.
//!
//! ## Address windows
//!
//! - **Outbound** (CPU → device): CPU [`WINDOW_BASE`] is PCIe address 0,
//!   the start of `pcie1`'s 32-bit non-prefetchable window. Only its first
//!   [`WINDOW_LEN`] bytes are mapped by [`crate::mmu`], which is where
//!   [`Pcie::enable`] places BARs; that is more than any one SSD asks for.
//! - **Inbound** (device → RAM): PCIe address `0x10_0000_0000` is CPU
//!   physical 0, for 64 GiB, so a device reaches all of RAM at its physical
//!   address plus [`DMA_OFFSET`] ([`dma_address`]).
//!
//! The controller does not snoop this core's caches (`pcie1` has no
//! `dma-coherent`), so a driver cleans what a device will read and
//! invalidates what it wrote, as [`crate::nvme`] does.
//!
//! ## No interrupts
//!
//! Neither MSI nor INTx is set up: the Pi 5 has no interrupt controller
//! driver here yet, and [`Pcie::enable`] sets the function's
//! *Interrupt Disable* bit. A driver polls its device.

use crate::pac::{PCIE1, SOC_BASE};
use crate::timer::Timer;

/// CPU address of PCIe address 0: the start of `pcie1`'s outbound 32-bit
/// memory window (`ranges`: `0x02000000 0x00 0x00000000 0x1b 0x00000000`).
pub const WINDOW_BASE: usize = 0x1b_0000_0000;

/// Bytes of the outbound window [`crate::mmu`] maps, from [`WINDOW_BASE`],
/// and so the space [`Pcie::enable`] assigns BARs from.
pub const WINDOW_LEN: usize = 1 << 30;

/// What a device adds to a CPU physical address to reach it over the
/// inbound window (`dma-ranges`: PCIe `0x10_0000_0000` is CPU 0, 64 GiB).
pub const DMA_OFFSET: u64 = 0x10_0000_0000;

/// Bytes of RAM the inbound window spans from CPU address 0.
const DMA_LEN: u64 = 64 << 30;

/// The address a device on this bus uses to read or write the RAM at
/// physical (and, under this crate's identity map, virtual) address
/// `address`.
pub fn dma_address(address: usize) -> u64 {
    address as u64 + DMA_OFFSET
}

// Shared resets, outside the controller's own register file. Both from
// `bcm2712.dtsi`: `pcie_rescal` and `bcm_reset`, on the SoC bus.

/// The PCIe/SATA PHY resistor calibration block (`pcie_rescal`), shared by
/// all three controllers.
const RESCAL: usize = SOC_BASE + 0x0011_9500;
/// `RESCAL_START`: bit 0 starts a calibration.
const RESCAL_START: usize = 0x0;
/// `RESCAL_STATUS`: bit 0 is set once the calibration finished.
const RESCAL_STATUS: usize = 0x8;

/// The SoC's software reset controller (`bcm_reset`, `brcm,brcmstb-reset`):
/// banks of `SET`/`CLEAR`/`STATUS` words, 0x18 bytes apart, one bit per
/// line.
const SW_INIT: usize = SOC_BASE + 0x0150_4318;
/// `pcie1`'s "bridge" reset line on [`SW_INIT`]: line 43, bank 1 bit 11.
const BRIDGE_RESET_LINE: usize = 43;

// Controller registers, as offsets into `PCIE1::ADDR`. Names and values are
// Linux's `pcie-brcmstb.c` (its BCM7712 register set, which the BCM2712
// shares).

/// Where the root port's PCI Express capability starts in its own
/// configuration space.
const CAP_REGS: usize = 0x00ac;
/// `Link Status` (upper half of the dword at `CAP_REGS + 0x10`): current
/// speed in bits 19:16, width in bits 25:20 of that dword.
const LINK_CONTROL_STATUS: usize = CAP_REGS + 0x10;
/// `Link Control 2` (lower half of the dword at `CAP_REGS + 0x30`):
/// target link speed in bits 3:0.
const LINK_CONTROL_2: usize = CAP_REGS + 0x30;

/// Endian mode of the inbound window: bits 3:2, 0 = little-endian.
const VENDOR_SPECIFIC_REG1: usize = 0x0188;
/// Class code the root port reports (bits 23:0).
const PRIV1_ID_VAL3: usize = 0x043c;
/// Advertised link capabilities: max speed (bits 3:0), ASPM (bits 11:10).
const PRIV1_LINK_CAPABILITY: usize = 0x04dc;
/// Root capabilities: the L1 substates mode in bits 7:3.
const PRIV1_ROOT_CAP: usize = 0x04f8;

/// The PHY's MDIO bus: command, write data and read data.
const MDIO_ADDR: usize = 0x1100;
const MDIO_WR_DATA: usize = 0x1104;
/// `MDIO_WR_DATA` bit 31: set to start a write, cleared when done.
const MDIO_DATA_DONE: u32 = 1 << 31;

/// PHY control 15: the PM clock period in bits 7:0.
const PL_PHY_CTL_15: usize = 0x184c;

const MISC_CTRL: usize = 0x4008;
const MISC_CTRL_RCB_64B_MODE: u32 = 0x80;
const MISC_CTRL_RCB_MPS_MODE: u32 = 0x400;
const MISC_CTRL_SCB_ACCESS_EN: u32 = 0x1000;
const MISC_CTRL_CFG_READ_UR_MODE: u32 = 0x2000;
/// `SCB_MAX_BURST_SIZE`, bits 21:20: `2` is 512 bytes.
const MISC_CTRL_MAX_BURST_MASK: u32 = 0x30_0000;
const MISC_CTRL_MAX_BURST_512: u32 = 0x20_0000;

/// Outbound window 0: the PCIe address its base maps to (low, high).
const MEM_WIN0_LO: usize = 0x400c;
const MEM_WIN0_HI: usize = 0x4010;
/// Outbound window 0's CPU base and limit, in MiB: low 12 bits of each
/// here (base in bits 15:4, limit in bits 31:20), the rest in the two
/// registers below (bits 7:0).
const MEM_WIN0_BASE_LIMIT: usize = 0x4070;
const MEM_WIN0_BASE_HI: usize = 0x4080;
const MEM_WIN0_LIMIT_HI: usize = 0x4084;

/// Inbound window 1 ("BAR1"): the PCIe address it answers (low word, with
/// the encoded size in bits 4:0, then the high word).
const RC_BAR1_CONFIG_LO: usize = 0x402c;
const RC_BAR1_CONFIG_HI: usize = 0x4030;
/// Inbound window 1's CPU address (low word with `ACCESS_EN` in bit 0,
/// then the high word).
const UBUS_BAR1_CONFIG_REMAP: usize = 0x40ac;
const UBUS_BAR1_CONFIG_REMAP_HI: usize = 0x40b0;
const UBUS_REMAP_ACCESS_EN: u32 = 1;

const RC_CONFIG_RETRY_TIMEOUT: usize = 0x405c;

/// `PCIE_CTRL.PERSTB` (bit 2): 0 holds the device in fundamental reset
/// (`PERST#` low).
const PCIE_CTRL: usize = 0x4064;
const PCIE_CTRL_PERSTB: u32 = 0x4;

const PCIE_STATUS: usize = 0x4068;
const STATUS_PHYLINKUP: u32 = 0x10;
const STATUS_DL_ACTIVE: u32 = 0x20;
/// Set when the controller is strapped as a root complex, not an endpoint.
const STATUS_PORT: u32 = 0x80;

const MISC_CTRL_1: usize = 0x40a0;
const MISC_CTRL_1_EN_VDM_QOS_CONTROL: u32 = 1 << 5;

const UBUS_CTRL: usize = 0x40a4;
const UBUS_CTRL_REPLY_ERR_DIS: u32 = 1 << 13;
const UBUS_CTRL_REPLY_DECERR_DIS: u32 = 1 << 19;
const UBUS_TIMEOUT: usize = 0x40a8;

const AXI_INTF_CTRL: usize = 0x416c;
const AXI_EN_RCLK_QOS_ARRAY_FIX: u32 = 1 << 13;
const AXI_EN_QOS_UPDATE_TIMING_FIX: u32 = 1 << 12;
const AXI_DIS_QOS_GATING_IN_MASTER: u32 = 1 << 11;
const AXI_REQFIFO_EN_QOS_PROPAGATION: u32 = 1 << 7;
const AXI_MASTER_MAX_OUTSTANDING_MASK: u32 = 0x3f;
const AXI_READ_ERROR_DATA: usize = 0x4170;

const HARD_DEBUG: usize = 0x4304;
const HARD_DEBUG_CLKREQ_DEBUG_ENABLE: u32 = 0x2;
const HARD_DEBUG_REFCLK_OVRD_ENABLE: u32 = 0x1_0000;
const HARD_DEBUG_REFCLK_OVRD_OUT: u32 = 0x10_0000;
const HARD_DEBUG_L1SS_ENABLE: u32 = 0x20_0000;
const HARD_DEBUG_SERDES_IDDQ: u32 = 0x0800_0000;
const HARD_DEBUG_CLKREQ_MASK: u32 = HARD_DEBUG_CLKREQ_DEBUG_ENABLE
    | HARD_DEBUG_REFCLK_OVRD_ENABLE
    | HARD_DEBUG_REFCLK_OVRD_OUT
    | HARD_DEBUG_L1SS_ENABLE;

/// Configuration space of everything below the root port goes through a
/// window: write the target's bus/device/function to `EXT_CFG_INDEX`, then
/// its 4 KiB of configuration space appears at `EXT_CFG_DATA`.
const EXT_CFG_DATA: usize = 0x8000;
const EXT_CFG_INDEX: usize = 0x9000;

// Standard configuration space offsets.
const COMMAND: u16 = 0x04;
const COMMAND_MEMORY: u32 = 1 << 1;
const COMMAND_BUS_MASTER: u32 = 1 << 2;
const COMMAND_INTX_DISABLE: u32 = 1 << 10;
const CLASS_REVISION: u16 = 0x08;
const HEADER_TYPE: u16 = 0x0c;
const BAR0: u16 = 0x10;
/// Type 1 (bridge) header: primary, secondary and subordinate bus numbers.
const BRIDGE_BUSES: u16 = 0x18;
const BRIDGE_IO_BASE_LIMIT: u16 = 0x1c;
const BRIDGE_MEMORY_BASE_LIMIT: u16 = 0x20;
const BRIDGE_PREFETCH_BASE_LIMIT: u16 = 0x24;
const BRIDGE_PREFETCH_BASE_HI: u16 = 0x28;
const BRIDGE_PREFETCH_LIMIT_HI: u16 = 0x2c;

/// The bus the root port's secondary side is, and so where the plugged-in
/// device answers.
const DEVICE_BUS: u8 = 1;

/// The highest link speed [`Pcie::init`] lets the link train to.
///
/// The Pi 5 connector is specified for Gen 2, which is what Linux uses
/// unless `dtparam=pciex1_gen=3`; Gen 3 is not certified, though widely
/// used with SSDs, and a link that will not train at it can be tried again
/// at Gen 2.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinkSpeed {
    /// 2.5 GT/s.
    Gen1 = 1,
    /// 5 GT/s.
    Gen2 = 2,
    /// 8 GT/s.
    Gen3 = 3,
}

/// What the link trained to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Link {
    /// The speed, as a PCIe generation (1 = 2.5 GT/s, 2 = 5 GT/s,
    /// 3 = 8 GT/s).
    pub generation: u8,
    /// The number of lanes; 1 on this connector.
    pub width: u8,
}

/// Why bring-up or enumeration failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The PHY's resistor calibration did not report done within 1 ms.
    Rescal,
    /// A write to the PHY over MDIO did not complete within 100 µs.
    Mdio,
    /// The controller is strapped as an endpoint, not a root complex.
    NotRootComplex,
    /// The link did not come up within 200 ms of releasing `PERST#`:
    /// nothing is plugged in, the device is unpowered, or it would not
    /// train at the speed asked for.
    LinkDown,
    /// The link is up but no function answered at bus 1, device 0,
    /// function 0 within a second.
    NoDevice,
    /// The function's BARs together do not fit in [`WINDOW_LEN`].
    WindowFull,
}

/// A function's place on the bus.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Address {
    /// Bus number; the plugged-in device is on bus 1.
    pub bus: u8,
    /// Device number, 0-31.
    pub device: u8,
    /// Function number, 0-7.
    pub function: u8,
}

/// A function found on bus 1 by [`Pcie::functions`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FunctionInfo {
    /// Where it answered.
    pub address: Address,
    /// Its vendor ID.
    pub vendor_id: u16,
    /// Its device ID.
    pub device_id: u16,
    /// Its class code: base class, subclass and programming interface in
    /// bits 23:0 (`0x01_08_02` is an NVMe controller).
    pub class: u32,
}

/// One memory BAR, as [`Pcie::enable`] placed it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bar {
    /// The CPU address it is reached at, inside the mapped window.
    pub address: usize,
    /// Its size in bytes.
    pub len: usize,
}

/// A function [`Pcie::enable`] has set up: its memory BARs are assigned,
/// and it decodes them and may master the bus.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Function {
    /// Identity, as [`Pcie::functions`] reported it.
    pub info: FunctionInfo,
    /// Its memory BARs, indexed by BAR number; a 64-bit BAR fills its
    /// first slot and leaves the next `None`, as do I/O BARs and
    /// unimplemented ones.
    pub bars: [Option<Bar>; 6],
}

/// The external PCIe connector's root complex, with the link trained.
pub struct Pcie {
    _pcie: PCIE1,
    link: Link,
    /// Bytes of [`WINDOW_LEN`] already given to BARs.
    window_used: usize,
}

impl Pcie {
    /// Resets and configures the controller, releases the device from
    /// reset and trains the link at up to `max_speed`, then waits for the
    /// function at bus 1, device 0 to answer.
    ///
    /// Takes about 120 ms with a device present: the 100 ms the PCIe
    /// specification gives a device after `PERST#`, plus training. Without
    /// one, [`Error::LinkDown`] after 200 ms.
    pub fn init(pcie: PCIE1, max_speed: LinkSpeed, timer: &Timer) -> Result<Self, Error> {
        let mut pcie = Self {
            _pcie: pcie,
            link: Link {
                generation: 0,
                width: 0,
            },
            window_used: 0,
        };

        // Linux's probe releases the bridge reset, then calibrates the
        // PHY (unless the firmware already did; see `rescal`).
        set_bridge_reset(false, timer);
        rescal(timer)?;

        pcie.setup(timer)?;
        pcie.start_link(max_speed, timer)?;
        pcie.setup_root_port();

        // A device may answer configuration requests with "retry" for a
        // while after the link comes up; the controller retries those
        // itself for up to its retry timeout (about 240 ms, set in
        // `post_setup`) and then reads all-ones. So poll for up to a second.
        let device = Address {
            bus: DEVICE_BUS,
            device: 0,
            function: 0,
        };
        let start = timer.now_micros();
        while pcie.read_config(device, 0) & 0xffff == 0xffff {
            if timer.now_micros() - start > 1_000_000 {
                return Err(Error::NoDevice);
            }
            timer.delay_ms(10);
        }
        Ok(pcie)
    }

    /// What the link trained to.
    pub fn link(&self) -> Link {
        self.link
    }

    /// Whether the link is up right now: the PHY has a link, and the data
    /// link layer is active.
    pub fn link_up(&self) -> bool {
        let status = self.read(PCIE_STATUS);
        status & STATUS_PHYLINKUP != 0 && status & STATUS_DL_ACTIVE != 0
    }

    /// Reads the configuration space dword at `offset` (4-byte aligned,
    /// below 4096) of the function at `address`.
    ///
    /// Bus 0 is the root port itself (device 0 only); anything else goes
    /// out over the link. A function that does not exist, or a link that
    /// is down, reads all-ones rather than faulting: the controller is set
    /// to answer failed reads that way.
    pub fn read_config(&self, address: Address, offset: u16) -> u32 {
        match self.config_register(address, offset) {
            // SAFETY: a register in the controller's configuration window.
            Some(register) => unsafe { register.read_volatile() },
            None => u32::MAX,
        }
    }

    /// Writes the configuration space dword at `offset` (4-byte aligned,
    /// below 4096) of the function at `address`. Ignored for a function
    /// [`read_config`](Self::read_config) could not reach either.
    pub fn write_config(&mut self, address: Address, offset: u16, value: u32) {
        if let Some(register) = self.config_register(address, offset) {
            // SAFETY: a register in the controller's configuration window.
            unsafe { register.write_volatile(value) };
        }
    }

    /// The functions answering on bus 1: device 0's function 0, and its
    /// functions 1-7 if it is a multi-function device. Behind a root port
    /// only device 0 exists.
    pub fn functions(&self) -> impl Iterator<Item = FunctionInfo> + '_ {
        let first = Address {
            bus: DEVICE_BUS,
            device: 0,
            function: 0,
        };
        let multi_function = self.read_config(first, HEADER_TYPE) & (0x80 << 16) != 0;
        let count = if multi_function { 8 } else { 1 };
        (0..count).filter_map(move |function| {
            let address = Address { function, ..first };
            let id = self.read_config(address, 0);
            if id & 0xffff == 0xffff {
                return None;
            }
            Some(FunctionInfo {
                address,
                vendor_id: id as u16,
                device_id: (id >> 16) as u16,
                class: self.read_config(address, CLASS_REVISION) >> 8,
            })
        })
    }

    /// Sizes the function's memory BARs and places each in the outbound
    /// window, naturally aligned, after anything an earlier call placed;
    /// widens the root port's memory window over them; then turns on the
    /// function's memory decoding and bus mastering, with its legacy
    /// interrupt disabled.
    ///
    /// Call it once per function: a second call places the BARs again,
    /// further along the window.
    pub fn enable(&mut self, info: FunctionInfo) -> Result<Function, Error> {
        let address = info.address;
        let command = self.read_config(address, COMMAND) & 0xffff;
        // Decoding off while the BARs read back sizes rather than
        // addresses.
        self.write_config(
            address,
            COMMAND,
            command & !(COMMAND_MEMORY | COMMAND_BUS_MASTER),
        );

        let mut bars = [None; 6];
        let mut index = 0;
        while index < 6 {
            let offset = BAR0 + 4 * index as u16;
            let original = self.read_config(address, offset);
            // Bit 0 set is an I/O BAR, which this window cannot reach.
            if original & 1 != 0 {
                index += 1;
                continue;
            }
            let is_64 = original & 0b110 == 0b100;

            self.write_config(address, offset, u32::MAX);
            let mut mask = u64::from(self.read_config(address, offset) & !0xf);
            if is_64 {
                self.write_config(address, offset + 4, u32::MAX);
                mask |= u64::from(self.read_config(address, offset + 4)) << 32;
            } else {
                mask |= 0xffff_ffff_0000_0000;
            }

            // An unimplemented BAR reads back zero however it is written.
            let unimplemented = if is_64 {
                mask == 0
            } else {
                mask & 0xffff_ffff == 0
            };
            if unimplemented {
                self.write_config(address, offset, 0);
            } else {
                let len = (!mask).wrapping_add(1);
                if len > WINDOW_LEN as u64 {
                    return Err(Error::WindowFull);
                }
                let len = len as usize;
                let start = self.window_used.next_multiple_of(len);
                if start + len > WINDOW_LEN {
                    return Err(Error::WindowFull);
                }
                self.window_used = start + len;
                // PCIe address = window offset: the window starts at
                // PCIe address 0.
                self.write_config(address, offset, start as u32);
                if is_64 {
                    self.write_config(address, offset + 4, 0);
                }
                bars[index] = Some(Bar {
                    address: WINDOW_BASE + start,
                    len,
                });
            }
            index += if is_64 { 2 } else { 1 };
        }

        // The root port forwards a memory request downstream only if it
        // falls in its memory window: base and limit in 1 MiB units, bits
        // 31:20 of each address in bits 15:4 (base) and 31:20 (limit).
        if self.window_used > 0 {
            let limit = (self.window_used.next_multiple_of(1 << 20) - 1) as u32;
            self.write_config(ROOT_PORT, BRIDGE_MEMORY_BASE_LIMIT, limit & 0xfff0_0000);
        }

        self.write_config(
            address,
            COMMAND,
            command | COMMAND_MEMORY | COMMAND_BUS_MASTER | COMMAND_INTX_DISABLE,
        );
        Ok(Function { info, bars })
    }

    /// Linux's `brcm_pcie_setup` plus its BCM2712 `post_setup`: reset the
    /// bridge, power the SerDes, set the windows, and apply the BCM2712's
    /// PHY reference-clock setting and bus-error behaviour.
    fn setup(&mut self, timer: &Timer) -> Result<(), Error> {
        set_bridge_reset(true, timer);
        timer.delay_us(200);
        set_bridge_reset(false, timer);

        // Hold the device in reset until `start_link`. Linux leaves this
        // to the register's reset value on the BCM2712; written here so a
        // device left running by a bootloader that booted from it gets a
        // fundamental reset all the same.
        self.modify(PCIE_CTRL, PCIE_CTRL_PERSTB, 0);

        self.modify(HARD_DEBUG, HARD_DEBUG_SERDES_IDDQ, 0);
        // SerDes settle time.
        timer.delay_us(200);

        self.modify(
            MISC_CTRL,
            MISC_CTRL_MAX_BURST_MASK,
            MISC_CTRL_SCB_ACCESS_EN
                | MISC_CTRL_CFG_READ_UR_MODE
                | MISC_CTRL_MAX_BURST_512
                | MISC_CTRL_RCB_MPS_MODE
                | MISC_CTRL_RCB_64B_MODE,
        );

        // Inbound window 1: PCIe `DMA_OFFSET` onward is CPU 0 onward, the
        // size encoded as log2(bytes) - 15 for 64 KiB to 64 GiB.
        let size_code = DMA_LEN.trailing_zeros() - 15;
        self.write(RC_BAR1_CONFIG_LO, DMA_OFFSET as u32 | size_code);
        self.write(RC_BAR1_CONFIG_HI, (DMA_OFFSET >> 32) as u32);
        self.write(UBUS_BAR1_CONFIG_REMAP, UBUS_REMAP_ACCESS_EN);
        self.write(UBUS_BAR1_CONFIG_REMAP_HI, 0);

        if self.read(PCIE_STATUS) & STATUS_PORT == 0 {
            return Err(Error::NotRootComplex);
        }

        // Advertise ASPM L0s and L1, as Linux does for `pcie1` (it has no
        // `aspm-no-l0s`). Nothing here enables either on the device.
        self.modify(PRIV1_LINK_CAPABILITY, 0xc00, 0x3 << 10);
        // Report a PCI-to-PCI bridge's class, so the root port reads as
        // one; its reset value is an endpoint's.
        self.modify(PRIV1_ID_VAL3, 0xff_ffff, 0x06_0400);

        // Outbound window 0: CPU `WINDOW_BASE` onward is PCIe 0 onward.
        let base_mb = WINDOW_BASE / (1 << 20);
        let limit_mb = (WINDOW_BASE + WINDOW_LEN - 1) / (1 << 20);
        self.write(MEM_WIN0_LO, 0);
        self.write(MEM_WIN0_HI, 0);
        self.modify(
            MEM_WIN0_BASE_LIMIT,
            0xfff0_fff0,
            ((limit_mb as u32 & 0xfff) << 20) | ((base_mb as u32 & 0xfff) << 4),
        );
        self.modify(MEM_WIN0_BASE_HI, 0xff, (base_mb >> 12) as u32);
        self.modify(MEM_WIN0_LIMIT_HI, 0xff, (limit_mb >> 12) as u32);

        // Little-endian inbound data.
        self.modify(VENDOR_SPECIFIC_REG1, 0xc, 0);

        self.post_setup(timer)
    }

    /// Linux's `brcm_pcie_post_setup_bcm2712`, minus the QoS maps Pi 5
    /// device trees leave unset.
    fn post_setup(&mut self, timer: &Timer) -> Result<(), Error> {
        // Let the PHY's PLL run from the 54 MHz crystal: register writes on
        // its MDIO port 0, page 0x1600 (register 0x1f selects the page).
        // Values are Linux's; no document says what each one is.
        const REGISTERS: [u8; 7] = [0x16, 0x17, 0x18, 0x19, 0x1b, 0x1c, 0x1e];
        const VALUES: [u16; 7] = [0x50b9, 0xbda1, 0x0094, 0x97b4, 0x5030, 0x5030, 0x0007];
        self.mdio_write(0x1f, 0x1600, timer)?;
        for (register, value) in REGISTERS.into_iter().zip(VALUES) {
            self.mdio_write(register, value, timer)?;
        }
        timer.delay_us(200);

        // L1 substate timers count this PM clock period: 18.52 ns
        // (1/54 MHz) rounded down.
        self.modify(PL_PHY_CTL_15, 0xff, 0x12);

        // Turn a failed read (nothing there, or the link down) into
        // all-ones data rather than a bus error, which would be an
        // asynchronous abort on this core.
        self.modify(
            UBUS_CTRL,
            0,
            UBUS_CTRL_REPLY_ERR_DIS | UBUS_CTRL_REPLY_DECERR_DIS,
        );
        self.write(AXI_READ_ERROR_DATA, u32::MAX);

        // Bus timeout 250 ms and configuration retry timeout about 240 ms,
        // in 750 MHz clocks: the retry is cut short if either expires.
        self.write(UBUS_TIMEOUT, 0x0b2d_0000);
        self.write(RC_CONFIG_RETRY_TIMEOUT, 0x0aba_0000);

        // Linux's workarounds for the QoS logic: forwarding search off,
        // the BCM2712 D0 fix bits on, and where the timing fix is absent
        // (C1 silicon, or a one-lane controller like this one) at most 15
        // AXI requests in flight to RAM.
        self.modify(
            AXI_INTF_CTRL,
            AXI_REQFIFO_EN_QOS_PROPAGATION,
            AXI_EN_RCLK_QOS_ARRAY_FIX | AXI_EN_QOS_UPDATE_TIMING_FIX | AXI_DIS_QOS_GATING_IN_MASTER,
        );
        if self.read(AXI_INTF_CTRL) & AXI_EN_QOS_UPDATE_TIMING_FIX == 0 {
            self.modify(AXI_INTF_CTRL, AXI_MASTER_MAX_OUTSTANDING_MASK, 15);
        }
        // Vendor-defined QoS messages off.
        self.modify(MISC_CTRL_1, MISC_CTRL_1_EN_VDM_QOS_CONTROL, 0);
        Ok(())
    }

    /// Linux's `brcm_pcie_start_link`: cap the speed, release `PERST#`,
    /// wait for the link, then set the reference clock to "safe" mode.
    fn start_link(&mut self, max_speed: LinkSpeed, timer: &Timer) -> Result<(), Error> {
        let generation = max_speed as u32;
        self.modify(PRIV1_LINK_CAPABILITY, 0xf, generation);
        self.modify(LINK_CONTROL_2, 0xf, generation);

        self.modify(HARD_DEBUG, HARD_DEBUG_CLKREQ_MASK, 0);
        self.modify(PCIE_CTRL, 0, PCIE_CTRL_PERSTB);

        // PCIe CEM 2.2: 100 ms from PERST# release before the first
        // configuration request. Then up to 100 ms more for the link.
        timer.delay_ms(100);
        let mut waited = 0;
        while !self.link_up() {
            if waited >= 100 {
                return Err(Error::LinkDown);
            }
            timer.delay_ms(5);
            waited += 5;
        }

        // "safe" clkreq mode, which the Pi 5 device tree asks for on
        // `pcie1` (`brcm,clkreq-mode`): the root complex drives the
        // reference clock unconditionally, ignoring CLKREQ#, which the
        // connector may leave unconnected. L1 substates need CLKREQ#, so
        // stop advertising them (mode 2).
        self.modify(
            HARD_DEBUG,
            HARD_DEBUG_CLKREQ_MASK,
            HARD_DEBUG_REFCLK_OVRD_OUT | HARD_DEBUG_REFCLK_OVRD_ENABLE,
        );
        self.modify(PRIV1_ROOT_CAP, 0xf8, 2 << 3);

        let status = self.read(LINK_CONTROL_STATUS);
        self.link = Link {
            generation: ((status >> 16) & 0xf) as u8,
            width: ((status >> 20) & 0x3f) as u8,
        };
        Ok(())
    }

    /// What the Linux PCI core does to the root port when it enumerates:
    /// number the buses behind it, close its I/O and prefetchable windows
    /// (the memory window opens in [`enable`](Self::enable)), and let it
    /// forward memory requests both ways.
    fn setup_root_port(&mut self) {
        // Primary 0, secondary and subordinate `DEVICE_BUS`.
        let buses = (u32::from(DEVICE_BUS) << 16) | (u32::from(DEVICE_BUS) << 8);
        self.write_config(ROOT_PORT, BRIDGE_BUSES, buses);
        // A base above its limit closes a window. The upper halves of the
        // I/O word are the secondary status, write-1-to-clear, so zeros
        // there leave it alone.
        self.write_config(ROOT_PORT, BRIDGE_IO_BASE_LIMIT, 0x00f0);
        self.write_config(ROOT_PORT, BRIDGE_MEMORY_BASE_LIMIT, 0x0000_fff0);
        self.write_config(ROOT_PORT, BRIDGE_PREFETCH_BASE_LIMIT, 0x0000_fff0);
        self.write_config(ROOT_PORT, BRIDGE_PREFETCH_BASE_HI, 0);
        self.write_config(ROOT_PORT, BRIDGE_PREFETCH_LIMIT_HI, 0);
        // The status half is write-1-to-clear here too.
        let command = self.read_config(ROOT_PORT, COMMAND) & 0xffff;
        self.write_config(
            ROOT_PORT,
            COMMAND,
            command | COMMAND_MEMORY | COMMAND_BUS_MASTER,
        );
    }

    /// The configuration register for `address`/`offset`, or `None` for a
    /// function the root complex cannot reach (a second device on bus 0).
    fn config_register(&self, address: Address, offset: u16) -> Option<*mut u32> {
        assert!(
            offset < 4096 && offset.is_multiple_of(4),
            "bad config offset {offset:#x}"
        );
        if address.bus == 0 {
            // The root port's own configuration space is the start of the
            // register file.
            return (address.device == 0 && address.function == 0)
                .then(|| (PCIE1::ADDR + usize::from(offset)) as *mut u32);
        }
        // The ECAM-style index: bus in bits 27:20, device 19:15, function
        // 14:12.
        let index = (u32::from(address.bus) << 20)
            | (u32::from(address.device & 0x1f) << 15)
            | (u32::from(address.function & 0x7) << 12);
        self.write(EXT_CFG_INDEX, index);
        Some((PCIE1::ADDR + EXT_CFG_DATA + usize::from(offset)) as *mut u32)
    }

    /// Writes `value` to PHY register `register` over MDIO port 0.
    fn mdio_write(&mut self, register: u8, value: u16, timer: &Timer) -> Result<(), Error> {
        // Port 0 in bits 19:16 (and 21), command 0 (write) in bit 20.
        self.write(MDIO_ADDR, u32::from(register));
        self.read(MDIO_ADDR);
        self.write(MDIO_WR_DATA, MDIO_DATA_DONE | u32::from(value));
        for _ in 0..10 {
            if self.read(MDIO_WR_DATA) & MDIO_DATA_DONE == 0 {
                return Ok(());
            }
            timer.delay_us(10);
        }
        Err(Error::Mdio)
    }

    fn read(&self, offset: usize) -> u32 {
        // SAFETY: a register of the controller this owns.
        unsafe { ((PCIE1::ADDR + offset) as *const u32).read_volatile() }
    }

    fn write(&self, offset: usize, value: u32) {
        // SAFETY: a register of the controller this owns.
        unsafe { ((PCIE1::ADDR + offset) as *mut u32).write_volatile(value) }
    }

    /// Clears `clear` then sets `set` in the register at `offset`.
    fn modify(&self, offset: usize, clear: u32, set: u32) {
        self.write(offset, (self.read(offset) & !clear) | set);
    }
}

/// The root port: bus 0, device 0.
const ROOT_PORT: Address = Address {
    bus: 0,
    device: 0,
    function: 0,
};

/// Asserts or releases `pcie1`'s bridge reset. Releasing waits 200 µs, the
/// time Linux's `brcmstb-reset` gives a block to come out of it.
fn set_bridge_reset(asserted: bool, timer: &Timer) {
    let bank = SW_INIT + (BRIDGE_RESET_LINE / 32) * 0x18;
    // `SET` at +0, `CLEAR` at +4.
    let register = if asserted { bank } else { bank + 4 };
    // SAFETY: the reset controller's set/clear words, which act only on
    // the bits written.
    unsafe { (register as *mut u32).write_volatile(1 << (BRIDGE_RESET_LINE % 32)) };
    if !asserted {
        timer.delay_us(200);
    }
}

/// Runs the PHY resistor calibration and waits for it, as Linux's
/// `reset-brcmstb-rescal.c` does on the "rescal" reset's deassert — unless
/// it already reports done.
///
/// The calibration trims a resistor shared by all three controllers'
/// PHYs, and by the time this runs the firmware has calibrated it to bring
/// up RP1's link, which carries the console. Recalibrating under that live
/// link would be a risk taken for nothing, so a calibration already done
/// is kept.
fn rescal(timer: &Timer) -> Result<(), Error> {
    let start = (RESCAL + RESCAL_START) as *mut u32;
    let status = (RESCAL + RESCAL_STATUS) as *const u32;
    // SAFETY: the calibration block's own registers.
    unsafe {
        if status.read_volatile() & 1 != 0 {
            return Ok(());
        }
        start.write_volatile(start.read_volatile() | 1);
        if start.read_volatile() & 1 == 0 {
            return Err(Error::Rescal);
        }
        let mut done = false;
        for _ in 0..10 {
            if status.read_volatile() & 1 != 0 {
                done = true;
                break;
            }
            timer.delay_us(100);
        }
        start.write_volatile(start.read_volatile() & !1);
        if done {
            Ok(())
        } else {
            Err(Error::Rescal)
        }
    }
}

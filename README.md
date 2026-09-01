# rpi-hal

[![CI](https://img.shields.io/github/actions/workflow/status/joeferner/rpi-hal/ci.yml?branch=main&label=CI)](https://github.com/joeferner/rpi-hal/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/rpi-hal.svg)](https://crates.io/crates/rpi-hal)
[![docs.rs](https://img.shields.io/docsrs/rpi-hal)](https://docs.rs/rpi-hal)

Hardware abstraction layer for the Raspberry Pi.

A green badge means it compiles, and nothing more: every check that runs in
CI is a compile-time one, because the hardware-in-the-loop tests need a real
board and a fixture wired to it. Whether a driver *works* is established on
hardware, not there.

Supported Devices:

- Pi 2 Model B rev 1.1 (BCM2836, Cortex-A7)
- Pi 3 Model B v1.2 (BCM2837, Cortex-A53)
- Pi 4 (BCM2711, Cortex-A72) — **preliminary**: the `bcm2711` feature
  selects the relocated peripheral memory map and PAC. HW-verified on
  real Pi 4 hardware in both 32-bit (`armv7a-none-eabi`) and 64-bit
  (`aarch64-unknown-none-softfloat`) builds: boot/GPIO/System Timer
  (`blink`), the MMU identity map and mailbox coherency
  (`aarch64_smoke`), the UART console, and the SD card via the
  BCM2711-specific `EMMC2` controller (`sd_read`, `rpi-loader`'s
  `sd-list`/`sd-read`/etc.). Most other drivers are still untested and
  there is no interrupt controller at all yet — see
  [issue #29](https://github.com/joeferner/rpi-hal/issues/29) for exactly
  what's verified and the bring-up plan for the rest.

Building for any of these requires picking exactly one of the
`bcm2837`/`bcm2711` features (see "Features" below) — neither is a
default, since there's no sensible default target chip.

## Toolchain

Stable Rust 1.88 or newer, and one of the two bare-metal targets:

```
rustup target add armv7a-none-eabi              # 32-bit, kernel7.img
rustup target add aarch64-unknown-none-softfloat # 64-bit, kernel8.img
```

1.88 is the floor because the FPU bring-up needs `#[unsafe(naked)]` and
`naked_asm!`. Enabling the `smoltcp` feature raises it to 1.91, which is
smoltcp's own requirement rather than this crate's.

A binary also needs the linker script that sets the load address and
brackets `.bss` for the boot code, which this crate publishes on the
linker's search path as `rpi-link.x` — so pointing at it is one line, the
same line for both architectures, and there's no script to copy or build
script to write:

```toml
# .cargo/config.toml
[target.aarch64-unknown-none-softfloat]
rustflags = ["-C", "link-arg=-Trpi-link.x"]
```

Nightly isn't needed. `rustup` ships a precompiled `core` and `alloc` for
both targets, so stable links a complete kernel image — `alloc` included,
if you supply a `#[global_allocator]`. This repository does pin nightly in
its own `rust-toolchain.toml` and build `core`/`alloc` from source
(`-Zbuild-std`) for its examples, but that's a local convenience and not
something a consumer inherits.

## Interrupts: the `__irq_handler` contract

Read this before writing anything interrupt-driven — a missing or
incomplete handler is the one mistake here that produces no error message
at all.

Interrupt dispatch belongs to the application. The `rt` feature installs
the exception vector table, which saves the caller-saved registers and
branches to a single symbol the application defines:

```rust
#[no_mangle]
pub extern "C" fn __irq_handler() {
    let lic = Lic::new(unsafe { pac::Peripherals::steal() }.LIC);

    if lic.is_gpio_pending(BUTTON) {
        let mut button =
            unsafe { Pin::<BUTTON, Input>::assume_mode(pac::Peripherals::steal().GPIO) };
        button.clear_interrupt();
        // ... and whatever this interrupt was for.
    }
}
```

Every live source arrives at that one function, which tests each
`is_*_pending` in turn; there is no registration call and no callback
table. Three gates all have to be open before it runs — the CPU mask
(`irq::enable_irq`), the interrupt controller routing the source
(`Lic::enable_*_irq`), and the peripheral configured to raise it.

**Both ways of getting it wrong look like a hang.** An interrupt that is
never cleared is still asserted when the handler returns, so the core takes
it again immediately, forever — no panic, no fault, no output, and the
apparent freeze is wherever the program happened to be rather than
anywhere near the cause:

- **Not defining `__irq_handler`.** The vector table's branch target is a
  *weak* no-op, so the link succeeds and the program behaves normally until
  the first interrupt fires. The strong definition must also reach the final
  binary: a `#[no_mangle]` function in a library the binary never
  references may not be linked at all, which leaves the weak stub in place
  and is indistinguishable from having written no handler.
- **Returning without clearing the source** — including clearing the wrong
  one, or handling one source while a second is still asserted. A few
  peripherals don't ack with a write-1-to-clear; the ARM generic timer, for
  one, acks a fired tick by moving its comparator forward, and each
  driver's documentation says which it is.

Under the `async` feature the handler usually just calls the driver's
`on_irq` (`gpio::on_irq`, `uart::on_irq`), which clears the source and
wakes the stored waker. Out-of-crate drivers needing an interrupt expose
their own equivalent — the `rpi-hal-embassy` crate's time driver is one —
because dispatch is application-owned, so nothing can register itself.

Worked examples: `examples/gpio_irq_button.rs` (one source),
`examples/uart_rx_irq_echo.rs` (two sources dispatched independently, with
a ring buffer shared via `critical_section::with`),
`examples/irq_timer_blink.rs` (an LED driven entirely from the handler).

## Status

Boot/startup runtime (`src/boot.s`, `linker.ld`, exception vectors),
plus GPIO, UART0, SPI0, I2C1, the BCM System Timer, CPU-level IRQ
enable/disable plus the legacy interrupt controller, a
`critical-section` implementation, and the VideoCore mailbox/
framebuffer — all with `embedded-hal`/`embedded-io` trait
implementations where applicable, and all verified on real hardware:

- **GPIO** (`src/gpio.rs`): typestated `Pin<const N: u8, MODE>`, generic
  over all 54 pins, with `embedded_hal::digital` traits. Inputs also
  support edge/level interrupts (`enable_interrupt(Trigger)` +
  `clear_interrupt`, routed via `Lic::enable_gpio_irq`) and blocking
  `wait_for_high`/`wait_for_low` — see `examples/gpio_irq_button.rs`. The
  internal pull resistors are configurable per pin (`set_pull(Pull)`,
  `into_pull_up_input`/`into_pull_down_input`/`into_floating_input`), so a
  button needs no external resistor — see `examples/gpio_pull.rs`. Note
  that no pin arrives floating: each powers up with the pull its datasheet
  pin-table entry gives it, which the boot firmware may then change, and
  nothing but these calls touches it.
- **UART0** (`src/uart.rs`): blocking read/write plus interrupt-driven
  RX (`enable_rx_irq`/`try_read_byte`), `embedded_io::Read`/`Write`.
- **SPI0** (`src/spi.rs`): `embedded_hal::spi::SpiBus`, both
  hardware-driven chip-selects (`ChipSelect::Cs0`/`Cs1`) or
  externally-managed (`ChipSelect::None`). Verified against a real
  independent STM32 fixture (see `bench-link`, below), not just a
  MOSI→MISO loopback. `init` takes a raw `CDIV`; `spi::divider_for`
  turns a target SCLK into one — see the note under I2C below, which
  applies to both buses.
- **I2C** (`src/i2c.rs`): `embedded_hal::i2c::I2c`, master-only
  (matches this hardware), generic over the BSC instance. `I2c<BSC1>`
  drives I2C1 on the 40-pin header (GPIO2/3), verified against three real
  devices/checks: a DS1307 RTC, an SH1106 OLED, and a full bus scan.
  Examples on that bus: `examples/i2c_scan.rs`, `i2c_sh1106_oled.rs`,
  `i2c_sht41.rs` (an SHT41 temperature/humidity sensor) and
  `i2c_ads1115.rs` (an ADS1115 16-bit ADC).
  `I2c<BSC0>` drives BSC0 on either of its two routings, one controller
  and two pin pairs: `init` takes GPIO44/45 (ALT1), the Pi 3
  camera/display connector bus, used to read an OV5647 camera sensor's
  chip ID (see `examples/camera_probe.rs`); `init_id` takes GPIO0/1
  (ALT0), `ID_SD`/`ID_SC` on header pins 27/28 — the HAT ID EEPROM bus,
  where a board's identity and per-unit calibration live (see
  `examples/i2c_hat_eeprom.rs`). Only one of the two can be live at a
  time, which is why the choice is a constructor rather than an argument.

  Despite the "reserved for HAT ID EEPROM detection" warning those pins
  carry in Raspberry Pi's own documentation, a bare-metal program is free
  to take them: the firmware reads the EEPROM early in boot, before the
  kernel image runs, and then leaves the pins alone, and the board fits
  1.8k pull-ups on both lines. What the warning still means is that a
  fitted HAT may expect to be the only thing on that bus.

  Both buses take a raw divider rather than a frequency, because the
  core clock they divide is not a constant: it moves with `config.txt`
  and with the firmware's own scaling. The reset default of 1500 is
  called 100kHz on the strength of the datasheet's nominal 150MHz core,
  and is 166kHz on a board running 250MHz. `i2c::divider_for(core_hz,
  target_hz)` and `spi::divider_for` do the conversion, including the
  rounding these registers need — always upwards, so the bus never
  clocks faster than asked, since what a device states is a maximum:

  ```rust
  let core_hz = mailbox.clock_rate_hz(ClockId::Core)?;
  let i2c = I2c::init(&gpio, bsc1, i2c::divider_for(core_hz, 100_000), &timer);
  ```

  The mailbox query stays in the caller's hands rather than being folded
  into `init`: it can fail, it costs a round trip to the GPU, and an
  application that brings up several buses wants to ask once.

  With the `async` feature the same type also implements
  `embedded_hal_async::i2c::I2c`, parking on the controller's
  `DONE`/`TXW`/`RXR` interrupts instead of spinning — the millisecond a
  six-byte read at 100kHz costs goes to the executor rather than to a
  polling loop. It needs the usual wiring: `Lic::enable_i2c_irq`, the CPU
  mask, and `i2c::on_irq` called from `__irq_handler`. Timeouts work
  differently there and deliberately: wrap the future in your executor's
  own (`embassy_time::with_timeout`), which puts the number where the
  application's judgement is. Dropping a transfer part-way is safe —
  the controller is left masked, cleared and ready for the next one.

  `init` takes a `&Timer` because every blocking transfer is bounded
  against the System Timer. I2C is the one bus here where a *foreign* device decides
  whether a transfer finishes: a slave that acknowledges its address and
  then stops driving sets neither `S.ERR` nor `S.DONE`, so an unbounded
  poll never returns and, this being a blocking driver, takes the rest of
  the program (an executor, a network stack) with it. On expiry the
  caller gets `Error::Timeout`, or `Error::Incomplete { received,
  requested }` when the transfer did finish but the slave delivered fewer
  bytes than were asked for — how many arrived is what distinguishes a
  mute device from a truncating one. The controller is then returned to a
  usable baseline on a best-effort basis (FIFOs and status cleared);
  nothing can make a slave that is holding SDA let go, so a genuinely
  stuck bus simply times out again, which is survivable where a hang
  isn't.

  One thing a bus scan can't tell you: `examples/i2c_scan.rs` probes with
  a 1-byte read (`DLEN=0` isn't a real transaction on this hardware — see
  `i2c::Error::ZeroLengthUnsupported`), so it enumerates what answers
  *reads*, which is not the same as what is on the bus. A device that
  only answers a read while it has a result pending — every Sensirion
  SHT4x, among others — is reported absent while happily acknowledging
  commands. `i2cdetect` finds those because it probes with a zero-length
  write, which the BSC cannot issue at all.
- **System Timer** (`src/timer.rs`): free-running microsecond counter,
  `delay_us`/`delay_ms`, `embedded_hal::delay::DelayNs`.
- **ARM generic timer** (`src/generic_timer.rs`): the per-core architected
  timer, distinct from the shared System Timer above — a monotonic
  `now()`/`frequency()` counter, blocking delays + `DelayNs`, and
  interrupt-driven deadlines (`arm_after_us`/`set_deadline`) routed to the
  calling core through the ARM-local interrupt controller — a per-core
  compare interrupt, where the System Timer's is global. Proven end-to-end
  via `examples/irq_generic_timer_uart.rs`.

  Not what `rpi-hal-embassy`'s `embassy-time` driver runs on, despite the
  better primitives: its 64-bit compare needs no clamping and it would
  suit per-core executors, but at 19.2MHz it matches none of the fixed
  `tick-hz-*` rates `embassy-time` offers, so a global timebase built on
  it would need lossy scaling on every `now()`. The System Timer's 1MHz
  maps exactly, so that is what the driver uses; this one is held for
  per-core deadlines if per-core executors are added.
- **Interrupts**: exception vector table, CPU-level enable/disable
  (`src/irq.rs`), the legacy interrupt controller (`src/lic.rs`), and
  a `critical-section` implementation (`src/critical_section.rs`) —
  proven end-to-end via `examples/irq_timer_blink.rs` (LED toggled
  entirely from an IRQ handler), `examples/uart_rx_irq_echo.rs`
  (two interrupt sources dispatched independently, a real ring buffer
  shared via `critical_section::with`), and `examples/gpio_irq_button.rs`
  (a GPIO input edge waking the core from `wfe`). Dispatch is the
  application's — see "Interrupts: the `__irq_handler` contract" above
  before using any of it.
- **MMU** (`src/mmu.rs`, `mmu` feature): identity-mapped, RAM as Normal
  (Cacheable, Shareable) memory, peripheral MMIO kept as Device — makes
  `core::sync::atomic` (`ldrex`/`strex`) actually work, which it
  architecturally can't on the Strongly-Ordered/Device memory every
  address defaulted to with the MMU off. Needs both Shareable *and*
  Cacheable, not just Shareable — this core's local exclusive monitor
  ties into cache line state, so `strex` never succeeds against
  Non-cacheable memory here regardless of shareability. See "Features"
  below for how to supply your own table instead.
- **Mailbox / 2D framebuffer** (`src/mailbox.rs`): the VideoCore
  property-interface RPC channel — clock rates, board/firmware info,
  ARM/VC memory split, power-domain control, die temperature and
  throttling status, and a mailbox-allocated
  scanout framebuffer (`Framebuffer::flush()` writes back cache lines
  before VideoCore reads them).

  `temperature_millicelsius` is the SoC's own thermometer (`58_000` is
  58°C), and it is worth reading with `throttled` beside it: the
  firmware caps the ARM clock as the die heats, so thermal throttling
  reaches a bare-metal program as its code inexplicably getting slower
  rather than as any kind of event. `throttled`'s word has two halves —
  bits 0-3 for what is happening now, bits 16-19 sticky since boot,
  which is the only way to see an under-voltage dip that has already
  passed. `examples/soc_temperature.rs` prints all three once a second. Tear-free output is available too:
  `allocate_framebuffer_paged` asks for a buffer several screens tall and
  `set_virtual_offset` brings a finished page on screen in one step, so
  nothing is ever written to the page being scanned out (there is also
  `wait_for_vsync`, for drawing straight into a single buffer).
  Resolution/timing negotiation with the attached display is handled
  entirely by VideoCore firmware, not this crate — but `display_size`
  reports what it settled on, which is what a framebuffer request should
  be sized from, since the firmware scales a buffer that doesn't match
  the real mode instead of refusing it. Mind the overscan border
  (`overscan`/`set_overscan`): the reported size is the image *inside*
  it, so a 1080p HDMI display answers 1824x984 with the stock 48-pixel
  border, and covering the whole screen means clearing the border and
  allocating at the size it was hiding. `edid_block` reads the display's
  own description of itself (the modes it supports, not just the one
  firmware chose) as raw bytes; EDID is display-standard wire format with
  no Pi in it, so parsing it is left to the consumer the same way TCP/IP
  is left to smoltcp — `examples/display_edid.rs` does it. See
  `examples/display_test_pattern.rs`, `examples/
  display_page_flip.rs` (the same animation drawn both ways, so the
  tearing and its absence can be compared on a real display), and
  `examples/display_touch.rs` (combining it with the FT5406 touch
  driver, `src/touch.rs`).
- **SD card** (`src/sd.rs`): the Arasan EMMC host controller — card
  identification (`CMD0`/`CMD8`/`ACMD41`/`CMD2`/`CMD3`/`CMD7`), a
  best-effort switch to the 4-bit bus (`ACMD51`/`ACMD6`, falling back
  to 1-bit for a card that doesn't support it), and 512-byte block
  transfers: single-block (`CMD17`/`CMD24`), multi-block for a
  consecutive run in one command (`CMD18`/`CMD25` with an auto-`CMD12`
  stop, `read_blocks`/`write_blocks`), and DMA-backed variants
  (`read_blocks_dma`/`write_blocks_dma`) that move the data phase over the
  system DMA controller (`src/dma.rs`) through the EMMC FIFO's DREQ
  instead of the CPU. Verified on real hardware by reading a card's boot
  sector and checking its `0x55AA` signature (`examples/sd_read.rs`), and
  by cross-checking the polled and DMA multi-block read paths against each
  other (`examples/sd_multi_block.rs`). Files on the boot FAT partition
  can be read on top of this through either of two FAT crates, each behind
  a feature adding its own `BlockDevice` adapter over the driver: the
  `embedded-sdmmc` feature adds `sd::SdCard` (`examples/sd_fat_read.rs`
  mounts the boot partition and reads files; `examples/sd_fat_write.rs`
  writes a random value to a scratch `TEST.TXT` and reads it back to verify
  the round-trip), and the `resident-fat` feature adds `sd::SdBlockDevice`
  (`examples/sd_resident_fat_read.rs`). Both wire `read` and `write` to the
  driver's polled multi-block paths; the difference is that `resident-fat`
  transfers byte slices spanning a whole run, which reach the driver with no
  staging buffer and no copy in between. Card-detect (GPIO47) is not
  implemented yet
  ([issue #14](https://github.com/joeferner/rpi-hal/issues/14)).

  With the `async` feature every one of those transfer methods gains a
  `_async` twin (`read_blocks_async`, `write_blocks_dma_async`, …) that
  parks on the controller's interrupt where the blocking one spins. The
  wait that pays for it is the `DATA_DONE` closing a write: it only
  arrives once the card has programmed a whole internal erase block, so a
  blocking write hands the CPU nothing back for milliseconds at a time. It
  needs the usual wiring — `Lic::enable_emmc_irq`, the CPU mask, and
  `sd::on_irq` called from `__irq_handler` — and takes `&mut Sd` where the
  blocking methods take `&self`, since one waker slot cannot serve two
  transfers at once. Timeouts belong to the caller
  (`embassy_time::with_timeout`); dropping a transfer part-way stops the
  card and resets the controller's data circuit before the drop returns,
  so the next transfer starts clean rather than reading the abandoned
  one's leftovers. `examples/sd_async.rs` runs both paths against each
  other and reports how much of each transfer the core spent idle.
  BCM2836/7 only: routing the line needs the legacy interrupt controller.

  On BCM2711 (Pi 4), the physical SD slot is wired to a different
  controller entirely — `EMMC2`, not the classic `EMMC` — so `bcm2711`
  switches this driver to `sd::Emmc2` (not in the PAC, wired up by
  reusing its own `emmc::RegisterBlock` type at EMMC2's address) via
  `Sd::steal_emmc`, plus a mailbox clock-enable and a `POWER_CONTROL`
  register write BCM2711 requires that older chips don't. HW-verified
  (32-bit only) the same way: `examples/sd_read.rs` and `rpi-loader`'s
  `sd-list`. The DMA-backed variants aren't available under `bcm2711`
  yet — see [issue #29](https://github.com/joeferner/rpi-hal/issues/29).
- **Wi-Fi SDIO** (`src/sdio.rs`, Pi 3 only): the on-board BCM43438
  wireless chip's SDIO interface, over the *same* Arasan EMMC controller
  as the SD card but routed to the wireless pins (GPIO34-39). Enumerates
  the SDIO card (`CMD0`/`CMD5`/`CMD3`/`CMD7`) at the 400kHz
  identification clock, then switches to the 4-bit bus at 25MHz; enables
  the function-1 backplane register window and reads chip registers over
  it both a register at a time (`CMD52`) and in bulk blocks through the
  DATA FIFO (`CMD53`), with the chip's ChipCommon ID as end-to-end proof
  of the whole path. The radio is powered by the boot firmware; a
  `WL_ON` assertion via the VideoCore GPIO expander is attempted
  best-effort. This completes the SDIO host controller. On top of it,
  `Sdio::load_firmware` walks the chip's core enumeration ROM, downloads
  the BCM43430's firmware image into its RAM and the nvram at the top of
  RAM, and starts the on-chip CPU — reporting success once the WLAN data
  function (F2) comes ready. Once firmware is running, `wifi::Wifi`
  (`src/wifi.rs`) wraps the `Sdio` and speaks Broadcom's SDPCM framing and
  CDC control protocol over function 2: `get_iovar` round-trips a
  firmware variable (reading back the firmware version string and the
  chip's MAC address over the air), `scan` lists nearby access points,
  and `join_wpa2` associates with a WPA2-PSK network — driving the chip's
  in-firmware supplicant and returning the AP's BSSID once connected. The
  regulatory (CLM) blob must be loaded first (`load_clm`); see the file
  notes below. Network data frames move over the SDPCM data channel
  wrapped in a BDC header (`send_ethernet`/`recv_ethernet`), and — with
  the `smoltcp` feature — `wifi::WifiPhy` presents that as a
  `phy::Device`, so a full TCP/IP stack runs on top. `examples/wifi_scan.rs`
  reads the blobs off the SD card, brings the chip up (with the ChipCommon
  ID as a liveness check), and scans; `examples/wifi_smoltcp.rs` does the
  same, joins the network in `WIFI.CFG`, then gets an address over DHCP
  and answers pings and a UDP echo. Driving this gives up the SD card slot
  (one controller, two possible routes — it hands the slot pins to SDHOST
  to claim the controller).

  The firmware download needs Broadcom's proprietary firmware blobs,
  which aren't redistributable here. Obtain them and place them in a
  `wifi` directory on the FAT boot partition of the SD card, under 8.3
  names (which the FAT reader resolves most reliably) — `wifi/FW.BIN`,
  `wifi/NVRAM.TXT`, and `wifi/CLM.DAT`:

  - `brcmfmac43430-sdio.bin` → `wifi/FW.BIN`: the firmware image, from the
    `linux-firmware` collection (e.g.
    `git clone --depth 1 https://gitlab.com/kernel-firmware/linux-firmware.git`).
    In a fresh clone the file is at `cypress/cyfmac43430-sdio.bin` (the
    original Pi 3B's 43430 is a Cypress part; `brcm/brcmfmac43430-sdio.bin`
    is only an install-time symlink to it, so it isn't in the raw
    checkout). Equivalently, copy `brcmfmac43430-sdio.bin` from
    `/lib/firmware/brcm/` on any Raspberry Pi OS install.
  - `brcmfmac43430-sdio.txt` → `wifi/NVRAM.TXT`: the board nvram, from
    [RPi-Distro/firmware-nonfree](https://github.com/RPi-Distro/firmware-nonfree)
    at `debian/added-firmware/brcm/brcmfmac43430-sdio.txt` (the
    `…raspberrypi,3-model-b.txt` name is a symlink to it), or the same
    `/lib/firmware/brcm/`.

  - `cyfmac43430-sdio.clm_blob` → `wifi/CLM.DAT`: the CLM regulatory
    blob (channel/country database). The Cypress `.bin` does *not* bake
    this in — the radio comes up but `country` reads back garbage and
    scan/join fail until it's loaded. In a fresh `linux-firmware` clone
    it's at `cypress/cyfmac43430-sdio.clm_blob`.
- **Bluetooth HCI** (`src/bluetooth.rs`, Pi 3 only): the on-board BCM43438
  Bluetooth controller, reached over a UART HCI attachment entirely
  separate from the Wi-Fi SDIO path despite the shared silicon. The
  controller's HCI UART is wired to the SoC's PL011 on GPIO30-33 — the
  *same* PL011 the GPIO14/15 console uses — so `Uart::init_bluetooth`
  routes it there (ALT3) with hardware RTS/CTS flow control, and the debug
  console moves to the mini UART (below). `BT_ON` is asserted through the
  VideoCore GPIO expander (best-effort, like Wi-Fi's `WL_ON`). The driver
  speaks the H4 (UART) HCI transport — a one-byte packet-type prefix, with
  commands answered by Command Complete / Command Status events — and,
  since the controller boots inert, `Bluetooth::load_firmware` runs the
  Broadcom "patchram" download: reset, `Download_Minidriver`, replay every
  HCI record in the `.hcd` blob (a run of Write-RAM chunks ending in a
  Launch-RAM), then reset to resync once the patched firmware restarts.
  `read_local_version`/`read_bd_addr` then answer with real data —
  end-to-end proof the HCI path round-trips (a genuine BCM43438 reports
  manufacturer `0x000f`, Broadcom, and a `B8:27:EB…` Raspberry Pi OUI
  address). `Bluetooth::set_baud` raises the link from the 115200 the
  controller boots at to a higher rate (the Broadcom `Update_Baudrate`
  vendor command, then the host PL011 reprogrammed to follow — flow
  control held across the switch); 3 Mbaud, the rate Raspberry Pi OS uses,
  is exact on the Pi's PL011. `examples/bt_probe.rs` reads the blob off the
  SD card, brings the controller up, bumps to 3 Mbaud, and prints the
  version and address at the new rate. Both GAP roles' controller-level
  halves sit on top: `Bluetooth::start_advertising`/`stop_advertising` (LE
  Set Advertising Parameters/Data/Enable) advertise a named peripheral —
  `examples/ble_advertise.rs` advertises non-connectable as `rpi-hal`,
  visible by name in a phone scanner (nRF Connect, LightBlue) — and
  `Bluetooth::start_scan`/`stop_scan`/`next_advertising_report` scan as a
  central, parsing LE Advertising Report events into an `AdvReport`
  (address, RSSI, name), which `examples/ble_scan.rs` prints as a live list
  of nearby devices. Everything above the controller (L2CAP, and SDP/RFCOMM
  or ATT/GATT/SMP — the connection and service layers) isn't implemented
  yet.

  The patchram download needs Broadcom's proprietary `.hcd` blob, which
  isn't redistributable here. For the original Pi 3B's BCM43438 the file is
  `BCM43430A1.hcd` (use this one, not the 3B+/Zero 2 W's `BCM4345C0.hcd`),
  from [RPi-Distro/bluez-firmware](https://github.com/RPi-Distro/bluez-firmware/blob/master/broadcom/BCM43430A1.hcd),
  or copied from `/lib/firmware/brcm/` on any Raspberry Pi OS install. Place
  it in a `bt` directory on the FAT boot partition under an 8.3 name —
  `bt/BT.HCD`.
- **Hardware RNG** (`src/rng.rs`): the dedicated true-RNG block (ring-
  oscillator entropy source, not software timing jitter) —
  `next_u32`/`next_u64`/`fill_bytes` blocking draws plus a non-blocking
  `try_next_u32`. Verified on real hardware by streaming words and byte
  fills over UART (`examples/rng_hello.rs`).
- **Watchdog timer** (`src/watchdog.rs`): the PM block's hardware
  countdown, resetting the board if not periodically re-armed —
  `start`/`feed`/`disable` (see `examples/watchdog_reset.rs`). Timeout
  tops out around 16 seconds (`MAX_TIMEOUT_MS`), a limit of the
  hardware's 20-bit countdown field.
- **Reboot / shutdown** (`src/power.rs`): `power::reboot()` resets the
  board via the PM block; `power::shutdown()` writes the firmware's "halt"
  boot-partition sentinel first, so the board stays off after the reset
  until physically power-cycled (the closest this hardware has to a
  power-off). See `examples/reboot.rs` and `examples/shutdown.rs`.
- **Multi-core** (`src/multicore.rs`, `multicore` feature): starts a
  plain `extern "C" fn() -> !` entry point running on secondary cores
  1-3, each given a caller-supplied `Stack`. Comes with a cross-core
  spinlock on top of `critical_section.rs`'s existing IRQ-mask
  implementation, needed the moment a second core is genuinely live,
  and a `CacheAligned<T>` to wrap any atomic two cores touch — an
  exclusive monitor reserves a whole 64-byte granule, so two atomics
  sharing a line make each other's compare-exchange fail. See
  `examples/multicore_blink.rs`.
- **DMA** (`src/dma.rs`): the DMA controller — `Dma::new().channel(n)`
  vends one of the 16 channels (0-6 full, 7-14 "lite"), and
  `Channel::memcpy` runs a blocking memory-to-memory copy through a
  control block, handling the VideoCore bus-address translation and
  cache maintenance. It cleans both source *and* destination before the
  transfer, not just the source: a dirty destination line left in cache
  (e.g. from the caller having just written the buffer) will otherwise
  write back over the engine's result in RAM, so the copy silently reads
  back stale data. See `examples/dma_memcpy.rs`. For peripherals there are
  also two non-blocking, DREQ-paced memory-to-peripheral transfers:
  `Channel::write_peripheral` (single buffer, optionally cyclic) and
  `Channel::stream_peripheral` (double-buffered ping-pong with a
  poll-driven `feed`), each returning an RAII guard that halts the channel
  on drop — used by the PWM audio path below.
- **PWM** (`src/pwm.rs`): the two-channel PWM controller sharing one
  clock. `Pwm::channel1`/`channel2` give plain duty-cycle outputs
  (`embedded_hal::pwm::SetDutyCycle`) on GPIO12/13/18/19 —
  `examples/pwm.rs`. On top of that, an analog-audio path drives the 3.5
  mm jack (GPIO40/45): `Pwm::audio`/`audio_mono` put the channels in FIFO
  (`USEF`) mode fed by the DMA controller off the PWM's DREQ, with a
  `pcm_to_duty` helper for signed-16-bit PCM. Verified on real hardware
  across `examples/pwm_audio.rs` (looping tone), `pwm_audio_stream.rs`
  (double-buffered swept siren), `pwm_audio_mono.rs` (single channel), and
  `pwm_audio_stereo.rs` (genuine stereo from embedded speech — distinct
  "left"/"right" per channel).
- **PCM / I2S** (`src/pcm.rs`): digital audio out to an external I2S DAC
  (e.g. a PCM5102 / UDA1334) — the digital counterpart to the analog PWM
  path above. `Pcm::i2s_out` brings the PCM peripheral up as an I2S clock
  master (standard Philips I2S, 16-bit stereo) driving `PCM_CLK`/`PCM_FS`/
  `PCM_DOUT` on GPIO18/19/21, clocked from `CM_PCM` the same way PWM uses
  `CM_PWM`. Like the PWM audio path it's FIFO-fed by the DMA controller off
  the PCM TX DREQ, with a `pcm_sample` helper for signed-16-bit PCM and an
  `I2sOut` handle exposing the FIFO bus address + DREQ — see
  `examples/i2s_dac_tone.rs` (double-buffered stereo tone, distinct left/
  right pitches) and `examples/i2s_wav_player.rs` (streams a stereo-16-bit
  WAV off the SD card to the DAC, joining this path to the `embedded-sdmmc`
  FAT read path). The peripheral isn't in the PAC, so like `src/unicam.rs`
  the driver pokes its registers directly (per the BCM2835 datasheet §8).
- **Camera** (`src/unicam.rs`, Pi 3): the Unicam1 CSI-2 receiver, capturing
  raw Bayer frames from a camera sensor straight into RAM via its own
  write-to-memory engine (no DMA controller or GPU firmware in the path).
  Brings up the sensor over BSC0 (`I2c<BSC0>`, GPIO44/45), powers the
  camera analog D-PHY (the `PM_CAM1` LDO in the power manager — the piece
  the closed firmware normally handles, and what makes high-speed reception
  work), and configures the receiver for 2-lane RAW10. `Unicam::arm` +
  repeated `Unicam::wait_frame` capture successive frames continuously. The
  sensor half — a reference OV5647 driver — is `src/ov5647.rs` (a
  third-party device, not a SoC peripheral). The full pipeline runs end to
  end on an OV5647 (Camera v1): `examples/
  camera_probe.rs` identifies the sensor, `camera_capture.rs` grabs one
  frame headless (a self-test), and `camera_display.rs` shows a live
  auto-exposed preview on the mailbox framebuffer (with a 2×2-binning
  demosaic + gamma). Image-quality polish
  (white balance, a better demosaic, full resolution) is left to do — see
  [issue #27](https://github.com/joeferner/rpi-hal/issues/27).
- **Hardware H.264 decode** (`src/video_decode.rs`, on `src/mmal.rs` and
  `src/vchiq.rs`, Pi 3, behind the `mmal` feature): decoding through the
  VideoCore's `ril.video_decode` component. Feed it a raw H.264 Annex B
  byte stream in arbitrary chunks and whole I420 frames come back — the
  firmware owns the bitstream front end, so there is no NAL parsing or
  reference-picture management to do on the ARM side, and the mid-stream
  format change the decoder announces once it has parsed the stream's
  geometry is handled internally. `examples/h264_decode.rs` plays a file
  off the SD card on the display.

  Getting there meant the two layers underneath, which are the reusable
  part: **VCHIQ** (`src/vchiq.rs`), the firmware's shared-memory message
  transport — slot ring, service open/close, messages, and page-list bulk
  DMA — and an **MMAL** client on top of it (`src/mmal.rs`) for
  components, ports, parameters and buffer exchange. Every
  firmware-mediated subsystem this crate doesn't reach today (the camera
  ISP path, the encoders, audio) is an MMAL component, so it is the same
  road for all of them. Neither is interrupt-driven: the application polls
  (`Vchiq::poll`/`Mmal::poll`), which is what keeps the whole subsystem
  free of the interrupt plumbing a doorbell would need. Both carry
  `Stats` counters, because a shared-memory exchange that stalls reports
  nothing about itself and sent-versus-returned is what makes one
  diagnosable.

  The one thing this changes outside itself: the shared region has to be
  non-cacheable, because both sides write different fields of the same
  cache line and no maintenance sequence survives that. `src/mmu.rs` grew
  `mmu::set_uncached` for it — see "Virtual memory" below.

  Pi 3 only in practice. The transport itself is chip-agnostic, but the
  H.264 block on a Pi 4 is firmware-mediated in the same way while its
  HEVC decoder is a real ARM-side register block with no driver here — see
  [issue #28](https://github.com/joeferner/rpi-hal/issues/28). Getting a
  decoded frame on *screen* is not something the driver does: output is
  planar YUV and the mailbox framebuffer takes RGB, so something has to
  convert, and where that belongs is an application decision — the
  example does it on the ARM, a pass that costs more per frame than the
  decode itself.
- **HDMI audio** (`src/audio_render.rs`, on the same `mmal`/`vchiq` stack
  and behind the same feature): audio out through the VideoCore's
  `ril.audio_render` component. HDMI carries audio inside the video signal,
  which is a link this side never programs — the display's timing and
  audio capabilities are negotiated by the firmware — so like the decoder
  above this is messages rather than a register-level driver.
  `AudioRenderer::new` names the destination (`Destination::Hdmi`, or
  `Destination::Local` for the 3.5 mm jack the PWM path already reaches by
  driving the hardware directly), and `feed` hands it interleaved
  signed-16-bit samples. Nothing has to keep time: the renderer takes
  samples no faster than it plays them, so feeding until `feed` returns
  zero is paced by the audio clock — after the third of a second it queues
  up front, which is also why the last buffer coming back is not the last
  sample being played.
  `examples/hdmi_audio.rs` plays a stereo tone and prints the rate the
  hardware consumed it at.

Implemented against the same trait surface as their verified siblings
above, and now hardware-verified too, but called out separately because
each carries a hardware caveat worth stating up front (the mini UART's
core-clock pin and the aux SPI's CPHA=1 limitation, both below). The mini
UART's console is legible once the core clock is pinned; the aux SPI's
transmit path and mode-0/2 mapping are logic-analyzer-verified and its
full-duplex/MISO path is verified against an external SPI slave. The
remaining (optional) automation is
[issue #12](https://github.com/joeferner/rpi-hal/issues/12):

- **Mini-UART** (`src/mini_uart.rs`): a second serial console on the AUX
  block's UART1, GPIO14/15 — blocking read/write, `set_baud`, optional RX
  interrupt (`enable_rx_irq`, routed via `Lic::enable_aux_irq`), and
  `embedded_io::Read`/`Write`, mirroring the PL011 `uart::Uart` API. This
  is what a debug console moves to once PL011 is committed to Bluetooth
  (the two share GPIO14/15). See `examples/mini_uart_hello.rs`. Unlike
  PL011's fixed reference clock, the mini UART is clocked from the
  dynamically-scaled VPU/core clock, so a legible console requires pinning
  it with **`core_freq=250` in `config.txt`** (`enable_uart=1` alone is
  not sufficient when PL011 is the primary UART).
- **Auxiliary SPI** (`src/aux_spi.rs`): the AUX block's SPI1/SPI2
  "Universal SPI Master" controllers as `embedded_hal::spi::SpiBus`,
  generic over the instance (`AuxSpi<SPI1>`/`AuxSpi<SPI2>`) with the same
  `ChipSelect` shape as `spi::Spi` — hardware CE0/CE1/CE2 or
  externally-managed `None`. `AuxSpi<SPI1>` drives GPIO16-21 on the 40-pin
  header; `AuxSpi<SPI2>` (GPIO40-45) isn't broken out on a Pi 2/3 and is
  there for custom boards. **Modes 0 and 2 (CPHA=0) only** — the aux SPI
  can't generate CPHA=1, so modes 1 and 3 come out one bit shifted on MOSI
  (a Broadcom hardware limitation, confirmed on a logic analyzer). The
  transmit path and mode-0/2 CPOL/CPHA mapping were verified against a
  logic-analyzer capture (`examples/aux_spi_mode_probe.rs`), and the
  full-duplex/MISO path against an external SPI slave — `bench-link`'s
  SPI-slave mode — reading back its armed reply while it confirmed the
  bytes sent (`examples/aux_spi_slave_check.rs`). `examples/
  aux_spi_loopback.rs` is a no-fixture MOSI→MISO jumper self-test.

Everything still outstanding is in the
[issue tracker](https://github.com/joeferner/rpi-hal/issues); hardware this
crate doesn't touch at all yet is labelled
[`area: peripherals`](https://github.com/joeferner/rpi-hal/labels/area%3A%20peripherals),
so "what's supported" and "what's missing" aren't just implicitly the
complement of each other.

The `bench-link` crate (a separate, standalone project) is a
UART-controlled command-and-control tool used as a real
hardware-in-the-loop test fixture for this crate's SPI0 driver — see
[`tests/hil/`](tests/hil/) for the test runner that drives it.

See [`docs/getting-started.md`](docs/getting-started.md) for a working
first build: boots from an SD card and blinks an external LED.

## Scope

Turn raw register access into ergonomic APIs, e.g. `led.set_high()`,
`uart.write(...)` — the "Status" section above is the driver-by-driver
state, and the [issue tracker](https://github.com/joeferner/rpi-hal/issues)
has what's left.

## Features

- **`rt`** (default): provides the standard (non-relocating) `_start`,
  exception vector table installation, and banked IRQ-mode stack setup
  — everything a normal firmware image needs to boot straight into
  `kmain`. Every example in `examples/` depends on this.
- **`mmu`** (default, independent of `rt` at the Cargo level): identity-
  mapped MMU bring-up (`src/mmu.rs`) — see "Virtual memory" below for
  exactly what it configures and why. `rt`'s boot sequence calls it
  automatically, once per core, right after that core's vector table
  is installed.
- **`multicore`** (off by default, implies `mmu`): starting code on
  secondary cores (`src/multicore.rs`) and the cross-core spinlock
  that becomes necessary once one actually is (`src/
  critical_section.rs`). Implied `mmu` because cross-core signaling
  relies on `core::sync::atomic`'s compare-exchange, which needs the
  MMU/cacheable-RAM setup `mmu` provides to behave correctly on this
  core — without it, `ldrex`/`strex` are architecturally
  UNPREDICTABLE.
- **`async`** (off by default): adds async counterparts to the blocking
  trait implementations, from `embedded-hal-async` and
  `embedded-io-async` — one capability spanning the two trait crates,
  mirroring the blocking split between `embedded-hal` and `embedded-io`.
  So far:
  - `embedded_hal_async::digital::Wait` on an input `Pin` — awaiting a
    level or an edge instead of busy-polling GPLEV.
  - `embedded_io_async::{Read, Write}` on `Uart` — a full transmit FIFO
    yields the core instead of spinning ~87us per byte at 115200 baud,
    which is what stops one task's logging stalling every other task
    under an executor.
  - `usb::dwc2::asynch` — interrupt-driven twins of every
    `usb::dwc2::Channel` transfer primitive
    (`control_*_async`, `interrupt_in_async`, `bulk_*_async`), plus
    `Dwc2Host::wait_for_port_change`. The blocking versions busy-wait on
    `HCINT` for a transfer, on `HFNUM` for a periodic split's microframe,
    and on the clock between complete-split polls; each of those becomes
    an await. Notably this needs no time crate — a transfer completing is
    a channel-halt interrupt and every split delay is a whole number of
    microframes, so start-of-frame is the clock (which is also what
    `Channel::wait_microframes` exposes for pacing a periodic endpoint).
    An async transfer therefore has no timeout of its own; drop the
    future to impose one, which aborts the channel. See
    `examples/usb_irq.rs`.
  - `usb::control::{vendor_in_async, vendor_out_async}` — the vendor
    register access a device driver is built on, over those primitives.
  - `usb::lan9514` — async twins of the LAN9514's frame and register
    methods (`send_frame_async`, `receive_frame_async`, `start_async`,
    `is_link_up_async`), plus `Lan9514::split`, which borrows the two
    bulk endpoints apart so a receive can stay parked on one host
    channel while transmits go out on another. `receive_frame_async` is
    the one that is not merely the blocking method with awaits in it:
    the blocking twin has to ask `RX_FIFO_INF` whether a frame is
    waiting, because a bulk IN against an empty FIFO NAKs and the DWC2
    retries it in hardware without halting — which would burn the whole
    transfer timeout on every idle poll. Parked on an interrupt that
    same behaviour is free, so the receive becomes event-driven rather
    than polled. The `rpi-hal-embassy` crate's `embassy-net` adapter is
    built on this.

  Each is driven by the same interrupt the blocking API already exposes,
  with the application routing it to `gpio::on_irq`, `uart::on_irq` or
  `usb::dwc2::on_irq` from its own `__irq_handler` — the same dispatch
  contract every other source here uses. Off by default because each
  implementation carries a waker slot a purely blocking consumer
  shouldn't pay for. The futures are plain `poll`/`Waker` code with no
  executor dependency, so any executor drives them — see the
  `rpi-hal-embassy` crate for one, and `examples/usb_irq.rs` for a
  hand-rolled `block_on` that needs none.
- **`embedded-sdmmc`** (off by default): adds `sd::SdCard`, an adapter
  implementing `embedded-sdmmc`'s `BlockDevice` trait over the SD
  driver, so a FAT filesystem can be layered on the card (see
  `examples/sd_fat_read.rs`). Both `read` and `write` are wired to the
  driver's polled multi-block paths, so a run of consecutive blocks costs
  one command; the `TimeSource` a filesystem also needs is left to the
  caller, since a real clock is application policy.
- **`resident-fat`** (off by default): adds `sd::SdBlockDevice`, an
  adapter implementing `resident-fat`'s `BlockDevice` trait over the same
  driver (see `examples/sd_resident_fat_read.rs`). Alongside the
  `embedded-sdmmc` adapter rather than instead of it — the two traits
  differ in their unit of transfer, and which suits depends on the
  filesystem above. `resident-fat` hands a device a plain `&[u8]`
  spanning a whole run of consecutive blocks, which is already the shape
  the driver's multi-block path wants, so the adapter splits it and hands
  the pieces straight over: no staging buffer, no copy, and a transfer
  limit of the controller's real 65535 blocks. Reaching `resident-fat`
  through its own `embedded-sdmmc` bridge and `sd::SdCard` also works and
  is the right route for a consumer already invested in that trait, but
  it pays for the block newtype in both memory and copying. Note that
  `resident-fat` uses `alloc`, so a binary enabling this feature must
  register a `#[global_allocator]` — this crate neither has nor needs
  one, and cannot supply it (see `examples/heap_alloc.rs`).
- **`smoltcp`** (off by default): adds `usb::lan9514::Lan9514Phy`, an
  adapter implementing `smoltcp`'s `phy::Device` trait over the LAN9514
  Ethernet driver, so a TCP/IP stack can run over the on-board Ethernet
  (see `examples/usb_ethernet_smoltcp.rs`). Pulls `smoltcp` in with only
  `medium-ethernet`; the protocol/socket features a stack needs are the
  consumer's to add, since the stack itself is application policy.
- **`vchiq`** (off by default, implies `mmu`): adds `vchiq::Vchiq`, the
  shared-memory message transport to the VideoCore firmware — the channel
  every firmware-mediated subsystem beyond `mailbox`'s simple property
  queries goes through. Implies `mmu` because its shared region has to be
  remapped non-cacheable (`mmu::set_uncached`), which needs this crate's
  translation tables to remap. Its own feature because it is a large
  subsystem, and 2MB of `.bss` for the shared region, that a consumer who
  never talks to the firmware's services shouldn't pay for.
- **`mmal`** (off by default, implies `vchiq`): adds `mmal::Mmal`, a
  client for the firmware's multimedia framework (components, ports,
  parameters, buffer exchange), and the two drivers built on it —
  `video_decode::VideoDecoder`, the hardware H.264 decoder, and
  `audio_render::AudioRenderer`, audio out over HDMI or the analog jack.
  All three are described under "Status" above.
- **`v3d`** (off by default): adds `v3d::V3d` and the control-list
  builders around it (`v3d::bcl`, `v3d::rcl`, `v3d::shader_record`,
  `v3d::texture`, `v3d::uniforms`) — the V3D 3D pipeline, VideoCore
  IV's QPU shader cores and the tile-based binning/render hardware.
  Enough to submit a binning pass and a render pass and get a
  depth-tested, textured 3D scene on screen; `examples/gpu_cube.rs`
  draws a tumbling textured cube. The QPU machine code it runs is
  extracted from Mesa's compiler rather than assembled here, so the
  shaders themselves are fixed — see
  [issue #21](https://github.com/joeferner/rpi-hal/issues/21) for what that
  and the other limits imply. Pi 3 (BCM2836/BCM2837) only; the Pi 4 ships a
  different GPU generation under the same "V3D" name that this feature
  does not cover. No dependency to gate — the register block isn't in
  either PAC's SVD, so this pokes its known physical address directly
  like `dma.rs`/`rng.rs` — kept as its own feature anyway since it's a
  large subsystem a consumer who doesn't need 3D shouldn't have to
  compile.
- **`bcm2837`**/**`bcm2711`** (chip selection — neither on by default,
  pick exactly one): each pulls in that chip's PAC (`bcm2837-lpa`/
  `bcm2711-lpa`) as `pac` (see "Relationship to other crates" below),
  and, for `bcm2711`, also switches `src/mmu.rs`'s `PERIPHERAL_BASE`/
  `LOCAL_PERIPHERAL_BASE` — and anything computed from them (`src/
  dma.rs`'s `DMA_BASE`, `src/watchdog.rs`'s `PM_BASE`) — to the
  BCM2711's relocated addresses. Compile-time, not runtime model
  detection, matching every other board/config choice in this crate.
  If both are enabled at once, `bcm2711` wins `pac`'s re-export;
  `bcm2837-lpa` still compiles as an unused dependency, harmless but
  avoidable with `default-features = false` plus whichever of `rt`/
  `mmu` are still wanted (the same opt-out shape `rt` documents above).

  `bcm2711` is **preliminary** (see
  [issue #29](https://github.com/joeferner/rpi-hal/issues/29)): most
  drivers are untested against real hardware, and `lic`
  (the legacy interrupt controller) doesn't compile under it at all —
  BCM2711 wires a different peripheral set through that block, so the
  PAC's bit-level field names genuinely diverge, not just the base
  address — meaning nothing IRQ-driven (`irq`, the async GPIO/UART
  traits under the `async` feature, `rpi-hal-embassy`'s time driver) is
  available under `bcm2711` until GIC-400 support lands.
- Consumers that need their own boot sequence instead — e.g.
  `rpi-loader`'s self-relocating loader, which must control exactly
  how and where its own code executes during relocation — depend on
  this crate with `default-features = false` to supply their own
  `_start` and avoid a duplicate symbol. `bench-link`'s `pi` feature is
  the contrasting case: it wants a normal firmware image (same as any
  plain example), so it depends on this crate with default features
  left on, getting `rt`'s boot sequence for free rather than writing
  its own.

### Supplying your own MMU table

`rt`'s boot sequence calls a symbol named `rpi_hal_mmu_init`
unconditionally — but *which* definition of that symbol ends up in
your binary depends on the `mmu` feature:

- **`mmu` on** (default): `src/mmu.rs`'s real implementation — a
  *strong* `#[no_mangle]` symbol — builds the identity-mapped table
  described in "Virtual memory" below and enables the MMU.
- **`mmu` off**: a *weak* no-op fallback (`src/mmu_fallback.s`) takes
  its place instead, leaving the MMU untouched (this crate's
  long-standing MMU-off default, from before `mmu.rs` existed).

To use a different memory map instead of this crate's, disable `mmu`
(`default-features = false`, or explicitly list `features = ["rt"]`
without `mmu`) and define your own:

```rust
#[no_mangle]
pub unsafe extern "C" fn rpi_hal_mmu_init() {
    // Build and install your own page table, then enable the MMU.
    // Called once, early in boot, before any code relies on the
    // MMU-off memory ordering guarantees changing underneath it.
}
```

Your definition is *strong*, so it wins over the weak fallback with no
duplicate-symbol error — confirmed for real, not just asserted: a
throwaway example providing its own `rpi_hal_mmu_init` while `mmu` was
off built clean, and disassembly showed the override's own code at
that symbol, not the fallback's `bx lr`. This specifically depends on
your override living in a genuinely separate crate compilation from
`rpi-hal`'s own — weak/strong symbol resolution is a property of the
*final link step* across separate object files, not something that
works within a single crate's own compilation (which is exactly why
`mmu.rs` and `mmu_fallback.s` are mutually exclusive via `cfg` in this
crate, never both compiled in at once: two definitions of one symbol
within the same compilation is a hard assembler error regardless of
weak/strong, a real error this crate's own development hit before
splitting them apart).

If you disable `mmu` and don't define your own `rpi_hal_mmu_init`
either, the weak fallback runs and the MMU simply stays off.

## Relationship to other crates

Depends on [`bcm2837-lpa`](https://crates.io/crates/bcm2837-lpa) and
[`bcm2711-lpa`](https://crates.io/crates/bcm2711-lpa) for register
access, each optional and behind the `bcm2837`/`bcm2711` feature it's
named after (see "Features" above) — whichever is selected is
re-exported as `pac`.

[Embassy](https://embassy.dev) support — an `embassy-time` driver and an
executor, so an application can `spawn` tasks and `await` deadlines — is
in the separate `rpi-hal-embassy` crate, not behind a feature here. An
`embassy-time` driver is installed by *linkage*: it defines
`#[unsafe(no_mangle)]` symbols that `embassy-time` resolves against, and
a program links only if exactly one driver exists in its crate graph.
Behind a feature on this crate, Cargo's feature unification would let any
dependency force that driver onto the whole program and conflict with an
application supplying its own — an opt-in that could not be opted out of.
The same split as `esp-hal` / `esp-hal-embassy`. The drivers here stay
blocking; the async trait implementations they will grow
(`embedded-hal-async`) belong in this crate, since they need each
driver's interrupt handler and private state.

`bcm2837-lpa` is a Peripheral Access Crate (PAC) for the BCM2837 SoC
(Cortex-A53, used in Pi 2 rev 1.2 and Pi 3). Broadcom never published
an official SVD for this chip family, so the crate is generated by
`svd2rust` from a community-maintained SVD instead. It's maintained as
part of a small family of sibling PACs (`bcm2835-lpa`, `bcm2837-lpa`,
`bcm2711-lpa`) generated from the same SVD lineage, one per Broadcom
SoC generation. "LPA" stands for Low-level Peripheral Access: it gives
typed register access at fixed physical addresses, with no HAL-level
ergonomics on top — that's what this crate exists to add.

`bcm2711-lpa` is a peer in that same family, not an extension of
`bcm2837-lpa` — confirmed by diffing the two crates' generated source,
not assumed: every peripheral this crate drives (GPIO, UART0, the
System Timer, EMMC, the VideoCore mailbox, AUX/UART1/SPI1/SPI2) has an
identical register layout between them, aside from a few GPIO
alt-function names BCM2711 reassigned (`pwm.rs` already accounts for
the one rename this crate's drivers actually touch) and BCM2711's wider
GPIO count. The legacy interrupt controller (`LIC`) is the exception —
its bit-level fields diverge throughout, which is why `src/lic.rs`
isn't built under `bcm2711` at all (see "Features" above).

## Virtual memory

Identity-mapped only (`src/mmu.rs`) — every virtual address equals its
physical address, matching `bcm2837-lpa`'s fixed physical addresses
per peripheral. This exists purely to give RAM real Normal-memory
attributes for `core::sync::atomic`, not to relocate anything. If a
higher-half kernel or non-identity peripheral mapping is ever needed,
either patch PAC generation to parameterize the base address, or wrap
register access behind an indirection layer in this crate.

The one thing that changes the map after boot is `mmu::set_uncached`,
which remaps a region of RAM as Normal **Non-cacheable** — one 1MB
section (AArch32) or 2MB block (AArch64) at a time, since that is the
smallest thing a translation table entry covers. It exists for memory the
VideoCore writes *concurrently* with this core, where the usual
clean-before-give/invalidate-after-take maintenance every other bus-master
driver here uses cannot work: both sides write different fields of the
same cache line, so cleaning it publishes a stale copy of the peer's field
and invalidating it discards this core's own. `vchiq` is the caller; it is
the same trade Linux makes by allocating the same region with
`dma_alloc_coherent`.

## The stack

The linker script reserves the stack as a named region and the boot code
points `sp` at it, so its size is a number you can read and change:

| Symbol | Default | What it is |
| --- | --- | --- |
| `__stack_size` | 1 MiB | The main stack (SVC mode on AArch32, `SP_EL1` on AArch64). |
| `__stack_slack` | 2 MiB | Reserved margin *below* the stack. An overflow walks into this before it can reach `.data`/`.rodata`/`.text`. |
| `__irq_stack_size` | 64 KiB | AArch32 IRQ mode's banked stack. |
| `__abt_stack_size` / `__und_stack_size` / `__fiq_stack_size` | 32 KiB each | AArch32 abort/undefined/FIQ modes, so a fault handler can be ordinary Rust. |

Change any of them without editing the script, from the consumer's own
`.cargo/config.toml`:

```toml
rustflags = ["-C", "link-arg=-Trpi-link.x",
             "-C", "link-arg=--defsym=__stack_size=0x400000"]
```

None of this costs image bytes (the region is `NOLOAD`), and on a board
with at least 1 GiB — every Pi this crate supports — a few MiB of
address space is noise.

It used to work the other way round: `sp` started at the load address
and the stack was whatever sat below it, which meant 32 KiB on a 32-bit
kernel at `0x8000` and 512 KiB on a 64-bit one at `0x80000` — a number
nobody chose, differing 16-fold between the two architectures, and
documented nowhere. A program that outgrew it took a data abort and
parked silently.

Which is the other half of this: **an overflow should be loud**.
`__unhandled_exception` is weak (like `__irq_handler`), so an
application can define its own and report the fault rather than go
quiet:

```rust
#[no_mangle]
pub extern "C" fn __unhandled_exception() {
    // AArch64: ESR_EL1 (class) / FAR_EL1 (address) / ELR_EL1 (instruction).
    // AArch32: CPSR mode says which exception; DFAR/DFSR or IFAR/IFSR say
    // where and why.
}
```

On AArch32 that handler runs in the exception's own mode on its own
banked `sp`, which is why the boot code initializes all of them. On
AArch64 it runs on the same `SP_EL1` the faulting code was using — so a
handler that has to survive a stack overflow specifically should switch
`sp` before doing real work.

And to answer "how close am I?" from inside a running program,
`stack::headroom()` reports the bytes left below `sp` (with
`stack::bottom`/`top`/`size`/`pointer` alongside it). One line at
startup is usually enough:

```rust
writeln!(uart, "sp {:#x}, {} KiB free", stack::pointer(),
         stack::headroom().unwrap_or(0) / 1024)?;
```

## Dynamic memory allocation (`alloc`)

This crate is `#![no_std]` and defines **no** global allocator, by
design: a program may only have one `#[global_allocator]`, and that
choice belongs to the final binary, not a library. So `rpi-hal` never
takes it away from you — but nothing here stops you from adding one and
using the `alloc` crate (`Box`, `Vec`, `String`, `BTreeMap`, ...).

`examples/heap_alloc.rs` is a complete, runnable example. The pattern:

1. Pick a heap allocator crate. The example uses
   [`embedded-alloc`](https://crates.io/crates/embedded-alloc), which
   locks through `critical-section` — and `rpi-hal` already provides a
   `critical-section` implementation under its default `rt` feature, so
   it drops in with no extra wiring.
2. Register it as the global allocator and give it a region of RAM. The
   example uses everything from the end of `.bss` (the `__bss_end`
   linker symbol) up to the top of the ARM/VideoCore memory split, which
   the VideoCore firmware reports via the mailbox — so the same binary
   sizes its heap correctly regardless of the board's RAM size or
   `gpu_mem` setting. The whole ARM region below the peripheral base is
   identity-mapped as cacheable Normal memory by the `mmu` feature (see
   "Virtual memory" above), so it's all safe to hand out. The stack is
   reserved below `.bss` (see "The stack" above), so it sits outside this
   region and the two never collide.
3. Add `extern crate alloc;` to your binary. Nothing else is needed —
   `rustup`'s precompiled target libraries include `alloc`, so a stable
   build links it as soon as a `#[global_allocator]` exists. (This crate's
   own `.cargo/config.toml` builds it from source instead, via
   `build-std = ["core", "alloc"]`; either route works.)

```rust
extern crate alloc;

use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

extern "C" {
    static __bss_end: u8; // from linker.ld
}

// Once, before the first allocation:
let start = &raw const __bss_end as usize;
let end = { /* base + size from Mailbox::arm_memory() */ };
unsafe { HEAP.init(start, end - start) };
```

## Disabling the boot splash

On power-up the VideoCore firmware paints a coloured square gradient
(its boot splash / test pattern) to the display before any ARM code
runs. It's drawn by the firmware, not by this crate, so it can't be
turned off in code — it lives in the firmware config on the SD card.
Until your application sets up its own framebuffer (via the mailbox)
and overwrites the screen, that image stays visible.

To suppress it, add this line to `config.txt` on the boot (FAT)
partition of the SD card:

```
disable_splash=1
```

This only stops the firmware from painting the splash; the
mailbox/framebuffer path is unaffected and your own scanout still works
exactly as before.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or
  <http://opensource.org/licenses/MIT>)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual licensed as above, without any additional terms or
conditions.

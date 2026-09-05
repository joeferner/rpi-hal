# Compatibility

Which drivers work on which Raspberry Pi.

A **blank cell means unknown** — not broken, and not absent. A cell gets a
mark only once that combination has been exercised on that board and the
result recorded, including cells that look obvious from the datasheet,
because "the hardware has it" and "this crate drives it" are different
claims.

The one exception is **—**, for hardware that genuinely is not on the
board. That is a fact about silicon rather than a test result, so it is
filled in ahead of any bench work: it marks cells that will never be
filled, which is different information from cells nobody has got to yet.

The matrix deliberately lists hardware this crate does **not** drive at
all, and features that exist only on the newer chips. A missing row hides a
gap; a blank row shows one.

Legend for filling cells in:

| Mark | Meaning |
| --- | --- |
| ✅ | exercised on this board, works |
| ⚠️ | works with a caveat — footnote required |
| ❌ | exercised on this board, does not work |
| — | the hardware genuinely isn't there |
| 🚧 | implemented and compiles for this chip, never run on it |
| ⬜ | not implemented for this chip |

## Boards

| Column | Boards | SoC | Core | Execution states |
| --- | --- | --- | --- | --- |
| **Pi 1 / Zero** | Pi 1 A/B/A+/B+, Zero, Zero W, CM1 | BCM2835 | ARM1176JZF-S, 1 core | ARMv6, 32-bit only |
| **Pi 2** | Pi 2 B rev 1.1 | BCM2836 | Cortex-A7 ×4 | AArch32 only |
| **Pi 3** | Pi 3 B, 3 B+, 3 A+, Pi 2 B rev 1.2, CM3 | BCM2837 / BCM2837B0 | Cortex-A53 ×4 | AArch32, AArch64 |
| **Zero 2 W** | Zero 2 W | BCM2710A1 | Cortex-A53 ×4 | AArch32, AArch64 |
| **Pi 4 / 400** | Pi 4 B, Pi 400, CM4 | BCM2711 | Cortex-A72 ×4 | AArch32, AArch64 |
| **Pi 5** | Pi 5, CM5 | BCM2712 + RP1 | Cortex-A76 ×4 | AArch64 only |

Two columns share silicon but not boards. **Zero 2 W** is a BCM2710A1, the
same die as BCM2837, so it matches Pi 3 for anything on the SoC — but it
has no Ethernet, no 3.5 mm jack and a single OTG USB port, so board-level
rows diverge. **Pi 2 B rev 1.2** shipped with BCM2837 and belongs in the Pi
3 column despite the name; only rev 1.1 is a BCM2836.

Compute Modules share their Pi counterpart's SoC but route peripherals to
carrier-board pins, so SoC rows carry over and connector-dependent rows do
not.

**Pi 1 / Zero is the loosest column**, and board-level rows in it should be
read with care: it spans a Pi 1 B+ with a LAN9514 and a 3.5 mm jack but no
radio, and a Zero W with a BCM43438 for Wi-Fi and Bluetooth but no Ethernet
and no jack. Where those boards disagree the cell is left blank rather than
given a single answer. Splitting the column is worthwhile if ARMv6 is ever
targeted; until then nothing in it can be tested anyway.

**Pi 5 is 64-bit only.** The Cortex-A76 supports AArch32 at EL0 only, so
there is no 32-bit bare-metal path — that is not a gap to be filled in,
it is architecture.

## Runtime

| Feature | Pi 1 / Zero | Pi 2 | Pi 3 | Zero 2 W | Pi 4 / 400 | Pi 5 |
| --- | :-: | :-: | :-: | :-: | :-: | :-: |
| Boot / `rt`, AArch32 | | | ✅ | | | — |
| Boot / `rt`, AArch64 | — | — | ✅ | | | |
| Relocating `_start` | | | | | | |
| MMU, AArch32 | | | ✅ | | | — |
| MMU, AArch64 | — | — | ✅ | | | |
| 40-bit physical addressing (LPAE, >4 GB) | — | — | | | | |
| Secondary-core bring-up (`multicore`) | — | | | | | |
| FPU / NEON bring-up | | | | | | |
| ARMv8 crypto extensions | — | — | — | — | — | |
| Cache maintenance | | | ✅[^cache] | | | |
| `critical-section` | | | | | | |
| `alloc` / heap | | | ✅ | | | |
| PMU / performance counters | | | | | | |
| Reboot | | | ✅[^reboot] | | | |
| Shutdown / power-off | | | | | | |

## Interrupts and timers

| Feature | Pi 1 / Zero | Pi 2 | Pi 3 | Zero 2 W | Pi 4 / 400 | Pi 5 |
| --- | :-: | :-: | :-: | :-: | :-: | :-: |
| CPU-level IRQ enable/disable | | | | | | |
| Legacy interrupt controller | | | | | — | — |
| ARM-local interrupt controller | — | | | | | — |
| GIC-400 | — | — | — | — | | — |
| RP1 interrupt routing | — | — | — | — | — | |
| System Timer | | | ✅ | | | |
| ARM generic timer | — | | ✅ | | | |
| ARM local timer | | | | | | |
| Watchdog | | | | | | |

## GPIO and buses

| Feature | Pi 1 / Zero | Pi 2 | Pi 3 | Zero 2 W | Pi 4 / 400 | Pi 5 |
| --- | :-: | :-: | :-: | :-: | :-: | :-: |
| GPIO | | | | | | |
| GPIO interrupts | | | | | | |
| GPIO expander (VideoCore) | | | | | | |
| UART0 (PL011) | | | ✅ | | | |
| UART2–UART5 (PL011) | — | — | — | — | | |
| Mini-UART (AUX UART1) | | | | | | |
| Dedicated UART debug connector | — | — | — | — | — | |
| SPI0 | | | | | | |
| Aux SPI1 | | | | | | |
| Aux SPI2 | | | | | | |
| SPI3–SPI6 | — | — | — | — | | |
| I2C — BSC1 (GPIO2/3) | | | | | | |
| I2C — BSC0 (GPIO44/45) | | | | | | |
| I2C — BSC3–BSC6 | — | — | — | — | | |
| PWM | | | | | | |
| PCM / I2S | | | | | | |
| DMA | | | ✅ | | | |
| DMA4 / 40-bit channels | — | — | — | — | | |
| SMI (secondary memory interface) | | | | | | |
| RP1 PIO | — | — | — | — | — | |

## VideoCore and firmware services

| Feature | Pi 1 / Zero | Pi 2 | Pi 3 | Zero 2 W | Pi 4 / 400 | Pi 5 |
| --- | :-: | :-: | :-: | :-: | :-: | :-: |
| Mailbox property interface | | | ✅ | | | |
| Board / firmware info | | | ✅ | | | |
| Clock rate get/set | | | ⚠️[^clock] | | | |
| ARM/VC memory split | | | ✅ | | | |
| Power-domain control | | | | | | |
| Temperature sensor | | | ✅ | | | |
| OTP read | | | | | | |
| Framebuffer | | | | | | |
| Paged framebuffer / `set_virtual_offset` | | | | | | |
| `wait_for_vsync` | | | | | | |
| Overscan control | | | | | | |
| EDID read | | | | | | |
| Boot splash suppression | | | | | | |
| Second HDMI output | — | — | — | — | | |
| HDMI CEC | | | | | | |
| DSI display output | | | | | | |
| DPI (parallel display) | | | | | | |
| VCHIQ | | | | | | |
| MMAL | | | | | | |
| H.264 decode | | | | | | — |
| HEVC / H.265 decode | — | — | — | — | | |
| Hardware video encode | | | | | | — |
| JPEG encode / decode | | | | | | |
| ISP | | | | | | |
| HDMI audio (`ril.audio_render`) | | | | | | |
| V3D (VC4) | | | | | — | — |
| V3D (VC6) | — | — | — | — | | — |
| VideoCore VII GPU | — | — | — | — | — | |

## Storage

| Feature | Pi 1 / Zero | Pi 2 | Pi 3 | Zero 2 W | Pi 4 / 400 | Pi 5 |
| --- | :-: | :-: | :-: | :-: | :-: | :-: |
| SD via EMMC (Arasan) | | | | | | |
| SD via EMMC2 | — | — | — | — | | — |
| SD via RP1 | — | — | — | — | — | |
| Single-block transfer | | | | | | |
| Multi-block transfer | | | | | | |
| DMA-backed transfer | | | | | | |
| 4-bit bus | | | | | | |
| High-speed modes (SDR104) | | | | | | |
| Card-detect | | | | | | |
| `embedded-sdmmc` adapter | | | | | | |
| SDIO host | | | | | | |

## Radios

| Feature | Pi 1 / Zero | Pi 2 | Pi 3 | Zero 2 W | Pi 4 / 400 | Pi 5 |
| --- | :-: | :-: | :-: | :-: | :-: | :-: |
| Wi-Fi firmware load | | — | | | | |
| Wi-Fi scan | | — | | | | |
| Wi-Fi WPA2 join | | — | | | | |
| Wi-Fi data path (BDC) | | — | | | | |
| Bluetooth HCI transport |  | — | | | | |
| Bluetooth patchram (`.hcd`) |  | — | | | | |
| Bluetooth classic inquiry |  | — | | | | |
| BLE advertising |  | — | | | | |
| BLE scanning |  | — | | | | |
| BLE peripheral / GATT server |  | — | | | | |
| BLE central / GATT client |  | — | | | | |
| BLE pairing and bonding (SMP) |  | — | | | | |

## USB and networking

| Feature | Pi 1 / Zero | Pi 2 | Pi 3 | Zero 2 W | Pi 4 / 400 | Pi 5 |
| --- | :-: | :-: | :-: | :-: | :-: | :-: |
| USB host (DWC2) | | | | | | |
| USB host (xHCI) | — | — | — | — | | |
| USB device / gadget (DWC2 OTG) | | | | | | |
| Hub enumeration | | | | | | |
| HID class | | | | | | |
| Ethernet (LAN9514) |  | | | — | — | — |
| Ethernet (GENET) | — | — | — | — | | — |
| Ethernet (RP1) | — | — | — | — | — | |
| `smoltcp` phy adapter | | | | | | |
| PCIe root complex | — | — | — | — | | |

## Other peripherals

| Feature | Pi 1 / Zero | Pi 2 | Pi 3 | Zero 2 W | Pi 4 / 400 | Pi 5 |
| --- | :-: | :-: | :-: | :-: | :-: | :-: |
| RNG (BCM2835 family) | | | ✅ | | — | — |
| RNG200 | — | — | — | — | | |
| Camera (Unicam / CSI-2) | | | | | | |
| OV5647 sensor | | | | | | |
| Touch (FT5406) | | | | | | |
| PWM audio to the analog jack |  | | | — | | — |
| IR receive (GPIO edge timing) | | | | | | |
| Real-time clock | — | — | — | — | — | |
| Power button | — | — | — | — | — | |
| PMIC | — | — | — | — | — | |
| Fan control and tachometer | | | | | | |

## Chip selection is compile-time

Building requires picking exactly one of the `bcm2837` or `bcm2711`
features — neither is a default, because there is no sensible default
target chip. The choice selects the peripheral memory map and the PAC
(`bcm2837-lpa` or `bcm2711-lpa`), and under `bcm2711` also switches the
MMU's peripheral base and the SD driver to `EMMC2`. It is a compile-time
decision, not runtime model detection, so one binary does not span columns
of these tables.

There is no feature for BCM2835 (Pi 1 / Zero) or BCM2712 (Pi 5). Those are
ports that do not exist, so those two columns describe boards the crate
cannot currently target at all rather than boards awaiting validation.

## Per-model notes

### Pi 4 / BCM2711

`bcm2711` is **preliminary**. It selects the relocated peripheral memory
map and PAC, and a subset of drivers has been brought up in both 32-bit
(`armv7a-none-eabi`) and 64-bit (`aarch64-unknown-none-softfloat`) builds:
boot, GPIO and the System Timer, the MMU identity map and mailbox
coherency, the UART console, and the SD card. Most other drivers have never
run on the hardware. See
[issue #29](https://github.com/joeferner/rpi-hal/issues/29) for the
bring-up plan.

**There is no interrupt controller support.** BCM2711 uses a GIC-400 where
earlier chips use the legacy controller, so `src/lic.rs` does not apply and
is not built under `bcm2711`. Everything interrupt-driven is unavailable
until GIC-400 support lands.

The SD slot is wired to a different controller than on earlier chips —
`EMMC2`, not the classic Arasan `EMMC` — so `bcm2711` switches the driver
to `sd::Emmc2`, which also needs a mailbox clock-enable and a
`POWER_CONTROL` register write that older chips do not. The DMA-backed
block transfers are not available under `bcm2711`.

BCM2711 adds a substantial amount of hardware that has no driver here at
all: four more PL011 UARTs (UART2–UART5), four more SPI controllers
(SPI3–SPI6), four more I2C controllers (BSC3–BSC6), the DMA4 channels with
40-bit addressing, 40-bit physical addressing generally, a GENET gigabit
MAC in place of the LAN9514, a PCIe root complex with a VL805 xHCI
controller behind it, a DWC2 OTG port on USB-C, a second HDMI output, and
an HEVC decoder distinct from the older H.264 block.

The RNG is a different block from the BCM2835-family one, so `src/rng.rs`
does not carry over. Likewise the GPU: Pi 4 ships a VC6 under the same
"V3D" name that the `v3d` feature does not cover, and BCM2711 reassigns
some GPIO alt-function names relative to BCM2837.

### Pi 5 / BCM2712

Not a target. GPIO, UART, SPI, I2C, PWM, PCM, USB and Ethernet all moved
onto the **RP1** southbridge, reached across PCIe — a different bus topology
rather than a relocated register map, so it is a larger port than BCM2711
was. RP1 also brings a PIO block, and BCM2712 brings a real-time clock, a
soft power button, a PMIC and a fan tachometer, none of which exist on
earlier boards.

Two things go the other way and can never be filled in: there is no
3.5 mm jack, so analog PWM audio does not apply, and the hardware H.264
decoder and video encoder were both removed, leaving HEVC decode only.

### Pi 3 / BCM2837

The reference target: both execution states, and the only board where the
on-board radio drivers apply. The BCM43438's SDIO interface, the Wi-Fi
driver above it, and the Bluetooth HCI transport are all specific to this
board's wiring.

Driving Wi-Fi gives up the SD card slot — one controller with two possible
routes, and claiming it for SDIO hands the slot pins to SDHOST.

`I2c<BSC0>` on GPIO44/45 (ALT1) is the camera/display connector bus, not
BSC0's GPIO0/1 HAT-EEPROM routing. `AuxSpi<SPI2>` (GPIO40-45) is not broken
out on the 40-pin header.

### Pi 2 / BCM2836

Shares BCM2837's peripheral map, so SoC-level drivers are expected to carry
over, but it is 32-bit only and has no on-board radio, so the Wi-Fi and
Bluetooth rows cannot apply.

### Pi 1 / Zero / BCM2835

Not a target. ARMv6 needs a Rust target this crate does not build for and a
single-core boot path, and ARM1176 has no ARM generic timer — so the
secondary-core and generic-timer rows can never apply.

Its USB is DWC2 with no hub in the path, unlike Pi 2/3 where every port
sits behind the LAN9514, so the USB host driver would face a different
topology. Zero and Zero W have no Ethernet and no analog jack.

### VideoCore-mediated features

VCHIQ, MMAL, H.264 decode and HDMI audio go through VideoCore firmware
rather than ARM-side registers. The transport is chip-agnostic, but what
the firmware exposes behind it is not, so each of these needs validating
per board independently of whether the transport comes up.

Every Pi 3 mark above rests on `hil_smoke` and `hil_core` run in **both**
execution states, not one. That is not thoroughness for its own sake: the
generic timer passed in AArch64 and failed in AArch32 by a factor of 19.2,
because the two states reach different firmware setup. A single-arch run
would have marked that row green.

## Footnotes

[^cache]: Not asserted directly. Covered by the DMA copy, which the driver
    performs cache maintenance around: without it the transfer would read
    stale data or the destination invalidate would discard neighbours, and
    the byte-for-byte comparison would fail.

[^reboot]: Covered as a side effect rather than by its own case. Each case
    binary ends with `power::reboot()` so it hands the board back to the
    loader, and the next binary in the run finding a live loader is the
    evidence that the reset took effect. No case asserts the reset *cause*.

[^clock]: Only the read side. `clock_rate_hz` and `max_clock_rate_hz` are
    exercised and cross-checked against each other; `set_clock_rate_hz` is
    not, because changing a clock rate mid-suite would alter the timing every
    other case measures against.

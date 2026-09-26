# Changelog

Notable changes to `rpi-hal`, in the format of
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). This crate
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **`usb::ethernet::Ethernet`, one description both Ethernet drivers
  answer to.** `usb::lan9514` and `usb::lan7800` were written to the same
  surface on purpose so that a consumer ports between them by changing a
  type name; this makes that a type rather than a convention, so code can
  be written once and compiled for either board without being told which.

  The receive iterator is what needed it. Each driver returns its own
  `Frames` — the same item type, different types — so a function handling
  both had nothing it could return and had to consume the frames where it
  produced them. A generic associated type (`type Frames<'a>`) removes
  that, and `examples/usb_ethernet.rs` lost 208 lines of per-method
  dispatch to it: `run_ethernet` is now generic over the trait and its
  receive loop reads like either driver's own.

  `IdRevision` moved here and is re-exported from both drivers, since it
  is the same two numbers in the same places and only the ID that means a
  working part differs. `usb::lan9514::IdRevision` still resolves.

  This is the blocking surface only. The split into borrow-disjoint halves
  that an executor needs will be a separate `EthernetAsync` extension —
  it is not here because only one of the two drivers has an async half so
  far, and a trait with one implementation would be guessing at the shape.

- **`usb::lan7800`, a driver for the Pi 3B+'s Ethernet** — the Microchip
  LAN7800 half of the LAN7515 that board carries in place of a Pi 2B/3B's
  SMSC LAN9514 ([issue #101](https://github.com/joeferner/rpi-hal/issues/101)).
  Until now a 3B+ had no wired network under this crate at all.

  The same surface as `usb::lan9514` — `from_device`, `start`,
  `is_link_up`, `set_all_multicast`, `send_frame`, `receive_frames` — so a
  consumer ports between them by changing a type name. The contents share
  nothing: a different register map, a different PHY bring-up, an 8-byte
  transmit header against the LAN9514's, and a 10-byte receive header whose
  padding is computed on the frame length *plus two* rather than on the
  length alone. That last one is worth knowing, because getting it wrong
  doesn't lose one frame, it loses every frame behind it in the transfer.

  `examples/usb_ethernet.rs` now drives whichever chip the board has,
  offering each enumerated device to both drivers and taking the one that
  claims it, so a single image covers a 2B/3B and a 3B+. It also walks the
  bus with `usb::Bus` rather than `usb::enumerate`, and has to: on a 3B+
  the LAN7800 sits behind two cascaded hubs and attaches seconds after
  power-on, so a one-shot walk finishes before it exists.

  `set_promiscuous` is here and has no LAN9514 counterpart yet, because
  this chip filters unicast against a perfect-address table by default and
  the bit to turn that off is a natural pair with `set_all_multicast`.

  **The link is 100BASE-TX, not gigabit, and that is deliberate.** Left
  advertising 1000BASE-T, the link never comes up at all: base pages
  exchange and the partner acknowledges them, there is no parallel-detect
  or master/slave fault, but 1000BASE-T training never converges — the
  remote-receiver-OK bit never sets — so negotiation restarts and fails
  indefinitely without ever falling back. Training needs analogue setup
  particular to this PHY; Linux gets gigabit because phylib binds
  `drivers/net/phy/microchip.c` to the LAN88xx core inside the chip, which
  programs DSP and MDIX registers `lan78xx.c` never mentions. Withdrawing
  gigabit costs little here, since the chip reaches the host over USB 2.0
  and a gigabit line could not be filled through it anyway.

  Two things caught only on hardware, both recorded in the code. The
  receive header's padding is computed on the frame length *plus two*, not
  on the length alone. And `USB_CFG0.BIR` has the opposite sense to the
  LAN9514 bit of the same name: here setting it selects NAK and clearing
  it selects the zero-length reply, so carrying the polarity across left
  every idle poll blocked in a NAK'd transfer until it timed out, dropping
  most of the traffic that arrived meanwhile.

- **`usb::Bus`, which keeps the bus state a walk used to throw away** —
  so a device that attaches after startup can still be brought up, and one
  that is unplugged is reported rather than silently left addressed
  ([issue #102](https://github.com/joeferner/rpi-hal/issues/102)).

  `enumerate` is a snapshot, and the reason it could not be anything else
  is that it owned the two things bringing a device up requires and then
  dropped them: the next free address, and the hub topology every split
  target is derived from. A caller that later noticed a connection had
  neither. `Bus` holds both across calls, so `Bus::poll` does what
  `Bus::enumerate` did, later and for one port.

  `poll` reads every port of every tracked hub with GET_PORT_STATUS and
  compares what the hub reports against what it last recorded there. A
  port that has gained a device gets the full bring-up, including being
  walked in turn if it is a hub with devices already on it; a port that
  has lost one reports every device that was behind it, deepest first, and
  frees their addresses. The comparison is against the port's present
  state rather than its change bits, so a missed sweep is harmless where a
  missed edge would not be.

  A hub's status-change endpoint would name just the ports that moved, and
  that is what a full host polls. It is deliberately not used here. This
  crate's DWC2 driver cannot yet schedule high-speed periodic transfers
  reliably, and on the bench that endpoint both returned `FrameOverrun`
  and — worse — completed *successfully* with an all-zero bitmap while the
  hub's own port status showed a connection change outstanding, so a
  device plugged into that hub was never seen at all. Reading the ports
  says the same thing over transfers that work. Linux arranges the same
  safety net from the other side: its hub driver runs on that endpoint,
  but `hub_activate()` reads every port directly on init, resume and
  reset, and ten consecutive endpoint errors force exactly that reset.
  `Hub::status_endpoint` is exposed for a caller who wants it, and for
  `poll` to build on once the periodic scheduling is fixed — which would
  not change what `poll` promises.

  A port whose device fails to come up is tried once and then left until
  the port's connection changes again, so a device this can't talk to
  costs one attempt rather than a port reset on every sweep.

  This is what a late device needs, and no amount of waiting substitutes:
  on a Pi 3B+ the soldered LAN7800 Ethernet attaches seconds after
  power-on, long after any debounce, and a snapshot taken at boot simply
  does not contain it.

  Addresses are now a 128-bit bitmap rather than a counter, because they
  come back when a device is unplugged and a bus that is replugged all day
  would otherwise run out. A freed address can be reused immediately,
  which is why `Event::Detached` carries it — a driver still talking to it
  afterwards is talking to whatever arrived next.

  `usb::enumerate` is unchanged and now a thin wrapper over
  `Bus::enumerate`, so nothing that only needs the one-shot walk has to
  move. `usb::hub::Hub` gained `status_endpoint` (located while
  `configure` was already reading the configuration descriptor) and is now
  `Copy`.

- **`usb::enumerate` walks hubs plugged into hubs.** It used to stop one
  level down: a hub in one of the board's ports was reported as a device
  and everything behind it was invisible, so a keyboard on a hub simply
  did not exist as far as this crate was concerned
  ([issue #15](https://github.com/joeferner/rpi-hal/issues/15)). The
  per-port bring-up — reset, probe, address, and now configure-and-descend
  when the descriptor says class 9 — is applied to each hub it meets, in
  depth-first order, down to the four levels below the root hub that USB
  2.0 §4.1.1 allows.

  `usb::Device` gained `hub_address` and `depth` to say where on the bus
  the device was found, which a flat walk had no need to report. The
  callback is generic, so the recursion goes through a `dyn FnMut`
  internally: monomorphizing a generic callback per nesting level is a
  recursion the compiler does not terminate.

  What actually makes a device several levels down reachable is the split
  target, and it is not simply "the hub it is plugged into". A transaction
  translator lives in a *high-speed* hub, so a full-speed hub below one
  has none of its own — everything under it is on the same full-speed
  segment and belongs to the translator further up, which is now what
  `usb::hub::Hub::split_target` returns. Getting this wrong is not a
  degraded transfer, it is a transfer addressed to a translator that isn't
  there.

### Changed

- **The hub port requests in `usb::control` take the hub's
  `ControlEndpoint` instead of its address and low-speed flag** —
  `set_port_power`, `get_port_status`, `set_port_reset` and
  `clear_port_feature`. Breaking, and it is what makes the recursion above
  possible: those requests are addressed *to the hub*, so reaching a hub
  that is itself behind another hub means carrying that hub's own split
  target, which an address and a bool cannot express.

- **`usb::hub::Hub::configure` waits the connect debounce on top of
  `bPwrOn2PwrGood`**, 100ms more per hub. The two waits are for different
  things: the power-good delay is the hub's rail reaching the port, and
  only once it has does the attached device start up and drive the pull-up
  that announces it — which USB 2.0 §7.1.7.3 allows a further 100ms
  (`TATTDB`) to settle. This is conformance rather than a fix for anything
  observed; no device on the bench was found to need it. It is also not a
  general answer to a device that attaches late, which no fixed delay is:
  that is what a hub's status-change endpoint is for, and `enumerate`
  remains a one-shot snapshot of the bus.

- **`usb::hub::Hub::configure` takes a `high_speed` flag** and exposes it
  as a field, alongside a new `Hub::endpoint` accessor. Breaking.
  `ControlEndpoint` distinguishes only low speed from the rest, because
  that is the one bit a transfer puts on the wire — but whether a hub has
  a transaction translator at all turns on high speed versus full, so the
  caller that reset the port (and read its speed bits) has to pass it in.

## [0.7.0] - 2026-09-25

### Added

- **`sdhost`, a driver for the SoC's *other* SD host controller** — and
  with it, the ability to drive the SD card and the wireless chip at the
  same time.

  The Arasan controller `sd` and `sdio` share can be muxed to the card
  slot (GPIO48-53, ALT3) or to the radio (GPIO34-39, ALT3), never both,
  so until now a program that brought Wi-Fi up gave the card away for
  good. SDHOST is Broadcom's own, simpler controller at `0x7E20_2000`,
  reaching the card slot at ALT0 — so the card goes there and the radio
  keeps Arasan to itself. That is the split Raspberry Pi OS makes on a
  Pi 3 and a Zero W, and it does not go the other way round: SDHOST has
  no usable SDIO, so the radio cannot move instead.

  `sdio::Sdio::init` already parked GPIO48-53 on ALT0 when it took the
  wireless pins — it has to, or one controller would be wired to both
  pin groups — so the card slot was already pointed here and waiting for
  a driver.

  Blocking PIO only: the identification sequence, the 4-bit bus,
  single- and multi-block reads and writes paced against the FIFO level
  in `SDEDM`, and an `SdhostBlockDevice` for `resident-fat` that moves a
  whole run in one command. The register map is absent from the SVD *and*
  from the BCM2835 datasheet, so it is poked directly and follows Linux's
  `bcm2835-sdhost`. Not built for `bcm2711`, where the card is on EMMC2
  and GPIO48-53 are the Ethernet PHY's RGMII interface.

  The card-side protocol is deliberately written out again rather than
  shared with `sd`: what the two have in common is a page of the SD
  specification, what differs is every register it is expressed in, and
  factoring it out would put a hardware-verified driver's bring-up at the
  mercy of edits made for this one.

  HW-verified on a Pi Zero W 1.1 by `examples/sdhost_read.rs`, which is
  the test of the claim rather than of the driver: it mounts the card on
  SDHOST, brings the radio up on the Arasan controller, and then reads
  the card again with both running. Writes are verified separately, by an
  over-the-air update writing a bundle to the card and reading every
  entry back — reads measured around 10 MB/s.

  Two things about writing that reads do not reach, and that cost a
  round of debugging each. A write must let the FIFO drain *before* its
  stop command, which is the opposite of a read: the card streams until
  `CMD12` on the way in, so stopping first is a deadlock, while on the
  way out the last words are still in the FIFO and stopping first
  truncates them. And nothing may touch the bus until the card has
  finished committing — the controller reports the release of the busy
  signal in `SDHSTS`, and the `SDCMD` start bit clearing does not mean an
  `R1b` command is done. Both produce a write that reports success and
  fails its read-back.

- **`wifi::Wifi::bssid`**, the firmware's own answer to whether the chip
  is on a network: the associated AP's address, or `None`.

  What it is for is watching an association rather than making one.
  `WLC_GET_BSSID` refuses the command while unassociated and reads back
  zeros in the window between a join being issued and it landing, so both
  come back as `None`; a command that did not get through at all stays an
  error, because a chip that has stopped answering is a different fault
  from a radio that is off the network and is not fixed by rejoining.
  `join_wpa2` now waits on this rather than open-coding the same ioctl.

- **`wifi::Wifi::resync_rx`**, which abandons the receive frame in
  progress and waits for the chip to flush it.

  This is the way out of a desynchronized function-2 stream and there is
  no other: the FIFO is a byte stream with no frame boundary a host can
  search for, so once a read has stopped part-way through a frame, every
  read after it is offset by whatever is left. Called automatically when
  a malformed header is read, which is what turns one bad header from
  permanent into a single dropped frame. Modelled on `brcmfmac`'s
  `rxfail`.

- **Coalesced receive frames are unpacked.** Under load the firmware
  packs several Ethernet frames into one SDPCM *superframe* on channel 3
  — nine of them, 13,856 bytes, measured on a 43430 — and it does so
  whether or not the host asks it not to. `recv_ethernet` reads one once
  and hands back the packets inside it one call at a time, so callers
  see no difference; `WifiPhy` and any receive loop already written
  against it need no change.

  Before this, such a frame was refused whole. That is not one packet
  lost but every packet in it, which is why a link would carry DHCP and
  a web page perfectly and then stall the moment anything downloaded —
  a download being exactly what gives the firmware frames to coalesce.

  The subframes are found by searching rather than by striding: each
  carries a length and that length's complement, so a header is
  recognizable on sight, and the padding between them can be skipped
  without knowing the rule the firmware pads by. Measured, a 1530-byte
  frame occupies 1536, which is consistent with several alignments and
  settles none of them.

  Two consequences worth naming. `MAX_FRAME` is now the length field's
  own 64 KiB ceiling, so no frame the chip can describe is too large to
  read — at the cost of that much RAM in a `Wifi`. And outgoing data
  frames are built in a buffer of their own, because a transmit sharing
  the receive buffer would throw away the rest of a superframe still
  being handed out, which during a download is most of them.

- **`wifi::Wifi::set_power_management`** and `wifi::PowerManagement`,
  which say how aggressively the radio may sleep between frames.

  The firmware powers on sleeping, so a program that never asks gets a
  radio that does — the right default for something battery-powered and
  the wrong one for a board on a wall supply, which is why this is a
  decision rather than a default. `brcmfmac` and `cyw43` both set it
  explicitly after associating, for the same reason.

  What sleeping costs is latency, not throughput directly, and only when
  the link goes briefly quiet — which is why a healthy ping says nothing
  about it. A ping is one packet against an idle radio, the one case a
  sleeping chip handles well. A window-limited bulk transfer is quiet by
  construction instead: the sender fills the receive window and waits,
  and a radio that reads that pause as idleness adds its wake-up to
  every round trip. Throughput being the window over the round trip, the
  cost lands on the whole transfer rather than on the pauses.

- **`wifi::Wifi::counters`** and `wifi::Counters`, the firmware's own
  MAC-layer counters.

  Everything else this driver reports is counted above the chip: frames
  the host moved and errors the host could see. A frame the radio
  retried four times and then delivered is an error nowhere in that
  picture — it cost air time and latency and arrived intact — so a link
  working hard and a link working well look identical from the host.
  `txretrans` against `txframe` is how much of the transmit effort is
  repeat work, and `rxoflo` counts frames the chip took off the air and
  dropped because the host had not emptied its FIFO: the one loss on
  this path nothing above the chip can see, since the frame never
  arrives to be counted.

  Only the leading fields, through `rxuflo`, are decoded — they are what
  every layout version agrees on. The reply buffer is nonetheless sized
  for the largest layout, because the firmware checks the room it is
  offered against the whole structure and refuses the command outright
  if it is short: a buffer big enough for the fields wanted is not big
  enough to ask with. A version outside the known-good range gives
  `Error::UnsupportedFormat` rather than a misread, since later firmware
  answers this iovar with a tagged format under a much higher version
  number.

- **`wifi::Wifi::link_rate_kbps`** and **`wifi::Wifi::rssi_dbm`**, the
  rate the two ends settled on and the signal strength behind it. A
  transfer that seems mysteriously slow is often just a link that has
  fallen back to the 802.11b rates, which is a ceiling nothing above it
  can argue with; the RSSI says why it fell back.

- **`wifi::Wifi::set_rx_glom`**, which asks the firmware to coalesce
  received frames or not to.

  A preference rather than a requirement, now that both answers work. A
  43430 running 7.45.98 returns success for `false` and coalesces
  anyway. `Wifi::new` asks at bring-up and ignores the answer; this is
  public so a caller can see what the firmware said.

- **`fault-report`, an optional handler that says what the fault was
  instead of parking silently.** The vector table's
  `__unhandled_exception` is weak and its default body is `wfe; b .`, so
  an unhandled data abort stops the core and prints nothing. On a console
  that is indistinguishable from a hang in a driver, a deadlock, or a
  wedged peripheral — three bugs investigated in three different
  directions, and the one that produces no evidence is the one you get.

  ```text
  FAULT: data abort on core 0
    pc     0x00008364
    addr   0xf0000000  read
    cause  translation fault, first level
    dfsr   0x00000005   spsr 0x600001d3
    stack  0x00400000..0x00500000
  ```

  `pc` has the per-exception bias already subtracted (8 for a data abort,
  4 for the rest on AArch32; none on AArch64, where `ELR_EL1` is exact),
  so it goes straight into a disassembly rather than being adjusted by
  hand, correctly, by whoever is reading at the time.

  Off by default, and the reason is not cost — it is a few hundred bytes
  of `.text` and a 4 KiB `.bss` stack on AArch64. It is that the symbol is
  a definition rather than a hook: an application that already has its own
  and enables this gets a duplicate-symbol link error, and there is no
  sensible way to guess which was meant. Opting in is how an application
  says it has none.

  Each vector slot now passes its index, so a handler is
  `extern "C" fn(kind: u32)`. Both architectures need that number and for
  different reasons: on AArch32 a data abort and a prefetch abort share a
  CPSR mode and differ in which register pair holds the answer, and on
  AArch64 `ESR_EL1` is not written by an IRQ or an FIQ at all, so reading
  it for those prints whatever the last synchronous exception left — a
  confident wrong diagnosis. The stubs branch rather than call, so `lr`
  still holds the faulting address and a handler written against the older
  no-argument shape keeps working.

- **A stack overflow now faults where it happens**, under `mmu` with
  `rt`. `linker.ld` has reserved `__stack_slack` below the main stack
  since the stacks moved out of low memory, against the day the MMU could
  leave that block unmapped. Until now an overflow was not an error at
  all: the map is an identity map of everything below the peripheral base,
  so `sp` descending past `__stack_bottom` crossed nothing the hardware
  objected to — it walked the margin, reached `.data` and `.text`, and
  overwrote the running program. What that looks like from outside is not
  a fault but a board that behaves strangely and later stops, arbitrarily
  far from the call that went too deep.

  The descriptors lying entirely within the margin are now cleared, from
  `rpi_hal_mmu_init` after the table is built and before the MMU is
  enabled — which is what makes it free of cache and TLB maintenance: the
  write reaches RAM with the caches off, and there is no stale translation
  to shoot down for an address that has never been translated. Writing the
  same zeroes on every core is why a secondary core calling it again is
  harmless.

  The region is rounded inward, so a margin overridden to something that
  is not a multiple of a descriptor gives up only the descriptors inside
  it, and one smaller than a descriptor yields no guard rather than an
  approximate one. It covers the main stack of the boot core and nothing
  else: the AArch32 exception-mode stacks are above `__stack_top`, and a
  secondary core's is a `multicore::Stack` in `.bss`, both with their
  neighbours right below them as this one used to have.

  With `fault-report` on, the report names it:

  ```text
  FAULT: data abort on core 0
    pc     0x00009a5c
    addr   0x003fffa8  write
    cause  translation fault, first level
    *** past the bottom of the stack: this is a stack overflow
  ```

  `pc` there is inside `__aeabi_memclr4`, zeroing a frame that no longer
  fits, rather than in the function that recursed — the address is the
  half worth reading.

- **`mem`, which says where free memory starts and stops.**
  `mem::heap_region` answers the range from the end of `.bss` up to the
  top of the ARM side of the memory split, with the top asked of the
  firmware rather than hardcoded, so one image is right whatever `gpu_mem`
  a board is set to. `mem::image_end` is the lower bound on its own, for a
  program placing something other than a heap up there. What stays in the
  binary is the `#[global_allocator]` — which has to, since a program may
  have only one and a HAL must not choose it.

  Three things the hand-written copies of this had wrong or left implicit.
  `__bss_end` is raised to a multiple of 8: the linker script promises
  only word alignment, which is all the boot code's `.bss` zeroing needs,
  while an allocator hands out blocks aligned for `u64` — 8 even on
  AArch32 where a pointer is 4. It has not bitten, because `.bss` has so
  far ended 8-aligned by luck, and is one byte away from not. The upper
  bound is computed in `u64`, since both fields the firmware reports are
  `u32` and a board reporting memory up to the top of the address space
  would wrap the sum to zero — turning "all of it" into "none of it". And
  an empty region is `Error::NoRoom` carrying both addresses rather than a
  zero-length range: which of the two is surprising is the whole
  diagnosis, a large `image_end` being a kernel that has grown and a small
  `memory_end` a `gpu_mem` that has been raised.

- **`Mailbox::select_display` and `Mailbox::display_mode`**, the two
  things every framebuffer consumer has to do before it can allocate —
  both of them about firmware behaviour rather than about what the program
  wants to draw.

  `select_display` takes a preference list, most preferred first, and
  points the framebuffer tags at the first display attached. Which display
  the firmware enumerates as number 0 is not stable from boot to boot, so
  a board with two attached and no preference gets a coin toss.

  `display_mode` answers the size to allocate so the picture is not
  scaled. That is more than one query, because the firmware keeps a blank
  overscan border, "Get Physical Width/Height" reports the image inside
  it, and clearing the border does not resize a framebuffer already made —
  so the border has to be read first and added back arithmetically. Get it
  wrong and the firmware stretches the buffer to the mode by a different
  factor per axis, which is what a logo drawn round and shown oval means.

  Neither logs, because this crate has no console. Everything worth saying
  comes back instead: `Selection` carries what was found and under which
  numbers, what was taken, and an `Outcome` saying whether that was the
  preference, the firmware's own choice, or a fallback — four cases rather
  than a bool, since only the caller knows whether an unmet preference is
  worth saying loudly. `DisplayMode` carries the border it found and
  whether it managed to clear it, which is the difference between "the
  full mode" and "the inside of a border the firmware would not give up".

  `Selection::reported_count` exists because the hardware asked for it.
  With a panel and HDMI both plugged in, this still reported one display,
  and `Outcome::Sole` could not say whether the firmware did not know the
  tag or knew it and meant it. Those want different fixes, and the second
  one is `max_framebuffers`, which is 1 by default in `config.txt` and
  caps what the firmware will enumerate however much is plugged in.
  Without the count that reads as a cabling problem.

### Fixed

- **`Rng::new` handed out words the generator had queued before it was
  asked to warm up.** The constructor arms a discard of 262,144 samples
  and documents the first read as transparently waiting it out. On a
  board whose generator was already enabled — left running by the
  firmware, or by a previous image loaded over a UART loader, which is
  the normal case when developing without power-cycling — arming the
  count does not flush the output FIFO, so the first reads returned the
  queued words and the ~0.74 s warmup stall landed on the read after
  they ran out. Timed per word on a Pi 3: 6 µs, 5 µs, 4 µs, 4 µs, then
  736,688 µs.

  Those four may well be good output from the generator's previous run,
  but nothing readable from inside `Rng` says whether whatever enabled
  the block armed a warmup of its own; if it did not, they are exactly
  the early biased samples the discard exists to remove. For a function
  whose callers include a TLS client random and an ECDHE private half,
  on a system with no entropy pool and no seed file behind it, that is
  not a distinction to leave to chance. `new` now drains the FIFO until
  the status word count reads zero. A cold boot is unaffected — the FIFO
  is empty and the loop does not run — and the stall moves back onto the
  first read, where it is predictable.

- **The instruction cache was never enabled on ARMv7-A**, so every
  instruction fetch went to DRAM over a bus shared with the VideoCore.
  `mmu32.rs` set `SCTLR.M` and `SCTLR.C` and left `SCTLR.I` clear, on the
  stated grounds that the I-cache was "unrelated" to the `ldrex`/`strex`
  problem `C` was there for. That was true and beside the point.

  Measured on a Pi 3, reading a buffer with a summing loop:

  | | before | after |
  |---|---|---|
  | buffer that fits in L1 | 17 MB/s | 1597 MB/s |
  | buffer far larger than any cache | 30 MB/s | 1115 MB/s |

  The giveaway is in the *before* column: the L1 figure is the lower of
  the two, which can only mean the loop was bound by fetching its own
  instructions rather than by the data it was reading. For scale, an
  application parsing a TrueType face on the same board went from 84.5
  seconds to 2.3, and rasterizing eleven glyphs from 287 ms to 6.5 ms.

  It costs everything, not one workload — card reads, the network stack,
  any decode — and it is close to invisible, because a board that is
  merely a hundred times slower than it should be still boots, still
  serves pages and still keeps time. Nothing reports it. An application
  that wants to know can read the three bits itself:

  ```rust
  let sctlr: u32;
  unsafe { core::arch::asm!("mrc p15, 0, {0}, c1, c0, 0", out(reg) sctlr) };
  // bit 0 MMU, bit 2 D-cache, bit 12 I-cache
  ```

  ARMv6 and AArch64 were never affected — both set all three already,
  and the ARMv6 arm carried the written-down reasoning for why an
  uncached fetch of every instruction is expensive. Only the ARMv7 path
  was missing it.

- **A secondary core launched on AArch32 never reached its entry
  function**, because `__secondary_core_entry` did not drop out of Hyp
  mode. `_start` has done that for core 0 since the firmware was found to
  hand off in Hyp — the secondary trampoline, written later, did not, and
  neither did anything notice, because the failure is silent by
  construction.

  Every step the trampoline takes next is *banked*, so in Hyp each one
  succeeds at writing a register nothing will read. `cps` cannot leave
  Hyp at all — UNPREDICTABLE per the architecture, which is why `_start`
  uses `eret` — so the banked stacks are never set and `sp` stays
  whatever the firmware's stub left. VBAR is not the vector base in Hyp,
  HVBAR is, so the core takes its exceptions somewhere never
  initialized and an application's `__unhandled_exception` cannot run.
  The MMU enable programs `SCTLR`/`TTBR0` rather than `HSCTLR`/`HTTBR`.

  What that looks like from the other side is worth writing down, because
  it is what makes it hard: the mailbox handoff is written, the
  firmware's stub acknowledges it (mailbox 3 reads back as zero), the
  core really is executing — and it produces no output whatsoever, not
  even a fault report, while core 0 carries on perfectly. Nothing about
  it resembles a core that failed to start.

  AArch64 was never affected: `secondary64.s` drops EL2→EL1 exactly as
  `boot64.s` does, which is why a four-core AArch64 application works
  while the same arrangement on AArch32 does not.

- **`power`, `rng`, `pcm` and `unicam` addressed the wrong chip.** Each
  wrote its register base out as a `0x3f…` literal — the BCM2836/2837
  peripheral base — with no chip selection, so on a BCM2835, whose
  peripherals are at `0x2000_0000`, they addressed nothing at all.

  They now take the base from `soc::PERIPHERAL_BASE`, which is the one
  place in this crate that knows which map is being built for, and which
  `dma` and `watchdog` were already using.

  This is not a wrong-answer bug, which is the only reason it was found
  rather than shipped: the MMU maps the peripheral region the chip
  actually has, so the write faults. `power::reboot` on a Pi Zero took a
  data abort — `dfar 0x3f100024`, the watchdog register at the other
  chip's base — instead of rebooting, which meant a board could be told
  to reboot exactly once and what it did was hang with a fault report.
  `rng` would have done the same on the first random number asked of it.

- **`__unhandled_exception` had two definitions, not a weak one and an
  override.** `vectors.s` declared the symbol `.weak` and parked, and
  `fault.s` under `fault-report` defined it `.global` and reported. Both
  are `global_asm!` in this crate, which is one stream of assembly to the
  assembler rather than two objects for the linker to resolve between — so
  that is a duplicate definition.

  It assembled anyway, which is the part worth recording: whether the two
  landed in the same codegen unit decided whether anyone noticed. Adding a
  call from `mmu32`/`mmu64` into `mmu.rs` was enough to merge them, and
  the build that had worked all along stopped working, several commits
  away from anything to do with faults.

  The default now lives in its own file, included only when
  `fault-report` is off — the shape `mmu_fallback.s` already uses for
  `rpi_hal_mmu_init`, and for the same reason. Exactly one definition is
  compiled either way, and the weak binding stays on the fallback so an
  application can still supply its own when the feature is off.

- **A `mmu`-without-`rt` build warned about dead code.** The stack
  guard's `invalidate_block` helper is reachable only from code that is
  `rt`-gated, because it needs that feature's linker script for the
  symbols naming the region; the helper was not gated, so the combination
  compiled it as unreachable and said so.

  Nothing here saw it: every line of the Makefile has `rt` on — it is a
  default feature — and the one combination that turns it off was not
  linted. The warning surfaced in a downstream crate's doc build instead,
  which is a poor way to find out, so that combination is linted now. It
  is not a hypothetical target: `mmu` is documented as independent of `rt`
  at the Cargo level precisely so a consumer with its own boot sequence
  can take this crate's translation tables, and that consumer is the one
  who was seeing this.

### Changed

- **`wifi::Error::BadFrame` carries `len`, `len_check` and `channel`.**
  Breaking, and the fields are the whole point: they say which
  malformation it was, and the answers are opposite. A `len_check` that
  is not the complement of `len` is a stream that has lost its place. A
  valid complement means the stream is in step and the header itself is
  unusable.

  Those two numbers are what turned a guess into a diagnosis: the flood
  of `BadFrame` that prompted all of this looked like a desynchronized
  stream and was not — `!0x3620 == 0xC9DF`, a valid complement, on
  channel 3. That is a coalesced frame, which is now read rather than
  refused (above).

- **`mailbox::Error` has a new variant, `NoDisplayMode`**, for a firmware
  that answers a display-size query with zero. Breaking for an exhaustive
  `match` on that enum. It is separate from the protocol errors beside it
  because it is the firmware answering rather than failing, and what to do
  about it stays with the caller — a fallback resolution is a policy, not
  a fact about the hardware.

## [0.6.0] - 2026-09-20

### Added

- **Pi 1 and Pi Zero support (BCM2835, ARM1176JZF-S)** — the third
  architecture this crate reaches, behind a `bcm2835` chip feature and
  the `armv6-none-eabi` target. HW-verified on a Pi Zero W 1.1: boot,
  GPIO, the UART console, the System Timer, and the MMU with both caches
  on, including the exclusive monitor (`examples/atomics_check.rs`).

  The chip half is only addresses — `bcm2835-lpa`'s register block
  modules are byte-identical to `bcm2837-lpa`'s, so every driver is the
  same code at a different base. The architecture half is the real work,
  and it is selected by the target rather than by a feature, so it cannot
  disagree with what is being compiled: `src/boot6.s` (no Hyp drop, no
  `MPIDR` check, no secondary cores), the ARMv6 CP15 barriers, the FPU
  enable for a VFPv2-without-NEON core, and `mmu32.rs`'s ARMv6 arm.

  `multicore`, `generic_timer` and `pmu` are not built there: one core,
  and no ARM-local peripheral block for the other two to reach. Asking
  for `multicore` on ARMv6 is a `compile_error!` rather than a quietly
  missing module.

  This is the only chip that needs a nightly toolchain, because
  `armv6-none-eabi` is tier 3 and has no precompiled `core`. That
  requirement belongs to the target rather than to this crate, and the
  other two chips still build on stable — see README.md's "Toolchain".

- **`cpu::main_id`**, the `MIDR` Main ID register, on all three
  architectures. Worth printing as the first line out of a new board's
  console: it says both that the console works and that the chip
  underneath is the one the binary was built for.

- **`examples/atomics_check.rs`**, which proves the exclusive monitor
  works rather than assuming it. Not ARMv6-specific: it is what any board
  should run after a change to the translation table. A broken monitor
  hangs rather than reporting, so each step announces itself before
  running and the line you never see names the operation that never
  finished.

### Changed

- **`cache::barrier` became the `barrier` module** (`dsb`/`dmb`/`isb`),
  internal, with the architecture split inside it instead of at each call
  site. `dsb` and `isb` are ARMv7 mnemonics that ARMv6 has only as CP15
  operations, and a barrier appears in nearly every
  architecture-specific file here. No behaviour change on ARMv7 or
  AArch64: the disassembly is identical instruction for instruction.

## [0.5.0] - 2026-09-04

### Fixed

- **The LAN9514's MAC ran half duplex** — frames were discarded whenever
  the interface transmitted and received at the same time, on every link
  this driver has ever run on.

  `start` wrote `MAC_CR` with `RCVOWN` set and `FDPX` clear, which is a
  half-duplex MAC doing CSMA/CD against a switch that had auto-negotiated
  full duplex. Transmitting while receiving is a collision, and the frame
  goes away.

  Only bidirectional traffic showed it, which is why it lasted. A single
  request was always fast, and receiving alone was lossless — a 256-frame
  back-to-back burst arrived complete, and a sustained 4,000 frames a
  second lost nothing. It took eight files fetched over eight concurrent
  connections: **62% of requests stalled, median 1.0 s against 3.9 ms for
  the same file fetched alone**, worst case 7 s, sitting exactly on the
  peer's retransmission timeout. Against the fix, on the same board and
  the same test: median **13.7 ms**, **0** stalls out of 200, and the
  client's `TcpRetransSegs`, `TcpExtTCPSynRetrans` and
  `TcpExtTCPTimeouts` all zero.

  Nothing above or below the MAC could see it. The driver handed each
  frame over and the transfer succeeded, so a send-failure count read
  zero; the receive loop was healthy, so its counters read zero and the
  window with no bulk IN pending measured 7 µs. Loss inside the MAC is
  invisible from both sides of it.

### Changed

- **`Lan9514::receive_frame` is now `receive_frames` and returns an
  iterator** — breaking, for anyone calling it or its `_async` twin
  directly. Consumers going through `Lan9514Phy` or `rpi-hal-embassy` are
  unaffected.

  ```rust
  // before
  if let Ok(Some(frame)) = lan9514.receive_frame(channel, timer) { ... }
  // after
  for frame in lan9514.receive_frames(channel, timer)? { ... }
  ```

  A bulk IN can carry several frames, each behind its own status word.
  With `HW_CFG.MEF` clear — as this driver leaves it — the chip sends one
  per transfer, so the old signature was correct by accident of a bit that
  is not set rather than by design. Returning an iterator means the API
  can no longer express the bug, and enabling coalescing later becomes a
  change to `start` rather than to every caller.

  Iteration stops at the first status word that cannot describe a frame,
  because past that point the offsets are guesses and a guess yields
  corrupt frames rather than a gap. `start` also clears `HW_CFG.RXDOFF`
  explicitly: zero is the reset default, so nothing changes, but the
  parser depends on it.

### Added

- `Lan9514::set_duplex` and `Lan9514::is_full_duplex`, for programming the
  MAC from what auto-negotiation actually settled on. `start` still
  assumes full duplex, because it runs before the link is up and half
  duplex needs a hub; the sequence for certainty is `start`, poll
  `is_link_up`, then `set_duplex`. `is_full_duplex` intersects the two
  standard MII ability registers the way auto-negotiation does, rather
  than trusting one PHY's summary of the result.
- `Lan9514::set_all_multicast` and its `_async` twin, for the `MAC_CR`
  multicast filter. The chip comes up dropping multicast before the host
  sees it, which anything speaking only unicast or broadcast never notices
  — DHCP is broadcast — and which makes mDNS fail completely and silently,
  since its queries and announcements are multicast. A read-modify-write,
  so it composes with `start` and `set_duplex`, which share that register.

## [0.4.0] - 2026-09-02

### Changed

- **`sd::Error` is now `#[non_exhaustive]`, and has a new variant** —
  breaking, for any consumer matching it exhaustively. Add a `_` arm.

  The variant is `NoCard` (below), and `#[non_exhaustive]` comes with it
  deliberately rather than later: without it, teaching the driver to tell
  one failure from another costs a major version every time, which is
  exactly why `SdBlockDeviceError` was given its own enum instead of a
  variant here. One breaking release now, and none for this reason again.

### Fixed

- **`Sd::init` muxed the Ethernet PHY's pins away on a Pi 4.** It routed
  GPIO48-53 to alternate function 7 on every chip, but on BCM2711 the
  card slot is on EMMC2, which drives dedicated pads outside the 54-pin
  bank — `bcm2711.dtsi`'s `emmc2` node has no `pinctrl` property at all,
  which is why the Pi 4 SD path worked regardless. What GPIO48-53 carry
  on that board is the gigabit Ethernet PHY's RGMII interface
  (`RGMII_RXD0`..`RXD3`, `RGMII_TXCLK`, `RGMII_TXCTL`), so the mux was
  pure side effect: it severs the MAC from the PHY, and points four
  lines the PHY drives at a host controller that drives them back during
  a transfer. `route_gpio_to_emmc` is now compiled out under `bcm2711`;
  `Sd::init` keeps its `GPIO` argument on both chips so a call site
  doesn't have to change. Untested on hardware in the direction that
  matters — nothing in this crate drives BCM2711 Ethernet yet, so
  nothing here could have noticed.

  The comment that justified sharing the routing said GPIO48-53's ALT3
  assignment was "unchanged (confirmed by diffing `bcm2711-lpa` against
  `bcm2837-lpa`)". That was true and beside the point: a PAC diff
  describes the SoC's function numbering, not what a board wired to the
  pads.

- **PWM and PCM clock divisors were silently masked, not clamped.** The
  Clock Manager's `DIVI` field is 12 bits, but `Pwm::init` and `Pcm::init`
  take a `u16` and said nothing about the limit — so a larger value was
  neither rejected nor saturated. The PAC's field writer masked it, making a
  divisor of 12500 program as `12500 & 0xFFF` = 212 and run the clock 59
  times too fast, with every register reading back exactly as written. Both
  now clamp. `Pwm::audio_clock_divisor` and `Pcm::clock_divisor` had the same
  fault from the other end, clamping their results to `u16::MAX` — sixteen
  times what the field holds — and now clamp to the real maximum.

### Added

- **`sd::Error::NoCard`**, so an empty slot says so. `Sd::init` used to
  report it as `CardError` carrying a raw `INTERRUPT` word, indis-
  tinguishable without decoding from a card that is present and
  misbehaving. It now returns `NoCard` when `CMD8` — the first command in
  the identification sequence that expects an answer — times out and a
  `CMD55` sent afterwards times out too. Both, because `CMD8` arrived
  with SD 2.0 and a v1.x card doesn't answer it either; a single silent
  command would report an absent card for one sitting in the slot. (Such
  a card still fails `init` exactly as before, with `CMD8`'s own error.
  Supporting one is a separate feature.)

  Presence can only be discovered by asking: no Pi wires a card-detect
  line anywhere a driver could read it — GPIO47, the pin usually named
  for the job, is the ACT LED on a Pi 1/2, the PMIC's I²C data line on a
  Pi 3 and part of the Ethernet PHY's RGMII interface on a Pi 4 — and
  this controller doesn't implement the SDHCI present-state bits.
  `examples/sd_presence.rs` demonstrates it, card in and card out, and
  decodes the controller state behind whatever error comes back.

- **Interrupt-driven SD transfers**, behind the `async` feature:
  `Sd::read_block_async`/`read_blocks_async`/`write_block_async`/
  `write_blocks_async` and the DMA pair
  `read_blocks_dma_async`/`write_blocks_dma_async`, plus `sd::on_irq` and
  `Lic::enable_emmc_irq`/`disable_emmc_irq`/`is_emmc_pending` to route the
  controller's line. The blocking methods are unchanged and untouched by
  this; the async ones park on the controller's interrupt where those
  spin, which matters most for a write, whose closing `DATA_DONE` is the
  card programming an entire internal erase block — milliseconds per
  command that an executor previously lost in full.
  `examples/sd_async.rs` reports, for each transfer, the share of its
  duration during which the core had nothing to do.

  Dropping a transfer future — `embassy_time::with_timeout`, `select!`, a
  cancelled task — stops the card and resets the controller's data
  circuit before the drop returns, and so does an error return. Without
  that, an abandoned data phase would leave part of an aborted block in
  the host FIFO for the *next* transfer to return as though it were data.

  Two things it deliberately does not do: enable anything in `IRPT_EN`
  outside a wait (a level source nobody services is a hang on this
  controller, so each wait opens only the bits it parks on and closes
  them again), and impose its own timeout beyond the blocking path's
  one-second backstop — wrap the future in the executor's own. BCM2836/7
  only for now: routing the line needs `lic`, which BCM2711 has no
  equivalent of yet.

- **Non-blocking DMA transfers to and from a peripheral FIFO**:
  `Channel::start_from_peripheral` and `Channel::start_to_peripheral`,
  which start the transfers `copy_from_peripheral`/`copy_to_peripheral`
  block on and hand back a `Transfer` guard instead, so a caller can wait
  on something better than a polling loop. The read side defers its cache
  invalidate to the guard's drop, which is the first point at which the
  engine is known to have finished.

- **GPIO internal pull resistors.** `gpio::Pull`, `Pin::set_pull`, and
  `Pin::into_pull_up_input`/`into_pull_down_input`/`into_floating_input`
  configure a pin's internal pull-up/pull-down — previously unreachable
  from outside the crate, so a consumer wiring a button or an
  open-collector sensor had to add an external resistor or poke
  `GPPUD` themselves. `Pin::pull` reads the setting back, on `bcm2711`
  only: the legacy `GPPUD`/`GPPUDCLK` pair clocks a value into a pin
  without storing it anywhere readable. `examples/gpio_pull.rs` checks
  both resistors against an unconnected pin, and
  `examples/gpio_irq_button.rs` now uses the internal pull-down instead
  of asking for a 10k resistor.

  The two SoCs use unrelated registers here — the legacy
  `GPPUD`/`GPPUDCLK` clock-in sequence versus BCM2711's
  `GPIO_PUP_PDN_CNTRL_REG0..3`, with *different encodings* of the pull
  value — and four drivers (`uart`, `mini_uart`, `sd`, `sdio`) each
  carried their own copy of the sequence for their own pins. They now all
  route through the one implementation in `src/gpio.rs`, which is the
  only place that knows which scheme applies.

- **`resident-fat` feature**: `sd::SdBlockDevice`, an adapter implementing
  `resident-fat`'s `BlockDevice` trait over the SD driver, with
  `sd::SdBlockDeviceError` for its errors.
  `examples/sd_resident_fat_read.rs` mounts the boot partition and reads
  files.

  Alongside the `embedded-sdmmc` adapter rather than replacing it: the two
  traits differ in their unit of transfer, and which one suits depends on
  the filesystem above. `resident-fat` transfers a plain `&[u8]` spanning a
  whole run of consecutive blocks, which is already what the driver's
  multi-block path takes, so the adapter splits the caller's buffer with
  `as_chunks` and hands the pieces over — no staging buffer, no copy, and
  `max_transfer_blocks` is the controller's real 65535 rather than a
  buffer's size. Reaching `resident-fat` through its own `embedded-sdmmc`
  bridge and `sd::SdCard` still works, and remains the right route for a
  consumer already invested in that trait.

  Unlike every other feature here, this one carries an allocator
  requirement: `resident-fat` uses `alloc`, so a binary that enables it
  must register a `#[global_allocator]`. This crate still neither defines
  nor needs one.
- `Pwm::MAX_CLOCK_DIVISOR` and `Pcm::MAX_CLOCK_DIVISOR`, so a caller can
  check its own constant at compile time rather than discovering the limit as
  a peripheral running at an inexplicable rate.
- `Pwm::clock_hz` and `Pcm::clock_hz`, reporting the rate a divisor will
  actually produce. They apply the same clamp `init` does, so they describe
  the hardware rather than echoing the request back; logging one beside the
  intended rate is how an out-of-range divisor becomes visible.
- `Pwm::MIN_CLOCK_HZ` and `Pcm::MIN_CLOCK_HZ`, the floor the 12-bit divisor
  imposes — roughly 122 kHz, which is a real design constraint and not a
  rounding concern.
- `Pwm::divisor_for`, picking a divisor from a target clock rate. The
  counterpart to `audio_clock_divisor` for callers not on the audio path,
  where computing `500_000_000 / target` by hand is exactly where an
  out-of-range divisor comes from.

## [0.3.0] - 2026-08-30

### Added

- **HDMI audio** (`mmal` feature): `audio_render::AudioRenderer`, audio out
  through the firmware's `ril.audio_render` component —
  `Destination::Hdmi`, or `Destination::Local` for the 3.5 mm jack.
  Interleaved signed-16-bit samples in, paced by the renderer itself rather
  than by any timer on this side. `examples/hdmi_audio.rs` plays a stereo
  tone.
- `mmal::AudioFormat`, and `mmal::PortInfo::audio` alongside the existing
  `video`: the two are the same bytes read two ways, since the message
  carries one type-specific union, and `port_info_set` now writes whichever
  the port's `es_type` names.
- `mmal::parameter_set_string` and `mmal::ENCODING_PCM_SIGNED_LE`, the
  parameter shape and the encoding the audio renderer is configured with.

- **Async LAN9514** (`async` feature): `send_frame_async`,
  `receive_frame_async`, `start_async`, `is_link_up_async` and the
  register accessors behind them, as twins of the blocking methods.
  `receive_frame_async` differs from its twin in more than spelling: it
  leaves the bulk IN parked on an empty receive FIFO rather than first
  asking `RX_FIFO_INF` whether a frame is waiting, so the receive becomes
  interrupt-driven instead of polled. The blocking method cannot do that
  — the DWC2 retries a NAK'd bulk transfer in hardware without halting
  the channel, so it would spin out its whole transfer timeout on every
  idle poll — which is why the pre-check stays there and only there.
- `usb::lan9514::Lan9514::split` (`async` feature), returning
  `Lan9514Rx`/`Lan9514Tx`: the two bulk endpoints borrowed apart, so a
  receive can stay parked on one host channel while transmits go out on
  another. Without it a transmit could only happen by cancelling a parked
  receive, which loses any frame the chip was part-way through handing
  over.
- `usb::control::vendor_in_async` / `vendor_out_async`, the vendor
  register access those methods are built on.
- `usb::lan9514::MTU` is now unconditional rather than gated on an
  adapter feature — it is a property of Ethernet and of this chip, and an
  out-of-crate adapter needs the same number.
- **Async I2C** (`async` feature): `embedded_hal_async::i2c::I2c` on the
  same `I2c` type, parking on the controller's `DONE`/`TXW`/`RXR`
  interrupts rather than polling `S`, so the millisecond a six-byte read
  at 100kHz costs goes to the executor instead of a spin loop. With it,
  `i2c::on_irq` and `Lic::enable_i2c_irq`/`disable_i2c_irq`/
  `is_i2c_pending`. BSC0 and BSC1 share one interrupt line, so the
  handler checks both controllers, and leaves alone any that a blocking
  transfer is driving (it arms none of these conditions).

  Timeouts are the caller's here rather than the driver's: wrap the
  future in `embassy_time::with_timeout` or equivalent. Cancelling one
  that way is safe — the drop masks the interrupts, clears the FIFOs and
  cleans the status, so the next transfer starts from a known state. The
  stored `Timer` deadline still applies as a backstop, but only where the
  future is polled at all, which the module docs spell out.
- **`examples/soc_temperature.rs`**, printing die temperature, ARM clock
  and throttling status together once a second. No new API —
  `Mailbox::temperature_millicelsius` and `Mailbox::throttled` have been
  there all along, and a consumer asking for a way to read the CPU
  temperature is what showed they could not be found. The README's
  mailbox entry now names them too.
- **`i2c::divider_for` and `spi::divider_for`**: `(core_hz, target_hz)`
  to the raw `CDIV` those drivers' `init` takes. Every consumer was
  writing the same arithmetic and getting the same chance to be wrong,
  the reset default of 1500 being documented as 100kHz against a nominal
  150MHz core clock and actually being 166kHz on a board running
  250MHz. Rounding is upwards in both, so the bus never clocks faster
  than asked — what a device states is a maximum, and erring the other
  way fails intermittently rather than visibly.

  `core_hz` is still the caller's to fetch (`Mailbox::clock_rate_hz`
  with `ClockId::Core`) rather than something `init` queries: it can
  fail, it costs a round trip to the GPU, and an application bringing up
  several buses should ask once.
- **`i2c::I2c::<BSC0>::init_id`**: BSC0 on its GPIO0/1 (ALT0) routing —
  `ID_SD`/`ID_SC` on header pins 27/28, the HAT ID EEPROM bus — beside the
  existing `init`, which stays on GPIO44/45. One controller, two
  electrically separate buses, so the routing is a constructor rather than
  an argument, and only one of them can be live at a time. Previously the
  ID bus was unreachable from this crate, which put any board-identity or
  per-unit calibration part sitting on it out of reach too.
  `examples/i2c_hat_eeprom.rs` reads a HAT EEPROM's vendor info atom over
  it.
- **`stack`** (`rt` feature): `stack::headroom`, `used`, `pointer`,
  `bottom`, `top` and `size` — how much of the main stack is left, from
  inside the running program. `headroom`/`used` are `Option` because a
  secondary core runs on its own `multicore::Stack` and the AArch32
  exception modes on their own banked regions, where the question has no
  meaningful answer.

### Removed

- **The `embassy-net-driver` feature**, with `usb::lan9514::Lan9514Driver`
  and `usb::lan9514::wake_rx`. The `embassy-net` adapter now lives in the
  `rpi-hal-embassy` crate, built on `Lan9514::split` and the async methods
  above, and is a `Driver` plus a runner task rather than a `Driver` that
  does its own USB work. `embassy_net_driver::Driver` is synchronous, so
  an adapter shaped that way could never have awaited anything.

  `wake_rx` goes with it, and that is the point of the exercise: an
  application no longer has to poll the driver on a ticker and guess an
  interval, because there is now a real event to wake on.

  Nothing here affects the `smoltcp` adapter or the blocking frame
  methods. `smoltcp`'s `phy::Device` is synchronous by construction, so
  those stay exactly as they were.

### Changed

- **The stack is a reserved region with a stated size**, rather than
  whatever happened to sit below the load address. The linker scripts
  reserve `__stack_size` (1 MiB), `__stack_slack` (2 MiB of margin below
  it), and on AArch32 `__irq_stack_size` (64 KiB) plus
  `__abt_stack_size`/`__und_stack_size`/`__fiq_stack_size` (32 KiB each);
  the boot code points each `sp` at its own region. Any of them can be
  changed without editing the script, via
  `-Wl,--defsym=__stack_size=0x400000` in the consumer's own flags. The
  region is `NOLOAD`, so none of it costs image bytes.

  Programs that supply their own linker script *and* use the `rt` feature
  must define `__stack_top` (and, on AArch32, `__irq_stack_top`,
  `__abt_stack_top`, `__und_stack_top`, `__fiq_stack_top`); the link
  fails loudly naming the missing symbol otherwise. A program using the
  crate's `rpi-link.x` needs no changes.
- **`__unhandled_exception` is now weak** on both architectures, so an
  application can define its own and report a fault instead of parking
  silently. The crate's default (a `wfe` loop) is unchanged when nothing
  overrides it.
- **`i2c::I2c` gained a lifetime and `init` a parameter**: both
  `I2c::<BSC1>::init` and `I2c::<BSC0>::init` now take a `&Timer`, which
  the driver stores as `I2c<'_, I>`. The timer bounds every transfer (see
  Fixed, below); it has to be stored rather than passed per call because
  transfers are reached through `embedded_hal::i2c::I2c::transaction`,
  whose signature this crate doesn't control.
- `i2c::Error` gained `Timeout` and `Incomplete { received, requested }`,
  so it is no longer exhaustively matchable on the two previous variants.
  Both map to `ErrorKind::Other` — `embedded-hal` 1.0 has no closer
  variant, since its `Overrun` means the receive buffer was overrun.
- **A clock-stretch timeout (`S.CLKT`) is now reported**, as
  `Error::Timeout`, by the blocking transfers as well as the new async
  ones — a slave that held SCL past the `CLKT` allowance produced a
  transfer the hardware cut short, and returning its bytes as if nothing
  had happened was wrong. `CLKT` is also cleared alongside `DONE`/`ERR`
  now: it latches, so one uncleared timeout would have been read as a
  fault by every transfer after it, on a bus that had recovered.

### Fixed

- **The IRQ stack no longer sits inside the main stack.** It was set to
  `_start - IRQ_STACK_SIZE`, 4 KiB into the region main mode was growing
  down through, so any main-mode frame deeper than 4 KiB occupied memory
  the first interrupt would push onto — the opposite of what the comment
  there claimed. The two are now adjacent reserved regions.
- **The stack no longer grows down through low memory**, where the
  firmware leaves the ATAGs and, on AArch64, the armstub8 spin table that
  `multicore` starts cores 1-3 through.
- **An I2C transfer can no longer hang the program.** Both transfer loops
  polled `S` with no exit but `ERR` or `DONE`, and a slave that
  acknowledges and then stops driving — one stretching the clock
  indefinitely, a half-soldered part, a line held low — sets neither. The
  loop was then infinite, and since this is a blocking driver it took
  whatever else the program had to do with it: an executor, a network
  stack, everything. Transfers are now bounded against the System Timer
  (a fixed allowance plus a per-byte one) and report `Error::Timeout`.
- **A short read no longer spins forever.** `read_one` waited for `DONE`
  *and* a full buffer, so a transfer that completed having delivered
  fewer bytes than `DLEN` asked for was waiting on a condition that had
  already become unreachable. That case is now `Error::Incomplete`, which
  carries both counts — how many bytes arrived is what says whether a
  device is mute, truncating, or was simply over-read.
- After either failure the controller is returned to a usable baseline
  (FIFOs and status cleared) so a subsequent transfer starts from a known
  state. Best-effort by necessity: the BSC has no documented abort and
  owns the pins while enabled, so nothing here can walk a slave off a bus
  it is still holding — that transfer times out too, which is survivable
  where a hang wasn't.

## [0.2.0] - 2026-08-19

### Added

- **VCHIQ** (`vchiq` feature, implies `mmu`): the VideoCore firmware's
  shared-memory message transport — slot ring, service open/close,
  messages, and page-list bulk DMA — as `vchiq::Vchiq`. Polled rather than
  interrupt-driven; see `vchiq::Vchiq::poll`.
- **MMAL** (`mmal` feature, implies `vchiq`): `mmal::Mmal`, a client for
  the firmware's multimedia framework — components, ports, parameters and
  buffer exchange, with buffers moving by `&'static mut [u8]` ownership
  transfer in both directions.
- **Hardware H.264 decode** (`mmal` feature): `video_decode::VideoDecoder`,
  driving the firmware's `ril.video_decode` component. Takes an H.264
  Annex B byte stream in arbitrary chunks and returns whole I420 frames,
  handling the mid-stream format change the decoder announces once it has
  parsed the stream's geometry. `examples/h264_decode.rs` plays a file off
  the SD card on the display, converting each frame to RGB on the ARM.
- `mmu::set_uncached`, which remaps a granule-aligned region of RAM as
  Normal Non-cacheable — what makes a shared-memory protocol with a second
  bus master possible at all, and why `vchiq` implies `mmu`. The `mmu`
  module is public for it.
- `mailbox::Mailbox::vchiq_init`, the property tag that hands the firmware
  the VCHIQ shared region.
- `vchiq::Stats` and `mmal::Stats`: counts of what has crossed each
  interface. A stalled shared-memory exchange reports nothing about
  itself, so comparing what was sent against what came back is what makes
  one diagnosable.
- crates.io version and docs.rs badges in `README.md`, alongside CI.
- **USB host channels as owned handles**: `usb::dwc2::Dwc2Host::alloc_channel`
  hands out a `usb::dwc2::Channel`, and every transfer primitive now lives
  on that rather than on the controller. A channel carries its own DMA
  staging buffer and borrows the controller immutably, so several can be
  outstanding at once and two endpoints can be driven independently —
  where before one `&mut Dwc2Host` and one shared buffer meant one
  transfer at a time, on a channel index every caller hardcoded to 0.
  Exhaustion is reported (`None`, and `EnumerationError::OutOfChannels`)
  rather than queued.
- **Interrupt-driven USB** (`async` feature): `usb::dwc2::asynch`, with an
  `_async` twin of each `Channel` transfer primitive plus
  `Dwc2Host::wait_for_port_change` and `Channel::wait_microframes`, all
  serviced by `usb::dwc2::on_irq` from the application's `__irq_handler`.
  No time crate is involved: the channel-halt interrupt reports
  completion and start-of-frame supplies the microframe scheduling a
  periodic split needs, so the whole path is bus-clocked. An async
  transfer has no timeout of its own — dropping the future imposes one
  and aborts the channel. `examples/usb_irq.rs` drives it with a
  hand-rolled `block_on`, no executor.
- `lic::Lic::enable_usb_irq`/`disable_usb_irq`/`is_usb_pending`, routing
  the DWC2 controller's line to the ARM core.
- `usb::lan9514::Lan9514::from_endpoint`, for a LAN9514 that an external
  host stack has already addressed and configured — the counterpart to
  `from_device` when this crate's `enumerate` isn't the thing walking the
  bus.
- `cpu::core_id`, the calling core's id from `MPIDR`. Deliberately not part
  of `multicore`, which is compiled only behind its own feature: code that
  runs on every core — an interrupt handler, a panic handler naming where
  it died — needs to ask this without opting into the machinery for
  *starting* cores, and `generic_timer` already needed it to address a
  per-core register on a single-core build.

### Changed

- `usb::enumerate` takes `&Dwc2Host` rather than `&mut`, and its callback
  receives a `&mut Channel` in place of the controller. A callback that
  needs a channel outliving enumeration captures the same `&Dwc2Host` and
  allocates its own.
- `usb::lan9514::Lan9514Phy::new` and `Lan9514Driver::new` take an owned
  `Channel` instead of `&mut Dwc2Host`, so the rest of the controller's
  channels stay free while a network stack runs.
- Every `usb::control`, `usb::hub` and `usb::hid` entry point takes
  `&mut Channel` in place of `&mut Dwc2Host`.
- `Dwc2Host::last_channel_interrupt` is now `Channel::last_interrupt`,
  reporting per channel rather than per controller.
- `Dwc2Host::init` leaves `GINTMSK.SOFM` masked. It was set
  unconditionally and harmless only because nothing routed USB to the ARM
  core; now that `Lic::enable_usb_irq` exists, an unmasked 8kHz level
  source with nothing to acknowledge it would be a hang rather than
  merely wasted cycles. The async path unmasks it only while a channel is
  waiting on a microframe.
- A channel start programs `HCINTMSK` to `CHH` alone instead of every
  condition, so "this channel raised `HAINT`" means "it halted". The
  `HCINT` bits themselves still latch, so nothing that reads them at the
  halt — including the split logic's `ACK` check — sees any difference.

### Fixed

- The README's CI badge no longer renders as a broken image on crates.io.
  GitHub refuses to serve its own `actions/workflows/…/badge.svg` to pages
  on another origin — browsers report `ERR_HTTP2_SERVER_REFUSED_STREAM` —
  so the badge worked on GitHub and nowhere else. It now comes from
  shields.io, which serves the same status to both. crates.io renders a
  README once, at publish time, so 0.1.0's page keeps the broken badge and
  the fix is visible only from this version on.

## [0.1.0] - 2026-08-12

First release, so there is nothing to diff against: the entry below is
what the crate covers rather than what changed in it. Later entries will
be actual changes against this baseline. `README.md` documents each item
in detail, and the [issue tracker](https://github.com/joeferner/rpi-hal/issues)
has what is deliberately not here yet.

### Added

- **Boot runtime** (`rt`, on by default): `_start`, `.bss` zeroing,
  exception vectors, and a `critical-section` implementation, for both
  `armv7a-none-eabi` (32-bit, `kernel7.img`) and
  `aarch64-unknown-none-softfloat` (64-bit, `kernel8.img`). A consumer
  that owns its own boot sequence turns the feature off.
- **Linker script**, published on the linker search path as `rpi-link.x`
  and already carrying the load address for the target being built, so a
  downstream binary names it with one `-T` line and keeps no copy of its
  own.
- **Identity-mapped MMU** (`mmu`, on by default), which is what makes
  `core::sync::atomic` work at all: `ldrex`/`strex` are architecturally
  UNPREDICTABLE against the Device memory every address defaults to with
  translation off. A consumer can supply its own table instead.
- **Secondary cores** (`multicore`), plus the cross-core spinlock that
  becomes necessary once one is running.
- **Chip selection**, exactly one of `bcm2837` (Pi 2, Pi 3) or `bcm2711`
  (Pi 4) — neither is a default, since there is no sensible default target
  chip. `bcm2711` is preliminary: boot, GPIO, the System Timer, UART and
  SD via EMMC2 are verified on hardware, most other drivers are untested,
  and there is no interrupt controller for it yet, so nothing
  interrupt-driven is available under it.
- **Peripheral drivers**, with `embedded-hal` 1.0 and `embedded-io` trait
  implementations alongside inherent methods where the traits fit:
  - GPIO (typestated pins, edge/level interrupts), UART0, the mini UART,
    SPI0, the AUX block's SPI1/SPI2, and I2C on BSC0/BSC1.
  - The System Timer, the per-core ARM generic timer, the watchdog, and
    reboot/shutdown through the PM block.
  - The VideoCore mailbox property interface, including a scanout
    framebuffer with page flipping, and the FT5406 touch panel.
  - SD/MMC over the Arasan EMMC controller (and BCM2711's separate EMMC2),
    DMA, PWM and PCM/I2S audio out, the hardware RNG, and the performance
    monitor unit.
  - USB: the DWC2 host controller, the LAN9514 Ethernet chip, and HID
    keyboard/mouse/gamepad devices over a shared report-descriptor parser.
  - Pi 3 on-board radio: Wi-Fi over SDIO (firmware and CLM blob load,
    scanning, WPA2, and a full TCP/IP path), and Bluetooth over HCI — BLE
    advertising, scanning, connections, L2CAP, an ATT/GATT server and
    client, SMP pairing with bonding, and HID over GATT.
  - Camera: the Unicam1 CSI-2 receiver with an OV5647 sensor driver.
- **3D graphics** (`v3d`, Pi 3 only): VideoCore IV's binning/render
  pipeline and control-list builders, enough for a depth-tested, textured
  draw per frame.
- **Interrupts**: CPU-level masking, the legacy interrupt controller, and
  the `__irq_handler` contract that makes dispatch the application's — see
  `README.md`, because a missing or incomplete handler is the one mistake
  here that produces no error message at all.
- **Async** (`async`): `embedded-hal-async`'s `digital::Wait` on input
  pins and `embedded-io-async`'s `Read`/`Write` on `Uart`, as plain
  `poll`/`Waker` code with no executor dependency — see the
  `rpi-hal-embassy` crate for an executor and time driver built on it.
- **Integration adapters**, each behind a feature of the same name:
  `embedded-sdmmc` (`BlockDevice` over SD), `smoltcp` (`phy::Device` over
  Ethernet and Wi-Fi), and `embassy-net-driver` (`Driver` over Ethernet).
- Around 75 examples, each of which runs on real hardware rather than in a
  simulator.

### Notes

- Requires stable Rust **1.88** or newer — `#[unsafe(naked)]` and
  `naked_asm!` in the FPU bring-up are the floor. Enabling `smoltcp`
  raises it to 1.91, which is smoltcp's own requirement, not this crate's.
  Nightly is not needed.
- Licensed under either MIT or Apache-2.0, at your option.

[0.7.0]: https://github.com/joeferner/rpi-hal/releases/tag/v0.7.0
[0.6.0]: https://github.com/joeferner/rpi-hal/releases/tag/v0.6.0
[0.5.0]: https://github.com/joeferner/rpi-hal/releases/tag/v0.5.0
[0.4.0]: https://github.com/joeferner/rpi-hal/releases/tag/v0.4.0
[0.3.0]: https://github.com/joeferner/rpi-hal/releases/tag/v0.3.0
[0.2.0]: https://github.com/joeferner/rpi-hal/releases/tag/v0.2.0
[0.1.0]: https://github.com/joeferner/rpi-hal/releases/tag/v0.1.0

# V3D capture tooling

Ground truth for the `v3d` feature — `src/v3d.rs` and the control-list
builders under `src/v3d/`.

## Why this exists

The `v3d` driver has no shader assembler. The QPU machine code in
`examples/gpu_cube.rs` was **extracted from Mesa's `vc4` compiler**,
by running a GLES program on a Pi 3 under Debian with `VC4_DEBUG` set
and pulling the disassembled instruction words out of its output. The
control-list packet layouts and field values were confirmed the same
way, against Mesa's own annotated dumps.

So the bytes in that example are only valid for the exact GLSL the
programs here compile. **Changing what the shaders do means recapturing
them**, and this directory is the only way to do that.

Keeping the captures alongside the programs matters as much as the
programs themselves: several driver values are only justifiable by
pointing at a specific line of a specific dump, and re-deriving them
from scratch is expensive. Three separate bugs during bring-up came
from extrapolating beyond what a capture actually covered rather than
capturing the case in question.

## Requirements

A Pi 3 (BCM2836/BCM2837) running Debian/Raspberry Pi OS, with the Mesa
`vc4` driver — not the closed firmware stack, and not `llvmpipe`. Each
program prints `GL_RENDERER` on startup; it should name VideoCore IV.
`llvmpipe` means the render node wasn't reached (check
`/dev/dri/renderD128` exists and that `groups` includes `render`).

```sh
sudo apt install build-essential pkg-config libgbm-dev libegl-dev \
    libgles2-mesa-dev libdrm-dev
```

Both programs are headless — they render into a GBM buffer via EGL and
never touch a display, so they work fine over SSH.

## The programs

### `cube_reference.c` — the reference renderer

Draws the *same scene* as `examples/gpu_cube.rs`: identical 24 vertices
and 36 indices, identical 4x4 checkerboard, identical matrices,
512x512, depth-tested. Two distinct uses:

```sh
gcc -O2 -o cube_reference cube_reference.c -lm \
    $(pkg-config --cflags --libs gbm egl glesv2)

# As a picture: writes cube_000.ppm ... cube_090.ppm, 15 degrees apart.
./cube_reference

# As a capture. One frame only, or the dump is enormous and every
# draw looks alike.
VC4_DEBUG=qpu,qir,shaderdb,cl ./cube_reference 1 2> captures/cube_dump.log
```

As a picture it settles "is the bare-metal renderer wrong, or is the
scene wrong?" — if Mesa draws it correctly and the bare-metal version
doesn't, geometry, texture coordinates, matrix math and shaders are all
exonerated and the fault is in the hand-built control lists.

As a capture it is the authority on anything depth-related, because it
is the only capture here whose draw actually tests depth.

`.ppm` files open in most viewers, or `pnmtopng cube_000.ppm > x.png`.
Note that some viewers render these with the channels misordered — the
background is `(32, 64, 128)`, a dark blue; if it looks green, that's
the viewer.

### `shader_dump.c` — minimal shader capture

One flat triangle at 4x4, no depth testing. Simpler and faster to read
than `cube_reference.c`, but **its draw tests depth as `ALWAYS`**,
which makes it actively misleading for depth questions: Mesa compiles a
*different* fragment shader when depth testing is on (one extra
`mov tlb_z`), and its `Configuration Bits` differ too. Use
`cube_reference.c` for anything involving depth.

```sh
gcc -o shader_dump shader_dump.c $(pkg-config --cflags --libs gbm egl glesv2)
VC4_DEBUG=qpu,shaderdb,cl ./shader_dump 2> captures/dump.log
```

### `ground_truth.sh` — hardware facts

Prints the V3D register base (from `/proc/iomem` and the device tree)
and the `ENABLE_QPU` mailbox tag (from kernel headers). Confirms the
constants in `src/v3d.rs` against a running system rather than trusting
documentation. Some sections may come up empty and say what to install.

```sh
sh ground_truth.sh
```

## The captures

| file | draw | what the driver takes from it |
| --- | --- | --- |
| `captures/dump.log` | flat triangle, `glDrawArrays`, no depth | `COORD_SHADER` and `VERTEX_SHADER` bytes |
| `captures/dump_indexed.log` | same, `glDrawElements` | `Indexed Primitive List` field values |
| `captures/dump_qir.log` | same, with QIR | uniform stream order for both shader stages |
| `captures/cube_dump.log` | the real cube, depth-tested, 8x8 tiles | `FRAGMENT_SHADER` bytes, depth `Configuration Bits`, 8x8 tile config |

`dump_qir.log` is the one to reach for when uniforms are in question.
It contains both the pre-scheduling QIR (which tags each uniform read
with an index) *and* the scheduled QPU listing (which shows the order
reads are actually issued in). Those are different, and only the
scheduled listing determines the stream order — see
`src/v3d/uniforms.rs`, which got this wrong twice before settling it.

## `vc4_packet.xml`

Copied from Mesa's `src/broadcom/cle/vc4_packet.xml` — the
machine-readable packet spec its CL decoder reads, and the source for
every opcode, size and bit offset in `src/v3d/bcl.rs` and
`src/v3d/rcl.rs`. Kept here so those modules can be checked without a
Mesa checkout.

It contains one real error, worth knowing before trusting it: "Vertex
Shader Uniforms Address" is declared at the same byte offset as "Vertex
Shader Code Address" (`start="16b"` for both), which cannot be right.
`src/v3d/shader_record.rs` implements the pattern-consistent offset
(byte 20) instead, and explains why.

Broadcom's *VideoCore IV 3D Architecture Reference Guide*
(`docs.broadcom.com/doc/12358545`) is the other primary reference — the
only official register-level documentation for this block, and the
source for the `V3D_*` register offsets, `CTnCS` bit meanings and
performance-counter IDs. Not vendored here; it is a public PDF.

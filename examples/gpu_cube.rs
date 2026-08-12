//! Tumbling textured cube, drawn by the V3D 3D pipeline — the whole of
//! `rpi_hal::v3d` exercised end to end: `Mailbox::set_clock_rate_hz`
//! and `set_enable_qpu` to bring the block up, `v3d::GpuBuffer` for
//! everything V3D reads or writes over the bus, `v3d::bcl`/`v3d::rcl`
//! for the binning and render control lists, `v3d::shader_record` and
//! the raw QPU instruction bytes below, `v3d::texture`, and
//! `V3d::submit_bin`/`submit_render` per frame.
//!
//! Cube geometry (24 vertices — 4 per face, not 8 shared corners) and
//! the rotation/projection matrix math live here, not in `rpi-hal`
//! itself — chip-agnostic demo logic on top of the HAL, the same split
//! IR protocol decode and FAT filesystem logic already have.
//!
//! ## Things that look like bugs but aren't
//!
//! - **Texels are authored `B, G, R, A`.** The framebuffer's byte 0 is
//!   R, for both the clear color and scanout, but the texture path
//!   exchanges red and blue between the TMU and the fragment shader's
//!   tile-buffer write. Measured, not assumed — see `checkerboard`.
//! - **Culling is off**, so a wrong winding-convention assumption
//!   can't silently hide faces; occlusion comes from the depth test
//!   alone (see `bcl.rs`'s `configuration_bits`).
//! - **The render fills a square, not the screen.** The framebuffer is
//!   `512x512`; the firmware pillarboxes it on a widescreen display.
//!   Both dimensions must stay multiples of 64 — V3D renders whole
//!   tiles, so a non-multiple would have it write past the
//!   framebuffer's real width.
//!
//! ## Still a guess
//!
//! - The fragment shader's `Number of Varyings` field
//!   (`shader_record::ShaderRecordParams::fragment_shader_num_varyings`)
//!   is `2` (one per texcoord component) — a guess between counting
//!   components and counting slots, per that field's own doc comment.
//!   It renders correctly at `2`, which is evidence but not proof: a
//!   single `vec2` varying can't distinguish the two readings.

#![no_std]
#![no_main]

use core::fmt::Write;

use rpi_hal::halt;
use rpi_hal::mailbox::{ClockId, Mailbox, PixelOrder};
use rpi_hal::pac;
use rpi_hal::uart::Uart;
use rpi_hal::v3d::{self, bcl, rcl, shader_record, texture, uniforms, GpuBuffer, V3d};

// ---- Render target configuration ----

/// Render target width, in pixels. An exact multiple of the 64-pixel
/// tile size, deliberately — see [`WIDTH_IN_TILES`]'s doc comment.
///
/// `512` is `8x8` tiles. During bring-up this was temporarily `64`
/// (exactly one tile) to test whether multi-tile binning was why the
/// binner produced nothing; it wasn't, and the real cause turned out
/// to be elsewhere entirely, so the shrink is no longer needed. A
/// 64-pixel target also scales up to a display blockily enough to be
/// hard to look at.
const WIDTH_PX: u16 = 512;
/// Render target height, in pixels. Same exact-tile-multiple reasoning
/// as [`WIDTH_PX`]; kept square so the perspective projection's aspect
/// ratio stays `1.0`.
const HEIGHT_PX: u16 = 512;
/// Render target width, in 64-pixel tiles. Chosen as an exact multiple
/// of 64 specifically to avoid a real risk: V3D's fixed-function tile
/// rendering always processes whole tiles, so a non-multiple size
/// would have the renderer write pixel data for rows/columns past the
/// requested width/height — memory the framebuffer might not actually
/// have allocated past its tightly-sized real dimensions. An exact
/// multiple sidesteps needing to reason about that at all.
const WIDTH_IN_TILES: u8 = (WIDTH_PX / 64) as u8;
/// Render target height, in 64-pixel tiles — see [`WIDTH_IN_TILES`].
const HEIGHT_IN_TILES: u8 = (HEIGHT_PX / 64) as u8;

/// Whether to run a real depth test, or the color-only,
/// always-passing configuration — see
/// [`bcl::BclParams::depth_test_enabled`].
///
/// A solid cube needs this: with the test always passing, whichever
/// face was submitted last wins each pixel regardless of how far away
/// it is, so the far side of the cube draws over the near side. Since
/// culling is deliberately left off (see this file's "known open
/// risks"), depth is the only thing providing occlusion here. It was
/// temporarily disabled during bring-up, when it was a suspect for a
/// hang — it wasn't the cause.
///
/// Running with it off measured `quads_failed_z = 0` and
/// `quads_written = 27534` — every fragment drawn, none rejected —
/// which is exactly the "parts of the cube appearing and disappearing
/// as it turns" that no occlusion produces.
const DEPTH_TEST_ENABLED: bool = true;

/// Whether to also write the depth buffer out to memory each frame,
/// separately from testing against it — see
/// [`rcl::RclParams::depth_write_address`].
///
/// Back to `true`. The reasoning for turning it off was that depth
/// testing should work purely within the tile buffer's own Z storage,
/// making a store to memory pointless when nothing reads it back. The
/// hardware disagrees: with the store off, nothing occluded at all —
/// the cube painted strictly in submission order, the last face drawn
/// winning every pixel regardless of distance. Declaring the Z surface
/// in the render control list appears to be what gives the render pass
/// a depth buffer to test against in the first place.
///
/// The one earlier run with the store enabled rejected 86% of
/// fragments, which looked like evidence against it — but that run
/// also had the vertex shader's X and Y uniforms swapped, so its depth
/// values were meaningless. That confound is gone now.
const DEPTH_STORE_ENABLED: bool = true;

/// Spin the cube about the Y axis only, instead of tumbling about X
/// and Y together.
///
/// `false` now that the pipeline renders correctly — a tumble is the
/// more interesting demo. Kept as a switch because of what it was for:
/// with two axes turning at once there is no simple expected
/// appearance to check against, so "it looks wrong" is hard to turn
/// into anything specific. A single-axis spin does have one — each
/// face sweeps across, narrows to a vertical line edge-on, and widens
/// again every quarter turn — which is what made the remaining faults
/// describable, and ultimately what identified them.
const SINGLE_AXIS_ROTATION: bool = false;

// ---- Buffer sizes ----

/// Tile state data array size — [`bcl::tile_state_size`]'s real,
/// kernel-sourced formula, not a guess.
const TILE_STATE_SIZE: usize = bcl::tile_state_size(WIDTH_IN_TILES, HEIGHT_IN_TILES) as usize;
/// Tile allocation memory size. No confirmed formula for this one (see
/// [`bcl::BclParams::tile_alloc_address`]'s doc comment), so it is
/// scaled per tile with deliberate slack rather than tuned.
///
/// What the binner actually consumes was measured with `V3D_BPCA`:
/// about 14KB total for this 12-triangle cube across all 64 tiles at
/// `512x512` — and about 8KB of that for a *single* tile at `64x64`,
/// so the cost is mostly fixed overhead (the pipeline state copied
/// into each tile's list) rather than something that scales steeply
/// with tile count. 16KB per tile is far more than needed; it is
/// sized this way so raising the resolution can never quietly starve
/// it. Running out is not silent either: the binner reports it through
/// `V3D_PCS`'s `BMOOM` bit, which this example checks every frame.
const TILE_ALLOC_SIZE: usize = 16 * 1024 * (WIDTH_IN_TILES as usize * HEIGHT_IN_TILES as usize);
/// Render control list size — [`rcl::size`]'s exact formula for this
/// render target's tile count and [`DEPTH_TEST_ENABLED`].
const RCL_SIZE: usize = rcl::size(WIDTH_IN_TILES, HEIGHT_IN_TILES, DEPTH_STORE_ENABLED);
/// Depth/stencil buffer size: same pixel count as the color target, 4
/// bytes per pixel (`vc4_state.c`'s `cpp = 4` for a Z/stencil surface).
const DEPTH_BUFFER_SIZE: usize = WIDTH_PX as usize * HEIGHT_PX as usize * 4;
/// Texture width, in pixels — a small checkerboard, not a realistic
/// texture, since this demo only needs something visibly textured.
const TEXTURE_WIDTH: usize = 4;
/// Texture height, in pixels.
const TEXTURE_HEIGHT: usize = 4;
/// Texture size in bytes (`GL_RGBA`/`GL_UNSIGNED_BYTE`, tightly
/// packed).
const TEXTURE_SIZE: usize = TEXTURE_WIDTH * TEXTURE_HEIGHT * 4;

// ---- Compiled shader machine code ----
//
// Extracted mechanically (a script parsing the raw hex instruction
// words, not hand-transcription) from Mesa's `vc4` driver compiling
// this file's GLSL on a Pi 3 under `VC4_DEBUG=qpu`. There is no
// assembler here, so these bytes are only valid for those exact
// shaders: changing what the shaders do means recapturing them.
//
// `tools/v3d-capture/` holds the programs that produce these captures
// and the captures themselves, with instructions.
//
// The GLSL they were compiled from:
//
//   vertex:   attribute vec4 aPosition; attribute vec2 aTexCoord;
//             uniform mat4 uMvp; varying vec2 vTexCoord;
//             void main() { gl_Position = uMvp * aPosition;
//                           vTexCoord = aTexCoord; }
//   fragment: varying vec2 vTexCoord; uniform sampler2D uTexture;
//             void main() { gl_FragColor =
//                           texture2D(uTexture, vTexCoord); }

/// A shader's raw QPU machine code, 8-byte aligned to match the QPU's
/// natural instruction size — no confirmed hardware requirement for
/// this specific alignment, just cheap insurance given every other
/// address in this pipeline needs *some* alignment.
#[repr(C, align(8))]
struct Shader<const N: usize>([u8; N]);

/// 40 instructions (320 bytes) — `MESA_SHADER_COORD` in the capture.
/// This is the binning-pass shader: it transforms position only, and
/// its output feeds the binner's tile assignment.
static COORD_SHADER: Shader<320> = Shader([
    0x00, 0x1a, 0x40, 0x00, 0x67, 0x4c, 0x02, 0xe0, 0x80, 0x7d, 0xc2, 0x15, 0xe7, 0x10, 0x02, 0x10,
    0x80, 0x7d, 0xc2, 0x15, 0x27, 0x01, 0x02, 0x10, 0x37, 0x30, 0x80, 0x20, 0xe1, 0x49, 0x00, 0x10,
    0x3e, 0x00, 0x12, 0x20, 0xe0, 0x49, 0x00, 0x10, 0x37, 0x32, 0x80, 0x21, 0x62, 0x51, 0x02, 0x10,
    0x3e, 0x00, 0x12, 0x20, 0xe3, 0x49, 0x00, 0x10, 0x3e, 0x00, 0x12, 0x20, 0xe1, 0x49, 0x00, 0x10,
    0x77, 0x34, 0x80, 0x21, 0xe0, 0x40, 0x02, 0x10, 0xfe, 0x00, 0x12, 0x21, 0x22, 0x51, 0x02, 0x10,
    0x37, 0x30, 0x80, 0x20, 0xe3, 0x49, 0x00, 0x10, 0xb6, 0x76, 0xc2, 0x81, 0x22, 0x50, 0x02, 0x10,
    0x32, 0x70, 0x82, 0x20, 0xe0, 0x49, 0x00, 0x10, 0x32, 0x5e, 0x80, 0x21, 0x63, 0x40, 0x02, 0x10,
    0xf2, 0x4e, 0x80, 0x21, 0xa1, 0x50, 0x02, 0x10, 0x7a, 0x0c, 0x0e, 0x21, 0x62, 0x50, 0x02, 0x10,
    0xb6, 0x0e, 0xc0, 0x81, 0xa2, 0x40, 0x02, 0x10, 0x32, 0x70, 0x82, 0x20, 0xe3, 0x49, 0x00, 0x10,
    0xf2, 0x2e, 0x80, 0x21, 0xe0, 0x48, 0x02, 0x10, 0xf2, 0x76, 0x82, 0x35, 0x21, 0x4d, 0x02, 0x10,
    0x3a, 0x0c, 0x06, 0x21, 0xa2, 0x51, 0x02, 0x10, 0x40, 0x1e, 0x9c, 0x01, 0xa7, 0x01, 0x02, 0x10,
    0x9c, 0x7c, 0x0a, 0x21, 0xe0, 0x41, 0x02, 0x10, 0x00, 0x1a, 0x00, 0x00, 0x67, 0x5c, 0x02, 0xe0,
    0x00, 0x1e, 0x9e, 0x02, 0x67, 0x08, 0x02, 0xd0, 0xe1, 0x6f, 0x9c, 0x35, 0x05, 0x5c, 0x02, 0x10,
    0x80, 0x7d, 0x1a, 0x15, 0x27, 0x0c, 0x02, 0x10, 0x3e, 0x60, 0x80, 0x20, 0xe1, 0x49, 0x00, 0x10,
    0x80, 0x7d, 0x1e, 0x15, 0x27, 0x0c, 0x02, 0x10, 0xce, 0x76, 0x16, 0x35, 0x22, 0x4c, 0x02, 0x10,
    0xb7, 0x04, 0x1a, 0x27, 0x23, 0x40, 0x12, 0x10, 0x1e, 0x70, 0x16, 0x20, 0xe0, 0x49, 0x00, 0x10,
    0x37, 0x00, 0x1e, 0x27, 0x22, 0x40, 0x22, 0x10, 0x16, 0x70, 0x16, 0x20, 0xe3, 0x49, 0x00, 0x10,
    0x80, 0x7d, 0x02, 0x15, 0x27, 0x0c, 0x02, 0x10, 0x80, 0x77, 0x82, 0x01, 0x27, 0x0c, 0x02, 0x10,
    0x80, 0x7d, 0x16, 0x15, 0x27, 0x0c, 0x02, 0x10, 0x00, 0x70, 0x9e, 0x00, 0xe7, 0x09, 0x00, 0x30,
    0x00, 0x70, 0x9e, 0x00, 0xe7, 0x09, 0x00, 0x10, 0x00, 0x70, 0x9e, 0x00, 0xe7, 0x09, 0x00, 0x10,
]);

/// 41 instructions (328 bytes) — `MESA_SHADER_VERTEX` in the capture.
/// The render-pass shader: same transform as the coordinate shader,
/// plus the texcoord varyings the fragment shader interpolates.
static VERTEX_SHADER: Shader<328> = Shader([
    0x00, 0x1a, 0x60, 0x00, 0x67, 0x4c, 0x02, 0xe0, 0x80, 0x7d, 0xc2, 0x15, 0x27, 0x01, 0x02, 0x10,
    0x80, 0x7d, 0xc2, 0x15, 0x27, 0x11, 0x02, 0x10, 0x3e, 0x00, 0x12, 0x20, 0xe0, 0x49, 0x00, 0x10,
    0x37, 0x40, 0x80, 0x20, 0xe1, 0x49, 0x00, 0x10, 0x77, 0x40, 0x80, 0x21, 0xa2, 0x51, 0x02, 0x10,
    0x3e, 0x00, 0x12, 0x20, 0xe1, 0x49, 0x00, 0x10, 0xb7, 0x42, 0x80, 0x21, 0x63, 0x41, 0x02, 0x10,
    0x3e, 0x00, 0x12, 0x20, 0xe2, 0x49, 0x00, 0x10, 0xf7, 0x44, 0x80, 0x21, 0xa0, 0x50, 0x02, 0x10,
    0x3e, 0x00, 0x12, 0x20, 0xe3, 0x49, 0x00, 0x10, 0x36, 0x76, 0xc2, 0x81, 0x60, 0x40, 0x02, 0x10,
    0x30, 0x70, 0x82, 0x20, 0xe1, 0x49, 0x00, 0x10, 0x70, 0x6e, 0x80, 0x21, 0x62, 0x50, 0x02, 0x10,
    0xb8, 0x0c, 0x16, 0x21, 0x63, 0x48, 0x02, 0x10, 0xf0, 0x2e, 0x80, 0x21, 0xe0, 0x40, 0x02, 0x10,
    0x00, 0x7c, 0x06, 0x01, 0xe7, 0x10, 0x02, 0x10, 0x80, 0x7d, 0xc2, 0x15, 0xe7, 0x08, 0x02, 0x10,
    0x33, 0x70, 0x82, 0x20, 0xe0, 0x49, 0x00, 0x10, 0x33, 0x1e, 0x80, 0x21, 0x22, 0x50, 0x02, 0x10,
    0xb3, 0x72, 0x82, 0x21, 0xa1, 0x40, 0x02, 0x10, 0x7f, 0x0c, 0x0c, 0x81, 0x34, 0x48, 0x02, 0x10,
    0x33, 0x70, 0x82, 0x20, 0xe3, 0x49, 0x00, 0x10, 0xc6, 0x3e, 0x80, 0x21, 0x60, 0x51, 0x02, 0x10,
    0x3c, 0x00, 0x9c, 0x20, 0xe1, 0x49, 0x00, 0x10, 0x40, 0x1e, 0x9e, 0x02, 0xa7, 0x08, 0x02, 0xd0,
    0x22, 0x70, 0x9e, 0x20, 0xc6, 0x59, 0x00, 0x10, 0x37, 0x00, 0x0a, 0x20, 0xe2, 0x49, 0x00, 0x10,
    0x16, 0x70, 0x1a, 0x20, 0xe3, 0x49, 0x00, 0x10, 0xc6, 0x76, 0x1a, 0x27, 0x21, 0x40, 0x12, 0x10,
    0x7e, 0x52, 0x80, 0x27, 0x23, 0x40, 0x22, 0x10, 0x00, 0x1a, 0x00, 0x00, 0x67, 0x5c, 0x02, 0xe0,
    0x1e, 0x70, 0x1a, 0x20, 0xe0, 0x49, 0x00, 0x10, 0x80, 0x7d, 0x02, 0x15, 0x27, 0x0c, 0x02, 0x10,
    0xf6, 0x01, 0xc2, 0x81, 0x20, 0x4c, 0x02, 0x10, 0x80, 0x7d, 0x1a, 0x15, 0x27, 0x0c, 0x02, 0x10,
    0x36, 0x70, 0xc2, 0x95, 0x21, 0x4c, 0x02, 0x10, 0x40, 0x72, 0x9e, 0x15, 0x27, 0x0c, 0x02, 0x10,
    0x00, 0x70, 0x9e, 0x00, 0xe7, 0x09, 0x00, 0x30, 0x00, 0x70, 0x9e, 0x00, 0xe7, 0x09, 0x00, 0x10,
    0x00, 0x70, 0x9e, 0x00, 0xe7, 0x09, 0x00, 0x10,
]);

/// 16 instructions (128 bytes) — `MESA_SHADER_FRAGMENT` from a capture
/// of this scene **with depth testing enabled**, which matters.
///
/// Mesa compiles a *different* fragment shader depending on the depth
/// configuration: the depth-tested one carries an extra
/// `mov tlb_z, rb15` immediately before its `mov tlb_color_all`,
/// writing the fragment's Z into the tile buffer. A capture taken with
/// the depth test set to `ALWAYS` has no such instruction, because
/// nothing needs to update Z.
///
/// Using the shorter one with depth testing on is why nothing ever
/// occluded: the depth test ran, but no fragment ever wrote a depth
/// value, so the buffer stayed at its cleared far value and every
/// fragment passed. The cube painted strictly in submission order —
/// the back face drawing over the front purely because it is submitted
/// last, and the first-submitted face never visible at all.
///
/// The coordinate and vertex shaders are byte-identical between the
/// two captures; only this one changes.
static FRAGMENT_SHADER: Shader<128> = Shader([
    0x3e, 0x30, 0x3e, 0x20, 0xe0, 0x49, 0x00, 0x10, 0x7e, 0x31, 0x3e, 0x21, 0xe1, 0x48, 0x02, 0x10,
    0x40, 0x73, 0x9e, 0x01, 0xa7, 0x08, 0x02, 0x60, 0x80, 0x74, 0x9e, 0x15, 0x67, 0x1e, 0x02, 0x10,
    0xc0, 0x76, 0x9e, 0x15, 0x27, 0x1e, 0x02, 0x10, 0x00, 0x70, 0x9e, 0x00, 0xe7, 0x09, 0x00, 0xa0,
    0x00, 0x79, 0x9e, 0x04, 0x67, 0x08, 0x02, 0x1d, 0x09, 0x79, 0x9e, 0x84, 0x21, 0x48, 0x42, 0x1b,
    0x00, 0x79, 0x9e, 0x84, 0xe1, 0x48, 0x52, 0x19, 0x1b, 0x79, 0x9e, 0x84, 0xa1, 0x48, 0x62, 0x1f,
    0x12, 0x70, 0x9e, 0x80, 0xe1, 0x49, 0x70, 0x11, 0xc0, 0xff, 0x9c, 0x15, 0x27, 0x0b, 0x02, 0x10,
    0x40, 0x72, 0x9e, 0x15, 0xa7, 0x0b, 0x02, 0x10, 0x00, 0x70, 0x9e, 0x00, 0xe7, 0x09, 0x00, 0x30,
    0x00, 0x70, 0x9e, 0x00, 0xe7, 0x09, 0x00, 0x10, 0x00, 0x70, 0x9e, 0x00, 0xe7, 0x09, 0x00, 0x50,
]);

// ---- Texture data ----

/// A buffer 4096-byte aligned to satisfy `Tile 0's `OFFSET` field
/// (address `>> 12`) — see `v3d::texture`'s doc comment. Plain `u8`
/// arrays only guarantee 1-byte alignment, so this wrapper exists
/// purely for that one requirement.
#[repr(C, align(4096))]
struct AlignedBuffer<const N: usize>([u8; N]);

/// Give each face of the cube a single flat colour instead of the
/// checkerboard, so a face can be named rather than guessed at.
///
/// Diagnostic, not decorative. With every face carrying the same
/// checkerboard there is no way to say *which* face is drawing in
/// front of which, so descriptions of what is wrong have to be
/// geometric ("the left side", "the middle") and stay ambiguous.
/// Distinct colours make an occlusion fault directly reportable —
/// "cyan is drawing over magenta" names both faces involved.
///
/// The face-to-colour mapping, in the order [`VERTICES`] declares
/// them:
///
/// | face | direction | colour |
/// | --- | --- | --- |
/// | 0 | `+X` | red |
/// | 1 | `-X` | green |
/// | 2 | `+Y` | blue |
/// | 3 | `-Y` | yellow |
/// | 4 | `+Z` | magenta |
/// | 5 | `-Z` | cyan |
///
/// With the camera down `-Z` and no rotation, `+Z` (magenta) faces the
/// viewer, `+Y` (blue) is up, and `+X` (red) is to the right.
const FACE_COLORS: bool = true;

/// The texel each face samples when [`FACE_COLORS`] is set: face `f`
/// takes the centre of texel `(f % 4, f / 4)`, so all four of its
/// vertices sample one solid colour. Centres rather than corners
/// because `GL_NEAREST` filtering must land unambiguously inside the
/// intended texel rather than on a boundary between two.
fn face_texcoord(face: usize) -> (f32, f32) {
    let u = ((face % 4) as f32 + 0.5) / TEXTURE_WIDTH as f32;
    let v = ((face / 4) as f32 + 0.5) / TEXTURE_HEIGHT as f32;
    (u, v)
}

/// Six distinct flat colours, one per face, in the first six texels —
/// the palette [`FACE_COLORS`] documents. Authored `B, G, R, A` for
/// the same reason [`checkerboard`] is.
const fn face_palette() -> [u8; TEXTURE_SIZE] {
    let mut data = [0u8; TEXTURE_SIZE];
    //                     B    G    R    A
    let colors: [[u8; 4]; 6] = [
        [0, 0, 255, 255],   // +X red
        [0, 255, 0, 255],   // -X green
        [255, 0, 0, 255],   // +Y blue
        [0, 255, 255, 255], // -Y yellow
        [255, 0, 255, 255], // +Z magenta
        [255, 255, 0, 255], // -Z cyan
    ];
    let mut face = 0;
    while face < 6 {
        let idx = face * 4;
        data[idx] = colors[face][0];
        data[idx + 1] = colors[face][1];
        data[idx + 2] = colors[face][2];
        data[idx + 3] = colors[face][3];
        face += 1;
    }
    data
}

/// A small red/white checkerboard — this demo only needs something
/// visibly textured, not a realistic image.
///
/// Texels are authored **B, G, R, A**, not R, G, B, A. The texture
/// path exchanges red and blue relative to the framebuffer, which was
/// measured rather than assumed: a texel written as `FF 00 00 FF`
/// (red, if byte 0 were R) read back out of the framebuffer as
/// `00 00 FF FF`, with alpha untouched. The clear-color path is *not*
/// affected — a clear of `0xff804020` displays blue-dominant, which
/// only holds if byte 0 is R there — so the exchange happens
/// specifically between the TMU and the fragment shader's tile-buffer
/// write, not in the framebuffer's own interpretation. Authoring the
/// texture in the order that path actually wants keeps both correct;
/// swapping the framebuffer's `PixelOrder` instead would fix the
/// texture and break the clear.
const fn checkerboard() -> [u8; TEXTURE_SIZE] {
    const RED: [u8; 4] = [0, 0, 255, 255];
    const WHITE: [u8; 4] = [255, 255, 255, 255];

    let mut data = [0u8; TEXTURE_SIZE];
    let mut y = 0;
    while y < TEXTURE_HEIGHT {
        let mut x = 0;
        while x < TEXTURE_WIDTH {
            let color = if (x + y) % 2 == 0 { RED } else { WHITE };
            let idx = (y * TEXTURE_WIDTH + x) * 4;
            data[idx] = color[0];
            data[idx + 1] = color[1];
            data[idx + 2] = color[2];
            data[idx + 3] = color[3];
            x += 1;
        }
        y += 1;
    }
    data
}

static TEXTURE_DATA: AlignedBuffer<TEXTURE_SIZE> = AlignedBuffer(if FACE_COLORS {
    face_palette()
} else {
    checkerboard()
});

// ---- Cube geometry ----
//
// 24 vertices (4 per face), not 8 shared corners — a shared-corner
// cube can't give each face its own texcoords without seams. Winding
// order doesn't matter for visibility (see the module doc comment on
// why culling stays off), so faces are listed in whatever order was
// convenient. Each vertex is `[x, y, z, w, u, v]`, matching the
// `vec4` position + `vec2` texcoord interleaved layout the captured
// shaders were compiled against.
#[rustfmt::skip]
const VERTICES: [f32; 24 * 6] = [
    // +X face
     1.0, -1.0, -1.0, 1.0,  0.0, 0.0,
     1.0,  1.0, -1.0, 1.0,  1.0, 0.0,
     1.0,  1.0,  1.0, 1.0,  1.0, 1.0,
     1.0, -1.0,  1.0, 1.0,  0.0, 1.0,
    // -X face
    -1.0, -1.0,  1.0, 1.0,  0.0, 0.0,
    -1.0,  1.0,  1.0, 1.0,  1.0, 0.0,
    -1.0,  1.0, -1.0, 1.0,  1.0, 1.0,
    -1.0, -1.0, -1.0, 1.0,  0.0, 1.0,
    // +Y face
    -1.0,  1.0, -1.0, 1.0,  0.0, 0.0,
    -1.0,  1.0,  1.0, 1.0,  1.0, 0.0,
     1.0,  1.0,  1.0, 1.0,  1.0, 1.0,
     1.0,  1.0, -1.0, 1.0,  0.0, 1.0,
    // -Y face
    -1.0, -1.0,  1.0, 1.0,  0.0, 0.0,
    -1.0, -1.0, -1.0, 1.0,  1.0, 0.0,
     1.0, -1.0, -1.0, 1.0,  1.0, 1.0,
     1.0, -1.0,  1.0, 1.0,  0.0, 1.0,
    // +Z face
    -1.0, -1.0,  1.0, 1.0,  0.0, 0.0,
     1.0, -1.0,  1.0, 1.0,  1.0, 0.0,
     1.0,  1.0,  1.0, 1.0,  1.0, 1.0,
    -1.0,  1.0,  1.0, 1.0,  0.0, 1.0,
    // -Z face
     1.0, -1.0, -1.0, 1.0,  0.0, 0.0,
    -1.0, -1.0, -1.0, 1.0,  1.0, 0.0,
    -1.0,  1.0, -1.0, 1.0,  1.0, 1.0,
     1.0,  1.0, -1.0, 1.0,  0.0, 1.0,
];

/// Two triangles per face, sharing that face's own 4 vertices (never
/// the other faces' — see [`VERTICES`]'s doc comment on why there are
/// 24, not 8).
#[rustfmt::skip]
const INDICES: [u16; 36] = [
     0,  1,  2,   0,  2,  3, // +X
     4,  5,  6,   4,  6,  7, // -X
     8,  9, 10,   8, 10, 11, // +Y
    12, 13, 14,  12, 14, 15, // -Y
    16, 17, 18,  16, 18, 19, // +Z
    20, 21, 22,  20, 22, 23, // -Z
];

// ---- Matrix math ----
//
// Kept here, not in `rpi-hal` — model/projection math for one demo
// isn't something this HAL abstracts, same split as the cube geometry
// above. Column-major throughout (`col*4 + row` indexing), matching
// `glUniformMatrix4fv`'s own layout and `uniforms.rs`'s expectation.
mod math {
    /// A 4x4 matrix, column-major (`m[col * 4 + row]`).
    pub type Mat4 = [f32; 16];

    /// `a * b` — applying the result to a vector applies `b` first,
    /// then `a` (standard column-major composition).
    pub fn multiply(a: &Mat4, b: &Mat4) -> Mat4 {
        let mut r = [0.0; 16];
        for col in 0..4 {
            for row in 0..4 {
                let mut sum = 0.0;
                for k in 0..4 {
                    sum += a[k * 4 + row] * b[col * 4 + k];
                }
                r[col * 4 + row] = sum;
            }
        }
        r
    }

    /// Combined rotation about the X axis by `angle_x` then the Y axis
    /// by `angle_y` (applied in that order to a vector: Y-then-X, i.e.
    /// `rotation_y * rotation_x`) — purely for a visually interesting
    /// tumble, not modeling anything physical.
    pub fn rotation_xy(angle_x: f32, angle_y: f32) -> Mat4 {
        let (sx, cx) = (libm::sinf(angle_x), libm::cosf(angle_x));
        let (sy, cy) = (libm::sinf(angle_y), libm::cosf(angle_y));
        #[rustfmt::skip]
        let rotate_x: Mat4 = [
            1.0, 0.0, 0.0, 0.0,
            0.0,  cx,  sx, 0.0,
            0.0, -sx,  cx, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ];
        #[rustfmt::skip]
        let rotate_y: Mat4 = [
             cy, 0.0, -sy, 0.0,
            0.0, 1.0, 0.0, 0.0,
             sy, 0.0,  cy, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ];
        multiply(&rotate_y, &rotate_x)
    }

    /// Rotation about the Z axis (the view direction) by `angle` —
    /// the roll [`rotation_xy`] doesn't cover, so the three composed
    /// together give a tumble that turns about every axis.
    pub fn rotation_z(angle: f32) -> Mat4 {
        let (s, c) = (libm::sinf(angle), libm::cosf(angle));
        #[rustfmt::skip]
        let m: Mat4 = [
              c,   s, 0.0, 0.0,
             -s,   c, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ];
        m
    }

    /// Translation by `(x, y, z)`.
    pub fn translation(x: f32, y: f32, z: f32) -> Mat4 {
        #[rustfmt::skip]
        let m: Mat4 = [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
              x,   y,   z, 1.0,
        ];
        m
    }

    /// Standard OpenGL-style perspective projection: `fovy_radians`
    /// vertical field of view, `aspect` width/height, mapping the
    /// `near`/`far` planes to clip-space Z `-1`/`1` — which
    /// `bcl.rs`'s `clipper_z_scale_and_offset` (`0.5`/`0.5`) then maps
    /// to screen-space depth `0`/`1`, matching this demo's `LESS`
    /// depth test convention (nearer = smaller).
    pub fn perspective(fovy_radians: f32, aspect: f32, near: f32, far: f32) -> Mat4 {
        let f = 1.0 / libm::tanf(fovy_radians / 2.0);
        #[rustfmt::skip]
        let m: Mat4 = [
            f / aspect, 0.0, 0.0, 0.0,
            0.0, f, 0.0, 0.0,
            0.0, 0.0, (far + near) / (near - far), -1.0,
            0.0, 0.0, (2.0 * far * near) / (near - far), 0.0,
        ];
        m
    }
}

// ---- GPU-visible buffers ----

/// Unused slack allocated after each control list, so the list's end
/// address always lands inside its own buffer's padding rather than on
/// whatever the linker happened to place next.
///
/// V3D's control-list executor stops the instant its current address
/// reaches the end address in `CT0EA`/`CT1EA` — including while it is
/// *inside* a sub-list. An `Indexed Primitive List` makes the executor
/// branch to the index data and return afterwards, so if the index
/// buffer happens to begin exactly at the control list's end address,
/// the branch lands on that address and the executor halts before
/// reading a single index. That is not hypothetical: with `BCL_BUF`
/// and `INDEX_BUF` adjacent in `.bss`, `V3D_CT00RA0` read back the
/// address right after the `Indexed Primitive List` packet and
/// `CT0CS.CTRTSD` showed one level of nesting, with binning frozen at
/// its full primitive count and no error bit set anywhere.
const CL_END_SLACK: usize = 32;

static mut BCL_BUF: GpuBuffer<{ bcl::SIZE + CL_END_SLACK }> = GpuBuffer::new();
static mut RCL_BUF: GpuBuffer<{ RCL_SIZE + CL_END_SLACK }> = GpuBuffer::new();
static mut TILE_STATE_BUF: GpuBuffer<TILE_STATE_SIZE> = GpuBuffer::new();
static mut TILE_ALLOC_BUF: GpuBuffer<TILE_ALLOC_SIZE> = GpuBuffer::new();
static mut DEPTH_BUF: GpuBuffer<DEPTH_BUFFER_SIZE> = GpuBuffer::new();
static mut SHADER_RECORD_BUF: GpuBuffer<52> = GpuBuffer::new();
static mut VERTEX_BUF: GpuBuffer<{ VERTICES.len() * 4 }> = GpuBuffer::new();
static mut INDEX_BUF: GpuBuffer<{ INDICES.len() * 2 }> = GpuBuffer::new();
static mut COORD_UNIFORMS_BUF: GpuBuffer<{ uniforms::UNIFORM_COUNT * 4 }> = GpuBuffer::new();
static mut VERTEX_UNIFORMS_BUF: GpuBuffer<{ uniforms::UNIFORM_COUNT * 4 }> = GpuBuffer::new();
static mut FRAGMENT_UNIFORMS_BUF: GpuBuffer<8> = GpuBuffer::new();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // The V3D block needs a running core clock before any of it does
    // real work, and nothing upstream sets one up here: the boot
    // firmware leaves the 3D clock alone (it never renders anything
    // itself), and there's no Linux clock driver in a bare-metal build
    // to bring it up the way `vc4_v3d.c`'s `clk_prepare_enable` does.
    // Enabling the QPU alone isn't enough -- that's a power/gating
    // switch, not a clock. Set the rate explicitly, before
    // `set_enable_qpu`, matching the ordering of the one bare-metal V3D
    // bring-up known to work on this hardware (Peter Lemon's `V3DINIT`
    // sample, which sends "Set Clock Rate" for the V3D clock at 250MHz
    // and only then "Enable QPU").
    match mailbox.clock_rate_hz(ClockId::V3d) {
        Ok(hz) => {
            let _ = writeln!(uart, "v3d clock before: {hz} Hz");
        }
        Err(e) => {
            let _ = writeln!(uart, "v3d clock query failed: {e:?}");
        }
    }
    match mailbox.set_clock_rate_hz(ClockId::V3d, 250_000_000) {
        Ok(hz) => {
            let _ = writeln!(uart, "v3d clock set to: {hz} Hz");
        }
        Err(e) => {
            let _ = writeln!(uart, "v3d clock set failed: {e:?}");
        }
    }

    match mailbox.set_enable_qpu(true) {
        Ok(()) => {
            let _ = writeln!(uart, "set_enable_qpu(true): ok");
        }
        Err(e) => {
            let _ = writeln!(uart, "set_enable_qpu(true): error {e:?}");
        }
    }

    // SAFETY: single-threaded `kmain`; only one `V3d` is constructed,
    // here.
    let v3d = unsafe { V3d::new() };
    let (ident0, ident1, ident2) = v3d.ident();
    let _ = writeln!(
        uart,
        "V3D IDENT: 0x{ident0:08x} 0x{ident1:08x} 0x{ident2:08x}"
    );

    let framebuffer = match mailbox.allocate_framebuffer(
        u32::from(WIDTH_PX),
        u32::from(HEIGHT_PX),
        32,
        PixelOrder::Rgb,
    ) {
        Ok(fb) => fb,
        Err(e) => {
            let _ = writeln!(uart, "framebuffer allocation failed: {e:?}");
            halt();
        }
    };
    let _ = writeln!(
        uart,
        "framebuffer: {}x{} @ 0x{:08x}, pitch {}",
        framebuffer.width, framebuffer.height, framebuffer.address, framebuffer.pitch_bytes
    );
    // V3D's `Tile Rendering Mode Configuration` packet has no pitch
    // field of its own -- it assumes the render target's row stride is
    // exactly `width * 4` (see `rcl.rs`). If the firmware padded this
    // framebuffer's rows wider than that, rendering into it directly
    // would silently write to the wrong offsets.
    assert_eq!(
        framebuffer.pitch_bytes,
        framebuffer.width * 4,
        "framebuffer pitch has padding V3D's fixed-pitch render target can't handle"
    );

    // SAFETY: single-threaded `kmain`; these statics are touched only
    // here, from this point until the end of the function.
    let (
        bcl_buf,
        rcl_buf,
        tile_state_buf,
        tile_alloc_buf,
        depth_buf,
        shader_record_buf,
        vertex_buf,
        index_buf,
        coord_uniforms_buf,
        vertex_uniforms_buf,
        fragment_uniforms_buf,
    ) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(BCL_BUF),
            &mut *core::ptr::addr_of_mut!(RCL_BUF),
            &mut *core::ptr::addr_of_mut!(TILE_STATE_BUF),
            &mut *core::ptr::addr_of_mut!(TILE_ALLOC_BUF),
            &mut *core::ptr::addr_of_mut!(DEPTH_BUF),
            &mut *core::ptr::addr_of_mut!(SHADER_RECORD_BUF),
            &mut *core::ptr::addr_of_mut!(VERTEX_BUF),
            &mut *core::ptr::addr_of_mut!(INDEX_BUF),
            &mut *core::ptr::addr_of_mut!(COORD_UNIFORMS_BUF),
            &mut *core::ptr::addr_of_mut!(VERTEX_UNIFORMS_BUF),
            &mut *core::ptr::addr_of_mut!(FRAGMENT_UNIFORMS_BUF),
        )
    };

    // Vertex/index data never changes frame to frame -- populate once.
    // Under FACE_COLORS the positions are used as declared but each
    // vertex's texture coordinate is replaced by its face's palette
    // texel, so all four corners of a face sample one flat colour.
    // Vertices are grouped four per face, in VERTICES' declared order.
    for (i, value) in VERTICES.iter().enumerate() {
        let vertex = i / 6;
        let component = i % 6;
        let value = if FACE_COLORS && component >= 4 {
            let (u, v) = face_texcoord(vertex / 4);
            if component == 4 {
                u
            } else {
                v
            }
        } else {
            *value
        };
        vertex_buf.as_bytes_mut()[i * 4..i * 4 + 4].copy_from_slice(&value.to_le_bytes());
    }
    vertex_buf.flush();
    for (i, value) in INDICES.iter().enumerate() {
        index_buf.as_bytes_mut()[i * 2..i * 2 + 2].copy_from_slice(&value.to_le_bytes());
    }
    index_buf.flush();

    // These are never written by this core at all -- `GpuBuffer::new`
    // zero-initializes them, `bcl.rs` documents that the tile state
    // array needs to be genuinely zero in RAM before the first bin
    // pass. But whether that zero has actually reached RAM (versus
    // sitting dirty in this core's cache from however `.bss` got
    // zeroed at boot) isn't guaranteed without an explicit flush, so
    // do one here rather than assume it.
    tile_state_buf.flush();
    tile_alloc_buf.flush();

    // Shader code and texture data are compile-time constants -- flush
    // once so V3D's first read doesn't see stale cache-line contents.
    //
    // Plain pointers here, not `v3d::bus_address(...)`: cache
    // maintenance is by-VA (`DCCMVAC`/`dc cvac` under the hood), so it
    // has to operate on this core's own MMU-mapped address for the
    // buffer, not the `0xC000_0000` VideoCore bus alias -- that alias
    // isn't identity-mapped RAM from the ARM MMU's point of view, so a
    // cache-maintenance instruction targeting it faults. `bus_address`
    // is only for the address V3D itself will read, embedded into a
    // control list/uniform stream/shader-state record -- never for a
    // cache op. (`GpuBuffer::flush` gets this right internally; these
    // four calls are the exception, since these buffers aren't
    // `GpuBuffer`s.)
    v3d::flush(COORD_SHADER.0.as_ptr() as u32, COORD_SHADER.0.len());
    v3d::flush(VERTEX_SHADER.0.as_ptr() as u32, VERTEX_SHADER.0.len());
    v3d::flush(FRAGMENT_SHADER.0.as_ptr() as u32, FRAGMENT_SHADER.0.len());
    v3d::flush(TEXTURE_DATA.0.as_ptr() as u32, TEXTURE_DATA.0.len());

    // Texture uniforms never change either -- build once.
    texture::build_fragment_shader_uniforms(
        fragment_uniforms_buf.as_bytes_mut(),
        &texture::TextureParams {
            address: v3d::bus_address(TEXTURE_DATA.0.as_ptr() as u32),
            width_px: TEXTURE_WIDTH as u16,
            height_px: TEXTURE_HEIGHT as u16,
        },
    );
    fragment_uniforms_buf.flush();

    // The shader-state record only references buffer *addresses*, all
    // of which are fixed for the whole demo -- build once.
    shader_record::build(
        shader_record_buf.as_bytes_mut(),
        &shader_record::ShaderRecordParams {
            fragment_shader_code_address: v3d::bus_address(FRAGMENT_SHADER.0.as_ptr() as u32),
            fragment_shader_uniforms_address: fragment_uniforms_buf.bus_address(),
            fragment_shader_num_varyings: 2,
            vertex_shader_code_address: v3d::bus_address(VERTEX_SHADER.0.as_ptr() as u32),
            vertex_shader_uniforms_address: vertex_uniforms_buf.bus_address(),
            coordinate_shader_code_address: v3d::bus_address(COORD_SHADER.0.as_ptr() as u32),
            coordinate_shader_uniforms_address: coord_uniforms_buf.bus_address(),
            vertex_buffer_address: vertex_buf.bus_address(),
        },
    );
    shader_record_buf.flush();

    // Geometry, shaders, and the render target are all fixed for the
    // whole demo too -- the binning and render control lists never
    // change frame to frame, only the uniform buffers they reference
    // do. Build both once, outside the loop.
    let bcl_len = bcl::build(
        bcl_buf.as_bytes_mut(),
        &bcl::BclParams {
            tile_alloc_address: tile_alloc_buf.bus_address(),
            tile_alloc_size: TILE_ALLOC_SIZE as u32,
            tile_state_address: tile_state_buf.bus_address(),
            width_in_tiles: WIDTH_IN_TILES,
            height_in_tiles: HEIGHT_IN_TILES,
            width_px: WIDTH_PX,
            height_px: HEIGHT_PX,
            shader_state_address: shader_record_buf.bus_address(),
            attribute_array_count: 2,
            index_buffer_address: index_buf.bus_address(),
            index_count: INDICES.len() as u32,
            max_index: (VERTICES.len() / 6 - 1) as u16,
            depth_test_enabled: DEPTH_TEST_ENABLED,
        },
    );
    bcl_buf.flush();

    let rcl_len = rcl::build(
        rcl_buf.as_bytes_mut(),
        &rcl::RclParams {
            tile_alloc_address: tile_alloc_buf.bus_address(),
            width_in_tiles: WIDTH_IN_TILES,
            height_in_tiles: HEIGHT_IN_TILES,
            color_write_address: v3d::bus_address(framebuffer.address),
            depth_write_address: DEPTH_STORE_ENABLED.then(|| depth_buf.bus_address()),
            width_px: WIDTH_PX,
            height_px: HEIGHT_PX,
            // Deliberately a different value in every channel --
            // R=0x20, G=0x40, B=0x80, A=0xff, packed assuming byte 0
            // is R. Reading a cleared pixel back then shows which byte
            // actually landed where, settling the channel order this
            // file's "known open risks" flags as unverified. Non-black
            // on purpose too: a successful clear to black is
            // indistinguishable from nothing having been drawn at all.
            clear_color_rgba8888: 0xff80_4020,
        },
    );
    rcl_buf.flush();

    let bcl_range = (
        bcl_buf.bus_address(),
        bcl_buf.bus_address() + bcl_len as u32,
    );
    let rcl_range = (
        rcl_buf.bus_address(),
        rcl_buf.bus_address() + rcl_len as u32,
    );

    let projection = math::perspective(1.0, f32::from(WIDTH_PX) / f32::from(HEIGHT_PX), 0.1, 100.0);
    let mut angle = 0.0f32;
    let mut frame = 0u32;

    loop {
        // Spin about a single axis rather than tumbling about two.
        // A tumble makes it genuinely hard to say what is wrong with
        // the result -- every face is changing shape at once, so there
        // is no expected appearance to compare against. Rotating about
        // Y alone gives one: each face sweeps across, narrows to a
        // vertical line when edge-on, and widens again, repeating every
        // quarter turn. Anything that doesn't do that is describable.
        let model = if SINGLE_AXIS_ROTATION {
            math::rotation_xy(0.0, angle)
        } else {
            // Deliberately unequal, non-repeating-looking rates about
            // the three axes: equal rates would make the tumble settle
            // into an obvious short cycle, and every face would keep
            // returning to the viewer in the same orientation.
            math::multiply(
                &math::rotation_z(angle * 0.43),
                &math::rotation_xy(angle, angle * 0.7),
            )
        };
        let view = math::translation(0.0, 0.0, -4.0);
        let mvp = math::multiply(&projection, &math::multiply(&view, &model));

        uniforms::build_coordinate_shader_uniforms(
            coord_uniforms_buf.as_bytes_mut(),
            &mvp,
            WIDTH_PX,
            HEIGHT_PX,
        );
        coord_uniforms_buf.flush();
        uniforms::build_vertex_shader_uniforms(
            vertex_uniforms_buf.as_bytes_mut(),
            &mvp,
            WIDTH_PX,
            HEIGHT_PX,
        );
        vertex_uniforms_buf.flush();

        match v3d.submit_bin(bcl_range) {
            Ok(()) => {
                // `V3D_PCS` bit 8 (`BMOOM`) latches when the binner
                // runs out of tile allocation memory. Worth checking
                // every frame rather than once: how much the binner
                // needs depends on how the geometry happens to
                // distribute across tiles, so a pool that is merely
                // marginal overflows only at some orientations, and
                // the result looks like parts of the model flickering
                // in and out rather than like a memory problem.
                if v3d.debug_status().pcs & (1 << 8) != 0 {
                    let _ = writeln!(
                        uart,
                        "frame {frame}: binner out of memory -- tile allocation pool ({TILE_ALLOC_SIZE} bytes) too small"
                    );
                }
            }
            Err(e) => {
                let status = v3d.debug_status();
                let _ = writeln!(uart, "frame {frame}: submit_bin failed: {e:?}");
                let _ = writeln!(
                    uart,
                    "  ct0cs=0x{:08x} pcs=0x{:08x} bfc=0x{:08x} ct0pc=0x{:08x} errstat=0x{:08x}",
                    status.ct0cs, status.pcs, status.bfc, status.ct0pc, status.errstat
                );
            }
        }

        match v3d.submit_render(rcl_range) {
            Ok(()) => {}
            Err(e) => {
                let status = v3d.debug_status();
                let _ = writeln!(uart, "frame {frame}: submit_render failed: {e:?}");
                let _ = writeln!(
                    uart,
                    "  ct1cs=0x{:08x} pcs=0x{:08x} rfc=0x{:08x} errstat=0x{:08x}",
                    status.ct1cs, status.pcs, status.rfc, status.errstat
                );
            }
        }

        if frame.is_multiple_of(60) {
            let _ = writeln!(uart, "frame {frame}, angle {angle}");
        }

        // Small enough that the cube turns at a readable speed rather
        // than blurring: the loop has no frame pacing beyond `delay`
        // below, so the rotation per frame is what sets the apparent
        // speed, and the GPU renders this scene far faster than it
        // needs to.
        angle += 0.008;
        frame = frame.wrapping_add(1);
        delay(2_000_000);
    }
}

/// A plain busy-wait, matching `mailbox_props.rs`'s own delay helper —
/// pacing here is a rough, tunable "looks reasonable" choice, not tied
/// to any real time reference.
fn delay(cycles: u32) {
    for _ in 0..cycles {
        unsafe { core::arch::asm!("nop") };
    }
}

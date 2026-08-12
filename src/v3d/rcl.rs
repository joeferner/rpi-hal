//! Render control list (RCL) builder for the one fixed draw shape this
//! crate's V3D bring-up targets: a color *and* depth render target
//! (no stencil use, no MSAA), cleared then drawn from one prior
//! binning pass. Not a general control-list API — same framing as
//! `bcl.rs`.
//!
//! Unlike `bcl.rs`, this isn't grounded in a real capture at all, and
//! can't be: Mesa's userspace driver never builds an RCL on real
//! hardware — the kernel does, so no `VC4_DEBUG` capture contains one.
//! Instead this is a direct, deliberately narrowed port
//! of the actual Linux `vc4` kernel driver's RCL builder
//! (`vc4_get_rcl`/`vc4_create_rcl_bo`/`emit_tile` in
//! `kernel/vc4_render_cl.c`, which Mesa also vendors into its own
//! source tree for its software simulator). That file handles
//! color read/write, depth/stencil read/write, and MSAA surfaces, all
//! independently optional; this builder keeps only the
//! color-write-plus-depth-write, clear-then-draw path those functions
//! take when the read and MSAA surfaces are absent, which collapses
//! `emit_tile`'s branching down to one fixed per-tile sequence.
//!
//! Depth support was added after the color-only first version turned
//! out to be under-scoped: a real rotating cube needs occlusion to
//! look correct (later-submitted faces would otherwise always draw
//! over earlier ones, regardless of actual distance from the camera),
//! which color-only rendering with `Configuration Bits`' depth test
//! forced to `ALWAYS` can't provide.
//!
//! Packet fields (opcodes, sizes) are from the same
//! `vc4_packet.xml` `bcl.rs` uses; the *sequence* —
//! which packets, in what order, and the `WAIT_ON_SEMAPHORE`/
//! `BRANCH_TO_SUB_LIST` synchronization with the binning pass in
//! particular, and (for depth) exactly when a second `Tile
//! Coordinates` needs to be re-emitted between the depth and color
//! stores of the same tile — is ported from the kernel source itself,
//! not reconstructed from the packet spec alone. `BRANCH_TO_SUB_LIST`'s
//! target address formula (`tile_alloc_address + (y * width_in_tiles +
//! x) * 32`, one fixed 32-byte slot per tile) comes directly from
//! `vc4_create_rcl_bo`'s call into it — the CPU can compute this
//! without reading back anything the binner wrote, since it only
//! depends on values this same builder's caller already chose.
//!
//! Not yet resolved: whether `Tile Rendering Mode Configuration`'s
//! pixel format field should be `RGBA8888` unconditionally, or needs
//! to track whichever `PixelOrder` the mailbox-allocated framebuffer
//! actually negotiated (`examples/display_page_flip.rs` uses
//! `PixelOrder::Bgr`) — a real channel-order mismatch this hasn't been
//! checked against yet, left for Phase 7 once there's a real
//! framebuffer to render into.

/// Runtime parameters for [`build`]. `tile_alloc_address`,
/// `width_in_tiles`, and `height_in_tiles` must match the values
/// passed to the corresponding [`crate::v3d::bcl::BclParams`] for the
/// same frame — this builder's `BRANCH_TO_SUB_LIST` addresses are
/// computed from them, not read back from anything the binner wrote.
pub struct RclParams {
    /// Bus address of the tile allocation memory buffer — must match
    /// [`crate::v3d::bcl::BclParams::tile_alloc_address`].
    pub tile_alloc_address: u32,
    /// Render target width, in 64x64-pixel tiles — must match
    /// [`crate::v3d::bcl::BclParams::width_in_tiles`].
    pub width_in_tiles: u8,
    /// Render target height, in 64x64-pixel tiles — must match
    /// [`crate::v3d::bcl::BclParams::height_in_tiles`].
    pub height_in_tiles: u8,
    /// Bus address of the color render target (e.g. a
    /// [`crate::mailbox::Framebuffer`] page).
    pub color_write_address: u32,
    /// Bus address to write the depth/stencil buffer out to, or `None`
    /// to not write it at all — a plain buffer the same pixel
    /// dimensions as the color target, 4 bytes per pixel (matching
    /// `vc4_state.c`'s `cpp = 4` for a Z/stencil surface), holding a
    /// packed 24-bit depth + 8-bit stencil value per pixel.
    ///
    /// This controls only whether depth is *stored to memory*, not
    /// whether depth testing happens — that is
    /// [`crate::v3d::bcl::BclParams::depth_test_enabled`], and it works
    /// against the tile buffer's own Z storage, which the hardware
    /// clears per tile from `Clear Colors`. `None` with testing enabled
    /// is the normal combination for a render that never reads its
    /// depth buffer back afterwards, and is worth preferring: the
    /// store path is the one part of this module with no real capture
    /// behind it (see the module doc comment), so not emitting it keeps
    /// it out of the critical path.
    pub depth_write_address: Option<u32>,
    /// Render target width, in pixels.
    pub width_px: u16,
    /// Render target height, in pixels.
    pub height_px: u16,
    /// Clear color, packed to match `Tile Rendering Mode
    /// Configuration`'s `RGBA8888` pixel format — exact channel byte
    /// order not yet checked against a real framebuffer (see the
    /// module doc comment).
    pub clear_color_rgba8888: u32,
}

/// Exact size, in bytes, of the render control list [`build`] emits
/// for a `width_in_tiles` x `height_in_tiles` render target — unlike
/// [`crate::v3d::bcl::tile_state_size`], this isn't a kernel-sourced
/// formula, just the real byte cost of this module's own fixed
/// sequence: a 35-byte header (`Tile Rendering Mode Configuration` +
/// `Clear Colors` + one `Tile Coordinates` + the "store in None mode"
/// clear-trigger), then per tile either 9 bytes (`Tile Coordinates` +
/// `Branch to sub-list` + the color store byte, if `with_depth` is
/// `false`) or 19 (the same plus the depth store and a second `Tile
/// Coordinates`, if `with_depth` is `true` — matching
/// [`RclParams::depth_write_address`] being `Some`), plus 1 extra byte
/// for the first tile's `Wait on Semaphore`.
pub const fn size(width_in_tiles: u8, height_in_tiles: u8, with_depth: bool) -> usize {
    let tile_count = width_in_tiles as usize * height_in_tiles as usize;
    let per_tile = if with_depth { 19 } else { 9 };
    35 + per_tile * tile_count + 1
}

/// Builds the fixed render control list into `cl`, returning the
/// number of bytes written (matches [`size`], called with the same
/// `params.depth_write_address.is_some()`). Panics (via slice
/// indexing) if `cl` is too small, same rationale as
/// [`crate::v3d::bcl::build`].
pub fn build(cl: &mut [u8], params: &RclParams) -> usize {
    let mut b = Builder { bytes: cl, len: 0 };
    b.tile_rendering_mode_configuration(params);
    b.clear_colors(params);
    b.tile_coordinates(0, 0);
    b.store_tile_buffer_general_none();

    let width_in_tiles = u32::from(params.width_in_tiles);
    let height_in_tiles = u32::from(params.height_in_tiles);
    for y in 0..height_in_tiles {
        for x in 0..width_in_tiles {
            let first = x == 0 && y == 0;
            let last = x == width_in_tiles - 1 && y == height_in_tiles - 1;
            b.tile_coordinates(x as u8, y as u8);
            if first {
                b.wait_on_semaphore();
            }
            b.branch_to_sub_list(params.tile_alloc_address + (y * width_in_tiles + x) * 32);
            if let Some(depth_write_address) = params.depth_write_address {
                b.store_depth(depth_write_address);
                // Re-emitted before the color store: `emit_tile` in
                // the real kernel driver does this whenever both a
                // depth and a color write are configured for the same
                // tile, not just once per tile overall.
                b.tile_coordinates(x as u8, y as u8);
            }
            b.store_ms_tile_buffer(last);
        }
    }

    b.len
}

/// Raw packet-serialization cursor, same shape as
/// [`crate::v3d::bcl`]'s own `Builder`.
struct Builder<'a> {
    bytes: &'a mut [u8],
    len: usize,
}

impl<'a> Builder<'a> {
    fn push_u8(&mut self, v: u8) {
        self.bytes[self.len] = v;
        self.len += 1;
    }

    fn push_u16(&mut self, v: u16) {
        self.bytes[self.len..self.len + 2].copy_from_slice(&v.to_le_bytes());
        self.len += 2;
    }

    fn push_u32(&mut self, v: u32) {
        self.bytes[self.len..self.len + 4].copy_from_slice(&v.to_le_bytes());
        self.len += 4;
    }

    /// Opcode `0x71` (`VC4_PACKET_TILE_RENDERING_MODE_CONFIG`, code
    /// 113). 11 bytes: the render target's bus address, width, height,
    /// then one `u16` of format/mode flags. `0x0004` selects
    /// `RGBA8888` pixel format (`VC4_RENDER_CONFIG_FORMAT_RGBA8888 =
    /// 1`, at bit shift 2) with raster (non-tiled) memory layout, no
    /// multisampling, no 64-bit color depth — matching
    /// `vc4_create_rcl_bo`'s `args->color_write.bits` for this same
    /// fixed configuration. This packet only carries the *color*
    /// target's address — the depth target isn't configured here at
    /// all, only in each tile's own depth store (`store_depth`).
    fn tile_rendering_mode_configuration(&mut self, params: &RclParams) {
        self.push_u8(0x71);
        self.push_u32(params.color_write_address);
        self.push_u16(params.width_px);
        self.push_u16(params.height_px);
        self.push_u16(0x0004);
    }

    /// Opcode `0x72` (`VC4_PACKET_CLEAR_COLORS`, code 114). 14 bytes:
    /// the 64-bit clear color as two `u32` words (both set to the same
    /// packed color — this target is 32-bit, not 64-bit, so the
    /// second word is likely unused, but `vc4_create_rcl_bo` always
    /// writes both from the same two-word `args->clear_color` array,
    /// so this mirrors that shape rather than assuming which half
    /// matters), then a `u32` Z/VG-mask clear and a `u8` stencil clear.
    /// The Z clear is `0x00ff_ffff` — the maximum 24-bit value, i.e.
    /// "as far away as possible" — matching this demo's `LESS` depth
    /// test (`bcl.rs`'s `configuration_bits`): every real fragment's
    /// depth must compare as nearer than a cleared pixel for the first
    /// draw at that pixel to ever pass.
    fn clear_colors(&mut self, params: &RclParams) {
        self.push_u8(0x72);
        self.push_u32(params.clear_color_rgba8888);
        self.push_u32(params.clear_color_rgba8888);
        self.push_u32(0x00ff_ffff);
        self.push_u8(0);
    }

    /// Opcode `0x73` (`VC4_PACKET_TILE_COORDINATES`, code 115). 3
    /// bytes: tile column, then tile row, each a plain `u8`. Selects
    /// which tile the packets that follow apply to, and triggers any
    /// pending load — `vc4_render_cl.c`'s comment: "Clipping depends
    /// on tile coordinates having been emitted, so we always need one
    /// here." With both a depth and a color store per tile, this gets
    /// emitted *twice* per tile — see `build`'s loop and this module's
    /// doc comment.
    fn tile_coordinates(&mut self, x: u8, y: u8) {
        self.push_u8(0x73);
        self.push_u8(x);
        self.push_u8(y);
    }

    /// Opcode `0x1c` (`VC4_PACKET_STORE_TILE_BUFFER_GENERAL`, code
    /// 28), in "None" mode (`Buffer to Store` = `0`, `bits = 0x0000`,
    /// address `0`, unused since nothing is actually written). Ported
    /// directly from `vc4_render_cl.c`'s `vc4_store_before_load` and
    /// the comment in `vc4_create_rcl_bo`: the tile buffer only
    /// actually clears when the *previous* tile is stored, so the
    /// first tile needs a no-op store here first to trigger its own
    /// clear before the real per-tile loop begins.
    fn store_tile_buffer_general_none(&mut self) {
        self.push_u8(0x1c);
        self.push_u16(0x0000);
        self.push_u32(0);
    }

    /// Opcode `0x1c` (`VC4_PACKET_STORE_TILE_BUFFER_GENERAL`, code 28)
    /// again, this time configured for a real depth store: 7 bytes —
    /// opcode, a `u16` of `Buffer to Store = Z/stencil (2)` (bits 0-2)
    /// with `Disable Color buffer clear on store/dump` also set (bit
    /// 13, `0x2000`) — ported from `emit_tile`'s real bit expression
    /// (`args->zs_write.bits | (last_tile_write ? 0 :
    /// DISABLE_COLOR_CLEAR)`, and `last_tile_write` is always false
    /// here since this demo always has a color write too) — giving
    /// `0x2002`, then the depth buffer's plain bus address as a `u32`
    /// (16-byte aligned; unlike the `0x2002` bits' own packing, the
    /// address carries no extra flags in this demo, since `EOF` only
    /// ever applies to the final *color* store of the final tile).
    fn store_depth(&mut self, address: u32) {
        self.push_u8(0x1c);
        self.push_u16(0x2002);
        self.push_u32(address);
    }

    /// Opcode `0x08` (`VC4_PACKET_WAIT_ON_SEMAPHORE`, code 8). 1 byte,
    /// opcode only. Emitted once, before the first tile's
    /// `branch_to_sub_list` — waits for the binning pass this render
    /// depends on to finish before the renderer starts walking its
    /// output. Missing this would be a real race, not just a
    /// correctness nicety.
    fn wait_on_semaphore(&mut self) {
        self.push_u8(0x08);
    }

    /// Opcode `0x11` (`VC4_PACKET_BRANCH_TO_SUB_LIST`, code 17). 5
    /// bytes: one `u32` bus address, unmodified — jumps into the
    /// current tile's already-binned primitive list. `address` is
    /// computed by [`build`], not by this method, since the formula
    /// (`tile_alloc_address + (y * width_in_tiles + x) * 32`) needs
    /// the tile's `(x, y)` position, not just its address.
    fn branch_to_sub_list(&mut self, address: u32) {
        self.push_u8(0x11);
        self.push_u32(address);
    }

    /// Opcode `0x18` (`VC4_PACKET_STORE_MS_TILE_BUFFER`, code 24) or,
    /// on the last tile, opcode `0x19`
    /// (`VC4_PACKET_STORE_MS_TILE_BUFFER_AND_EOF`, code 25) — both 1
    /// byte, opcode only. Writes the current tile's color back to the
    /// address `tile_rendering_mode_configuration` already set, at the
    /// position the second `tile_coordinates` of this tile just
    /// selected — no per-store address needed, unlike `store_depth`,
    /// since this simpler store packet only exists for exactly this
    /// single-color-target case.
    fn store_ms_tile_buffer(&mut self, last: bool) {
        self.push_u8(if last { 0x19 } else { 0x18 });
    }
}

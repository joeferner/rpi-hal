//! Binning control list (BCL) builder for the one fixed draw shape
//! this crate's V3D bring-up targets: a single indexed, textured
//! triangle-list draw per frame, real depth testing (see
//! `configuration_bits`), no blending, no antialiasing. Not a general
//! control-list API -- building one for arbitrary GL-like state is far
//! more than this bring-up needs.
//!
//! Packet opcodes, ordering, sizes, and exact bit-level field layout
//! all come from Mesa's `src/broadcom/cle/vc4_packet.xml` -- the
//! machine-readable packet spec its own `vc4` driver decoder
//! (`v3d_spec_load`) reads to produce the annotated `BCL:` dumps this
//! builder was cross-checked against, captured by running a GLES
//! program under `VC4_DEBUG=cl` on a Pi 3 (see this crate's
//! `examples/gpu_cube.rs` for what those shaders were).
//!
//! Every packet below is confirmed against that spec, including
//! `indexed_primitive_list`'s total size -- the XML's field offsets
//! settle it independently (`Maximum Index` starts at bit 72 and is 32
//! bits wide, putting the whole packet at 14 bytes), since the printed
//! `BCL:` dump doesn't show what actually follows it in memory (see
//! `increment_semaphore`/`flush` for why that matters: two more real
//! packets follow that this crate initially missed entirely, because
//! Mesa's CL dumper doesn't print them).
//!
//! Two fields (`configuration_bits`'s second and third bytes, and
//! `gl_shader_state`'s address word) were originally implemented from
//! general recollection of Broadcom's spec before that XML turned up,
//! and were wrong; both match the XML exactly now.

/// Runtime parameters for [`build`] — everything about a binning
/// control list that varies per render target, per shader, or per
/// frame, as opposed to the fixed pipeline state (culling, depth
/// function, blending) [`build`] always emits the same way.
pub struct BclParams {
    /// Bus address of the tile allocation memory buffer (grows as the
    /// binner sorts primitives into tiles). No confirmed sizing formula
    /// for this one — unlike [`tile_state_size`], the real kernel driver
    /// just hands the binner "the rest of" a pre-sized pool it manages
    /// itself, which isn't a number this crate has an equivalent of.
    /// Pick something generously larger than the scene's real primitive
    /// count needs; the binner only fails (`VC4_INT_OUTOMEM`) if it
    /// actually runs out mid-frame.
    pub tile_alloc_address: u32,
    /// Size of the tile allocation memory buffer, in bytes — see
    /// [`tile_alloc_address`](Self::tile_alloc_address)'s caveat.
    pub tile_alloc_size: u32,
    /// Bus address of the tile state data array. Size it with
    /// [`tile_state_size`], and zero it before the first frame that
    /// reuses it — `build` sets the hardware auto-init flag so V3D
    /// re-clears it on every subsequent bin pass itself, matching the
    /// real kernel driver, but that flag doesn't cover the very first
    /// use.
    pub tile_state_address: u32,
    /// Render target width, in 64x64-pixel tiles.
    pub width_in_tiles: u8,
    /// Render target height, in 64x64-pixel tiles.
    pub height_in_tiles: u8,
    /// Render target width, in pixels (the clip window).
    pub width_px: u16,
    /// Render target height, in pixels (the clip window).
    pub height_px: u16,
    /// Bus address of the shader-state record (coordinate, vertex, and
    /// fragment shader machine code plus attribute/uniform
    /// configuration — see [`crate::v3d::shader_record`]). Must be
    /// 16-byte aligned — see this module's
    /// `gl_shader_state`.
    pub shader_state_address: u32,
    /// Number of `glVertexAttribPointer`-equivalent attribute arrays
    /// the shader-state record describes (2 for this crate's cube:
    /// position, texcoord). Must be 1-8 — see `gl_shader_state`.
    pub attribute_array_count: u8,
    /// Bus address of the 16-bit index buffer.
    pub index_buffer_address: u32,
    /// Number of indices to draw.
    pub index_count: u32,
    /// Largest index value in the buffer at `index_buffer_address`.
    pub max_index: u16,
    /// Whether to run a real depth test (`LESS`, with Z-updates on) or
    /// the color-only, always-passing configuration. See
    /// `configuration_bits`, which gives the captured byte values
    /// behind both.
    ///
    /// Independent of
    /// [`crate::v3d::rcl::RclParams::depth_write_address`], despite
    /// both being about depth. This one selects whether fragments are
    /// tested against the *tile buffer's* Z, which the hardware clears
    /// per tile; that one selects whether the resulting Z is also
    /// written out to memory. Testing without storing is the normal
    /// case for a render that never reads its depth buffer back.
    pub depth_test_enabled: bool,
}

/// Required size, in bytes, of the tile state data array for a
/// `width_in_tiles` x `height_in_tiles` render target — `48` bytes per
/// tile, rounded up to a 4096-byte boundary. Not derived from the
/// packet spec (nothing in `vc4_packet.xml` says how big this array
/// needs to be) — this is the real Linux kernel driver's own sizing
/// formula, read directly out of `vc4_validate.c`'s tile-binning-mode
/// packet validation (`tile_state_size = 48 * tile_count;
/// roundup(tile_state_size, 4096)`).
pub const fn tile_state_size(width_in_tiles: u8, height_in_tiles: u8) -> u32 {
    let tile_count = width_in_tiles as u32 * height_in_tiles as u32;
    (48 * tile_count).next_multiple_of(4096)
}

/// Exact size, in bytes, of the binning control list [`build`] emits —
/// always `96`, regardless of resolution or tile count: unlike
/// [`crate::v3d::rcl::build`], this module's packet sequence has no
/// per-tile loop, so nothing about it scales with the render target.
pub const SIZE: usize = 96;

/// Builds the fixed binning control list into `cl`, returning the
/// number of bytes written (matches [`SIZE`]). Panics (via slice
/// indexing) if `cl` is too small — a fixed, known-at-compile-time
/// sequence overrunning its buffer is this crate's own sizing bug, not
/// a runtime condition to recover from.
pub fn build(cl: &mut [u8], params: &BclParams) -> usize {
    let mut b = Builder { bytes: cl, len: 0 };
    b.tile_binning_mode_configuration(params);
    b.start_tile_binning();
    b.primitive_list_format();
    b.clip_window(params);
    b.configuration_bits(params);
    b.depth_offset();
    b.point_size();
    b.line_width();
    b.clipper_xy_scaling(params);
    b.clipper_z_scale_and_offset();
    b.viewport_offset(params);
    b.flat_shade_flags();
    b.gl_shader_state(params);
    b.indexed_primitive_list(params);
    b.increment_semaphore();
    b.flush();
    b.len
}

/// Raw packet-serialization cursor over a caller-owned byte slice
/// (typically a [`crate::v3d::GpuBuffer`]'s
/// [`as_bytes_mut`](crate::v3d::GpuBuffer::as_bytes_mut)).
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

    fn push_f32(&mut self, v: f32) {
        self.push_u32(v.to_bits());
    }

    /// Opcode `0x70` (`VC4_PACKET_TILE_BINNING_MODE_CONFIG`, code 112).
    /// 16 bytes: 3 address/size `u32`s, a tile-count byte pair, then
    /// one flags byte (bits 112-119 of the packet: multisample at bit
    /// 0, 64-bit color at bit 1, auto-init TSDA at bit 2, initial
    /// block size at bits 3-4, block size at bits 5-6, double-buffer at
    /// bit 7). No multisample, 32-bit color, single-buffered — but
    /// auto-init TSDA *on* and block sizes `32`/`128` (initial/regular),
    /// matching `vc4_validate.c`'s real submission path exactly (the
    /// kernel forces these three specific values on every real
    /// submission, overriding whatever userspace originally asked for)
    /// rather than this demo's own guess at reasonable defaults.
    fn tile_binning_mode_configuration(&mut self, params: &BclParams) {
        self.push_u8(0x70);
        self.push_u32(params.tile_alloc_address);
        self.push_u32(params.tile_alloc_size);
        self.push_u32(params.tile_state_address);
        self.push_u8(params.width_in_tiles);
        self.push_u8(params.height_in_tiles);
        self.push_u8(0x44); // auto-init TSDA (bit 2) | block size 128 (bits 5-6, value 2).
    }

    /// Opcode `0x06` (`VC4_PACKET_START_TILE_BINNING`, code 6). 1 byte,
    /// opcode only.
    fn start_tile_binning(&mut self) {
        self.push_u8(0x06);
    }

    /// Opcode `0x38` (`VC4_PACKET_PRIMITIVE_LIST_FORMAT`, code 56). 2
    /// bytes: one data byte, "Primitive Type" in bits 0-3 (`2` =
    /// Triangles List) and "Data Type" in bits 4-7 (`1` = 16-bit
    /// index) — `2 | (1 << 4) = 0x12`. Fixed for this demo: it's what
    /// every draw in the real capture used, indexed or not.
    fn primitive_list_format(&mut self) {
        self.push_u8(0x38);
        self.push_u8(0x12);
    }

    /// Opcode `0x66` (`VC4_PACKET_CLIP_WINDOW`, code 102). 9 bytes:
    /// four `u16` fields in order (left, bottom, width, height), all
    /// in pixels.
    fn clip_window(&mut self, params: &BclParams) {
        self.push_u8(0x66);
        self.push_u16(0); // left
        self.push_u16(0); // bottom
        self.push_u16(params.width_px);
        self.push_u16(params.height_px);
    }

    /// Opcode `0x60` (`VC4_PACKET_CONFIGURATION_BITS`, code 96). 4
    /// bytes: three data bytes. Byte 0: forward-facing enable (bit 0),
    /// reverse-facing enable (bit 1), clockwise primitives (bit 2),
    /// depth-offset enable (bit 3), AA points/lines (bit 4), coverage
    /// read type (bit 5), rasterizer oversample mode (bits 6-7). Byte
    /// 1: coverage pipe select (bit 0), coverage read mode (bit 3),
    /// depth-test function (bits 4-6, same values as
    /// `V3D_COMPARE_FUNC`/Gallium's `PIPE_FUNC_*`), Z-updates enable
    /// (bit 7). Byte 2: early-Z enable (bit 0), early-Z-updates enable
    /// (bit 1).
    ///
    /// Both-facing culling stays enabled (no back-face culling)
    /// regardless of `depth_test_enabled` — deliberately: this crate's
    /// real render-vertex-shader winding convention (`Clockwise
    /// Primitives = 1`, taken unchanged from the captures) has never
    /// been checked against a scene where it would matter, so getting
    /// culling wrong would make the *entire* model vanish rather than
    /// just look visually wrong.
    ///
    /// Both bytes come from real captures of the two configurations,
    /// rather than from reasoning about which features to switch on:
    ///
    /// - `depth_test_enabled`: `0x90, 0x03` — `LESS`, Z-updates on,
    ///   early-Z and early-Z-updates on. Captured from Mesa drawing a
    ///   depth-tested textured cube at 8x8 tiles.
    /// - Otherwise: `0x70, 0x02` — `ALWAYS`, Z-updates off, early-Z
    ///   off. Captured from a single flat triangle with no depth test.
    ///
    /// Early-Z was briefly forced off in *both* cases, on the grounds
    /// that the flat-triangle capture had it off and nothing justified
    /// turning it on. That used the wrong capture: that draw tested
    /// depth as `ALWAYS`, which makes early-Z irrelevant, so its value
    /// there said nothing about what a depth-tested draw should use. A
    /// capture of a genuinely depth-tested scene settles it — Mesa
    /// enables early-Z.
    fn configuration_bits(&mut self, params: &BclParams) {
        self.push_u8(0x60);
        self.push_u8(0x07);
        if params.depth_test_enabled {
            self.push_u8(0x90);
            self.push_u8(0x03);
        } else {
            self.push_u8(0x70);
            self.push_u8(0x02);
        }
    }

    /// Opcode `0x65` (`VC4_PACKET_DEPTH_OFFSET`, code 101). 5 bytes:
    /// two `u16` fields (factor, then units), both float-1-8-7 encoded
    /// (the top 16 bits of an `f32`) per the packet spec — irrelevant
    /// here since both are exactly `0` (no depth offset), and `0` is
    /// `0` in any float encoding.
    fn depth_offset(&mut self) {
        self.push_u8(0x65);
        self.push_u16(0); // factor
        self.push_u16(0); // units
    }

    /// Opcode `0x62` (`VC4_PACKET_POINT_SIZE`, code 98). 5 bytes: one
    /// `f32`. `1.0` — unused by triangles, but the real capture always
    /// set it.
    fn point_size(&mut self) {
        self.push_u8(0x62);
        self.push_f32(1.0);
    }

    /// Opcode `0x63` (`VC4_PACKET_LINE_WIDTH`, code 99). 5 bytes: one
    /// `f32`. `1.0` — same as `point_size`, unused by triangles.
    fn line_width(&mut self) {
        self.push_u8(0x63);
        self.push_f32(1.0);
    }

    /// Opcode `0x69` (`VC4_PACKET_CLIPPER_XY_SCALING`, code 105). 9
    /// bytes: two `f32` fields (half-width, then half-height), both in
    /// 1/16th-pixel units — so half the pixel width/height, times 16,
    /// goes in directly as a float. The height half is negated: Mesa's
    /// own emission code (`vc4_emit.c`) writes `vc4->viewport.scale[1]`
    /// here un-modified, and that scale is negative by construction —
    /// a Y-axis flip between GL's bottom-up convention and this
    /// hardware's top-down tile/scanout convention, a real Mesa
    /// decision to replicate, not a hardware quirk.
    fn clipper_xy_scaling(&mut self, params: &BclParams) {
        self.push_u8(0x69);
        self.push_f32(f32::from(params.width_px) / 2.0 * 16.0);
        self.push_f32(-(f32::from(params.height_px) / 2.0 * 16.0));
    }

    /// Opcode `0x6a` (`VC4_PACKET_CLIPPER_Z_SCALING`, code 106). 9
    /// bytes: two `f32` fields (scale, then offset) mapping clip-space
    /// Z (`-1..1`) to screen-space depth (`0..1`). `0.5`/`0.5` —
    /// standard scale-and-bias, matches the real capture exactly.
    fn clipper_z_scale_and_offset(&mut self) {
        self.push_u8(0x6a);
        self.push_f32(0.5); // scale
        self.push_f32(0.5); // offset
    }

    /// Opcode `0x67` (`VC4_PACKET_VIEWPORT_OFFSET`, code 103). 5 bytes:
    /// two fields (center X, then center Y), each a signed 12.4
    /// fixed-point `u16` (i.e. the pixel value times 16, per the
    /// packet spec's `s12.4` field type) — the same 1/16th-pixel unit
    /// `clipper_xy_scaling` uses, just applied to a narrower field.
    fn viewport_offset(&mut self, params: &BclParams) {
        self.push_u8(0x67);
        self.push_u16((f32::from(params.width_px) / 2.0 * 16.0) as u16);
        self.push_u16((f32::from(params.height_px) / 2.0 * 16.0) as u16);
    }

    /// Opcode `0x61` (`VC4_PACKET_FLAT_SHADE_FLAGS`, code 97). 5 bytes:
    /// one `u32` bitmask, one bit per varying that should use flat
    /// (not smooth) shading. Always `0` for this demo — the texcoord
    /// varying must interpolate smoothly.
    fn flat_shade_flags(&mut self) {
        self.push_u8(0x61);
        self.push_u32(0);
    }

    /// Opcode `0x40` (`VC4_PACKET_GL_SHADER_STATE`, code 64). 5 bytes:
    /// one `u32` word packing the shader-state record's address with
    /// two small fields into its low bits — the same low-bits-as-flags
    /// trick this crate's mailbox message word and DMA control-block
    /// alignment already rely on. Bits 0-2: number of attribute arrays
    /// — `1..=7` mean themselves, but `0` means *8*, not zero arrays
    /// (confirmed directly in Mesa's `vc4_draw.c`: `num_elements_emit &
    /// 0x7`, the same wraparound `&`-mask this method reproduces).
    /// This demo always uses 2, so the wraparound case is untested
    /// here, just implemented correctly for when it stops being fixed
    /// at 2. Bit 3: extended shader record flag (always `0` here —
    /// this demo doesn't use extended records). Bits 4-31: the
    /// address, meaning `shader_state_address` must be 16-byte
    /// aligned.
    fn gl_shader_state(&mut self, params: &BclParams) {
        debug_assert_eq!(
            params.shader_state_address & 0xf,
            0,
            "shader-state record must be 16-byte aligned"
        );
        debug_assert!(
            (1..=8).contains(&params.attribute_array_count),
            "attribute array count must be 1-8"
        );
        self.push_u8(0x40);
        let word =
            (params.shader_state_address & !0xf) | u32::from(params.attribute_array_count & 0x7);
        self.push_u32(word);
    }

    /// Opcode `0x20` (`VC4_PACKET_GL_INDEXED_PRIMITIVE`, code 32). 14
    /// bytes: one data byte packing "Primitive mode" in bits 0-3 (the
    /// standard OpenGL primitive-mode enum — `4` = `GL_TRIANGLES`) and
    /// "Index type" in bits 4-7 (`1` = 16-bit), then `u32` length,
    /// `u32` index-buffer bus address, and `u32` maximum index. `4 |
    /// (1 << 4) = 0x14`. `max_index` is computed by the caller from the
    /// real index buffer, not copied from the real capture — that
    /// capture's own `Maximum Index` field read back an implausible
    /// value (`43687`, for indices that only went up to `2`), almost
    /// certainly uninitialized memory in Mesa's own test path, not a
    /// value to replicate.
    fn indexed_primitive_list(&mut self, params: &BclParams) {
        self.push_u8(0x20);
        self.push_u8(0x14);
        self.push_u32(params.index_count);
        self.push_u32(params.index_buffer_address);
        self.push_u32(u32::from(params.max_index));
    }

    /// Opcode `0x07` (`VC4_PACKET_INCREMENT_SEMAPHORE`, code 7). 1 byte,
    /// opcode only. Missing entirely from this builder's first version
    /// — Mesa's own `VC4_DEBUG=cl` decoder doesn't print this packet
    /// (nor `flush`, immediately after it) by name, so neither showed
    /// up in the captures this whole module was cross-checked against,
    /// even though the real kernel driver requires both
    /// (`vc4_validate_bin_cl` in `vc4_validate.c`: "Bin CL missing
    /// VC4_PACKET_INCREMENT_SEMAPHORE + VC4_PACKET_FLUSH"). Queues the
    /// semaphore increment [`crate::v3d::rcl`]'s `Wait on Semaphore`
    /// blocks on — actually incrementing it is deferred to `flush`,
    /// immediately after (see that method's doc comment).
    ///
    /// Must be the second-to-last byte of the whole binning control
    /// list — `vc4_validate.c`'s `validate_increment_semaphore` checks
    /// this packet starts at exactly `bin_cl_size - 2`.
    fn increment_semaphore(&mut self) {
        self.push_u8(0x07);
    }

    /// Opcode `0x04` (`VC4_PACKET_FLUSH`, code 4). 1 byte, opcode only —
    /// must be the very last byte of the whole binning control list
    /// (`vc4_validate.c`'s `validate_flush` checks this packet starts
    /// at exactly `bin_cl_size - 1`). Real hardware behavior observed
    /// on a Pi 3 before this packet was added: the binner does write
    /// real primitive data into the tile allocation memory (confirmed
    /// via `V3D_BPCA` advancing well past
    /// [`BclParams::tile_alloc_address`]), but the render pass's `Wait
    /// on Semaphore` never unblocks — matching Broadcom's reference
    /// guide exactly: `Increment Semaphore` only actually fires "after
    /// tile lists are flushed or last tile written", and `V3D_BFC`
    /// (the PTB's own flush-completion counter, not just the
    /// control-list executor reaching this opcode) never increments
    /// without a real `Flush`. `vc4_validate.c`'s own comment
    /// independently confirms the same mechanism from the kernel's
    /// side: "the FLUSH is what caps the bin lists with
    /// `VC4_PACKET_RETURN_FROM_SUB_LIST` ... and actually triggers the
    /// queued semaphore increment."
    fn flush(&mut self) {
        self.push_u8(0x04);
    }
}

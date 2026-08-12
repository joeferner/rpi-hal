//! Fragment shader texture uniforms — the two config words
//! (`tex_p0[0]`/`tex_p1[0]`) that make up the *entire* uniform stream
//! the captured fragment shader fetches, per its QIR dump. Fixed for
//! this demo's exact texture setup: `GL_RGBA`/`GL_UNSIGNED_BYTE`,
//! `GL_NEAREST` filtering both directions, no mipmaps, default
//! `GL_REPEAT` wrap — the same configuration the GLES program that
//! produced that shader used, since that is what compiled the
//! fragment shader this builder's output must match.
//!
//! Field layout is `vc4_packet.h`'s `VC4_TEX_P0_*`/`VC4_TEX_P1_*`
//! macros (saved conceptually alongside `vc4_packet.xml`, though this
//! particular header has no XML counterpart — it's C-only), with the
//! actual field *values* cross-checked against Mesa's real assignment
//! code (`vc4_state.c`'s texture state setup) rather than derived from
//! the macros alone:
//!
//! - `WIDTH`/`HEIGHT` are literal pixel dimensions masked to 11 bits
//!   (`width0 & 2047`) — not a "size minus one" encoding, confirmed by
//!   reading the real assignment rather than assuming either
//!   convention.
//! - `OFFSET` is the texture data's bus address, shifted right by 12
//!   — meaning the texture buffer must be 4096-byte aligned. This is
//!   coarser than anything else in this crate's V3D pipeline (the
//!   16/32/64-byte alignments other packets need) and
//!   [`crate::v3d::GpuBuffer`]'s own `align(32)` is not sufficient on
//!   its own — whatever allocates real texture data (Phase 7) needs
//!   its own stronger alignment, not just a plain `GpuBuffer`.
//! - Filter/wrap values resolve through small lookup tables in
//!   `vc4_state.c` down to `1` (nearest) and `0` (repeat)
//!   respectively for this exact GL setup — read directly from that
//!   code, not guessed from the macro names.

/// Runtime parameters for [`build_fragment_shader_uniforms`].
pub struct TextureParams {
    /// Bus address of the texture's pixel data (`GL_RGBA`,
    /// `GL_UNSIGNED_BYTE`, tightly packed rows). Must be 4096-byte
    /// aligned — see this module's doc comment.
    pub address: u32,
    /// Texture width in pixels. Must fit in 11 bits (`0..=2047`).
    pub width_px: u16,
    /// Texture height in pixels. Must fit in 11 bits (`0..=2047`).
    pub height_px: u16,
}

/// Builds the fragment shader's two-word texture uniform stream into
/// `cl`, returning the number of bytes written (always 8). Panics (via
/// slice indexing) if `cl` is too small.
pub fn build_fragment_shader_uniforms(cl: &mut [u8], params: &TextureParams) -> usize {
    debug_assert_eq!(
        params.address & 0xfff,
        0,
        "texture data must be 4096-byte aligned"
    );
    debug_assert!(
        params.width_px <= 2047 && params.height_px <= 2047,
        "texture dimensions must fit in 11 bits"
    );

    // P0: offset (address >> 12) in the high 20 bits; color swizzle,
    // cube-map mode, and Y-flip all `0`; type `0` (RGBA8888, low 4
    // bits of `VC4_TEX_P0_TYPE`); 0 mip levels beyond the base.
    let p0 = params.address & !0xfff;
    // P1: type4 bit `0` (RGBA8888 fits in the low 4 type bits, so its
    // 5th bit is 0); height in bits 20-30; ETC1-flip `0`; width in
    // bits 8-18; mag filter `1` (nearest, bit 7); min filter `1`
    // (nearest, no mipmap, bits 4-6); wrap S/T both `0` (repeat, bits
    // 0-3).
    let p1 = (u32::from(params.height_px) << 20)
        | (u32::from(params.width_px) << 8)
        | (1 << 7)
        | (1 << 4);

    cl[0..4].copy_from_slice(&p0.to_le_bytes());
    cl[4..8].copy_from_slice(&p1.to_le_bytes());
    8
}

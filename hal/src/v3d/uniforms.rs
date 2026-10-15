//! Uniform stream builders for the coordinate and vertex shaders.
//!
//! Grounded directly in Mesa's QIR and scheduled QPU disassembly, as
//! dumped by `VC4_DEBUG=qir,qpu` on a Pi 3. QIR tags every uniform
//! read with its index (`unif[N]`) — information the final QPU
//! disassembly doesn't preserve — while the scheduled listing is what
//! shows the order those reads are actually issued in.
//!
//! The stream is the matrix reordered into the sequence the shader
//! actually reads its uniforms in — *not* natural column-major order —
//! followed by four compiler-injected viewport values.
//!
//! The QPU reads uniforms through `uni`, a sequential FIFO: the Nth
//! read pops stream slot N. So the stream order is whatever order the
//! *scheduled* code issues its `uni` reads in — which is neither plain
//! matrix order nor QIR's instruction order, since scheduling reorders
//! both.
//!
//! Both permutations below are therefore derived from the scheduled
//! QPU disassembly directly, by tracing every `uni`
//! read to the accumulator it feeds and identifying each accumulator
//! from what the shader ultimately does with it. Two landmarks make
//! that unambiguous without having to trust any earlier reading:
//!
//! - The value moved to `sfu_recip` is clip `W`, so its four uniforms
//!   are `mvp[3]`, `mvp[7]`, `mvp[11]`, `mvp[15]`.
//! - The three accumulators later multiplied by `vp_x_scale`,
//!   `vp_y_scale` and `vp_z_scale` are clip `X`, `Y` and `Z`, giving
//!   the remaining three rows.
//!
//! Worth stating plainly, since this module got it wrong twice: the
//! `unif[N]` tags printed in the *pre-scheduling* QIR are element
//! identities, not stream slots, and QIR instruction order is not
//! stream order either. Only the scheduled listing settles it. The
//! failure mode is also worth recording — a wrong permutation puts the
//! wrong element where the shader expects `mvp[15]`, which makes clip
//! `W` come out `0`, `rcp(0)` infinite, and every vertex land at
//! infinite screen coordinates. That shows up not as distorted
//! geometry but as the binner discarding every primitive as "outside
//! the viewport" (V3D performance counter source `10`), with every
//! control-list register still reporting a clean, successful bin.
//!
//! The four trailing uniforms (QIR names them
//! `vp_x_scale`/`vp_y_scale`/`vp_z_scale`/`vp_z_offset`) are not part
//! of the GL uniform array at all. They must equal the same numbers
//! [`crate::v3d::bcl`]'s `clipper_xy_scaling`/
//! `clipper_z_scale_and_offset` write into the binning control list for
//! the same frame — computed here with the same formula rather than
//! taken as a separate parameter, so the two can't drift out of sync.

/// Number of `f32` uniforms each of [`build_coordinate_shader_uniforms`]
/// and [`build_vertex_shader_uniforms`] writes: 16 matrix elements
/// plus 4 viewport values.
pub const UNIFORM_COUNT: usize = 20;

/// Builds the coordinate shader's uniform stream (`prog 0/4`'s fetch
/// order) into `cl`, returning the number of bytes written
/// (`UNIFORM_COUNT * 4`). `mvp` is the MVP matrix as a plain
/// column-major array (`glUniformMatrix4fv`'s own layout — column 0 in
/// `mvp[0..4]`, column 1 in `mvp[4..8]`, and so on). `width_px`/
/// `height_px` must match the render target
/// [`crate::v3d::bcl::BclParams`] uses for the same frame, since the
/// viewport values derived from them must agree with what the binning
/// control list's own clipper packets use.
///
/// Stream order, in terms of `mvp`'s column-major indices: `0, 4, 1,
/// 7, 5, 3, 6, 2, 8, 11, 9, 10, 15, 12, 13, 14`, then `vp_x_scale`,
/// `vp_y_scale`, `vp_z_scale`, `vp_z_offset` — see this module's
/// documentation for how this was traced out of the scheduled code.
pub fn build_coordinate_shader_uniforms(
    cl: &mut [u8],
    mvp: &[f32; 16],
    width_px: u16,
    height_px: u16,
) -> usize {
    let (vp_x_scale, vp_y_scale, vp_z_scale, vp_z_offset) = viewport_values(width_px, height_px);
    let mut b = Builder { bytes: cl, len: 0 };
    for &i in &[0, 4, 1, 7, 5, 3, 6, 2, 8, 11, 9, 10, 15, 12, 13, 14] {
        b.push_f32(mvp[i]);
    }
    b.push_f32(vp_x_scale);
    b.push_f32(vp_y_scale);
    b.push_f32(vp_z_scale);
    b.push_f32(vp_z_offset);
    b.len
}

/// Builds the vertex (render-pass) shader's uniform stream (`prog
/// 0/3`'s fetch order) into `cl`, returning the number of bytes
/// written. Same parameters and viewport-value derivation as
/// [`build_coordinate_shader_uniforms`], but *both* the matrix order
/// and the viewport-value order differ — Mesa scheduled this stage's
/// uniform reads into a different sequence.
///
/// Stream order, in terms of `mvp`'s column-major indices: `3, 7, 4,
/// 0, 5, 1, 6, 2, 11, 8, 9, 10, 15, 12, 13, 14`, then `vp_y_scale`,
/// `vp_x_scale`, `vp_z_scale`, `vp_z_offset` — note the first two are
/// swapped relative to the coordinate shader.
///
/// That swap is not a guess. In this stage's scheduled code the packed
/// screen-coordinate word is built as `ftoi ra0.16a` from
/// `ra2 * U18 / W` and `ftoi ra0.16b` from `r0 * U17 / W`; `.16a` is
/// bits 15:0, which Figure 10 defines as `Xs`, and `.16b` is bits
/// 31:16, `Ys`. So the 18th uniform scales X and the 17th scales Y.
/// The coordinate shader is the other way round — there, `U17`
/// multiplies `rb6`, which is the first VPM write and therefore `Xc`.
///
/// Getting this wrong exchanges the rendered X and Y, which mirrors
/// the image about its diagonal: a cube spinning left-to-right renders
/// as one spinning top-to-bottom. It also corrupts the depth values,
/// so early-Z rejects most fragments and the cube comes out in pieces.
pub fn build_vertex_shader_uniforms(
    cl: &mut [u8],
    mvp: &[f32; 16],
    width_px: u16,
    height_px: u16,
) -> usize {
    let (vp_x_scale, vp_y_scale, vp_z_scale, vp_z_offset) = viewport_values(width_px, height_px);
    let mut b = Builder { bytes: cl, len: 0 };
    for &i in &[3, 7, 4, 0, 5, 1, 6, 2, 11, 8, 9, 10, 15, 12, 13, 14] {
        b.push_f32(mvp[i]);
    }
    // Y before X here, unlike the coordinate shader -- see this
    // function's doc comment for the disassembly that establishes it.
    b.push_f32(vp_y_scale);
    b.push_f32(vp_x_scale);
    b.push_f32(vp_z_scale);
    b.push_f32(vp_z_offset);
    b.len
}

/// `(vp_x_scale, vp_y_scale, vp_z_scale, vp_z_offset)` — must match
/// [`crate::v3d::bcl`]'s `clipper_xy_scaling`/
/// `clipper_z_scale_and_offset`, so computed with the identical
/// formula rather than passed in separately.
fn viewport_values(width_px: u16, height_px: u16) -> (f32, f32, f32, f32) {
    let vp_x_scale = f32::from(width_px) / 2.0 * 16.0;
    let vp_y_scale = -(f32::from(height_px) / 2.0 * 16.0);
    (vp_x_scale, vp_y_scale, 0.5, 0.5)
}

/// Raw serialization cursor, same shape as this crate's other `v3d`
/// builders.
struct Builder<'a> {
    bytes: &'a mut [u8],
    len: usize,
}

impl<'a> Builder<'a> {
    fn push_f32(&mut self, v: f32) {
        self.bytes[self.len..self.len + 4].copy_from_slice(&v.to_bits().to_le_bytes());
        self.len += 4;
    }
}

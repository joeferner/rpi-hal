//! Shader-state record + attribute records: the small block of memory
//! `bcl.rs`'s `GL Shader State` packet points to, describing which
//! compiled shaders to run and how to read vertex attributes into
//! them. Fixed for this crate's one cube shape (two interleaved
//! attributes — `vec4` position, `vec2` texcoord — matching the GLSL
//! the captured shaders were compiled from), not a general
//! N-attribute builder.
//!
//! Layout comes from `vc4_packet.xml`'s `Shader Record`/`Attribute
//! Record` structs, cross-checked against the real field usage in
//! Mesa's `vc4_draw.c` (`cl_emit(&job->shader_rec, SHADER_RECORD,
//! ...)`/`cl_emit(&job->shader_rec, ATTRIBUTE_RECORD, ...)`). The XML
//! has one real error, caught by cross-referencing it against
//! itself: "Vertex Shader
//! Uniforms Address" is declared at the same byte offset as "Vertex
//! Shader Code Address" (`start="16b"` for both), which can't be
//! right — the Fragment and Coordinate blocks each space their own
//! code/uniforms address pair 4 bytes apart, and only the Vertex block
//! breaks that pattern. Implemented here at the pattern-consistent
//! offset (byte 20) instead of the XML's literal text.
//!
//! Two fields Mesa always hardcodes the same way regardless of
//! program, reproduced here rather than exposed as parameters: `Enable
//! Clipping` (always on) and `Fragment Shader is single threaded`
//! (forced on — Mesa sets this from whether the compiler judged the
//! real shader eligible for multi-threading, which this crate has no
//! way to determine for hand-extracted QPU code; single-threaded is
//! always a valid, if potentially slower, choice).
//!
//! There is no kernel here to fill in the three "Uniforms Address"
//! fields the way Linux's `vc4` driver does (`vc4_packet.xml` notes
//! each one "set up by the kernel") — this builder takes them as
//! plain parameters and writes them directly, same as every other
//! address in this crate's control lists.

/// Runtime parameters for [`build`].
pub struct ShaderRecordParams {
    /// Bus address of the fragment shader's compiled QPU code
    /// (`MESA_SHADER_FRAGMENT` in the shader capture).
    pub fragment_shader_code_address: u32,
    /// Bus address of the fragment shader's uniform stream.
    pub fragment_shader_uniforms_address: u32,
    /// Number of varyings the fragment shader reads. The capture shows
    /// it reading the texcoord varying via two `vary` operand reads
    /// (one per component), but whether this field counts varying
    /// *components* or varying *slots* isn't confirmed — left as an
    /// explicit parameter rather than a hardcoded guess.
    pub fragment_shader_num_varyings: u8,
    /// Bus address of the vertex (render-pass) shader's compiled QPU
    /// code (`MESA_SHADER_VERTEX prog 0/3` in that capture).
    pub vertex_shader_code_address: u32,
    /// Bus address of the vertex shader's uniform stream.
    pub vertex_shader_uniforms_address: u32,
    /// Bus address of the coordinate (binning-pass) shader's compiled
    /// QPU code (`MESA_SHADER_COORD prog 0/4` in that capture).
    pub coordinate_shader_code_address: u32,
    /// Bus address of the coordinate shader's uniform stream.
    pub coordinate_shader_uniforms_address: u32,
    /// Bus address of the interleaved vertex buffer: `vec4` position
    /// immediately followed by `vec2` texcoord, repeated per vertex,
    /// stride 24 bytes — the layout the captured shaders were compiled
    /// against.
    pub vertex_buffer_address: u32,
}

/// Builds the shader-state record followed immediately by its two
/// attribute records (52 bytes total) into `cl`, returning the number
/// of bytes written. Both `Attribute Record`s must sit right after the
/// `Shader Record` in memory — that's the actual layout Mesa's own
/// driver relies on (`job->shader_rec` is one buffer both are emitted
/// into back to back), not an arbitrary choice made here. Panics (via
/// slice indexing) if `cl` is too small, same rationale as
/// [`crate::v3d::bcl::build`].
pub fn build(cl: &mut [u8], params: &ShaderRecordParams) -> usize {
    let mut b = Builder { bytes: cl, len: 0 };
    b.shader_record(params);
    // Attribute 0: position, vec4 (16 bytes), offset 0 in the
    // interleaved vertex. Read by both the coordinate shader (which
    // needs position for the MVP transform) and the vertex shader.
    b.attribute_record(
        params.vertex_buffer_address,
        16,
        24,
        0, // vertex shader VPM offset: position is read first.
        0, // coordinate shader VPM offset: likewise first (and only).
    );
    // Attribute 1: texcoord, vec2 (8 bytes), offset 16 in the
    // interleaved vertex. Read by the vertex shader only — the
    // coordinate shader's `select_bits` (below) excludes it, since
    // the shader capture showed the coordinate shader reading only 4
    // `vpm_read`s (position's 4 floats), never touching texcoord.
    b.attribute_record(
        params.vertex_buffer_address + 16,
        8,
        24,
        16, // vertex shader VPM offset: right after position.
        0,  // coordinate shader VPM offset: unused, not selected.
    );
    b.len
}

/// Raw serialization cursor, same shape as [`crate::v3d::bcl`]'s and
/// [`crate::v3d::rcl`]'s own `Builder`s.
struct Builder<'a> {
    bytes: &'a mut [u8],
    len: usize,
}

impl<'a> Builder<'a> {
    fn push_u8(&mut self, v: u8) {
        self.bytes[self.len] = v;
        self.len += 1;
    }

    fn push_u32(&mut self, v: u32) {
        self.bytes[self.len..self.len + 4].copy_from_slice(&v.to_le_bytes());
        self.len += 4;
    }

    fn skip(&mut self, n: usize) {
        self.bytes[self.len..self.len + n].fill(0);
        self.len += n;
    }

    /// 36 bytes. Byte 0: three flag bits (fragment shader single
    /// threaded, point size included — always `false`, this demo
    /// never draws points, enable clipping — always `true`, matching
    /// `vc4_draw.c`). Bytes 1-2 unused (a dead "number of uniforms"
    /// field Broadcom's hardware never reads, per the packet spec's
    /// own "not used currently" note; left zeroed rather than given a
    /// real value). Byte 3: fragment shader varying count. Bytes 4-7:
    /// fragment shader code address. Bytes 8-11: fragment shader
    /// uniforms address. Bytes 12-13 unused (same dead field, for the
    /// vertex shader this time). Byte 14: vertex shader attribute
    /// select bits (`0b11` — both attributes live, per the shader
    /// capture showing 6 `vpm_read`s). Byte 15: vertex shader total
    /// attribute size (`24` bytes — both attributes). Bytes 16-19:
    /// vertex shader code address. Bytes 20-23: vertex shader uniforms
    /// address (byte 20, not the XML's literal `16b` — see this
    /// module's doc comment). Bytes 24-25 unused (dead field, for the
    /// coordinate shader). Byte 26: coordinate shader attribute select
    /// bits (`0b01` — position only, per the shader capture showing 4
    /// `vpm_read`s). Byte 27: coordinate shader total attribute size
    /// (`16` bytes — position only). Bytes 28-31: coordinate shader
    /// code address. Bytes 32-35: coordinate shader uniforms address.
    fn shader_record(&mut self, params: &ShaderRecordParams) {
        self.push_u8(0b0000_0101); // single-threaded (bit 0) + enable clipping (bit 2).
        self.skip(2); // dead "FS number of uniforms" field.
        self.push_u8(params.fragment_shader_num_varyings);
        self.push_u32(params.fragment_shader_code_address);
        self.push_u32(params.fragment_shader_uniforms_address);
        self.skip(2); // dead "VS number of uniforms" field.
        self.push_u8(0b0000_0011); // both attributes live.
        self.push_u8(24); // both attributes' total size.
        self.push_u32(params.vertex_shader_code_address);
        self.push_u32(params.vertex_shader_uniforms_address);
        self.skip(2); // dead "CS number of uniforms" field.
        self.push_u8(0b0000_0001); // position only.
        self.push_u8(16); // position's size only.
        self.push_u32(params.coordinate_shader_code_address);
        self.push_u32(params.coordinate_shader_uniforms_address);
    }

    /// 8 bytes: address, number of bytes minus 1, stride, vertex
    /// shader VPM offset, coordinate shader VPM offset — all plain,
    /// byte-aligned fields, no packing ambiguity.
    fn attribute_record(
        &mut self,
        address: u32,
        bytes: u8,
        stride: u8,
        vertex_shader_vpm_offset: u8,
        coordinate_shader_vpm_offset: u8,
    ) {
        self.push_u32(address);
        self.push_u8(bytes - 1);
        self.push_u8(stride);
        self.push_u8(vertex_shader_vpm_offset);
        self.push_u8(coordinate_shader_vpm_offset);
    }
}

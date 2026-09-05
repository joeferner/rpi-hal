//! Picks the memory map for the fixture board this build targets.
//!
//! The two chips cannot share one `memory.x`: the RP2040 reserves the first
//! 256 bytes of flash for its second-stage loader, the RP2350 has twice the
//! RAM and needs an extra section for the boot ROM's image-definition block.
//!
//! Copying the chosen one into `OUT_DIR` and putting that on the link search
//! path is how `cortex-m-rt`'s `link.x` finds it. A `memory.x` sitting in the
//! package root would be found too — that is how this crate used to work — but
//! only one file can have that name, which is the thing that has to change.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let rp2040 = env::var_os("CARGO_FEATURE_RP2040").is_some();
    let rp235x = env::var_os("CARGO_FEATURE_RP235X").is_some();

    // Both at once links an image with one chip's memory map and the other's
    // register access, which is a runtime mystery rather than a build error,
    // so it is worth refusing here.
    let source = match (rp2040, rp235x) {
        (true, false) => "memory-rp2040.x",
        (false, true) => "memory-rp235x.x",
        (false, false) => panic!("no fixture board selected: enable `rp2040` or `rp235x`"),
        (true, true) => panic!("`rp2040` and `rp235x` are alternatives, not a pair"),
    };

    let out = PathBuf::from(env::var_os("OUT_DIR").expect("cargo sets OUT_DIR"));
    fs::copy(source, out.join("memory.x")).expect("copy the memory map into OUT_DIR");
    println!("cargo:rustc-link-search={}", out.display());

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=memory-rp2040.x");
    println!("cargo:rerun-if-changed=memory-rp235x.x");
}

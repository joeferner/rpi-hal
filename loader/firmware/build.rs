use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    // The two architectures load at different addresses (0x8000 for
    // AArch32, 0x80000 for AArch64), so each has its own linker script.
    // Copy the matching one to a fixed name the link arg below refers to.
    let arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap();
    let script = if arch == "aarch64" {
        "linker64.ld"
    } else {
        "linker.ld"
    };
    let linker_path = out_dir.join("linker.ld");
    fs::copy(script, &linker_path).unwrap();

    // Pass the script by absolute path rather than `-Tlinker.ld` + a
    // link-search dir: rpi-hal's build script also emits a link-search
    // path containing a file named `linker.ld` (its own, at 0x8000), and
    // that path propagates into this binary's link. A bare `-Tlinker.ld`
    // resolves by search order and can silently pick up rpi-hal's script
    // instead of ours — harmless while both used 0x8000, but wrong the
    // moment ours moved to 0x80000 for AArch64. An absolute path can't
    // be shadowed.
    println!("cargo:rustc-link-arg=-T{}", linker_path.display());
    println!("cargo:rerun-if-changed=linker.ld");
    println!("cargo:rerun-if-changed=linker64.ld");
    println!("cargo:rerun-if-changed=src/boot.s");
    println!("cargo:rerun-if-changed=src/boot64.s");
}

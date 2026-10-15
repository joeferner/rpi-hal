use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    // The two architectures load at different addresses -- 0x8000 for a
    // 32-bit kernel7.img, 0x80000 for a 64-bit kernel8.img (the firmware's
    // default when `arm_64bit=1`). Link each at its own load address so the
    // image direct-boots off the SD card (link address == load address);
    // pick the matching script by target arch.
    let arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap();
    let script = if arch == "aarch64" {
        "linker64.ld"
    } else {
        "linker.ld"
    };

    // `cfg(armv6)`: the ARM1176JZF-S in the BCM2835 (Pi 1, Pi Zero) rather
    // than the ARMv7-A cores of every later 32-bit Pi. It decides which
    // boot assembly is assembled, how the barriers are spelled, and which
    // CPU-state drivers exist at all -- none of which is a choice a
    // consumer should be able to get wrong, so it is derived from the
    // target rather than exposed as a Cargo feature.
    //
    // The test is on the target *triple*, and it has to be. The natural
    // discriminator is `target_feature = "v7"`, which is exactly what
    // distinguishes the two architectures -- but `v7` is not a stabilized
    // target feature name, so *stable* rustc does not report it at all:
    // not in `cfg` at a use site, and not in a build script's
    // `CARGO_CFG_TARGET_FEATURE`. `nightly` does, which is the trap,
    // because this repository pins nightly while consumers build on
    // stable. A predicate resting on it reads as "no v7" for every target
    // there, quietly turning every ARMv7 build into an ARMv6 one: ARMv6
    // barriers, no `generic_timer`, `core_id` hardcoded to 0, and
    // `boot6.s` -- which has no Hyp-mode drop -- on a Pi 3.
    //
    // `TARGET` is always set, on any toolchain, and its `armv6` prefix
    // covers both the soft- and hard-float spellings without matching
    // `thumbv6m` (Cortex-M, also `target_arch = "arm"`, and not something
    // this crate supports).
    //
    // Emitted here rather than written at each use because there is a
    // site for it in every architecture-specific file, and one that got
    // the predicate subtly wrong would be missed.
    println!("cargo::rustc-check-cfg=cfg(armv6)");
    let target = env::var("TARGET").unwrap();
    if arch == "arm" && target.starts_with("armv6") {
        println!("cargo::rustc-cfg=armv6");
    }

    // Pass the script by absolute path rather than a bare `-Tlinker.ld` +
    // link-search dir. A bare name is resolved from the linker's working
    // directory (the crate root) first, where `linker.ld` (0x8000) lives,
    // so it would silently shadow the arch-selected script -- harmless
    // while both were 0x8000, but wrong once linker64.ld moved to 0x80000.
    // An absolute path can't be shadowed.
    //
    // This applies only to this crate's own targets (its examples and
    // tests): `rustc-link-arg` does not reach a downstream package's
    // binaries. Consumers are served by the copy published below instead.
    let script_path = manifest_dir.join(script);
    println!("cargo:rustc-link-arg=-T{}", script_path.display());

    // Publish the same script for downstream binaries, which need one
    // providing the symbols `rt`'s boot code links against (`_start`,
    // `__bss_start`, `__bss_end`) or they fail to link with those
    // undefined. `rustc-link-search` *does* reach a dependent's link step,
    // so a consumer names the script with a bare `-T` and the linker
    // resolves it out of this directory:
    //
    //     # .cargo/config.toml
    //     [target.aarch64-unknown-none-softfloat]
    //     rustflags = ["-C", "link-arg=-Trpi-link.x"]
    //
    // No build script of their own and no copied file, which is the point:
    // a copy silently keeps whatever this crate's boot code expected on the
    // day it was copied (the `ALIGN(4)` fix in these scripts would have had
    // to be made in every one of them), while this tracks the version of
    // the crate actually being compiled against.
    //
    // One canonical name for both architectures, rather than exposing
    // `linker.ld`/`linker64.ld` under their own names, so the consumer's
    // `-T` line is the same whichever target they build for -- the arch is
    // already decided here. The `.x` extension follows the convention other
    // `rt` crates in this ecosystem use for a crate-provided linker script,
    // and keeps the published name from colliding with a `linker.ld` a
    // consumer may still have sitting in their own crate root.
    fs::copy(&script_path, out_dir.join("rpi-link.x")).unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());

    println!("cargo:rerun-if-changed=linker.ld");
    println!("cargo:rerun-if-changed=linker64.ld");
    println!("cargo:rerun-if-changed=src/boot.s");
    println!("cargo:rerun-if-changed=src/boot64.s");
}

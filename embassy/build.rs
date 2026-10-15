use std::env;

fn main() {
    // `cfg(armv6)`: the ARM1176JZF-S (Pi 1, Pi Zero) rather than the
    // ARMv7-A cores of every later 32-bit Pi. One line in this crate
    // depends on it -- the pender's barrier in `src/executor.rs`, which
    // ARMv6 has only as a CP15 operation.
    //
    // The test is on the target *triple*, and it has to be. The natural
    // spelling is `cfg(all(target_arch = "arm", not(target_feature =
    // "v7")))` written at the use site, with no build script at all --
    // but `v7` is not a stabilized target feature name, so *stable* rustc
    // does not report it in `cfg` (nor in a build script's
    // `CARGO_CFG_TARGET_FEATURE`). The predicate then reads as "no v7" on
    // every target, and an ARMv7-A build quietly compiles the ARMv6 arm.
    // Which builds, because the CP15 form still assembles for ARMv7 with
    // a deprecation warning -- so the only sign is a warning nobody reads.
    // This crate builds on stable for consumers even though its own
    // repository pins nightly, so the difference is not theoretical.
    //
    // `TARGET` is always set, on any toolchain, and its `armv6` prefix
    // covers both the soft- and hard-float spellings without matching
    // `thumbv6m` (Cortex-M, also `target_arch = "arm"`).
    println!("cargo::rustc-check-cfg=cfg(armv6)");
    let target = env::var("TARGET").unwrap();
    if env::var("CARGO_CFG_TARGET_ARCH").unwrap() == "arm" && target.starts_with("armv6") {
        println!("cargo::rustc-cfg=armv6");
    }
}

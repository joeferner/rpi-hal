#!/usr/bin/env bash
# Builds the `fpu_demo` example for AArch32 against the HARD-FLOAT target
# (armv7a-none-eabihf) so its float math compiles to real VFP/NEON
# instructions instead of the soft-float `compiler_builtins` calls the
# crate's default soft-float target (armv7a-none-eabi) would emit.
#
# Only this example is built this way; every other build stays soft-float.
# rpi-loader is unaffected -- it just loads and jumps to the image, and the
# FPU is turned on in rpi-hal's boot path before any of this code runs.
#
# After building it disassembles the result and greps for FP opcodes, so a
# successful run is itself the proof the compiler emitted hardware FP.
set -euo pipefail

# Neither of rpi-hal's chip features is a default (see its Cargo.toml) --
# `bcm2837` here since this example targets Pi 2/3 unless told otherwise.
chip="${1:-bcm2837}"
target="armv7a-none-eabihf"

cd "$(dirname "$0")/.."

build_args=(--example fpu_demo --release --target "$target" --features "$chip")

cargo build "${build_args[@]}"
cargo objcopy "${build_args[@]}" -- -O binary target/kernel7.img

echo
echo "Built target/kernel7.img (AArch32 hard-float)."
echo "Copy it to the SD card boot partition to run on a Pi 2."
echo
echo "=== FP opcodes emitted (VFP/NEON) ==="
# `--mcpu=cortex-a7` (the BCM2836's core) so llvm-objdump decodes VFP
# instructions instead of printing them as `<unknown>` -- without it the
# grep below sees nothing even though the ops are there.
# `|| true` so an unexpected empty match still shows the count line below
# rather than aborting under `set -e`.
fp=$(cargo objdump "${build_args[@]}" -- -d --no-show-raw-insn --mcpu=cortex-a7 2>/dev/null |
    grep -iE '\b(vadd|vsub|vmul|vdiv|vmla|vmls|vfma|vsqrt|vcvt|vmov)\.(f32|f64|s32|u32|i32)\b' || true)
if [ -z "$fp" ]; then
    echo "NONE found -- something is wrong (soft-float fallback?)." >&2
    exit 1
fi
echo "$fp" | sort | uniq -c | sort -rn
echo
echo "total FP instructions: $(echo "$fp" | grep -c .)"

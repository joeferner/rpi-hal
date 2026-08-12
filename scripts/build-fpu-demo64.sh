#!/usr/bin/env bash
# Builds the `fpu_demo` example for AArch64 against the HARD-FLOAT target
# (aarch64-unknown-none) so its float math compiles to real FP/SIMD
# instructions instead of the soft-float `compiler_builtins` calls the
# crate's default target (aarch64-unknown-none-softfloat) would emit.
#
# The AArch64 counterpart to build-fpu-demo.sh -- see that script's header
# for the rationale (only this example is hard-float; rpi-loader unaffected;
# the FPU is enabled in boot64.s before any of this code runs).
set -euo pipefail

# See build-fpu-demo.sh's identical default/caveat comment.
chip="${1:-bcm2837}"
target="aarch64-unknown-none"

cd "$(dirname "$0")/.."

build_args=(--example fpu_demo --release --target "$target" --features "$chip")

cargo build "${build_args[@]}"
cargo objcopy "${build_args[@]}" -- -O binary target/kernel8.img

echo
echo "Built target/kernel8.img (AArch64 hard-float, linked at 0x80000)."
echo "Deploy either way:"
echo "  - SD card: copy target/kernel8.img to the boot partition (with"
echo "    arm_64bit=1 in config.txt) and it direct-boots, no loader."
echo "  - rpi-loader over UART, matching the 0x80000 link address:"
echo "    python3 <rpi-loader>/scripts/upload.py --load-addr 0x80000 <device> target/kernel8.img"
echo
echo "=== FP opcodes emitted (scalar FP + SIMD) ==="
fp=$(cargo objdump "${build_args[@]}" -- -d --no-show-raw-insn 2>/dev/null |
    grep -iwE 'fadd|fsub|fmul|fdiv|fmadd|fmsub|fnmadd|fsqrt|scvtf|ucvtf|fcvtzs|fcvtzu|fcvt|fmov' || true)
if [ -z "$fp" ]; then
    echo "NONE found -- something is wrong (soft-float fallback?)." >&2
    exit 1
fi
echo "$fp" | awk '{print $2}' | sort | uniq -c | sort -rn
echo
echo "total FP instructions: $(echo "$fp" | grep -c .)"

#!/usr/bin/env sh
# Builds the AArch64 test payload (test_payload64.s) into a raw binary
# that can be uploaded to the 64-bit loader to prove out the handoff:
#
# Paths below are relative to the repository root, one level above this
# package:
#
#   make build64-bcm2837                     # flash firmware/target/kernel8.img once
#   firmware/scripts/build_test_payload.sh   # -> firmware/target/test_payload64.bin
#   rpi-loader --device <device> boot --load-addr 0x80000 \
#       firmware/target/test_payload64.bin
#
# Uses clang (integrated assembler) plus the LLVM tools that ship with
# the Rust toolchain (rust-lld, rust-objcopy) — the same rust-objcopy
# `make build` already uses via `cargo objcopy` — so it needs no extra
# cross toolchain. Links at 0x80000, the AArch64 kernel load address.
set -eu

cd "$(dirname "$0")/.."
mkdir -p target

clang --target=aarch64-none-elf -c -o target/test_payload64.o scripts/test_payload64.s
rust-lld -flavor gnu --image-base=0x80000 -Ttext=0x80000 -e _start \
    -o target/test_payload64.elf target/test_payload64.o
rust-objcopy -O binary target/test_payload64.elf target/test_payload64.bin

echo "wrote target/test_payload64.bin"

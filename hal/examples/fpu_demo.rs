//! Hardware floating-point demo: does some ordinary `f32`/`f64` math and
//! prints the results over UART0.
//!
//! Unlike every other example, this one only exercises real hardware FP
//! when built against a **hard-float** target. The crate's default targets
//! (`armv7a-none-eabi` / `aarch64-unknown-none-softfloat`) are soft-float,
//! so a plain `cargo build --example fpu_demo` compiles fine but lowers the
//! arithmetic below to `compiler_builtins` software routines -- correct
//! results, but not a single VFP/NEON instruction. Build it for a
//! hard-float target to get hardware FP:
//!
//! ```text
//! scripts/build-fpu-demo.sh      # AArch32, armv7a-none-eabihf  -> kernel7.img
//! scripts/build-fpu-demo64.sh    # AArch64, aarch64-unknown-none -> kernel8.img
//! ```
//!
//! Each script also disassembles the result and greps for FP opcodes, so
//! you can confirm the compiler emitted `vadd.f*`/`vmul.f*` (AArch32) or
//! `fadd`/`fmul`/`fdiv`/`scvtf` (AArch64) rather than soft-float calls.
//!
//! The math is deliberately seeded from the System Timer's runtime counter
//! rather than compile-time constants: constant inputs would let the
//! optimizer fold the whole computation away at build time, leaving no FP
//! instructions to observe (and nothing for the hardware unit to do).

#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::halt;
use rpi_hal::timer::Timer;
use rpi_hal::{pac, uart::Uart};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);

    writeln!(uart, "FPU demo: hardware floating point").ok();

    // Runtime seeds off the microsecond counter. Reducing the 64-bit count
    // into a small non-zero range keeps the printed numbers readable and,
    // more importantly, keeps these genuinely runtime values so the math
    // below can't be constant-folded away.
    let now = timer.now_micros();
    let a = (now % 1000) as f32 + 1.5;
    let b = (now % 97) as f32 + 0.25;
    writeln!(uart, "seeds: a = {a}, b = {b}").ok();

    // Scalar f32: the four basic ops -> vadd/vsub/vmul/vdiv (.f32).
    writeln!(uart, "a + b = {}", a + b).ok();
    writeln!(uart, "a - b = {}", a - b).ok();
    writeln!(uart, "a * b = {}", a * b).ok();
    writeln!(uart, "a / b = {}", a / b).ok();

    // f64 to show double-precision codegen too, plus an int<->float round
    // trip (scvtf / fcvtzs on AArch64, the vcvt forms on AArch32).
    let x = a as f64 * 1.000_001_f64;
    let y = (x * 3.0 - 1.0) / 7.0;
    writeln!(uart, "f64: y = {y}").ok();
    writeln!(uart, "y as u32 = {}", y as u32).ok();

    // A short reduction: dot product of two runtime-seeded vectors. This
    // is the FP multiply-accumulate the software-mixing/DSP work this was
    // enabled for would lean on -- and a spot where a hard-float build may
    // fuse into vmla/fmadd.
    let u = [a, b, a - 1.0, b + 2.0];
    let v = [b, a, 0.5_f32, 1.25_f32];
    let mut dot = 0.0_f32;
    for i in 0..u.len() {
        dot += u[i] * v[i];
    }
    writeln!(uart, "dot(u, v) = {dot}").ok();

    // Newton-Raphson sqrt of `a`: iterative refinement using only + and *
    // and /, so it needs no `libm`/`core` transcendental support while
    // still being a real computation (each step is fmul/fadd/fdiv).
    let mut guess = a;
    for _ in 0..8 {
        guess = 0.5 * (guess + a / guess);
    }
    writeln!(uart, "sqrt(a) ~= {guess}").ok();

    writeln!(uart, "done").ok();
    halt();
}

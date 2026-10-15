// Self-relocating entry point (ARMv6).
//
// The ARM1176JZF-S counterpart to boot.s, which targets ARMv7-A. The job
// is identical and so is the layout it works against (linker.ld, load at
// 0x8000, run the relocated copy from 0x00200000) -- see boot.s's header
// for why the relocation exists at all and why linker.ld rather than
// hand-computed offsets is what makes the addresses come out right.
//
// Three differences, all of them forced by the core:
//
//   - No core-id check. `MPIDR` is an ARMv7 register, and the BCM2835 is
//     a uniprocessor part: there is no second core that could arrive
//     here, which is also why there is nothing here for a loaded kernel
//     to wake.
//   - `dsb` and `isb` are ARMv7 mnemonics. ARMv6 has the same two
//     barriers as CP15 operations, and they are just as necessary: the
//     relocated copy is written as data and then executed.
//   - `wfe` needs the assembler widened to ARMv6K below. rustc reports
//     `target_feature = "v6k"` for `armv6-none-eabi`, but the assembler
//     this file is handed to defaults to plain ARMv6 and rejects the
//     instruction.
//
// Everything else -- the copy loop, the stack, the .bss zeroing -- is
// ARMv6 instruction for instruction.

.arch armv6k

.equ RELOC_ADDR, 0x00200000

.section ".text.boot"
.global _start

_start:
    // Copy the relocatable part of the image from where firmware
    // physically loaded it (__reloc_src) to where it's linked to run
    // from (RELOC_ADDR), for exactly __reloc_size bytes -- both
    // computed by linker.ld.
    ldr     r0, =__reloc_src
    ldr     r1, =RELOC_ADDR
    ldr     r2, =__reloc_size
    mov     r3, #0
copy_loop:
    cmp     r3, r2
    bge     copy_done
    ldr     r4, [r0, r3]
    str     r4, [r1, r3]
    add     r3, r3, #4
    b       copy_loop
copy_done:

    // We just wrote the relocated copy as data; without a barrier the
    // core isn't guaranteed to fetch those bytes as instructions when
    // we jump there next. DSB (`c7, c10, 4`) waits for the writes to
    // complete, ISB (`c7, c5, 4`, "flush prefetch buffer") flushes the
    // pipeline so the next fetch actually sees them. Both take a
    // register operand that is ignored.
    mov     r5, #0
    mcr     p15, 0, r5, c7, c10, 4
    mcr     p15, 0, r5, c7, c5, 4

    // _reloc_start's address is already correct (0x00200000-based):
    // it's linked as part of the relocated .text section (see
    // linker.ld), not this boot stub.
    ldr     r0, =_reloc_start
    bx      r0

halt:
    wfe
    b       halt

.section ".text.reloc_start"
.global _reloc_start

_reloc_start:
    // Everything from here on executes out of RELOC_ADDR, so it's
    // safe to overwrite the original 0x8000 with the received kernel.
    ldr     sp, =__stack_top

    // __bss_start/__bss_end are already correct (0x00200000-based) --
    // .bss links as part of the same relocated region.
    ldr     r4, =__bss_start
    ldr     r9, =__bss_end
    mov     r5, #0
    mov     r6, #0
    mov     r7, #0
    mov     r8, #0
    b       2f
1:
    stmia   r4!, {{r5-r8}}
2:
    cmp     r4, r9
    blo     1b

    bl      kmain

halt2:
    wfe
    b       halt2

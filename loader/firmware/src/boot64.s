// Self-relocating entry point (AArch64).
//
// The AArch64 counterpart to boot.s. The GPU firmware loads a 64-bit
// image at 0x80000 (the AArch64 kernel load address, vs 0x8000 for
// AArch32) — selected by `arm_64bit=1` in config.txt. As with the
// 32-bit stub, only this tiny boot stub (_start) runs from there: it
// copies everything else (kmain, .rodata, .data — see linker64.ld) to
// 0x00200000 and jumps into that copy, since the kernel we're about to
// receive over UART also expects to run at 0x80000, and nothing can
// still be executing from (or referencing static data living at) that
// address range once we start receiving into it.
//
// Addressing is handled by linker64.ld, not manual offsets here: the
// same reasoning as boot.s applies — Rust-compiled code references its
// own static data via absolute addresses (here `ldr xN, =sym` literal
// loads), not PC-relative like our hand-written branches, so the
// linker links everything except this boot stub as if it already runs
// from 0x00200000, making every address correct as linked.
//
// Execution level: the firmware hands a 64-bit image control at the EL
// it chose (EL2 by default on BCM2837). We do NOT change EL here —
// jumping to the received kernel at the current EL faithfully mimics a
// direct firmware handoff, letting the loaded kernel perform whatever
// EL transition it wants, exactly as it would if booted from the SD
// card. So there is nothing EL-specific for this stub to do.
//
// No cache maintenance is done here: nothing in this project ever
// enables the MMU or caches, so there's no instruction-cache staleness
// hazard when we later jump into freshly-written memory.
//
// Single core by design: the GPU firmware only ever releases core 0 to
// this entry point. Cores 1-3 are held in the firmware's own stub,
// watching their ARM-local mailbox registers, and never reach this
// code at all -- so this loader needs to do nothing for them. A
// multi-core kernel loaded through here wakes them itself, straight out
// of that firmware stub, exactly as it would if the firmware had loaded
// that kernel directly.

// STACK_TOP is not a fixed offset here: the stack must clear the whole
// relocated region, which outgrew a fixed RELOC_ADDR + 0x10000 once the
// FAT/SD command code landed. linker64.ld exports `__stack_top` above
// the region for that reason — see its comment.
.equ RELOC_ADDR, 0x00200000

.section ".text.boot"
.global _start

_start:
    // Only core 0 is ever released here (see this file's module doc);
    // park any other core that somehow arrives rather than run the
    // relocation a second time. The low bits of MPIDR_EL1 are the
    // core id (Aff0).
    mrs     x1, mpidr_el1
    and     x1, x1, #3
    cbnz    x1, halt

    // Copy the relocatable part of the image from where firmware
    // physically loaded it (__reloc_src) to where it's linked to run
    // from (RELOC_ADDR), for exactly __reloc_size bytes — both
    // computed by linker64.ld. Word-at-a-time (32-bit) copy, matching
    // the 4-byte alignment linker64.ld guarantees for the region.
    ldr     x0, =__reloc_src
    ldr     x1, =RELOC_ADDR
    ldr     x2, =__reloc_size
    mov     x3, #0
copy_loop:
    cmp     x3, x2
    b.ge    copy_done
    ldr     w4, [x0, x3]
    str     w4, [x1, x3]
    add     x3, x3, #4
    b       copy_loop
copy_done:

    // We just wrote the relocated copy as data; without a barrier the
    // core isn't guaranteed to fetch those bytes as instructions when
    // we jump there next — dsb waits for the writes to complete, isb
    // flushes the pipeline so the next fetch actually sees them.
    dsb     sy
    isb

    // _reloc_start's address is already correct (0x00200000-based):
    // it's linked as part of the relocated .text section (see
    // linker64.ld), not this boot stub.
    ldr     x0, =_reloc_start
    br      x0

halt:
    wfe
    b       halt

    // Dump the literal pool for the loads above while we are still
    // executing from the boot stub's own address range.
    .ltorg

.section ".text.reloc_start"
.global _reloc_start

_reloc_start:
    // Everything from here on executes out of RELOC_ADDR, so it's
    // safe to overwrite the original 0x80000 with the received kernel.
    ldr     x0, =__stack_top
    mov     sp, x0

    // Zero .bss. __bss_start/__bss_end are already correct
    // (0x00200000-based) — .bss links as part of the same relocated
    // region. linker64.ld aligns both to 4, matching this 4-byte store.
    ldr     x4, =__bss_start
    ldr     x9, =__bss_end
    b       2f
1:
    str     wzr, [x4], #4
2:
    cmp     x4, x9
    b.lo    1b

    bl      kmain

halt2:
    wfe
    b       halt2

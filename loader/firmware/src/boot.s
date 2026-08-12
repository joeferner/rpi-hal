// Self-relocating entry point.
//
// The GPU firmware always loads this whole image at 0x8000. Only the
// tiny boot stub below (_start) is meant to run from there — it
// copies everything else (kmain, .rodata, .data — see linker.ld) to
// 0x00200000 and jumps into that copy, since the kernel we're about
// to receive over UART also expects to run at 0x8000, and nothing can
// still be executing from (or referencing static data living at)
// that address range once we start receiving into it.
//
// Addressing is handled by linker.ld, not manual offsets here (unlike
// an earlier version of this file): Rust-compiled code references its
// own static data (string literals, the format-args table behind
// writeln!) via absolute addresses, not PC-relative like our
// hand-written branches. Manually adding a fixed offset only to the
// handful of symbols referenced directly from this file (as before)
// didn't fix those compiler-generated references elsewhere — so
// linker.ld now links everything except this boot stub as if it
// already runs from 0x00200000, making every address correct as
// linked, with nothing left to manually adjust.
//
// No cache maintenance is done here: nothing in this project ever
// enables the MMU or caches (see notes.md's deferred virtual-memory
// plan), so there's no instruction-cache staleness hazard when we
// later jump into freshly-written memory.
//
// Single core by design: the GPU firmware only ever releases core 0 to
// this entry point. Cores 1-3 are held in the firmware's own stub,
// watching their ARM-local mailbox registers, and never reach this
// code at all -- so this loader needs to do nothing for them. A
// multi-core kernel loaded through here wakes them itself, straight
// out of that firmware stub (see rpi-hal's `multicore` module), exactly
// as it would if the firmware had loaded that kernel directly.

// STACK_TOP is not a fixed offset here: the stack must clear the whole
// relocated region, which outgrew a fixed RELOC_ADDR + 0x10000 once the
// FAT/SD command code landed. linker.ld exports `__stack_top` above the
// region for that reason — see its comment.
.equ RELOC_ADDR, 0x00200000

.section ".text.boot"
.global _start

_start:
    // Only core 0 is ever released here (see this file's module doc);
    // park any other core that somehow arrives rather than run the
    // relocation a second time.
    mrc     p15, 0, r1, c0, c0, 5
    and     r1, r1, #3
    cmp     r1, #0
    bne     halt

    // Copy the relocatable part of the image from where firmware
    // physically loaded it (__reloc_src) to where it's linked to run
    // from (RELOC_ADDR), for exactly __reloc_size bytes — both
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
    // we jump there next — dsb waits for the writes to complete, isb
    // flushes the pipeline so the next fetch actually sees them.
    dsb
    isb

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

    // __bss_start/__bss_end are already correct (0x00200000-based) —
    // .bss links as part of the same relocated region, unlike an
    // earlier version of this file where they needed a manual offset.
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

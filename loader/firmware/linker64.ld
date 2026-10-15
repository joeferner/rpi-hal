ENTRY(_start)

SECTIONS
{
    /* Physically loaded here by GPU firmware; this is where the boot
       stub (_start, in boot64.s) actually executes, briefly, before
       relocating everything else. 0x80000 is the AArch64 kernel load
       address (vs 0x8000 for AArch32) — selected by arm_64bit=1 in
       config.txt. It never runs again after jumping to _reloc_start,
       so it doesn't need to be part of the relocated copy. */
    . = 0x80000;
    .text.boot :
    {
        KEEP(*(.text.boot))
    }

    /* Everything else links assuming it already runs from 0x00200000
       (see boot64.s's relocation copy) — not because it's actually
       loaded there (GPU firmware only ever loads the whole image at
       0x80000), but so that Rust-compiled code's absolute references to
       its own static data (string literals, the format-args table
       behind writeln!) are correct once relocated. See linker.ld (the
       32-bit script) for the full reasoning; it applies identically
       here, with `ldr xN, =sym` literal loads in place of the 32-bit
       movw/movt pair. */
    . = 0x00200000;
    .text : AT (LOADADDR(.text.boot) + SIZEOF(.text.boot))
    {
        KEEP(*(.text.reloc_start))
        *(.text .text.*)
    }

    /* No explicit AT() on .rodata/.data (only .text pins the load
       address): once .text has fixed the VMA-to-LMA difference, the
       linker gives each following allocatable section that *same*
       difference, so the load layout is a byte-exact copy of the run
       layout -- including any alignment padding the linker inserts
       between sections in the VMA (e.g. before .rodata). Computing each
       LMA by hand as LOADADDR(prev)+SIZEOF(prev) instead (as this script
       used to) packed the sections tightly at the load address while the
       run addresses stayed padded, so boot64.s's flat copy landed
       .rodata and everything after it a few bytes low -- silently
       corrupting every absolute reference into it (string literals, the
       HELLO/ACK magic, the format-args tables). It only bit when
       SIZEOF(.text) wasn't a multiple of .rodata's alignment; whenever
       it happened to be, the padding was zero and the bug stayed
       hidden. */
    .rodata :
    {
        *(.rodata .rodata.*)
    }

    .data :
    {
        /* Same reasoning as .bss's ALIGN(4) below: .rodata's total size
           (string literals, packed without padding) isn't guaranteed to
           be a multiple of 4, so without this any word-sized item .data
           starts with could land on a non-word-aligned address -- an
           unaligned word access hangs, since this loader has no vector
           table to catch the resulting alignment fault. */
        . = ALIGN(4);
        *(.data .data.*)
        /* Round the relocated region up to a whole word so boot64.s's
           word-at-a-time copy covers every byte of it. */
        . = ALIGN(4);
    }
    __reloc_end = .;

    .bss (NOLOAD) :
    {
        /* The word-at-a-time zeroing loop in boot64.s requires a
           word-aligned start and end; without this, __bss_start lands
           on whatever non-aligned byte .data happened to end on, and an
           unaligned store data-aborts with no vector table to catch it.
           ALIGN at the end too so the loop's `str wzr, [x4], #4` can't
           step past __bss_end into a partial final word. */
        . = ALIGN(4);
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    }

    /* Exported for boot64.s: where firmware actually put the relocatable
       part of the image (LMA), and exactly how big it is. The size spans
       the whole run-time region (.text through .data, including any
       inter-section padding), not the sum of section sizes -- that sum
       omits the padding and is what made the copy come up short. */
    __reloc_src = LOADADDR(.text);
    __reloc_size = __reloc_end - ADDR(.text);

    /* Top of the stack, loaded into `sp` by boot64.s. Placed above the
       whole relocated region (.text/.rodata/.data/.bss) rather than at a
       fixed offset from RELOC_ADDR: a fixed `RELOC_ADDR + 0x10000` landed
       *inside* .rodata once the image grew past 64KB (the FAT/SD command
       code pushed it to ~70KB), which would have let the stack clobber
       string literals and lookup tables. Anchoring to the location
       counter here means the stack always sits clear of the region no
       matter how large it grows. Everything at/above RELOC_ADDR is
       loader-only — received images are validated to end at or below
       RELOC_ADDR (see main.rs) — so there's ample room to reserve a
       generous stack. This advances the location counter but emits no
       bytes (no output section), so it doesn't enlarge the image. */
    . = ALIGN(16);
    . = . + 0x20000; /* 128 KiB */
    __stack_top = .;

    /DISCARD/ :
    {
        *(.comment)
        *(.note.*)
        *(.eh_frame*)
    }
}

ENTRY(_start)

SECTIONS
{
    /* Physically loaded here by GPU firmware; this is where the boot
       stub (_start, in boot.s) actually executes, briefly, before
       relocating everything else. It never runs again after jumping
       to _reloc_start, so it doesn't need to be part of the relocated
       copy. */
    . = 0x8000;
    .text.boot :
    {
        KEEP(*(.text.boot))
    }

    /* Everything else links assuming it already runs from 0x00200000
       (see boot.s's relocation copy) — not because it's actually
       loaded there (GPU firmware only ever loads the whole image at
       0x8000), but so that Rust-compiled code's absolute references to
       its own static data (string literals, the format-args table
       behind writeln!) are correct once relocated. Confirmed by
       disassembly that such references use movw/movt with a fixed
       link-time constant, not PC-relative addressing like our
       hand-written branches — so without this, any such reference is
       silently wrong the moment something overwrites its target at
       the original 0x8000-based location (exactly what receiving an
       upload into memory at 0x8000 does). */
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
       run addresses stayed padded, so boot.s's flat copy landed .rodata
       and everything after it a few bytes low -- silently corrupting
       every absolute reference into it (string literals, the HELLO/ACK
       magic, the format-args tables). It only bit when SIZEOF(.text)
       wasn't a multiple of .rodata's alignment; whenever it happened to
       be, the padding was zero and the bug stayed hidden. */
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
        /* Round the relocated region up to a whole word so boot.s's
           word-at-a-time copy covers every byte of it. */
        . = ALIGN(4);
    }
    __reloc_end = .;

    .bss (NOLOAD) :
    {
        /* stmia-based zeroing in boot.s requires a word-aligned start;
           without this, __bss_start lands on whatever non-aligned byte
           .data happened to end on, and an unaligned stm data-aborts
           with no vector table to catch it. */
        . = ALIGN(4);
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        __bss_end = .;
    }

    /* Exported for boot.s: where firmware actually put the relocatable
       part of the image (LMA), and exactly how big it is. The size spans
       the whole run-time region (.text through .data, including any
       inter-section padding), not the sum of section sizes -- that sum
       omits the padding and is what made the copy come up short. */
    __reloc_src = LOADADDR(.text);
    __reloc_size = __reloc_end - ADDR(.text);

    /* Top of the stack, loaded into `sp` by boot.s. Placed above the
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
        *(.ARM.attributes)
        *(.ARM.exidx*)
        *(.ARM.extab*)
    }
}

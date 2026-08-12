ENTRY(_start)

SECTIONS
{
    /* Standard load address for a 32-bit ARM kernel.img on the Pi. */
    . = 0x8000;

    .text :
    {
        KEEP(*(.text.boot))
        *(.text .text.*)
    }

    .rodata :
    {
        *(.rodata .rodata.*)
    }

    .data :
    {
        *(.data .data.*)
    }

    .bss (NOLOAD) :
    {
        /* stmia-based zeroing in boot.s requires a word-aligned start;
           without this, __bss_start lands on whatever non-aligned byte
           .data happened to end on, and an unaligned stm data-aborts
           with no vector table to catch it. Every example so far has
           happened to land aligned by coincidence of code size — this
           was a latent bug, caught when rpi-loader's size shifted it
           off by one byte. */
        . = ALIGN(4);
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        /* Word-align the end too, not just the start above. boot.s's
           stmia zeroing already requires the aligned start; aligning the
           end as well keeps __bss_end word-aligned, which is what
           consumers that build on top of it expect — e.g. a heap placed
           at __bss_end (see examples/heap_alloc.rs) then starts on a
           clean boundary instead of whatever odd byte .bss happened to
           end on. */
        . = ALIGN(4);
        __bss_end = .;
    }

    /DISCARD/ :
    {
        *(.comment)
        *(.note.*)
        *(.ARM.attributes)
        *(.ARM.exidx*)
        *(.ARM.extab*)
    }
}

ENTRY(_start)

SECTIONS
{
    /* Load address for an AArch64 kernel8.img on the Pi. When
       `arm_64bit=1`, the firmware loads kernel8.img here by default, so
       linking at the same address means the image direct-boots straight
       off the SD card with no loader (link address == load address). This
       is the 64-bit counterpart to linker.ld's 0x8000 (the 32-bit
       kernel7.img convention); build.rs picks between the two by target
       architecture. */
    . = 0x80000;

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

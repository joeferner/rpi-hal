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

    /* Sizes of the regions reserved below, overridable from outside
       without editing this script -- a consumer passes e.g.
       `-Wl,--defsym=__stack_size=0x400000` in its own `rustflags` and
       that definition wins, since `DEFINED` sees it before this
       assignment runs. */
    __stack_size  = DEFINED(__stack_size)  ? __stack_size  : 0x100000;
    __stack_slack = DEFINED(__stack_slack) ? __stack_slack : 0x200000;

    /* The stack, reserved here rather than left to grow down from the
       load address -- see linker.ld's copy of this comment for why (the
       size was a consequence of the load address, and differed 16-fold
       between the two architectures for no reason anybody chose).

       There is one stack rather than AArch32's several: exceptions taken
       to EL1h run on `SP_EL1`, the same one `kmain` is using, so there
       are no banked per-mode stacks to reserve.

       `__stack_slack` below it is deliberate margin -- an overflow walks
       into reserved nothing for that far before it can reach
       .data/.rodata/.text -- and is 2 MiB-aligned so the `mmu` feature
       can later leave that block unmapped and make an overflow fault
       where it happens. 2 MiB is the AArch64 block size. */
    .stack (NOLOAD) : ALIGN(0x200000)
    {
        __stack_slack_bottom = .;
        . += __stack_slack;
        __stack_bottom = .;
        . += __stack_size;
        __stack_top = .;
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

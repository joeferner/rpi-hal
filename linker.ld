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

    /* Sizes of the regions reserved below. Each is overridable from
       outside without editing this script -- a consumer passes e.g.
       `-Wl,--defsym=__stack_size=0x400000` in its own `rustflags` and
       that definition wins, since `DEFINED` sees it before this
       assignment runs. */
    __stack_size      = DEFINED(__stack_size)      ? __stack_size      : 0x100000;
    __stack_slack     = DEFINED(__stack_slack)     ? __stack_slack     : 0x200000;
    __irq_stack_size  = DEFINED(__irq_stack_size)  ? __irq_stack_size  : 0x10000;
    __abt_stack_size  = DEFINED(__abt_stack_size)  ? __abt_stack_size  : 0x8000;
    __und_stack_size  = DEFINED(__und_stack_size)  ? __und_stack_size  : 0x8000;
    __fiq_stack_size  = DEFINED(__fiq_stack_size)  ? __fiq_stack_size  : 0x8000;

    /* The stacks, reserved here rather than left to grow down from the
       load address.

       `boot.s` used to set `sp` to `_start` itself, which made the size
       of a program's stack a consequence of where the firmware loads the
       image: 32 KiB below 0x8000 on AArch32, 512 KiB below 0x80000 on
       AArch64. A number nobody chose, differing 16-fold between the two
       architectures, discoverable only by reading the boot assembly, and
       invisible until an application does enough real work to outgrow it
       -- at which point it presents as an unexplained hang somewhere
       else entirely. It also grew *downward through low memory*, where
       the firmware leaves the ATAGs and (on AArch64) the armstub8 spin
       table `multicore` uses to start cores 1-3.

       Reserving the region here fixes the size at a number in a file,
       the same on both architectures, and puts it somewhere nothing else
       claims. `NOLOAD` means it costs no image bytes; on a board with at
       least 1 GiB (every Pi this crate supports) the few MiB of address
       space are noise.

       `__stack_slack` below the stack is deliberate margin: an overflow
       walks into reserved nothing for that far before it can reach
       .data/.rodata/.text, which turns a class of silent corruption into
       a survivable overrun. It is 2 MiB-aligned so that the `mmu`
       feature can later leave that block unmapped and make an overflow
       fault at the moment it happens -- 2 MiB is the coarser of the two
       granules in play (AArch64 block, vs AArch32's 1 MiB section). */
    .stack (NOLOAD) : ALIGN(0x200000)
    {
        __stack_slack_bottom = .;
        . += __stack_slack;

        /* Main (SVC) mode. */
        __stack_bottom = .;
        . += __stack_size;
        __stack_top = .;

        /* The banked modes, each with its own region: adjacent to the
           main stack rather than carved out of the middle of it. The
           previous arrangement set the IRQ stack to `_start -
           IRQ_STACK_SIZE`, 4 KiB into the region main mode was growing
           through, so any main-mode frame deeper than 4 KiB sat in
           memory the first interrupt would push onto.

           ABT/UND/FIQ get stacks so that an application overriding the
           weak `__unhandled_exception` (see vectors.s) can be ordinary
           Rust: those handlers run in their own mode, on their own
           banked `sp`, and pushing onto one that was never initialized
           faults again immediately -- reporting nothing, which is the
           failure the override exists to fix. */
        . += __irq_stack_size;
        __irq_stack_top = .;
        . += __abt_stack_size;
        __abt_stack_top = .;
        . += __und_stack_size;
        __und_stack_top = .;
        . += __fiq_stack_size;
        __fiq_stack_top = .;
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

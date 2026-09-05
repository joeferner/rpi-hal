/* RP2350B, as fitted to the Olimex PICO2-XL and PICO2-XXL.
 *
 * FLASH is the XL's 2 MB rather than the XXL's 16 MB, deliberately: the two
 * boards are otherwise identical, and an image linked for the smaller one runs
 * on both.
 *
 * There is no BOOT2 region. An RP2040 begins by copying a second-stage loader
 * out of the first 256 bytes of flash; an RP2350's boot ROM instead looks for
 * an IMAGE_DEF block in the first 4 KB, which is what `.start_block` below
 * places. `embassy-rp` emits the block itself, so nothing in the firmware
 * declares it.
 */
MEMORY {
    FLASH : ORIGIN = 0x10000000, LENGTH = 2048K
    RAM   : ORIGIN = 0x20000000, LENGTH = 512K
}

SECTIONS {
    /* Directly after the vector table, so it stays inside the first 4 KB where
     * the boot ROM and picotool both look for it. */
    .start_block : ALIGN(4)
    {
        KEEP(*(.start_block));
    } > FLASH
} INSERT AFTER .vector_table;

/* And .text starts after the block rather than on top of it. */
_stext = ADDR(.start_block) + SIZEOF(.start_block);

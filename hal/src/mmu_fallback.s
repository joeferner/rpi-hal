// Weak no-op fallback for rpi_hal_mmu_init, only assembled in when the
// `mmu` feature is off (see boot.rs) -- boot.s's `bl rpi_hal_mmu_init`
// is unconditional, so *something* must define this symbol or the
// link fails outright. Kept out of boot.s itself (rather than always
// assembled in) because a *weak* and a *strong* definition of the same
// symbol only resolve correctly when the linker sees them in separate
// object files -- within a single object file (which boot.s and
// mmu.rs's `#[no_mangle]` function would otherwise both end up in),
// two definitions of the same symbol is a hard assembler error
// regardless of weak/strong. With this file only included when `mmu`
// is off, any given build of this crate only ever defines
// `rpi_hal_mmu_init` once itself; a consumer's own strong definition
// (a genuinely separate crate/object file) then correctly overrides
// this weak one at final link time.
.weak rpi_hal_mmu_init
rpi_hal_mmu_init:
    bx      lr

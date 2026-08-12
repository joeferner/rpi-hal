// AArch64 weak no-op fallback for rpi_hal_mmu_init -- the counterpart to
// mmu_fallback.s, only assembled in when the `mmu` feature is off (see
// boot.rs). boot64.s's `bl rpi_hal_mmu_init` is unconditional, so
// something must define this symbol or the link fails; a consumer's own
// strong definition (a separate object file) overrides this weak one at
// final link time. See mmu_fallback.s for the full weak/strong reasoning.
.weak rpi_hal_mmu_init
rpi_hal_mmu_init:
    ret

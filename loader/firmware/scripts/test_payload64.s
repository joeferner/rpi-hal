// Minimal AArch64 payload to prove out the 64-bit loader handoff
// without needing a full 64-bit kernel (rpi-hal is still AArch32).
//
// rpi-loader receives this over UART, writes it to 0x80000, and jumps
// to it in AArch64 state. All this does is print a line over UART0 —
// which the loader already brought up — then park. Seeing the line
// appear in the host CLI's passthrough terminal confirms the whole 64-bit
// path: firmware -> AArch64 boot stub -> relocation -> receive -> jump.
//
// Register addresses are the BCM2836/BCM2837 low peripheral base
// (0x3F000000). UART0 (PL011) sits at offset 0x201000; DR is the data
// register, FR the flag register whose TXFF bit (0x20) is set while the
// transmit FIFO is full.
//
// Position-independent: the message is reached PC-relatively (adr), and
// the peripheral addresses are absolute constants, so this runs
// correctly at whatever address the loader was told to place it.

.equ UART0_DR, 0x3F201000
.equ UART0_FR, 0x3F201018
.equ TXFF,     0x20

.section .text
.global _start

_start:
    ldr     x1, =UART0_DR
    ldr     x2, =UART0_FR
    adr     x0, msg
puts:
    ldrb    w3, [x0], #1
    cbz     w3, done
wait:
    ldr     w4, [x2]
    tst     w4, #TXFF
    b.ne    wait
    str     w3, [x1]
    b       puts
done:
    wfe
    b       done

    .ltorg

msg:
    .asciz "\r\n[rpi-loader: 64-bit payload running]\r\n"

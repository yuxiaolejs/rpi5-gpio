.global _start
.section .text
_start:
    // stack
    ldr x0, =0x80000 + 0x40000
    mov sp, x0

    b main
halt:
    wfe
    b halt
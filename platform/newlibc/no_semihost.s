.syntax unified
.arch armv7e-m

.section .text
.global __use_no_semihosting  @ 关键声明

__use_no_semihosting:
    bx lr
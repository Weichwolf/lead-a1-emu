/* SPDX-License-Identifier: GPL-3.0-only */
.syntax unified
.cpu arm926ej-s
.arm
.text
/* 0: owned interrupted instruction. */
mov r0, #7
.balign 256
/* 1: owned callback; r6 points to an explicitly initialized counter. */
ldr r0, [r6]
add r0, r0, #1
str r0, [r6]
bx lr
.balign 256
/* 2/3: first access changes a device, second reaches an unsupported register. */
ldmia r0!, {r1, r2}
.balign 256
stmia r0!, {r1, r2}
.balign 32

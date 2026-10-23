/* SPDX-License-Identifier: GPL-3.0-only */
/* Owned caller fixture for isolated original cache helpers. */
.syntax unified
.cpu arm926ej-s
.arm
.text
ldr r1, [r0]
str r2, [r0]
ldr r3, [r4]
str r6, [r4]
mov lr, pc
bx r5
mov lr, pc
bx r7
nop
.balign 32

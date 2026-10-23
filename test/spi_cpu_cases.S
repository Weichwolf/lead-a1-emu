/* SPDX-License-Identifier: GPL-3.0-only */
.syntax unified
.cpu arm926ej-s
.arm
.text
stmia r0!, {r1, r2, r3}
ldmia r0!, {r1, r2, r3, r4, r5, r6, r7, r8, r9}
ldrh r2, [r0, #2]
ldrh r2, [r0]

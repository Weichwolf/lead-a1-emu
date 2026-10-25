/* SPDX-License-Identifier: GPL-3.0-only */
.syntax unified
.cpu arm926ej-s
.arm
.text
/* Caller supplies GPIO group, clear/set mask and direction. */
strh r1, [r0, #12]
strh r2, [r0]
ldrh r3, [r0, #16]
and r3, r3, #2
strh r1, [r0, #8]
ldrh r3, [r0, #16]

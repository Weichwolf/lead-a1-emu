/* SPDX-License-Identifier: GPL-3.0-only */
/* Owned ARM926 system tests. Cases start on 256-byte boundaries. */
.syntax unified
.cpu arm926ej-s
.arm
.text

/* 0: startup cache maintenance and control transition. */
mov r0, #0
mcr p15, 0, r0, c7, c7, 0
mcr p15, 0, r0, c7, c10, 4
mrc p15, 0, r1, c1, c0, 0
orr r1, r1, #0x1000
mcr p15, 0, r1, c1, c0, 0
mov r2, #1
mrc p15, 0, r3, c1, c0, 0

.balign 256
/* 1: data writes do not implicitly invalidate instruction cache contents. */
mov r0, #1
mov r1, #2
mcr p15, 0, r2, c7, c5, 0
mov r3, #3
ldr r4, [r9]
mcr p15, 0, r5, c1, c0, 0
mov r6, #6
mcr p15, 0, r7, c1, c0, 0
bx r8

.balign 256
/* 2: a hardware CPU ID must not be invented. */
mrc p15, 0, r0, c0, c0, 0

.balign 256
/* 3: explicit reference control state. */
mrc p15, 0, r0, c1, c0, 0

.balign 256
/* 4: unsupported or unresolved control transitions must fail atomically. */
mcr p15, 0, r0, c1, c0, 0

.balign 256
/* 5: draining explicitly supplied pending writes. */
mcr p15, 0, r0, c7, c10, 4

.balign 256
/* 6: a failed condition must not inspect unknown system/operand state. */
mcreq p15, 0, r0, c7, c7, 0

.balign 256
/* 7: opcode 1 is not interchangeable with zero. */
mcr p15, 1, r0, c7, c7, 0

.balign 256
/* 8: the maintenance operand is architecturally SBZ. */
mcr p15, 0, r0, c7, c7, 0

.balign 256
/* 9: WFI requires a scheduler/interrupt implementation. */
mcr p15, 0, r0, c7, c0, 4

.balign 256
/* 10: MRC to PC updates NZCV, rather than jumping or changing Q. */
.inst 0xee11ff10

.balign 256
/* 11-15: five different lines conflict in a four-way, one-set fixture. */
next11: b next12
.balign 256
next12: b next13
.balign 256
next13: b next14
.balign 256
next14: b next15
.balign 256
next15: mov r0, #15
b next11
.balign 32

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

.balign 256
/* 16: distinct I/D lockdown registers, changed through read-modify-write. */
mrc p15, 0, r0, c9, c0, 1
mrc p15, 0, r1, c9, c0, 0
orr r0, r0, #1
mcr p15, 0, r0, c9, c0, 1
mrc p15, 0, r2, c9, c0, 1
orr r1, r1, #2
mcr p15, 0, r1, c9, c0, 0
mrc p15, 0, r3, c9, c0, 0

.balign 256
/* 17: invalid/unknown lockdown write operands. */
mcr p15, 0, r0, c9, c0, 1

.balign 256
/* 18: TCM region control is a different c9 encoding, still unsupported. */
mrc p15, 0, r0, c9, c1, 1

.balign 256
/* 19: unknown instruction lockdown read. */
mrc p15, 0, r0, c9, c0, 1

.balign 256
/* 20: unknown data lockdown read. */
mrc p15, 0, r0, c9, c0, 0

.balign 256
/* 21-24: changing allocation ways preserves existing locked cache lines. */
mov r0, #1
mcr p15, 0, r1, c9, c0, 1
bx r2
.balign 256
mrc p15, 0, r3, c9, c0, 1
bx r4
.balign 256
locked_third_line:
mcr p15, 0, r5, c9, c0, 1
bx r6
.balign 256
mov r0, #4
sub r9, r9, #1
cmp r9, #0
bne locked_third_line
bx r7
.balign 32

.balign 256
/* 25: invalidate the current instruction line by MVA. */
mov r0, #1
mcr p15, 0, r1, c7, c5, 1
mov r2, #2
.balign 256
/* 26: skipped entry maintenance must not inspect an unknown operand. */
mov r0, #1
mcreq p15, 0, r1, c7, c5, 1
mov r2, #2
.balign 256
/* 27: invalidate the line filled by this very instruction, by Set/Way. */
mcr p15, 0, r1, c7, c5, 2
mov r2, #2
.balign 256
/* 28/29: a fetched new line and an invalidated old line commit together. */
mov r0, #1
bx r8
.balign 256
mcr p15, 0, r1, c7, c5, 1
mov r2, #2
bx r9
.balign 256
/* 30/31: reject nonzero word/SBZ and unused index bits. */
mcr p15, 0, r1, c7, c5, 1
.balign 256
mcr p15, 0, r1, c7, c5, 2
.balign 256
/* 32: entry invalidation with an unresolved operand. */
mcr p15, 0, r1, c7, c5, 1
.balign 256
/* 33: a maintenance miss must not fetch an unmapped target. */
mcr p15, 0, r1, c7, c5, 1
mov r2, #2
.balign 256
/* 34: nonzero Set field, retaining a different set. */
mov r0, #1
bx r8
.balign 32
mov r2, #2
mcr p15, 0, r1, c7, c5, 2
mov r3, #3
.balign 32

.balign 256
/* 35/36: a first PC-to-LR copy must not depend on decoder scratch history. */
mov lr, pc
.balign 256
moveq lr, pc
.balign 32

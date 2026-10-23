/* SPDX-License-Identifier: GPL-3.0-only */
/* Owned ARM926 MMU integration programs; cases start 256 bytes apart. */
.syntax unified
.cpu arm926ej-s
.arm
.text

/* 0: initialize registers/TLB, enable identity-mapped code, access virtual RAM. */
mcr p15, 0, r0, c2, c0, 0
mcr p15, 0, r1, c3, c0, 0
mcr p15, 0, r2, c13, c0, 0
mcr p15, 0, r2, c10, c0, 0
mcr p15, 0, r2, c8, c7, 0
mrc p15, 0, r3, c2, c0, 0
mrc p15, 0, r4, c3, c0, 0
mcr p15, 0, r5, c1, c0, 0
nop
nop
ldr r6, [r7]
str r6, [r8]

.balign 256
/* 1: data remapping after TLB invalidation, followed by I-cache invalidation. */
ldr r0, [r1]
mcr p15, 0, r2, c8, c6, 1
ldr r3, [r1]
mcr p15, 0, r9, c7, c5, 0
mov r4, #1
bx r8

.balign 256
/* 2: VIVT instruction bytes remain stale after changing a physical mapping. */
nop
mcr p15, 0, r2, c8, c7, 1
mov r0, #1
mcr p15, 0, r9, c7, c5, 0
mov r1, #2

.balign 256
/* 3/4: second subpage denies access after a successful first word. */
ldmia r0!, {r1, r2}
.balign 256
stmia r0!, {r1, r2}

.balign 256
/* 5: control transition; nonidentity prefetch and D-cache remain explicit. */
mcr p15, 0, r0, c1, c0, 0
nop
nop
.balign 32

.balign 256
/* 6/7: privileged instructions that explicitly request User data permissions. */
ldrt r1, [r0], #4
.balign 256
strt r1, [r0], #4
.balign 32

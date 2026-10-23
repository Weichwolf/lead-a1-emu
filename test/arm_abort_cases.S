/* SPDX-License-Identifier: GPL-3.0-only */
.syntax unified
.cpu arm926ej-s
.arm
.text
/* 0: owned low/high vector stubs; pointers are fixture input. */
.rept 8
ldr pc, [pc, #24]
.endr
.balign 256
/* 1/2: scalar load/store writeback must be restored on abort. */
ldr r1, [r0], #4
.balign 256
str r1, [r0, #4]!
.balign 256
/* 3/4/5: multiple transfers and a load of the base before a later fault. */
ldmia r0!, {r1, r2}
.balign 256
stmia r0!, {r1, r2}
.balign 256
ldmia r0, {r0, r1, r2}
.balign 256
/* 6: SWP's destination is preserved when its write permission fails. */
swp r1, r2, [r0]
.balign 256
/* 7/8: privileged code explicitly requests user data permissions. */
ldrt r1, [r0], #4
.balign 256
strt r1, [r0], #4
.balign 256
/* 9..12: extra scalar and doubleword transfer classes. */
ldrh r1, [r0], #2
.balign 256
ldrsh r1, [r0]
.balign 256
ldrd r2, r3, [r0], #8
.balign 256
strd r2, r3, [r0], #8
.balign 256
/* 13: c5/c6 round-trip, ignored bit 8 and MRC-to-PC flags. */
mcr p15, 0, r0, c5, c0, 0
mcr p15, 0, r1, c5, c0, 1
mcr p15, 0, r2, c6, c0, 0
mrc p15, 0, r3, c5, c0, 0
mrc p15, 0, r4, c5, c0, 1
mrc p15, 0, r5, c6, c0, 0
mrc p15, 0, apsr_nzcv, c5, c0, 0
.balign 256
/* 14..18: unknown/reserved/unsupported fault-register inputs. */
mrc p15, 0, r0, c5, c0, 0
.balign 256
mrc p15, 0, r0, c5, c0, 1
.balign 256
mrc p15, 0, r0, c6, c0, 0
.balign 256
mcr p15, 0, r0, c5, c0, 0
.balign 256
mrc p15, 0, r0, c6, c0, 1
.balign 256
/* 19/20: guest repairs its descriptor, invalidates and retries the access. */
str r3, [r4]
mcr p15, 0, r5, c8, c7, 1
mrc p15, 0, r6, c6, c0, 0
subs pc, lr, #8
.balign 256
str r3, [r4]
mcr p15, 0, r5, c8, c7, 1
subs pc, lr, #4
.balign 256
/* 21: alignment handler repairs the operand and retries. */
bic r0, r0, #3
subs pc, lr, #8
.balign 256
/* 22: code reached after a successful prefetch-abort repair. */
mov r8, #9
.balign 256
/* 23: untaken multiple transfer must not fault on its operands. */
ldmiaeq r0!, {r1, r2}
.balign 256
/* 24: the owned exception-return adapter can also abort during its loads. */
ldmia r0!, {r1, pc}^
.balign 256
/* 25: a condition-skipped unknown fault-register read. */
mrceq p15, 0, r0, c5, c0, 0
.balign 32
.balign 256
/* 26: MRC-to-PC uses only known-zero high fields of the reference FSR. */
mrc p15, 0, apsr_nzcv, c5, c0, 0
.balign 32
.balign 256
/* 27: transition qualification is not an actual instruction fetch. */
mcr p15, 0, r0, c1, c0, 0
nop
nop
.balign 32
.balign 256
/* 28/29: dirty cache effects survive abort and can be cleaned by its handler. */
ldr r3, [r0]
stmia r0!, {r1, r2}
.balign 256
bic r5, r0, #31
mcr p15, 0, r5, c7, c10, 1
mov r5, #0
mcr p15, 0, r5, c7, c10, 4
ldr r4, [r0]
.balign 256
/* 30: a descriptor buffer drain belongs to the actual aborting access. */
str r2, [r1]
ldr r3, [r4]
.balign 32

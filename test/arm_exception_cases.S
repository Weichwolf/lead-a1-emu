/* SPDX-License-Identifier: GPL-3.0-only */
.syntax unified
.cpu arm926ej-s
.arm
.text
/* 0: owned vector words, copied to low/high vector RAM by the fixture. */
.rept 8
ldr pc, [pc, #24]
.endr
.balign 256
/* 1/2: ordinary IRQ/FIQ and SWI returns. */
subs pc, lr, #4
.balign 256
movs pc, lr
.balign 256
/* 3/4: user code and a conditional SWI. */
swi #0x1234
mov r0, #7
.balign 256
swieq #0x42
mov r1, #1
.balign 256
/* 5/6: arbitrary saved-status return and data-abort-style subtraction. */
movs pc, r0
.balign 256
subs pc, lr, #8
.balign 256
/* 7..10: all four LDM exception-return addressing modes. */
ldmia sp!, {r0, r8, r12, pc}^
.balign 256
ldmib sp!, {r0, r8, r12, pc}^
.balign 256
ldmda sp!, {r0, r8, r12, pc}^
.balign 256
ldmdb sp!, {r0, r8, r12, pc}^
.balign 256
/* 11/12/13: no writeback, unpredictable base overlap, partial read failure. */
ldmia r4, {r0, r1, pc}^
.balign 256
ldmia r0!, {r0, pc}^
.balign 256
ldmia r0!, {r1, pc}^
.balign 256
/* 14/15: skipped returns must not read unknown saved state or operands. */
movseq pc, lr
.balign 256
ldmiaeq sp!, {r0, pc}^
.balign 256
/* 16..27: every data-processing result operation with S=1/Rd=PC. */
ands pc, r0, r1
.balign 256
eors pc, r0, r1
.balign 256
subs pc, r0, r1
.balign 256
rsbs pc, r0, r1
.balign 256
adds pc, r0, r1
.balign 256
adcs pc, r0, r1
.balign 256
sbcs pc, r0, r1
.balign 256
rscs pc, r0, r1
.balign 256
orrs pc, r0, r1
.balign 256
movs pc, r1
.balign 256
bics pc, r0, r1
.balign 256
mvns pc, r1
.balign 256
/* 28..34: shifter boundaries, RRX and rotated immediates. */
movs pc, r0, lsl #31
.balign 256
movs pc, r0, lsr #32
.balign 256
movs pc, r0, asr #32
.balign 256
movs pc, r0, ror #8
.balign 256
movs pc, r0, rrx
.balign 256
movs pc, #0x80000000
.balign 256
subs pc, r0, #0xff000000
.balign 256
/* 35: register-controlled shift with destination PC is unpredictable. */
.inst 0xe1b0f110
.balign 256
/* 36: a visible ordinary instruction at an interrupt boundary. */
mov r0, #7
.balign 256
/* 37: load SP/LR in the current bank before restoring the saved mode. */
ldmia r4!, {sp, lr, pc}^
.balign 32
.balign 256
/* 38/39: owned callers for the original ARM0 SVC selector. */
swi #0x050000
mov r0, #7
.balign 256
swi #0x050001
mov r0, #7
.balign 32
.balign 256
/* 40..45: nonzero arithmetic/logical shifts, PC operand and no rotation. */
movs pc, r0, asr #1
.balign 256
movs pc, r0, lsr #1
.balign 256
movs pc, r0, lsl #1
.balign 256
movs pc, pc
.balign 256
movs pc, r0, asr #31
.balign 256
movs pc, #16
.balign 32

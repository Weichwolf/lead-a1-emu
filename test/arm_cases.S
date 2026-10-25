/* SPDX-License-Identifier: GPL-3.0-only */
/* Owned instruction expectations, not extracted firmware. */
.syntax unified
.cpu arm926ej-s
.arm
.text

/* Offset 0: overflow, little-endian RAM, condition codes and branch. */
mvn r0, #0x80000000
adds r1, r0, #1
mov r2, #0x100
str r1, [r2]
ldr r3, [r2]
cmp r3, r1
moveq r4, #9
movne r5, #12
b 1f
mov r6, #7
1: mov r7, #11

.balign 256
/* Full CPSR read, including processor mode and interrupt masks. */
mrs r0, cpsr

.balign 256
/* Unknown operand register. */
ldr r0, [r1]

.balign 256
/* Second word is unknown: earlier register effects must roll back. */
ldmia r1!, {r0, r2}

.balign 256
/* Unknown second source register: earlier staged RAM write must roll back. */
stmia r1!, {r0, r2}

.balign 256
/* An unsupported system operation must fail visibly. */
mcr p15, 0, r0, c7, c7, 0

.balign 256
/* Decoder must observe changed RAM at the next instruction. */
mov r0, #1
mov r0, #2

.balign 256
/* Fetch checking must not require bytes after this isolated instruction. */
mov r0, #13

.balign 256
/* Interworking: even target stays ARM; odd target must fail before committing. */
bx r0

.balign 256
/* 9: reuse arithmetic/conditional/memory translations with changed operands. */
mov r0, #0
mov r1, #0x100
mov r2, #1
cache_loop:
add r0, r0, r2
str r0, [r1]
ldr r3, [r1]
cmp r3, r0
addeq r4, r0, #1
b cache_loop

.balign 256
/* 10: identical opcodes at distinct PCs exceed the host decode-cache bound. */
cache_many:
.rept 129
adr r0, .
.endr
b cache_many

.balign 256
/* 13: ARM indirect calls retain architectural LR without decoder flow hints. */
mov r1, #0
adr r0, call_target
mov r3, #0x100
str r0, [r3]
mov lr, pc
bx r0
mov lr, pc
ldr pc, [r3]
b call_done
call_target:
add r1, r1, #1
bx lr
call_done:
mov r2, #7
b call_done

.balign 256
/* 14: warm rollback snapshots across register growth, failures and retries. */
mov r0, #7
mov r1, #0x100
ldmia r1!, {r0, r2}
adds r2, r2, #1
ldmia r1!, {r0, r3}

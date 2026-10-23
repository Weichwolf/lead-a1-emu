/* SPDX-License-Identifier: GPL-3.0-only */
/* Owned ARMv5 status/bank expectations. Each case starts on a 256-byte boundary. */
.syntax unified
.cpu arm926ej-s
.arm
.text

/* 0: full CPSR, immediate MSR and synchronized carry into native ADC. */
mrs r0, cpsr
mov r1, #0
msr cpsr_f, #0x20000000
adc r2, r1, #0
mrs r3, cpsr
msr cpsr_f, #0x90000000
mrs r4, cpsr
msr cpsr_c, #0x1f
mrs r5, cpsr
msr cpsr_f, #0x08000000
mrs r6, cpsr
msr cpsr_f, #0
mrs r7, cpsr

.balign 256
/* 1: independent high registers, SP/LR, SPSRs, User/System alias and privilege. */
mov r8, #1
mov r12, #2
mov sp, #0x400
mov lr, #0x40
msr cpsr_c, #0xd1
mov r8, #3
mov r12, #4
mov sp, #0x500
mov lr, #0x50
msr spsr_fc, r0
msr cpsr_c, #0xd2
mov sp, #0x600
mov lr, #0x60
msr spsr_fc, r2
msr cpsr_c, #0xd3
msr cpsr_c, #0xdf
mov sp, #0x700
mov lr, #0x70
msr cpsr_c, #0xd0
msr cpsr_c, r3
mrs r4, cpsr
mov r5, #0
msr cpsr_f, #0x20000000
adc r6, r5, #0

.balign 256
/* 2: saved FIQ and IRQ state must remain independent. */
msr cpsr_c, #0xd1
mrs r0, spsr
msr cpsr_c, #0xd2
mrs r1, spsr

.balign 256
/* 3: failed conditions must not read unknown register/SPSR operands. */
msreq cpsr_c, r0
mrseq r1, spsr
msreq spsr_fc, r0
mrs r2, cpsr

.balign 256
/* 4: partially initialized saved status cannot be read as a known full status. */
msr spsr_c, #0xd2
mrs r0, spsr

.balign 256
/* 5: complete the saved flags independently of the known control byte. */
msr spsr_f, #0x20000000
mrs r0, spsr

.balign 256
/* 6: User and System do not have an SPSR. */
mrs r0, spsr

.balign 256
/* 7: unimplemented mode is unpredictable, never silently mapped to a bank. */
msr cpsr_c, #0xd4

.balign 256
/* 8: changing execution state with MSR is unpredictable. */
msr cpsr_c, #0xf3

.balign 256
/* 9: register operand, including reserved-bit/unknown-operand failure tests. */
msr cpsr_f, r0

.balign 256
/* 10: owned invalid encoding, MRS with PC destination. */
.inst 0xe10ff000

.balign 256
/* 11: owned invalid encoding, MSR with PC source. */
.inst 0xe121f00f

.balign 256
/* 12: reserved extension/status byte fields do not alter implemented bits. */
msr cpsr_xs, r0

.balign 256
/* 13: independent Abort and Undefined SP/LR banks. */
msr cpsr_c, #0xd7
mov sp, #0x800
mov lr, #0x80
msr cpsr_c, #0xdb
mov sp, #0x900
mov lr, #0x90
msr cpsr_c, #0xd3
mov r0, sp
msr cpsr_c, #0xd7
mov r1, sp
msr cpsr_c, #0xdb
mov r2, sp

.balign 256
/* 14: A32 exception return with a complete saved status. */
subs pc, lr, #4

.balign 256
/* 15: user-bank block transfer may not use the current privileged bank. */
ldmia r0, {r1, r2}^

.balign 256
/* 16: an untaken exception-return form must not read its unknown operands. */
subseq pc, lr, #4

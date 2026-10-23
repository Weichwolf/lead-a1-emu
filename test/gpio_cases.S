/* SPDX-License-Identifier: GPL-3.0-only */
.syntax unified
.cpu arm926ej-s
.arm
.text
/* Independent GPIO bus operations; addresses/values are caller inputs. */
strh r1, [r0]
ldrh r2, [r0]
stmia r0!, {r1, r2, r3}
ldmia r0!, {r1, r2}

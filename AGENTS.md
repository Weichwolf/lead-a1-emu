# Project context

- Speak German with the user. Code, comments, documentation and UI are English.
- Read doc/objective.txt before changing scope or claiming completion. Maintain
  doc/coverage.txt requirement by requirement. Bootstrap is not a working synth.
- Execute unmodified Nord Lead A1 OS 1.36 firmware. Do not substitute a host
  synthesizer, scripted MIDI replies or silently successful peripheral stubs.
- Determine processors, part roles, clocks, memory maps and board wiring through
  evidence. Do not import Blofeld hardware assumptions. Two bundle parts do not
  establish MCU/DSP roles or Lead A1/A1R variants.
- Unsupported accesses must fail with address, operation, PC and execution
  context. Distinguish observations, hypotheses and hardware-verified behavior.
- Separate CPU/DSP cores, board/scheduler, host MIDI/audio and UI. Future audio
  callbacks must not allocate or throw. Run as a regular user.
- SIMD acceleration and multiple host cores are optional design choices; the
  user explicitly delegated those choices, not mandatory acceptance criteria.
  Choose from measured guest workloads. Preserve exact arithmetic and causal
  emulated ordering; validate accelerated/concurrent paths against a reference.
- The UI is a native parameter table with current values for slots A-D. Values
  must come from observed firmware state. Unknown, missing and stale values must
  remain explicit; do not display guessed defaults as current values. MIDI CC
  numbers are references, not firmware addresses or readback evidence.

# Quality gates

- C++23, CMake/Ninja, LLVM 19; warnings as errors, clang-format, mandatory
  clang-tidy with zero findings and at most 120 lines per function.
- Instrument project code and future native dependencies with ASan/UBSan.
  Use separate TSan once concurrent code exists. Preserve dependency licenses.
- Never weaken checks, add diagnostic baselines or NOLINT to hide findings.
- Test independent instruction expectations, malformed inputs, state transitions
  and recovered firmware behavior. Self-consistency is insufficient evidence.
- Scope tests to affected behavior. Run required checks; broaden only for a
  coherent integration or unresolved concern. Fuzz only on explicit user request,
  offline; never send malformed traffic to physical hardware.
- Do not flash physical hardware. Detection is not validation. Any later physical
  programming work needs explicit authorization and an established recovery path.
- Commit verified increments and push reviewed commits to upstream/main only.

# References, coordination and publication

- Public implementation: Weichwolf/lead-a1-emu, GPL-3.0-only. New source files
  carry matching SPDX identifiers. THIRD_PARTY.txt records reused material.
- Private source reconstruction: ../lead-a1-src, Weichwolf/lead-a1-src. The user
  explicitly authorizes coordination through issues in that private repository.
  Put findings, interface questions and test summaries there, never in public
  emulator issues. Do not modify the reconstruction worktree during coordination.
- Firmware, installers, manuals, factory banks, raw disassembly and derived
  manufacturer assets stay in ignored doc/local/ or local/. Never commit them.
  Track URLs, sizes and hashes instead. Public tests use synthetic owned inputs
  and optional local reference checks; do not embed original program bytes.
- Private reconstruction sources are not implicitly authorized for publication.
  Import only reviewed original emulator implementation and factual metadata.
- Keep docs concise: contracts, reproducible commands, evidence and gaps. Do not
  keep an activity diary. Do not claim boot, audio, USB or recovery from parser
  checks or an instruction budget being exhausted.

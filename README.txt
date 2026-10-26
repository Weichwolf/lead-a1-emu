Nord Lead A1 firmware emulator — development bootstrap

Execute unmodified OS 1.36 with an evidence-based board model, original MIDI and
audio, and reproducible firmware-development tests. The native GTK4 UI presents
current parameter values in a table for slots A-D. The complete acceptance scope
is in doc/objective.txt; actual evidence and missing work are in doc/coverage.txt.

Current state: native ARM-state reference execution through original cache setup,
banked stacks, BSS initialization and MMU register setup under explicit fixture
inputs; independently tested MMU, D-cache, RAM-bus buffers and ARM IRQ/FIQ/SWI
entry/return; explicit transactional MMIO and a TI-family AINTC reference. Both
original SVC and FIQ/IRQ dispatch handlers are exercised, including nested priority
dispatch after original vector/controller initialization under explicit fixtures.
Image parsing, safe LZO decoding, verified references, strict build gates and the
snapshot viewer work. Complete CPU/board/DSP boot, MIDI, audio and a parameter-state producer
remain pending. The UI starts with unknown values and waits for snapshots.
See doc/arm-reference.txt, doc/arm-exceptions.txt and doc/aintc.txt.
Native SHARC references load all ordinary records of four original DSP streams
at two explicit serial cadences, with independent payload-cell checks. The
clocked IOP profile additionally executes final vector DMA, original P1I/RTI,
native/VISA transition and common clock initialization. It reaches422/446
application instructions through SPI, DAI clock/frame/pin/enable/data and SPORT
control/DMA, PCG/PICR, IDP, DCPR, transport0 SPORT2 and both transports' FLAG
setup, including transport1 LIRPTL/MODE1. Both next FLAG0 waits have unknown
input (doc/sharc-lirptl.txt).
Separate original MCU-uploader references execute the first kernel transfer and
consume its genuine readiness output through explicit candidate GPIO/FLAG wires.
Legacy cases pause MCU execution while DSP readiness is established. Additional
scheduled references interleave actual MCU polls and kernel initialization with
explicit initial-latch/cadence inputs; unknown driven output stops their common
frontier. Continuous full upload and physical wiring remain unqualified
(doc/processor-links.txt, doc/board-scheduler.txt).
These explicit reference inputs do not establish a complete cold boot, physical
processor/slot wiring, MHz or playable audio. Other profiles retain independent
execution boundaries. See doc/sharc-iop-bridge.txt, doc/sharc-dai-data.txt and
doc/dsp-roles.txt. Compact Type17b sign extension remains an explicitly
unproven model assumption; vendor-simulator/hardware qualification is pending.
SPORT control and external-clock standard/TDM serial components have owned
RAM/chain/buffer/framing/completion and CPU-access checks. Original initialization
is bound, but its serial source clocks/data remain unknown: no firmware audio
is claimed (doc/sharc-sport.txt, doc/sharc-sport-dma.txt).

Build and checks (Linux, regular user)
Requires LLVM 19 (clang++, clang-tidy, clang-format, sanitizer runtimes),
llvm-nm/readelf/mc/objcopy 19, CMake >= 3.25, Ninja, patch and Python >= 3.11.
Ghidra 12.1.2 native sources and ARM5_le.sla are required: set GHIDRA_ROOT to an
existing distribution or prepare the pinned native subset below. No Java runtime
is used. GTK viewer additionally requires Debian
packages python3-gi and gir1.2-gtk-4.0. GUI checks use xvfb and dbus-run-session.
No dependencies are downloaded during configure, build or tests.

  python3 tools/dependencies.py --fetch --ghidra
  cmake --preset strict
  cmake --build --preset strict
  cmake --build build/strict --target quality
  xvfb-run -a dbus-run-session -- /usr/bin/python3 test/gui_smoke.py

release keeps diagnostics/analysis/hardening but disables sanitizers. thread is
a separate TSan preset reserved for concurrent integration. No fuzz runs are
part of routine checks. See doc/development.txt for architecture and next work.

Local official references (never published)
Reuse the verified bundle already prepared by the reconstruction project:

  python3 tools/references.py --import-bundle ../lead-a1-src/local/reference/firmware.bin
  python3 tools/references.py --verify

Alternatively extract the pinned official updater ZIP statically (requires 7z):

  python3 tools/references.py --archive '/path/to/Update Nord Lead A1 OS v1.36.zip'
  python3 tools/references.py --fetch

--fetch explicitly downloads the pinned official archive. The installer is never
executed. reference/1.36.json records URL, sizes and SHA-256 of the ZIP, updater,
complete instrument bundle and both parts. No private repo is needed to build.

  build/strict/lead-a1-emu --inspect doc/local/reference/firmware.bin

The C++ inspector checks both complete images, section checksums, safe LZO1X
decoding, padding and load ranges; references.py separately establishes the pinned
1.36 identity. --extract BUNDLE LOCAL_DIRECTORY exports the six decoded sections
for local analysis. See doc/firmware-format.txt. Executing without --inspect exits visibly
because original firmware execution is not implemented. This is deliberate.

Native parameter UI

  tools/lead-a1-ui

The searchable table groups parameters by section, shows slots A, B, C and D
side by side, and includes documented MIDI CC references. It follows atomic
parameters.json snapshots at $XDG_RUNTIME_DIR/lead-a1-emu/parameters.json,
including directory recreation. Missing/invalid/expired snapshots clear values.
No defaults or synthetic live state are shown. See doc/parameter-ui.txt for the
versioned contract and the current missing firmware-to-UI binding.
For an isolated UI preview with explicitly marked synthetic values:

  tools/lead-a1-ui --demo

Install without root (optional)

  cmake --install build/strict --prefix "$HOME/.local"

Coordination and license
Firmware reconstruction is separate in ../lead-a1-src. Cross-project issue
communication belongs in private Weichwolf/lead-a1-src; see doc/coordination.txt.
Original code: GPL-3.0-only (LICENSE). Reused build policies and notices are in
THIRD_PARTY.txt. Manufacturer firmware and documentation retain their rights;
they are not included or relicensed by this project.

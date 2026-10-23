# SPDX-License-Identifier: GPL-3.0-only
# Native SLEIGH reference execution. No JVM or manufacturer code is linked.
set(GHIDRA_ROOT "$ENV{GHIDRA_ROOT}" CACHE PATH "Pinned Ghidra 12.1.2 distribution")
if(NOT GHIDRA_ROOT)
  set(candidates "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/dependencies/ghidra_12.1.2_PUBLIC"
    "$ENV{HOME}/.local/opt/ghidra_12.1.2_PUBLIC")
  foreach(candidate IN LISTS candidates)
    if(EXISTS "${candidate}/Ghidra/Features/Decompiler/src/decompile/cpp/sleigh.cc")
      set(GHIDRA_ROOT "${candidate}")
      break()
    endif()
  endforeach()
endif()
if(NOT GHIDRA_ROOT)
  message(FATAL_ERROR "Set GHIDRA_ROOT or run python3 tools/dependencies.py --fetch --ghidra")
endif()
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/reference/ghidra.json" manifest)
string(JSON count LENGTH "${manifest}" files)
math(EXPR last "${count} - 1")
foreach(index RANGE ${last})
  string(JSON relative MEMBER "${manifest}" files ${index})
  string(JSON expected GET "${manifest}" files "${relative}")
  set(path "${GHIDRA_ROOT}/${relative}")
  if(NOT EXISTS "${path}")
    message(FATAL_ERROR "Missing pinned Ghidra file: ${relative}")
  endif()
  file(SHA256 "${path}" actual)
  if(NOT actual STREQUAL expected)
    message(FATAL_ERROR "Ghidra 12.1.2 file mismatch: ${relative}")
  endif()
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${path}")
endforeach()
set(sleigh_source "${GHIDRA_ROOT}/Ghidra/Features/Decompiler/src/decompile/cpp")
set(names xml marshal space float address pcoderaw translate opcodes globalcontext
  sleigh pcodeparse pcodecompile sleighbase slghsymbol slghpatexpress slghpattern
  semantics context slaformat compression filemanage loadimage emulate opbehavior)
set(sources)
foreach(name IN LISTS names)
  list(APPEND sources "${sleigh_source}/${name}.cc")
endforeach()
find_program(LEAD_A1_PATCH NAMES patch REQUIRED)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated")
execute_process(COMMAND "${LEAD_A1_PATCH}" --batch --silent
  --input "${CMAKE_CURRENT_SOURCE_DIR}/patches/ghidra-byte-memory.patch"
  --output "${CMAKE_CURRENT_BINARY_DIR}/generated/memstate.cc"
  "${sleigh_source}/memstate.cc" COMMAND_ERROR_IS_FATAL ANY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/patches/ghidra-byte-memory.patch"
  "${CMAKE_CURRENT_SOURCE_DIR}/reference/ghidra.json")
list(APPEND sources "${CMAKE_CURRENT_BINARY_DIR}/generated/memstate.cc")
add_library(lead_a1_sleigh STATIC ${sources})
target_include_directories(lead_a1_sleigh SYSTEM PUBLIC "${sleigh_source}")
target_link_libraries(lead_a1_sleigh PUBLIC zlibstatic)
lead_a1_harden_native(lead_a1_sleigh)
if(DEFINED lead_a1_dependency_sanitizers)
  target_compile_options(lead_a1_sleigh PRIVATE
    "-fsanitize=${lead_a1_dependency_sanitizers}" -fno-sanitize-recover=all
    -fno-omit-frame-pointer -g)
endif()
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/share")
configure_file("${GHIDRA_ROOT}/Ghidra/Processors/ARM/data/languages/ARM5_le.sla"
  "${CMAKE_CURRENT_BINARY_DIR}/share/ARM5_le.sla" COPYONLY)

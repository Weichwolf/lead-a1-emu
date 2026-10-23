# SPDX-License-Identifier: GPL-3.0-only
# Strict project checks; third-party source keeps its own warning policy.
set(LEAD_A1_SANITIZER "address-undefined" CACHE STRING "none, address-undefined, or thread")
set_property(CACHE LEAD_A1_SANITIZER PROPERTY STRINGS none address-undefined thread)

include("${CMAKE_CURRENT_LIST_DIR}/Hardening.cmake")
add_library(lead_a1_quality INTERFACE)
target_link_libraries(lead_a1_quality INTERFACE lead_a1_hardening)
target_compile_options(lead_a1_quality INTERFACE
  -Wall -Wextra -Wpedantic -Werror -pedantic-errors
  -Wconversion -Wsign-conversion -Wshadow -Wundef
  -Wcast-align -Wcast-qual -Wdouble-promotion -Wnull-dereference
  -Wimplicit-fallthrough -Wnon-virtual-dtor -Wold-style-cast
  -Woverloaded-virtual -Wextra-semi -Wvla -Wswitch-enum -Wswitch-default
  -ffp-contract=off -fno-omit-frame-pointer)
target_link_options(lead_a1_quality INTERFACE
  -Wl,--fatal-warnings -Wl,-z,defs)

if(LEAD_A1_SANITIZER STREQUAL "address-undefined")
  set(lead_a1_sanitizers "address,undefined,float-divide-by-zero,unsigned-integer-overflow,implicit-conversion")
  set(lead_a1_dependency_sanitizers "address,undefined,float-divide-by-zero")
elseif(LEAD_A1_SANITIZER STREQUAL "thread")
  set(lead_a1_sanitizers "thread")
  set(lead_a1_dependency_sanitizers "thread")
elseif(NOT LEAD_A1_SANITIZER STREQUAL "none")
  message(FATAL_ERROR "Unsupported LEAD_A1_SANITIZER=${LEAD_A1_SANITIZER}")
endif()
if(DEFINED lead_a1_sanitizers)
  target_compile_options(lead_a1_quality INTERFACE
    "-fsanitize=${lead_a1_sanitizers}" -fno-sanitize-recover=all -g)
  target_link_options(lead_a1_quality INTERFACE "-fsanitize=${lead_a1_sanitizers}")
endif()

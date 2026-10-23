# SPDX-License-Identifier: GPL-3.0-only
cmake_minimum_required(VERSION 3.25)
project(lead_a1_emu VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
foreach(language C CXX)
  if(NOT CMAKE_${language}_COMPILER_ID STREQUAL "Clang" OR
     NOT CMAKE_${language}_COMPILER_VERSION VERSION_GREATER_EQUAL 19 OR
     NOT CMAKE_${language}_COMPILER_VERSION VERSION_LESS 20)
    message(FATAL_ERROR "Use LLVM 19: cmake --preset strict")
  endif()
endforeach()
include(cmake/Quality.cmake)
find_program(LEAD_A1_CLANG_TIDY NAMES clang-tidy-19 REQUIRED)
find_program(LEAD_A1_CLANG_FORMAT NAMES clang-format-19 REQUIRED)
find_package(Python3 3.11 REQUIRED COMPONENTS Interpreter)
set(CMAKE_CXX_CLANG_TIDY "${LEAD_A1_CLANG_TIDY};--warnings-as-errors=*")

add_library(lead_a1_core STATIC src/firmware.cpp)
target_include_directories(lead_a1_core PUBLIC include)
target_link_libraries(lead_a1_core PUBLIC lead_a1_quality)
add_executable(lead-a1-emu src/main.cpp)
target_link_libraries(lead-a1-emu PRIVATE lead_a1_core)

if(BUILD_TESTING)
  add_executable(lead-a1-firmware-test test/firmware_test.cpp)
  target_link_libraries(lead-a1-firmware-test PRIVATE lead_a1_core)
  add_test(NAME firmware_structure COMMAND lead-a1-firmware-test)
  add_test(NAME bootstrap_python COMMAND "${Python3_EXECUTABLE}" -B -m unittest
    discover -s test -p "test_*.py" -v)
  set_tests_properties(bootstrap_python PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
  add_test(NAME original_reference COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/references.py" --verify --allow-missing)
  set_tests_properties(original_reference PROPERTIES SKIP_RETURN_CODE 77)
  add_test(NAME unsupported_runtime COMMAND lead-a1-emu)
  set_tests_properties(unsupported_runtime PROPERTIES WILL_FAIL TRUE)
endif()

file(GLOB_RECURSE LEAD_A1_FORMAT_SOURCES CONFIGURE_DEPENDS
  "include/*.hpp" "src/*.hpp" "src/*.cpp" "test/*.cpp")
add_custom_target(format-check
  COMMAND "${LEAD_A1_CLANG_FORMAT}" --dry-run --Werror ${LEAD_A1_FORMAT_SOURCES}
  VERBATIM)
if(BUILD_TESTING)
  add_custom_target(quality
    COMMAND "${CMAKE_CTEST_COMMAND}" --output-on-failure
    DEPENDS format-check lead-a1-emu lead-a1-firmware-test VERBATIM)
endif()

install(TARGETS lead-a1-emu RUNTIME DESTINATION bin)
install(PROGRAMS tools/lead-a1-ui DESTINATION bin)
install(FILES gui/__init__.py gui/parameters.py gui/snapshot.py gui/watch.py gui/table.py
  DESTINATION share/lead-a1-emu/gui)
install(FILES reference/parameters.json DESTINATION share/lead-a1-emu/reference)
install(FILES packaging/org.lead_a1_emu.Parameters.desktop DESTINATION share/applications)
install(FILES LICENSE THIRD_PARTY.txt README.txt DESTINATION share/doc/lead-a1-emu)

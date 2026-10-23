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
find_program(LEAD_A1_LLVM_NM NAMES llvm-nm-19 REQUIRED)
find_program(LEAD_A1_LLVM_READELF NAMES llvm-readelf-19 REQUIRED)
find_program(LEAD_A1_LLVM_MC NAMES llvm-mc-19 REQUIRED)
find_program(LEAD_A1_LLVM_OBJCOPY NAMES llvm-objcopy-19 REQUIRED)
find_package(Python3 3.11 REQUIRED COMPONENTS Interpreter)
include(cmake/Dependencies.cmake)
include(cmake/ArmCore.cmake)
set(CMAKE_CXX_CLANG_TIDY "${LEAD_A1_CLANG_TIDY};--warnings-as-errors=*")

add_library(lead_a1_core STATIC src/firmware.cpp src/image.cpp)
target_include_directories(lead_a1_core PUBLIC include)
target_link_libraries(lead_a1_core PUBLIC lead_a1_quality PRIVATE lead_a1_lzo)
add_library(lead_a1_arm STATIC src/memory.cpp src/arm.cpp)
target_include_directories(lead_a1_arm PUBLIC include)
target_link_libraries(lead_a1_arm PUBLIC lead_a1_quality PRIVATE lead_a1_sleigh)
add_executable(lead-a1-emu src/main.cpp)
target_link_libraries(lead-a1-emu PRIVATE lead_a1_core)

if(BUILD_TESTING)
  add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cases.bin"
    COMMAND "${LEAD_A1_LLVM_MC}" --triple=armv5-none-eabi --mcpu=arm926ej-s
      --filetype=obj "${CMAKE_CURRENT_SOURCE_DIR}/test/arm_cases.S"
      -o "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cases.o"
    COMMAND "${LEAD_A1_LLVM_OBJCOPY}" -O binary -j .text
      "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cases.o"
      "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cases.bin"
    DEPENDS test/arm_cases.S VERBATIM)
  add_custom_target(lead-a1-arm-cases DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cases.bin")
  add_executable(lead-a1-arm-test test/arm_test.cpp)
  target_link_libraries(lead-a1-arm-test PRIVATE lead_a1_arm)
  add_dependencies(lead-a1-arm-test lead-a1-arm-cases)
  add_test(NAME arm_instructions COMMAND lead-a1-arm-test
    "${CMAKE_CURRENT_BINARY_DIR}/share/ARM5_le.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cases.bin")
  add_executable(lead-a1-memory-test test/memory_test.cpp)
  target_link_libraries(lead-a1-memory-test PRIVATE lead_a1_arm)
  add_test(NAME reference_memory COMMAND lead-a1-memory-test)
  add_executable(lead-a1-original-arm-test test/original_arm_test.cpp)
  target_link_libraries(lead-a1-original-arm-test PRIVATE lead_a1_arm)
  add_executable(lead-a1-firmware-test test/firmware_test.cpp)
  target_link_libraries(lead-a1-firmware-test PRIVATE lead_a1_core)
  add_test(NAME firmware_structure COMMAND lead-a1-firmware-test)
  add_executable(lead-a1-image-test test/image_test.cpp)
  target_link_libraries(lead-a1-image-test PRIVATE lead_a1_core)
  add_test(NAME image_sections COMMAND lead-a1-image-test)
  add_test(NAME original_images COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_images.py" $<TARGET_FILE:lead-a1-emu>
    $<TARGET_FILE:lead-a1-original-arm-test> "${CMAKE_CURRENT_BINARY_DIR}/share/ARM5_le.sla")
  set_tests_properties(original_images PROPERTIES SKIP_RETURN_CODE 77)
  add_test(NAME native_build COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/native_build.py"
    --build "${CMAKE_CURRENT_BINARY_DIR}" --sanitizer "${LEAD_A1_SANITIZER}"
    --nm "${LEAD_A1_LLVM_NM}" --readelf "${LEAD_A1_LLVM_READELF}"
    --binary $<TARGET_FILE:lead-a1-arm-test> --native $<TARGET_FILE:lead_a1_lzo>
    --native $<TARGET_FILE:zlibstatic> --native $<TARGET_FILE:lead_a1_sleigh>)
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
    DEPENDS format-check lead-a1-emu lead-a1-firmware-test lead-a1-image-test
      lead-a1-arm-test lead-a1-memory-test lead-a1-original-arm-test VERBATIM)
endif()

install(TARGETS lead-a1-emu RUNTIME DESTINATION bin)
install(PROGRAMS tools/lead-a1-ui DESTINATION bin)
install(FILES gui/__init__.py gui/parameters.py gui/snapshot.py gui/watch.py gui/table.py
  DESTINATION share/lead-a1-emu/gui)
install(FILES reference/parameters.json DESTINATION share/lead-a1-emu/reference)
install(FILES packaging/org.lead_a1_emu.Parameters.desktop DESTINATION share/applications)
install(FILES LICENSE THIRD_PARTY.txt README.txt DESTINATION share/doc/lead-a1-emu)
install(FILES LICENSES/LZO-GPL2.txt DESTINATION share/doc/lead-a1-emu/licenses)

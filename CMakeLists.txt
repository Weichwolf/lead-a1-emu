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
add_library(lead_a1_arm STATIC src/memory.cpp src/arm.cpp src/arm_status.cpp src/arm_system.cpp src/arm_mmu.cpp src/arm_data.cpp src/arm_exception.cpp src/arm_fault.cpp src/aintc.cpp src/gpio.cpp src/spi.cpp src/spi_shift.cpp)
target_include_directories(lead_a1_arm PUBLIC include)
target_link_libraries(lead_a1_arm PUBLIC lead_a1_quality PRIVATE lead_a1_sleigh)
add_library(lead_a1_sharc STATIC src/sharc_memory.cpp src/sharc_boot.cpp src/sharc.cpp src/sharc_system.cpp)
target_include_directories(lead_a1_sharc PUBLIC include)
target_link_libraries(lead_a1_sharc PUBLIC lead_a1_quality)
add_executable(lead-a1-emu src/main.cpp)
target_link_libraries(lead-a1-emu PRIVATE lead_a1_core)

if(BUILD_TESTING)
  add_executable(lead-a1-sharc-cpu-test test/sharc_cpu_test.cpp)
  target_link_libraries(lead-a1-sharc-cpu-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_native_instructions COMMAND lead-a1-sharc-cpu-test)
  add_executable(lead-a1-sharc-boot-test test/sharc_boot_test.cpp)
  target_link_libraries(lead-a1-sharc-boot-test PRIVATE lead_a1_sharc lead_a1_arm)
  add_test(NAME sharc_boot_serial COMMAND lead-a1-sharc-boot-test)
  add_executable(lead-a1-sharc-memory-test test/sharc_memory_test.cpp)
  target_link_libraries(lead-a1-sharc-memory-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_memory COMMAND lead-a1-sharc-memory-test)
  add_executable(lead-a1-spi-test test/spi_test.cpp)
  target_link_libraries(lead-a1-spi-test PRIVATE lead_a1_arm)
  add_executable(lead-a1-original-spi-test test/original_spi_test.cpp)
  target_link_libraries(lead-a1-original-spi-test PRIVATE lead_a1_arm lead_a1_sharc)
  add_test(NAME spi_serial COMMAND lead-a1-spi-test)
  add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/spi-cpu-cases.bin"
    COMMAND "${LEAD_A1_LLVM_MC}" --triple=armv5-none-eabi --mcpu=arm926ej-s
      --filetype=obj "${CMAKE_CURRENT_SOURCE_DIR}/test/spi_cpu_cases.S"
      -o "${CMAKE_CURRENT_BINARY_DIR}/generated/spi-cpu-cases.o"
    COMMAND "${LEAD_A1_LLVM_OBJCOPY}" -O binary -j .text
      "${CMAKE_CURRENT_BINARY_DIR}/generated/spi-cpu-cases.o"
      "${CMAKE_CURRENT_BINARY_DIR}/generated/spi-cpu-cases.bin"
    DEPENDS test/spi_cpu_cases.S VERBATIM)
  add_custom_target(lead-a1-spi-cpu-cases DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/generated/spi-cpu-cases.bin")
  add_executable(lead-a1-spi-cpu-test test/spi_cpu_test.cpp)
  target_link_libraries(lead-a1-spi-cpu-test PRIVATE lead_a1_arm)
  add_dependencies(lead-a1-spi-cpu-test lead-a1-spi-cpu-cases)
  add_test(NAME spi_cpu COMMAND lead-a1-spi-cpu-test
    "${CMAKE_CURRENT_BINARY_DIR}/share/ARM5_le.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/spi-cpu-cases.bin")
  add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/gpio-cases.bin"
    COMMAND "${LEAD_A1_LLVM_MC}" --triple=armv5-none-eabi --mcpu=arm926ej-s
      --filetype=obj "${CMAKE_CURRENT_SOURCE_DIR}/test/gpio_cases.S"
      -o "${CMAKE_CURRENT_BINARY_DIR}/generated/gpio-cases.o"
    COMMAND "${LEAD_A1_LLVM_OBJCOPY}" -O binary -j .text
      "${CMAKE_CURRENT_BINARY_DIR}/generated/gpio-cases.o"
      "${CMAKE_CURRENT_BINARY_DIR}/generated/gpio-cases.bin"
    DEPENDS test/gpio_cases.S VERBATIM)
  add_custom_target(lead-a1-gpio-cases DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/generated/gpio-cases.bin")
  add_executable(lead-a1-gpio-test test/gpio_test.cpp)
  target_link_libraries(lead-a1-gpio-test PRIVATE lead_a1_arm)
  add_dependencies(lead-a1-gpio-test lead-a1-gpio-cases)
  add_executable(lead-a1-original-gpio-test test/original_gpio_test.cpp)
  target_link_libraries(lead-a1-original-gpio-test PRIVATE lead_a1_arm)
  add_test(NAME gpio_mmio_cpu COMMAND lead-a1-gpio-test
    "${CMAKE_CURRENT_BINARY_DIR}/share/ARM5_le.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/gpio-cases.bin")
  add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-abort-cases.bin"
    COMMAND "${LEAD_A1_LLVM_MC}" --triple=armv5-none-eabi --mcpu=arm926ej-s
      --filetype=obj "${CMAKE_CURRENT_SOURCE_DIR}/test/arm_abort_cases.S"
      -o "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-abort-cases.o"
    COMMAND "${LEAD_A1_LLVM_OBJCOPY}" -O binary -j .text
      "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-abort-cases.o"
      "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-abort-cases.bin"
    DEPENDS test/arm_abort_cases.S VERBATIM)
  add_custom_target(lead-a1-arm-abort-cases DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-abort-cases.bin")
  add_executable(lead-a1-arm-abort-test test/arm_abort_test.cpp)
  target_link_libraries(lead-a1-arm-abort-test PRIVATE lead_a1_arm)
  add_dependencies(lead-a1-arm-abort-test lead-a1-arm-abort-cases)
  add_executable(lead-a1-original-abort-test test/original_abort_test.cpp)
  target_link_libraries(lead-a1-original-abort-test PRIVATE lead_a1_arm)
  add_dependencies(lead-a1-original-abort-test lead-a1-arm-abort-cases)
  add_test(NAME arm_mmu_aborts COMMAND lead-a1-arm-abort-test
    "${CMAKE_CURRENT_BINARY_DIR}/share/ARM5_le.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-abort-cases.bin")
  add_executable(lead-a1-original-aintc-test test/original_aintc_test.cpp)
  target_link_libraries(lead-a1-original-aintc-test PRIVATE lead_a1_arm)
  add_executable(lead-a1-aintc-nesting-test test/aintc_nesting_test.cpp)
  target_link_libraries(lead-a1-aintc-nesting-test PRIVATE lead_a1_arm)
  add_test(NAME aintc_nesting COMMAND lead-a1-aintc-nesting-test)
  add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/aintc-cases.bin"
    COMMAND "${LEAD_A1_LLVM_MC}" --triple=armv5-none-eabi --mcpu=arm926ej-s
      --filetype=obj "${CMAKE_CURRENT_SOURCE_DIR}/test/aintc_cases.S"
      -o "${CMAKE_CURRENT_BINARY_DIR}/generated/aintc-cases.o"
    COMMAND "${LEAD_A1_LLVM_OBJCOPY}" -O binary -j .text
      "${CMAKE_CURRENT_BINARY_DIR}/generated/aintc-cases.o"
      "${CMAKE_CURRENT_BINARY_DIR}/generated/aintc-cases.bin"
    DEPENDS test/aintc_cases.S VERBATIM)
  add_custom_target(lead-a1-aintc-cases DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/generated/aintc-cases.bin")
  add_executable(lead-a1-aintc-cpu-test test/aintc_cpu_test.cpp)
  target_link_libraries(lead-a1-aintc-cpu-test PRIVATE lead_a1_arm)
  add_dependencies(lead-a1-aintc-cpu-test lead-a1-aintc-cases)
  add_test(NAME aintc_cpu COMMAND lead-a1-aintc-cpu-test
    "${CMAKE_CURRENT_BINARY_DIR}/share/ARM5_le.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/aintc-cases.bin")
  add_executable(lead-a1-original-interrupt-test test/original_interrupt_test.cpp)
  target_link_libraries(lead-a1-original-interrupt-test PRIVATE lead_a1_arm)
  add_dependencies(lead-a1-original-interrupt-test lead-a1-aintc-cases)
  add_executable(lead-a1-aintc-test test/aintc_test.cpp)
  target_link_libraries(lead-a1-aintc-test PRIVATE lead_a1_arm)
  add_test(NAME aintc_mmio COMMAND lead-a1-aintc-test)
  add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-exception-cases.bin"
    COMMAND "${LEAD_A1_LLVM_MC}" --triple=armv5-none-eabi --mcpu=arm926ej-s
      --filetype=obj "${CMAKE_CURRENT_SOURCE_DIR}/test/arm_exception_cases.S"
      -o "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-exception-cases.o"
    COMMAND "${LEAD_A1_LLVM_OBJCOPY}" -O binary -j .text
      "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-exception-cases.o"
      "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-exception-cases.bin"
    DEPENDS test/arm_exception_cases.S VERBATIM)
  add_custom_target(lead-a1-arm-exception-cases DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-exception-cases.bin")
  add_executable(lead-a1-arm-exception-test test/arm_exception_test.cpp)
  target_include_directories(lead-a1-arm-exception-test PRIVATE src)
  target_link_libraries(lead-a1-arm-exception-test PRIVATE lead_a1_arm)
  add_dependencies(lead-a1-arm-exception-test lead-a1-arm-exception-cases)
  add_executable(lead-a1-original-exception-test test/original_exception_test.cpp)
  target_link_libraries(lead-a1-original-exception-test PRIVATE lead_a1_arm)
  add_dependencies(lead-a1-original-exception-test lead-a1-arm-exception-cases)
  add_test(NAME arm_exceptions COMMAND lead-a1-arm-exception-test
    "${CMAKE_CURRENT_BINARY_DIR}/share/ARM5_le.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-exception-cases.bin")
  add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cache-call.bin"
    COMMAND "${LEAD_A1_LLVM_MC}" --triple=armv5-none-eabi --mcpu=arm926ej-s
      --filetype=obj "${CMAKE_CURRENT_SOURCE_DIR}/test/arm_cache_call.S"
      -o "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cache-call.o"
    COMMAND "${LEAD_A1_LLVM_OBJCOPY}" -O binary -j .text
      "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cache-call.o"
      "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cache-call.bin"
    DEPENDS test/arm_cache_call.S VERBATIM)
  add_custom_target(lead-a1-arm-cache-call DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cache-call.bin")
  add_executable(lead-a1-original-cache-test test/original_cache_test.cpp)
  target_link_libraries(lead-a1-original-cache-test PRIVATE lead_a1_arm)
  add_dependencies(lead-a1-original-cache-test lead-a1-arm-cache-call)
  add_executable(lead-a1-arm-data-test test/arm_data_test.cpp)
  target_include_directories(lead-a1-arm-data-test PRIVATE src)
  target_link_libraries(lead-a1-arm-data-test PRIVATE lead_a1_arm)
  add_test(NAME arm_data_path COMMAND lead-a1-arm-data-test)
  add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-mmu-cases.bin"
    COMMAND "${LEAD_A1_LLVM_MC}" --triple=armv5-none-eabi --mcpu=arm926ej-s
      --filetype=obj "${CMAKE_CURRENT_SOURCE_DIR}/test/arm_mmu_cases.S"
      -o "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-mmu-cases.o"
    COMMAND "${LEAD_A1_LLVM_OBJCOPY}" -O binary -j .text
      "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-mmu-cases.o"
      "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-mmu-cases.bin"
    DEPENDS test/arm_mmu_cases.S VERBATIM)
  add_custom_target(lead-a1-arm-mmu-cases DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-mmu-cases.bin")
  add_executable(lead-a1-mmu-cpu-test test/arm_mmu_cpu_test.cpp)
  target_link_libraries(lead-a1-mmu-cpu-test PRIVATE lead_a1_arm)
  add_dependencies(lead-a1-mmu-cpu-test lead-a1-arm-mmu-cases)
  add_test(NAME arm_mmu_cpu COMMAND lead-a1-mmu-cpu-test
    "${CMAKE_CURRENT_BINARY_DIR}/share/ARM5_le.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-mmu-cases.bin")
  add_executable(lead-a1-mmu-walk-test test/arm_mmu_walk_test.cpp)
  target_include_directories(lead-a1-mmu-walk-test PRIVATE src)
  target_link_libraries(lead-a1-mmu-walk-test PRIVATE lead_a1_arm)
  add_test(NAME arm_mmu_walk COMMAND lead-a1-mmu-walk-test)
  add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-system-cases.bin"
    COMMAND "${LEAD_A1_LLVM_MC}" --triple=armv5-none-eabi --mcpu=arm926ej-s
      --filetype=obj "${CMAKE_CURRENT_SOURCE_DIR}/test/arm_system_cases.S"
      -o "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-system-cases.o"
    COMMAND "${LEAD_A1_LLVM_OBJCOPY}" -O binary -j .text
      "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-system-cases.o"
      "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-system-cases.bin"
    DEPENDS test/arm_system_cases.S VERBATIM)
  add_custom_target(lead-a1-arm-system-cases DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-system-cases.bin")
  add_executable(lead-a1-arm-system-test test/arm_system_test.cpp)
  target_link_libraries(lead-a1-arm-system-test PRIVATE lead_a1_arm)
  add_dependencies(lead-a1-arm-system-test lead-a1-arm-system-cases)
  add_test(NAME arm_system COMMAND lead-a1-arm-system-test
    "${CMAKE_CURRENT_BINARY_DIR}/share/ARM5_le.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-system-cases.bin")
  add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cases.bin"
    COMMAND "${LEAD_A1_LLVM_MC}" --triple=armv5-none-eabi --mcpu=arm926ej-s
      --filetype=obj "${CMAKE_CURRENT_SOURCE_DIR}/test/arm_cases.S"
      -o "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cases.o"
    COMMAND "${LEAD_A1_LLVM_OBJCOPY}" -O binary -j .text
      "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cases.o"
      "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cases.bin"
    DEPENDS test/arm_cases.S VERBATIM)
  add_custom_target(lead-a1-arm-cases DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cases.bin")
  add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-status-cases.bin"
    COMMAND "${LEAD_A1_LLVM_MC}" --triple=armv5-none-eabi --mcpu=arm926ej-s
      --filetype=obj "${CMAKE_CURRENT_SOURCE_DIR}/test/arm_status_cases.S"
      -o "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-status-cases.o"
    COMMAND "${LEAD_A1_LLVM_OBJCOPY}" -O binary -j .text
      "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-status-cases.o"
      "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-status-cases.bin"
    DEPENDS test/arm_status_cases.S VERBATIM)
  add_custom_target(lead-a1-arm-status-cases DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-status-cases.bin")
  add_executable(lead-a1-arm-status-test test/arm_status_test.cpp)
  target_link_libraries(lead-a1-arm-status-test PRIVATE lead_a1_arm)
  add_dependencies(lead-a1-arm-status-test lead-a1-arm-status-cases)
  add_test(NAME arm_status_banks COMMAND lead-a1-arm-status-test
    "${CMAKE_CURRENT_BINARY_DIR}/share/ARM5_le.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-status-cases.bin")
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
    $<TARGET_FILE:lead-a1-original-arm-test> "${CMAKE_CURRENT_BINARY_DIR}/share/ARM5_le.sla"
    $<TARGET_FILE:lead-a1-original-cache-test> "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cache-call.bin"
    $<TARGET_FILE:lead-a1-original-exception-test> "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-exception-cases.bin"
    $<TARGET_FILE:lead-a1-original-interrupt-test> "${CMAKE_CURRENT_BINARY_DIR}/generated/aintc-cases.bin"
    $<TARGET_FILE:lead-a1-original-aintc-test>
    $<TARGET_FILE:lead-a1-original-abort-test> "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-abort-cases.bin"
    $<TARGET_FILE:lead-a1-original-gpio-test> $<TARGET_FILE:lead-a1-original-spi-test> $<TARGET_FILE:lead-a1-sharc-boot-test>)
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
      lead-a1-arm-test lead-a1-arm-status-test lead-a1-arm-system-test
      lead-a1-memory-test lead-a1-original-arm-test lead-a1-mmu-walk-test
      lead-a1-mmu-cpu-test lead-a1-arm-data-test lead-a1-original-cache-test
      lead-a1-arm-exception-test lead-a1-original-exception-test lead-a1-aintc-test
      lead-a1-aintc-cpu-test lead-a1-original-interrupt-test lead-a1-aintc-nesting-test
      lead-a1-original-aintc-test lead-a1-arm-abort-test lead-a1-original-abort-test
      lead-a1-gpio-test lead-a1-original-gpio-test lead-a1-spi-test lead-a1-original-spi-test
      lead-a1-spi-cpu-test lead-a1-sharc-memory-test lead-a1-sharc-boot-test lead-a1-sharc-cpu-test VERBATIM)
endif()

install(TARGETS lead-a1-emu RUNTIME DESTINATION bin)
install(PROGRAMS tools/lead-a1-ui DESTINATION bin)
install(FILES gui/__init__.py gui/parameters.py gui/snapshot.py gui/watch.py gui/table.py
  DESTINATION share/lead-a1-emu/gui)
install(FILES reference/parameters.json DESTINATION share/lead-a1-emu/reference)
install(FILES packaging/org.lead_a1_emu.Parameters.desktop DESTINATION share/applications)
install(FILES LICENSE THIRD_PARTY.txt README.txt DESTINATION share/doc/lead-a1-emu)
install(FILES LICENSES/LZO-GPL2.txt DESTINATION share/doc/lead-a1-emu/licenses)

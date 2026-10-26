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

add_library(lead_a1_core STATIC
  src/firmware/firmware.cpp
  src/firmware/image.cpp
)
target_include_directories(lead_a1_core PUBLIC src)
target_link_libraries(lead_a1_core PUBLIC lead_a1_quality PRIVATE lead_a1_lzo)

add_library(lead_a1_arm STATIC
  src/arm/core/arm.cpp
  src/arm/core/arm_data.cpp
  src/arm/core/arm_exception.cpp
  src/arm/core/arm_fault.cpp
  src/arm/core/arm_mmu.cpp
  src/arm/core/arm_status.cpp
  src/arm/core/arm_system.cpp
  src/arm/peripherals/aintc.cpp
  src/arm/peripherals/gpio.cpp
  src/arm/peripherals/spi.cpp
  src/arm/peripherals/spi_shift.cpp
  src/common/memory.cpp
)
target_include_directories(lead_a1_arm PUBLIC src)
target_link_libraries(lead_a1_arm PUBLIC lead_a1_quality PRIVATE lead_a1_sleigh)

add_library(lead_a1_sharc STATIC
  src/sharc/board/sharc_dma_fabric.cpp
  src/sharc/memory/sharc_dma_arbiter.cpp
  src/sharc/core/sharc.cpp
  src/sharc/core/sharc_arithmetic_loop.cpp
  src/sharc/core/sharc_bit_shift.cpp
  src/sharc/core/sharc_cache.cpp
  src/sharc/core/sharc_calls.cpp
  src/sharc/core/sharc_compare.cpp
  src/sharc/core/sharc_compute.cpp
  src/sharc/core/sharc_dag.cpp
  src/sharc/core/sharc_dual_move.cpp
  src/sharc/core/sharc_flags.cpp
  src/sharc/core/sharc_float.cpp
  src/sharc/core/sharc_float_compute.cpp
  src/sharc/core/sharc_four_loop.cpp
  src/sharc/core/sharc_idle.cpp
  src/sharc/core/sharc_immediate_indexed.cpp
  src/sharc/core/sharc_indexed.cpp
  src/sharc/core/sharc_interrupt.cpp
  src/sharc/core/sharc_interrupt_source.cpp
  src/sharc/core/sharc_long_loop.cpp
  src/sharc/core/sharc_loop.cpp
  src/sharc/core/sharc_move.cpp
  src/sharc/core/sharc_px.cpp
  src/sharc/core/sharc_registers.cpp
  src/sharc/core/sharc_shift.cpp
  src/sharc/core/sharc_short.cpp
  src/sharc/core/sharc_system.cpp
  src/sharc/memory/sharc_iop_bridge.cpp
  src/sharc/memory/sharc_iop_port.cpp
  src/sharc/memory/sharc_memory.cpp
  src/sharc/memory/sharc_memory_bus.cpp
  src/sharc/memory/sharc_memory_view.cpp
  src/sharc/peripherals/sharc_boot.cpp
  src/sharc/peripherals/sharc_clock.cpp
  src/sharc/peripherals/sharc_control.cpp
  src/sharc/peripherals/sharc_dai_data.cpp
  src/sharc/peripherals/sharc_dai_enable.cpp
  src/sharc/peripherals/sharc_dai_pin.cpp
  src/sharc/peripherals/sharc_dai_timing.cpp
  src/sharc/peripherals/sharc_dpi_routing.cpp
  src/sharc/peripherals/sharc_pdap.cpp
  src/sharc/peripherals/sharc_pdap_input.cpp
  src/sharc/peripherals/sharc_pdap_dma.cpp
  src/sharc/peripherals/sharc_idp_dma.cpp
  src/sharc/peripherals/sharc_pcg.cpp
  src/sharc/peripherals/sharc_pcg_signal.cpp
  src/sharc/peripherals/sharc_picr.cpp
  src/sharc/peripherals/sharc_power.cpp
  src/sharc/peripherals/sharc_spi.cpp
  src/sharc/peripherals/sharc_spi_bus.cpp
  src/sharc/peripherals/sharc_spi_port.cpp
  src/sharc/peripherals/sharc_sport_dma.cpp
  src/sharc/peripherals/sharc_sport.cpp
  src/sharc/peripherals/sharc_sport_serial.cpp
  src/sharc/peripherals/sharc_sport_pair.cpp
  src/sharc/pipeline/sharc_fetch.cpp
  src/sharc/pipeline/sharc_fetch_arithmetic.cpp
  src/sharc/pipeline/sharc_fetch_branch.cpp
  src/sharc/pipeline/sharc_fetch_cache.cpp
  src/sharc/pipeline/sharc_fetch_clock.cpp
  src/sharc/pipeline/sharc_fetch_dag.cpp
  src/sharc/pipeline/sharc_fetch_effect.cpp
  src/sharc/pipeline/sharc_fetch_fixed.cpp
  src/sharc/pipeline/sharc_fetch_flow.cpp
  src/sharc/pipeline/sharc_fetch_hazard.cpp
  src/sharc/pipeline/sharc_fetch_interrupt.cpp
  src/sharc/pipeline/sharc_fetch_iop.cpp
  src/sharc/pipeline/sharc_fetch_mode.cpp
  src/sharc/pipeline/sharc_fetch_long.cpp
  src/sharc/pipeline/sharc_fetch_long_state.cpp
  src/sharc/pipeline/sharc_fetch_loop.cpp
  src/sharc/pipeline/sharc_fetch_loop_retirement.cpp
  src/sharc/pipeline/sharc_fetch_loop_state.cpp
  src/sharc/pipeline/sharc_fetch_memory.cpp
  src/sharc/pipeline/sharc_fetch_multiplier.cpp
  src/sharc/pipeline/sharc_fetch_operand.cpp
  src/sharc/pipeline/sharc_fetch_visa.cpp
  src/sharc/pipeline/sharc_fetch_visa_loop.cpp
  src/sharc/visa/sharc_visa.cpp
  src/sharc/visa/sharc_visa_decode.cpp
  src/sharc/visa/sharc_visa_execute.cpp
  src/sharc/visa/sharc_visa_length.cpp
)
target_include_directories(lead_a1_sharc PUBLIC src)
target_link_libraries(lead_a1_sharc PUBLIC lead_a1_quality)

add_library(lead_a1_board STATIC src/board/processor_links.cpp src/board/spi_sharc_link.cpp
  src/board/spi_sharc_bus.cpp)
target_include_directories(lead_a1_board PUBLIC src)
target_link_libraries(lead_a1_board PUBLIC lead_a1_arm lead_a1_sharc)

add_executable(lead-a1-emu src/app/main.cpp)
target_link_libraries(lead-a1-emu PRIVATE lead_a1_core)

if(BUILD_TESTING)
  add_executable(lead-a1-spi-sharc-bus-test test/spi_sharc_bus_test.cpp)
  target_link_libraries(lead-a1-spi-sharc-bus-test PRIVATE lead_a1_board)
  add_test(NAME processor_shared_spi_sharc_bus COMMAND lead-a1-spi-sharc-bus-test)
  add_executable(lead-a1-sharc-visa-interrupt-test test/sharc_visa_interrupt_test.cpp)
  target_link_libraries(lead-a1-sharc-visa-interrupt-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_visa_programmable_interrupts COMMAND lead-a1-sharc-visa-interrupt-test)
  add_executable(lead-a1-original-visa-interrupt-test test/original_visa_interrupt_test.cpp)
  target_link_libraries(lead-a1-original-visa-interrupt-test PRIVATE lead_a1_sharc)
  add_executable(lead-a1-spi-slave-boot-test test/spi_slave_boot_test.cpp)
  target_link_libraries(lead-a1-spi-slave-boot-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_spi_slave_boot_profile COMMAND lead-a1-spi-slave-boot-test)
  add_executable(lead-a1-spi-sharc-link-test test/spi_sharc_link_test.cpp)
  target_link_libraries(lead-a1-spi-sharc-link-test PRIVATE lead_a1_board)
  add_test(NAME processor_spi_sharc_wires COMMAND lead-a1-spi-sharc-link-test)
  add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/processor-link-cases.bin"
    COMMAND "${LEAD_A1_LLVM_MC}" --triple=armv5-none-eabi --mcpu=arm926ej-s
      --filetype=obj "${CMAKE_CURRENT_SOURCE_DIR}/test/processor_link_cases.S"
      -o "${CMAKE_CURRENT_BINARY_DIR}/generated/processor-link-cases.o"
    COMMAND "${LEAD_A1_LLVM_OBJCOPY}" -O binary -j .text
      "${CMAKE_CURRENT_BINARY_DIR}/generated/processor-link-cases.o"
      "${CMAKE_CURRENT_BINARY_DIR}/generated/processor-link-cases.bin"
    DEPENDS test/processor_link_cases.S VERBATIM)
  add_custom_target(lead-a1-processor-link-cases
    DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/generated/processor-link-cases.bin")
  add_executable(lead-a1-processor-links-test test/processor_links_test.cpp)
  target_link_libraries(lead-a1-processor-links-test PRIVATE lead_a1_board)
  add_dependencies(lead-a1-processor-links-test lead-a1-processor-link-cases)
  add_test(NAME processor_gpio_sharc_links COMMAND lead-a1-processor-links-test
    "${CMAKE_CURRENT_BINARY_DIR}/share/ARM5_le.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/processor-link-cases.bin")
  add_executable(lead-a1-sharc-dma-fabric-test test/sharc_dma_fabric_test.cpp)
  target_link_libraries(lead-a1-sharc-dma-fabric-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_dma_real_producer_fabric COMMAND lead-a1-sharc-dma-fabric-test)
  add_executable(lead-a1-sharc-dma-arbiter-test test/sharc_dma_arbiter_test.cpp)
  target_link_libraries(lead-a1-sharc-dma-arbiter-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_dma_group_arbitration COMMAND lead-a1-sharc-dma-arbiter-test)
  add_executable(lead-a1-sharc-idp-dma-bus-test test/sharc_idp_dma_bus_test.cpp)
  target_link_libraries(lead-a1-sharc-idp-dma-bus-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_idp_dma_posted_parameters COMMAND lead-a1-sharc-idp-dma-bus-test)
  add_executable(lead-a1-sharc-idp-dma-test test/sharc_idp_dma_test.cpp)
  target_link_libraries(lead-a1-sharc-idp-dma-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_idp_fifo_dma COMMAND lead-a1-sharc-idp-dma-test)
  add_executable(lead-a1-sharc-pcg-bus-test test/sharc_pcg_bus_test.cpp)
  target_link_libraries(lead-a1-sharc-pcg-bus-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_pcg_posted_control COMMAND lead-a1-sharc-pcg-bus-test)
  add_executable(lead-a1-sharc-pcg-test test/sharc_pcg_test.cpp)
  target_link_libraries(lead-a1-sharc-pcg-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_pcg_source_clock_frames COMMAND lead-a1-sharc-pcg-test)
  add_executable(lead-a1-sharc-picr-test test/sharc_picr_test.cpp)
  target_link_libraries(lead-a1-sharc-picr-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_programmable_interrupt_routing COMMAND lead-a1-sharc-picr-test)
  add_executable(lead-a1-sharc-picr-bus-test test/sharc_picr_bus_test.cpp)
  target_link_libraries(lead-a1-sharc-picr-bus-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_picr_posted_control COMMAND lead-a1-sharc-picr-bus-test)
  add_executable(lead-a1-sharc-sport-bus-test test/sharc_sport_bus_test.cpp)
  target_link_libraries(lead-a1-sharc-sport-bus-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_sport_posted_control COMMAND lead-a1-sharc-sport-bus-test)
  add_executable(lead-a1-sharc-sport-test test/sharc_sport_test.cpp)
  target_link_libraries(lead-a1-sharc-sport-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_sport_control_and_serial COMMAND lead-a1-sharc-sport-test)
  add_executable(lead-a1-sharc-sport-dma-bus-test test/sharc_sport_dma_bus_test.cpp)
  target_link_libraries(lead-a1-sharc-sport-dma-bus-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_sport_dma_posted_parameters COMMAND lead-a1-sharc-sport-dma-bus-test)
  add_executable(lead-a1-sharc-sport-dma-test test/sharc_sport_dma_test.cpp)
  target_link_libraries(lead-a1-sharc-sport-dma-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_sport_dma_channels COMMAND lead-a1-sharc-sport-dma-test)
  add_executable(lead-a1-sharc-dai-data-test test/sharc_dai_data_test.cpp)
  target_link_libraries(lead-a1-sharc-dai-data-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_dai_serial_data_routing COMMAND lead-a1-sharc-dai-data-test)
  add_executable(lead-a1-sharc-dai-data-bus-test test/sharc_dai_data_bus_test.cpp)
  target_link_libraries(lead-a1-sharc-dai-data-bus-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_dai_data_posted_io COMMAND lead-a1-sharc-dai-data-bus-test)
  add_executable(lead-a1-sharc-dai-pin-bus-test test/sharc_dai_pin_bus_test.cpp)
  target_link_libraries(lead-a1-sharc-dai-pin-bus-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_dai_pin_posted_io COMMAND lead-a1-sharc-dai-pin-bus-test)
  add_executable(lead-a1-sharc-dai-pin-test test/sharc_dai_pin_test.cpp)
  target_link_libraries(lead-a1-sharc-dai-pin-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_dai_pin_data_routing COMMAND lead-a1-sharc-dai-pin-test)
  add_executable(lead-a1-sharc-spi-port-bus-test test/sharc_spi_port_bus_test.cpp)
  target_link_libraries(lead-a1-sharc-spi-port-bus-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_spi_posted_control_buffers COMMAND lead-a1-sharc-spi-port-bus-test)
  add_executable(lead-a1-sharc-spi-port-test test/sharc_spi_port_test.cpp)
  target_link_libraries(lead-a1-sharc-spi-port-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_spi_control_buffers COMMAND lead-a1-sharc-spi-port-test)
  add_executable(lead-a1-sharc-dpi-bus-test test/sharc_dpi_bus_test.cpp)
  target_link_libraries(lead-a1-sharc-dpi-bus-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_dpi_posted_io COMMAND lead-a1-sharc-dpi-bus-test)
  add_executable(lead-a1-sharc-dpi-test test/sharc_dpi_routing_test.cpp)
  target_link_libraries(lead-a1-sharc-dpi-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_dpi_pin_output_enable_routing COMMAND lead-a1-sharc-dpi-test)
  add_executable(lead-a1-sharc-fetch-iop-test test/sharc_fetch_iop_test.cpp)
  target_link_libraries(lead-a1-sharc-fetch-iop-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_fetched_iop_waits COMMAND lead-a1-sharc-fetch-iop-test)
  add_executable(lead-a1-sharc-fetch-visa-loop-test test/sharc_fetch_visa_loop_test.cpp)
  target_link_libraries(lead-a1-sharc-fetch-visa-loop-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_clocked_visa_counter_loops COMMAND lead-a1-sharc-fetch-visa-loop-test)
  add_executable(lead-a1-sharc-fetch-visa-test test/sharc_fetch_visa_test.cpp)
  target_link_libraries(lead-a1-sharc-fetch-visa-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_clocked_visa_fetch COMMAND lead-a1-sharc-fetch-visa-test)
  add_executable(lead-a1-sharc-pdap-test test/sharc_pdap_test.cpp)
  target_link_libraries(lead-a1-sharc-pdap-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_pdap_acquisition_fifo COMMAND lead-a1-sharc-pdap-test)
  add_executable(lead-a1-sharc-dai-timing-test test/sharc_dai_timing_test.cpp)
  target_link_libraries(lead-a1-sharc-dai-timing-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_dai_clock_frame_routing COMMAND lead-a1-sharc-dai-timing-test)
  add_executable(lead-a1-sharc-iop-bridge-test test/sharc_iop_bridge_test.cpp)
  target_link_libraries(lead-a1-sharc-iop-bridge-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_iop_posted_control_transfers COMMAND lead-a1-sharc-iop-bridge-test)
  add_executable(lead-a1-sharc-dai-enable-test test/sharc_dai_enable_test.cpp)
  target_link_libraries(lead-a1-sharc-dai-enable-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_dai_pin_enable_routing COMMAND lead-a1-sharc-dai-enable-test)
  add_executable(lead-a1-sharc-power-test test/sharc_power_test.cpp)
  target_link_libraries(lead-a1-sharc-power-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_peripheral_clock_gates COMMAND lead-a1-sharc-power-test)
  add_executable(lead-a1-sharc-visa-cpu-test test/sharc_visa_cpu_test.cpp)
  target_link_libraries(lead-a1-sharc-visa-cpu-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_visa_instruction_execution COMMAND lead-a1-sharc-visa-cpu-test)
  add_executable(lead-a1-sharc-visa-test test/sharc_visa_test.cpp)
  target_link_libraries(lead-a1-sharc-visa-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_visa_alignment_buffer COMMAND lead-a1-sharc-visa-test)
  add_executable(lead-a1-sharc-fetch-memory-test test/sharc_fetch_memory_test.cpp)
  target_link_libraries(lead-a1-sharc-fetch-memory-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_f2_memory_responses COMMAND lead-a1-sharc-fetch-memory-test)
  add_executable(lead-a1-sharc-immediate-indexed-test test/sharc_immediate_indexed_test.cpp)
  target_link_libraries(lead-a1-sharc-immediate-indexed-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_immediate_indexed_moves COMMAND lead-a1-sharc-immediate-indexed-test)
  add_executable(lead-a1-sharc-fetch-arithmetic-cache-test test/sharc_fetch_arithmetic_cache_test.cpp)
  target_link_libraries(lead-a1-sharc-fetch-arithmetic-cache-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_fetched_arithmetic_cache COMMAND lead-a1-sharc-fetch-arithmetic-cache-test)
  add_executable(lead-a1-sharc-loop-idle-test test/sharc_loop_idle_test.cpp)
  target_link_libraries(lead-a1-sharc-loop-idle-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_loop_idle_dma_interrupt_return COMMAND lead-a1-sharc-loop-idle-test)
  add_executable(lead-a1-sharc-fetch-arithmetic-short-test test/sharc_fetch_arithmetic_short_test.cpp)
  target_link_libraries(lead-a1-sharc-fetch-arithmetic-short-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_fetched_arithmetic_short_loops COMMAND lead-a1-sharc-fetch-arithmetic-short-test)
  add_executable(lead-a1-sharc-fetch-arithmetic-test test/sharc_fetch_arithmetic_test.cpp)
  target_link_libraries(lead-a1-sharc-fetch-arithmetic-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_fetched_arithmetic_loops COMMAND lead-a1-sharc-fetch-arithmetic-test)
  add_executable(lead-a1-sharc-fetch-deferred-test test/sharc_fetch_deferred_test.cpp)
  target_link_libraries(lead-a1-sharc-fetch-deferred-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_fetched_deferred_arithmetic_setup COMMAND lead-a1-sharc-fetch-deferred-test)
  add_executable(lead-a1-sharc-fetch-count1-test test/sharc_fetch_count1_test.cpp)
  target_link_libraries(lead-a1-sharc-fetch-count1-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_fetched_four_single_iteration COMMAND lead-a1-sharc-fetch-count1-test)
  add_executable(lead-a1-sharc-fetch-long-test test/sharc_fetch_long_test.cpp)
  target_link_libraries(lead-a1-sharc-fetch-long-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_fetched_long_counter_loops COMMAND lead-a1-sharc-fetch-long-test)
  add_executable(lead-a1-sharc-dag-loop-test test/sharc_dag_loop_test.cpp)
  target_link_libraries(lead-a1-sharc-dag-loop-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_dag_load_and_single_loop COMMAND lead-a1-sharc-dag-loop-test)
  add_executable(lead-a1-sharc-idle-branch-test test/sharc_idle_branch_test.cpp)
  target_link_libraries(lead-a1-sharc-idle-branch-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_delayed_idle_and_interrupt_fetch COMMAND lead-a1-sharc-idle-branch-test)
  add_executable(lead-a1-sharc-float-fixed-stall-test test/sharc_float_fixed_stall_test.cpp)
  target_link_libraries(lead-a1-sharc-float-fixed-stall-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_floating_to_fixed_stalls COMMAND lead-a1-sharc-float-fixed-stall-test)
  add_executable(lead-a1-sharc-multiplier-branch-test test/sharc_multiplier_branch_test.cpp)
  target_link_libraries(lead-a1-sharc-multiplier-branch-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_multiplier_branch_stalls COMMAND lead-a1-sharc-multiplier-branch-test)
  add_executable(lead-a1-sharc-compare-test test/sharc_compare_test.cpp)
  target_link_libraries(lead-a1-sharc-compare-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_compare_and_relational_conditions COMMAND lead-a1-sharc-compare-test)
  add_executable(lead-a1-sharc-float-multiply-test test/sharc_float_multiply_test.cpp)
  target_link_libraries(lead-a1-sharc-float-multiply-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_floating_multiply COMMAND lead-a1-sharc-float-multiply-test)
  add_test(NAME sharc_float_product_oracle COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/sharc_float_oracle.py" --multiply $<TARGET_FILE:lead-a1-sharc-float-multiply-test>)
  add_executable(lead-a1-sharc-float-test test/sharc_float_test.cpp)
  target_link_libraries(lead-a1-sharc-float-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_finite_floating_alu COMMAND lead-a1-sharc-float-test)
  add_test(NAME sharc_float_exact_oracle COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/sharc_float_oracle.py" $<TARGET_FILE:lead-a1-sharc-float-test>)
  add_executable(lead-a1-sharc-memory-bus-test test/sharc_memory_bus_test.cpp)
  target_link_libraries(lead-a1-sharc-memory-bus-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_ram_bus_arbitration COMMAND lead-a1-sharc-memory-bus-test)
  add_executable(lead-a1-sharc-spi-bus-test test/sharc_spi_bus_test.cpp)
  target_link_libraries(lead-a1-sharc-spi-bus-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_spi_word_grants COMMAND lead-a1-sharc-spi-bus-test)
  add_executable(lead-a1-sharc-operand-test test/sharc_operand_test.cpp)
  target_link_libraries(lead-a1-sharc-operand-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_captured_dag_addresses COMMAND lead-a1-sharc-operand-test)
  add_executable(lead-a1-sharc-dag-stall-test test/sharc_dag_stall_test.cpp)
  target_link_libraries(lead-a1-sharc-dag-stall-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_dag_load_stalls COMMAND lead-a1-sharc-dag-stall-test)
  add_executable(lead-a1-sharc-fetch-loop-test test/sharc_fetch_loop_test.cpp)
  target_link_libraries(lead-a1-sharc-fetch-loop-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_fetch_single_loops COMMAND lead-a1-sharc-fetch-loop-test)
  add_executable(lead-a1-sharc-quiet-clock-test test/sharc_quiet_clock_test.cpp)
  target_link_libraries(lead-a1-sharc-quiet-clock-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_quiet_clock COMMAND lead-a1-sharc-quiet-clock-test)
  add_executable(lead-a1-sharc-branch-pipeline-test test/sharc_branch_pipeline_test.cpp)
  target_link_libraries(lead-a1-sharc-branch-pipeline-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_branch_pipeline COMMAND lead-a1-sharc-branch-pipeline-test)
  add_executable(lead-a1-sharc-fetch-flow-test test/sharc_fetch_flow_test.cpp)
  target_link_libraries(lead-a1-sharc-fetch-flow-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_fetch_flow COMMAND lead-a1-sharc-fetch-flow-test)
  add_executable(lead-a1-sharc-fetch-cache-test test/sharc_fetch_cache_test.cpp)
  target_link_libraries(lead-a1-sharc-fetch-cache-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_fetch_cache COMMAND lead-a1-sharc-fetch-cache-test)
  add_executable(lead-a1-sharc-fetch-test test/sharc_fetch_test.cpp)
  target_link_libraries(lead-a1-sharc-fetch-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_instruction_fetch COMMAND lead-a1-sharc-fetch-test)
  add_executable(lead-a1-sharc-interrupt-test test/sharc_interrupt_test.cpp)
  target_link_libraries(lead-a1-sharc-interrupt-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_p1i_interrupts COMMAND lead-a1-sharc-interrupt-test)
  add_executable(lead-a1-sharc-lirptl-test test/sharc_lirptl_test.cpp)
  target_link_libraries(lead-a1-sharc-lirptl-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_lirptl_programmable_interrupts COMMAND lead-a1-sharc-lirptl-test)
  add_executable(lead-a1-sharc-idle-test test/sharc_idle_test.cpp)
  target_link_libraries(lead-a1-sharc-idle-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_idle_dma COMMAND lead-a1-sharc-idle-test)
  add_executable(lead-a1-sharc-cache-test test/sharc_cache_test.cpp)
  target_link_libraries(lead-a1-sharc-cache-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_cache_invalidation COMMAND lead-a1-sharc-cache-test)
  add_executable(lead-a1-sharc-dual-move-test test/sharc_dual_move_test.cpp)
  target_link_libraries(lead-a1-sharc-dual-move-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_dual_moves COMMAND lead-a1-sharc-dual-move-test)
  add_executable(lead-a1-sharc-arithmetic-loop-test test/sharc_arithmetic_loop_test.cpp)
  target_link_libraries(lead-a1-sharc-arithmetic-loop-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_arithmetic_loops COMMAND lead-a1-sharc-arithmetic-loop-test)
  add_executable(lead-a1-sharc-bit-shift-test test/sharc_bit_shift_test.cpp)
  target_link_libraries(lead-a1-sharc-bit-shift-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_bit_operations COMMAND lead-a1-sharc-bit-shift-test)
  add_executable(lead-a1-sharc-parallel-move-test test/sharc_parallel_move_test.cpp)
  target_link_libraries(lead-a1-sharc-parallel-move-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_parallel_register_moves COMMAND lead-a1-sharc-parallel-move-test)
  add_executable(lead-a1-sharc-px-test test/sharc_px_test.cpp)
  target_link_libraries(lead-a1-sharc-px-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_px_transfers COMMAND lead-a1-sharc-px-test)
  add_executable(lead-a1-sharc-four-loop-test test/sharc_four_loop_test.cpp)
  target_link_libraries(lead-a1-sharc-four-loop-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_four_instruction_loops COMMAND lead-a1-sharc-four-loop-test)
  add_executable(lead-a1-sharc-ram-move-test test/sharc_ram_move_test.cpp)
  target_link_libraries(lead-a1-sharc-ram-move-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_short_and_indexed_moves COMMAND lead-a1-sharc-ram-move-test)
  add_executable(lead-a1-sharc-long-loop-test test/sharc_long_loop_test.cpp)
  target_link_libraries(lead-a1-sharc-long-loop-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_long_count_loops COMMAND lead-a1-sharc-long-loop-test)
  add_executable(lead-a1-sharc-shift-test test/sharc_shift_test.cpp)
  target_link_libraries(lead-a1-sharc-shift-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_shifts COMMAND lead-a1-sharc-shift-test)
  add_executable(lead-a1-sharc-register-move-test test/sharc_register_move_test.cpp)
  target_link_libraries(lead-a1-sharc-register-move-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_register_moves COMMAND lead-a1-sharc-register-move-test)
  add_executable(lead-a1-sharc-arithmetic-test test/sharc_arithmetic_test.cpp)
  target_link_libraries(lead-a1-sharc-arithmetic-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_integer_arithmetic COMMAND lead-a1-sharc-arithmetic-test)
  add_executable(lead-a1-sharc-spi-test test/sharc_spi_test.cpp)
  target_link_libraries(lead-a1-sharc-spi-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_runtime_spi COMMAND lead-a1-sharc-spi-test)
  add_executable(lead-a1-sharc-compute-test test/sharc_compute_test.cpp)
  target_link_libraries(lead-a1-sharc-compute-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_logical_compute COMMAND lead-a1-sharc-compute-test)
  add_executable(lead-a1-sharc-control-test test/sharc_control_test.cpp)
  target_link_libraries(lead-a1-sharc-control-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_control COMMAND lead-a1-sharc-control-test)
  add_executable(lead-a1-sharc-data-test test/sharc_data_test.cpp)
  target_link_libraries(lead-a1-sharc-data-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_data COMMAND lead-a1-sharc-data-test)
  add_executable(lead-a1-sharc-flags-test test/sharc_flags_test.cpp)
  target_link_libraries(lead-a1-sharc-flags-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_flags COMMAND lead-a1-sharc-flags-test)
  add_executable(lead-a1-sharc-clock-test test/sharc_clock_test.cpp)
  target_link_libraries(lead-a1-sharc-clock-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_clock COMMAND lead-a1-sharc-clock-test)
  add_executable(lead-a1-sharc-loop-test test/sharc_loop_test.cpp)
  target_link_libraries(lead-a1-sharc-loop-test PRIVATE lead_a1_sharc)
  add_test(NAME sharc_short_count_loops COMMAND lead-a1-sharc-loop-test)
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
  target_link_libraries(lead-a1-original-spi-test PRIVATE lead_a1_board)
  add_executable(lead-a1-original-sharc-test test/original_sharc_test.cpp)
  target_link_libraries(lead-a1-original-sharc-test PRIVATE lead_a1_sharc)
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
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_images.py" $<TARGET_FILE:lead-a1-emu>)
  set_tests_properties(original_images PROPERTIES SKIP_RETURN_CODE 77)
  set(LEAD_A1_ORIGINAL_NATIVE_CHECK_ARGS
    $<TARGET_FILE:lead-a1-original-arm-test> "${CMAKE_CURRENT_BINARY_DIR}/share/ARM5_le.sla"
    $<TARGET_FILE:lead-a1-original-cache-test> "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-cache-call.bin"
    $<TARGET_FILE:lead-a1-original-exception-test> "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-exception-cases.bin"
    $<TARGET_FILE:lead-a1-original-interrupt-test> "${CMAKE_CURRENT_BINARY_DIR}/generated/aintc-cases.bin"
    $<TARGET_FILE:lead-a1-original-aintc-test>
    $<TARGET_FILE:lead-a1-original-abort-test> "${CMAKE_CURRENT_BINARY_DIR}/generated/arm-abort-cases.bin"
    $<TARGET_FILE:lead-a1-original-gpio-test> $<TARGET_FILE:lead-a1-original-spi-test> $<TARGET_FILE:lead-a1-sharc-boot-test>)
  function(lead_a1_add_original_check name check)
    add_test(NAME "${name}" COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/original_images.py" $<TARGET_FILE:lead-a1-emu>
      ${LEAD_A1_ORIGINAL_NATIVE_CHECK_ARGS} --native-check "${check}")
    set_tests_properties("${name}" PROPERTIES SKIP_RETURN_CODE 77)
  endfunction()
  # The checks already own separate processes/state. Preserve every executable,
  # fixture and instruction expectation while keeping CTest's default per-test
  # time bound for each group instead of one cumulative ARM/DSP aggregate.
  lead_a1_add_original_check(original_arm_initialization arm)
  lead_a1_add_original_check(original_arm_cache cache)
  lead_a1_add_original_check(original_arm_exceptions exceptions)
  lead_a1_add_original_check(original_arm_interrupts interrupts)
  lead_a1_add_original_check(original_arm_aintc aintc)
  lead_a1_add_original_check(original_arm_aborts aborts)
  lead_a1_add_original_check(original_arm_gpio gpio)
  foreach(part RANGE 0 1)
    lead_a1_add_original_check("original_arm_spi_controls_${part}" "spi-controls-${part}")
    lead_a1_add_original_check("original_arm_spi_uploader_${part}" "spi-uploader-${part}")
    foreach(channel RANGE 0 1)
      lead_a1_add_original_check("original_arm_spi_reset_kernel_${part}_${channel}"
        "spi-reset-kernel-${part}-${channel}")
      lead_a1_add_original_check("original_arm_spi_kernel_${part}_${channel}"
        "spi-kernel-${part}-${channel}")
    endforeach()
  endforeach()
  lead_a1_add_original_check(original_sharc_boot_images boot)
  add_test(NAME original_sharc_images COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_images.py" $<TARGET_FILE:lead-a1-emu>
    --sharc-only $<TARGET_FILE:lead-a1-original-sharc-test>)
  set_tests_properties(original_sharc_images PROPERTIES SKIP_RETURN_CODE 77)
  add_test(NAME original_sharc_visa_interrupts COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_images.py" $<TARGET_FILE:lead-a1-emu>
    --sharc-only $<TARGET_FILE:lead-a1-original-visa-interrupt-test>)
  set_tests_properties(original_sharc_visa_interrupts PROPERTIES SKIP_RETURN_CODE 77)
  add_test(NAME original_sharc_deferred_f2 COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_images.py" $<TARGET_FILE:lead-a1-emu>
    --sharc-only $<TARGET_FILE:lead-a1-original-sharc-test> --sharc-deferred-f2)
  set_tests_properties(original_sharc_deferred_f2 PROPERTIES SKIP_RETURN_CODE 77)
  add_test(NAME original_sharc_visa_instructions COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_images.py" $<TARGET_FILE:lead-a1-emu>
    --sharc-only $<TARGET_FILE:lead-a1-original-sharc-test> --sharc-visa-instructions)
  set_tests_properties(original_sharc_visa_instructions PROPERTIES SKIP_RETURN_CODE 77)
  add_test(NAME original_sharc_clocked_iop COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_images.py" $<TARGET_FILE:lead-a1-emu>
    --sharc-only $<TARGET_FILE:lead-a1-original-sharc-test> --sharc-clocked-iop)
  set_tests_properties(original_sharc_clocked_iop PROPERTIES SKIP_RETURN_CODE 77)
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
  "src/*.hpp" "src/*.cpp" "test/*.cpp")
add_custom_target(format-check
  COMMAND "${LEAD_A1_CLANG_FORMAT}" --dry-run --Werror ${LEAD_A1_FORMAT_SOURCES}
  VERBATIM)
if(BUILD_TESTING)
  add_custom_target(quality
    COMMAND "${CMAKE_CTEST_COMMAND}" --output-on-failure
    DEPENDS lead-a1-sharc-dma-fabric-test lead-a1-sharc-dma-arbiter-test lead-a1-sharc-idp-dma-bus-test lead-a1-sharc-idp-dma-test lead-a1-sharc-pcg-bus-test lead-a1-sharc-pcg-test lead-a1-sharc-picr-test lead-a1-sharc-picr-bus-test format-check lead-a1-emu lead-a1-firmware-test lead-a1-image-test
      lead-a1-processor-links-test
      lead-a1-spi-sharc-link-test
      lead-a1-spi-sharc-bus-test
      lead-a1-spi-slave-boot-test
      lead-a1-sharc-visa-interrupt-test
      lead-a1-original-visa-interrupt-test
      lead-a1-sharc-lirptl-test
      lead-a1-sharc-sport-test
      lead-a1-sharc-sport-bus-test
      lead-a1-sharc-sport-dma-test
      lead-a1-sharc-sport-dma-bus-test
      lead-a1-sharc-dai-data-test
      lead-a1-sharc-dai-data-bus-test
      lead-a1-sharc-dai-pin-test
      lead-a1-sharc-dai-pin-bus-test
      lead-a1-sharc-fetch-test
      lead-a1-sharc-fetch-iop-test
      lead-a1-sharc-spi-port-bus-test
      lead-a1-sharc-spi-port-test
      lead-a1-sharc-dpi-test
      lead-a1-sharc-dpi-bus-test
      lead-a1-sharc-fetch-visa-test
      lead-a1-sharc-fetch-visa-loop-test
      lead-a1-sharc-fetch-cache-test
      lead-a1-sharc-fetch-flow-test
      lead-a1-sharc-branch-pipeline-test
      lead-a1-sharc-quiet-clock-test
      lead-a1-sharc-fetch-loop-test
      lead-a1-sharc-dag-stall-test
      lead-a1-sharc-dag-loop-test
      lead-a1-sharc-fetch-long-test
      lead-a1-sharc-fetch-count1-test
      lead-a1-sharc-fetch-deferred-test
      lead-a1-sharc-fetch-arithmetic-test
      lead-a1-sharc-fetch-arithmetic-short-test
      lead-a1-sharc-fetch-arithmetic-cache-test
      lead-a1-sharc-immediate-indexed-test
      lead-a1-sharc-fetch-memory-test
      lead-a1-sharc-visa-test
      lead-a1-sharc-visa-cpu-test
      lead-a1-sharc-power-test
      lead-a1-sharc-dai-enable-test
      lead-a1-sharc-dai-timing-test
      lead-a1-sharc-pdap-test
      lead-a1-sharc-iop-bridge-test
      lead-a1-sharc-loop-idle-test
      lead-a1-sharc-operand-test lead-a1-sharc-memory-bus-test lead-a1-sharc-spi-bus-test lead-a1-sharc-float-test lead-a1-sharc-float-multiply-test lead-a1-sharc-compare-test lead-a1-sharc-multiplier-branch-test lead-a1-sharc-float-fixed-stall-test lead-a1-sharc-idle-branch-test
      lead-a1-arm-test lead-a1-arm-status-test lead-a1-arm-system-test
      lead-a1-memory-test lead-a1-original-arm-test lead-a1-mmu-walk-test
      lead-a1-mmu-cpu-test lead-a1-arm-data-test lead-a1-original-cache-test
      lead-a1-arm-exception-test lead-a1-original-exception-test lead-a1-aintc-test
      lead-a1-aintc-cpu-test lead-a1-original-interrupt-test lead-a1-aintc-nesting-test
      lead-a1-original-aintc-test lead-a1-arm-abort-test lead-a1-original-abort-test
      lead-a1-gpio-test lead-a1-original-gpio-test lead-a1-spi-test lead-a1-original-spi-test lead-a1-original-sharc-test lead-a1-sharc-arithmetic-loop-test lead-a1-sharc-cache-test lead-a1-sharc-dual-move-test lead-a1-sharc-interrupt-test lead-a1-sharc-idle-test
      lead-a1-spi-cpu-test lead-a1-sharc-memory-test lead-a1-sharc-boot-test lead-a1-sharc-cpu-test lead-a1-sharc-clock-test lead-a1-sharc-loop-test lead-a1-sharc-flags-test lead-a1-sharc-control-test lead-a1-sharc-data-test lead-a1-sharc-compute-test lead-a1-sharc-spi-test lead-a1-sharc-arithmetic-test lead-a1-sharc-register-move-test lead-a1-sharc-shift-test lead-a1-sharc-long-loop-test lead-a1-sharc-ram-move-test lead-a1-sharc-four-loop-test lead-a1-sharc-px-test lead-a1-sharc-bit-shift-test lead-a1-sharc-parallel-move-test VERBATIM)
endif()

install(TARGETS lead-a1-emu RUNTIME DESTINATION bin)
install(PROGRAMS tools/lead-a1-ui DESTINATION bin)
install(FILES gui/__init__.py gui/parameters.py gui/snapshot.py gui/watch.py gui/table.py
  DESTINATION share/lead-a1-emu/gui)
install(FILES reference/parameters.json DESTINATION share/lead-a1-emu/reference)
install(FILES packaging/org.lead_a1_emu.Parameters.desktop DESTINATION share/applications)
install(FILES LICENSE THIRD_PARTY.txt README.txt DESTINATION share/doc/lead-a1-emu)
install(FILES LICENSES/LZO-GPL2.txt DESTINATION share/doc/lead-a1-emu/licenses)

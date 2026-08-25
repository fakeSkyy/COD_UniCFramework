# Host performance/resource gate. This file is standalone today and remains a
# valid child CMakeLists when the tests root elects to add_subdirectory later.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.22)
    project(cod_host_performance C)
    enable_testing()
    get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
endif()

if(NOT DEFINED REPO_ROOT)
    message(FATAL_ERROR "REPO_ROOT must name the repository when used as a child CMakeLists")
endif()
if(NOT CMAKE_C_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR "The resource gate requires GCC -fstack-usage output")
endif()

find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter)
find_program(PERF_GNU_SIZE NAMES size REQUIRED)

set(PERF_CALIBRATION_MAX_SPREAD "0.30" CACHE STRING
    "Maximum MAD/median spread of the calibration workload")
set(PERF_RINGBUF_MAX_RATIO "6.0" CACHE STRING
    "Maximum RingBuf Put/Get time relative to same-process calibration")
set(PERF_PID_MAX_RATIO "20.0" CACHE STRING
    "Maximum configured PID step time relative to same-process calibration")
set(PERF_AHRS_MAX_RATIO "1000.0" CACHE STRING
    "Maximum AHRS update time relative to same-process calibration")
set(PERF_MAX_STACK_BYTES "512" CACHE STRING
    "Maximum static stack usage of one host-compiled production function")
set(PERF_BENCHMARK_MAX_TEXT "30000" CACHE STRING "Benchmark host ELF text limit")
set(PERF_BENCHMARK_MAX_DATA "2048" CACHE STRING "Benchmark host ELF data limit")
set(PERF_BENCHMARK_MAX_BSS "4096" CACHE STRING "Benchmark host ELF bss limit")
set(PERF_REPRESENTATIVE_MAX_TEXT "25000" CACHE STRING
    "Representative host ELF text limit")
set(PERF_REPRESENTATIVE_MAX_DATA "2048" CACHE STRING
    "Representative host ELF data limit")
set(PERF_REPRESENTATIVE_MAX_BSS "4096" CACHE STRING
    "Representative host ELF bss limit")

set(PERF_UTILS "${REPO_ROOT}/06_utils")
set(PERF_PRODUCTION_SOURCES
    "${PERF_UTILS}/util_ringbuf/util_ringbuf.c"
    "${PERF_UTILS}/util_fast_math/util_fast_math.c"
    "${PERF_UTILS}/util_lpf/util_lpf.c"
    "${PERF_UTILS}/util_pid/util_pid.c"
    "${PERF_UTILS}/util_kf/util_kf.c"
    "${PERF_UTILS}/util_ahrs/util_ahrs.c")

add_library(performance_production OBJECT ${PERF_PRODUCTION_SOURCES})
target_include_directories(performance_production PUBLIC
    "${PERF_UTILS}/util_ringbuf"
    "${PERF_UTILS}/util_fast_math"
    "${PERF_UTILS}/util_lpf"
    "${PERF_UTILS}/util_pid"
    "${PERF_UTILS}/util_kf"
    "${PERF_UTILS}/util_ahrs")
target_compile_features(performance_production PUBLIC c_std_11)
target_compile_options(performance_production PRIVATE
    -O2 -g0 -fstack-usage -ffunction-sections -fdata-sections -Wall -Wextra)

add_executable(performance_benchmark performance_benchmark.c)
target_link_libraries(performance_benchmark PRIVATE performance_production m)
target_compile_options(performance_benchmark PRIVATE
    -O2 -g0 -ffunction-sections -fdata-sections -Wall -Wextra)
target_link_options(performance_benchmark PRIVATE -Wl,--gc-sections)

add_executable(performance_resource_probe resource_probe.c)
target_link_libraries(performance_resource_probe PRIVATE performance_production m)
target_compile_options(performance_resource_probe PRIVATE
    -O2 -g0 -ffunction-sections -fdata-sections -Wall -Wextra)
target_link_options(performance_resource_probe PRIVATE -Wl,--gc-sections)

configure_file(performance_thresholds.json.in performance_thresholds.json @ONLY)
set(PERF_REPORT_DIR "${CMAKE_CURRENT_BINARY_DIR}/reports")
set(PERF_STACK_ROOT "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/performance_production.dir")
set(PERF_GATE_COMMAND
    "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/performance_gate.py"
    --benchmark "$<TARGET_FILE:performance_benchmark>"
    --representative "$<TARGET_FILE:performance_resource_probe>"
    --stack-root "${PERF_STACK_ROOT}"
    --size-tool "${PERF_GNU_SIZE}"
    --config "${CMAKE_CURRENT_BINARY_DIR}/performance_thresholds.json"
    --output-dir "${PERF_REPORT_DIR}")

add_custom_target(performance_report
    COMMAND ${PERF_GATE_COMMAND} --mode report
    DEPENDS performance_benchmark performance_resource_probe
    COMMENT "Reporting native-host performance and resource measurements"
    VERBATIM USES_TERMINAL)

add_custom_target(performance_check
    COMMAND ${PERF_GATE_COMMAND} --mode check
    DEPENDS performance_benchmark performance_resource_probe
    COMMENT "Enforcing native-host performance and resource thresholds"
    VERBATIM USES_TERMINAL)

add_test(NAME performance_report
    COMMAND ${PERF_GATE_COMMAND} --mode report)
set_tests_properties(performance_report PROPERTIES LABELS "performance;host" FIXTURES_SETUP perf_report)

add_test(NAME performance_check
    COMMAND ${PERF_GATE_COMMAND} --mode check)
set_tests_properties(performance_check PROPERTIES LABELS "performance;resource;host"
    FIXTURES_REQUIRED perf_report)

# Host tests use the native compiler and remain independent of the firmware build.
cmake_minimum_required(VERSION 3.22)
project(util_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CodTestPaths)
include(CodAddTest)
include(CodCMock)

file(GLOB UTIL_DIRS ${UTILS}/util_*)
file(GLOB DEVICE_ALL_DIRS ${DEVICES}/dev_*)
set(DEVICE_DIRS ${DEVICES}/dev_watchdog)

option(COVERAGE "Build with gcov instrumentation" OFF)
set(SANITIZER "" CACHE STRING "Host sanitizer: address, undefined, or empty")
set_property(CACHE SANITIZER PROPERTY STRINGS "" address undefined)
option(SANITIZERS "Deprecated: build with combined address and undefined sanitizers" OFF)

if(SANITIZERS)
    if(NOT SANITIZER STREQUAL "")
        message(FATAL_ERROR "Use either SANITIZER or deprecated SANITIZERS, not both")
    endif()
    message(DEPRECATION "SANITIZERS=ON is deprecated; use SANITIZER=address or undefined")
    set(SANITIZER "address,undefined")
endif()

if(NOT SANITIZER STREQUAL "")
    if(NOT SANITIZER STREQUAL "address" AND
       NOT SANITIZER STREQUAL "undefined" AND
       NOT SANITIZER STREQUAL "address,undefined")
        message(FATAL_ERROR "SANITIZER must be address, undefined, or empty")
    endif()
    if(COVERAGE)
        message(FATAL_ERROR "COVERAGE and SANITIZER builds must use separate build directories")
    endif()
    if(NOT CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        message(FATAL_ERROR "SANITIZER requires GCC or Clang")
    endif()

    add_compile_options(-fsanitize=${SANITIZER} -fno-omit-frame-pointer
                        -fno-sanitize-recover=all)
    add_link_options(-fsanitize=${SANITIZER} -fno-omit-frame-pointer
                     -fno-sanitize-recover=all)
    if(SANITIZER MATCHES "address")
        add_compile_options(-fsanitize-address-use-after-scope)
    endif()
endif()

# Warnings remain non-fatal so assertion diagnostics do not block a whole suite.
add_compile_options(-Wall -Wextra -g -O0)

if(COVERAGE)
    add_compile_options(--coverage)
    add_link_options(--coverage)
endif()

include_directories(
    ${TESTS_ROOT}
    ${TESTS_ROOT}/unity
    ${TESTS_ROOT}/stubs
    ${UTIL_DIRS}
    ${DEVICE_DIRS}
    ${PLATFORM}/rtos/mutex
    ${PLATFORM}/rtos/task
)

# Shared broad-suite support and production utilities.
add_library(testsupport STATIC
    ${TESTS_ROOT}/unity/unity.c
    ${TESTS_ROOT}/stubs/SEGGER_RTT_stub.c
    ${TESTS_ROOT}/stubs/plat_stub.c
)

set(UTIL_SOURCES
    ${UTILS}/util_ahrs/util_ahrs.c
    ${UTILS}/util_crc/util_crc.c
    ${UTILS}/util_fast_math/util_fast_math.c
    ${UTILS}/util_kf/util_kf.c
    ${UTILS}/util_log/util_log.c
    ${UTILS}/util_lpf/util_lpf.c
    ${UTILS}/util_maf/util_maf.c
    ${UTILS}/util_msgbus/util_msgbus.c
    ${UTILS}/util_pid/util_pid.c
    ${UTILS}/util_registry/util_registry.c
    ${UTILS}/util_ringbuf/util_ringbuf.c
    ${UTILS}/util_rls/util_rls.c
    ${UTILS}/util_seq/util_seq.c
    ${UTILS}/util_td/util_td.c
    ${UTILS}/util_traj_limit/util_traj_limit.c
)
add_library(utils STATIC ${UTIL_SOURCES})

enable_testing()

if(COVERAGE)
    if(NOT CMAKE_C_COMPILER_ID STREQUAL "GNU")
        message(FATAL_ERROR "COVERAGE requires GCC and its matching gcov executable")
    endif()
    if(CMAKE_C_COMPILER_VERSION VERSION_LESS 9)
        message(FATAL_ERROR "COVERAGE requires GCC 9 or newer for gcov JSON output")
    endif()

    find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter)
    set(GCOV_EXECUTABLE "" CACHE FILEPATH "gcov executable matching the selected GCC")
    if(NOT GCOV_EXECUTABLE)
        execute_process(
            COMMAND ${CMAKE_C_COMPILER} -print-prog-name=gcov
            RESULT_VARIABLE GCOV_LOOKUP_STATUS
            OUTPUT_VARIABLE GCOV_PROGRAM
            OUTPUT_STRIP_TRAILING_WHITESPACE)
        if(NOT GCOV_LOOKUP_STATUS EQUAL 0 OR GCOV_PROGRAM STREQUAL "")
            message(FATAL_ERROR "The selected GCC could not locate its gcov executable")
        endif()
        if(IS_ABSOLUTE "${GCOV_PROGRAM}")
            set(GCOV_EXECUTABLE "${GCOV_PROGRAM}" CACHE FILEPATH
                "gcov executable matching the selected GCC" FORCE)
        else()
            find_program(DISCOVERED_GCOV_EXECUTABLE NAMES "${GCOV_PROGRAM}" REQUIRED)
            set(GCOV_EXECUTABLE "${DISCOVERED_GCOV_EXECUTABLE}" CACHE FILEPATH
                "gcov executable matching the selected GCC" FORCE)
        endif()
    endif()

    execute_process(COMMAND ${GCOV_EXECUTABLE} --version RESULT_VARIABLE GCOV_VERSION_STATUS)
    if(NOT GCOV_VERSION_STATUS EQUAL 0)
        message(FATAL_ERROR "GCOV_EXECUTABLE is not runnable: ${GCOV_EXECUTABLE}")
    endif()

    set(COVERAGE_MIN_LINES "89" CACHE STRING "Minimum production line coverage percentage")
    set(COVERAGE_MIN_FUNCTIONS "92" CACHE STRING "Minimum production function coverage percentage")
    set(COVERAGE_MIN_BRANCHES "76" CACHE STRING "Minimum production branch coverage percentage")

    add_custom_target(coverage
        COMMAND ${Python3_EXECUTABLE} -B ${TESTS_ROOT}/gates/coverage/coverage.py
                --root ${REPO_ROOT}
                --build-dir ${CMAKE_BINARY_DIR}
                --output-dir ${CMAKE_BINARY_DIR}/coverage
                --gcov ${GCOV_EXECUTABLE}
                --min-lines ${COVERAGE_MIN_LINES}
                --min-functions ${COVERAGE_MIN_FUNCTIONS}
                --min-branches ${COVERAGE_MIN_BRANCHES}
        WORKING_DIRECTORY ${REPO_ROOT}
        COMMENT "Merging production gcov data and enforcing coverage thresholds"
        VERBATIM
        USES_TERMINAL)
endif()

option(ENABLE_QUALITY_GATES "Register static and architecture quality gates" ON)
option(ENABLE_PROPERTY_TESTS "Register deterministic property tests" ON)
option(ENABLE_PERFORMANCE_GATES "Register native-host performance/resource gates" ON)
option(ENABLE_FUZZING "Build Clang libFuzzer targets" OFF)

add_subdirectory(unit/utils)
add_subdirectory(unit/platform/bsp)
add_subdirectory(unit/platform/rtos)
add_subdirectory(unit/device)
add_subdirectory(unit/application)
add_subdirectory(impl/bsp/stm32h7)
add_subdirectory(impl/bsp/stm32f4)
add_subdirectory(impl/rtos/freertos)
add_subdirectory(integration)

if(ENABLE_QUALITY_GATES)
    add_subdirectory(gates/quality)
endif()
if(ENABLE_PROPERTY_TESTS)
    add_subdirectory(property)
endif()
if(ENABLE_PERFORMANCE_GATES AND CMAKE_C_COMPILER_ID STREQUAL "GNU" AND
   SANITIZER STREQUAL "" AND NOT COVERAGE)
    add_subdirectory(gates/performance)
elseif(ENABLE_PERFORMANCE_GATES)
    message(STATUS "Skipping native performance gates for this instrumented/non-GNU build")
endif()
if(ENABLE_FUZZING)
    add_subdirectory(fuzz EXCLUDE_FROM_ALL)
endif()

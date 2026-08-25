# Clang/libFuzzer targets. The targets are intentionally not CTest tests because
# campaign duration and corpus ownership belong to tests/run_fuzz.sh.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.22)
    project(fuzz_tests C)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)
    get_filename_component(TESTS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
    get_filename_component(REPO_ROOT "${TESTS_ROOT}/.." ABSOLUTE)
    set(UTILS "${REPO_ROOT}/06_utils")
    set(DEVICES "${REPO_ROOT}/02_device")
    set(PLATFORM "${REPO_ROOT}/03_platform")
    set(IMPL "${REPO_ROOT}/04_impl")
    set(IMPL_COMMON "${IMPL}/common")
endif()

if(NOT CMAKE_C_COMPILER_ID STREQUAL "Clang" OR CMAKE_C_COMPILER_VERSION VERSION_LESS 18)
    message(FATAL_ERROR "Fuzz targets require Clang 18 or newer")
endif()

function(fuzz_target target source)
    add_executable(${target} ${source} ${ARGN})
    target_compile_options(${target} PRIVATE
        -Wall -Wextra -Wpedantic -UNDEBUG -fno-omit-frame-pointer
        -fsanitize=fuzzer,address,undefined)
    target_link_options(${target} PRIVATE
        -fno-omit-frame-pointer -fsanitize=fuzzer,address,undefined)
endfunction()

fuzz_target(fuzz_ringbuf
    fuzz_ringbuf.c
    ${UTILS}/util_ringbuf/util_ringbuf.c)
target_include_directories(fuzz_ringbuf PRIVATE ${UTILS}/util_ringbuf)

fuzz_target(fuzz_crc
    fuzz_crc.c
    ${UTILS}/util_crc/util_crc.c)
target_include_directories(fuzz_crc PRIVATE ${UTILS}/util_crc)

fuzz_target(fuzz_remote
    fuzz_remote.c
    ${TESTS_ROOT}/property/remote_test_stub.c
    ${DEVICES}/dev_remote/dev_remote.c)
target_include_directories(fuzz_remote PRIVATE
    ${TESTS_ROOT}/property
    ${DEVICES}/dev_remote
    ${PLATFORM}/bsp/uart
    ${PLATFORM}/bsp/dma_buf
    ${PLATFORM}/rtos/memory
    ${UTILS}/util_ringbuf
    ${IMPL_COMMON})

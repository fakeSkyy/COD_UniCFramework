# Deterministic property tests. This file works both as a parent-included child
# and as a standalone project for local validation.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.22)
    project(property_tests C)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)
    get_filename_component(TESTS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
    get_filename_component(REPO_ROOT "${TESTS_ROOT}/.." ABSOLUTE)
    set(UTILS "${REPO_ROOT}/06_utils")
    set(DEVICES "${REPO_ROOT}/02_device")
    set(PLATFORM "${REPO_ROOT}/03_platform")
    set(IMPL "${REPO_ROOT}/04_impl")
    set(IMPL_COMMON "${IMPL}/common")
    enable_testing()
endif()

function(property_test target source)
    add_executable(${target} ${source} ${ARGN})
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME ${target} COMMAND ${target})
    set_tests_properties(${target} PROPERTIES LABELS "property;host;deterministic")
endfunction()

property_test(property_ringbuf
    property_ringbuf.c
    ${UTILS}/util_ringbuf/util_ringbuf.c)
target_include_directories(property_ringbuf PRIVATE ${UTILS}/util_ringbuf)

property_test(property_crc
    property_crc.c
    ${UTILS}/util_crc/util_crc.c)
target_include_directories(property_crc PRIVATE ${UTILS}/util_crc)

property_test(property_remote
    property_remote.c
    remote_test_stub.c
    ${DEVICES}/dev_remote/dev_remote.c)
target_include_directories(property_remote PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${DEVICES}/dev_remote
    ${PLATFORM}/bsp/uart
    ${PLATFORM}/bsp/dma_buf
    ${PLATFORM}/rtos/memory
    ${UTILS}/util_ringbuf
    ${IMPL_COMMON})

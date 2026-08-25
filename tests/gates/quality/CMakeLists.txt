cmake_minimum_required(VERSION 3.22)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(quality_gate NONE)
    enable_testing()
endif()

find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter)

get_filename_component(QUALITY_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(QUALITY_REPORT "${CMAKE_BINARY_DIR}/quality-report.json")
set(QUALITY_COMPILE_DB_ARGS)
if(NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    list(APPEND QUALITY_COMPILE_DB_ARGS
        --compile-db "${CMAKE_BINARY_DIR}/compile_commands.json")
endif()

add_custom_target(quality
    COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/quality_gate.py"
            --root "${QUALITY_REPO_ROOT}"
            --json-out "${QUALITY_REPORT}"
            ${QUALITY_COMPILE_DB_ARGS}
    WORKING_DIRECTORY "${QUALITY_REPO_ROOT}"
    COMMENT "Running architecture and static-analysis quality gates"
    VERBATIM
    USES_TERMINAL)

add_test(
    NAME quality_gate
    COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/quality_gate.py"
            --root "${QUALITY_REPO_ROOT}"
            --json-out "${QUALITY_REPORT}"
            ${QUALITY_COMPILE_DB_ARGS})
set_tests_properties(quality_gate PROPERTIES
    WORKING_DIRECTORY "${QUALITY_REPO_ROOT}"
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1"
    LABELS "quality;static;architecture")

add_test(
    NAME quality_selftest
    COMMAND "${Python3_EXECUTABLE}" -B -m unittest -v test_quality_gate.py)
set_tests_properties(quality_selftest PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1"
    LABELS "quality;selftest")

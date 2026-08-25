# Device suites compile one production module with official generated CMock code.
set(DEVICE_TEST ${CMAKE_CURRENT_SOURCE_DIR})
set(IMPL_COMMON_DEVICE ${IMPL_COMMON})

add_library(device_cmock_support STATIC)
cod_add_cmock_runtime(
    TARGET device_cmock_support
    SOURCES
        ${TEST_ALLOC_SUPPORT}/host_alloc_tracker.c
        device_cmock_order.c
        mocks/mock_device_platform.c
    INCLUDES
        ${DEVICE_TEST}
        ${DEVICE_TEST}/contracts
        ${DEVICE_TEST}/mocks
        ${DEVICE_ALL_DIRS}
        ${UTIL_DIRS}
        ${IMPL_COMMON_DEVICE}
        ${PLATFORM}/rtos/memory
        ${PLATFORM}/bsp/can
        ${PLATFORM}/bsp/dma_buf
        ${PLATFORM}/bsp/dwt
        ${PLATFORM}/bsp/flash
        ${PLATFORM}/bsp/pwm
        ${PLATFORM}/bsp/spi
        ${PLATFORM}/bsp/uart
    DEFINITIONS
    OPTIONS "-iquote${DEVICE_TEST}/contracts")

function(add_device_test module)
    set(name test_dev_${module})
    add_executable(${name}
        suites/${name}.c
        ${DEVICES}/dev_${module}/dev_${module}.c
        ${ARGN}
    )
    target_link_libraries(${name} PRIVATE device_cmock_support m)
    cod_add_test(TARGET ${name} CASES LABELS device unit cmock)
endfunction()

add_device_test(bmi088
    ${DEVICES}/dev_watchdog/dev_watchdog.c)
add_executable(test_dev_bmi088_store
    suites/test_dev_bmi088_store.c
    ${DEVICES}/dev_bmi088/dev_bmi088_store.c
    ${UTILS}/util_crc/util_crc.c
    mocks/mock_bmi088_peer.c
)
target_link_libraries(test_dev_bmi088_store PRIVATE device_cmock_support m)
cod_add_test(TARGET test_dev_bmi088_store CASES LABELS device unit cmock)
add_device_test(buzzer
    ${UTILS}/util_seq/util_seq.c)
add_device_test(dji_motor
    ${DEVICES}/dev_watchdog/dev_watchdog.c
    ${UTILS}/util_fast_math/util_fast_math.c)
add_device_test(dm_motor
    ${DEVICES}/dev_watchdog/dev_watchdog.c)
add_device_test(motor_pid
    ${UTILS}/util_pid/util_pid.c
    ${UTILS}/util_lpf/util_lpf.c)
add_device_test(power_limit
    ${UTILS}/util_rls/util_rls.c)
add_device_test(remote)
add_device_test(steer_chassis)
add_device_test(ws2812
    ${DEVICES}/dev_watchdog/dev_watchdog.c)

add_executable(test_dev_watchdog
    suites/test_dev_watchdog.c
    ${DEVICES}/dev_watchdog/dev_watchdog.c
)
target_link_libraries(test_dev_watchdog PRIVATE utils testsupport m)
cod_add_test(TARGET test_dev_watchdog CASES LABELS device unit)

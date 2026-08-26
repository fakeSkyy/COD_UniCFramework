set(INTEGRATION ${CMAKE_CURRENT_SOURCE_DIR})
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(APP ${ROOT}/01_application)
set(DEV ${ROOT}/02_device)
set(PLAT ${ROOT}/03_platform)
set(IMPL ${ROOT}/04_impl)
set(UTIL ${ROOT}/06_utils)

set(INTEGRATION_COMMON_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../unity
    ${CMAKE_CURRENT_SOURCE_DIR}/../cmock/src
    ${IMPL}/common
    ${APP}/board ${APP}/telemetry ${APP}/indicator ${APP}/imu
    ${DEV}/dev_remote ${DEV}/dev_ws2812 ${DEV}/dev_watchdog ${DEV}/dev_buzzer
    ${DEV}/dev_bmi088 ${DEV}/dev_dji_motor ${DEV}/dev_dm_motor
    ${PLAT}/bsp/uart ${PLAT}/bsp/spi ${PLAT}/bsp/can ${PLAT}/bsp/dwt
    ${PLAT}/bsp/pwm ${PLAT}/bsp/flash ${PLAT}/bsp/dma_buf
    ${PLAT}/rtos/memory ${PLAT}/rtos/mutex ${PLAT}/rtos/task
    ${UTIL}/util_ringbuf ${UTIL}/util_seq ${UTIL}/util_fast_math
    ${UTIL}/util_ahrs ${UTIL}/util_kf ${UTIL}/util_msgbus)

function(integration_mock_support target area mock_source)
    target_sources(${target} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../unity/unity.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../cmock/src/cmock.c
        ${TEST_ALLOC_SUPPORT}/host_alloc_tracker.c
        ${INTEGRATION}/integration_cmock_order.c
        ${INTEGRATION}/${area}/mocks/${mock_source})
    target_include_directories(${target} PRIVATE ${INTEGRATION_COMMON_INCLUDES})
    target_include_directories(${target} BEFORE PRIVATE
        ${INTEGRATION}/${area}/contracts ${INTEGRATION}/${area}/mocks)
    target_link_libraries(${target} PRIVATE m)
endfunction()

function(integration_test target)
    add_test(NAME ${target} COMMAND ${target})
    set_tests_properties(${target} PROPERTIES LABELS "integration;host;cmock")
endfunction()

add_executable(integration_telemetry_uart
    telemetry_uart/test_telemetry_uart.c
    ${INTEGRATION}/support/host_allocator.c
    ${APP}/telemetry/app_telemetry.c
    ${PLAT}/bsp/uart/plat_uart.c
    ${UTIL}/util_ringbuf/util_ringbuf.c
    ${UTIL}/util_fast_math/util_fast_math.c)
integration_mock_support(integration_telemetry_uart telemetry_uart mock_telemetry_contract.c)
integration_test(integration_telemetry_uart)

add_executable(integration_remote_uart
    remote_uart/test_remote_uart.c
    ${DEV}/dev_remote/dev_remote.c
    ${PLAT}/bsp/uart/plat_uart.c
    ${UTIL}/util_ringbuf/util_ringbuf.c)
integration_mock_support(integration_remote_uart remote_uart mock_remote_contract.c)
integration_test(integration_remote_uart)

add_executable(integration_indicator_ws2812_spi
    indicator_ws2812_spi/test_indicator_ws2812_spi.c
    ${INTEGRATION}/support/host_allocator.c
    ${APP}/indicator/app_indicator.c
    ${DEV}/dev_ws2812/dev_ws2812.c
    ${DEV}/dev_buzzer/dev_buzzer.c
    ${DEV}/dev_watchdog/dev_watchdog.c
    ${PLAT}/bsp/spi/plat_spi.c
    ${PLAT}/bsp/pwm/plat_pwm.c
    ${UTIL}/util_seq/util_seq.c)
integration_mock_support(integration_indicator_ws2812_spi indicator_ws2812_spi mock_indicator_contract.c)
foreach(case_name spi_callback_trampolines heartbeat_wire fault_priority_and_send_failure)
    add_test(NAME integration_indicator_ws2812_spi.${case_name}
        COMMAND integration_indicator_ws2812_spi ${case_name})
    set_tests_properties(integration_indicator_ws2812_spi.${case_name}
        PROPERTIES LABELS "integration;host;cmock")
endforeach()

add_executable(integration_imu_bmi088_spi_dwt
    imu_bmi088_spi_dwt/test_imu_bmi088_spi_dwt.c
    ${INTEGRATION}/support/host_allocator.c
    ${APP}/imu/app_imu.c
    ${APP}/telemetry/app_telemetry.c
    ${DEV}/dev_bmi088/dev_bmi088.c
    ${DEV}/dev_watchdog/dev_watchdog.c
    ${PLAT}/bsp/spi/plat_spi.c
    ${PLAT}/bsp/dwt/plat_dwt.c
    ${PLAT}/bsp/uart/plat_uart.c
    ${UTIL}/util_ahrs/util_ahrs.c
    ${UTIL}/util_kf/util_kf.c
    ${UTIL}/util_fast_math/util_fast_math.c
    ${UTIL}/util_ringbuf/util_ringbuf.c
    ${UTIL}/util_pid/util_pid.c
    ${UTIL}/util_lpf/util_lpf.c)
integration_mock_support(integration_imu_bmi088_spi_dwt imu_bmi088_spi_dwt mock_imu_contract.c)
foreach(case_name sample_to_ahrs_to_telemetry bottom_failure_suspends_offline)
    add_test(NAME integration_imu_bmi088_spi_dwt.${case_name}
        COMMAND integration_imu_bmi088_spi_dwt ${case_name})
    set_tests_properties(integration_imu_bmi088_spi_dwt.${case_name}
        PROPERTIES LABELS "integration;host;cmock")
endforeach()

add_executable(integration_motor_can_dji
    motor_can_dji/test_motor_can_dji.c
    ${DEV}/dev_dji_motor/dev_dji_motor.c
    ${DEV}/dev_watchdog/dev_watchdog.c
    ${PLAT}/bsp/can/plat_can.c
    ${UTIL}/util_fast_math/util_fast_math.c)
integration_mock_support(integration_motor_can_dji motor_can_dji mock_motor_dji_contract.c)
integration_test(integration_motor_can_dji)

add_executable(integration_motor_can_dm
    motor_can_dm/test_motor_can_dm.c
    ${DEV}/dev_dm_motor/dev_dm_motor.c
    ${DEV}/dev_watchdog/dev_watchdog.c
    ${PLAT}/bsp/can/plat_can.c
    ${UTIL}/util_fast_math/util_fast_math.c)
integration_mock_support(integration_motor_can_dm motor_can_dm mock_motor_dm_contract.c)
integration_test(integration_motor_can_dm)

add_executable(integration_msgbus_rtos
    msgbus_rtos/test_msgbus_rtos.c
    ${UTIL}/util_msgbus/util_msgbus.c
    ${PLAT}/rtos/mutex/plat_mutex.c
    ${PLAT}/rtos/task/plat_task.c
    ${IMPL}/rtos/freertos/mutex/impl_mutex.c
    ${IMPL}/rtos/freertos/task/impl_task.c)
target_sources(integration_msgbus_rtos PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../unity/unity.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../cmock/src/cmock.c
    ${INTEGRATION}/integration_cmock_order.c
    msgbus_rtos/mocks/mock_FreeRTOS.c
    msgbus_rtos/mocks/mock_semphr.c
    msgbus_rtos/mocks/mock_task.c)
target_include_directories(integration_msgbus_rtos PRIVATE ${INTEGRATION_COMMON_INCLUDES}
    ${IMPL}/rtos/freertos ${IMPL}/rtos/freertos/mutex ${IMPL}/rtos/freertos/task)
target_include_directories(integration_msgbus_rtos BEFORE PRIVATE
    msgbus_rtos/contracts msgbus_rtos/mocks)
target_link_libraries(integration_msgbus_rtos PRIVATE m)
integration_test(integration_msgbus_rtos)

add_executable(integration_board_platform
    board_platform/test_board_platform.c
    board_platform/host/host_vendor.c
    ${APP}/board/board_devices.c
    ${PLAT}/bsp/dwt/plat_dwt.c
    ${PLAT}/bsp/spi/plat_spi.c
    ${PLAT}/bsp/uart/plat_uart.c
    ${PLAT}/bsp/pwm/plat_pwm.c
    ${PLAT}/bsp/flash/plat_flash.c
    ${PLAT}/bsp/can/plat_can.c
    ${UTIL}/util_ringbuf/util_ringbuf.c)
integration_mock_support(integration_board_platform board_platform mock_board_backend_contract.c)
target_include_directories(integration_board_platform BEFORE PRIVATE
    board_platform/host ${TESTS_ROOT}/unit/application/host)
integration_test(integration_board_platform)

# Application translation units use target-local aggregate mocks; cases run in
# separate processes to isolate production file-static state.
set(APPLICATION_TEST ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB APPLICATION_DIRS ${APPLICATION}/*)

add_library(application_test_support STATIC)
cod_add_cmock_runtime(
    TARGET application_test_support
    SOURCES application_cmock_order.c
    INCLUDES
        ${APPLICATION_TEST}
        ${APPLICATION_TEST}/contracts
        ${APPLICATION_TEST}/mocks
        ${APPLICATION_DIRS}
        ${DEVICE_ALL_DIRS}
        ${UTIL_DIRS}
        ${IMPL_COMMON}
        ${PLATFORM}/rtos/task
        ${PLATFORM}/bsp/can
        ${PLATFORM}/bsp/dma_buf
        ${PLATFORM}/bsp/dwt
        ${PLATFORM}/bsp/flash
        ${PLATFORM}/bsp/pwm
        ${PLATFORM}/bsp/spi
        ${PLATFORM}/bsp/uart
    DEFINITIONS
    OPTIONS "-iquote${APPLICATION_TEST}/contracts")

function(add_application_test target production_source mock_source)
    add_executable(${target}
        suites/${target}.c
        ${production_source}
        mocks/${mock_source}
    )
    target_link_libraries(${target} PRIVATE application_test_support m)

    cod_add_test(TARGET ${target} CASES ${ARGN} LABELS application unit cmock)
endfunction()

add_application_test(test_app_tasks
    ${APPLICATION}/tasks/app_tasks.c
    mock_tasks_deps.c
    indicator_failure_short_circuits
    health_failure_short_circuits
    imu_failure_short_circuits
    scheduler_failure_after_ordered_creation
    scheduler_true_defensive_return
    chassis_failure_is_not_fatal)

add_application_test(test_app_telemetry
    ${APPLICATION}/telemetry/app_telemetry.c
    mock_telemetry_deps.c
    step_before_init_is_noop
    init_rejects_missing_uart
    fifth_call_emits_golden_justfloat_frame
    null_rates_encode_zero
    nonfinite_channels_are_replaced
    in_flight_skips_until_completion
    inline_completion_does_not_stick
    send_failure_self_clears_and_retries
    reinit_resets_busy_divider_and_skipped)

add_application_test(test_app_indicator
    ${APPLICATION}/indicator/app_indicator.c
    mock_indicator_deps.c
    start_task_forwards_parameters_and_result
    invalid_heartbeat_idempotent_and_priority
    fault_zero_clears
    heartbeat_init_watchdog_frames_tick_and_show
    led_init_failure_is_nonfatal_and_dark
    device_condition_builds_blue_five_flash_pattern
    fault_code_clamps_to_nine_red_flashes
    steady_condition_does_not_restart_sequence
    output_is_narrowed_to_pixel_bytes
    buzzer_ticked_once_per_loop_iteration
    null_buzzer_leaves_led_working
    setfault_sounds_alert_once_with_divisible_frames)

add_application_test(test_app_health
    ${APPLICATION}/health/app_health.c
    mock_health_deps.c
    start_task_forwards_parameters_and_result
    start_task_propagates_failure
    report_counts_and_logs_each_node
    healthy_level_and_twenty_ms_period
    failure_persistence_logs_only_first_edge
    recovery_logs_edge_and_clears_level
    null_failed_name_uses_safe_failure_path)

add_application_test(test_app_chassis
    ${APPLICATION}/chassis/app_chassis.c
    mock_chassis_deps.c
    start_task_creates_two_nodes_and_the_task
    transmit_node_starts_last
    missing_timebase_creates_no_nodes
    refused_transmit_node_does_not_attach_motors
    refused_receive_node_stops_bring_up
    a_node_that_cannot_start_fails_bring_up
    feedback_routes_to_the_motor_owning_the_identifier
    unknown_identifier_is_dropped
    non_consecutive_feedback_ids_are_refused
    online_is_false_before_bring_up
    start_task_propagates_task_creation_failure)

add_application_test(test_app_imu
    ${APPLICATION}/imu/app_imu.c
    mock_imu_deps.c
    start_task_forwards_parameters
    accessors_before_ready_are_safe
    all_bmi_failure_statuses_do_not_create_task_and_raise_fault
    ahrs_failure_does_not_create_task_and_raises_fault
    accessors_safe_after_failed_start_task
    success_config_warnings_sample_overrun_and_accessors
    initial_read_failure_skips_alignment_but_runs
    ninety_nine_failures_do_not_raise_fault
    hundred_failures_report_once_and_recovery_clears
    persistent_failure_after_threshold_faults_only_once
    null_heater_pwm_leaves_attitude_running_and_commands_nothing
    heater_duty_rises_below_setpoint_and_is_floored_above_it
    heater_output_never_exceeds_configured_cap
    heater_out_of_range_temp_commands_zero_duty
    heater_over_temperature_cuts_output
    heater_below_cutoff_still_regulates
    imu_outage_stops_heater
    heater_steps_at_refresh_rate_not_every_loop_iteration)
# UTIL_PID_Init/Step run for real here (app_imu.c includes util_pid.h directly,
# not through imu_deps.h), so the tests exercise actual PID math on the duty
# output rather than a mocked stand-in. util_lpf and util_fast_math are pulled
# in because util_pid.c calls into both for its derivative/output filters.
target_sources(test_app_imu PRIVATE
    ${UTILS}/util_pid/util_pid.c
    ${UTILS}/util_lpf/util_lpf.c
    ${UTILS}/util_fast_math/util_fast_math.c)

add_application_test(test_board_devices
    ${APPLICATION}/board/board_devices.c
    mock_board_deps.c
    accessors_are_null_before_first_init
    eight_devices_initialize_in_strict_table_order
    each_null_context_is_first_failure_and_short_circuits
    each_platform_init_failure_is_first_and_short_circuits
    reinit_clears_failure_and_rebuilds_all_accessor_state
    can_rejects_negative_and_count_without_backend_calls
    can1_and_can2_forward_handle_and_ids
    can_null_context_short_circuits_before_ops_and_create
    can_create_failure_is_returned
    reinit_destroys_each_previous_context_exactly_once
    midway_failure_leaves_contexts_that_the_next_call_still_releases
    third_consecutive_init_frees_nothing_twice)
target_sources(test_board_devices PRIVATE host/host_vendor.c)
target_include_directories(test_board_devices BEFORE PRIVATE ${APPLICATION_TEST}/host)

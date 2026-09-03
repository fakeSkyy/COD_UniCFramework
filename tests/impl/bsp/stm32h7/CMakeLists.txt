# STM32H7 production backends run against the host HAL/CMock contract.
set(STM32H7_IMPL ${IMPL}/bsp/stm32h7)
set(STM32H7_TEST ${CMAKE_CURRENT_SOURCE_DIR})

add_library(stm32h7_test_support STATIC)
cod_add_cmock_runtime(
    TARGET stm32h7_test_support
    SOURCES
        hal/host_hal.c
        mocks/mock_stm32h7xx_hal.c
        mocks/mock_impl_memory.c
        ${UTILS}/util_registry/util_registry.c
    INCLUDES
        ${STM32H7_TEST}
        ${STM32H7_TEST}/hal
        ${STM32H7_TEST}/mocks
        ${IMPL_COMMON}
        ${UTILS}/util_registry
        ${STM32H7_IMPL}/adc
        ${STM32H7_IMPL}/can
        ${STM32H7_IMPL}/dwt
        ${STM32H7_IMPL}/gpio
        ${STM32H7_IMPL}/iic
        ${STM32H7_IMPL}/pwm
        ${STM32H7_IMPL}/spi
        ${STM32H7_IMPL}/flash
        ${STM32H7_IMPL}/uart
    DEFINITIONS _GNU_SOURCE
    OPTIONS)

function(add_stm32h7_impl_test module)
    set(name test_impl_stm32_${module})
    add_executable(${name}
        suites/${name}.c
        ${STM32H7_IMPL}/${module}/impl_stm32_${module}.c
    )
    target_link_libraries(${name} PRIVATE stm32h7_test_support)
    target_compile_options(${name} PRIVATE -Wno-int-to-pointer-cast)
    cod_add_test(TARGET ${name} CASES ${ARGN} LABELS stm32h7 unit cmock)
endfunction()

add_stm32h7_impl_test(adc)
add_stm32h7_impl_test(can
    test_create_guards_allocator_duplicate_and_capacity
    test_start_rolls_back_failures_retries_and_adds_live_filter
    test_send_requires_started_bus_and_builds_classic_frame
    test_receive_routes_fifo0_fifo1_lost_and_malformed_frames
    test_destroy_stops_routing_and_frees_the_identifier
    test_range_claim_installs_one_element_and_routes_the_span
    test_overlapping_claims_are_refused
    test_single_identifier_outside_a_range_still_gets_a_filter
    test_inverted_range_is_refused
    test_destroying_a_range_node_stops_the_whole_span
    test_status_and_protocol_errors_map_broadcast_and_recover
    test_bus_and_filter_capacity_follow_hardware_limits
    test_null_callbacks_ignore_frames_and_recovered_status)
add_stm32h7_impl_test(dwt)
add_stm32h7_impl_test(flash)
add_stm32h7_impl_test(gpio)
add_stm32h7_impl_test(iic
    test_create_guards_allocator_address_and_fields
    test_blocking_calls_guard_shift_address_and_release_bus
    test_async_it_dma_arbitrate_callback_and_rollback
    test_memory_async_modes_complete_and_recover
    test_sequence_validates_steps_maps_frames_and_final_direction
    test_sequence_start_and_middle_failures_release_bus
    test_error_maps_all_flags_and_allows_retry
    test_bus_capacity_rejects_fifth_bus
    test_masked_arbitration_dma_tx_rollback_and_late_callbacks
    test_sequence_middle_failure_without_error_callback_releases_bus)
add_stm32h7_impl_test(pwm)
add_stm32h7_impl_test(spi
    test_create_guards_allocator_csless_and_bus_capacity
    test_blocking_transfers_guard_and_release_after_hal_failure
    test_async_it_arbitrates_and_rolls_back_failed_start
    test_cs_hold_is_exclusive_idempotent_and_spans_transfers
    test_async_callbacks_preserve_receive_buffer_and_length
    test_dma_reachability_cache_and_failed_start_rollback
    test_error_maps_all_flags_forces_cs_high_and_releases_bus
    test_dma_boundaries_masked_claim_and_null_callbacks
    test_failed_async_start_inside_cs_hold_preserves_owner)
add_stm32h7_impl_test(uart
    test_create_guards_duplicate_allocator_and_capacity
    test_blocking_send_receive_and_non_timeout_failure
    test_async_it_dma_reachability_cache_and_tx_callback
    test_normal_receive_rearms_reports_failure_and_gates_late_event
    test_circular_dma_delivers_only_new_ranges_and_wrap
    test_error_maps_flags_and_rearms_only_stopped_receive
    test_dma_boundaries_missing_stream_and_null_callbacks
    test_circular_null_callback_delivers_tail_only_on_zero_wrap)

# Registered-callback variants are separate processes and compile production sources directly.
function(add_stm32h7_register_test module callback_macro)
    set(name test_impl_stm32_${module}_register)
    add_executable(${name}
        suites/${name}.c
        ${STM32H7_IMPL}/${module}/impl_stm32_${module}.c
    )
    target_link_libraries(${name} PRIVATE stm32h7_test_support)
    target_compile_definitions(${name} PRIVATE ${callback_macro}=1U)
    target_compile_options(${name} PRIVATE -Wno-int-to-pointer-cast)
    cod_add_test(TARGET ${name} CASES ${ARGN} LABELS stm32h7 register cmock)
endfunction()

add_stm32h7_register_test(can USE_HAL_FDCAN_REGISTER_CALLBACKS
    test_registered_error_callback_routes_to_context
    test_partial_registration_failure_does_not_publish_failed_route
    test_registered_bus_capacity_rejects_fourth_handle)
add_stm32h7_register_test(spi USE_HAL_SPI_REGISTER_CALLBACKS
    test_registered_tx_thunk_routes_to_context
    test_partial_registration_failure_frees_and_old_thunk_cannot_route
    test_duplicate_handle_reuses_registration)
add_stm32h7_register_test(uart USE_HAL_UART_REGISTER_CALLBACKS
    test_registered_tx_thunk_routes_to_context
    test_partial_registration_failure_frees_and_old_thunk_cannot_route
    test_duplicate_handle_is_rejected_without_rebinding)
add_stm32h7_register_test(iic USE_HAL_I2C_REGISTER_CALLBACKS
    test_registered_tx_thunk_routes_to_active_context
    test_partial_registration_failure_old_thunk_cannot_route_reused_slot
    test_shared_handle_registers_once_and_routes_current_owner)

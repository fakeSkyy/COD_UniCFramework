# STM32F4 backends use an independent host HAL contract and the real registry.
set(STM32F4_IMPL ${IMPL}/bsp/stm32f4)
set(STM32F4_TEST ${CMAKE_CURRENT_SOURCE_DIR})

add_library(stm32f4_test_support STATIC)
cod_add_cmock_runtime(
    TARGET stm32f4_test_support
    SOURCES
        stm32f4_cmock_order.c
        hal/host_hal.c
        mocks/mock_stm32f4xx_hal.c
        mocks/mock_impl_memory.c
        ${UTILS}/util_registry/util_registry.c
    INCLUDES
        ${STM32F4_TEST}
        ${STM32F4_TEST}/hal
        ${STM32F4_TEST}/mocks
        ${IMPL_COMMON}
        ${UTILS}/util_registry
        ${STM32F4_IMPL}/adc
        ${STM32F4_IMPL}/can
        ${STM32F4_IMPL}/dwt
        ${STM32F4_IMPL}/flash
        ${STM32F4_IMPL}/gpio
        ${STM32F4_IMPL}/iic
        ${STM32F4_IMPL}/pwm
        ${STM32F4_IMPL}/spi
        ${STM32F4_IMPL}/uart
    DEFINITIONS _GNU_SOURCE
    OPTIONS)

function(add_stm32f4_impl_test module)
    set(name test_impl_stm32f4_${module})
    add_executable(${name}
        suites/${name}.c
        ${STM32F4_IMPL}/${module}/impl_stm32_${module}.c
    )
    target_link_libraries(${name} PRIVATE stm32f4_test_support)
    target_compile_options(${name} PRIVATE -Wno-int-to-pointer-cast)

    cod_add_test(TARGET ${name} CASES ${ARGN} LABELS stm32f4 unit cmock)
endfunction()

add_stm32f4_impl_test(adc
    test_get_ops_and_create_validate_allocator
    test_blocking_read_config_start_poll_stop_and_preserve_output_on_failure
    test_interrupt_start_rollback_callback_and_late_event_gate
    test_dma_guards_start_rollback_half_full_stop_and_late_callbacks
    test_route_capacity_failure_frees_allocated_context)
add_stm32f4_impl_test(can
    test_get_ops_create_guards_allocator_and_duplicate_id
    test_filter_start_notification_rollback_and_retry_idempotent
    test_filter_bank_packing_uses_second_bank_fifo1
    test_can2_filter_allocator_starts_at_bank14
    test_filter_hal_failure_rolls_back_allocator_and_retry_succeeds
    test_hal_start_failure_can_retry_without_consuming_filter_slot
    test_route_capacity_failure_frees_allocated_context
    test_range_claim_routes_every_identifier_in_the_span
    test_range_overlap_and_inversion_are_refused
    test_destroying_a_range_node_retires_the_whole_span
    test_standard_tx_golden_header_zero_fill_send_to_and_free_level
    test_fifo0_and_fifo1_route_short_payload_and_ignore_unknown
    test_error_maps_all_families_resets_hal_and_routes_to_node)
add_stm32f4_impl_test(dwt)
add_stm32f4_impl_test(flash
    test_get_ops_create_geometry_and_invalid_ranges
    test_read_and_is_erased_enforce_bounds_and_copy_mapped_bytes
    test_write_success_unlocks_programs_locks_flushes_and_verifies
    test_write_reports_mid_program_failure_and_still_locks
    test_write_reports_post_program_verify_failure
    test_erase_success_uses_f407_parameters_and_verifies_window
    test_erase_reports_hal_failure_failed_sector_and_non_ff_verify)
add_stm32f4_impl_test(gpio)
add_stm32f4_impl_test(iic
    test_get_ops_create_null_alloc_failure_and_fields
    test_blocking_memory_master_and_readiness_shift_address_and_release_bus
    test_async_it_dma_selection_bus_arbitration_callbacks_and_rollback
    test_sequence_advances_frame_options_and_reports_final_direction
    test_memory_async_it_and_dma_complete_through_memory_callbacks
    test_memory_start_failure_rolls_back_and_releases_bus
    test_sequence_first_and_middle_hal_failures_rollback_and_release_bus
    test_dma_sequence_uses_dma_steps_and_releases_after_final_callback
    test_error_maps_flags_releases_bus_and_allows_retry)
add_stm32f4_impl_test(pwm)
add_stm32f4_impl_test(spi
    test_get_ops_create_guards_allocator_and_initial_cs_high
    test_blocking_send_receive_txrx_select_and_release_cs
    test_async_it_bus_busy_tx_callback_and_start_failure_rollback
    test_cs_hold_is_idempotent_exclusive_and_spans_multiple_transfers
    test_async_receive_and_txrx_callbacks_preserve_buffer_and_length
    test_error_maps_flags_forces_cs_high_and_releases_bus)
add_stm32f4_impl_test(uart
    test_get_ops_create_null_alloc_failure_and_route_rollback
    test_blocking_send_read_success_and_hal_failures
    test_async_send_selects_it_and_dma_and_routes_tx_callback
    test_receive_to_idle_start_event_rearm_zero_event_and_stop_late_gate
    test_error_maps_all_flags_notifies_and_rearms_armed_receive
    test_weak_callbacks_ignore_unknown_unarmed_and_missing_user_callbacks
    test_route_capacity_failure_frees_allocated_context)

# Registered-callback mode stays separate from weak-symbol coverage.
add_executable(test_impl_stm32f4_uart_register
    suites/test_impl_stm32f4_uart_register.c
    ${STM32F4_IMPL}/uart/impl_stm32_uart.c
)
target_link_libraries(test_impl_stm32f4_uart_register PRIVATE stm32f4_test_support)
target_compile_definitions(test_impl_stm32f4_uart_register PRIVATE
    USE_HAL_UART_REGISTER_CALLBACKS=1U)
target_compile_options(test_impl_stm32f4_uart_register PRIVATE -Wno-int-to-pointer-cast)
cod_add_test(
    TARGET test_impl_stm32f4_uart_register
    CASES
        test_rx_event_registration_failure_frees_context
        test_tx_registration_failure_frees_context_and_clears_slot
        test_registered_callbacks_route_users_and_rearm_receive
        test_error_registration_failure_clears_old_thunks_before_slot_reuse
        test_all_eight_thunk_sets_execute_and_ninth_context_is_rejected
    LABELS stm32f4 register cmock)

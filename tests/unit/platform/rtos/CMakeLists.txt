# Vendor-neutral RTOS platform sources use generated impl getter and vtable mocks.
set(PLATFORM_RTOS_TEST ${CMAKE_CURRENT_SOURCE_DIR})
set(PLATFORM_RTOS ${PLATFORM}/rtos)

add_library(platform_rtos_test_support STATIC)
cod_add_cmock_runtime(
    TARGET platform_rtos_test_support
    SOURCES
        platform_rtos_cmock_order.c
        mocks/mock_prtos_impl_memory.c
        mocks/mock_prtos_impl_mutex.c
        mocks/mock_prtos_impl_sem.c
        mocks/mock_prtos_impl_task.c
        mocks/mock_prtos_platform_rtos_backend.c
    INCLUDES
        ${PLATFORM_RTOS_TEST}
        ${PLATFORM_RTOS_TEST}/contracts
        ${PLATFORM_RTOS_TEST}/mocks
        ${IMPL_COMMON}
        ${PLATFORM_RTOS}/memory
        ${PLATFORM_RTOS}/mutex
        ${PLATFORM_RTOS}/sem
        ${PLATFORM_RTOS}/task
    DEFINITIONS
    OPTIONS)

function(add_platform_rtos_test module)
    set(name test_plat_${module})
    add_executable(${name}
        suites/${name}.c
        ${PLATFORM_RTOS}/${module}/plat_${module}.c
    )
    target_link_libraries(${name} PRIVATE platform_rtos_test_support)
    cod_add_test(TARGET ${name} CASES LABELS platform_rtos rtos_platform unit cmock)
endfunction()

add_platform_rtos_test(memory)
add_platform_rtos_test(mutex)
add_platform_rtos_test(sem)
add_platform_rtos_test(task)

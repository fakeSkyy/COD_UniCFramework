# FreeRTOS backends compile against a minimal host port contract.
set(FREERTOS_IMPL ${IMPL}/rtos/freertos)
set(RTOS_TEST ${CMAKE_CURRENT_SOURCE_DIR})

add_library(rtos_test_support STATIC)
cod_add_cmock_runtime(
    TARGET rtos_test_support
    SOURCES
        host_rtos.c
        mocks/mock_FreeRTOS.c
        mocks/mock_task.c
        mocks/mock_semphr.c
    INCLUDES
        ${RTOS_TEST}
        ${RTOS_TEST}/freertos
        ${RTOS_TEST}/mocks
        ${IMPL_COMMON}
        ${FREERTOS_IMPL}
        ${FREERTOS_IMPL}/task
        ${FREERTOS_IMPL}/mutex
        ${FREERTOS_IMPL}/sem
        ${FREERTOS_IMPL}/memory
    DEFINITIONS _GNU_SOURCE
    OPTIONS)

function(add_freertos_impl_test module source)
    set(name test_impl_freertos_${module})
    add_executable(${name}
        suites/${name}.c
        ${FREERTOS_IMPL}/${source}
    )
    target_link_libraries(${name} PRIVATE rtos_test_support)
    if(module STREQUAL "fault")
        target_compile_options(${name} PRIVATE -Wno-int-to-pointer-cast)
    endif()
    cod_add_test(TARGET ${name} CASES LABELS rtos freertos unit)
endfunction()

add_freertos_impl_test(task task/impl_task.c)
add_freertos_impl_test(mutex mutex/impl_mutex.c)
add_freertos_impl_test(sem sem/impl_sem.c)
add_freertos_impl_test(memory memory/impl_memory.c)
add_freertos_impl_test(hooks rtos_hooks.c)
add_freertos_impl_test(fault rtos_fault.c)

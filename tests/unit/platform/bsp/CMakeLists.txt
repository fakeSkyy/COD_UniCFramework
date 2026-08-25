# Vendor-neutral BSP platform sources run against generated backend mocks.
set(PLATFORM_BSP_TEST ${CMAKE_CURRENT_SOURCE_DIR})
set(PLATFORM_BSP ${PLATFORM}/bsp)

add_library(platform_bsp_test_support STATIC)
cod_add_cmock_runtime(
    TARGET platform_bsp_test_support
    SOURCES
        platform_bsp_cmock_order.c
        mocks/mock_plat_memory.c
        mocks/mock_platform_bsp_backend.c
    INCLUDES
        ${PLATFORM_BSP_TEST}
        ${PLATFORM_BSP_TEST}/contracts
        ${PLATFORM_BSP_TEST}/mocks
        ${IMPL_COMMON}
        ${PLATFORM}/rtos/memory
        ${PLATFORM_BSP}/adc
        ${PLATFORM_BSP}/can
        ${PLATFORM_BSP}/dma_buf
        ${PLATFORM_BSP}/dwt
        ${PLATFORM_BSP}/flash
        ${PLATFORM_BSP}/gpio
        ${PLATFORM_BSP}/iic
        ${PLATFORM_BSP}/pwm
        ${PLATFORM_BSP}/spi
        ${PLATFORM_BSP}/uart
        ${UTILS}/util_ringbuf
    DEFINITIONS
    OPTIONS)

function(add_platform_bsp_test module)
    set(name test_plat_${module})
    add_executable(${name}
        suites/${name}.c
        ${PLATFORM_BSP}/${module}/plat_${module}.c
    )
    target_link_libraries(${name} PRIVATE platform_bsp_test_support)
    cod_add_test(TARGET ${name} CASES LABELS platform_bsp bsp unit cmock)
endfunction()

add_platform_bsp_test(adc)
add_platform_bsp_test(can)
add_platform_bsp_test(dwt)
add_platform_bsp_test(flash)
add_platform_bsp_test(gpio)
add_platform_bsp_test(iic)
add_platform_bsp_test(pwm)
add_platform_bsp_test(spi)
add_platform_bsp_test(uart)
target_sources(test_plat_uart PRIVATE ${UTILS}/util_ringbuf/util_ringbuf.c)

add_executable(test_plat_dma_buf suites/test_plat_dma_buf.c)
target_include_directories(test_plat_dma_buf PRIVATE
    ${TESTS_ROOT}/unity
    ${PLATFORM_BSP}/dma_buf
)
target_link_libraries(test_plat_dma_buf PRIVATE testsupport)
cod_add_test(TARGET test_plat_dma_buf CASES LABELS platform_bsp bsp unit)

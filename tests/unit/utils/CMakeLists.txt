# Broad behavior suites: one executable per source isolates hostile-input crashes.
file(GLOB UTIL_SUITES ${CMAKE_CURRENT_SOURCE_DIR}/suites/test_util_*.c)

foreach(suite ${UTIL_SUITES})
    get_filename_component(name ${suite} NAME_WE)
    if(NOT name MATCHES "_cmock$")
        add_executable(${name} ${suite})
        target_link_libraries(${name} PRIVATE utils testsupport m)
        cod_add_test(TARGET ${name} CASES LABELS utils unit)
    endif()
endforeach()

# Focused CMock suites for the two utility dependency boundaries.
set(UTILS_CMOCK ${CMAKE_CURRENT_SOURCE_DIR})

add_library(utils_cmock_support STATIC)
cod_add_cmock_runtime(
    TARGET utils_cmock_support
    SOURCES
        utils_cmock_order.c
        mocks/mock_SEGGER_RTT.c
        mocks/mock_plat_mutex.c
        mocks/mock_plat_task.c
    INCLUDES
        ${UTILS_CMOCK}
        ${UTILS_CMOCK}/contracts
        ${UTILS_CMOCK}/mocks
    DEFINITIONS
    OPTIONS "-iquote${UTILS_CMOCK}/contracts")

add_executable(test_util_log_cmock
    suites/test_util_log_cmock.c
    ${UTILS}/util_log/util_log.c
)
target_link_libraries(test_util_log_cmock PRIVATE utils_cmock_support)
cod_add_test(TARGET test_util_log_cmock CASES LABELS utils cmock unit)

add_executable(test_util_msgbus_cmock
    suites/test_util_msgbus_cmock.c
    ${UTILS}/util_msgbus/util_msgbus.c
)
target_link_libraries(test_util_msgbus_cmock PRIVATE utils_cmock_support)
cod_add_test(TARGET test_util_msgbus_cmock CASES LABELS utils cmock unit)

# ------------------------------------------------------------------------------
# The same two solver suites, rebuilt against CMSIS-DSP.
#
# UTIL_MATH_USE_CMSIS_DSP defaults to 0, so without these the DSP path would be
# code no test ever compiles, let alone runs -- and it would rot silently, since a
# firmware build only exercises it when someone remembers to set the flag.
#
# The suite sources are reused verbatim rather than copied: asserting the SAME
# behaviour under both backends is the whole point, and a forked copy would drift.
# The module sources are compiled into these targets directly instead of linking
# the `utils` library, because that library is built for the default backend and
# one translation unit cannot hold both.
#
# DSP's own sources compile clean on the host with the system gcc (verified
# 2026/9/29, zero warnings): they are plain C on the non-MVE path, which is the
# only path a Cortex-M7 takes anyway.
set(DSP_ROOT ${REPO_ROOT}/05_vender/stm32cubemx/Drivers/CMSIS/DSP)

set(DSP_SOURCES
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_mult_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_add_f32.c
    ${DSP_ROOT}/Source/FastMathFunctions/arm_atan2_f32.c
)

# util_kf is listed for BOTH targets, not just its own: util_ahrs calls into it,
# and the point of this pair is that the filter underneath is the DSP build too.
# Linking the default-backend `utils` library for it would test the DSP shim on top
# of a standard-library filter, which is not a configuration that ships.
foreach(mod util_kf util_ahrs)
    add_executable(test_${mod}_dsp
        ${CMAKE_CURRENT_SOURCE_DIR}/suites/test_${mod}.c
        ${UTILS}/${mod}/${mod}.c
        ${UTILS}/util_kf/util_kf.c
        ${UTILS}/util_fast_math/util_fast_math.c
        ${UTILS}/util_lpf/util_lpf.c
        ${DSP_SOURCES}
    )
    target_include_directories(test_${mod}_dsp PRIVATE
        ${DSP_ROOT}/Include
        ${DSP_ROOT}/PrivateInclude
        ${REPO_ROOT}/05_vender/stm32cubemx/Drivers/CMSIS/Include
    )
    target_compile_definitions(test_${mod}_dsp PRIVATE UTIL_MATH_USE_CMSIS_DSP=1)
    target_link_libraries(test_${mod}_dsp PRIVATE testsupport m)
    cod_add_test(TARGET test_${mod}_dsp CASES LABELS utils dsp unit)
endforeach()

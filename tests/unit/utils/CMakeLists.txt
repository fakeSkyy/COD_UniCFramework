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

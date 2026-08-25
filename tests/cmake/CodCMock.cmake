include_guard(GLOBAL)

function(cod_add_cmock_runtime)
    foreach(required_keyword TARGET SOURCES INCLUDES DEFINITIONS OPTIONS)
        list(FIND ARGN "${required_keyword}" keyword_index)
        if(keyword_index EQUAL -1)
            message(FATAL_ERROR "cod_add_cmock_runtime requires explicit ${required_keyword}")
        endif()
    endforeach()

    cmake_parse_arguments(PARSE_ARGV 0 COD "" "TARGET" "SOURCES;INCLUDES;DEFINITIONS;OPTIONS")
    if(COD_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "invalid cod_add_cmock_runtime arguments: ${ARGN}")
    endif()
    if(NOT COD_TARGET)
        message(FATAL_ERROR "cod_add_cmock_runtime requires a non-empty TARGET")
    endif()

    target_sources(${COD_TARGET} PRIVATE
        "${COD_UNITY_ROOT}/unity.c"
        "${COD_CMOCK_RUNTIME_ROOT}/cmock.c"
        ${COD_SOURCES})
    target_include_directories(${COD_TARGET} PUBLIC
        "${COD_TESTS_ROOT}"
        "${COD_UNITY_ROOT}"
        "${COD_CMOCK_RUNTIME_ROOT}"
        ${COD_INCLUDES})
    if(COD_DEFINITIONS)
        target_compile_definitions(${COD_TARGET} PUBLIC ${COD_DEFINITIONS})
    endif()
    if(COD_OPTIONS)
        target_compile_options(${COD_TARGET} PUBLIC ${COD_OPTIONS})
    endif()
endfunction()

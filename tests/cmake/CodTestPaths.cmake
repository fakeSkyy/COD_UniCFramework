# Central absolute paths for the native-host test project.
get_filename_component(COD_TESTS_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
get_filename_component(COD_REPO_ROOT "${COD_TESTS_ROOT}/.." ABSOLUTE)

set(COD_APPLICATION_ROOT "${COD_REPO_ROOT}/01_application")
set(COD_DEVICE_ROOT "${COD_REPO_ROOT}/02_device")
set(COD_PLATFORM_ROOT "${COD_REPO_ROOT}/03_platform")
set(COD_IMPL_ROOT "${COD_REPO_ROOT}/04_impl")
set(COD_UTILS_ROOT "${COD_REPO_ROOT}/06_utils")
set(COD_IMPL_COMMON_ROOT "${COD_IMPL_ROOT}/common")

set(COD_TEST_SUPPORT_ROOT "${COD_TESTS_ROOT}/support")
set(COD_TEST_COMMON_SUPPORT "${COD_TEST_SUPPORT_ROOT}/common")
set(COD_TEST_ALLOC_SUPPORT "${COD_TEST_SUPPORT_ROOT}/alloc")
set(COD_UNITY_ROOT "${COD_TESTS_ROOT}/unity")
set(COD_CMOCK_ROOT "${COD_TESTS_ROOT}/cmock")
set(COD_CMOCK_RUNTIME_ROOT "${COD_CMOCK_ROOT}/src")
set(COD_STUBS_ROOT "${COD_TESTS_ROOT}/stubs")

# Compatibility aliases keep module source lists concise while all values remain
# centralized and absolute.
set(TESTS_ROOT "${COD_TESTS_ROOT}")
set(REPO_ROOT "${COD_REPO_ROOT}")
set(APPLICATION "${COD_APPLICATION_ROOT}")
set(DEVICES "${COD_DEVICE_ROOT}")
set(PLATFORM "${COD_PLATFORM_ROOT}")
set(IMPL "${COD_IMPL_ROOT}")
set(UTILS "${COD_UTILS_ROOT}")
set(IMPL_COMMON "${COD_IMPL_COMMON_ROOT}")
set(TEST_SUPPORT_ROOT "${COD_TEST_SUPPORT_ROOT}")
set(TEST_COMMON_SUPPORT "${COD_TEST_COMMON_SUPPORT}")
set(TEST_ALLOC_SUPPORT "${COD_TEST_ALLOC_SUPPORT}")

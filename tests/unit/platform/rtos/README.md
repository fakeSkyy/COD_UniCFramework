# Platform RTOS host tests

This directory directly compiles and tests the production sources in `03_platform/rtos`: one independent Unity executable each for `memory`, `mutex`, `sem`, and `task`.

The committed mocks are generated artifacts from the repository's pinned official CMock 2.7.0. Getter mocks are generated from the real `04_impl/common/impl_memory.h`, `impl_mutex.h`, `impl_sem.h`, and `impl_task.h`. `mock_prtos_platform_rtos_backend` is generated from a synthetic function contract whose signatures mirror every vtable callback used by those headers. The `mock_prtos_` prefix and target-local include paths isolate these files from existing utils and backend mocks.

Generate into a disposable directory and verify the committed copy:

```sh
./tests/scripts/cmock/generate.sh \
  --domain unit-platform-rtos --output /tmp/cod-cmock-platform-rtos
./tests/scripts/cmock/check.sh
```

Run all host tests or only this group:

```sh
./tests/run_tests.sh /tmp/COD_UniCFramework-build-platform-rtos
ctest --test-dir /tmp/COD_UniCFramework-build-platform-rtos -L platform_rtos --output-on-failure
```

CMock strict ordering is enabled through `GlobalExpectCount` and `GlobalVerifyOrder`. Every suite initializes all five generated mocks in `setUp`, then verifies all mocks before destroying all of them in `tearDown`.

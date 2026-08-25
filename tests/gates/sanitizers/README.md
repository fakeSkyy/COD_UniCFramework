# Host sanitizer matrix

The sanitizer matrix instruments every host target, including utils, both STM32 backends, RTOS,
platform, device, application, and cross-layer integration tests. It uses two separate builds so a
failure identifies the sanitizer family directly:

| Configuration | Instrumentation | Runtime policy |
|---|---|---|
| `address` | AddressSanitizer and LeakSanitizer | leak detection enabled; first error aborts |
| `undefined` | UndefinedBehaviorSanitizer | recovery disabled; first error aborts |

Coverage and sanitizer instrumentation are deliberately mutually exclusive. Use separate build
directories so gcov runtime behavior cannot obscure a sanitizer diagnostic.

## Run the complete matrix

```sh
tests/run_sanitizers.sh /tmp/COD_UniCFramework-build-sanitizer
```

The build base must be outside the repository. The command creates:

```text
/tmp/COD_UniCFramework-build-sanitizer-address
/tmp/COD_UniCFramework-build-sanitizer-undefined
```

Each configuration performs a complete clean rebuild and runs all registered CTest processes
serially with `--output-on-failure --no-tests=error`. `sanitizer-status.txt` begins with
`INCOMPLETE` while a configuration is running. Success atomically publishes `PASS`; any capturable
configure, build, test, or signal failure atomically publishes `FAIL` and makes the matrix command
exit nonzero. Only an uncatchable interruption such as `SIGKILL` or runner loss can leave
`INCOMPLETE`.

Run one configuration while diagnosing a failure:

```sh
SANITIZER_CONFIGS=address \
    tests/run_sanitizers.sh /tmp/COD_UniCFramework-build-sanitizer

SANITIZER_CONFIGS=undefined \
    tests/run_sanitizers.sh /tmp/COD_UniCFramework-build-sanitizer
```

## Strict runtime settings

The runner owns the failure policy instead of inheriting ambient sanitizer variables:

```text
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1:abort_on_error=1:
             strict_string_checks=1:detect_stack_use_after_return=1
LSAN_OPTIONS=exitcode=23:report_objects=1
UBSAN_OPTIONS=halt_on_error=1:abort_on_error=1:print_stacktrace=1:
              report_error_type=1
```

There is no LSan suppression file. Device APIs such as buzzer, remote, and steer chassis intentionally
expose create-only opaque objects with process/firmware lifetime. Their host allocator callbacks use
`tests/support/alloc/host_alloc_tracker.c`, which keeps those objects alive for every Unity case and releases them with an
`atexit` handler before LSan runs. Explicit rollback frees are still forwarded immediately, so double
free and use-after-free diagnostics remain active. This helper is linked only into tests and does not
change production allocation behavior.

## Direct CMake use

For a focused target:

```sh
cmake -S tests -B /tmp/unic-asan -DCMAKE_BUILD_TYPE=Debug -DSANITIZER=address
cmake --build /tmp/unic-asan --target test_util_ringbuf
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1:abort_on_error=1 \
    ctest --test-dir /tmp/unic-asan -R '^test_util_ringbuf$' --output-on-failure
```

Allowed `SANITIZER` values are `address`, `undefined`, or empty. The historical
`SANITIZERS=ON` switch remains as a deprecated combined compatibility mode, but CI and local matrix
runs should use the separate configurations above.

# Host cross-layer integration tests

These executables exercise production code across App/Device/Platform or Utils/Platform boundaries. Only the final board/task/allocator contracts and backend vtable callbacks are replaced. UART, SPI, CAN, DWT, mutex and task trampolines remain production code.

Targets are isolated under `<target>/contracts` and `<target>/mocks`; no executable links a mock for a production `PLAT_*` or `DEV_*` function that it also compiles. All generated files come from the repository CMock 2.7 copy with `enforce_strict_ordering: true`, and every suite initializes, verifies, then destroys its generated mock.

The current matrix contains 11 CTest cases across 8 executables. In particular, the indicator/SPI target drives backend-held SPI trampolines into registered user callbacks, while the IMU target switches from 2000 stationary calibration samples and one alignment sample to dynamic gyro data and validates the resulting attitude against the decoded 32-byte UART telemetry frame.

Regenerate deterministically:

```sh
./tests/scripts/cmock/generate.sh \
  --domain integration --output /tmp/cod-cmock-integration
./tests/scripts/cmock/check.sh --json-out /tmp/cod-cmock-check.json
```

Run from a clean host build:

```sh
cmake -S tests -B /tmp/COD_UniCFramework-build-integration -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/COD_UniCFramework-build-integration -j
ctest --test-dir /tmp/COD_UniCFramework-build-integration --output-on-failure
ctest --test-dir /tmp/COD_UniCFramework-build-integration -L integration --output-on-failure
```

The application task suites register separate CTest cases so each case runs in a fresh process and production file-static state cannot leak between scenarios.

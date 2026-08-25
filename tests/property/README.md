# Deterministic property tests

This directory contains three fast host-side CTest properties. Each executable prints its fixed seed on success and includes the same seed in every failure diagnostic.

- `property_ringbuf` drives the real `util_ringbuf.c` through 50,000 mixed single-byte, bulk, flush and forced-full operations. A plain linear reference queue is compared after every operation, including logical contents, occupancy, wrap and untouched bulk-output suffixes.
- `property_crc` compares random payloads and seeds against independent, bit-at-a-time reflected CRC-8 and CRC-16 implementations. It also checks fragmented continuation and append/wire/verify behavior.
- `property_remote` first sends explicit legal DBUS endpoint vectors (all channels at `-660` and `+660`, mouse axes at `-32000` and `+32000`, and switches at legal endpoints 1 and 3), then generates legal frames and delivers one contiguous stream using random callback boundaries (partial frames and multi-frame merged runs). It verifies the real `dev_remote.c` snapshot, key/link state, frame/error counters, and the exact timeout boundary: tick N-1 remains online and tick N publishes the neutral lost-link state. Every byte goes through the UART callback registered by `DEV_Remote_Create`; `remote_test_stub.c` only supplies allocation and callback delivery and contains no DBUS parser.

Standalone run:

```sh
cmake -S tests/property -B /tmp/cod-property
cmake --build /tmp/cod-property --parallel
ctest --test-dir /tmp/cod-property --output-on-failure
```

Parent integration requires only:

```cmake
add_subdirectory(property)
```

The child uses the parent's `TESTS_ROOT`, `UTILS`, `DEVICES`, `PLATFORM`, and `IMPL_COMMON` variables. Production `.c` files are compiled directly into each executable so the tested code is the repository implementation rather than a copied test variant.

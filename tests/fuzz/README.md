# Clang 18 libFuzzer tests

Three harnesses compile and link the repository production sources directly:

- `fuzz_ringbuf`: mixed single/bulk/flush operations checked against a linear queue.
- `fuzz_crc`: CRC-8/CRC-16 checked against independent bitwise references, plus CRC-16 append/verify.
- `fuzz_remote`: arbitrary byte streams and callback chunking through the real `dev_remote.c`; the shared bottom stub only captures and invokes the UART callback.

Every target is compiled and linked with `-fsanitize=fuzzer,address,undefined`. Use the repository runner, which copies the fixed corpus into an out-of-tree build before mutation:

```sh
tests/run_fuzz.sh /tmp/cod-fuzz
FUZZ_RUNS=100000 tests/run_fuzz.sh /tmp/cod-fuzz-long
FUZZ_RUNS=0 FUZZ_MAX_TOTAL_TIME=300 tests/run_fuzz.sh /tmp/cod-fuzz-timeboxed
FUZZ_SEED=20260824 tests/run_fuzz.sh /tmp/cod-fuzz-replay
```

The default is a bounded smoke campaign of 512 runs per harness with fixed seed `1337`. `FUZZ_RUNS`, `FUZZ_MAX_TOTAL_TIME`, and `FUZZ_SEED` can override those controls for longer campaigns or exact replay. All three harnesses receive the same explicit libFuzzer `-seed`, and `fuzz-status.txt` records that seed in terminal `PASS` or `FAIL` state.

`CC` may select another Clang 18+ binary; it defaults to `clang-18`. After resolving and rejecting repository-internal build paths, the runner immediately invalidates any stale status and writes `INCOMPLETE` before checking the compiler or other prerequisites. Its exit trap atomically replaces that state with `FAIL` on any failure; success atomically replaces it with `PASS`. The executable regression check exercises the missing-compiler path:

```sh
tests/fuzz/test_run_fuzz_failure.sh
```

For parent integration (normally excluded from the default all target):

```cmake
add_subdirectory(fuzz EXCLUDE_FROM_ALL)
```

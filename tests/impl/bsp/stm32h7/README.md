# STM32H7 BSP host tests

The suites in `tests/impl/bsp/stm32h7/suites/` compile each production source from
`04_impl/bsp/stm32h7` with the host compiler. Unity runs assertions and CMock
2.7.0 supplies generated mocks for the HAL and `IMPL_malloc`/`IMPL_free`.
`util_registry.c` is intentionally real so interrupt-routing behavior is tested.

Run all host tests from the repository root:

```sh
./tests/run_tests.sh /tmp/COD_UniCFramework-build-host
ctest --test-dir /tmp/COD_UniCFramework-build-host -L stm32h7 --output-on-failure
```

The generated files in `tests/impl/bsp/stm32h7/mocks/` are committed, so normal builds need no
Ruby. To regenerate after changing `tests/impl/bsp/stm32h7/hal/stm32h7xx_hal.h`, install Ruby
3.0 or newer and run:

```sh
./tests/scripts/cmock/generate.sh \
  --domain impl-stm32h7 --output /tmp/cod-cmock-stm32h7
./tests/scripts/cmock/check.sh
```

The host HAL is a test contract, not a replacement vendor SDK. It defines only
the types, constants, registers, and functions consumed by these implementation
sources. Callback-registration switches are set to the HAL weak-callback mode,
which is also supported by production and makes callbacks directly invocable in
tests.

## Coverage cases

Stateful CAN/SPI/UART/IIC cases are registered individually in CTest, so each case runs in a fresh
process and cannot inherit append-only route tables or bus ownership. The focused cases exercise
hardware capacity limits, bus arbitration and sequence rollback, DMA-visible SRAM boundaries and
32-bit range overflow, plus null, late, and mismatched callback delivery. They intentionally do not
reach file-local defensive branches by including production `.c` files into a test translation unit;
each executable continues to compile the production source directly and uses official CMock 2.7.0.

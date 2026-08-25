/**
 * @file README.md
 * @author Gao Xing
 * @date 2026/8/5
 */

# 04_impl/common — vendor-neutral backend contracts

Each `impl_<peripheral>.h` here defines one ops vtable: the contract between
`03_platform` and *any* backend that can satisfy it. These headers are the
reason the platform layer contains no vendor code — it includes only the
contract, never an implementation.

## Why they are not under a `*_bsp/` directory

They used to live in `04_impl/bsp/stm32f4/<peripheral>/`, which made the
contract look like part of the STM32F4 backend. Adding a second backend then
forces a bad choice: keep an otherwise-empty F4 directory alive just to host
the shared headers, or copy them and let the two versions drift.

A contract shared by N backends belongs to none of them, so it sits here.

## What belongs here, and what does not

Here: the `<Peripheral>_Ops_s` struct, its callback typedefs, and the prose
documenting what an implementation must guarantee. Nothing that names a vendor,
and nothing that would stop the header compiling with no HAL present.

Not here: `impl_stm32_*.{h,c}`. Those are backends — they include the contract
from this directory and a vendor SDK, and live under their own `*_bsp/`.

## Adding a backend

Implement every member of the ops for the new chip, expose
`IMPL_<VENDOR>_<PERIPHERAL>_GetOps()` and `..._CreateCtx()`, and wire it in
`01_application/board/board_devices.c`. No file in `03_platform` changes.

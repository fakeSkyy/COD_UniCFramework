# structure.md

The authoritative design and naming rules for this repository. Read this before touching any layer.

## Layout

The layered refactor is **complete**. `01_application` … `06_utils` is the only layout; every source
in the build lives there. There is **no `ref/` directory** in this repository and none in its git
history — earlier revisions of this document said the pre-refactor tree
(`application / components / bsp / algorithm`) had been moved there, but no such tree was ever
committed here. Do not go looking for it.

The one real reference backend is `04_impl/bsp/stm32f4/`: the F407 code this project was ported
from, present and not built. It must not be edited or called into, and it is the worked example of
the "second backend beside the existing one" rule below.

There is no `00_config` layer. An earlier draft of this document reserved one; it was never created
and the idea is dropped — configuration lives in the header of whatever module owns it.

### Layer overview

- **01_application** — Application layer. Written by the end user. `01_application/board/` is the
  composition root and is the one place allowed to include both platform and vendor headers.
- **02_device** — Drivers for concrete devices. Depends on the platform layer only; it must never
  name a vendor type.
- **03_platform** — The core of the framework: one vendor-neutral API per peripheral and RTOS
  service. Binds to `04_impl` through the **ops (vtable) + opaque context** pattern.
- **04_impl** — The concrete implementation behind each platform interface. It implements
  interfaces and nothing else: it must not manage instances, and it must not call upward into
  `03_platform`.
- **05_vender** — Third-party code, replaceable wholesale. Three subtrees: `stm32cubemx/` (CubeMX
  owns it, rewritten on Generate Code), `freertos/` and `segger_rtt/` (upstream, byte-for-byte,
  each with a `MANIFEST.sha256`). **Never edit any of it.**
- **06_utils** — Hardware-independent algorithms and services. Usable from any layer; may not
  depend on anything above it. Depending *downward* is allowed and happens: `util_log` and
  `util_assert` include `SEGGER_RTT.h` from `05_vender`, which is below them.

Dependency direction is strictly downward: `01 → 02 → 03 → 04 → 05`, with `06_utils` available to
all. Never call upward, never skip a layer.

**The one sanctioned exception: `util_msgbus` depends on `03_platform`.** It includes
`plat_mutex.h` and `plat_task.h`, which is upward, and that is deliberate rather than an
oversight — see the rationale at the top of `util_msgbus.c`. A message bus whose whole purpose is
to hand data between tasks needs mutual exclusion and a way to wake a waiter, and both are
properties of the RTOS, not of any algorithm. The alternatives were considered and are worse:

- *Inject the lock and the wake as callbacks.* Every caller then supplies four function pointers
  that can only ever be the `PLAT_Mutex_*`/`PLAT_Task_*` ones, so the dependency still exists —
  it is just no longer greppable, and each call site can now get it wrong.
- *Move the module to `03_platform`.* It contains no vendor knowledge and is fully host-testable,
  so it would be the only thing in that layer with no backend beneath it.

So the coupling is real and named here rather than hidden. Two consequences worth knowing: it is
the only module in `06_utils` that cannot be reused in a project without this platform layer, and
this exception is not a precedent — a new util that wants a lock should take one as a parameter,
and this list should stay one item long. `tests/gates` has no check for this rule, so the guard
is review, not the build.

### Two-level directories: capability first, backend second

`03_platform` and `04_impl` are split by capability, then by backend:

```
03_platform/{bsp,rtos}/<class>/
04_impl/{bsp/stm32h7,bsp/stm32f4,rtos/freertos}/<class>/     stm32h7 is the one built
04_impl/common/                     ops vtable declarations, shared by all backends
```

A second MCU or a different RTOS is a new directory beside the existing one, not an edit to it.
`04_impl/bsp/stm32f4/` is a real instance of this: it is the F407 backend this project was ported
from, still present, not built.

## Naming

1. General

   - Variables: lower `snake_case`
   - Structs: `UpperCamel_Snake_Case` suffixed `_s`, declared as anonymous
     `typedef struct { ... } Xxx_s;`
   - Enums: `UpperCamel_Snake_Case` suffixed `_e`
   - Macros and constants: `UPPER_SNAKE_CASE`

2. Public functions carry their layer's prefix

   | Layer | Files | Public functions |
   |---|---|---|
   | 02_device | `dev_xxx.c/h` | `DEV_Xxx_VerbObject` — `DEV_DJIMotor_SetOutput` |
   | 03_platform | `plat_xxx.c/h` | `PLAT_Xxx_VerbObject` — `PLAT_CAN_SendTo` |
   | 04_impl | `impl_xxx.c/h` | `IMPL_Xxx_VerbObject` — `IMPL_STM32_CAN_GetOps` |
   | 06_utils | `util_xxx.c/h` | `UTIL_Xxx_Verb` — `UTIL_PID_Step` |

   Vendor-specific impl backends put the vendor in the name: `IMPL_STM32_UART_CreateCtx`.

3. Private (file-local) `static` functions use **unprefixed** lower `snake_case` —
   `bind_callbacks`, `bus_pick_fifo`. They still get a full Doxygen comment on the definition.

## Error handling

There is **no global status enum**. Do not go looking for one, and do not add one.

- A call that can fail returns `bool`.
- A call that cannot fail returns `void`.
- A pure computation returns its result.

Entry checks are early-return guard clauses with explicit `== NULL` and mandatory braces:

```c
if (f == NULL)
{
    return false;
}
```

Hot paths deliberately do **not** check the instance pointer — see `util_ringbuf.c` and
`util_registry.c`. A per-sample NULL check on a 1 kHz control path buys nothing a bring-up test
would not have caught.

`UTIL_ASSERT` exists but has **zero call sites**. Do not introduce it into new code without
deciding what it should do in a release build first.

## Comments

- Every public function needs an English Doxygen comment, **on the declaration in the header
  only**. The definition in the `.c` does not repeat it.
- Every file starts with a Doxygen header containing exactly and only these four fields:
  `@file`, `@author`, `@date`, `@version`.
- Private `static` functions get a Doxygen comment on their definition, since they have no
  declaration in a header.
- Comments explain **why**, not what. The reader can see what the code does. What they cannot see
  is the constraint that forced it, the alternative that was rejected, or the hazard being avoided.
  Design trade-offs, hardware errata, and anything that looks wrong but is not all belong in a
  comment.
- Section banners are 79 columns, and the `.h` and `.c` use the same section order:

```c
/* ========================================================================= */
/*  Configuration helpers                                                    */
/* ========================================================================= */
```

## Formatting

`clang-format` with the repository `.clang-format`: LLVM base, 4-space indent, 100-column limit,
Allman braces. Struct-member `/**< */` alignment padding is manual — clang-format aligns the
opening column only.

## Floats

`float` throughout, never `double`. Every literal carries the `f` suffix and every math call uses
the `f` variant (`sinf`, `sqrtf`). An unsuffixed literal silently promotes the whole expression to
double, which on this target means software emulation in the middle of a control loop.

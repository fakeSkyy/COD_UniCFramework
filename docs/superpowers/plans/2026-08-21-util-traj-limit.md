# util_traj_limit 运动学轨迹限制器 Implementation Plan

> **Archived 2026-08-24:** This implementation plan is retained as historical design evidence.
> Its in-tree `build-tests` commands are retired; use `tests/README.md` and external `/tmp` builds
> for all current verification.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 一个把突变的目标位置整形成受速度与加速度双上限约束的位置轨迹的模块，供指令整形使用。

**Architecture:** `06_utils/util_traj_limit/` 提供 `{pos, vel}` 两状态的梯形限制器。`Step(target)` 每步比较剩余距离与**离散**制动距离，据此加速或减速，速度夹到 `±v_max`，接近目标时 snap 收尾。（连续时间的 `v²/(2a)` 不适用 —— 见 Task 2 的修正说明。）纯 float、零分配、不碰 HAL。

**Tech Stack:** C11，ARM GCC（`-Og -Wall`，`CFLAGS` 无 `-flto`），Unity host 测试（CMake + ctest），gcov 覆盖率。

**Spec:** `docs/superpowers/specs/2026-08-21-util-traj-limit-design.md`

## Global Constraints

- 文件头恰好 4 个 Doxygen 字段，无多余：`@file` / `@author Gao Xing` / `@date 2026/8/21` / `@version 1.0`。`.c` 重复同样的头。
- 头文件守卫 `UTIL_TRAJ_LIMIT_H`，收尾 `#endif /* UTIL_TRAJ_LIMIT_H */`。
- 79 列 `/* ===... */` 段落横幅；`.h` 与 `.c` 段落顺序一致。
- 匿名 `typedef struct {...} UTIL_TrajLimit_s;`；公开函数 `UTIL_TrajLimit_<Verb>`。
- **没有全局 status enum。** 可失败用 `bool`，不可失败用 `void`，纯计算返回值本身。
- 入口检查用 early-return 卫语句 + 显式 `== NULL` + 必带花括号。**不用 `UTIL_ASSERT`**（全仓库零调用点）。
- Doxygen 只写在头文件声明上，`.c` 的定义**不重复**。`.c` 内的私有 static 仍带完整 Doxygen。
- 只用 `float`，字面量全带 `f`，数学调 `f` 变体。
- 注释写「为什么」不写「是什么」。
- 缩进 4 空格，花括号独占一行，行宽 100 列。
- 有限性检查用 `util_fast_math.h` 已有的 `UTIL_IsFinitef`，**不要**自己写副本（七个模块已在用它）。
- 复用 `util_fast_math.h` 的 `UTIL_Absf` / `UTIL_Signf` / `UTIL_Clampf`。
- `clang-format -i` 用 `/home/stg/platform_ws/.clang-format`。
- 本仓库**不是 git 仓库**，所以所有 `git commit` 步骤替换为「跑构建 + 跑测试确认干净」。
- 基线：固件零 warning，`ctest` 16 suite 全过。每个 Task 结束都必须仍然如此。

---

## File Structure

| 文件 | 责任 |
|---|---|
| `06_utils/util_traj_limit/util_traj_limit.h` | 类型、6 个公开函数的声明与全部 Doxygen、两个取值器的 inline 定义 |
| `06_utils/util_traj_limit/util_traj_limit.c` | `Init` / `Reset` / `Step` / `IsSettled` |
| `tests/suites/test_util_traj_limit.c` | 11 组行为测试，目标 100% 行 + 分支 |
| `CMakeLists.txt` | 源文件一处 + include 目录一处 |
| `tests/CMakeLists.txt` | `UTIL_SOURCES` 一处（include 目录由 `util_*` glob 自动覆盖） |
| `06_utils/util_td/util_td.h` | 加一句交叉引用，指向本模块 |

Task 1 建立类型与生命周期（Init/Reset/取值器/IsSettled）并接入构建，Task 2 实现 `Step` 的运动学。两个 Task 结束时固件与测试都能构建通过。

---

### Task 1: 类型、生命周期与构建接入

**Files:**
- Create: `06_utils/util_traj_limit/util_traj_limit.h`
- Create: `06_utils/util_traj_limit/util_traj_limit.c`
- Create: `tests/suites/test_util_traj_limit.c`
- Modify: `CMakeLists.txt`（源文件列表 + include 目录）
- Modify: `tests/CMakeLists.txt`（`UTIL_SOURCES`）

**Interfaces:**
- Consumes: `util_fast_math.h` 的 `UTIL_IsFinitef`、`UTIL_Absf`、`UTIL_Signf`、`UTIL_Clampf`（均已存在）
- Produces:
  - `typedef struct { float pos; float vel; float v_max; float a_max; float dt; bool initialized; } UTIL_TrajLimit_s;`
  - `bool UTIL_TrajLimit_Init(UTIL_TrajLimit_s* t, float v_max, float a_max, float dt_s);`
  - `void UTIL_TrajLimit_Reset(UTIL_TrajLimit_s* t, float pos);`
  - `bool UTIL_TrajLimit_IsSettled(const UTIL_TrajLimit_s* t);`
  - `static inline float UTIL_TrajLimit_Get(const UTIL_TrajLimit_s* t);`
  - `static inline float UTIL_TrajLimit_GetRate(const UTIL_TrajLimit_s* t);`
  - `float UTIL_TrajLimit_Step(UTIL_TrajLimit_s* t, float target);`（本 Task 只放一个返回 `t->pos` 的桩，Task 2 实现）

- [ ] **Step 1: 写头文件**

创建 `06_utils/util_traj_limit/util_traj_limit.h`：

```c
/**
 * @file util_traj_limit.h
 * @author Gao Xing
 * @date 2026/8/21
 * @version 1.0
 */

#ifndef UTIL_TRAJ_LIMIT_H
#define UTIL_TRAJ_LIMIT_H

#include <stdbool.h>

/* ==========================================================================
 * Kinematic limiting for a command that can jump
 * ==========================================================================
 *
 * Turns a target that may change instantly into a position trajectory bounded
 * by a speed and an acceleration ceiling. A gimbal told to point somewhere new,
 * a chassis told to drive to a new place: the command jumps, and feeding that
 * jump to a controller asks for infinite acceleration — which shows up as a
 * current spike, a mechanical bang, or overshoot once the controller saturates.
 *
 * @par What this is not: it is not a filter
 * util_td also bounds acceleration, so the two look interchangeable. They are
 * not. util_td ESTIMATES a signal and its derivative out of a noisy
 * measurement, through a nonlinear law, and puts no ceiling on speed. This
 * limits a CLEAN command deterministically, and guarantees both ceilings.
 *
 * Picking the wrong one produces something that runs but has the wrong
 * property, which no compiler will catch:
 *   - a noisy sensor through this limiter comes out rate-limited but still
 *     noisy, and the limiting hides how noisy;
 *   - a joystick command through util_td can exceed what the mechanism can do,
 *     because nothing in it bounds speed.
 *
 * @par The profile is trapezoidal, so acceleration steps
 * Speed ramps at a_max, holds at v_max, and ramps down again. Acceleration is
 * therefore discontinuous at each corner — there is no jerk limit, and a
 * mechanism stiff enough to care will be heard doing it. Bounding jerk needs a
 * third state and a seven-phase profile; it is deliberately not here.
 *
 * @par Allocation and concurrency
 * Caller-owned storage, no dynamic allocation. An instance must be stepped from
 * one context only.
 * ==========================================================================
 */

/**
 * @brief A limiter's state and its two ceilings. Treat every field as private.
 */
typedef struct
{
    float pos;         /**< Limited output; the value handed out.          */
    float vel;         /**< Current rate; the brake test needs it.         */
    float v_max;       /**< Speed ceiling, always positive.                */
    float a_max;       /**< Acceleration ceiling, always positive.         */
    float dt;          /**< Step period, seconds. Not a tuning knob.       */
    bool  initialized; /**< Seeds pos from the first target when false.    */
    bool  settled;     /**< True only right after Step lands on target.    */
} UTIL_TrajLimit_s;

/* ========================================================================= */
/*  Lifecycle                                                                */
/* ========================================================================= */

/**
 * @brief Set the ceilings and the step period.
 *
 * Leaves the instance unseeded, so the first Step adopts its target instead of
 * driving there from zero — which is what a limiter created mid-flight wants,
 * and avoids a full-speed sweep from 0 on the first call.
 *
 * @param t      Instance to initialise.
 * @param v_max  Speed ceiling in units per second. MUST be positive.
 * @param a_max  Acceleration ceiling in units per second squared. MUST be
 *               positive. Applies to both speeding up and slowing down;
 *               asymmetric limits would be a second field, and are not here.
 * @param dt_s   Period at which Step will be called, in seconds (e.g. 0.001
 *               for a 1 kHz loop). MUST be positive. Fixed here rather than
 *               passed per Step, matching util_td and util_lpf — so the caller
 *               must actually call Step at this rate, or the kinematic bounds
 *               describe a timebase that does not exist.
 * @return true when every argument was accepted. On false the instance is
 *         loaded with safe defaults (all ceilings 1.0, dt 1 ms) and stays
 *         usable, so a mistake during bring-up shows up as motion that is
 *         wrong rather than as a NaN propagating downstream.
 */
bool UTIL_TrajLimit_Init(UTIL_TrajLimit_s* t, float v_max, float a_max, float dt_s);

/**
 * @brief Place the output at @p pos and stop it there.
 *
 * For re-engaging a control loop: without it the limiter would ramp from
 * wherever it was left, which is a transient the mechanism did not ask for.
 *
 * @param t    Instance to reset.
 * @param pos  Position to adopt. A non-finite value leaves the instance
 *             unseeded instead of storing it, so the next Step re-seeds from
 *             its target — storing it would make every later rem = target - pos
 *             non-finite and poison the instance for good.
 */
void UTIL_TrajLimit_Reset(UTIL_TrajLimit_s* t, float pos);

/* ========================================================================= */
/*  Stepping                                                                 */
/* ========================================================================= */

/**
 * @brief Advance one period towards @p target and return the limited position.
 *
 * Call at the rate given to Init.
 *
 * @param target  Where the output should end up. A non-finite value is ignored
 *                and the previous output is held — the bad value never enters
 *                the state, so one glitch costs one step rather than every
 *                step after it.
 * @return The limited position, same as UTIL_TrajLimit_Get.
 */
float UTIL_TrajLimit_Step(UTIL_TrajLimit_s* t, float target);

/**
 * @brief Whether the output has arrived and stopped.
 *
 * True only after a Step landed the output exactly on its target with zero
 * rate. Useful for sequencing — "do not start the next move until this one
 * finished" — which a position comparison alone cannot express, because a
 * limiter passing through its target at speed is not finished.
 *
 * @param t  Instance to test.
 * @return true when the output is stationary at its target.
 */
bool UTIL_TrajLimit_IsSettled(const UTIL_TrajLimit_s* t);

/* ========================================================================= */
/*  Output                                                                   */
/* ========================================================================= */

/**
 * @brief The limited position, without advancing.
 * @param t  Instance to read. Must not be NULL.
 * @return Current output.
 */
static inline float UTIL_TrajLimit_Get(const UTIL_TrajLimit_s* t) { return t->pos; }

/**
 * @brief The current rate, units per second, signed.
 *
 * Handed out because a controller that can accept a velocity feed-forward does
 * far better with the limiter's own rate than with a difference of consecutive
 * positions, which is the same number plus quantisation noise.
 *
 * @param t  Instance to read. Must not be NULL.
 * @return Current rate.
 */
static inline float UTIL_TrajLimit_GetRate(const UTIL_TrajLimit_s* t) { return t->vel; }

#endif /* UTIL_TRAJ_LIMIT_H */
```

Doxygen 只在这里；`.c` 的定义不重复。两个取值器故意不检查 NULL —— 与 `util_ringbuf` / `util_registry` 的热路径一致。

- [ ] **Step 2: 写 `.c` 的生命周期部分与 Step 桩**

创建 `06_utils/util_traj_limit/util_traj_limit.c`：

```c
/**
 * @file util_traj_limit.c
 * @author Gao Xing
 * @date 2026/8/21
 * @version 1.0
 */

#include "util_traj_limit.h"

#include <stddef.h> /* NULL */

#include "util_fast_math.h"

/* ========================================================================= */
/*  Lifecycle                                                                */
/* ========================================================================= */

bool UTIL_TrajLimit_Init(UTIL_TrajLimit_s* t, float v_max, float a_max, float dt_s)
{
    if (t == NULL)
    {
        return false;
    }

    t->pos         = 0.0f;
    t->vel         = 0.0f;
    t->initialized = false;

    /* Every ceiling has to be positive and finite: a zero or negative dt makes
     * one step advance nothing or run time backwards, and a zero ceiling means
     * the output can never move at all — a limiter that silently never reaches
     * its target is harder to diagnose than one that reports a bad argument. */
    if (!UTIL_IsFinitef(v_max) || !UTIL_IsFinitef(a_max) || !UTIL_IsFinitef(dt_s) ||
        v_max <= 0.0f || a_max <= 0.0f || dt_s <= 0.0f)
    {
        /* Defaults rather than a poisoned instance, so Step still produces a
         * bounded number. An unchecked caller then sees motion that is wrong,
         * which is far easier to trace than a NaN arriving somewhere downstream
         * with no indication of where it came from. */
        t->v_max = 1.0f;
        t->a_max = 1.0f;
        t->dt    = 0.001f;

        return false;
    }

    t->v_max = v_max;
    t->a_max = a_max;
    t->dt    = dt_s;

    return true;
}

void UTIL_TrajLimit_Reset(UTIL_TrajLimit_s* t, float pos)
{
    if (t == NULL)
    {
        return;
    }

    t->vel = 0.0f;

    /* A non-finite position is not stored: rem = target - pos would then be
     * non-finite on every later step and the instance would never recover.
     * Leaving it unseeded makes the next Step adopt its target instead. */
    if (!UTIL_IsFinitef(pos))
    {
        t->pos         = 0.0f;
        t->initialized = false;
        return;
    }

    t->pos         = pos;
    t->initialized = true;
}

/* ========================================================================= */
/*  Stepping                                                                 */
/* ========================================================================= */

float UTIL_TrajLimit_Step(UTIL_TrajLimit_s* t, float target)
{
    (void) target;

    return t->pos;
}

bool UTIL_TrajLimit_IsSettled(const UTIL_TrajLimit_s* t)
{
    if (t == NULL)
    {
        return false;
    }

    /* Both conditions: a limiter passing through its target at speed is not
     * finished, and one stopped short of it is not either. Step only ever
     * produces an exact zero rate by landing, so this is exact rather than a
     * tolerance comparison. */
    return t->initialized && t->vel == 0.0f;
}
```

`IsSettled` 的 `vel == 0.0f` 精确比较是刻意的：只有 snap 那条路径会写入精确的 0（加减速路径每步都加减 `dv`，永远到不了精确 0），所以这个比较不需要容差。Task 2 实现 Step 后要验证这个性质仍然成立。

- [ ] **Step 3: 写测试文件（生命周期部分）**

创建 `tests/suites/test_util_traj_limit.c`：

```c
/**
 * @file test_util_traj_limit.c
 * @author Gao Xing
 * @date 2026/8/21
 * @version 1.0
 */

#include "test_support.h"

#include "util_traj_limit.h"

void setUp(void) {}
void tearDown(void) {}

/** @brief Ceilings used by most cases: 10 units/s, 100 units/s^2, 1 kHz. */
#define TL_V_MAX 10.0f
#define TL_A_MAX 100.0f
#define TL_DT 0.001f

/** @brief One step's worth of speed change, a_max * dt. */
#define TL_DV (TL_A_MAX * TL_DT)

/* ========================================================================= */
/*  Rejected arguments                                                       */
/* ========================================================================= */

static void test_tl_null_instance_rejected(void)
{
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(NULL, TL_V_MAX, TL_A_MAX, TL_DT));

    /* Must not fault; a driver may reset an instance it failed to create. */
    UTIL_TrajLimit_Reset(NULL, 1.0f);
    TEST_ASSERT_FALSE(UTIL_TrajLimit_IsSettled(NULL));
}

static void test_tl_init_accepts_valid_ceilings(void)
{
    UTIL_TrajLimit_s t;

    TEST_ASSERT_TRUE(UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_TrajLimit_Get(&t));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_TrajLimit_GetRate(&t));
}

static void test_tl_init_rejects_bad_ceilings_with_safe_defaults(void)
{
    UTIL_TrajLimit_s t;

    /* Each bad argument in turn, and every one must still leave the instance
     * usable rather than poisoned. */
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, 0.0f, TL_A_MAX, TL_DT));
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, -1.0f, TL_A_MAX, TL_DT));
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, NAN, TL_A_MAX, TL_DT));
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, TL_V_MAX, 0.0f, TL_DT));
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, TL_V_MAX, -1.0f, TL_DT));
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, TL_V_MAX, INFINITY, TL_DT));
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, 0.0f));
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, -0.001f));
    TEST_ASSERT_FALSE(UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, NAN));

    /* Still bounded after the last rejection: run it and check nothing blew up. */
    for (int i = 0; i < 100; i++)
    {
        UTIL_TrajLimit_Step(&t, 5.0f);
    }
    TEST_ASSERT_TRUE(UTIL_IsFinitef(UTIL_TrajLimit_Get(&t)));
}

/* ========================================================================= */
/*  Reset                                                                    */
/* ========================================================================= */

static void test_tl_reset_adopts_position_and_zeroes_rate(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 3.5f);

    TEST_ASSERT_EQUAL_FLOAT(3.5f, UTIL_TrajLimit_Get(&t));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_TrajLimit_GetRate(&t));
}

static void test_tl_reset_nonfinite_leaves_unseeded(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 3.5f);
    UTIL_TrajLimit_Reset(&t, NAN);

    /* Not stored: rem would be non-finite on every later step. Unseeded means
     * the next Step adopts its target instead. */
    TEST_ASSERT_TRUE(UTIL_IsFinitef(UTIL_TrajLimit_Get(&t)));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_TrajLimit_GetRate(&t));
    /* The "next Step re-seeds" half of this belongs to Task 2 — Step is still a
     * stub here and would return pos. Task 2's Step 1 adds it. */
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_tl_null_instance_rejected);
    RUN_TEST(test_tl_init_accepts_valid_ceilings);
    RUN_TEST(test_tl_init_rejects_bad_ceilings_with_safe_defaults);

    RUN_TEST(test_tl_reset_adopts_position_and_zeroes_rate);
    RUN_TEST(test_tl_reset_nonfinite_leaves_unseeded);

    return UNITY_END();
}
```

`test_support.h` 已经拉进 `<math.h>`（`NAN` / `INFINITY`）和 Unity，照 `test_util_td.c` 的做法。

- [ ] **Step 4: 接入两个 CMakeLists**

`tests/CMakeLists.txt`：在第 82 行 `${UTILS}/util_td/util_td.c` **之后**插一行（保持字母序，`util_td` 之后就是 `util_traj_limit`）：

```cmake
    ${UTILS}/util_traj_limit/util_traj_limit.c
```

根 `CMakeLists.txt` 两处。第 169 行 `06_utils/util_seq/util_seq.c` 之后那一段里，找到 `util_td` 那行并在其后插入：

```cmake
    06_utils/util_traj_limit/util_traj_limit.c
```

第 265 行 `06_utils/util_seq` 所在的 include 目录列表里，同样在 `util_td` 之后插入：

```cmake
    06_utils/util_traj_limit
```

如果这两个列表里 `util_td` 的位置与预期不符，就插在 `util_seq` 之后 —— 两个列表都不是严格字母序，位置不影响正确性。

- [ ] **Step 5: 跑测试，确认 5 个用例全过**

```bash
cd /home/stg/platform_ws/COD_UniCFramework
cmake --build build-tests -j16
./build-tests/test_util_traj_limit
```

Expected: `5 Tests 0 Failures 0 Ignored` / `OK`

本 Task 的 `test_tl_reset_nonfinite_leaves_unseeded` **不包含**末尾那条 `Step` 断言 —— Step 还是桩，它会返回 `pos` 而不是 target。上面 Step 3 给出的测试代码里那一行 `TEST_ASSERT_EQUAL_FLOAT(7.0f, UTIL_TrajLimit_Step(&t, 7.0f));` 请在本 Task 先**删掉**，Task 2 的 Step 1 会把它加回来。

这样做而不是留一个注释掉的断言：注释掉的断言会一直留在文件里，而删掉再加回来在 Task 2 里是一个可见的改动。本 Task 结束时 5 个用例必须全绿，不允许有已知失败。

- [ ] **Step 6: 全量回归 + 固件**

```bash
ctest --test-dir build-tests
cmake --build build -j16 2>&1 | grep -cE "error:|warning:"
```

Expected: 17/17 suites（原 16 + 新增 1），固件 0。

- [ ] **Step 7: 格式化**

```bash
clang-format -i 06_utils/util_traj_limit/util_traj_limit.h \
                06_utils/util_traj_limit/util_traj_limit.c \
                tests/suites/test_util_traj_limit.c
```

再跑一次 Step 6 确认格式化没破坏任何东西。

---

### Task 2: `Step` 的运动学

**Files:**
- Modify: `06_utils/util_traj_limit/util_traj_limit.c`（替换 Step 桩）
- Modify: `tests/suites/test_util_traj_limit.c`（加 10 组测试）
- Modify: `06_utils/util_td/util_td.h`（交叉引用）

**Interfaces:**
- Consumes: Task 1 的 `UTIL_TrajLimit_s`、`Init`、`Reset`、`IsSettled`、`Get`、`GetRate`
- Produces: 完整的 `float UTIL_TrajLimit_Step(UTIL_TrajLimit_s* t, float target)`

- [ ] **Step 1: 先写会失败的测试**

在 `tests/suites/test_util_traj_limit.c` 的 Reset 段之后加入下面 10 组，并把它们加进 `main` 的 `RUN_TEST` 列表（顺序与下面一致）。同时把 Task 1 Step 5 里说的那条断言补上 —— 即 `test_tl_reset_nonfinite_leaves_unseeded` 末尾的 `TEST_ASSERT_EQUAL_FLOAT(7.0f, UTIL_TrajLimit_Step(&t, 7.0f));`。

```c
/* ========================================================================= */
/*  Seeding                                                                  */
/* ========================================================================= */

static void test_tl_first_step_seeds_from_target(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);

    /* Adopted, not driven to: a limiter created mid-flight must not sweep from
     * 0 at full speed on its first call. */
    TEST_ASSERT_EQUAL_FLOAT(50.0f, UTIL_TrajLimit_Step(&t, 50.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_TrajLimit_GetRate(&t));
}

/* ========================================================================= */
/*  Bounds                                                                   */
/* ========================================================================= */

static void test_tl_speed_never_exceeds_v_max(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    /* Asserted per step, not on the total: an average within the ceiling can
     * still hide a single step that broke it. A tiny epsilon absorbs the
     * float rounding of repeated a_max*dt additions. */
    float prev = UTIL_TrajLimit_Get(&t);

    for (int i = 0; i < 2000; i++)
    {
        const float now = UTIL_TrajLimit_Step(&t, 100.0f);

        TEST_ASSERT_TRUE(UTIL_Absf(now - prev) <= TL_V_MAX * TL_DT + 1e-5f);
        prev = now;
    }
}

static void test_tl_acceleration_never_exceeds_a_max(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    float prev_vel = UTIL_TrajLimit_GetRate(&t);

    for (int i = 0; i < 2000; i++)
    {
        UTIL_TrajLimit_Step(&t, 100.0f);

        const float vel = UTIL_TrajLimit_GetRate(&t);

        /* The landing step is exempt: snap zeroes the rate outright, which is
         * the documented trade for stopping exactly on target. */
        if (!UTIL_TrajLimit_IsSettled(&t))
        {
            TEST_ASSERT_TRUE(UTIL_Absf(vel - prev_vel) <= TL_DV + 1e-5f);
        }
        prev_vel = vel;
    }
}

static void test_tl_reaches_v_max_on_a_long_move(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    float peak = 0.0f;

    for (int i = 0; i < 3000; i++)
    {
        UTIL_TrajLimit_Step(&t, 100.0f);

        const float v = UTIL_Absf(UTIL_TrajLimit_GetRate(&t));

        if (v > peak)
        {
            peak = v;
        }
    }

    /* Far enough to finish accelerating, so the cruise phase must exist and sit
     * at the ceiling — this is what makes the profile trapezoidal rather than
     * merely bounded. */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, TL_V_MAX, peak);
}

/* ========================================================================= */
/*  Not overshooting                                                         */
/* ========================================================================= */

static void test_tl_never_overshoots_across_many_distances(void)
{
    /* Distances chosen around the braking distance at v_max, which is where the
     * brake test decides. That distance is the DISCRETE one, 0.505 units, not
     * the continuous v^2/(2a) = 0.5: braking sheds dv once per step, so the step
     * taken at v_max lasts a full dt and the stop needs v_max*dt/2 = 0.005 more.
     * Bracketing 0.5 instead of 0.505 would leave the marginal case untested. */
    static const float distances[] = {0.0005f, 0.005f, 0.05f, 0.5049f, 0.505f,
                                      0.5051f, 1.0f,   5.0f,  50.0f};

    for (unsigned d = 0; d < sizeof distances / sizeof distances[0]; d++)
    {
        UTIL_TrajLimit_s t;

        UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
        UTIL_TrajLimit_Reset(&t, 0.0f);

        const float target = distances[d];

        for (int i = 0; i < 5000; i++)
        {
            const float pos = UTIL_TrajLimit_Step(&t, target);

            /* Never past it, in either direction of approach. The epsilon is
             * for the snap step, which lands exactly on target. */
            TEST_ASSERT_TRUE(pos <= target + 1e-4f);
        }

        TEST_ASSERT_TRUE(UTIL_TrajLimit_IsSettled(&t));
        TEST_ASSERT_EQUAL_FLOAT(target, UTIL_TrajLimit_Get(&t));
    }
}

static void test_tl_settles_in_bounded_steps(void)
{
    /* The regression guard for the brake-predicate correction. The original
     * algorithm failed to settle on 78 of 400 swept distances — it circled the
     * target forever, so IsSettled never became true and Step was not
     * idempotent. A bounded step count is what distinguishes "converges" from
     * "oscillates below the assertion's tolerance", which a pos-only assertion
     * cannot see. 2000 steps is roughly 3x the worst observed settle for this
     * tuning, so it fails loudly on a regression without being brittle. */
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    const float target = 1.0f;
    int         steps  = 0;

    while (!UTIL_TrajLimit_IsSettled(&t) && steps < 2000)
    {
        const float pos = UTIL_TrajLimit_Step(&t, target);

        /* Never past the target on the way in — an overshoot here is the other
         * half of the same defect. */
        TEST_ASSERT_TRUE(pos <= target);
        steps++;
    }

    TEST_ASSERT_TRUE(UTIL_TrajLimit_IsSettled(&t));
    TEST_ASSERT_EQUAL_FLOAT(target, UTIL_TrajLimit_Get(&t));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_TrajLimit_GetRate(&t));
}

static void test_tl_never_overshoots_going_negative(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    for (int i = 0; i < 5000; i++)
    {
        const float pos = UTIL_TrajLimit_Step(&t, -20.0f);

        TEST_ASSERT_TRUE(pos >= -20.0f - 1e-4f);
    }

    TEST_ASSERT_TRUE(UTIL_TrajLimit_IsSettled(&t));
    TEST_ASSERT_EQUAL_FLOAT(-20.0f, UTIL_TrajLimit_Get(&t));
}

/* ========================================================================= */
/*  Reversal                                                                 */
/* ========================================================================= */

static void test_tl_reversal_brakes_before_turning_around(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    /* Get it moving at the ceiling first. */
    for (int i = 0; i < 500; i++)
    {
        UTIL_TrajLimit_Step(&t, 100.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, TL_V_MAX, UTIL_TrajLimit_GetRate(&t));

    /* Now reverse the target while it is running. This is the path where vel
     * and rem have opposite signs: braking must follow -sign(vel), because
     * steering by rem would add speed while still travelling the wrong way. */
    float prev_vel = UTIL_TrajLimit_GetRate(&t);
    bool  crossed  = false;

    for (int i = 0; i < 3000; i++)
    {
        UTIL_TrajLimit_Step(&t, -100.0f);

        const float vel = UTIL_TrajLimit_GetRate(&t);

        if (!UTIL_TrajLimit_IsSettled(&t))
        {
            TEST_ASSERT_TRUE(UTIL_Absf(vel - prev_vel) <= TL_DV + 1e-5f);
            TEST_ASSERT_TRUE(UTIL_Absf(vel) <= TL_V_MAX + 1e-4f);
        }

        /* It must pass through zero rate rather than jumping sign. */
        if (prev_vel > 0.0f && vel <= 0.0f)
        {
            crossed = true;
        }
        prev_vel = vel;
    }

    TEST_ASSERT_TRUE(crossed);
    TEST_ASSERT_TRUE(UTIL_TrajLimit_GetRate(&t) < 0.0f ||
                     UTIL_TrajLimit_IsSettled(&t));
}

/* ========================================================================= */
/*  Landing and hostile input                                                */
/* ========================================================================= */

static void test_tl_settled_is_idempotent(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    for (int i = 0; i < 3000; i++)
    {
        UTIL_TrajLimit_Step(&t, 2.0f);
    }

    TEST_ASSERT_TRUE(UTIL_TrajLimit_IsSettled(&t));

    /* Once landed it must stay put: further steps at the same target must not
     * jitter the output, which is the whole reason snap exists. */
    for (int i = 0; i < 100; i++)
    {
        TEST_ASSERT_EQUAL_FLOAT(2.0f, UTIL_TrajLimit_Step(&t, 2.0f));
        TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_TrajLimit_GetRate(&t));
    }
}

static void test_tl_nonfinite_target_held_and_never_latched(void)
{
    UTIL_TrajLimit_s t;

    UTIL_TrajLimit_Init(&t, TL_V_MAX, TL_A_MAX, TL_DT);
    UTIL_TrajLimit_Reset(&t, 0.0f);

    for (int i = 0; i < 200; i++)
    {
        UTIL_TrajLimit_Step(&t, 1.0f);
    }

    const float held = UTIL_TrajLimit_Get(&t);

    TEST_ASSERT_EQUAL_FLOAT(held, UTIL_TrajLimit_Step(&t, NAN));
    TEST_ASSERT_EQUAL_FLOAT(held, UTIL_TrajLimit_Step(&t, INFINITY));
    TEST_ASSERT_EQUAL_FLOAT(held, UTIL_TrajLimit_Step(&t, -INFINITY));

    /* And it must recover completely — the bad value never entered the state. */
    for (int i = 0; i < 3000; i++)
    {
        UTIL_TrajLimit_Step(&t, 1.0f);
    }

    TEST_ASSERT_TRUE(UTIL_TrajLimit_IsSettled(&t));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, UTIL_TrajLimit_Get(&t));
}
```

加进 `main`：

```c
    RUN_TEST(test_tl_first_step_seeds_from_target);

    RUN_TEST(test_tl_speed_never_exceeds_v_max);
    RUN_TEST(test_tl_acceleration_never_exceeds_a_max);
    RUN_TEST(test_tl_reaches_v_max_on_a_long_move);

    RUN_TEST(test_tl_never_overshoots_across_many_distances);
    RUN_TEST(test_tl_never_overshoots_going_negative);

    RUN_TEST(test_tl_settles_in_bounded_steps);
    RUN_TEST(test_tl_reversal_brakes_before_turning_around);

    RUN_TEST(test_tl_settled_is_idempotent);
    RUN_TEST(test_tl_nonfinite_target_held_and_never_latched);
```

- [ ] **Step 2: 跑测试，确认新用例失败**

```bash
cd /home/stg/platform_ws/COD_UniCFramework
cmake --build build-tests -j16
./build-tests/test_util_traj_limit
```

Expected: 编译通过，但多数新用例 FAIL —— Step 还是桩，输出永远停在 `Reset` 给的位置。具体地，`test_tl_first_step_seeds_from_target` 会因为返回 0 而不是 50 失败，几个 `IsSettled` 断言会因为桩从不 snap 而失败。

`test_tl_never_overshoots_*` 里的 `pos <= target` 可能**意外通过**（桩不动，自然不过冲），这正常 —— 那两个用例的判别力来自末尾的 `IsSettled` 和精确相等断言。

- [ ] **Step 3: 实现 Step**

用下面这段替换 `util_traj_limit.c` 里的 Step 桩：

```c
float UTIL_TrajLimit_Step(UTIL_TrajLimit_s* t, float target)
{
    /* A bad target is ignored rather than stored: letting it into pos would make
     * every later rem non-finite, so one glitched sample would cost every step
     * after it instead of just this one. */
    if (!UTIL_IsFinitef(target))
    {
        return t->pos;
    }

    /* Unseeded means "start here", not "drive here from zero" — see Init. */
    if (!t->initialized)
    {
        t->pos         = target;
        t->vel         = 0.0f;
        t->initialized = true;

        return t->pos;
    }

    const float rem = target - t->pos;

    /* One step's worth of speed change: the resolution of this limiter. */
    const float dv = t->a_max * t->dt;

    /* The rate that covers exactly rem in one step. Landing on it is legal only
     * when the acceleration bound can also zero it on the following step, which
     * is what |v_land| <= dv says. v_max is checked separately: when dv > v_max,
     * rem/dt can be within dv of vel and still exceed v_max. */
    const float v_land = rem / t->dt;

    if (UTIL_Absf(v_land) <= dv && UTIL_Absf(v_land) <= t->v_max &&
        UTIL_Absf(v_land - t->vel) <= dv)
    {
        /* Not vel = 0: this step lands pos on target, and leaving vel at v_land
         * (which is <= dv) lets the next Step clear it legally instead of
         * claiming an instantaneous stop the acceleration bound cannot deliver. */
        t->vel = v_land;
        t->pos = target;

        return t->pos;
    }

    /* Evaluated at the rate this step would reach if it accelerated, not at the
     * current rate: the decision is whether accelerating is still safe, and by
     * the time the current rate's brake distance no longer fits, the extra dv
     * has already been committed. */
    const float v_next = UTIL_Absf(t->vel) + dv;
    const float v_test = (v_next < t->v_max) ? v_next : t->v_max;

    if (UTIL_Absf(rem) >= brake_distance(v_test, dv, t->dt))
    {
        t->vel += UTIL_Signf(rem) * dv;
    }
    else
    {
        /* Braking sheds up to dv but stops AT zero rather than through it.
         * Subtracting a full dv from a rate already smaller than dv reverses the
         * direction of travel, and the pair then flips sign around the target
         * forever — the root cause of every non-settling case in this module's
         * history. */
        if (UTIL_Absf(t->vel) <= dv)
        {
            t->vel = 0.0f;
        }
        else
        {
            t->vel -= UTIL_Signf(t->vel) * dv;
        }
    }

    t->vel = UTIL_Clampf(t->vel, -t->v_max, t->v_max);

    /* No cap on the advance. An earlier version clamped it to rem, which took
     * rem's SIGN too and so REVERSED the step whenever vel and rem disagreed —
     * pos moving backwards while vel reported the old forward speed, then
     * freezing at target while vel kept reporting motion. And no cap can work:
     * passing a target that fell behind a moving output is what a_max forces,
     * not a bug to suppress. vel is already within dv of its previous value and
     * within v_max; pos simply integrates it. */
    t->pos += t->vel * t->dt;

    /* This step moved without landing, so the settled flag cannot stand. */
    t->settled = false;

    return t->pos;
}
```

`brake_distance` 是同一个 `.c` 里的无前缀私有 static，放在 `Step` 之前：

```c
/**
 * @brief Distance this rate coasts through while braking at a_max, in discrete steps.
 *
 * Not the textbook v^2/(2*a): that is the continuous-time answer, and it is short
 * by about |v|*dt/2 — half a step's travel — because a discrete brake sheds dv only
 * once per step, so the step it takes at |v| lasts a full dt rather than an instant.
 *
 * The sum dt * (|v| + (|v|-dv) + (|v|-2dv) + ...) has n = ceil(|v|/dv) positive
 * terms, which closes to dt*(n*|v| - dv*n*(n-1)/2). Closed form rather than a loop
 * because this runs on a 1 kHz path and the loop's iteration count is |v|/dv — 200
 * at a typical tuning, and unbounded as a_max falls.
 *
 * No guard for n < 1 is needed or wanted: the only call site passes a lookahead
 * rate >= dv > 0, so n >= 1 always, and the expression already yields 0 at n == 0.
 */
static float brake_distance(float vel, float dv, float dt)
{
    const float v = UTIL_Absf(vel);
    const float n = ceilf(v / dv);

    return dt * (n * v - dv * n * (n - 1.0f) * 0.5f);
}
```

`ceilf` 需要 `<math.h>`；在 Cortex-M7 上编译成一条 `VRINTP.F32`，不是库调用。

### 这一节修正过三次

本计划的 `Step` 有过四个版本，前三个都被测试推翻，**没有一个是靠阅读代码发现问题的**。完整过程记在 spec 的「这个算法经过三次修正才对」一节，这里只列执行者必须知道的：

1. 最初的 `brake = vel²/(2·a_max)` + `|rem| <= |vel|·dt` 的 snap：400 个距离里 78 个不收敛。
2. 换成离散闭式制动距离 + 放宽到 `(|vel| + dv)·dt` 的窗口：五组固定参数全绿，但随机化参数后仍有反例。
3. 换成"解出 `v_land = rem/dt`"但落地时直接 `vel = 0`：收敛且不过冲，**但破了加速度上限** —— 普通接近固定目标时落地那一步跳 3 倍 `a_max·dt`，巡航中途 95 倍。

第 3 版能溜过去是因为加速度断言被 `if (!IsSettled(&t))` 包着，而那正是唯一违规的那一步。**覆盖率 100% 没有帮助**：那行执行了，只是断言被跳过。所以本模块的加速度与 `v_max` 断言一律无条件，落地不设豁免。

4. 第四版加了一段"把位置推进夹在 `rem` 上"，**那一段本身是缺陷**：它连 `rem` 的符号一起取，`vel` 与 `rem` 异号时反转这一步 —— 交付位置逆着报告的速率移动，交付加速度 5 倍 `a_max`，之后位置冻结而速率照报。内部 `vel` 全程守着上限，所以所有基于速率的断言都通过了。已删除；根本原因是目标落到运动中的输出身后时，越过它是加速度上限物理强制的，任何夹紧都只能伪造一个速率产生不出来的位置。

上面的代码是第五版（删掉夹紧后的形态），执行时照抄，不要按记忆里任何旧式子写。同一轮还改了：`IsSettled` 用独立的 `settled` 字段而不是 `vel == 0`（制动分支在未到达时也产生精确零速）、浮点下限不等式用 `|pos|` 而不是 `|target|`、`Step` 补 NULL 守卫、`Init` 校验 `a_max * dt` 的乘积溢出。

- [ ] **Step 4: 跑测试，确认全过**

```bash
./build-tests/test_util_traj_limit
```

Expected: `24 Tests 0 Failures 0 Ignored` / `OK`（5 个来自 Task 1 + 13 个新增 + 最终审查后的 6 条）

若 `test_tl_never_overshoots_across_many_distances` 在最小的那几个距离上失败，先查 snap 判据 —— 距离小于一步的量时，它必须在第一次 Step 就接住，而不是先加速再发现过冲。

若 `test_tl_reversal_brakes_before_turning_around` 的 `crossed` 为假，那是 `sign(vel)` / `sign(rem)` 用错了的典型症状。

- [ ] **Step 5: 加 `util_td` 的交叉引用**

在 `06_utils/util_td/util_td.h` 的模块横幅注释里，加一段指向本模块。放在 "The two knobs are independent" 那段之后：

```c
 * @par This is an estimator, not a command limiter
 * The acceleration bound here exists to make the estimate follow a noisy input
 * without amplifying its noise; it puts no ceiling on speed, and the law is
 * nonlinear, so the output is not a shape anyone can predict from r alone. A
 * clean command that must respect a mechanism's speed and acceleration limits
 * wants util_traj_limit instead, which guarantees both and produces a
 * trapezoidal profile. Feeding a joystick command through this one can exceed
 * what the mechanism can do.
```

- [ ] **Step 6: 覆盖率**

```bash
cd /home/stg/platform_ws/COD_UniCFramework
rm -rf /tmp/tlcov && cmake -S tests -B /tmp/tlcov -DCOVERAGE=ON > /dev/null
cmake --build /tmp/tlcov -j16 --target test_util_traj_limit > /dev/null
/tmp/tlcov/test_util_traj_limit > /dev/null
cd /tmp/tlcov && gcov -b $(find . -name 'util_traj_limit.c.gcda') 2>/dev/null | grep -A4 "util_traj_limit.c'"
```

要求：行 100%，分支 taken 100%。若有未覆盖分支，先定位它是哪一行：

```bash
find /tmp/tlcov -name 'util_traj_limit.c.gcov' -exec grep -n -B8 "branch.*never executed" {} \;
```

然后为它加测试，**不要**放宽目标。已知可能需要额外用例的分支：`Clampf` 的两端（正向和负向都要撞上 `v_max`，负向已由 `test_tl_never_overshoots_going_negative` 覆盖）。

跑完删掉 `/tmp/tlcov`。

- [ ] **Step 7: 全量回归 + 固件 + 格式化**

```bash
cd /home/stg/platform_ws/COD_UniCFramework
clang-format -i 06_utils/util_traj_limit/util_traj_limit.c \
                tests/suites/test_util_traj_limit.c \
                06_utils/util_td/util_td.h
cmake --build build-tests -j16 && ctest --test-dir build-tests
cmake --build build -j16 2>&1 | grep -cE "error:|warning:"
```

Expected: 17/17 suites，固件 0 warning。记录 FLASH 与 DTCMRAM 百分比 —— 基线 FLASH 14.47% / DTCMRAM 54.33%，本模块无实例，所以 `--gc-sections` 会把它整个丢掉，两个数字**应当不变**。若 FLASH 上升，说明有东西意外引用了它，值得查明。

---

## 验收

- 固件零 warning（`-Wall`）
- `test_util_traj_limit` 24/24 通过；现有 16 个 suite 仍全过（合计 17/17）
- `util_traj_limit.c` 行覆盖与分支覆盖 100%
- FLASH / DTCM 不变（无调用者，gc-sections 丢弃）

数量从最初的 14 变成 18：算法修正后加了一条有限步数的回归护栏、一条专守落地加速度上限的 `test_tl_landing_step_respects_a_max`，以及两条覆盖落地判据新增子句的用例（`test_tl_landing_speed_clamped_to_v_max`、`test_tl_close_retarget_from_cruise_does_not_snap`）。
- `util_td.h` 与 `util_traj_limit.h` 互相指向

## 已知边界

- **无 jerk 限制**，加速度阶跃。S 曲线要多一个状态量和 7 段相位判断，刻意不做。
- **加减速对称**，非对称是加一个字段的增量改动。
- **snap 那一步不严格满足速度约束** —— 明确接受的取舍，测试里用 epsilon 容纳。
- **`dt` 在 Init 固定**，调用方必须以固定周期调用。
- **无调用者**：本模块不接入任何现有代码路径，所以除编译和 host 测试外没有别的验证。上板验证也不可行 —— 这台机器没接调试探针。

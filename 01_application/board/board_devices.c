/**
 * @file board_devices.c
 * @author Gao Xing
 * @date 2026/8/6
 * @version 1.0
 *
 * The code half of board_devices.def: it expands that table into storage, brings
 * every entry up in table order, generates the accessors, and builds the CAN nodes
 * the application asks for.
 *
 * The pair is deliberate. The .def says what is on the board and is pure data; this
 * file says what to do with it and contains no peripheral list of its own. Which
 * backend implements each class comes from the impl layer's bind header, so no chip
 * is named here either — both facts are data this file expands.
 *
 * @par Why everything is in one translation unit
 * Because "the only file that names both the platform layer and a backend" is a
 * property worth being able to check: one grep for impl_stm32_bind.h should return
 * one result. The CAN factory was briefly its own file, on the grounds that it
 * creates instances at runtime while the rest are brought up once — a real
 * difference, but one a section banner expresses just as well. Splitting it
 * duplicated the PLAT_ALLOW_CONSTRUCTION gate, whose entire purpose is to exist in
 * exactly one place, and downgraded the invariant from "one file" to "one
 * directory", which nothing can verify mechanically.
 *
 * Retargeting to another MCU means pointing the bind include at another chip's
 * directory. Neither this file nor board_devices.def changes.
 */

/* This file is the composition root, so it is the one place allowed to build
 * platform instances. Every PLAT_*_Init and PLAT_*_Create is declared behind this
 * gate precisely so that a stray call from an application or device file is a
 * compile error rather than a review comment. Must precede the plat_*.h includes
 * below. */
#define PLAT_ALLOW_CONSTRUCTION

#include <stddef.h>

#include "board.h"

/* The binding: IMPL_OPS and IMPL_CTX resolve to whichever backend implements a
 * class on this chip. The only line here that names the chip. */
#include "impl_stm32_bind.h"

/* board.h forward-declares the handle types, which is all its callers need. This
 * file is the one place that calls PLAT_*_Init, so it needs the real
 * declarations — and being the composition root, naming both sides is its job. */
#include "plat_can.h"
#include "plat_dwt.h"
#include "plat_flash.h"
#include "plat_pwm.h"
#include "plat_spi.h"
#include "plat_uart.h"

#include "fdcan.h" /* CubeMX FDCAN handles (hfdcan1..3)              */
#include "main.h"  /* CubeMX pin macros (ACCEL_CS_*, GYRO_CS_*)      */
#include "spi.h"   /* CubeMX SPI handles (hspi2, hspi6)              */
#include "tim.h"   /* CubeMX timer handles (htim3, htim12)           */
#include "usart.h" /* CubeMX UART handles (huart10)                  */

/* ========================================================================= */
/*  Instances (private)                                                      */
/* ========================================================================= */

/* Caller-owned storage rather than PLAT_*_Create's heap allocation: the count is
 * fixed at build time, so allocating removes nothing and adds one way for
 * bring-up to fail that has nothing to do with the hardware.
 *
 * Each carries a companion flag so an accessor can report "not up" rather than
 * hand back a pointer to a zeroed instance, which the platform layer would
 * dereference on its hot paths without testing. */
#define BOARD_BUS(name, handle)
#define BOARD_DEVICE(getter, name, Class, ...)                                                     \
    static Class##_Instance_s s_##name;                                                            \
    static bool               s_##name##_up;                                                       \
    static void*              s_##name##_ctx;
#include "board_devices.def"
#undef BOARD_DEVICE
#undef BOARD_BUS

/**
 * @brief Name of the first device that failed, or NULL when none has.
 *
 * A string rather than an index, because an index would have to be mapped back to
 * a line of board_devices.def by hand at the exact moment — bring-up on
 * unfamiliar hardware — when that is least welcome.
 */
static const char* s_failed;

/**
 * @brief Release one device's context by table index; a no-op for a NULL slot.
 *
 * One function per device, generated so a new board_devices.def line gets a
 * teardown step without a hand-written switch anywhere. Indexed rather than
 * called by name so the reverse loop in Board_Init can walk the same table
 * that brought devices up, backwards, without knowing any of their names.
 */
typedef void (*board_teardown_fn)(void);

#define BOARD_BUS(name, handle)
#define BOARD_DEVICE(getter, name, Class, ...)                                                     \
    static void teardown_##name(void)                                                              \
    {                                                                                              \
        IMPL_DESTROY_CTX(IMPL_BACKEND_##Class, s_##name##_ctx);                                    \
        s_##name##_ctx = NULL;                                                                     \
    }
#include "board_devices.def"
#undef BOARD_DEVICE
#undef BOARD_BUS

/* Same order as bring-up, walked backwards below — so teardown always undoes
 * the most recently created device first, mirroring how bring-up depends on
 * what came before it. */
static const board_teardown_fn s_teardown[] = {
#define BOARD_BUS(name, handle)
#define BOARD_DEVICE(getter, name, Class, ...) teardown_##name,
#include "board_devices.def"
#undef BOARD_DEVICE
#undef BOARD_BUS
};

/* ========================================================================= */
/*  Assembly                                                                 */
/* ========================================================================= */

bool Board_Init(void)
{
    /* Release whatever the previous call built, before building anything new,
     * in the reverse of bring-up order — the loop steps from the last table
     * entry to the first. Each teardown function tests its own stored context
     * pointer rather than the _up flag: a call that failed partway leaves later
     * entries with neither a context nor an _up flag, and earlier, successful
     * entries with both, so testing _ctx (inside IMPL_DESTROY_CTX's backend,
     * which accepts NULL) covers both without needing to know which case this
     * is. NULLing it after the free is what makes a third consecutive call
     * safe: with nothing pending, this loop costs one function call per device
     * and each one frees nothing.
     *
     * This does not release the bus/routing state a backend published on the
     * vendor handle (e.g. the SPI backend's shared bus-arbitration record) — see
     * each backend's DestroyCtx. That state is deliberately kept: bus_acquire is
     * idempotent per handle, so bringing the same device back up finds the
     * existing record rather than accumulating a new one, and nothing here needs
     * to touch it. Only the per-device context is this call's to free. */
    for (size_t i = sizeof(s_teardown) / sizeof(s_teardown[0]); i > 0u; --i)
    {
        s_teardown[i - 1u]();
    }

    s_failed = NULL;

    /* One block per entry, in table order, each stopping the rest on failure.
     * Generating the check rather than writing it per peripheral is the point: an
     * earlier version assigned each result without testing it, so a failure was
     * stored as NULL and only surfaced later as a fault inside whatever first
     * used the handle — far from the cause. */
#define BOARD_BUS(name, handle)
#define BOARD_DEVICE(getter, name, Class, ...)                                                     \
    s_##name##_up = false;                                                                         \
                                                                                                   \
    if (s_failed == NULL)                                                                          \
    {                                                                                              \
        s_##name##_ctx = IMPL_CTX(IMPL_BACKEND_##Class, __VA_ARGS__);                              \
                                                                                                   \
        /* A NULL ctx means the backend refused the description — a bad pin, an                  \
         * out-of-range identifier, no memory. Passing it on would only have                       \
         * PLAT_*_Init reject it again, one step further from the cause. */                        \
        if (s_##name##_ctx == NULL)                                                                \
        {                                                                                          \
            s_failed = #name;                                                                      \
        }                                                                                          \
        else if (!PLAT_##Class##_Init(&s_##name, IMPL_OPS(IMPL_BACKEND_##Class), s_##name##_ctx))  \
        {                                                                                          \
            s_failed = #name;                                                                      \
        }                                                                                          \
        else                                                                                       \
        {                                                                                          \
            s_##name##_up = true;                                                                  \
        }                                                                                          \
    }
#include "board_devices.def"
#undef BOARD_DEVICE
#undef BOARD_BUS

    /* Grow the board by adding a line to board_devices.def. Nothing here changes:
     * the storage, this bring-up step, the failure name and the accessor are all
     * expanded from that one line.
     *
     * e.g. an ADC input (after enabling ADC in CubeMX, which is what defines
     * IMPL_BACKEND_ADC in the bind header):
     *   BOARD_DEVICE(CurrentSense, current, ADC, &hadc1, ADC_CHANNEL_0)
     */

    return s_failed == NULL;
}

const char* Board_FailedDevice(void) { return s_failed; }

/* ========================================================================= */
/*  Accessors                                                                */
/* ========================================================================= */

#define BOARD_BUS(name, handle)
#define BOARD_DEVICE(getter, name, Class, ...)                                                     \
    Class##_Instance_s* Board_##getter(void) { return s_##name##_up ? &s_##name : NULL; }
#include "board_devices.def"
#undef BOARD_DEVICE
#undef BOARD_BUS

/* ========================================================================= */
/*  CAN node factory                                                         */
/* ========================================================================= */

/* Everything above is brought up once, from the table, before Board_Init returns.
 * This is the one board facility that is not: a CAN node is created at runtime, as
 * many times as the robot has devices, by the application rather than by bring-up.
 *
 * What the two share is the bus table, which both halves expand out of the same
 * .def — which is why this is a section here rather than a second file. */

CAN_Instance_s* Board_CANCreate(Board_CANBus_e bus, uint32_t tx_id, uint32_t rx_id)
{
    /* Expanded from the same list as the selector enum and indexed by it, so a new
     * bus cannot end up with an enumerator and no handle. The hand-written if/else
     * chain this replaced was a second copy of the same fact in another file.
     *
     * FDCAN_HandleTypeDef, not the CAN_HandleTypeDef this said on the F407: an H7
     * has no bxCAN, and the two handle types share nothing but a naming pattern. */
    static FDCAN_HandleTypeDef* const handle_of[] = {
#define BOARD_DEVICE(getter, name, Class, ...)
#define BOARD_BUS(name, handle) &handle,
#include "board_devices.def"
#undef BOARD_BUS
#undef BOARD_DEVICE
    };

    _Static_assert((sizeof handle_of / sizeof handle_of[0]) == (size_t) BOARD_CAN_COUNT,
                   "handle table and Board_CANBus_e must come from the same list");

    /* One unsigned compare covers both ends, including BOARD_CAN_COUNT itself —
     * a valid enum constant a caller could reach by mistake. */
    if ((unsigned) bus >= (sizeof handle_of / sizeof handle_of[0]))
    {
        return NULL;
    }

    void* ctx = IMPL_CTX(IMPL_BACKEND_CAN, handle_of[bus], tx_id, rx_id);

    if (ctx == NULL)
    {
        return NULL;
    }

    /* Create, not Init: how many nodes exist is decided at runtime by the
     * application, so there is no fixed storage to hand in. */
    return PLAT_CAN_Create(IMPL_OPS(IMPL_BACKEND_CAN), ctx);
}

/**
 * @file board_stm32h7.c
 * @author Gao Xing
 * @date 2026/9/2
 * @version 1.0
 *
 * The composition root for this board on this chip: it names every peripheral, the
 * STM32H7 backend that implements it, and the CubeMX handle it sits on. This is the
 * one file in 01_application allowed to know both, and the one grep that proves it:
 * `grep -rl impl_stm32_ --include=*.c 01_application 02_device` returns this file
 * alone.
 *
 * @par Why the file is named for the chip
 * Retargeting means writing board_stm32f4.c beside this one and building that
 * instead. Everything above stays on Board_* and never learns which was chosen, so
 * the seam is a CMake source-list entry rather than an include path or a macro.
 *
 * @par Explicit calls, not a generated table
 * An earlier version expanded board_devices.def — one line per device — into
 * storage, bring-up, teardown, accessors and the failure name through six separate
 * macro expansions, reaching the backend through IMPL_CTX/IMPL_OPS and an
 * impl_stm32_bind.h that mapped a class to a backend prefix by token pasting.
 * That guaranteed the five copies of each device could never drift, which is a real
 * property and the reason it was built that way.
 *
 * It is written out longhand here because the cost outgrew the benefit. Reading any
 * one thing meant reading one file through six different macro definitions; a
 * mistake inside a macro body was reported at the #include line, multiplied once per
 * table row; IDE navigation, completion and debugger expression evaluation did not
 * work on any of it; and the token-paste layer needed two levels of indirection plus
 * a comment explaining that class names like DWT and ADC are themselves object-like
 * macros in the vendor headers. With eight devices and two buses, `grep` over this
 * file answers every question the generator's invariant was protecting, and the
 * compiler still catches the mistakes that matter — a missing accessor is an
 * undefined symbol, a wrong handle type is a type error at the call.
 *
 * What was NOT given up: the ops + opaque context seam, the caller-owned storage,
 * the bring-up order, the reverse-order idempotent teardown, and the NULL-on-not-up
 * accessor contract. Those are load-bearing and behave exactly as before. The
 * hardware knowledge that lived in board_devices.def's comments is carried over
 * verbatim at each device below; it is the most valuable thing in this file.
 *
 * @par What replaced the HAL_*_MODULE_ENABLED gating
 * impl_stm32_bind.h included each backend header only when CubeMX had enabled that
 * HAL module, so a board that used no ADC did not pay for one. Here the same effect
 * is structural: this file includes exactly the six backend headers it uses. A
 * seventh peripheral means one more include, and a board that does not use ADC never
 * mentions it. Using a peripheral CubeMX did not configure is still a compile error,
 * now because its handle does not exist rather than because a macro is undefined.
 */

/* ========================================================================= */
/*  Where to edit                                                            */
/* ========================================================================= */

/* Adding, removing or re-wiring a peripheral touches FOUR places in this file, all
 * marked "EDIT HERE". Everything else is machinery that does not change when the
 * board's device list does.
 *
 *   grep -n 'EDIT HERE' 01_application/board/board_stm32h7.c
 *
 *   1. EDIT HERE (1/4)  includes        -- the backend header, the plat header, and
 *                                         the CubeMX header holding the handle
 *   2. EDIT HERE (2/4)  device table    -- one BOARD_DEVICE line: storage + accessor
 *   3. EDIT HERE (3/4)  bring-up        -- one BOARD_BRING_UP call, plus the comment
 *                                         recording why its arguments are what they are
 *   4. EDIT HERE (4/4)  board.h         -- the accessor prototype (a different file)
 *
 * Teardown is NOT on that list: BOARD_BRING_UP records how to release each context as
 * it creates it, so there is no second list to keep in step. That used to be a fifth
 * edit site, and forgetting it leaked one context per Board_Init.
 *
 * Two mistakes are compile errors rather than silent ones: a missing BOARD_DEVICE line
 * (the storage does not exist) and an accessor whose name disagrees with board.h (the
 * #pragma below turns that into a named error). A missing bring-up call is not caught
 * — the device simply never comes up and its accessor returns NULL.
 *
 * The machinery is marked "MACHINERY" and is worth reading once, then leaving alone:
 * the three macros, the teardown ledger, and Board_Init's control flow. */

/* Composition root, so it is the one place allowed to build platform instances.
 * Every PLAT_*_Init is declared behind this gate precisely so a stray call from an
 * application or device file is a compile error rather than a review comment. Must
 * precede the plat_*.h includes. */
#define PLAT_ALLOW_CONSTRUCTION

#include "board.h"

#include <stddef.h>

/* ----- EDIT HERE (1/4): includes ---------------------------------------- *
 * Three per new peripheral class, and none at all when the class is already used by
 * another device: the plat header below, its impl_stm32_* backend header, and the
 * CubeMX header that declares the handle. Listing exactly the backends in use is what
 * keeps an unused peripheral out of the build.
 * ------------------------------------------------------------------------ */

/* The platform layer: vendor-neutral API and the ops contracts. */
#include "plat_can.h"
#include "plat_dwt.h"
#include "plat_flash.h"
#include "plat_pwm.h"
#include "plat_spi.h"
#include "plat_uart.h"

/* The backends. Naming these is what makes this file chip-specific, and listing
 * exactly the six in use is what keeps an unused peripheral out of the build. */
#include "impl_stm32_can.h"
#include "impl_stm32_dwt.h"
#include "impl_stm32_flash.h"
#include "impl_stm32_pwm.h"
#include "impl_stm32_spi.h"
#include "impl_stm32_uart.h"

#include "fdcan.h" /* hfdcan1, hfdcan2                          */
#include "main.h"  /* SystemCoreClock, ACCEL_CS_*, GYRO_CS_*    */
#include "spi.h"   /* hspi2, hspi6                              */
#include "tim.h"   /* htim3, htim12                             */
#include "usart.h" /* huart10                                   */

/* ========================================================================= */
/*  Storage -- MACHINERY                                                     */
/* ========================================================================= */

/* Caller-owned storage rather than PLAT_*_Create's heap allocation: the count is
 * fixed at build time, so allocating removes nothing and adds one way for bring-up
 * to fail that has nothing to do with the hardware. Consequently no PLAT_*_Create
 * reaches the image at all — grepping the linked ELF's symbols for PLAT_<CLASS>_Create
 * finds nothing, and that is what makes "zero dynamic allocation" a fact about the
 * firmware rather than about the source.
 *
 * Each instance is paired with an `up` flag so an accessor can report "not up"
 * rather than hand back a pointer into a zeroed instance, which the platform layer
 * dereferences on its hot paths without testing. The context pointer is kept so
 * teardown can release exactly what bring-up built.
 *
 * @par One line per device, and why the accessor is generated with the storage
 * The storage, the flag, the context and the accessor are four names derived from one
 * device. Writing the accessor by hand was a silent failure mode: omitting it built
 * cleanly and simply left the device unreachable. Generating it from the same line
 * removes that, and the `#pragma` below makes the remaining mistake — a getter name
 * that disagrees with board.h — a named compile error rather than an unused-variable
 * warning.
 *
 * @param Class   Peripheral class: <Class>_Instance_s and PLAT_<Class>_Init.
 * @param name    Storage base name; also the string Board_FailedDevice reports.
 * @param Getter  Accessor suffix; becomes Board_<Getter>(), declared in board.h.
 */
#define BOARD_DEVICE(Class, name, Getter)                                                          \
    static Class##_Instance_s s_##name;                                                            \
    static bool               s_##name##_up;                                                       \
    static void*              s_##name##_ctx;                                                      \
                                                                                                   \
    Class##_Instance_s* Board_##Getter(void) { return s_##name##_up ? &s_##name : NULL; }

/* Every accessor generated below must have a prototype in board.h. Without this the
 * compiler accepts a getter name that disagrees with the header and the only hint is
 * an unused-variable warning elsewhere; with it, the mismatch is named directly:
 *
 *   error: no previous prototype for 'Board_ImuAccel' [-Werror=missing-prototypes]
 *
 * Scoped to this file rather than added to CMAKE_C_FLAGS because the flag currently
 * reports 45 pre-existing violations across CubeMX-generated code and
 * 04_impl/rtos — none in 01_application. Enabling it globally is a separate job. */
#pragma GCC diagnostic error "-Wmissing-prototypes"

/* ----- EDIT HERE (2/4): the device table -------------------------------- *
 * One line per device: storage, the up flag, the context slot and the accessor.
 * Order here is irrelevant; bring-up order is set by Board_Init below.
 *
 * The Getter must match the prototype in board.h exactly, or the #pragma above makes
 * it a named compile error.
 * ------------------------------------------------------------------------ */

BOARD_DEVICE(DWT, timebase, Timebase)
BOARD_DEVICE(SPI, imu_accel, ImuAccel)
BOARD_DEVICE(SPI, imu_gyro, ImuGyro)
BOARD_DEVICE(SPI, status_led, StatusLed)
BOARD_DEVICE(UART, debug_uart, DebugUart)
BOARD_DEVICE(PWM, buzzer_pwm, BuzzerPWM)
BOARD_DEVICE(PWM, imu_heater, ImuHeater)
BOARD_DEVICE(Flash, param_flash, ParamFlash)

#undef BOARD_DEVICE

/**
 * @brief Name of the first device that failed, or NULL when none has.
 *
 * A string rather than an index, because an index would have to be mapped back to a
 * device by hand at exactly the moment — bring-up on unfamiliar hardware — when
 * that is least welcome.
 */
static const char* s_failed;

/* ========================================================================= */
/*  Teardown ledger -- MACHINERY (no edit site; derived from bring-up)       */
/* ========================================================================= */

/**
 * @brief What bring-up built, in the order it was built.
 *
 * @par Why a ledger rather than a second list of devices
 * Teardown used to be a hand-written list of eight BOARD_RELEASE lines, 240 lines
 * away from the bring-up calls they undid. Omitting one built cleanly and leaked that
 * device's context on every repeated Board_Init — one of only two silent failure
 * modes left in this file. Recording the release as a side effect of the create means
 * there is no second edit site to forget, and the reverse order falls out of the
 * append order rather than being maintained by hand.
 *
 * The cost is this table: 8 entries of a pointer pair, 64 bytes of .bss on this
 * build, walked only by Board_Init and never on a hot path.
 */
typedef struct
{
    void** ctx_slot;            /**< Where bring-up stored the context.        */
    bool*  up_slot;             /**< The device's up flag, cleared on release. */
    void (*destroy)(void* ctx); /**< That class's backend DestroyCtx.          */
} board_built_s;

/* Sized so the _Static_assert below bites if a device is added without room for it.
 * Deliberately not "however many devices exist": that count is not available to the
 * preprocessor here, and a number that has to be bumped with a named error beats a
 * silent overflow. */
#define BOARD_BUILT_MAX 12u

static board_built_s s_built[BOARD_BUILT_MAX];
static unsigned      s_built_n;

/* ========================================================================= */
/*  Bring-up -- MACHINERY, then the per-device calls (edit site 3/4)         */
/* ========================================================================= */

/* Each device is three explicit calls: build the backend context, fetch the ops
 * vtable, wrap both in the platform instance. The pattern repeats, and the repeated
 * part is exactly the part the compiler checks — each CreateCtx has its own
 * parameter list, so a wrong handle or a swapped pin argument is a type error at
 * the call rather than something a table row could hide.
 *
 * A NULL ctx means the backend refused the description: a bad pin, an out-of-range
 * identifier, no memory. Stopping at the first failure keeps the report pointing at
 * the cause instead of at whatever first used a handle that was never valid — an
 * early version assigned each result without testing it, so a failure surfaced as a
 * fault inside an unrelated driver.
 *
 * The context is recorded in the teardown ledger the moment it exists — before
 * PLAT_<Class>_Init is even attempted — so a device whose platform init fails is
 * still released by the next call. That ordering is the whole reason the ledger is
 * written here rather than after a successful bring-up.
 *
 * @param name       Storage base name, matching BOARD_DEVICE above.
 * @param Class      Peripheral class, for PLAT_<Class>_Init.
 * @param ctx        A full IMPL_STM32_<CLASS>_CreateCtx(...) call, written out.
 * @param ops        The matching IMPL_STM32_<CLASS>_GetOps() call.
 * @param DestroyCtx The matching IMPL_STM32_<CLASS>_DestroyCtx, named not called.
 */
#define BOARD_BRING_UP(name, Class, ctx, ops, DestroyCtx)                                          \
    do                                                                                             \
    {                                                                                              \
        s_##name##_up  = false;                                                                    \
        s_##name##_ctx = (ctx);                                                                    \
                                                                                                   \
        if (s_##name##_ctx == NULL)                                                                \
        {                                                                                          \
            s_failed = #name;                                                                      \
            return false;                                                                          \
        }                                                                                          \
                                                                                                   \
        /* Recorded before PLAT_<Class>_Init, so a platform-init failure still leaves              \
         * a context the next call releases. */                                                    \
        /* The bound is tested, not asserted at the table: a silent overrun of                     \
         * s_built would corrupt whatever follows it in .bss, and BOARD_BUILT_MAX is               \
         * a number a reader must bump when adding the (MAX+1)th device. Reporting it              \
         * as a bring-up failure names the problem instead of hiding it. */                        \
        if (s_built_n >= BOARD_BUILT_MAX)                                                          \
        {                                                                                          \
            s_failed = "board ledger full";                                                        \
            return false;                                                                          \
        }                                                                                          \
                                                                                                   \
        s_built[s_built_n].ctx_slot = &s_##name##_ctx;                                             \
        s_built[s_built_n].up_slot  = &s_##name##_up;                                              \
        s_built[s_built_n].destroy  = (DestroyCtx);                                                \
        s_built_n++;                                                                               \
                                                                                                   \
        if (!PLAT_##Class##_Init(&s_##name, (ops), s_##name##_ctx))                                \
        {                                                                                          \
            s_failed = #name;                                                                      \
            return false;                                                                          \
        }                                                                                          \
                                                                                                   \
        s_##name##_up = true;                                                                      \
    } while (0)

/**
 * @brief Release every context bring-up built, newest first.
 *
 * Teardown undoes the most recently created device first, mirroring how bring-up
 * depends on what came before it — which here is simply the ledger walked backwards,
 * so the order cannot drift from the bring-up order. A partial bring-up needs no
 * special case: the ledger holds exactly what was created, so there is nothing to
 * skip. Emptying it is what makes a third consecutive Board_Init release nothing.
 *
 * This does NOT release the bus/routing state a backend published against the vendor
 * handle (the SPI backend's shared bus-arbitration record, a UART/CAN routing table
 * entry). That state deliberately outlives the context: each backend's bus-acquire
 * step is idempotent per handle, so re-creating a device on the same handle finds the
 * existing record rather than accumulating a new one, and the append-only registries
 * have no way to retract an entry even if this wanted to.
 */
static void board_teardown(void)
{
    while (s_built_n > 0u)
    {
        s_built_n--;

        board_built_s* const built = &s_built[s_built_n];

        if (*built->ctx_slot != NULL)
        {
            built->destroy(*built->ctx_slot);
            *built->ctx_slot = NULL;
        }

        *built->up_slot = false;
    }
}

bool Board_Init(void)
{
    /* Release whatever a previous call built before building anything new, so
     * repeated calls (as bring-up tests do) do not leak one context per peripheral
     * per call. */
    board_teardown();

    s_failed = NULL;

    /* ===== EDIT HERE (3/4): bring-up ===================================== *
     * One BOARD_BRING_UP per device, in the order they should come up — the first
     * failure stops the rest, so a later device may rely on an earlier one.
     *
     * Write the hardware reasoning in the comment above each call: the pin's source,
     * the clock, why this transfer mode. That is the most valuable content in this
     * file and the part that cannot be recovered by reading the code.
     *
     * Nothing else needs editing when a device is added — teardown is derived from
     * these calls, not maintained separately.
     * ===================================================================== */

    /* --------------------------------------------------------------------- *
     * Timebase first: a plain cycle counter with no dependency on any other
     * peripheral, so having it up front means everything created later can rely on
     * microsecond delays being available.
     *
     * CYCCNT counts CPU cycles, so what it needs is the core clock — taken from a
     * value the RCC maintains rather than hardcoded, so that retuning the clock tree
     * in CubeMX cannot silently skew every conversion.
     *
     * Not HAL_RCC_GetHCLKFreq(), which is what this used to say. On the F407 HCLK and
     * SYSCLK were the same 168 MHz, so HCLK happened to be the core clock and the
     * wrong question got the right answer. Here AHBCLKDivider is RCC_HCLK_DIV2: the
     * core runs at 550 MHz while HCLK is 275 MHz, so passing HCLK would make every
     * microsecond conversion and every delay come out 2x long. SystemCoreClock is the
     * CMSIS variable HAL_RCC_ClockConfig() sets to the CM7 core clock, and
     * SystemClock_Config() has run by the time Board_Init is called.
     *
     * Consequence of 550 MHz for callers: CYCCNT is 32 bits, so it wraps every
     * ~7.8 s rather than the F407's ~25.6 s, and the 64-bit timeline is reconstructed
     * by observing those wraps. Anything relying on PLAT_DWT_GetTimeline_* must poll
     * more than twice as often as it did on the F407, or the timeline silently loses
     * 2^32 cycles per missed wrap.
     * --------------------------------------------------------------------- */
    BOARD_BRING_UP(timebase, DWT, IMPL_STM32_DWT_CreateCtx(SystemCoreClock),
                   IMPL_STM32_DWT_GetOps(), IMPL_STM32_DWT_DestroyCtx);

    /* --------------------------------------------------------------------- *
     * The BMI088 is two dies in one package with independent register maps and
     * separate chip selects, so it needs two handles rather than one. They are not
     * interchangeable: the accelerometer inserts a dummy byte into every read and the
     * gyroscope does not, so the read paths genuinely differ.
     *
     * The SPI backend owns both chip-select pins and drives them as part of every
     * transfer. There is deliberately no separate GPIO entry for either: a second
     * owner of a chip select desynchronises against the transfer, and a CS that goes
     * inactive mid-frame aborts it at the sensor.
     *
     * Both pass the same &hspi2, which is what makes the backend give them a shared
     * bus-arbitration record so their transfers cannot overlap. Interrupt-driven
     * rather than DMA: the transfers are a few bytes each, so DMA setup would cost
     * more than it saves — and on this part that is no longer only an efficiency
     * argument, because DMA1/DMA2 cannot reach DTCM, where .bss lives.
     *
     * hspi2, not the F407's hspi1. SPI2 is the only SPI peripheral CubeMX configured
     * here (PB13 SCK, PC1 MOSI, PC2_C MISO), and the two chip selects it configured —
     * PC0 and PC3_C — are labelled for this sensor. Confirmed working against the
     * hardware: both dies answer their who-am-I on these pins.
     *
     * The SPI2 prescaler is load-bearing: 32, not 8. A BMI088 accepts at most 10 MHz
     * on SPI. The kernel clock here is 240 MHz, so prescaler 32 gives 7.5 MHz — the
     * same value the vendor's own example for this board uses — while the 8 that was
     * configured gave 24 MHz, 2.4x over the limit.
     *
     * That failure is worth describing because it does not look like a clock problem.
     * The gyro die still answered correctly (0x0F) while the accelerometer returned a
     * consistent, plausible-looking 0x23 instead of 0x1E — stable across retries, not
     * obviously noise. Every SPI call reported success; only the payload was wrong.
     * The two dies differ because the accelerometer inserts a dummy byte into every
     * read, so it needs one more bus turnaround to survive. A single device answering
     * correctly is therefore NOT evidence that the bus rate is legal.
     *
     * If either die ever fails its ID check, verify this prescaler before suspecting
     * the wiring.
     * --------------------------------------------------------------------- */
    BOARD_BRING_UP(imu_accel, SPI,
                   IMPL_STM32_SPI_CreateCtx(&hspi2, ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, SPI_XFER_IT),
                   IMPL_STM32_SPI_GetOps(), IMPL_STM32_SPI_DestroyCtx);

    BOARD_BRING_UP(imu_gyro, SPI,
                   IMPL_STM32_SPI_CreateCtx(&hspi2, GYRO_CS_GPIO_Port, GYRO_CS_Pin, SPI_XFER_IT),
                   IMPL_STM32_SPI_GetOps(), IMPL_STM32_SPI_DestroyCtx);

    /* --------------------------------------------------------------------- *
     * A SPI instance that is not talking to a SPI device. The status LED is a WS2812
     * on PA7, which is SPI6_MOSI. It has no clock line and no chip select: it decodes
     * the width of the high pulse on a single wire, and SPI6 is here only because its
     * shift register can generate that waveform without a critical section. Hence
     * NULL for the port and 0 for the pin — the SPI backend accepts a CS-less device,
     * see IMPL_STM32_SPI_CreateCtx.
     *
     * The CubeMX settings this depends on, and why they are not obvious. SPI6 must be
     * Transmit Only Master, and three of its parameters are load-bearing for
     * dev_ws2812.c's byte-per-bit encoding:
     *
     *   Kernel clock   -> HSE, 24 MHz. Set in HAL_SPI_MspInit in spi.c, not in
     *                     main.c: H7's CubeMX puts each peripheral's own clock source
     *                     in its MspInit and reserves main.c for sources shared
     *                     between peripherals (PLL2, CKPER). Worth knowing because
     *                     grepping main.c for it finds nothing and suggests it was
     *                     never generated.
     *   Prescaler /4   -> 6.0 MHz, which is DEV_WS2812_SCK_HZ. That constant is the
     *                     driver's stated requirement; this line and the kernel clock
     *                     above are what satisfy it. Nothing checks the two agree, so
     *                     changing either without the other is silent.
     *   Data Size 8    -> one whole byte per colour bit. CubeMX defaults this to 4
     *                     bits when the .ioc has no DataSize key, which halves the
     *                     clock count and produces a waveform the strip cannot decode
     *                     — with no error anywhere, because HAL_SPI_Init accepts it
     *                     and the transfer reports success.
     *
     * All three are in the .ioc (RCC.SPI6CLockSelection, SPI6.BaudRatePrescaler,
     * SPI6.DataSize). If the LED ever shows the wrong colour or ignores data, check
     * them in that order.
     *
     * CubeMX also assigned PA5 to SPI6_SCK. A WS2812 does not use a clock, so that
     * pin is spent for nothing — harmless today because nothing else claims it.
     *
     * SPI_XFER_IT here selects nothing, and must not be changed to DMA. dev_ws2812
     * sends through PLAT_SPI_Send, the blocking path, which ignores this argument
     * entirely — 124 bytes at 6 MHz is 165 us, too short to be worth an interrupt.
     * The mode is only consulted by the async calls, and for SPI6 those would not work
     * as generated: CubeMX enabled no SPI6 interrupt and produced no SPI6_IRQHandler,
     * so an interrupt-driven transfer would never complete. Switching this to
     * SPI_XFER_DMA would additionally require a DMA stream and a buffer outside DTCM.
     * Leave it, and keep using the blocking send.
     * --------------------------------------------------------------------- */
    BOARD_BRING_UP(status_led, SPI, IMPL_STM32_SPI_CreateCtx(&hspi6, NULL, 0u, SPI_XFER_IT),
                   IMPL_STM32_SPI_GetOps(), IMPL_STM32_SPI_DestroyCtx);

    /* --------------------------------------------------------------------- *
     * The telemetry link, PE2/PE3 at 921600. USART10 is the port app_telemetry.c
     * streams the attitude estimate out of, in VOFA+ justFloat format for plotting.
     *
     * The baud rate is what sets the achievable frame rate: 921600, eight times
     * USART1's 115200, which is why the telemetry divider can be what it is. A
     * 32-byte frame is 320 bit-times, so the line carries ~2880 frames a second and
     * 200 Hz uses 7% of it rather than 56%. Raising the frame rate is now a question
     * of whether anything needs it, not of whether the wire can take it. That number
     * lives in CubeMX (USART10.BaudRate), so TELEM_DIVIDER in app_telemetry.c is
     * chosen against it and nothing checks the two agree.
     *
     * UART_XFER_IT rather than DMA, which is not the obvious choice. DMA is unusable
     * here regardless of what CubeMX configured: DMA1/DMA2 cannot reach DTCM, and this
     * linker script puts .bss there, so a send from an ordinary static buffer is
     * rejected outright by the backend (see dma_reachable in impl_stm32_uart.c).
     * Fixing that needs an AXI SRAM section in STM32H723xG_flash.ld — a CubeMX-owned
     * file with no user region around the memory map, so a regeneration would silently
     * drop it and telemetry would stop with every send returning false.
     *
     * Interrupt mode has no such constraint and costs one interrupt per byte, which at
     * 921600 is 92 k/s. That is no longer negligible next to the 1 kHz attitude loop,
     * so it is the reason to watch App_Telemetry_Skipped rather than assume headroom.
     * The send is still asynchronous, so the task is not blocked for the 350 us a
     * 32-byte frame occupies the wire.
     *
     * Unlike SPI6, this path works as generated: CubeMX enabled USART10_IRQn at
     * priority 5 and stm32h7xx_it.c forwards USART10_IRQHandler to HAL_UART_IRQHandler.
     * --------------------------------------------------------------------- */
    BOARD_BRING_UP(debug_uart, UART, IMPL_STM32_UART_CreateCtx(&huart10, UART_XFER_IT),
                   IMPL_STM32_UART_GetOps(), IMPL_STM32_UART_DestroyCtx);

    /* --------------------------------------------------------------------- *
     * TIM12 CH2, not the F407's TIM4 CH3. Only TIM3 (CH4 on PB1) and TIM12 (CH2 on
     * PB15) are configured on this part, and the F407's buzzer pin PD14/TIM4_CH3 does
     * not exist in this configuration at all, so the entry had to move to one of the
     * two. TIM12 is the one whose CubeMX setup is shaped like a tone generator —
     * prescaler 0 and period 20999, i.e. the full counter range left free for the
     * frequency to be rewritten — whereas TIM3 is preloaded to a fixed 172 Hz
     * (Prescaler=80-1, Period=20000 over the 275 MHz APB1 kernel clock) with
     * autoreload preload enabled, the shape of a servo or a heater where the duty
     * varies and the frequency does not.
     *
     * That reasoning was an inference from the timer configuration; it is now
     * confirmed. The vendor's own CtrBoard-H7_BUZZER example puts the buzzer on PB15
     * as S_TIM12_CH2 — the same pin, timer and channel this entry already used. The
     * potential conflict this paragraph used to warn about is therefore resolved
     * rather than merely unlikely: the buzzer is PB15/TIM12_CH2 and the heater below
     * is PB1/TIM3_CH4, so both claims hold on separate pins.
     *
     * Note what the example does not settle. It carries no GPIO_Label either — the
     * buzzer is identified by the project's purpose, exactly as the heater was — so
     * this is confirmation from the vendor's own working configuration, not from a
     * schematic this repository holds. Its period differs too (Prescaler 24-1,
     * Period 2000-1 for 5 kHz against a 240 MHz kernel, where ours is prescaler 0 and
     * 20999). That difference does not matter and must not be copied: dev_buzzer
     * rewrites the frequency per note through PLAT_PWM_SetFreqAndDuty, so the .ioc
     * value is only a starting point, and the wide-open counter is what makes an
     * arbitrary tone expressible.
     *
     * There is no clock argument. Which APB domain a timer is on, and whether the RCC
     * doubles that domain's clock for its timers, are properties of the chip — so the
     * backend derives the rate from the handle rather than having this file state a
     * number it has no way to check. See timer_input_clk_hz in impl_stm32_pwm.c.
     * --------------------------------------------------------------------- */
    BOARD_BRING_UP(buzzer_pwm, PWM, IMPL_STM32_PWM_CreateCtx(&htim12, TIM_CHANNEL_2),
                   IMPL_STM32_PWM_GetOps(), IMPL_STM32_PWM_DestroyCtx);

    /* --------------------------------------------------------------------- *
     * PB1 / TIM3_CH4, confirmed from the vendor's own example project, not our
     * schematic. The vendor's CtrBoard-H7_IMU_TempCtrl example
     * (gitee.com/kit-miao/dm-mc02) drives its BMI088 heater on this exact
     * pin/timer/channel, and our .ioc already configures PB1 as S_TIM3_CH4 PWM — so
     * this entry is a confirmed board fact, not the inference the buzzer entry above
     * still is.
     *
     * 172 Hz here, 1 kHz in the vendor example — deliberately not changed to match.
     * TIM3 is Prescaler=80-1 / Period=20000 over the 275 MHz APB1 kernel clock, i.e.
     * ~171.9 Hz and 20000 duty steps, where the vendor runs their timer at 1 kHz. A
     * heater is a resistive load with a sub-second thermal time constant, so a 5.8 ms
     * PWM period is invisible to it — nothing here calls for touching the CubeMX
     * timer configuration to chase the vendor's number.
     *
     * The control loop lives in 01_application/imu/app_imu.c, stepped from the same
     * task that already reads DEV_BMI088_GetTemperature, at that reading's own 10 Hz
     * refresh rate rather than the 1 kHz attitude-loop rate.
     * --------------------------------------------------------------------- */
    BOARD_BRING_UP(imu_heater, PWM, IMPL_STM32_PWM_CreateCtx(&htim3, TIM_CHANNEL_4),
                   IMPL_STM32_PWM_GetOps(), IMPL_STM32_PWM_DestroyCtx);

    /* --------------------------------------------------------------------- *
     * Parameter storage in the safe sector. Which sector that is comes from the
     * backend, not from here — it depends on the part's flash layout. It is also the
     * one value in this file that cannot be checked at runtime: every access is
     * bounds-tested against the region, so naming the wrong sector is what would let
     * a bad offset reach the firmware.
     * --------------------------------------------------------------------- */
    BOARD_BRING_UP(param_flash, Flash,
                   IMPL_STM32_FLASH_CreateCtx(IMPL_STM32_FLASH_PARAM_SECTOR, 1u),
                   IMPL_STM32_FLASH_GetOps(), IMPL_STM32_FLASH_DestroyCtx);

    /* ===== end EDIT HERE (3/4) =========================================== */

    return true;
}

#undef BOARD_BRING_UP

const char* Board_FailedDevice(void) { return s_failed; }

/* ========================================================================= */
/*  Accessors                                                                */
/* ========================================================================= */

/* ========================================================================= */
/*  CAN node factory                                                         */
/* ========================================================================= */

/* Everything above is brought up once, before Board_Init returns. This is the one
 * board facility that is not: a CAN node is created at runtime, as many times as the
 * robot has devices, by the application rather than by bring-up.
 *
 * FDCAN1 and FDCAN2 keep the names the application already uses. FDCAN3 (PD12/PD13)
 * exists on this part and is initialised by CubeMX, but it is deliberately not
 * listed: adding a bus here is what creates its selector, and an enumerator for a
 * connector nothing is plugged into only invites a node to be created on it.
 *
 * FDCAN2 receives on FIFO 1, and that is not a detail this file can hide. CubeMX gave
 * FDCAN1 and FDCAN3 eight elements in receive FIFO 0 and none in FIFO 1, and FDCAN2
 * the reverse. A FIFO with zero elements does not exist, so a filter routed to it
 * discards every match — the backend therefore picks whichever FIFO the peripheral
 * actually has rather than alternating between them as the bxCAN driver did. Nothing
 * here has to change for that; it is recorded because a future CubeMX edit that
 * zeroes both FIFOs on a bus would make every node on it fail to come up, and this is
 * where a reader would look. */

/**
 * @brief Shared body of both CAN factories.
 *
 * @param bus    Which peripheral.
 * @param tx_id  Transmit identifier.
 * @param first  First receive identifier claimed.
 * @param last   Last receive identifier claimed; equal to @p first for a single.
 * @return Vendor-neutral handle, or NULL on any refusal.
 */
static CAN_Instance_s* board_can_create(Board_CANBus_e bus, uint32_t tx_id, uint32_t first,
                                        uint32_t last)
{
    /* Indexed by the same enum board.h declares, and the _Static_assert below is what
     * keeps the two agreed now that no generator does it. Adding a bus is: one
     * enumerator in board.h, one row here. Miscount and the build stops. */
    static FDCAN_HandleTypeDef* const handle_of[] = {
        [BOARD_CAN1] = &hfdcan1,
        [BOARD_CAN2] = &hfdcan2,
    };

    _Static_assert((sizeof handle_of / sizeof handle_of[0]) == (size_t) BOARD_CAN_COUNT,
                   "handle_of and Board_CANBus_e must list the same buses");

    /* One unsigned compare covers both ends, including BOARD_CAN_COUNT itself — a
     * valid enum constant a caller could reach by mistake. */
    if ((unsigned) bus >= (sizeof handle_of / sizeof handle_of[0]))
    {
        return NULL;
    }

    void* ctx = (last > first) ? IMPL_STM32_CAN_CreateCtxRange(handle_of[bus], tx_id, first, last)
                               : IMPL_STM32_CAN_CreateCtx(handle_of[bus], tx_id, first);

    if (ctx == NULL)
    {
        return NULL;
    }

    /* Create, not Init: how many nodes exist is decided at runtime by the
     * application, so there is no fixed storage to hand in. */
    return PLAT_CAN_Create(IMPL_STM32_CAN_GetOps(), ctx);
}

CAN_Instance_s* Board_CANCreate(Board_CANBus_e bus, uint32_t tx_id, uint32_t rx_id)
{
    return board_can_create(bus, tx_id, rx_id, rx_id);
}

CAN_Instance_s* Board_CANCreateRange(Board_CANBus_e bus, uint32_t tx_id, uint32_t rx_id_first,
                                     uint32_t rx_id_last)
{
    return board_can_create(bus, tx_id, rx_id_first, rx_id_last);
}

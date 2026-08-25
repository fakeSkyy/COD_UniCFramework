/**
 * @file test_board_devices.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "unity.h"

#include <stdbool.h>
#include <stdint.h>

#include "board.h"
#include "case_runner.h"
#include "fdcan.h"
#include "impl_stm32_bind.h"
#include "main.h"
#include "mock_board_deps.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

enum
{
    DEVICE_TIMEBASE = 0,
    DEVICE_IMU_ACCEL,
    DEVICE_IMU_GYRO,
    DEVICE_STATUS_LED,
    DEVICE_DEBUG_UART,
    DEVICE_BUZZER_PWM,
    DEVICE_PARAM_FLASH,
    DEVICE_COUNT
};

static const char* const device_names[DEVICE_COUNT] = {
    "timebase", "imu_accel", "imu_gyro", "status_led", "debug_uart", "buzzer_pwm", "param_flash",
};

static unsigned       contexts[DEVICE_COUNT];
static DWT_Ops_s      dwt_ops;
static SPI_Ops_s      spi_ops;
static UART_Ops_s     uart_ops;
static PWM_Ops_s      pwm_ops;
static Flash_Ops_s    flash_ops;
static CAN_Ops_s      can_ops;
static CAN_Instance_s can_instances[2];

void setUp(void) { mock_board_deps_Init(); }

void tearDown(void)
{
    mock_board_deps_Verify();
    mock_board_deps_Destroy();
}

static void reset_mocks(void)
{
    mock_board_deps_Verify();
    mock_board_deps_Destroy();
    mock_board_deps_Init();
}

static void* accessor(unsigned device)
{
    switch (device)
    {
    case DEVICE_TIMEBASE:
        return Board_Timebase();
    case DEVICE_IMU_ACCEL:
        return Board_ImuAccel();
    case DEVICE_IMU_GYRO:
        return Board_ImuGyro();
    case DEVICE_STATUS_LED:
        return Board_StatusLed();
    case DEVICE_DEBUG_UART:
        return Board_DebugUart();
    case DEVICE_BUZZER_PWM:
        return Board_BuzzerPWM();
    case DEVICE_PARAM_FLASH:
        return Board_ParamFlash();
    default:
        return NULL;
    }
}

static void expect_device(unsigned device, bool ctx_ok, bool init_ok)
{
    void* const ctx = ctx_ok ? &contexts[device] : NULL;

    switch (device)
    {
    case DEVICE_TIMEBASE:
        IMPL_STM32_DWT_CreateCtx_ExpectAndReturn(SystemCoreClock, ctx);
        if (ctx_ok)
        {
            IMPL_STM32_DWT_GetOps_ExpectAndReturn(&dwt_ops);
            PLAT_DWT_Init_ExpectAnyArgsAndReturn(init_ok);
        }
        break;

    case DEVICE_IMU_ACCEL:
        IMPL_STM32_SPI_CreateCtx_ExpectAndReturn(&hspi2, ACCEL_CS_GPIO_Port, ACCEL_CS_Pin,
                                                 SPI_XFER_IT, ctx);
        if (ctx_ok)
        {
            IMPL_STM32_SPI_GetOps_ExpectAndReturn(&spi_ops);
            PLAT_SPI_Init_ExpectAnyArgsAndReturn(init_ok);
        }
        break;

    case DEVICE_IMU_GYRO:
        IMPL_STM32_SPI_CreateCtx_ExpectAndReturn(&hspi2, GYRO_CS_GPIO_Port, GYRO_CS_Pin,
                                                 SPI_XFER_IT, ctx);
        if (ctx_ok)
        {
            IMPL_STM32_SPI_GetOps_ExpectAndReturn(&spi_ops);
            PLAT_SPI_Init_ExpectAnyArgsAndReturn(init_ok);
        }
        break;

    case DEVICE_STATUS_LED:
        IMPL_STM32_SPI_CreateCtx_ExpectAndReturn(&hspi6, NULL, 0u, SPI_XFER_IT, ctx);
        if (ctx_ok)
        {
            IMPL_STM32_SPI_GetOps_ExpectAndReturn(&spi_ops);
            PLAT_SPI_Init_ExpectAnyArgsAndReturn(init_ok);
        }
        break;

    case DEVICE_DEBUG_UART:
        IMPL_STM32_UART_CreateCtx_ExpectAndReturn(&huart10, UART_XFER_IT, ctx);
        if (ctx_ok)
        {
            IMPL_STM32_UART_GetOps_ExpectAndReturn(&uart_ops);
            PLAT_UART_Init_ExpectAnyArgsAndReturn(init_ok);
        }
        break;

    case DEVICE_BUZZER_PWM:
        IMPL_STM32_PWM_CreateCtx_ExpectAndReturn(&htim12, TIM_CHANNEL_2, ctx);
        if (ctx_ok)
        {
            IMPL_STM32_PWM_GetOps_ExpectAndReturn(&pwm_ops);
            PLAT_PWM_Init_ExpectAnyArgsAndReturn(init_ok);
        }
        break;

    case DEVICE_PARAM_FLASH:
        IMPL_STM32_FLASH_CreateCtx_ExpectAndReturn(IMPL_FLASH_PARAM_SECTOR, 1u, ctx);
        if (ctx_ok)
        {
            IMPL_STM32_FLASH_GetOps_ExpectAndReturn(&flash_ops);
            PLAT_Flash_Init_ExpectAnyArgsAndReturn(init_ok);
        }
        break;

    default:
        TEST_FAIL_MESSAGE("unknown board device index");
        break;
    }
}

/**
 * @brief Expect the reverse-order teardown that opens every Board_Init call.
 *
 * Board_Init tears down before it builds, on every call including the first,
 * so every test that calls it must expect these before its bring-up
 * expectations — under enforce_strict_ordering, a mocked call that runs
 * before its expectation was set up fails the test rather than the call
 * simply going unverified. The teardown loop walks the device table
 * backwards, so the DestroyCtx calls are expected in reverse table order
 * regardless of which contexts are actually non-NULL: production code calls
 * IMPL_DESTROY_CTX unconditionally and leaves the NULL check to the backend.
 */
static void expect_teardown(void)
{
    for (unsigned i = DEVICE_COUNT; i > 0u; i--)
    {
        switch (i - 1u)
        {
        case DEVICE_TIMEBASE:
            IMPL_STM32_DWT_DestroyCtx_ExpectAnyArgs();
            break;
        case DEVICE_IMU_ACCEL:
        case DEVICE_IMU_GYRO:
        case DEVICE_STATUS_LED:
            IMPL_STM32_SPI_DestroyCtx_ExpectAnyArgs();
            break;
        case DEVICE_DEBUG_UART:
            IMPL_STM32_UART_DestroyCtx_ExpectAnyArgs();
            break;
        case DEVICE_BUZZER_PWM:
            IMPL_STM32_PWM_DestroyCtx_ExpectAnyArgs();
            break;
        case DEVICE_PARAM_FLASH:
            IMPL_STM32_FLASH_DestroyCtx_ExpectAnyArgs();
            break;
        default:
            TEST_FAIL_MESSAGE("unknown board device index");
            break;
        }
    }
}

static void expect_success(void)
{
    expect_teardown();
    for (unsigned i = 0u; i < DEVICE_COUNT; i++)
    {
        expect_device(i, true, true);
    }
}

static void expect_failure(unsigned failed_device, bool ctx_failure)
{
    expect_teardown();
    for (unsigned i = 0u; i <= failed_device; i++)
    {
        const bool at_failure = i == failed_device;
        expect_device(i, !at_failure || !ctx_failure, !at_failure);
    }
}

static void assert_accessors_around_failure(unsigned failed_device)
{
    for (unsigned i = 0u; i < DEVICE_COUNT; i++)
    {
        if (i < failed_device)
        {
            TEST_ASSERT_NOT_NULL(accessor(i));
        }
        else
        {
            TEST_ASSERT_NULL(accessor(i));
        }
    }
}

static void test_accessors_are_null_before_first_init(void)
{
    for (unsigned i = 0u; i < DEVICE_COUNT; i++)
    {
        TEST_ASSERT_NULL(accessor(i));
    }
    TEST_ASSERT_NULL(Board_FailedDevice());
}

static void test_seven_devices_initialize_in_strict_table_order(void)
{
    expect_success();
    TEST_ASSERT_TRUE(Board_Init());
    TEST_ASSERT_NULL(Board_FailedDevice());

    for (unsigned i = 0u; i < DEVICE_COUNT; i++)
    {
        TEST_ASSERT_NOT_NULL(accessor(i));
    }
}

static void test_each_null_context_is_first_failure_and_short_circuits(void)
{
    for (unsigned failed = 0u; failed < DEVICE_COUNT; failed++)
    {
        expect_failure(failed, true);
        TEST_ASSERT_FALSE(Board_Init());
        TEST_ASSERT_EQUAL_STRING(device_names[failed], Board_FailedDevice());
        assert_accessors_around_failure(failed);
        reset_mocks();
    }
}

static void test_each_platform_init_failure_is_first_and_short_circuits(void)
{
    for (unsigned failed = 0u; failed < DEVICE_COUNT; failed++)
    {
        expect_failure(failed, false);
        TEST_ASSERT_FALSE(Board_Init());
        TEST_ASSERT_EQUAL_STRING(device_names[failed], Board_FailedDevice());
        assert_accessors_around_failure(failed);
        reset_mocks();
    }
}

static void test_reinit_clears_failure_and_rebuilds_all_accessor_state(void)
{
    expect_success();
    TEST_ASSERT_TRUE(Board_Init());
    reset_mocks();

    expect_failure(DEVICE_STATUS_LED, true);
    TEST_ASSERT_FALSE(Board_Init());
    TEST_ASSERT_EQUAL_STRING("status_led", Board_FailedDevice());
    assert_accessors_around_failure(DEVICE_STATUS_LED);
    reset_mocks();

    expect_success();
    TEST_ASSERT_TRUE(Board_Init());
    TEST_ASSERT_NULL(Board_FailedDevice());
    for (unsigned i = 0u; i < DEVICE_COUNT; i++)
    {
        TEST_ASSERT_NOT_NULL(accessor(i));
    }
}

static void test_can_rejects_negative_and_count_without_backend_calls(void)
{
    TEST_ASSERT_NULL(Board_CANCreate((Board_CANBus_e) -1, 0x101u, 0x201u));
    TEST_ASSERT_NULL(Board_CANCreate(BOARD_CAN_COUNT, 0x102u, 0x202u));
}

static void test_can1_and_can2_forward_handle_and_ids(void)
{
    IMPL_STM32_CAN_CreateCtx_ExpectAndReturn(&hfdcan1, 0x111u, 0x211u, &contexts[0]);
    IMPL_STM32_CAN_GetOps_ExpectAndReturn(&can_ops);
    PLAT_CAN_Create_ExpectAndReturn(&can_ops, &contexts[0], &can_instances[0]);
    TEST_ASSERT_EQUAL_PTR(&can_instances[0], Board_CANCreate(BOARD_CAN1, 0x111u, 0x211u));

    IMPL_STM32_CAN_CreateCtx_ExpectAndReturn(&hfdcan2, 0x122u, 0x222u, &contexts[1]);
    IMPL_STM32_CAN_GetOps_ExpectAndReturn(&can_ops);
    PLAT_CAN_Create_ExpectAndReturn(&can_ops, &contexts[1], &can_instances[1]);
    TEST_ASSERT_EQUAL_PTR(&can_instances[1], Board_CANCreate(BOARD_CAN2, 0x122u, 0x222u));
}

static void test_can_null_context_short_circuits_before_ops_and_create(void)
{
    IMPL_STM32_CAN_CreateCtx_ExpectAndReturn(&hfdcan1, 0x123u, 0x321u, NULL);
    TEST_ASSERT_NULL(Board_CANCreate(BOARD_CAN1, 0x123u, 0x321u));
}

static void test_can_create_failure_is_returned(void)
{
    IMPL_STM32_CAN_CreateCtx_ExpectAndReturn(&hfdcan2, 0x124u, 0x421u, &contexts[0]);
    IMPL_STM32_CAN_GetOps_ExpectAndReturn(&can_ops);
    PLAT_CAN_Create_ExpectAndReturn(&can_ops, &contexts[0], NULL);
    TEST_ASSERT_NULL(Board_CANCreate(BOARD_CAN2, 0x124u, 0x421u));
}

static void test_reinit_destroys_each_previous_context_exactly_once(void)
{
    expect_success();
    TEST_ASSERT_TRUE(Board_Init());
    reset_mocks();

    /* Every context CreateCtx handed back on the first call must come back as
     * the exact pointer DestroyCtx is called with on the second — not just
     * "some non-NULL pointer" — so this uses the exact-argument expectation
     * rather than expect_teardown's _ExpectAnyArgs, and checks reverse table
     * order at the same time via enforce_strict_ordering. */
    IMPL_STM32_FLASH_DestroyCtx_Expect(&contexts[DEVICE_PARAM_FLASH]);
    IMPL_STM32_PWM_DestroyCtx_Expect(&contexts[DEVICE_BUZZER_PWM]);
    IMPL_STM32_UART_DestroyCtx_Expect(&contexts[DEVICE_DEBUG_UART]);
    IMPL_STM32_SPI_DestroyCtx_Expect(&contexts[DEVICE_STATUS_LED]);
    IMPL_STM32_SPI_DestroyCtx_Expect(&contexts[DEVICE_IMU_GYRO]);
    IMPL_STM32_SPI_DestroyCtx_Expect(&contexts[DEVICE_IMU_ACCEL]);
    IMPL_STM32_DWT_DestroyCtx_Expect(&contexts[DEVICE_TIMEBASE]);
    for (unsigned i = 0u; i < DEVICE_COUNT; i++)
    {
        expect_device(i, true, true);
    }
    TEST_ASSERT_TRUE(Board_Init());
}

static void test_midway_failure_leaves_contexts_that_the_next_call_still_releases(void)
{
    /* status_led (index 3) fails its PLAT_*_Init, so its context exists (ctx_ok)
     * but its _up flag never gets set — this is exactly the case the teardown
     * must cover by testing the stored context pointer rather than _up: an
     * entry can have a context with no _up, and the next call must still
     * release it. Devices after status_led (debug_uart, buzzer_pwm,
     * param_flash) never ran, so their contexts are NULL and their DestroyCtx
     * calls fire but touch nothing. */
    expect_failure(DEVICE_STATUS_LED, false);
    TEST_ASSERT_FALSE(Board_Init());
    TEST_ASSERT_EQUAL_STRING("status_led", Board_FailedDevice());
    reset_mocks();

    IMPL_STM32_FLASH_DestroyCtx_Expect(NULL);
    IMPL_STM32_PWM_DestroyCtx_Expect(NULL);
    IMPL_STM32_UART_DestroyCtx_Expect(NULL);
    IMPL_STM32_SPI_DestroyCtx_Expect(&contexts[DEVICE_STATUS_LED]);
    IMPL_STM32_SPI_DestroyCtx_Expect(&contexts[DEVICE_IMU_GYRO]);
    IMPL_STM32_SPI_DestroyCtx_Expect(&contexts[DEVICE_IMU_ACCEL]);
    IMPL_STM32_DWT_DestroyCtx_Expect(&contexts[DEVICE_TIMEBASE]);
    for (unsigned i = 0u; i < DEVICE_COUNT; i++)
    {
        expect_device(i, true, true);
    }
    TEST_ASSERT_TRUE(Board_Init());
    TEST_ASSERT_NULL(Board_FailedDevice());
    for (unsigned i = 0u; i < DEVICE_COUNT; i++)
    {
        TEST_ASSERT_NOT_NULL(accessor(i));
    }
}

static void test_third_consecutive_init_frees_nothing_twice(void)
{
    /* Two prior calls, each rebuilding cleanly, to reach a third with real
     * state behind it rather than starting from the all-NULL first-call case
     * every other test exercises. */
    expect_success();
    TEST_ASSERT_TRUE(Board_Init());
    reset_mocks();

    expect_success();
    TEST_ASSERT_TRUE(Board_Init());
    reset_mocks();

    /* The third call's teardown must destroy exactly what the second call's
     * bring-up created — the same exact-pointer check as
     * test_reinit_destroys_each_previous_context_exactly_once — and nothing
     * more: enforce_strict_ordering fails the test on any extra or
     * out-of-sequence mock call, which is what would happen if a stale
     * pointer from the first call had survived un-NULLed and been freed a
     * second time. */
    IMPL_STM32_FLASH_DestroyCtx_Expect(&contexts[DEVICE_PARAM_FLASH]);
    IMPL_STM32_PWM_DestroyCtx_Expect(&contexts[DEVICE_BUZZER_PWM]);
    IMPL_STM32_UART_DestroyCtx_Expect(&contexts[DEVICE_DEBUG_UART]);
    IMPL_STM32_SPI_DestroyCtx_Expect(&contexts[DEVICE_STATUS_LED]);
    IMPL_STM32_SPI_DestroyCtx_Expect(&contexts[DEVICE_IMU_GYRO]);
    IMPL_STM32_SPI_DestroyCtx_Expect(&contexts[DEVICE_IMU_ACCEL]);
    IMPL_STM32_DWT_DestroyCtx_Expect(&contexts[DEVICE_TIMEBASE]);
    for (unsigned i = 0u; i < DEVICE_COUNT; i++)
    {
        expect_device(i, true, true);
    }
    TEST_ASSERT_TRUE(Board_Init());
    TEST_ASSERT_NULL(Board_FailedDevice());
    for (unsigned i = 0u; i < DEVICE_COUNT; i++)
    {
        TEST_ASSERT_NOT_NULL(accessor(i));
    }
}

int main(int argc, char** argv)
{
    APP_CASES_BEGIN();
    APP_CASE(accessors_are_null_before_first_init);
    APP_CASE(seven_devices_initialize_in_strict_table_order);
    APP_CASE(each_null_context_is_first_failure_and_short_circuits);
    APP_CASE(each_platform_init_failure_is_first_and_short_circuits);
    APP_CASE(reinit_clears_failure_and_rebuilds_all_accessor_state);
    APP_CASE(can_rejects_negative_and_count_without_backend_calls);
    APP_CASE(can1_and_can2_forward_handle_and_ids);
    APP_CASE(can_null_context_short_circuits_before_ops_and_create);
    APP_CASE(can_create_failure_is_returned);
    APP_CASE(reinit_destroys_each_previous_context_exactly_once);
    APP_CASE(midway_failure_leaves_contexts_that_the_next_call_still_releases);
    APP_CASE(third_consecutive_init_frees_nothing_twice);
    APP_CASES_END();
}

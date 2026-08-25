/**
 * @file test_plat_pwm.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#define PLAT_ALLOW_CONSTRUCTION
#include "plat_pwm.h"

#include "platform_bsp_test_support.h"

static void*           backend_ctx = (void*) 0x9001u;
static const PWM_Ops_s pwm_ops     = {
        .start         = PBSP_PWM_Start,
        .stop          = PBSP_PWM_Stop,
        .set_compare   = PBSP_PWM_SetCompare,
        .get_period    = PBSP_PWM_GetPeriod,
        .set_frequency = PBSP_PWM_SetFrequency,
};

void setUp(void) { PlatformBsp_Test_MockInit(); }

void tearDown(void) { PlatformBsp_Test_MockVerify(); }

static void test_init_and_create_cover_allocator_paths(void)
{
    PWM_Instance_s storage = {0};

    TEST_ASSERT_FALSE(PLAT_PWM_Init(NULL, &pwm_ops, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_PWM_Init(&storage, NULL, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_PWM_Init(&storage, &pwm_ops, NULL));
    PLAT_malloc_ExpectAndReturn(sizeof(PWM_Instance_s), NULL);
    TEST_ASSERT_NULL(PLAT_PWM_Create(&pwm_ops, backend_ctx));
    PLAT_malloc_ExpectAndReturn(sizeof(PWM_Instance_s), &storage);
    PLAT_free_Expect(&storage);
    TEST_ASSERT_NULL(PLAT_PWM_Create(NULL, backend_ctx));
    PLAT_malloc_ExpectAndReturn(sizeof(PWM_Instance_s), &storage);
    TEST_ASSERT_EQUAL_PTR(&storage, PLAT_PWM_Create(&pwm_ops, backend_ctx));
    TEST_ASSERT_FALSE(storage.running);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, storage.duty_percent);
}

static void test_start_stop_are_idempotent_and_track_success(void)
{
    PWM_Instance_s pwm;

    TEST_ASSERT_TRUE(PLAT_PWM_Init(&pwm, &pwm_ops, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_PWM_IsRunning(&pwm));
    PBSP_PWM_Start_ExpectAndReturn(backend_ctx, false);
    TEST_ASSERT_FALSE(PLAT_PWM_Start(&pwm));
    TEST_ASSERT_FALSE(PLAT_PWM_IsRunning(&pwm));
    PBSP_PWM_Start_ExpectAndReturn(backend_ctx, true);
    TEST_ASSERT_TRUE(PLAT_PWM_Start(&pwm));
    TEST_ASSERT_TRUE(PLAT_PWM_Start(&pwm));
    PBSP_PWM_Stop_Expect(backend_ctx);
    PLAT_PWM_Stop(&pwm);
    PLAT_PWM_Stop(&pwm);
    TEST_ASSERT_FALSE(PLAT_PWM_IsRunning(&pwm));
}

static void test_duty_clamps_raw_and_percent_values(void)
{
    PWM_Instance_s pwm;

    TEST_ASSERT_TRUE(PLAT_PWM_Init(&pwm, &pwm_ops, backend_ctx));
    PBSP_PWM_GetPeriod_ExpectAndReturn(backend_ctx, 1000u);
    PBSP_PWM_SetCompare_Expect(backend_ctx, 1000u);
    PLAT_PWM_SetDuty(&pwm, 2000u);

    PBSP_PWM_GetPeriod_ExpectAndReturn(backend_ctx, 1000u);
    PBSP_PWM_SetCompare_Expect(backend_ctx, 0u);
    PLAT_PWM_SetDutyPercent(&pwm, -1.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pwm.duty_percent);
    PBSP_PWM_GetPeriod_ExpectAndReturn(backend_ctx, 1000u);
    PBSP_PWM_SetCompare_Expect(backend_ctx, 1000u);
    PLAT_PWM_SetDutyPercent(&pwm, 101.0f);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, pwm.duty_percent);
    PBSP_PWM_GetPeriod_ExpectAndReturn(backend_ctx, 1000u);
    PBSP_PWM_SetCompare_Expect(backend_ctx, 375u);
    PLAT_PWM_SetDutyPercent(&pwm, 37.5f);
}

static void test_frequency_failure_and_success_restore_duty(void)
{
    PWM_Instance_s pwm;

    TEST_ASSERT_TRUE(PLAT_PWM_Init(&pwm, &pwm_ops, backend_ctx));
    pwm.duty_percent = 25.0f;
    PBSP_PWM_SetFrequency_ExpectAndReturn(backend_ctx, 0u, 0u);
    TEST_ASSERT_FALSE(PLAT_PWM_SetFrequency(&pwm, 0u));
    PBSP_PWM_SetFrequency_ExpectAndReturn(backend_ctx, 20000u, 400u);
    PBSP_PWM_GetPeriod_ExpectAndReturn(backend_ctx, 400u);
    PBSP_PWM_SetCompare_Expect(backend_ctx, 100u);
    TEST_ASSERT_TRUE(PLAT_PWM_SetFrequency(&pwm, 20000u));

    PBSP_PWM_SetFrequency_ExpectAndReturn(backend_ctx, 5000u, 800u);
    PBSP_PWM_GetPeriod_ExpectAndReturn(backend_ctx, 800u);
    PBSP_PWM_SetCompare_Expect(backend_ctx, 800u);
    TEST_ASSERT_TRUE(PLAT_PWM_SetFreqAndDuty(&pwm, 5000u, 200.0f));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, pwm.duty_percent);
    PBSP_PWM_SetFrequency_ExpectAndReturn(backend_ctx, 1u, 0u);
    TEST_ASSERT_FALSE(PLAT_PWM_SetFreqAndDuty(&pwm, 1u, 50.0f));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, pwm.duty_percent);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_and_create_cover_allocator_paths);
    RUN_TEST(test_start_stop_are_idempotent_and_track_success);
    RUN_TEST(test_duty_clamps_raw_and_percent_values);
    RUN_TEST(test_frequency_failure_and_success_restore_duty);
    return UNITY_END();
}

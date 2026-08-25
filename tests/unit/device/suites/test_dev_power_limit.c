/**
 * @file test_dev_power_limit.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "unity.h"

#include <math.h>
#include <string.h>

#include "dev_power_limit.h"
#include "device_test_support.h"

static DEV_PowerLimit_Cfg_s cfg_for(unsigned count, float budget)
{
    DEV_PowerLimit_Cfg_s cfg = {0};
    cfg.motor_count          = (uint16_t) count;
    cfg.model.k_copper       = 2.0f;
    cfg.model.k_iron         = 0.5f;
    cfg.model.k_static       = 3.0f;
    cfg.power_budget         = budget;
    cfg.torque_per_output    = 0.1f;
    cfg.rate_per_velocity    = 0.2f;
    return cfg;
}

void setUp(void) { DEVICE_CMock_Init(); }

void tearDown(void)
{
    DEVICE_CMock_Verify();
    DEVICE_CMock_Destroy();
}

static void test_init_rejects_null_counts_and_nonphysical_values(void)
{
    DEV_PowerLimit_s     pl;
    DEV_PowerLimit_Cfg_s cfg = cfg_for(1u, 100.0f);
    TEST_ASSERT_FALSE(DEV_PowerLimit_Init(NULL, &cfg));
    TEST_ASSERT_FALSE(DEV_PowerLimit_Init(&pl, NULL));
    cfg.motor_count = 0u;
    TEST_ASSERT_FALSE(DEV_PowerLimit_Init(&pl, &cfg));
    cfg = cfg_for(9u, 100.0f);
    TEST_ASSERT_FALSE(DEV_PowerLimit_Init(&pl, &cfg));
    cfg = cfg_for(1u, 0.0f);
    TEST_ASSERT_FALSE(DEV_PowerLimit_Init(&pl, &cfg));
    cfg                   = cfg_for(1u, 100.0f);
    cfg.torque_per_output = NAN;
    TEST_ASSERT_FALSE(DEV_PowerLimit_Init(&pl, &cfg));
}

static void test_init_defaults_rls_and_bounds_and_model_setters_validate(void)
{
    DEV_PowerLimit_s     pl;
    DEV_PowerLimit_Cfg_s cfg = cfg_for(2u, 100.0f);
    TEST_ASSERT_TRUE(DEV_PowerLimit_Init(&pl, &cfg));
    TEST_ASSERT_TRUE(pl.rls_ready);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.5f, pl.cfg.k_copper_min);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 8.0f, pl.cfg.k_copper_max);
    DEV_PowerLimit_SetBudget(&pl, -1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, pl.cfg.power_budget);
    DEV_PowerLimit_SetBudget(&pl, 80.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 80.0f, pl.cfg.power_budget);
    DEV_Power_Model_s model = {4.0f, 1.0f, 2.0f};
    DEV_PowerLimit_SetModel(&pl, &model);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, DEV_PowerLimit_GetModel(&pl)->k_copper);
    model.k_copper = NAN;
    DEV_PowerLimit_SetModel(&pl, &model);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, DEV_PowerLimit_GetModel(&pl)->k_copper);
}

static void test_predict_matches_polynomial_and_sanitizes_nonfinite_entries(void)
{
    DEV_PowerLimit_s     pl;
    DEV_PowerLimit_Cfg_s cfg = cfg_for(2u, 1000.0f);
    TEST_ASSERT_TRUE(DEV_PowerLimit_Init(&pl, &cfg));
    const float          velocity[2] = {10.0f, NAN};
    const float          outputs[2]  = {5.0f, NAN};
    DEV_Power_Feedback_s fdb         = {velocity, NULL};
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.5f, DEV_PowerLimit_Predict(&pl, &fdb, outputs));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, DEV_PowerLimit_Predict(NULL, &fdb, outputs));
}

static void test_update_under_budget_passes_through_in_place(void)
{
    DEV_PowerLimit_s     pl;
    DEV_PowerLimit_Cfg_s cfg = cfg_for(2u, 1000.0f);
    TEST_ASSERT_TRUE(DEV_PowerLimit_Init(&pl, &cfg));
    float                velocity[2] = {1.0f, 2.0f};
    float                request[2]  = {10.0f, -20.0f};
    DEV_Power_Feedback_s fdb         = {velocity, NULL};
    TEST_ASSERT_TRUE(DEV_PowerLimit_Update(&pl, &fdb, request, request));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 10.0f, request[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -20.0f, request[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, DEV_PowerLimit_GetScale(&pl));
}

static void test_update_over_budget_applies_one_finite_common_scale(void)
{
    DEV_PowerLimit_s     pl;
    DEV_PowerLimit_Cfg_s cfg = cfg_for(2u, 20.0f);
    TEST_ASSERT_TRUE(DEV_PowerLimit_Init(&pl, &cfg));
    float                velocity[2] = {0.0f, 0.0f};
    float                request[2]  = {30.0f, -60.0f};
    float                output[2];
    DEV_Power_Feedback_s fdb = {velocity, NULL};
    TEST_ASSERT_TRUE(DEV_PowerLimit_Update(&pl, &fdb, request, output));
    TEST_ASSERT_TRUE(pl.scale > 0.0f && pl.scale < 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, output[0] * -2.0f, output[1]);
    TEST_ASSERT_TRUE(pl.limited_power <= cfg.power_budget + 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, DEV_PowerLimit_GetLimitRatio(&pl));
}

static void test_coast_over_budget_zeroes_and_nonfinite_request_fails_safe(void)
{
    DEV_PowerLimit_s     pl;
    DEV_PowerLimit_Cfg_s cfg = cfg_for(2u, 4.0f);
    TEST_ASSERT_TRUE(DEV_PowerLimit_Init(&pl, &cfg));
    float                velocity[2] = {10.0f, 10.0f};
    float                request[2]  = {1.0f, 2.0f};
    float                output[2]   = {9.0f, 9.0f};
    DEV_Power_Feedback_s fdb         = {velocity, NULL};
    TEST_ASSERT_TRUE(DEV_PowerLimit_Update(&pl, &fdb, request, output));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output[1]);
    request[0] = NAN;
    TEST_ASSERT_FALSE(DEV_PowerLimit_Update(&pl, &fdb, request, output));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output[1]);
}

static void test_feed_measured_requires_update_and_keeps_model_finite_bounded(void)
{
    DEV_PowerLimit_s     pl;
    DEV_PowerLimit_Cfg_s cfg = cfg_for(2u, 100.0f);
    TEST_ASSERT_TRUE(DEV_PowerLimit_Init(&pl, &cfg));
    TEST_ASSERT_FALSE(DEV_PowerLimit_FeedMeasured(&pl, 10.0f));
    float                velocity[2] = {5.0f, -5.0f};
    float                current[2]  = {3.0f, 4.0f};
    float                request[2]  = {2.0f, 2.0f};
    float                output[2];
    DEV_Power_Feedback_s fdb = {velocity, current};
    TEST_ASSERT_TRUE(DEV_PowerLimit_Update(&pl, &fdb, request, output));
    TEST_ASSERT_FALSE(DEV_PowerLimit_FeedMeasured(&pl, NAN));
    TEST_ASSERT_TRUE(DEV_PowerLimit_FeedMeasured(&pl, 12.0f));
    TEST_ASSERT_TRUE(isfinite(pl.model.k_copper));
    TEST_ASSERT_TRUE(pl.model.k_copper >= pl.cfg.k_copper_min);
    TEST_ASSERT_TRUE(pl.model.k_copper <= pl.cfg.k_copper_max);
    TEST_ASSERT_TRUE(pl.model.k_iron >= 0.0f);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_rejects_null_counts_and_nonphysical_values);
    RUN_TEST(test_init_defaults_rls_and_bounds_and_model_setters_validate);
    RUN_TEST(test_predict_matches_polynomial_and_sanitizes_nonfinite_entries);
    RUN_TEST(test_update_under_budget_passes_through_in_place);
    RUN_TEST(test_update_over_budget_applies_one_finite_common_scale);
    RUN_TEST(test_coast_over_budget_zeroes_and_nonfinite_request_fails_safe);
    RUN_TEST(test_feed_measured_requires_update_and_keeps_model_finite_bounded);
    return UNITY_END();
}

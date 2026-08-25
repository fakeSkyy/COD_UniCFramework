/**
 * @file test_dev_motor_pid.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "unity.h"

#include <math.h>

#include "dev_motor_pid.h"
#include "device_test_support.h"

void setUp(void) { DEVICE_CMock_Init(); }

void tearDown(void)
{
    DEVICE_CMock_Verify();
    DEVICE_CMock_Destroy();
}

static UTIL_PID_Cfg_s p_cfg(float kp)
{
    UTIL_PID_Cfg_s cfg = {0};
    cfg.kp             = kp;
    return cfg;
}

static void test_init_rejects_null_and_leaves_rejected_instance_unbound(void)
{
    DEV_MotorPID_s ctrl;
    UTIL_PID_Cfg_s cfg = p_cfg(2.0f);
    TEST_ASSERT_FALSE(DEV_MotorPID_Init(NULL, &cfg, UTIL_PID_POSITION));
    TEST_ASSERT_FALSE(DEV_MotorPID_Init(&ctrl, NULL, UTIL_PID_POSITION));
    TEST_ASSERT_NULL(ctrl.dji.step);
    TEST_ASSERT_NULL(ctrl.dm.step);
    TEST_ASSERT_NULL(DEV_MotorPID_AsDJI(NULL));
    TEST_ASSERT_NULL(DEV_MotorPID_AsDM(NULL));
}

static void test_single_loop_runs_through_both_vtable_offsets(void)
{
    DEV_MotorPID_s ctrl;
    UTIL_PID_Cfg_s cfg = p_cfg(2.0f);
    TEST_ASSERT_TRUE(DEV_MotorPID_Init(&ctrl, &cfg, UTIL_PID_POSITION));
    DEV_DJI_Controller_s* dji = DEV_MotorPID_AsDJI(&ctrl);
    DEV_DM_Controller_s*  dm  = DEV_MotorPID_AsDM(&ctrl);
    TEST_ASSERT_EQUAL_PTR(&ctrl, dji->data);
    TEST_ASSERT_EQUAL_PTR(&ctrl, dm->data);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.0f, dji->step(dji, 5.0f, 2.0f, 0.01f));
    dm->reset(dm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 8.0f, dm->step(dm, 5.0f, 1.0f, 0.01f));
}

static void test_invalid_pid_field_reports_unclean_but_binds_safe_controller(void)
{
    DEV_MotorPID_s ctrl;
    UTIL_PID_Cfg_s cfg = p_cfg(NAN);
    TEST_ASSERT_FALSE(DEV_MotorPID_Init(&ctrl, &cfg, UTIL_PID_POSITION));
    TEST_ASSERT_TRUE(ctrl.initialized);
    TEST_ASSERT_NOT_NULL(ctrl.dji.step);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, ctrl.dji.step(&ctrl.dji, 5.0f, 1.0f, 0.01f));
}

static void test_cascade_rejects_each_missing_argument(void)
{
    DEV_MotorPID_s ctrl;
    UTIL_PID_Cfg_s outer   = p_cfg(2.0f);
    UTIL_PID_Cfg_s inner   = p_cfg(3.0f);
    float          measure = 0.0f;
    TEST_ASSERT_FALSE(DEV_MotorPID_InitCascade(NULL, &outer, &inner, &measure));
    TEST_ASSERT_FALSE(DEV_MotorPID_InitCascade(&ctrl, NULL, &inner, &measure));
    TEST_ASSERT_FALSE(DEV_MotorPID_InitCascade(&ctrl, &outer, NULL, &measure));
    TEST_ASSERT_FALSE(DEV_MotorPID_InitCascade(&ctrl, &outer, &inner, NULL));
}

static void test_cascade_uses_inner_measurement_and_exposes_inner_target(void)
{
    DEV_MotorPID_s ctrl;
    UTIL_PID_Cfg_s outer      = p_cfg(2.0f);
    UTIL_PID_Cfg_s inner      = p_cfg(3.0f);
    float          inner_meas = 1.0f;
    TEST_ASSERT_TRUE(DEV_MotorPID_InitCascade(&ctrl, &outer, &inner, &inner_meas));
    float out = ctrl.dji.step(&ctrl.dji, 5.0f, 2.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.0f, DEV_MotorPID_GetInnerTarget(&ctrl));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 15.0f, out);
    inner_meas = 4.0f;
    out        = ctrl.dm.step(&ctrl.dm, 5.0f, 2.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.0f, out);
}

static void test_reset_clears_single_and_cascade_state(void)
{
    DEV_MotorPID_s ctrl;
    UTIL_PID_Cfg_s outer   = p_cfg(1.0f);
    outer.ki               = 1.0f;
    UTIL_PID_Cfg_s inner   = p_cfg(1.0f);
    float          measure = 0.0f;
    TEST_ASSERT_TRUE(DEV_MotorPID_InitCascade(&ctrl, &outer, &inner, &measure));
    (void) ctrl.dji.step(&ctrl.dji, 2.0f, 0.0f, 0.5f);
    TEST_ASSERT_TRUE(DEV_MotorPID_GetInnerTarget(&ctrl) > 0.0f);
    ctrl.dji.reset(&ctrl.dji);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, DEV_MotorPID_GetInnerTarget(&ctrl));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_PID_Get(&ctrl.outer));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, UTIL_PID_Get(&ctrl.inner));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_rejects_null_and_leaves_rejected_instance_unbound);
    RUN_TEST(test_single_loop_runs_through_both_vtable_offsets);
    RUN_TEST(test_invalid_pid_field_reports_unclean_but_binds_safe_controller);
    RUN_TEST(test_cascade_rejects_each_missing_argument);
    RUN_TEST(test_cascade_uses_inner_measurement_and_exposes_inner_target);
    RUN_TEST(test_reset_clears_single_and_cascade_state);
    return UNITY_END();
}

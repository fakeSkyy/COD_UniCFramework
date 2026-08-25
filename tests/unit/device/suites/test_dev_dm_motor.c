/**
 * @file test_dev_dm_motor.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "unity.h"

#include <math.h>
#include <string.h>

#include "dev_dm_motor.h"
#include "device_test_support.h"

static uint8_t  can_payload[8];
static uint32_t can_id;
static bool     can_result;
static unsigned step_calls;
static unsigned reset_calls;

static bool capture_can(CAN_Instance_s* can, uint32_t id, const uint8_t* data, uint8_t len,
                        int calls)
{
    (void) can;
    (void) calls;
    can_id = id;
    TEST_ASSERT_EQUAL_UINT8(8u, len);
    memcpy(can_payload, data, len);
    return can_result;
}

static float controller_step(DEV_DM_Controller_s* self, float target, float meas, float dt)
{
    (void) self;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, target);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, meas);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.01f, dt);
    step_calls++;
    return 20.0f;
}

static void controller_reset(DEV_DM_Controller_s* self)
{
    (void) self;
    reset_calls++;
}

static DEV_DM_Cfg_s base_cfg(void)
{
    DEV_DM_Cfg_s cfg = {0};
    cfg.tx_id        = 0x141u;
    cfg.rx_id        = 0x241u;
    return cfg;
}

static void init_motor(DEV_DM_Motor_s* motor, CAN_Instance_s* can, DEV_DM_Controller_s* ctrl)
{
    DEV_DM_Cfg_s cfg = base_cfg();
    TEST_ASSERT_TRUE(DEV_DMMotor_Init(motor, can, &cfg, ctrl));
}

void setUp(void)
{
    DEVICE_CMock_Init();
    memset(can_payload, 0, sizeof(can_payload));
    can_id      = 0u;
    can_result  = true;
    step_calls  = 0u;
    reset_calls = 0u;
}

void tearDown(void)
{
    DEVICE_CMock_Verify();
    DEVICE_CMock_Destroy();
}

static void test_init_validates_identifiers_and_defaults_ranges(void)
{
    DEV_DM_Motor_s motor;
    CAN_Instance_s can = {0};
    DEV_DM_Cfg_s   cfg = base_cfg();
    TEST_ASSERT_FALSE(DEV_DMMotor_Init(NULL, &can, &cfg, NULL));
    TEST_ASSERT_FALSE(DEV_DMMotor_Init(&motor, NULL, &cfg, NULL));
    cfg.rx_id = cfg.tx_id;
    TEST_ASSERT_FALSE(DEV_DMMotor_Init(&motor, &can, &cfg, NULL));
    cfg = base_cfg();
    TEST_ASSERT_TRUE(DEV_DMMotor_Init(&motor, &can, &cfg, NULL));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, DEV_DM_DEFAULT_P_MAX, motor.cfg.p_max);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, DEV_DM_DEFAULT_T_MAX, motor.cfg.t_max);
    TEST_ASSERT_FALSE(motor.enabled);
    TEST_ASSERT_TRUE(motor.initialized);
    cfg.p_max = -1.0f;
    TEST_ASSERT_FALSE(DEV_DMMotor_Init(&motor, &can, &cfg, NULL));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, DEV_DM_DEFAULT_P_MAX, motor.cfg.p_max);
}

static void test_one_shot_commands_encode_exact_bytes_and_failures_count(void)
{
    DEV_DM_Motor_s motor;
    CAN_Instance_s can = {0};
    init_motor(&motor, &can, NULL);
    PLAT_CAN_SendTo_StubWithCallback(capture_can);
    const uint8_t commands[] = {0xFCu, 0xFEu, 0xFBu, 0xFDu};
    TEST_ASSERT_TRUE(DEV_DMMotor_Enable(&motor));
    TEST_ASSERT_TRUE(motor.enabled);
    TEST_ASSERT_EQUAL_HEX8(commands[0], can_payload[7]);
    TEST_ASSERT_TRUE(DEV_DMMotor_SetZero(&motor));
    TEST_ASSERT_EQUAL_HEX8(commands[1], can_payload[7]);
    TEST_ASSERT_TRUE(DEV_DMMotor_ClearFault(&motor));
    TEST_ASSERT_EQUAL_HEX8(commands[2], can_payload[7]);
    TEST_ASSERT_TRUE(DEV_DMMotor_Disable(&motor));
    TEST_ASSERT_FALSE(motor.enabled);
    TEST_ASSERT_EQUAL_HEX8(commands[3], can_payload[7]);
    for (unsigned i = 0u; i < 7u; i++)
    {
        TEST_ASSERT_EQUAL_HEX8(0xFFu, can_payload[i]);
    }
    can_result = false;
    TEST_ASSERT_FALSE(DEV_DMMotor_Enable(&motor));
    TEST_ASSERT_EQUAL_UINT32(1u, motor.tx_fail);
}

static void test_commit_zero_and_saturated_mit_have_golden_wire_images(void)
{
    DEV_DM_Motor_s motor;
    CAN_Instance_s can = {0};
    init_motor(&motor, &can, NULL);
    PLAT_CAN_SendTo_StubWithCallback(capture_can);
    TEST_ASSERT_TRUE(DEV_DMMotor_Commit(&motor, 0.01f));
    const uint8_t zero[8] = {0x80u, 0x00u, 0x80u, 0x00u, 0x00u, 0x00u, 0x08u, 0x00u};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(zero, can_payload, 8u);
    DEV_DMMotor_SetMIT(&motor, 1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f);
    TEST_ASSERT_TRUE(DEV_DMMotor_Commit(&motor, 0.01f));
    const uint8_t maxed[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(maxed, can_payload, 8u);
    TEST_ASSERT_EQUAL_HEX32(0x141u, can_id);
}

static void test_nonfinite_mit_maps_to_safe_zero_fields(void)
{
    DEV_DM_Motor_s motor;
    CAN_Instance_s can = {0};
    init_motor(&motor, &can, NULL);
    PLAT_CAN_SendTo_StubWithCallback(capture_can);
    DEV_DMMotor_SetMIT(&motor, NAN, NAN, NAN, NAN, NAN);
    TEST_ASSERT_TRUE(DEV_DMMotor_Commit(&motor, 0.01f));
    const uint8_t zero[8] = {0x80u, 0x00u, 0x80u, 0x00u, 0x00u, 0x00u, 0x08u, 0x00u};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(zero, can_payload, 8u);
}

static void test_controller_runs_only_enabled_with_valid_dt_and_saturates_torque(void)
{
    DEV_DM_Controller_s ctrl = {controller_step, controller_reset, NULL};
    DEV_DM_Motor_s      motor;
    CAN_Instance_s      can = {0};
    init_motor(&motor, &can, &ctrl);
    PLAT_CAN_SendTo_StubWithCallback(capture_can);
    DEV_DMMotor_SetTarget(&motor, 4.0f);
    float external = 1.0f;
    DEV_DMMotor_SetMeasurementSource(&motor, &external);
    TEST_ASSERT_TRUE(DEV_DMMotor_Commit(&motor, 0.01f));
    TEST_ASSERT_EQUAL_UINT(0u, step_calls);
    TEST_ASSERT_TRUE(DEV_DMMotor_Enable(&motor));
    TEST_ASSERT_TRUE(DEV_DMMotor_Commit(&motor, 0.01f));
    TEST_ASSERT_EQUAL_UINT(1u, step_calls);
    TEST_ASSERT_EQUAL_HEX8(0xFFu, can_payload[7]);
    TEST_ASSERT_TRUE(DEV_DMMotor_Commit(&motor, NAN));
    TEST_ASSERT_EQUAL_UINT(1u, step_calls);
    TEST_ASSERT_TRUE(DEV_DMMotor_Disable(&motor));
    TEST_ASSERT_EQUAL_UINT(1u, reset_calls);
}

static void test_feedback_decodes_fault_ranges_temperatures_reverse_and_watchdog(void)
{
    DEV_DM_Motor_s motor;
    CAN_Instance_s can = {0};
    DEV_DM_Cfg_s   cfg = base_cfg();
    cfg.gear_ratio     = 2.0f;
    cfg.reverse        = true;
    TEST_ASSERT_TRUE(DEV_DMMotor_Init(&motor, &can, &cfg, NULL));
    const uint8_t frame[8] = {0xA3u, 0x80u, 0x00u, 0x80u, 0x08u, 0x00u, 50u, 60u};
    TEST_ASSERT_FALSE(DEV_DMMotor_OnFeedback(NULL, frame, 8u, 10u));
    TEST_ASSERT_FALSE(DEV_DMMotor_OnFeedback(&motor, frame, 7u, 10u));
    TEST_ASSERT_TRUE(DEV_DMMotor_OnFeedback(&motor, frame, 8u, 10u));
    TEST_ASSERT_EQUAL_UINT8(3u, motor.fdb.motor_id);
    TEST_ASSERT_EQUAL(DEV_DM_ERR_OVERCURRENT, DEV_DMMotor_GetFault(&motor));
    TEST_ASSERT_TRUE(DEV_DMMotor_HasFault(&motor));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, motor.fdb.position_rad);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, motor.fdb.velocity_rps);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, motor.fdb.torque_nm);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, motor.fdb.mos_temp_c);
    TEST_ASSERT_EQUAL_UINT32(10u, motor.wd.last_kick_ms);
}

static void test_offline_boundary_and_counter_wrap_are_unsigned_safe(void)
{
    DEV_DM_Motor_s motor;
    CAN_Instance_s can = {0};
    init_motor(&motor, &can, NULL);
    TEST_ASSERT_TRUE(DEV_DMMotor_IsOffline(&motor, 10u, 100u));
    const uint8_t frame[8] = {0u, 0x80u, 0u, 0x80u, 0u, 0u, 0u, 0u};
    TEST_ASSERT_TRUE(DEV_DMMotor_OnFeedback(&motor, frame, 8u, UINT32_MAX - 5u));
    TEST_ASSERT_FALSE(DEV_DMMotor_IsOffline(&motor, UINT32_MAX, 5u));
    TEST_ASSERT_FALSE(DEV_DMMotor_IsOffline(&motor, 4u, 10u));
    TEST_ASSERT_TRUE(DEV_DMMotor_IsOffline(&motor, 5u, 10u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_validates_identifiers_and_defaults_ranges);
    RUN_TEST(test_one_shot_commands_encode_exact_bytes_and_failures_count);
    RUN_TEST(test_commit_zero_and_saturated_mit_have_golden_wire_images);
    RUN_TEST(test_nonfinite_mit_maps_to_safe_zero_fields);
    RUN_TEST(test_controller_runs_only_enabled_with_valid_dt_and_saturates_torque);
    RUN_TEST(test_feedback_decodes_fault_ranges_temperatures_reverse_and_watchdog);
    RUN_TEST(test_offline_boundary_and_counter_wrap_are_unsigned_safe);
    return UNITY_END();
}

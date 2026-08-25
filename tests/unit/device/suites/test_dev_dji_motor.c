/**
 * @file test_dev_dji_motor.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "unity.h"

#include <math.h>
#include <string.h>

#include "dev_dji_motor.h"
#include "device_test_support.h"

static uint8_t  can_payload[8];
static uint32_t can_id;
static bool     can_result;
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

static float step_ctrl(DEV_DJI_Controller_s* self, float target, float meas, float dt)
{
    (void) self;
    (void) target;
    (void) meas;
    (void) dt;
    return 20000.0f;
}

static void reset_ctrl(DEV_DJI_Controller_s* self)
{
    (void) self;
    reset_calls++;
}

void setUp(void)
{
    DEVICE_CMock_Init();
    memset(can_payload, 0, sizeof(can_payload));
    can_id      = 0u;
    can_result  = true;
    reset_calls = 0u;
}

void tearDown(void)
{
    DEVICE_CMock_Verify();
    DEVICE_CMock_Destroy();
}

static void test_protocol_id_tables_cover_types_boundaries_and_invalid(void)
{
    TEST_ASSERT_EQUAL_HEX32(0x200u, DEV_DJIMotor_ControlIdFor(DEV_DJI_M3508, 1u));
    TEST_ASSERT_EQUAL_HEX32(0x1FFu, DEV_DJIMotor_ControlIdFor(DEV_DJI_M2006, 8u));
    TEST_ASSERT_EQUAL_HEX32(0x1FFu, DEV_DJIMotor_ControlIdFor(DEV_DJI_GM6020, 1u));
    TEST_ASSERT_EQUAL_HEX32(0x2FFu, DEV_DJIMotor_ControlIdFor(DEV_DJI_GM6020, 7u));
    TEST_ASSERT_EQUAL_UINT32(0u, DEV_DJIMotor_ControlIdFor(DEV_DJI_GM6020, 8u));
    TEST_ASSERT_EQUAL_UINT32(0u, DEV_DJIMotor_ControlIdFor(DEV_DJI_M3508, 0u));
    TEST_ASSERT_EQUAL_HEX32(0x201u, DEV_DJIMotor_FeedbackIdFor(DEV_DJI_M3508, 1u));
    TEST_ASSERT_EQUAL_HEX32(0x205u, DEV_DJIMotor_FeedbackIdFor(DEV_DJI_GM6020, 1u));
}

static void test_bus_and_attach_validate_frame_slot_and_collision(void)
{
    CAN_Instance_s  can = {0};
    DEV_DJI_Bus_s   bus;
    DEV_DJI_Motor_s first;
    DEV_DJI_Motor_s collision;
    TEST_ASSERT_FALSE(DEV_DJIMotor_BusInit(NULL, &can, 0x200u));
    TEST_ASSERT_FALSE(DEV_DJIMotor_BusInit(&bus, NULL, 0x200u));
    TEST_ASSERT_FALSE(DEV_DJIMotor_BusInit(&bus, &can, 0x123u));
    TEST_ASSERT_TRUE(DEV_DJIMotor_BusInit(&bus, &can, 0x200u));
    TEST_ASSERT_TRUE(DEV_DJIMotor_Attach(&first, &bus, DEV_DJI_M3508, 1u, NULL));
    TEST_ASSERT_FALSE(DEV_DJIMotor_Attach(&collision, &bus, DEV_DJI_M2006, 1u, NULL));
    TEST_ASSERT_FALSE(DEV_DJIMotor_Attach(&collision, &bus, DEV_DJI_GM6020, 1u, NULL));
    TEST_ASSERT_EQUAL_HEX32(0x201u, DEV_DJIMotor_FeedbackId(&first));
    TEST_ASSERT_TRUE(first.enabled);
    TEST_ASSERT_EQUAL_UINT32(100u, first.wd.timeout_ms);
}

static void test_feedback_decodes_big_endian_and_unwraps_both_directions(void)
{
    CAN_Instance_s  can = {0};
    DEV_DJI_Bus_s   bus;
    DEV_DJI_Motor_s motor;
    TEST_ASSERT_TRUE(DEV_DJIMotor_BusInit(&bus, &can, 0x1FFu));
    TEST_ASSERT_TRUE(DEV_DJIMotor_Attach(&motor, &bus, DEV_DJI_GM6020, 1u, NULL));
    const uint8_t high[8] = {0x1Fu, 0xFFu, 0xFFu, 0x9Cu, 0x12u, 0x34u, 55u, 0u};
    const uint8_t low[8]  = {0u, 0u, 0u, 100u, 0xFFu, 0x9Cu, 56u, 0u};
    TEST_ASSERT_TRUE(DEV_DJIMotor_OnFeedback(&motor, high, 8u, 100u));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -100.0f, motor.fdb.rpm);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 4660.0f, motor.fdb.torque_current);
    TEST_ASSERT_TRUE(DEV_DJIMotor_OnFeedback(&motor, low, 8u, 101u));
    TEST_ASSERT_TRUE(motor.fdb.angle_total_deg > 359.0f);
    TEST_ASSERT_EQUAL_UINT32(2u, motor.fdb.frame_count);
    TEST_ASSERT_EQUAL_UINT32(101u, motor.wd.last_kick_ms);
    TEST_ASSERT_FALSE(DEV_DJIMotor_IsOffline(&motor, 201u, 100u));
    TEST_ASSERT_TRUE(DEV_DJIMotor_IsOffline(&motor, 202u, 100u));
}

static void test_feedback_rejects_short_and_invalid_encoder_without_commit(void)
{
    CAN_Instance_s  can = {0};
    DEV_DJI_Bus_s   bus;
    DEV_DJI_Motor_s motor;
    TEST_ASSERT_TRUE(DEV_DJIMotor_BusInit(&bus, &can, 0x200u));
    TEST_ASSERT_TRUE(DEV_DJIMotor_Attach(&motor, &bus, DEV_DJI_M3508, 1u, NULL));
    const uint8_t invalid[8] = {0x20u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    TEST_ASSERT_FALSE(DEV_DJIMotor_OnFeedback(&motor, invalid, 7u, 1u));
    TEST_ASSERT_FALSE(DEV_DJIMotor_OnFeedback(&motor, invalid, 8u, 1u));
    TEST_ASSERT_EQUAL_UINT32(0u, motor.fdb.frame_count);
}

static void test_commit_packs_four_slots_clamps_and_tracks_send_failure(void)
{
    CAN_Instance_s  can = {0};
    DEV_DJI_Bus_s   bus;
    DEV_DJI_Motor_s motors[4];
    TEST_ASSERT_TRUE(DEV_DJIMotor_BusInit(&bus, &can, 0x200u));
    for (uint8_t i = 0u; i < 4u; i++)
    {
        TEST_ASSERT_TRUE(DEV_DJIMotor_Attach(&motors[i], &bus, DEV_DJI_M3508, i + 1u, NULL));
    }
    DEV_DJIMotor_SetOutput(&motors[0], 1.0f);
    DEV_DJIMotor_SetOutput(&motors[1], -2.0f);
    DEV_DJIMotor_SetOutput(&motors[2], 50000.0f);
    DEV_DJIMotor_SetEnabled(&motors[3], false);
    PLAT_CAN_SendTo_StubWithCallback(capture_can);
    TEST_ASSERT_TRUE(DEV_DJIMotor_CommitBus(&bus, 0.001f));
    const uint8_t expected[8] = {0u, 1u, 0xFFu, 0xFEu, 0x40u, 0u, 0u, 0u};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, can_payload, 8u);
    TEST_ASSERT_EQUAL_HEX32(0x200u, can_id);
    can_result = false;
    TEST_ASSERT_FALSE(DEV_DJIMotor_CommitBus(&bus, 0.001f));
    TEST_ASSERT_EQUAL_UINT32(1u, bus.tx_fail);
}

static void test_controller_measurement_nonfinite_reverse_and_disable_reset(void)
{
    DEV_DJI_Controller_s ctrl = {step_ctrl, reset_ctrl, NULL};
    CAN_Instance_s       can  = {0};
    DEV_DJI_Bus_s        bus;
    DEV_DJI_Motor_s      motor;
    TEST_ASSERT_TRUE(DEV_DJIMotor_BusInit(&bus, &can, 0x200u));
    TEST_ASSERT_TRUE(DEV_DJIMotor_Attach(&motor, &bus, DEV_DJI_M3508, 1u, &ctrl));
    PLAT_CAN_SendTo_StubWithCallback(capture_can);
    TEST_ASSERT_TRUE(DEV_DJIMotor_CommitBus(&bus, 0.01f));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 16384.0f, motor.output);
    float bad = NAN;
    DEV_DJIMotor_SetMeasurementSource(&motor, &bad);
    TEST_ASSERT_TRUE(DEV_DJIMotor_CommitBus(&bus, 0.01f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, motor.output);
    motor.fdb.angle_deg       = 10.0f;
    motor.fdb.angle_total_deg = 20.0f;
    motor.fdb.rpm             = 30.0f;
    DEV_DJIMotor_SetReverse(&motor, true);
    TEST_ASSERT_EQUAL_FLOAT(-10.0f, motor.fdb.angle_deg);
    TEST_ASSERT_EQUAL_FLOAT(-30.0f, motor.fdb.rpm);
    DEV_DJIMotor_SetEnabled(&motor, false);
    TEST_ASSERT_EQUAL_UINT(1u, reset_calls);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_protocol_id_tables_cover_types_boundaries_and_invalid);
    RUN_TEST(test_bus_and_attach_validate_frame_slot_and_collision);
    RUN_TEST(test_feedback_decodes_big_endian_and_unwraps_both_directions);
    RUN_TEST(test_feedback_rejects_short_and_invalid_encoder_without_commit);
    RUN_TEST(test_commit_packs_four_slots_clamps_and_tracks_send_failure);
    RUN_TEST(test_controller_measurement_nonfinite_reverse_and_disable_reset);
    return UNITY_END();
}

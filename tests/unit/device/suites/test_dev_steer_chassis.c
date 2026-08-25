/**
 * @file test_dev_steer_chassis.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "unity.h"

#include <stdlib.h>

#include "support/alloc/host_alloc_tracker.h"

#include "dev_steer_chassis.h"
#include "device_test_support.h"

static void* host_alloc(size_t size, int calls)
{
    (void) calls;
    return TEST_TrackedMalloc(size);
}

static DEV_Steer_Config_s valid_cfg(void)
{
    DEV_Steer_Config_s cfg = {100.0f, 400.0f, 300.0f, 2.0f, 1000.0f, 1000.0f, 4.0f};
    return cfg;
}

static DEV_SteerChassis_s* create_chassis(const float* offsets)
{
    PLAT_malloc_StubWithCallback(host_alloc);
    DEV_Steer_Config_s cfg = valid_cfg();
    return DEV_SteerChassis_Create(&cfg, offsets);
}

void setUp(void) { DEVICE_CMock_Init(); }

void tearDown(void)
{
    DEVICE_CMock_Verify();
    DEVICE_CMock_Destroy();
}

static void test_create_rejects_invalid_geometry_and_allocator_failure(void)
{
    DEV_Steer_Config_s cfg = valid_cfg();
    TEST_ASSERT_NULL(DEV_SteerChassis_Create(NULL, NULL));
    cfg.wheel_perimeter = 0.0f;
    TEST_ASSERT_NULL(DEV_SteerChassis_Create(&cfg, NULL));
    cfg        = valid_cfg();
    cfg.max_vw = -1.0f;
    TEST_ASSERT_NULL(DEV_SteerChassis_Create(&cfg, NULL));
    cfg = valid_cfg();
    PLAT_malloc_ExpectAnyArgsAndReturn(NULL);
    TEST_ASSERT_NULL(DEV_SteerChassis_Create(&cfg, NULL));
}

static void test_idle_holds_angles_and_stops_all_wheels(void)
{
    DEV_SteerChassis_s* ch = create_chassis(NULL);
    TEST_ASSERT_NOT_NULL(ch);
    const float        fdb[4] = {10.0f, -20.0f, 30.0f, 179.0f};
    DEV_Steer_Twist_s  cmd    = {1.0f, -1.0f, 0.01f};
    DEV_Steer_Output_s out    = {0};
    DEV_SteerChassis_Solve(ch, &cmd, fdb, &out);
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(fdb, out.steer_deg, 4u);
    for (unsigned i = 0u; i < 4u; i++)
    {
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out.drive_rpm[i]);
    }
}

static void test_forward_and_lateral_commands_have_golden_heading_and_speed(void)
{
    DEV_SteerChassis_s* ch     = create_chassis(NULL);
    float               fdb[4] = {0};
    DEV_Steer_Output_s  out;
    DEV_Steer_Twist_s   cmd = {0.0f, 100.0f, 0.0f};
    DEV_SteerChassis_Solve(ch, &cmd, fdb, &out);
    for (unsigned i = 0u; i < 4u; i++)
    {
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out.steer_deg[i]);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 120.0f, out.drive_rpm[i]);
    }
    cmd.vx = 100.0f;
    cmd.vy = 0.0f;
    DEV_SteerChassis_Solve(ch, &cmd, fdb, &out);
    for (unsigned i = 0u; i < 4u; i++)
    {
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, out.steer_deg[i]);
    }
}

static void test_shortest_arc_flips_drive_beyond_ninety_degrees(void)
{
    DEV_SteerChassis_s* ch     = create_chassis(NULL);
    float               fdb[4] = {-91.0f, -91.0f, -91.0f, -91.0f};
    DEV_Steer_Twist_s   cmd    = {0.0f, 100.0f, 0.0f};
    DEV_Steer_Output_s  out;
    DEV_SteerChassis_Solve(ch, &cmd, fdb, &out);
    for (unsigned i = 0u; i < 4u; i++)
    {
        TEST_ASSERT_FLOAT_WITHIN(0.01f, -180.0f, out.steer_deg[i]);
        TEST_ASSERT_TRUE(out.drive_rpm[i] < 0.0f);
    }
}

static void test_solve_estimate_round_trip_and_clamping(void)
{
    DEV_SteerChassis_s* ch     = create_chassis(NULL);
    float               fdb[4] = {0};
    DEV_Steer_Twist_s   cmd    = {2000.0f, 100.0f, 0.0f};
    DEV_Steer_Output_s  wheels;
    DEV_SteerChassis_Solve(ch, &cmd, fdb, &wheels);
    DEV_Steer_Twist_s estimate;
    DEV_SteerChassis_Estimate(ch, wheels.steer_deg, wheels.drive_rpm, &estimate);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 1000.0f, estimate.vx);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 100.0f, estimate.vy);
}

static void test_offsets_alignment_seam_and_rotate_alias(void)
{
    float               offsets[4] = {10.0f, 20.0f, 30.0f, 40.0f};
    DEV_SteerChassis_s* ch         = create_chassis(offsets);
    float               fdb[4]     = {10.0f, 20.0f, 30.0f, 40.0f};
    DEV_Steer_Twist_s   cmd        = {0.0f, 100.0f, 0.0f};
    DEV_Steer_Output_s  out;
    DEV_SteerChassis_Solve(ch, &cmd, fdb, &out);
    TEST_ASSERT_TRUE(DEV_SteerChassis_IsAligned(ch, out.steer_deg, 0.0f));

    DEV_SteerChassis_s* seam_ch     = create_chassis(NULL);
    float               seam_fdb[4] = {179.0f, 179.0f, 179.0f, 179.0f};
    DEV_Steer_Twist_s   seam_cmd    = {-1.745f, -99.985f, 0.0f};
    DEV_Steer_Output_s  seam_out;
    DEV_SteerChassis_Solve(seam_ch, &seam_cmd, seam_fdb, &seam_out);
    for (unsigned i = 0u; i < 4u; i++)
    {
        seam_fdb[i] = -179.0f;
    }
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, DEV_SteerChassis_GetSteerError(seam_ch, seam_fdb));

    DEV_Steer_Twist_s twist = {1.0f, 2.0f, 3.0f};
    DEV_SteerChassis_RotateTwist(90.0f, &twist, &twist);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -2.0f, twist.vx);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, twist.vy);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.0f, twist.vw);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_create_rejects_invalid_geometry_and_allocator_failure);
    RUN_TEST(test_idle_holds_angles_and_stops_all_wheels);
    RUN_TEST(test_forward_and_lateral_commands_have_golden_heading_and_speed);
    RUN_TEST(test_shortest_arc_flips_drive_beyond_ninety_degrees);
    RUN_TEST(test_solve_estimate_round_trip_and_clamping);
    RUN_TEST(test_offsets_alignment_seam_and_rotate_alias);
    return UNITY_END();
}

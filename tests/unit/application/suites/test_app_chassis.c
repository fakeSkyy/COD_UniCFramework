/**
 * @file test_app_chassis.c
 * @author Gao Xing
 * @date 2026/9/3
 * @version 1.0
 */

#include "unity.h"

#include <stdio.h>

#include <string.h>

#include "app_chassis.h"
#include "case_runner.h"
#include "mock_chassis_deps.h"

#define MOTOR_COUNT 4u

static DWT_Instance_s      dwt;
static CAN_Instance_s      tx_node;
static CAN_Instance_s      rx_node[MOTOR_COUNT];
static PLAT_CAN_RxCallback rx_cb;

/* Feedback identifiers the mocked driver reports, one per motor. Deliberately not
 * consecutive-by-accident: the routing must key off the value the driver returns, not
 * off the loop index. */
static const uint32_t feedback_id[MOTOR_COUNT] = {0x201u, 0x202u, 0x203u, 0x204u};

static DEV_DJI_Motor_s* attached[MOTOR_COUNT];
static unsigned         attach_count;
static DEV_DJI_Motor_s* fed_motor;
static unsigned         feeds;
static bool             gap_in_feedback_ids;

void setUp(void)
{
    mock_chassis_deps_Init();
    rx_cb               = NULL;
    attach_count        = 0u;
    fed_motor           = NULL;
    feeds               = 0u;
    gap_in_feedback_ids = false;
    memset(attached, 0, sizeof(attached));
}

void tearDown(void)
{
    mock_chassis_deps_Verify();
    mock_chassis_deps_Destroy();
}

/* ========================================================================= */
/*  Stubs                                                                    */
/* ========================================================================= */

static void capture_rx_cb(CAN_Instance_s* can, PLAT_CAN_RxCallback cb, int calls)
{
    (void) can;
    (void) calls;
    rx_cb = cb; /* every node installs the same one; keeping the last is enough */
}

static bool record_attach(DEV_DJI_Motor_s* motor, DEV_DJI_Bus_s* bus, DEV_DJI_Type_e type,
                          uint8_t id, DEV_DJI_Controller_s* ctrl, int calls)
{
    (void) bus;
    (void) calls;

    /* The ESC id must be 1..4 in order, and the controller must be NULL — these wheels
     * are commanded directly, and a stray controller would silently change what
     * SetTarget means. */
    TEST_ASSERT_EQUAL_UINT8((uint8_t) (attach_count + 1u), id);
    TEST_ASSERT_EQUAL_INT(DEV_DJI_M3508, type);
    TEST_ASSERT_NULL(ctrl);

    if (attach_count < MOTOR_COUNT)
    {
        attached[attach_count] = motor;
    }
    attach_count++;
    return true;
}

static uint32_t report_feedback_id(const DEV_DJI_Motor_s* motor, int calls)
{
    (void) calls;

    for (unsigned i = 0u; i < MOTOR_COUNT; i++)
    {
        if (attached[i] == motor)
        {
            /* The last wheel lands well past its neighbours, as a mis-dialled ESC
             * would, so the span is no longer MOTOR_COUNT wide. */
            if (gap_in_feedback_ids && i == (MOTOR_COUNT - 1u))
            {
                return 0x210u;
            }
            return feedback_id[i];
        }
    }
    return 0u;
}

static bool record_feed(DEV_DJI_Motor_s* motor, const uint8_t* data, uint8_t len, uint32_t now_ms,
                        int calls)
{
    (void) data;
    (void) len;
    (void) now_ms;
    (void) calls;
    fed_motor = motor;
    feeds++;
    return true;
}

/* ========================================================================= */
/*  Expectation helpers                                                      */
/* ========================================================================= */

/**
 * @brief Expect a complete, successful bring-up of all five nodes.
 */
static void expect_bring_up(void)
{
    Board_Timebase_ExpectAndReturn(&dwt);
    Board_CANCreate_ExpectAndReturn(BOARD_CAN1, 0x200u, 0x2FFu, &tx_node);
    DEV_DJIMotor_BusInit_ExpectAndReturn(NULL, &tx_node, 0x200u, true);
    DEV_DJIMotor_BusInit_IgnoreArg_bus();

    DEV_DJIMotor_Attach_StubWithCallback(record_attach);
    DEV_DJIMotor_FeedbackId_StubWithCallback(report_feedback_id);
    PLAT_CAN_OnReceive_StubWithCallback(capture_rx_cb);

    /* One range claim spanning every wheel's feedback, not one node each — the whole
     * point of Board_CANCreateRange. The span endpoints must come from the motors, so
     * they are asserted rather than ignored. */
    Board_CANCreateRange_ExpectAndReturn(BOARD_CAN1, 0x200u, feedback_id[0],
                                         feedback_id[MOTOR_COUNT - 1u], &rx_node[0]);
    PLAT_CAN_Start_ExpectAndReturn(&rx_node[0], true);
    PLAT_CAN_Start_ExpectAndReturn(&tx_node, true);
}

/* ========================================================================= */
/*  Cases                                                                    */
/* ========================================================================= */

/* Two nodes, not five: one transmit node and one range claim covering 0x201..0x204. */
static void test_start_task_creates_two_nodes_and_the_task(void)
{
    expect_bring_up();
    PLAT_Task_Create_ExpectAnyArgsAndReturn(true);

    TEST_ASSERT_TRUE(App_Chassis_StartTask(3u));
    TEST_ASSERT_EQUAL_UINT(MOTOR_COUNT, attach_count);
}

static void test_transmit_node_starts_last(void)
{
    /* Order matters and cmock.yml enforces it: the receive nodes bring the peripheral
     * up, so the transmit node's Start only adds its own filter. Reversing this would
     * open the bus before any receiver had a callback installed. */
    expect_bring_up();
    PLAT_Task_Create_ExpectAnyArgsAndReturn(true);

    TEST_ASSERT_TRUE(App_Chassis_StartTask(3u));
}

static void test_missing_timebase_creates_no_nodes(void)
{
    Board_Timebase_ExpectAndReturn(NULL);
    UTIL_Log_Write_Ignore();

    /* No Board_CANCreate expectation at all: a chassis that cannot age feedback must
     * not claim filter slots, because an unstarted node still holds its identifier. */
    TEST_ASSERT_FALSE(App_Chassis_StartTask(3u));
    TEST_ASSERT_EQUAL_UINT(0u, attach_count);
}

static void test_refused_transmit_node_does_not_attach_motors(void)
{
    Board_Timebase_ExpectAndReturn(&dwt);
    Board_CANCreate_ExpectAndReturn(BOARD_CAN1, 0x200u, 0x2FFu, NULL);
    UTIL_Log_Write_Ignore();

    TEST_ASSERT_FALSE(App_Chassis_StartTask(3u));
    TEST_ASSERT_EQUAL_UINT(0u, attach_count);
}

static void test_refused_receive_node_stops_bring_up(void)
{
    Board_Timebase_ExpectAndReturn(&dwt);
    Board_CANCreate_ExpectAndReturn(BOARD_CAN1, 0x200u, 0x2FFu, &tx_node);
    DEV_DJIMotor_BusInit_ExpectAnyArgsAndReturn(true);
    DEV_DJIMotor_Attach_StubWithCallback(record_attach);
    DEV_DJIMotor_FeedbackId_StubWithCallback(report_feedback_id);
    UTIL_Log_Write_Ignore();

    /* The range claim is refused — some identifier in the span is already claimed on
     * this bus, or the filter list is full. */
    Board_CANCreateRange_ExpectAndReturn(BOARD_CAN1, 0x200u, feedback_id[0],
                                         feedback_id[MOTOR_COUNT - 1u], NULL);

    TEST_ASSERT_FALSE(App_Chassis_StartTask(3u));
}

static void test_a_node_that_cannot_start_fails_bring_up(void)
{
    Board_Timebase_ExpectAndReturn(&dwt);
    Board_CANCreate_ExpectAndReturn(BOARD_CAN1, 0x200u, 0x2FFu, &tx_node);
    DEV_DJIMotor_BusInit_ExpectAnyArgsAndReturn(true);
    DEV_DJIMotor_Attach_StubWithCallback(record_attach);
    DEV_DJIMotor_FeedbackId_StubWithCallback(report_feedback_id);
    PLAT_CAN_OnReceive_StubWithCallback(capture_rx_cb);
    UTIL_Log_Write_Ignore();

    Board_CANCreateRange_ExpectAndReturn(BOARD_CAN1, 0x200u, feedback_id[0],
                                         feedback_id[MOTOR_COUNT - 1u], &rx_node[0]);
    PLAT_CAN_Start_ExpectAndReturn(&rx_node[0], false);

    TEST_ASSERT_FALSE(App_Chassis_StartTask(3u));
}

static void test_feedback_routes_to_the_motor_owning_the_identifier(void)
{
    expect_bring_up();
    PLAT_Task_Create_ExpectAnyArgsAndReturn(true);
    TEST_ASSERT_TRUE(App_Chassis_StartTask(3u));
    TEST_ASSERT_NOT_NULL(rx_cb);

    const uint8_t payload[8] = {0};

    /* Third motor's identifier: the scan must land on the third attached motor, not on
     * the first or on whichever node the frame arrived through. This is the property
     * that would break if the routing used the node instead of the identifier. */
    PLAT_DWT_GetTimeline_ms_ExpectAndReturn(&dwt, 1234u);
    DEV_DJIMotor_FeedbackId_StubWithCallback(report_feedback_id);
    DEV_DJIMotor_OnFeedback_StubWithCallback(record_feed);

    rx_cb(&rx_node[0], feedback_id[2], payload, sizeof payload);

    TEST_ASSERT_EQUAL_UINT(1u, feeds);
    TEST_ASSERT_EQUAL_PTR(attached[2], fed_motor);
}

static void test_unknown_identifier_is_dropped(void)
{
    expect_bring_up();
    PLAT_Task_Create_ExpectAnyArgsAndReturn(true);
    TEST_ASSERT_TRUE(App_Chassis_StartTask(3u));

    const uint8_t payload[8] = {0};

    /* A filter admits by mask, so an identifier nobody claimed can arrive. It must be
     * discarded rather than delivered to motor 0. */
    PLAT_DWT_GetTimeline_ms_ExpectAndReturn(&dwt, 5u);
    DEV_DJIMotor_FeedbackId_StubWithCallback(report_feedback_id);

    rx_cb(&rx_node[0], 0x7FFu, payload, sizeof payload);

    TEST_ASSERT_EQUAL_UINT(0u, feeds);
}

/**
 * @brief Non-consecutive feedback identifiers are refused, not widened into.
 *
 * A mis-dialled ESC breaks the assumption the single range claim rests on. Claiming a
 * span wide enough to still contain the wheels would admit identifiers this task does
 * not own, so bring-up must fail instead — and it must fail before Board_CANCreateRange
 * is reached, which is what the absent expectation asserts.
 */
static void test_non_consecutive_feedback_ids_are_refused(void)
{
    Board_Timebase_ExpectAndReturn(&dwt);
    Board_CANCreate_ExpectAndReturn(BOARD_CAN1, 0x200u, 0x2FFu, &tx_node);
    DEV_DJIMotor_BusInit_ExpectAnyArgsAndReturn(true);
    DEV_DJIMotor_Attach_StubWithCallback(record_attach);
    UTIL_Log_Write_Ignore();

    /* Report a gap: 0x201, 0x202, 0x203, 0x210 spans 16 rather than 4. */
    gap_in_feedback_ids = true;
    DEV_DJIMotor_FeedbackId_StubWithCallback(report_feedback_id);

    TEST_ASSERT_FALSE(App_Chassis_StartTask(3u));
}

/**
 * @brief Online reports false before bring-up rather than dereferencing a NULL timebase.
 *
 * Reachable because app_tasks treats a chassis failure as non-fatal: the firmware keeps
 * running with this module never brought up, and PLAT_DWT_GetTimeline_ms does not test
 * its argument.
 */
static void test_online_is_false_before_bring_up(void) { TEST_ASSERT_FALSE(App_Chassis_Online()); }

static void test_start_task_propagates_task_creation_failure(void)
{
    expect_bring_up();
    PLAT_Task_Create_ExpectAnyArgsAndReturn(false);

    TEST_ASSERT_FALSE(App_Chassis_StartTask(3u));
}

int main(int argc, char** argv)
{
    APP_CASES_BEGIN();
    APP_CASE(start_task_creates_two_nodes_and_the_task);
    APP_CASE(transmit_node_starts_last);
    APP_CASE(missing_timebase_creates_no_nodes);
    APP_CASE(refused_transmit_node_does_not_attach_motors);
    APP_CASE(refused_receive_node_stops_bring_up);
    APP_CASE(a_node_that_cannot_start_fails_bring_up);
    APP_CASE(feedback_routes_to_the_motor_owning_the_identifier);
    APP_CASE(unknown_identifier_is_dropped);
    APP_CASE(non_consecutive_feedback_ids_are_refused);
    APP_CASE(online_is_false_before_bring_up);
    APP_CASE(start_task_propagates_task_creation_failure);
    APP_CASES_END();
}

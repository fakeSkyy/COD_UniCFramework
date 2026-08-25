/**
 * @file test_motor_can_dji.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */
#define PLAT_ALLOW_CONSTRUCTION
#include "dev_dji_motor.h"
#include "mock_motor_dji_contract.h"
#include "plat_can.h"
#include "unity.h"
#include <stdlib.h>
#include <string.h>

#include "support/alloc/host_alloc_tracker.h"

static int             ctx;
static IMPL_CAN_RxCb   backend_rx;
static void*           backend_arg;
static uint8_t         wire[8];
static uint32_t        wire_id;
static bool            send_ok;
static DEV_DJI_Motor_s motor;

static void* alloc_cb(size_t n, int calls)
{
    (void) calls;
    return TEST_TrackedMalloc(n);
}
static void free_cb(void* p, int calls)
{
    (void) calls;
    TEST_TrackedFree(p);
}
static bool send(void* c, const uint8_t* d, uint8_t l)
{
    (void) c;
    (void) d;
    (void) l;
    return send_ok;
}
static bool send_to(void* c, uint32_t id, const uint8_t* d, uint8_t l)
{
    (void) c;
    wire_id = id;
    memcpy(wire, d, l);
    return send_ok;
}
static void attach(void* c, IMPL_CAN_RxCb rx, IMPL_CAN_ErrCb err, void* arg)
{
    (void) c;
    (void) err;
    backend_rx  = rx;
    backend_arg = arg;
}
static bool start(void* c)
{
    (void) c;
    return true;
}
static uint32_t tx_free(void* c)
{
    (void) c;
    return 3u;
}
static const CAN_Ops_s ops = {send, send_to, attach, start, tx_free};
static void            on_rx(CAN_Instance_s* can, uint32_t id, const uint8_t* data, uint8_t len)
{
    (void) can;
    TEST_ASSERT_EQUAL_HEX32(0x201u, id);
    TEST_ASSERT_TRUE(DEV_DJIMotor_OnFeedback(&motor, data, len, 77u));
}

void setUp(void)
{
    mock_motor_dji_contract_Init();
    PLAT_malloc_StubWithCallback(alloc_cb);
    PLAT_free_StubWithCallback(free_cb);
    backend_rx = NULL;
    send_ok    = true;
    memset(wire, 0, sizeof wire);
}
void tearDown(void)
{
    mock_motor_dji_contract_Verify();
    mock_motor_dji_contract_Destroy();
}

static void test_backend_feedback_and_device_commit_cross_real_platform(void)
{
    CAN_Instance_s* can = PLAT_CAN_Create(&ops, &ctx);
    TEST_ASSERT_NOT_NULL(can);
    TEST_ASSERT_NOT_NULL(backend_rx);
    PLAT_CAN_OnReceive(can, on_rx);
    TEST_ASSERT_TRUE(PLAT_CAN_Start(can));
    DEV_DJI_Bus_s bus;
    TEST_ASSERT_TRUE(DEV_DJIMotor_BusInit(&bus, can, 0x200u));
    TEST_ASSERT_TRUE(DEV_DJIMotor_Attach(&motor, &bus, DEV_DJI_M3508, 1u, NULL));
    const uint8_t feedback[8] = {0x10u, 0x00u, 0xFFu, 0x9Cu, 0x12u, 0x34u, 55u, 0u};
    backend_rx(backend_arg, 0x201u, feedback, 8u);
    TEST_ASSERT_EQUAL_UINT32(1u, motor.fdb.frame_count);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -100.0f / (3591.0f / 187.0f), motor.fdb.rpm);
    TEST_ASSERT_EQUAL_UINT32(77u, motor.wd.last_kick_ms);
    DEV_DJIMotor_SetOutput(&motor, 0x1234);
    TEST_ASSERT_TRUE(DEV_DJIMotor_CommitBus(&bus, 0.001f));
    const uint8_t expected[8] = {0x12u, 0x34u, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, wire, 8u);
    TEST_ASSERT_EQUAL_HEX32(0x200u, wire_id);
    send_ok = false;
    TEST_ASSERT_FALSE(DEV_DJIMotor_CommitBus(&bus, 0.001f));
    TEST_ASSERT_EQUAL_UINT32(1u, bus.tx_fail);
}
int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_backend_feedback_and_device_commit_cross_real_platform);
    return UNITY_END();
}

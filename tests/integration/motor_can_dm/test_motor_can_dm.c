/**
 * @file test_motor_can_dm.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */
#define PLAT_ALLOW_CONSTRUCTION
#include "dev_dm_motor.h"
#include "mock_motor_dm_contract.h"
#include "plat_can.h"
#include "unity.h"
#include <stdlib.h>
#include <string.h>

#include "support/alloc/host_alloc_tracker.h"
static int            ctx;
static IMPL_CAN_RxCb  backend_rx;
static void*          backend_arg;
static uint8_t        wire[8];
static uint32_t       wire_id;
static bool           send_ok;
static DEV_DM_Motor_s motor;
static void*          alloc_cb(size_t n, int c)
{
    (void) c;
    return TEST_TrackedMalloc(n);
}
static void free_cb(void* p, int c)
{
    (void) c;
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
static void attach(void* c, IMPL_CAN_RxCb rx, IMPL_CAN_ErrCb e, void* a)
{
    (void) c;
    (void) e;
    backend_rx  = rx;
    backend_arg = a;
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
static void            on_rx(CAN_Instance_s* c, uint32_t id, const uint8_t* d, uint8_t l)
{
    (void) c;
    TEST_ASSERT_EQUAL_HEX32(0x241u, id);
    TEST_ASSERT_TRUE(DEV_DMMotor_OnFeedback(&motor, d, l, 91u));
}
void setUp(void)
{
    mock_motor_dm_contract_Init();
    PLAT_malloc_StubWithCallback(alloc_cb);
    PLAT_free_StubWithCallback(free_cb);
    backend_rx = NULL;
    send_ok    = true;
    memset(wire, 0, sizeof wire);
}
void tearDown(void)
{
    mock_motor_dm_contract_Verify();
    mock_motor_dm_contract_Destroy();
}
static void test_backend_feedback_mit_wire_and_send_failure(void)
{
    CAN_Instance_s* can = PLAT_CAN_Create(&ops, &ctx);
    TEST_ASSERT_NOT_NULL(can);
    PLAT_CAN_OnReceive(can, on_rx);
    TEST_ASSERT_TRUE(PLAT_CAN_Start(can));
    DEV_DM_Cfg_s cfg = {0};
    cfg.tx_id        = 0x141u;
    cfg.rx_id        = 0x241u;
    TEST_ASSERT_TRUE(DEV_DMMotor_Init(&motor, can, &cfg, NULL));
    const uint8_t feedback[8] = {0xA3u, 0x80u, 0, 0x80u, 0x08u, 0, 50u, 60u};
    backend_rx(backend_arg, 0x241u, feedback, 8u);
    TEST_ASSERT_EQUAL_UINT32(1u, motor.fdb.frame_count);
    TEST_ASSERT_EQUAL(DEV_DM_ERR_OVERCURRENT, DEV_DMMotor_GetFault(&motor));
    TEST_ASSERT_EQUAL_UINT32(91u, motor.wd.last_kick_ms);
    DEV_DMMotor_SetMIT(&motor, 0, 0, 0, 0, 0);
    TEST_ASSERT_TRUE(DEV_DMMotor_Commit(&motor, 0.01f));
    const uint8_t expected[8] = {0x80u, 0, 0x80u, 0, 0, 0, 0x08u, 0};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, wire, 8u);
    TEST_ASSERT_EQUAL_HEX32(0x141u, wire_id);
    send_ok = false;
    TEST_ASSERT_FALSE(DEV_DMMotor_Commit(&motor, 0.01f));
    TEST_ASSERT_EQUAL_UINT32(1u, motor.tx_fail);
}
int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_backend_feedback_mit_wire_and_send_failure);
    return UNITY_END();
}

/**
 * @file test_msgbus_rtos.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */
#include "mock_FreeRTOS.h"
#include "mock_semphr.h"
#include "mock_task.h"
#include "unity.h"
#include "util_msgbus.h"

static BaseType_t scheduler_state;
static unsigned   take_calls;
static unsigned   give_calls;
static unsigned   notify_calls;
static TickType_t last_wait;
static BaseType_t wait_result;
static void*      current_handle = (void*) 0xCAFEu;

void                     RTOS_FaultInit(void) {}
static SemaphoreHandle_t create_mutex_cb(StaticSemaphore_t* storage, int calls)
{
    (void) calls;
    return storage;
}
static BaseType_t irq_cb(int calls)
{
    (void) calls;
    return pdFALSE;
}
static BaseType_t scheduler_cb(int calls)
{
    (void) calls;
    return scheduler_state;
}
static BaseType_t take_cb(SemaphoreHandle_t sem, TickType_t timeout, int calls)
{
    (void) sem;
    (void) calls;
    TEST_ASSERT_EQUAL_HEX32(UINT32_MAX, timeout);
    take_calls++;
    return pdTRUE;
}
static BaseType_t give_cb(SemaphoreHandle_t sem, int calls)
{
    (void) sem;
    (void) calls;
    give_calls++;
    return pdTRUE;
}
static TaskHandle_t current_cb(int calls)
{
    (void) calls;
    return current_handle;
}
static void notify_cb(TaskHandle_t task, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_PTR(current_handle, task);
    notify_calls++;
}
static uint32_t wait_cb(BaseType_t clear, TickType_t timeout, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL(pdTRUE, clear);
    last_wait = timeout;
    return wait_result ? 1u : 0u;
}

void setUp(void)
{
    mock_FreeRTOS_Init();
    mock_semphr_Init();
    mock_task_Init();
    scheduler_state = taskSCHEDULER_NOT_STARTED;
    take_calls      = 0;
    give_calls      = 0;
    notify_calls    = 0;
    last_wait       = 0;
    wait_result     = pdFALSE;
    xSemaphoreCreateMutexStatic_StubWithCallback(create_mutex_cb);
    xPortIsInsideInterrupt_StubWithCallback(irq_cb);
    xTaskGetSchedulerState_StubWithCallback(scheduler_cb);
    xSemaphoreTake_StubWithCallback(take_cb);
    xSemaphoreGive_StubWithCallback(give_cb);
    xTaskGetCurrentTaskHandle_StubWithCallback(current_cb);
    xTaskNotifyGive_StubWithCallback(notify_cb);
    ulTaskNotifyTake_StubWithCallback(wait_cb);
}
void tearDown(void)
{
    mock_FreeRTOS_Verify();
    mock_semphr_Verify();
    mock_task_Verify();
    mock_FreeRTOS_Destroy();
    mock_semphr_Destroy();
    mock_task_Destroy();
}
static void test_bringup_runtime_wake_publish_and_wait_passthrough(void)
{
    TEST_ASSERT_TRUE(UTIL_MsgBus_Init());
    UTIL_MsgBus_Id id = UTIL_MsgBus_Register("control", sizeof(uint32_t));
    TEST_ASSERT_NOT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, id);
    TEST_ASSERT_EQUAL_UINT(0u, take_calls);
    scheduler_state = taskSCHEDULER_RUNNING;
    UTIL_MsgBus_Sub_s sub;
    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&sub, id));
    TEST_ASSERT_TRUE(take_calls > 0u);
    TEST_ASSERT_EQUAL_UINT(take_calls, give_calls);
    TEST_ASSERT_TRUE(UTIL_MsgBus_WantWake(&sub));
    TEST_ASSERT_EQUAL_PTR(current_handle, sub.waiter);
    uint32_t value = 0x12345678u;
    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &value));
    TEST_ASSERT_EQUAL_UINT(1u, notify_calls);
    uint32_t out = 0;
    TEST_ASSERT_TRUE(UTIL_MsgBus_Copy(&sub, &out));
    TEST_ASSERT_EQUAL_HEX32(value, out);
    wait_result = pdFALSE;
    TEST_ASSERT_FALSE(UTIL_MsgBus_Wait(&sub, 77u));
    TEST_ASSERT_EQUAL_UINT32(77u, last_wait);
    wait_result = pdTRUE;
    TEST_ASSERT_FALSE(UTIL_MsgBus_Wait(&sub, UTIL_MSGBUS_WAIT_FOREVER));
    TEST_ASSERT_EQUAL_HEX32(UINT32_MAX, last_wait);
    value = 0xAABBCCDDu;
    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &value));
    TEST_ASSERT_EQUAL_UINT(2u, notify_calls);
    TEST_ASSERT_TRUE(UTIL_MsgBus_Wait(&sub, 5u));
}
int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_bringup_runtime_wake_publish_and_wait_passthrough);
    return UNITY_END();
}

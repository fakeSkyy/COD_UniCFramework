/**
 * @file test_util_msgbus_cmock.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#include "utils_cmock_test_support.h"

#include "util_msgbus.h"

static UTIL_MsgBus_Id locked_id;

void setUp(void) { Utils_CMock_Init(); }
void tearDown(void) { Utils_CMock_Verify(); }

static void expect_mutex_init(bool result)
{
    PLAT_Mutex_Init_ExpectAndReturn(NULL, result);
    PLAT_Mutex_Init_IgnoreArg_mutex();
}

static void expect_lock(bool result)
{
    PLAT_Mutex_LockRequired_ExpectAndReturn(true);
    PLAT_Mutex_Lock_ExpectAndReturn(NULL, PLAT_MUTEX_WAIT_FOREVER, result);
    PLAT_Mutex_Lock_IgnoreArg_mutex();
}

static void expect_unlock(void)
{
    PLAT_Mutex_Unlock_Expect(NULL);
    PLAT_Mutex_Unlock_IgnoreArg_mutex();
}

static void expect_locked_operation(void)
{
    expect_lock(true);
    expect_unlock();
}

static void test_msgbus_init_propagates_mutex_failure_and_can_retry(void)
{
    expect_mutex_init(false);
    TEST_ASSERT_FALSE(UTIL_MsgBus_Init());

    expect_mutex_init(true);
    TEST_ASSERT_TRUE(UTIL_MsgBus_Init());

    TEST_ASSERT_TRUE(UTIL_MsgBus_Init());
}

static void test_msgbus_bringup_phase_skips_mutex_take(void)
{
    PLAT_Mutex_LockRequired_ExpectAndReturn(false);
    locked_id = UTIL_MsgBus_Register("locked", sizeof(uint32_t));
    TEST_ASSERT_NOT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, locked_id);

    PLAT_Mutex_LockRequired_ExpectAndReturn(false);
    TEST_ASSERT_EQUAL_UINT8(locked_id, UTIL_MsgBus_Find("locked"));
}

static void test_msgbus_runtime_lock_failure_refuses_without_unlocking(void)
{
    uint32_t          value = 1u;
    UTIL_MsgBus_Sub_s sub;

    expect_lock(false);
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Register("denied", 4u));

    expect_lock(false);
    TEST_ASSERT_FALSE(UTIL_MsgBus_Publish(locked_id, &value));

    expect_lock(false);
    TEST_ASSERT_FALSE(UTIL_MsgBus_Subscribe(&sub, locked_id));
    TEST_ASSERT_FALSE(sub.attached);

    expect_lock(false);
    TEST_ASSERT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, UTIL_MsgBus_Find("locked"));
}

static void test_msgbus_locked_topic_balances_lock_notifies_and_forwards_wait_timeout(void)
{
    UTIL_MsgBus_Sub_s sub;
    uint32_t          value = 0x12345678u;
    uint32_t          out   = 0u;
    void*             task  = (void*) 0x7451u;

    expect_locked_operation();
    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&sub, locked_id));

    PLAT_Task_Current_ExpectAndReturn(task);
    expect_locked_operation();
    TEST_ASSERT_TRUE(UTIL_MsgBus_WantWake(&sub));
    TEST_ASSERT_EQUAL_PTR(task, sub.waiter);

    expect_locked_operation();
    PLAT_Task_Notify_Expect(task);
    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(locked_id, &value));

    expect_locked_operation();
    TEST_ASSERT_TRUE(UTIL_MsgBus_Copy(&sub, &out));
    TEST_ASSERT_EQUAL_HEX32(value, out);

    PLAT_Task_Wait_ExpectAndReturn(77u, false);
    TEST_ASSERT_FALSE(UTIL_MsgBus_Wait(&sub, 77u));

    PLAT_Task_Wait_ExpectAndReturn(UTIL_MSGBUS_WAIT_FOREVER, true);
    TEST_ASSERT_FALSE(UTIL_MsgBus_Wait(&sub, UTIL_MSGBUS_WAIT_FOREVER));
}

static void test_msgbus_wantwake_rejects_non_task_context_before_lock(void)
{
    UTIL_MsgBus_Sub_s sub;

    expect_locked_operation();
    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&sub, locked_id));

    PLAT_Task_Current_ExpectAndReturn(NULL);
    TEST_ASSERT_FALSE(UTIL_MsgBus_WantWake(&sub));
    TEST_ASSERT_NULL(sub.waiter);
}

static void test_msgbus_seqlock_publish_copy_and_notify_do_not_take_mutex(void)
{
    UTIL_MsgBus_Sub_s sub;
    uint32_t          value = 0xAABBCCDDu;
    uint32_t          out   = 0u;
    void*             task  = (void*) 0x8899u;

    expect_locked_operation();
    UTIL_MsgBus_Id id = UTIL_MsgBus_RegisterSeqlock("seqlock", sizeof value);
    TEST_ASSERT_NOT_EQUAL_UINT8(UTIL_MSGBUS_INVALID_ID, id);

    expect_locked_operation();
    TEST_ASSERT_TRUE(UTIL_MsgBus_Subscribe(&sub, id));

    PLAT_Task_Current_ExpectAndReturn(task);
    expect_locked_operation();
    TEST_ASSERT_TRUE(UTIL_MsgBus_WantWake(&sub));

    PLAT_Task_Notify_Expect(task);
    TEST_ASSERT_TRUE(UTIL_MsgBus_Publish(id, &value));
    TEST_ASSERT_TRUE(UTIL_MsgBus_Copy(&sub, &out));
    TEST_ASSERT_EQUAL_HEX32(value, out);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_msgbus_init_propagates_mutex_failure_and_can_retry);
    RUN_TEST(test_msgbus_bringup_phase_skips_mutex_take);
    RUN_TEST(test_msgbus_runtime_lock_failure_refuses_without_unlocking);
    RUN_TEST(test_msgbus_locked_topic_balances_lock_notifies_and_forwards_wait_timeout);
    RUN_TEST(test_msgbus_wantwake_rejects_non_task_context_before_lock);
    RUN_TEST(test_msgbus_seqlock_publish_copy_and_notify_do_not_take_mutex);
    return UNITY_END();
}

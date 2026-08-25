/**
 * @file test_dev_remote.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "unity.h"

#include <stdlib.h>

#include "support/alloc/host_alloc_tracker.h"

#include "dev_remote.h"
#include "device_test_support.h"

static PLAT_UART_RxCallback saved_rx;
static uint8_t*             saved_buf;
static uint16_t             saved_size;
static bool                 start_result;
static void*                freed_ptr;

static const uint8_t frame_idle[18] = {
    0x00u, 0x04u, 0x20u, 0x00u, 0x01u, 0x58u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x04u,
};

static const uint8_t frame_w_mouse[18] = {
    0x00u, 0x04u, 0x20u, 0x00u, 0x01u, 0x58u, 0x34u, 0x12u, 0xFEu,
    0xFFu, 0x03u, 0x00u, 0x01u, 0x00u, 0x01u, 0x00u, 0x00u, 0x04u,
};

static void* host_alloc(size_t size, int calls)
{
    (void) calls;
    return TEST_TrackedMalloc(size);
}

static void capture_free(void* ptr, int calls)
{
    (void) calls;
    freed_ptr = ptr;
    TEST_TrackedFree(ptr);
}

static void capture_on_receive(UART_Instance_s* uart, PLAT_UART_RxCallback cb, int calls)
{
    (void) uart;
    (void) calls;
    saved_rx = cb;
}

static bool capture_start(UART_Instance_s* uart, uint8_t* buf, uint16_t size, int calls)
{
    (void) uart;
    (void) calls;
    saved_buf  = buf;
    saved_size = size;
    return start_result;
}

static void install_uart(void)
{
    PLAT_malloc_StubWithCallback(host_alloc);
    PLAT_free_StubWithCallback(capture_free);
    PLAT_UART_OnReceive_StubWithCallback(capture_on_receive);
    PLAT_UART_StartReceive_StubWithCallback(capture_start);
}

static DEV_Remote_s* create_remote(UART_Instance_s* uart)
{
    install_uart();
    return DEV_Remote_Create(uart, 2u, 4u, 3u);
}

void setUp(void)
{
    DEVICE_CMock_Init();
    saved_rx     = NULL;
    saved_buf    = NULL;
    saved_size   = 0u;
    start_result = true;
    freed_ptr    = NULL;
}

void tearDown(void)
{
    DEVICE_CMock_Verify();
    DEVICE_CMock_Destroy();
}

static void test_create_validates_arguments_and_allocator(void)
{
    UART_Instance_s uart = {0};
    TEST_ASSERT_NULL(DEV_Remote_Create(NULL, 1u, 2u, 3u));
    TEST_ASSERT_NULL(DEV_Remote_Create(&uart, 1u, 1u, 3u));
    TEST_ASSERT_NULL(DEV_Remote_Create(&uart, 1u, 2u, 0u));
    PLAT_malloc_ExpectAnyArgsAndReturn(NULL);
    TEST_ASSERT_NULL(DEV_Remote_Create(&uart, 1u, 2u, 3u));
}

static void test_create_captures_callback_buffer_and_rolls_back_start_failure(void)
{
    UART_Instance_s uart = {0};
    install_uart();
    DEV_Remote_s* dev = DEV_Remote_Create(&uart, 1u, 2u, 3u);
    TEST_ASSERT_NOT_NULL(dev);
    TEST_ASSERT_EQUAL_PTR(dev, uart.id);
    TEST_ASSERT_NOT_NULL(saved_rx);
    TEST_ASSERT_NOT_NULL(saved_buf);
    TEST_ASSERT_EQUAL_UINT16(DEV_REMOTE_RX_BUF_SIZE, saved_size);

    UART_Instance_s failed_uart = {0};
    start_result                = false;
    DEV_Remote_s* failed        = DEV_Remote_Create(&failed_uart, 1u, 2u, 3u);
    TEST_ASSERT_NULL(failed);
    TEST_ASSERT_NULL(failed_uart.id);
    TEST_ASSERT_NULL(saved_rx);
    TEST_ASSERT_NOT_NULL(freed_ptr);
}

static void test_callback_injection_decodes_golden_channels_mouse_and_keys(void)
{
    UART_Instance_s uart = {0};
    DEV_Remote_s*   dev  = create_remote(&uart);
    TEST_ASSERT_TRUE(DEV_Remote_IsLinkLost(dev));
    saved_rx(&uart, frame_w_mouse, sizeof(frame_w_mouse));
    DEV_Remote_Tick(dev);
    const DEV_Remote_Input_s* in = DEV_Remote_GetInput(dev);
    for (unsigned i = 0u; i < 5u; i++)
    {
        TEST_ASSERT_EQUAL_INT16(0, in->ch[i]);
    }
    TEST_ASSERT_EQUAL_UINT8(1u, in->sw[0]);
    TEST_ASSERT_EQUAL_UINT8(1u, in->sw[1]);
    TEST_ASSERT_EQUAL_INT16(0x1234, in->mouse_x);
    TEST_ASSERT_EQUAL_INT16(-2, in->mouse_y);
    TEST_ASSERT_TRUE(in->mouse_l);
    TEST_ASSERT_TRUE(DEV_Remote_IsKeyPressed(dev, DEV_KEY_W));
    TEST_ASSERT_TRUE(DEV_Remote_GetKeyToggle(dev, DEV_KEY_W));
    TEST_ASSERT_EQUAL_UINT32(1u, DEV_Remote_GetFrameCount(dev));
}

static void test_split_frame_and_merged_frames_publish_latest(void)
{
    UART_Instance_s uart = {0};
    DEV_Remote_s*   dev  = create_remote(&uart);
    saved_rx(&uart, frame_idle, 17u);
    DEV_Remote_Tick(dev);
    TEST_ASSERT_TRUE(DEV_Remote_IsLinkLost(dev));
    saved_rx(&uart, frame_idle + 17u, 1u);
    DEV_Remote_Tick(dev);
    TEST_ASSERT_FALSE(DEV_Remote_IsLinkLost(dev));

    uint8_t merged[36];
    for (unsigned i = 0u; i < 18u; i++)
    {
        merged[i]       = frame_idle[i];
        merged[18u + i] = frame_w_mouse[i];
    }
    saved_rx(&uart, merged, sizeof(merged));
    DEV_Remote_Tick(dev);
    TEST_ASSERT_TRUE(DEV_Remote_GetInput(dev)->mouse_l);
    TEST_ASSERT_EQUAL_UINT32(3u, DEV_Remote_GetFrameCount(dev));
}

static void test_key_state_machine_mouse_key_toggle_and_release_edges(void)
{
    UART_Instance_s uart = {0};
    DEV_Remote_s*   dev  = create_remote(&uart);
    saved_rx(&uart, frame_w_mouse, sizeof(frame_w_mouse));
    DEV_Remote_Tick(dev);
    TEST_ASSERT_EQUAL(DEV_KEY_STATE_PRESSED, DEV_Remote_GetKeyState(dev, DEV_KEY_MOUSE_L));
    DEV_Remote_Tick(dev);
    TEST_ASSERT_EQUAL(DEV_KEY_STATE_DOWN, DEV_Remote_GetKeyState(dev, DEV_KEY_W));
    DEV_Remote_Tick(dev);
    TEST_ASSERT_EQUAL(DEV_KEY_STATE_HELD, DEV_Remote_GetKeyState(dev, DEV_KEY_W));
    saved_rx(&uart, frame_idle, sizeof(frame_idle));
    DEV_Remote_Tick(dev);
    TEST_ASSERT_TRUE(DEV_Remote_IsKeyReleased(dev, DEV_KEY_W));
    TEST_ASSERT_TRUE(DEV_Remote_GetKeyToggle(dev, DEV_KEY_MOUSE_L));
    DEV_Remote_SetKeyToggle(dev, DEV_KEY_W, false);
    TEST_ASSERT_FALSE(DEV_Remote_GetKeyToggle(dev, DEV_KEY_W));
    TEST_ASSERT_EQUAL(DEV_KEY_STATE_UP,
                      DEV_Remote_GetKeyState(dev, (DEV_Remote_Key_e) DEV_KEY_COUNT));
}

static void test_timeout_neutralizes_live_input_but_preserves_toggle(void)
{
    UART_Instance_s uart = {0};
    DEV_Remote_s*   dev  = create_remote(&uart);
    saved_rx(&uart, frame_w_mouse, sizeof(frame_w_mouse));
    DEV_Remote_Tick(dev);
    TEST_ASSERT_FALSE(DEV_Remote_IsLinkLost(dev));
    DEV_Remote_Tick(dev);
    DEV_Remote_Tick(dev);
    TEST_ASSERT_FALSE(DEV_Remote_IsLinkLost(dev));
    DEV_Remote_Tick(dev);
    TEST_ASSERT_TRUE(DEV_Remote_IsLinkLost(dev));
    TEST_ASSERT_EQUAL_INT16(0, DEV_Remote_GetInput(dev)->mouse_x);
    TEST_ASSERT_EQUAL_UINT8(0u, DEV_Remote_GetInput(dev)->sw[0]);
    TEST_ASSERT_TRUE(DEV_Remote_GetKeyToggle(dev, DEV_KEY_W));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_create_validates_arguments_and_allocator);
    RUN_TEST(test_create_captures_callback_buffer_and_rolls_back_start_failure);
    RUN_TEST(test_callback_injection_decodes_golden_channels_mouse_and_keys);
    RUN_TEST(test_split_frame_and_merged_frames_publish_latest);
    RUN_TEST(test_key_state_machine_mouse_key_toggle_and_release_edges);
    RUN_TEST(test_timeout_neutralizes_live_input_but_preserves_toggle);
    return UNITY_END();
}

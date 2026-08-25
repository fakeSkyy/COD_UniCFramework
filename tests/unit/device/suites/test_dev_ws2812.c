/**
 * @file test_dev_ws2812.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "unity.h"

#include <string.h>

#include "dev_ws2812.h"
#include "device_test_support.h"

static uint8_t  tx_copy[256];
static uint16_t tx_len;
static uint32_t tx_timeout;
static bool     tx_result;

static bool capture_send(SPI_Instance_s* spi, const uint8_t* tx, uint16_t len, uint32_t timeout,
                         int calls)
{
    (void) spi;
    (void) calls;
    memcpy(tx_copy, tx, len);
    tx_len     = len;
    tx_timeout = timeout;
    return tx_result;
}

void setUp(void)
{
    DEVICE_CMock_Init();
    memset(tx_copy, 0, sizeof(tx_copy));
    tx_len     = 0u;
    tx_timeout = 0u;
    tx_result  = true;
}

void tearDown(void)
{
    DEVICE_CMock_Verify();
    DEVICE_CMock_Destroy();
}

static void test_init_rejects_bad_arguments_and_short_buffer(void)
{
    DEV_WS2812_s   dev = {0};
    SPI_Instance_s spi = {0};
    uint8_t        buf[124];

    TEST_ASSERT_FALSE(DEV_WS2812_Init(NULL, &spi, buf, sizeof(buf), 1u));
    TEST_ASSERT_FALSE(DEV_WS2812_Init(&dev, NULL, buf, sizeof(buf), 1u));
    TEST_ASSERT_FALSE(DEV_WS2812_Init(&dev, &spi, NULL, sizeof(buf), 1u));
    TEST_ASSERT_FALSE(DEV_WS2812_Init(&dev, &spi, buf, sizeof(buf), 0u));
    TEST_ASSERT_FALSE(DEV_WS2812_Init(&dev, &spi, buf, 123u, 1u));
    TEST_ASSERT_FALSE(dev.initialized);
}

static void test_init_blanks_pixels_and_latch(void)
{
    DEV_WS2812_s   dev = {0};
    SPI_Instance_s spi = {0};
    uint8_t        buf[148];
    memset(buf, 0xAA, sizeof(buf));

    TEST_ASSERT_TRUE(DEV_WS2812_Init(&dev, &spi, buf, sizeof(buf), 2u));
    TEST_ASSERT_EQUAL_UINT16(148u, dev.len);
    TEST_ASSERT_EQUAL_UINT16(2u, dev.count);
    for (unsigned i = 0u; i < 48u; i++)
    {
        TEST_ASSERT_EQUAL_HEX8(0x60u, buf[i]);
    }
    for (unsigned i = 48u; i < sizeof(buf); i++)
    {
        TEST_ASSERT_EQUAL_HEX8(0u, buf[i]);
    }
}

static void test_set_pixel_encodes_grb_msb_first_without_touching_neighbor(void)
{
    DEV_WS2812_s   dev = {0};
    SPI_Instance_s spi = {0};
    uint8_t        buf[148];
    TEST_ASSERT_TRUE(DEV_WS2812_Init(&dev, &spi, buf, sizeof(buf), 2u));

    DEV_WS2812_SetPixel(&dev, 0u, 0x80u, 0x01u, 0x00u);
    TEST_ASSERT_EQUAL_HEX8(0x60u, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x78u, buf[7]);
    TEST_ASSERT_EQUAL_HEX8(0x78u, buf[8]);
    TEST_ASSERT_EQUAL_HEX8(0x60u, buf[23]);
    for (unsigned i = 24u; i < 48u; i++)
    {
        TEST_ASSERT_EQUAL_HEX8(0x60u, buf[i]);
    }
}

static void test_set_pixel_bounds_and_set_all_copy_exactly(void)
{
    DEV_WS2812_s   dev = {0};
    SPI_Instance_s spi = {0};
    uint8_t        buf[148];
    TEST_ASSERT_TRUE(DEV_WS2812_Init(&dev, &spi, buf, sizeof(buf), 2u));
    uint8_t before[148];
    memcpy(before, buf, sizeof(buf));
    DEV_WS2812_SetPixel(&dev, 2u, 1u, 2u, 3u);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(before, buf, sizeof(buf));

    DEV_WS2812_SetAll(&dev, 0xFFu, 0u, 0u);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buf, buf + 24u, 24u);
    for (unsigned i = 0u; i < 8u; i++)
    {
        TEST_ASSERT_EQUAL_HEX8(0x60u, buf[i]);
        TEST_ASSERT_EQUAL_HEX8(0x78u, buf[8u + i]);
        TEST_ASSERT_EQUAL_HEX8(0x60u, buf[16u + i]);
    }
}

static void test_show_captures_complete_frame_and_propagates_result(void)
{
    DEV_WS2812_s   dev = {0};
    SPI_Instance_s spi = {0};
    uint8_t        buf[124];
    TEST_ASSERT_TRUE(DEV_WS2812_Init(&dev, &spi, buf, sizeof(buf), 1u));
    PLAT_SPI_Send_StubWithCallback(capture_send);

    TEST_ASSERT_TRUE(DEV_WS2812_Show(&dev));
    TEST_ASSERT_EQUAL_UINT16(sizeof(buf), tx_len);
    TEST_ASSERT_EQUAL_UINT32(50u, tx_timeout);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buf, tx_copy, sizeof(buf));

    tx_result = false;
    TEST_ASSERT_FALSE(DEV_WS2812_Show(&dev));
}

static void test_show_rejects_null_and_uninitialized(void)
{
    DEV_WS2812_s dev = {0};
    TEST_ASSERT_FALSE(DEV_WS2812_Show(NULL));
    TEST_ASSERT_FALSE(DEV_WS2812_Show(&dev));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_rejects_bad_arguments_and_short_buffer);
    RUN_TEST(test_init_blanks_pixels_and_latch);
    RUN_TEST(test_set_pixel_encodes_grb_msb_first_without_touching_neighbor);
    RUN_TEST(test_set_pixel_bounds_and_set_all_copy_exactly);
    RUN_TEST(test_show_captures_complete_frame_and_propagates_result);
    RUN_TEST(test_show_rejects_null_and_uninitialized);
    return UNITY_END();
}

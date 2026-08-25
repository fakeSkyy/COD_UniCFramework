/**
 * @file test_dev_bmi088_store.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "unity.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "dev_bmi088_store.h"
#include "device_test_support.h"
#include "mock_bmi088_peer.h"
#include "util_crc.h"

typedef struct
{
    uint32_t magic;
    float    bias[3];
    uint16_t version;
    uint16_t crc;
    uint8_t  pad[12];
} Test_Record_s;

static Test_Record_s flash_record;
static uint8_t       written[DEV_BMI088_STORE_SIZE];
static bool          read_result;
static bool          erased_result;
static bool          erase_result;
static bool          write_result;
static float         supplied_bias[3];
static float         installed_bias[3];
static unsigned      install_count;

static uint16_t record_crc(const Test_Record_s* record)
{
    return UTIL_CRC16_Calc((const uint8_t*) record, offsetof(Test_Record_s, crc), UTIL_CRC16_INIT);
}

static bool flash_read(Flash_Instance_s* flash, uint32_t off, uint8_t* dst, size_t len, int calls)
{
    (void) flash;
    (void) calls;
    TEST_ASSERT_EQUAL_HEX32(0x40u, off);
    TEST_ASSERT_EQUAL_UINT(DEV_BMI088_STORE_SIZE, len);
    if (read_result)
    {
        memcpy(dst, &flash_record, len);
    }
    return read_result;
}

static bool flash_is_erased(Flash_Instance_s* flash, uint32_t off, size_t len, int calls)
{
    (void) flash;
    (void) off;
    (void) len;
    (void) calls;
    return erased_result;
}

static bool flash_erase(Flash_Instance_s* flash, uint32_t off, int calls)
{
    (void) flash;
    (void) off;
    (void) calls;
    return erase_result;
}

static bool flash_write(Flash_Instance_s* flash, uint32_t off, const uint8_t* src, size_t len,
                        int calls)
{
    (void) flash;
    (void) calls;
    TEST_ASSERT_EQUAL_HEX32(0x40u, off);
    TEST_ASSERT_EQUAL_UINT(DEV_BMI088_STORE_SIZE, len);
    memcpy(written, src, len);
    return write_result;
}

static void set_bias(DEV_BMI088_s* imu, const float* bias, int calls)
{
    (void) imu;
    (void) calls;
    memcpy(installed_bias, bias, sizeof(installed_bias));
    install_count++;
}

static void get_bias(const DEV_BMI088_s* imu, float* out, int calls)
{
    (void) imu;
    (void) calls;
    memcpy(out, supplied_bias, sizeof(supplied_bias));
}

static void valid_record(void)
{
    memset(&flash_record, 0, sizeof(flash_record));
    flash_record.magic   = 0xB1A5C0DEu;
    flash_record.bias[0] = 0.1f;
    flash_record.bias[1] = -0.2f;
    flash_record.bias[2] = 0.5f;
    flash_record.version = 1u;
    flash_record.crc     = record_crc(&flash_record);
}

void setUp(void)
{
    DEVICE_CMock_Init();
    mock_bmi088_peer_Init();
    memset(written, 0xAA, sizeof(written));
    memset(installed_bias, 0, sizeof(installed_bias));
    supplied_bias[0] = 0.1f;
    supplied_bias[1] = -0.2f;
    supplied_bias[2] = 0.3f;
    read_result = erased_result = erase_result = write_result = true;
    install_count                                             = 0u;
}

void tearDown(void)
{
    DEVICE_CMock_Verify();
    mock_bmi088_peer_Verify();
    DEVICE_CMock_Destroy();
    mock_bmi088_peer_Destroy();
}

static void test_load_rejects_null_and_flash_read_failure(void)
{
    DEV_BMI088_s     imu   = {0};
    Flash_Instance_s flash = {0};
    TEST_ASSERT_FALSE(DEV_BMI088_LoadBias(NULL, &flash, 0x40u));
    TEST_ASSERT_FALSE(DEV_BMI088_LoadBias(&imu, NULL, 0x40u));
    read_result = false;
    PLAT_Flash_Read_StubWithCallback(flash_read);
    TEST_ASSERT_FALSE(DEV_BMI088_LoadBias(&imu, &flash, 0x40u));
}

static void test_load_valid_record_uses_real_crc_and_installs_boundary_bias(void)
{
    DEV_BMI088_s     imu   = {0};
    Flash_Instance_s flash = {0};
    valid_record();
    PLAT_Flash_Read_StubWithCallback(flash_read);
    DEV_BMI088_SetGyroBias_StubWithCallback(set_bias);
    TEST_ASSERT_TRUE(DEV_BMI088_LoadBias(&imu, &flash, 0x40u));
    TEST_ASSERT_EQUAL_UINT(1u, install_count);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.5f, installed_bias[2]);
}

static void test_load_rejects_magic_version_crc_and_implausible_bias(void)
{
    DEV_BMI088_s     imu   = {0};
    Flash_Instance_s flash = {0};
    PLAT_Flash_Read_StubWithCallback(flash_read);
    valid_record();
    flash_record.magic++;
    TEST_ASSERT_FALSE(DEV_BMI088_LoadBias(&imu, &flash, 0x40u));
    valid_record();
    flash_record.version++;
    flash_record.crc = record_crc(&flash_record);
    TEST_ASSERT_FALSE(DEV_BMI088_LoadBias(&imu, &flash, 0x40u));
    valid_record();
    flash_record.crc ^= 1u;
    TEST_ASSERT_FALSE(DEV_BMI088_LoadBias(&imu, &flash, 0x40u));
    valid_record();
    flash_record.bias[1] = 0.5001f;
    flash_record.crc     = record_crc(&flash_record);
    TEST_ASSERT_FALSE(DEV_BMI088_LoadBias(&imu, &flash, 0x40u));
    valid_record();
    flash_record.bias[1] = NAN;
    flash_record.crc     = record_crc(&flash_record);
    TEST_ASSERT_FALSE(DEV_BMI088_LoadBias(&imu, &flash, 0x40u));
    TEST_ASSERT_EQUAL_UINT(0u, install_count);
}

static void install_save_mocks(void)
{
    DEV_BMI088_GetGyroBias_StubWithCallback(get_bias);
    PLAT_Flash_IsErased_StubWithCallback(flash_is_erased);
    PLAT_Flash_EraseSector_StubWithCallback(flash_erase);
    PLAT_Flash_Write_StubWithCallback(flash_write);
}

static void test_save_rejects_null_uncalibrated_and_nonfinite_bias(void)
{
    DEV_BMI088_s     imu   = {0};
    Flash_Instance_s flash = {0};
    TEST_ASSERT_FALSE(DEV_BMI088_SaveBias(NULL, &flash, 0x40u));
    TEST_ASSERT_FALSE(DEV_BMI088_SaveBias(&imu, NULL, 0x40u));
    TEST_ASSERT_FALSE(DEV_BMI088_SaveBias(&imu, &flash, 0x40u));
    imu.calibrated   = true;
    supplied_bias[1] = INFINITY;
    DEV_BMI088_GetGyroBias_StubWithCallback(get_bias);
    TEST_ASSERT_FALSE(DEV_BMI088_SaveBias(&imu, &flash, 0x40u));
}

static void test_save_blank_writes_deterministic_32_byte_wire_image_without_erase(void)
{
    DEV_BMI088_s     imu   = {.calibrated = true};
    Flash_Instance_s flash = {0};
    install_save_mocks();
    TEST_ASSERT_TRUE(DEV_BMI088_SaveBias(&imu, &flash, 0x40u));
    const Test_Record_s* record = (const Test_Record_s*) (const void*) written;
    TEST_ASSERT_EQUAL_HEX32(0xB1A5C0DEu, record->magic);
    TEST_ASSERT_EQUAL_UINT16(1u, record->version);
    TEST_ASSERT_EQUAL_UINT16(record_crc(record), record->crc);
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(supplied_bias, record->bias, 3u);
    for (unsigned i = 0u; i < sizeof(record->pad); i++)
    {
        TEST_ASSERT_EQUAL_HEX8(0u, record->pad[i]);
    }
    TEST_ASSERT_EQUAL_UINT(0u, PLAT_Flash_EraseSector_CallCount());
}

static void test_save_nonblank_erases_before_write_and_stops_on_erase_failure(void)
{
    DEV_BMI088_s     imu   = {.calibrated = true};
    Flash_Instance_s flash = {0};
    install_save_mocks();
    erased_result = false;
    TEST_ASSERT_TRUE(DEV_BMI088_SaveBias(&imu, &flash, 0x40u));
    TEST_ASSERT_EQUAL_UINT(1u, PLAT_Flash_EraseSector_CallCount());
    TEST_ASSERT_EQUAL_UINT(1u, PLAT_Flash_Write_CallCount());

    erase_result = false;
    TEST_ASSERT_FALSE(DEV_BMI088_SaveBias(&imu, &flash, 0x40u));
    TEST_ASSERT_EQUAL_UINT(2u, PLAT_Flash_EraseSector_CallCount());
    TEST_ASSERT_EQUAL_UINT(1u, PLAT_Flash_Write_CallCount());
}

static void test_save_propagates_write_failure(void)
{
    DEV_BMI088_s     imu   = {.calibrated = true};
    Flash_Instance_s flash = {0};
    install_save_mocks();
    write_result = false;
    TEST_ASSERT_FALSE(DEV_BMI088_SaveBias(&imu, &flash, 0x40u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_load_rejects_null_and_flash_read_failure);
    RUN_TEST(test_load_valid_record_uses_real_crc_and_installs_boundary_bias);
    RUN_TEST(test_load_rejects_magic_version_crc_and_implausible_bias);
    RUN_TEST(test_save_rejects_null_uncalibrated_and_nonfinite_bias);
    RUN_TEST(test_save_blank_writes_deterministic_32_byte_wire_image_without_erase);
    RUN_TEST(test_save_nonblank_erases_before_write_and_stops_on_erase_failure);
    RUN_TEST(test_save_propagates_write_failure);
    return UNITY_END();
}

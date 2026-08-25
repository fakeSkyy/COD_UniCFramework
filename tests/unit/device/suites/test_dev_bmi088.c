/**
 * @file test_dev_bmi088.c
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#include "unity.h"

#include <math.h>
#include <string.h>

#include "dev_bmi088.h"
#include "dev_bmi088_reg.h"
#include "device_test_support.h"

static SPI_Instance_s accel_spi;
static SPI_Instance_s gyro_spi;
static DWT_Instance_s dwt;
static uint8_t        last_addr[2];
static uint8_t        read_phase[2];
static uint8_t        registers[2][256];
static bool           select_ok;
static bool           wrong_acc_id;
static bool           wrong_gyro_id;
static bool           stationary_gyro;
static unsigned       accel_dummy_reads;
static unsigned       delay_1ms_count;

static unsigned die_of(SPI_Instance_s* spi) { return (spi == &accel_spi) ? 0u : 1u; }

static bool spi_select(SPI_Instance_s* spi, int calls)
{
    (void) spi;
    (void) calls;
    return select_ok;
}

static void spi_deselect(SPI_Instance_s* spi, int calls)
{
    (void) spi;
    (void) calls;
}

static bool spi_send(SPI_Instance_s* spi, const uint8_t* tx, uint16_t len, uint32_t timeout,
                     int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_UINT32(10u, timeout);
    unsigned die = die_of(spi);
    if (len == 1u)
    {
        last_addr[die]  = (uint8_t) (tx[0] & 0x7Fu);
        read_phase[die] = 0u;
    }
    else if (len == 2u)
    {
        registers[die][tx[0] & 0x7Fu] = tx[1];
    }
    else
    {
        TEST_FAIL_MESSAGE("unexpected BMI088 SPI send length");
    }
    return true;
}

static bool spi_receive(SPI_Instance_s* spi, uint8_t* rx, uint16_t len, uint32_t timeout, int calls)
{
    (void) calls;
    TEST_ASSERT_EQUAL_UINT32(10u, timeout);
    unsigned die = die_of(spi);
    if (die == 0u && read_phase[die]++ == 0u)
    {
        TEST_ASSERT_EQUAL_UINT16(1u, len);
        rx[0] = 0xA5u;
        accel_dummy_reads++;
        return true;
    }

    uint8_t reg = last_addr[die];
    if (reg == BMI088_ACC_CHIP_ID && die == 0u && len == 1u)
    {
        rx[0] = wrong_acc_id ? 0u : BMI088_ACC_CHIP_ID_VALUE;
    }
    else if (reg == BMI088_GYRO_CHIP_ID && die == 1u && len == 1u)
    {
        rx[0] = wrong_gyro_id ? 0u : BMI088_GYRO_CHIP_ID_VALUE;
    }
    else if (die == 0u && reg == BMI088_ACC_XOUT_L && len == 6u)
    {
        const uint8_t sample[6] = {0x00u, 0x40u, 0x00u, 0xC0u, 0xFFu, 0x7Fu};
        memcpy(rx, sample, sizeof(sample));
    }
    else if (die == 1u && reg == BMI088_GYRO_CHIP_ID && len == 8u)
    {
        const uint8_t moving[8] = {
            BMI088_GYRO_CHIP_ID_VALUE, 0u, 0xE8u, 0x03u, 0x18u, 0xFCu, 0x00u, 0x00u};
        const uint8_t still[8] = {BMI088_GYRO_CHIP_ID_VALUE, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
        memcpy(rx, stationary_gyro ? still : moving, sizeof(moving));
    }
    else if (die == 0u && reg == BMI088_ACC_TEMP_MSB && len == 2u)
    {
        rx[0] = 0x08u;
        rx[1] = 0x00u;
    }
    else if (len == 1u)
    {
        rx[0] = registers[die][reg];
    }
    else
    {
        memset(rx, 0, len);
    }
    return true;
}

static void delay_ms(DWT_Instance_s* timebase, uint32_t ms, int calls)
{
    (void) timebase;
    (void) calls;
    if (ms == 1u)
    {
        delay_1ms_count++;
    }
}

static DEV_BMI088_Cfg_s cfg(void)
{
    DEV_BMI088_Cfg_s out = {0};
    out.spi_accel        = &accel_spi;
    out.spi_gyro         = &gyro_spi;
    out.timebase         = &dwt;
    out.acc_range        = DEV_BMI088_ACC_RANGE_6G;
    out.gyro_range       = DEV_BMI088_GYRO_RANGE_2000;
    out.max_attempts     = 1u;
    out.temp_divider     = 100u;
    return out;
}

static void install_spi_script(void)
{
    PLAT_SPI_Select_StubWithCallback(spi_select);
    PLAT_SPI_Deselect_StubWithCallback(spi_deselect);
    PLAT_SPI_Send_StubWithCallback(spi_send);
    PLAT_SPI_Receive_StubWithCallback(spi_receive);
    PLAT_DWT_Delay_us_Ignore();
    PLAT_DWT_Delay_ms_StubWithCallback(delay_ms);
    PLAT_DWT_GetTimeline_ms_IgnoreAndReturn(123u);
}

void setUp(void)
{
    DEVICE_CMock_Init();
    memset(&accel_spi, 0, sizeof(accel_spi));
    memset(&gyro_spi, 0, sizeof(gyro_spi));
    memset(&dwt, 0, sizeof(dwt));
    memset(last_addr, 0, sizeof(last_addr));
    memset(read_phase, 0, sizeof(read_phase));
    memset(registers, 0, sizeof(registers));
    select_ok    = true;
    wrong_acc_id = wrong_gyro_id = false;
    stationary_gyro              = false;
    accel_dummy_reads = delay_1ms_count = 0u;
}

void tearDown(void)
{
    DEVICE_CMock_Verify();
    DEVICE_CMock_Destroy();
}

static void test_init_rejects_null_handles_and_ranges_and_clears_state(void)
{
    DEV_BMI088_s imu;
    memset(&imu, 0xA5, sizeof(imu));
    DEV_BMI088_Cfg_s conf = cfg();
    TEST_ASSERT_EQUAL(DEV_BMI088_ERR_ARG, DEV_BMI088_Init(NULL, &conf));
    TEST_ASSERT_EQUAL(DEV_BMI088_ERR_ARG, DEV_BMI088_Init(&imu, NULL));
    TEST_ASSERT_FALSE(imu.initialized);
    conf.spi_accel = NULL;
    TEST_ASSERT_EQUAL(DEV_BMI088_ERR_ARG, DEV_BMI088_Init(&imu, &conf));
    conf           = cfg();
    conf.acc_range = (DEV_BMI088_AccRange_e) 99;
    TEST_ASSERT_EQUAL(DEV_BMI088_ERR_ARG, DEV_BMI088_Init(&imu, &conf));
}

static void test_init_reports_select_and_chip_id_failures(void)
{
    DEV_BMI088_s     imu;
    DEV_BMI088_Cfg_s conf = cfg();
    install_spi_script();
    select_ok = false;
    TEST_ASSERT_EQUAL(DEV_BMI088_ERR_SPI, DEV_BMI088_Init(&imu, &conf));
    select_ok    = true;
    wrong_acc_id = true;
    TEST_ASSERT_EQUAL(DEV_BMI088_ERR_ACC_ID, DEV_BMI088_Init(&imu, &conf));
    wrong_acc_id  = false;
    wrong_gyro_id = true;
    TEST_ASSERT_EQUAL(DEV_BMI088_ERR_GYRO_ID, DEV_BMI088_Init(&imu, &conf));
}

static void test_init_success_verifies_registers_defaults_and_watchdog(void)
{
    DEV_BMI088_s     imu;
    DEV_BMI088_Cfg_s conf = cfg();
    conf.max_attempts     = 0u;
    conf.temp_divider     = 0u;
    install_spi_script();
    TEST_ASSERT_EQUAL(DEV_BMI088_OK, DEV_BMI088_Init(&imu, &conf));
    TEST_ASSERT_TRUE(imu.initialized);
    TEST_ASSERT_EQUAL_UINT8(3u, imu.cfg.max_attempts);
    TEST_ASSERT_EQUAL_UINT16(100u, imu.cfg.temp_divider);
    TEST_ASSERT_EQUAL_STRING("imu", imu.wd.name);
    TEST_ASSERT_EQUAL_UINT32(100u, imu.wd.timeout_ms);
    TEST_ASSERT_TRUE(accel_dummy_reads > 0u);
    TEST_ASSERT_EQUAL_HEX8(BMI088_ACC_RANGE_6G, registers[0][BMI088_ACC_RANGE]);
    TEST_ASSERT_EQUAL_HEX8(BMI088_GYRO_RANGE_2000, registers[1][BMI088_GYRO_RANGE]);
}

static void test_read_parses_signed_samples_temperature_bias_and_kicks_watchdog(void)
{
    DEV_BMI088_s     imu;
    DEV_BMI088_Cfg_s conf = cfg();
    install_spi_script();
    TEST_ASSERT_EQUAL(DEV_BMI088_OK, DEV_BMI088_Init(&imu, &conf));
    const float bias[3] = {0.1f, -0.2f, 0.0f};
    DEV_BMI088_SetGyroBias(&imu, bias);
    TEST_ASSERT_TRUE(DEV_BMI088_Read(&imu));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 16384.0f * imu.acc_sensitivity, imu.accel[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -16384.0f * imu.acc_sensitivity, imu.accel[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 32767.0f * imu.acc_sensitivity, imu.accel[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1000.0f * imu.gyro_sensitivity - 0.1f, imu.gyro[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -1000.0f * imu.gyro_sensitivity + 0.2f, imu.gyro[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 31.0f, imu.temp_c);
    TEST_ASSERT_EQUAL_UINT32(1u, imu.read_count);
    TEST_ASSERT_EQUAL_UINT32(123u, imu.wd.last_kick_ms);
}

static void test_read_failure_preserves_cached_sample_and_counts_error(void)
{
    DEV_BMI088_s imu = {0};
    TEST_ASSERT_FALSE(DEV_BMI088_Read(NULL));
    TEST_ASSERT_FALSE(DEV_BMI088_Read(&imu));
    imu.initialized = true;
    imu.accel[0]    = 9.0f;
    select_ok       = false;
    PLAT_SPI_Select_StubWithCallback(spi_select);
    TEST_ASSERT_FALSE(DEV_BMI088_Read(&imu));
    TEST_ASSERT_EQUAL_FLOAT(9.0f, imu.accel[0]);
    TEST_ASSERT_EQUAL_UINT32(1u, imu.spi_error_count);
}

static void test_calibration_clamps_to_minimum_and_adopts_still_mean(void)
{
    DEV_BMI088_s     imu;
    DEV_BMI088_Cfg_s conf = cfg();
    install_spi_script();
    TEST_ASSERT_EQUAL(DEV_BMI088_OK, DEV_BMI088_Init(&imu, &conf));
    stationary_gyro = true;
    TEST_ASSERT_TRUE(DEV_BMI088_CalibrateGyro(&imu, 1u));
    TEST_ASSERT_EQUAL_UINT(100u, delay_1ms_count);
    TEST_ASSERT_TRUE(DEV_BMI088_IsCalibrated(&imu));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, imu.gyro_bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, imu.gyro_bias[1]);
}

static void test_bias_updates_only_finite_axes_and_get_handles_null(void)
{
    DEV_BMI088_s imu    = {0};
    imu.gyro_bias[0]    = 1.0f;
    imu.gyro_bias[1]    = 2.0f;
    imu.gyro_bias[2]    = 3.0f;
    const float bias[3] = {NAN, 4.0f, INFINITY};
    DEV_BMI088_SetGyroBias(&imu, bias);
    float out[3] = {0};
    DEV_BMI088_GetGyroBias(&imu, out);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, out[0]);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, out[1]);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, out[2]);
    TEST_ASSERT_TRUE(DEV_BMI088_IsCalibrated(&imu));
    DEV_BMI088_SetGyroBias(NULL, bias);
    DEV_BMI088_GetGyroBias(&imu, NULL);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_rejects_null_handles_and_ranges_and_clears_state);
    RUN_TEST(test_init_reports_select_and_chip_id_failures);
    RUN_TEST(test_init_success_verifies_registers_defaults_and_watchdog);
    RUN_TEST(test_read_parses_signed_samples_temperature_bias_and_kicks_watchdog);
    RUN_TEST(test_read_failure_preserves_cached_sample_and_counts_error);
    RUN_TEST(test_calibration_clamps_to_minimum_and_adopts_still_mean);
    RUN_TEST(test_bias_updates_only_finite_axes_and_get_handles_null);
    return UNITY_END();
}

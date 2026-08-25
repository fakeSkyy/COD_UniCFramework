/**
 * @file test_imu_bmi088_spi_dwt.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */
#define PLAT_ALLOW_CONSTRUCTION
#include "app_imu.h"
#include "app_telemetry.h"
#include "dev_bmi088_reg.h"
#include "dev_watchdog.h"
#include "mock_imu_contract.h"
#include "plat_dwt.h"
#include "plat_spi.h"
#include "plat_uart.h"
#include "unity.h"
#include "util_fast_math.h"

#include <setjmp.h>
#include <string.h>

#define CALIBRATION_READS 2000u
#define ALIGNMENT_READS 1u
#define DYNAMIC_GYRO_Z_RAW 3277

static SPI_Instance_s  accel_spi, gyro_spi;
static DWT_Instance_s  timebase;
static UART_Instance_s uart;
static unsigned        accel_ctx, gyro_ctx, dwt_ctx, uart_ctx;
static uint8_t         last_addr[2], read_phase[2], registers[2][256];
static bool            spi_available;
static uint32_t        cycle_count;
static uint64_t        us_count;
static IMPL_UART_TxCb  uart_tx_done;
static void*           uart_arg;
static uint8_t         telemetry_wire[32];
static unsigned        telemetry_sends;
static PLAT_Task_Entry task_entry;
static void*           task_arg;
static unsigned        delays, loop_limit;
static unsigned        sensor_reads;
static int16_t         dynamic_gyro_raw[3];
static jmp_buf         task_exit;

static unsigned die(void* ctx) { return ctx == &accel_ctx ? 0u : 1u; }
static bool     spi_send(void* ctx, const uint8_t* tx, uint16_t len, uint32_t timeout)
{
    TEST_ASSERT_EQUAL_UINT32(10u, timeout);
    unsigned d = die(ctx);
    if (!spi_available)
    {
        return false;
    }
    if (len == 1u)
    {
        last_addr[d]  = (uint8_t) (tx[0] & 0x7Fu);
        read_phase[d] = 0u;
    }
    else if (len == 2u)
    {
        registers[d][tx[0] & 0x7Fu] = tx[1];
    }
    else
    {
        TEST_FAIL_MESSAGE("unexpected BMI SPI send");
    }
    return true;
}
static void put_i16_le(uint8_t* dst, int16_t value)
{
    uint16_t raw = (uint16_t) value;
    dst[0]       = (uint8_t) raw;
    dst[1]       = (uint8_t) (raw >> 8);
}
static bool spi_receive(void* ctx, uint8_t* rx, uint16_t len, uint32_t timeout)
{
    TEST_ASSERT_EQUAL_UINT32(10u, timeout);
    unsigned d = die(ctx);
    if (!spi_available)
    {
        return false;
    }
    if (d == 0u && read_phase[d]++ == 0u)
    {
        TEST_ASSERT_EQUAL_UINT16(1u, len);
        rx[0] = 0xA5u;
        return true;
    }
    uint8_t reg = last_addr[d];
    if (d == 0u && reg == BMI088_ACC_CHIP_ID && len == 1u)
    {
        rx[0] = BMI088_ACC_CHIP_ID_VALUE;
    }
    else if (d == 1u && reg == BMI088_GYRO_CHIP_ID && len == 1u)
    {
        rx[0] = BMI088_GYRO_CHIP_ID_VALUE;
    }
    else if (d == 0u && reg == BMI088_ACC_XOUT_L && len == 6u)
    {
        const uint8_t sample[6] = {0, 0, 0, 0, 0xA8u, 0x2Au};
        sensor_reads++;
        memcpy(rx, sample, 6u);
    }
    else if (d == 1u && reg == BMI088_GYRO_CHIP_ID && len == 8u)
    {
        memset(rx, 0, len);
        rx[0] = BMI088_GYRO_CHIP_ID_VALUE;
        if (sensor_reads > CALIBRATION_READS + ALIGNMENT_READS)
        {
            put_i16_le(&rx[2], dynamic_gyro_raw[0]);
            put_i16_le(&rx[4], dynamic_gyro_raw[1]);
            put_i16_le(&rx[6], dynamic_gyro_raw[2]);
        }
    }
    else if (d == 0u && reg == BMI088_ACC_TEMP_MSB && len == 2u)
    {
        rx[0] = 0;
        rx[1] = 0;
    }
    else if (len == 1u)
    {
        rx[0] = registers[d][reg];
    }
    else
    {
        memset(rx, 0, len);
    }
    return true;
}
static bool spi_txrx(void* c, const uint8_t* t, uint8_t* r, uint16_t l, uint32_t o)
{
    (void) c;
    (void) t;
    (void) r;
    (void) l;
    (void) o;
    return false;
}
static bool spi_txa(void* c, const uint8_t* d, uint16_t l)
{
    (void) c;
    (void) d;
    (void) l;
    return false;
}
static bool spi_rxa(void* c, uint8_t* d, uint16_t l)
{
    (void) c;
    (void) d;
    (void) l;
    return false;
}
static bool spi_txrxa(void* c, const uint8_t* t, uint8_t* r, uint16_t l)
{
    (void) c;
    (void) t;
    (void) r;
    (void) l;
    return false;
}
static void spi_attach(void* c, IMPL_SPI_TxCb t, IMPL_SPI_RxCb r, IMPL_SPI_ErrCb e, void* a)
{
    (void) c;
    (void) t;
    (void) r;
    (void) e;
    (void) a;
}
static bool spi_select(void* c)
{
    (void) c;
    return spi_available;
}
static void spi_deselect(void* c) { (void) c; }
static bool spi_busy(void* c)
{
    (void) c;
    return false;
}
static const SPI_Ops_s spi_ops = {spi_send,  spi_receive, spi_txrx,   spi_txa,      spi_rxa,
                                  spi_txrxa, spi_attach,  spi_select, spi_deselect, spi_busy};
static uint32_t        dwt_cycle(void* c)
{
    (void) c;
    cycle_count += 1000u;
    return cycle_count;
}
static uint64_t dwt_cycle64(void* c)
{
    (void) c;
    return cycle_count;
}
static uint32_t dwt_freq(void* c)
{
    (void) c;
    return 1000000u;
}
static uint64_t dwt_us(void* c)
{
    (void) c;
    us_count += 1000u;
    return us_count;
}
static void dwt_delay(void* c, uint32_t us)
{
    (void) c;
    (void) us;
}
static const DWT_Ops_s dwt_ops = {dwt_cycle, dwt_cycle64, dwt_freq, dwt_us, dwt_delay};
static bool            uart_block(void* c, const uint8_t* d, uint16_t l, uint32_t t)
{
    (void) c;
    (void) d;
    (void) l;
    (void) t;
    return true;
}
static uint16_t uart_receive(void* c, uint8_t* d, uint16_t l, uint32_t t)
{
    (void) c;
    (void) d;
    (void) l;
    (void) t;
    return 0u;
}
static bool uart_async(void* c, const uint8_t* d, uint16_t l)
{
    (void) c;
    TEST_ASSERT_EQUAL_UINT16(32u, l);
    memcpy(telemetry_wire, d, l);
    telemetry_sends++;
    if (uart_tx_done != NULL)
    {
        uart_tx_done(uart_arg);
    }
    return true;
}
static void uart_attach(void* c, IMPL_UART_RxCb r, IMPL_UART_TxCb t, IMPL_UART_ErrCb e, void* a)
{
    (void) c;
    (void) r;
    (void) e;
    uart_tx_done = t;
    uart_arg     = a;
}
static bool uart_start(void* c, uint8_t* b, uint16_t s)
{
    (void) c;
    (void) b;
    (void) s;
    return true;
}
static void             uart_stop(void* c) { (void) c; }
static const UART_Ops_s uart_ops = {uart_block,  uart_receive, uart_async,
                                    uart_attach, uart_start,   uart_stop};
static bool task_create(Task_s* t, PLAT_Task_Entry entry, void* arg, const char* n, void* s,
                        size_t b, uint8_t p, int calls)
{
    (void) t;
    (void) s;
    (void) calls;
    TEST_ASSERT_EQUAL_STRING("imu", n);
    TEST_ASSERT_EQUAL_size_t(2048u, b);
    TEST_ASSERT_EQUAL_UINT8(5u, p);
    task_entry = entry;
    task_arg   = arg;
    return true;
}
static uint32_t task_tick(int calls)
{
    (void) calls;
    return 100u;
}
static bool task_delay(uint32_t* c, uint32_t p, int calls)
{
    (void) c;
    (void) calls;
    TEST_ASSERT_EQUAL_UINT32(1u, p);
    delays++;
    if (delays >= loop_limit)
    {
        longjmp(task_exit, 1);
    }
    return true;
}
static void task_suspend(Task_s* t, int calls)
{
    (void) calls;
    TEST_ASSERT_NULL(t);
    longjmp(task_exit, 2);
}

void setUp(void)
{
    mock_imu_contract_Init();
    memset(&accel_spi, 0, sizeof accel_spi);
    memset(&gyro_spi, 0, sizeof gyro_spi);
    memset(&timebase, 0, sizeof timebase);
    memset(&uart, 0, sizeof uart);
    memset(last_addr, 0, sizeof last_addr);
    memset(read_phase, 0, sizeof read_phase);
    memset(registers, 0, sizeof registers);
    memset(telemetry_wire, 0, sizeof telemetry_wire);
    spi_available       = true;
    cycle_count         = 0;
    us_count            = 0;
    uart_tx_done        = NULL;
    uart_arg            = NULL;
    telemetry_sends     = 0;
    task_entry          = NULL;
    task_arg            = NULL;
    delays              = 0;
    loop_limit          = 5;
    sensor_reads        = 0u;
    dynamic_gyro_raw[0] = 0;
    dynamic_gyro_raw[1] = 0;
    dynamic_gyro_raw[2] = DYNAMIC_GYRO_Z_RAW;
    DEV_Watchdog_Reset();
    TEST_ASSERT_TRUE(PLAT_SPI_Init(&accel_spi, &spi_ops, &accel_ctx));
    TEST_ASSERT_TRUE(PLAT_SPI_Init(&gyro_spi, &spi_ops, &gyro_ctx));
    TEST_ASSERT_TRUE(PLAT_DWT_Init(&timebase, &dwt_ops, &dwt_ctx));
    TEST_ASSERT_TRUE(PLAT_UART_Init(&uart, &uart_ops, &uart_ctx));
    Board_ImuAccel_IgnoreAndReturn(&accel_spi);
    Board_ImuGyro_IgnoreAndReturn(&gyro_spi);
    Board_Timebase_IgnoreAndReturn(&timebase);
    Board_DebugUart_IgnoreAndReturn(&uart);
    PLAT_Task_Create_StubWithCallback(task_create);
    PLAT_Task_TickNow_StubWithCallback(task_tick);
    PLAT_Task_DelayUntil_StubWithCallback(task_delay);
    PLAT_Task_Suspend_StubWithCallback(task_suspend);
    App_Indicator_SetFault_Ignore();
    UTIL_Log_Write_Ignore();
}
void tearDown(void)
{
    mock_imu_contract_Verify();
    mock_imu_contract_Destroy();
}
static int run_task(void)
{
    TEST_ASSERT_TRUE(App_Imu_StartTask(5u));
    TEST_ASSERT_NOT_NULL(task_entry);
    int why = setjmp(task_exit);
    if (why == 0)
    {
        task_entry(task_arg);
        TEST_FAIL_MESSAGE("imu task returned");
    }
    return why;
}
static void test_sample_to_ahrs_to_telemetry(void)
{
    loop_limit = 20u;
    TEST_ASSERT_EQUAL_INT(1, run_task());
    TEST_ASSERT_EQUAL_UINT(CALIBRATION_READS + ALIGNMENT_READS + loop_limit, sensor_reads);
    TEST_ASSERT_TRUE(App_Imu_Online());

    const float* quat = App_Imu_Quat();
    const float* rate = App_Imu_Rate();
    float        yaw  = App_Imu_Yaw();
    float        channels[7];

    TEST_ASSERT_NOT_NULL(quat);
    TEST_ASSERT_NOT_NULL(rate);
    TEST_ASSERT_TRUE(yaw > 0.05f);
    TEST_ASSERT_TRUE(quat[3] > 0.02f);
    TEST_ASSERT_TRUE(rate[2] > 3.0f);

    TEST_ASSERT_EQUAL_UINT(4u, telemetry_sends);
    memcpy(channels, telemetry_wire, sizeof channels);
    TEST_ASSERT_TRUE(channels[2] > 3.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.25f, yaw * UTIL_RAD_TO_DEG, channels[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 200.0f, channels[5]);
    TEST_ASSERT_EQUAL_UINT32(0u, App_Telemetry_Skipped());
    TEST_ASSERT_EQUAL_HEX8(0x00u, telemetry_wire[28]);
    TEST_ASSERT_EQUAL_HEX8(0x00u, telemetry_wire[29]);
    TEST_ASSERT_EQUAL_HEX8(0x80u, telemetry_wire[30]);
    TEST_ASSERT_EQUAL_HEX8(0x7Fu, telemetry_wire[31]);
    TEST_ASSERT_TRUE(DEV_Watchdog_Count() >= 1u);
}
static void test_bottom_failure_suspends_offline(void)
{
    spi_available = false;
    loop_limit    = 0u;
    TEST_ASSERT_EQUAL_INT(2, run_task());
    TEST_ASSERT_FALSE(App_Imu_Online());
    TEST_ASSERT_NULL(App_Imu_Quat());
    TEST_ASSERT_EQUAL_UINT(0u, telemetry_sends);
}
int main(int argc, char** argv)
{
    if (argc != 2)
    {
        return 2;
    }
    UNITY_BEGIN();
    if (strcmp(argv[1], "sample_to_ahrs_to_telemetry") == 0)
    {
        RUN_TEST(test_sample_to_ahrs_to_telemetry);
    }
    else if (strcmp(argv[1], "bottom_failure_suspends_offline") == 0)
    {
        RUN_TEST(test_bottom_failure_suspends_offline);
    }
    else
    {
        return 2;
    }
    return UNITY_END();
}

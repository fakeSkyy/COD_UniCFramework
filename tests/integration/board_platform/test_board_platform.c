/**
 * @file test_board_platform.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */
#include "board.h"
#include "fdcan.h"
#include "main.h"
#include "mock_board_backend_contract.h"
#include "spi.h"
#include "tim.h"
#include "unity.h"
#include "usart.h"
#include <stdlib.h>

#include "support/alloc/host_alloc_tracker.h"

static int      contexts[9];
static unsigned spi_attaches, uart_attaches, can_attaches;
static uint32_t dwt_cycle(void* c)
{
    (void) c;
    return 10u;
}
static uint64_t dwt_cycle64(void* c)
{
    (void) c;
    return 10u;
}
static uint32_t dwt_freq(void* c)
{
    (void) c;
    return 1000000u;
}
static uint64_t dwt_us(void* c)
{
    (void) c;
    return 10u;
}
static void dwt_delay(void* c, uint32_t u)
{
    (void) c;
    (void) u;
}
static const DWT_Ops_s dwt_ops = {dwt_cycle, dwt_cycle64, dwt_freq, dwt_us, dwt_delay};
static bool            spi_tx(void* c, const uint8_t* d, uint16_t l, uint32_t t)
{
    (void) c;
    (void) d;
    (void) l;
    (void) t;
    return true;
}
static bool spi_rx(void* c, uint8_t* d, uint16_t l, uint32_t t)
{
    (void) c;
    (void) d;
    (void) l;
    (void) t;
    return true;
}
static bool spi_txrx(void* c, const uint8_t* t, uint8_t* r, uint16_t l, uint32_t o)
{
    (void) c;
    (void) t;
    (void) r;
    (void) l;
    (void) o;
    return true;
}
static bool spi_txa(void* c, const uint8_t* d, uint16_t l)
{
    (void) c;
    (void) d;
    (void) l;
    return true;
}
static bool spi_rxa(void* c, uint8_t* d, uint16_t l)
{
    (void) c;
    (void) d;
    (void) l;
    return true;
}
static bool spi_txrxa(void* c, const uint8_t* t, uint8_t* r, uint16_t l)
{
    (void) c;
    (void) t;
    (void) r;
    (void) l;
    return true;
}
static void spi_attach(void* c, IMPL_SPI_TxCb t, IMPL_SPI_RxCb r, IMPL_SPI_ErrCb e, void* a)
{
    (void) c;
    (void) t;
    (void) r;
    (void) e;
    (void) a;
    spi_attaches++;
}
static bool spi_sel(void* c)
{
    (void) c;
    return true;
}
static void spi_desel(void* c) { (void) c; }
static bool spi_busy(void* c)
{
    (void) c;
    return false;
}
static const SPI_Ops_s spi_ops = {spi_tx,    spi_rx,     spi_txrx, spi_txa,   spi_rxa,
                                  spi_txrxa, spi_attach, spi_sel,  spi_desel, spi_busy};
static bool            uart_tx(void* c, const uint8_t* d, uint16_t l, uint32_t t)
{
    (void) c;
    (void) d;
    (void) l;
    (void) t;
    return true;
}
static uint16_t uart_rx(void* c, uint8_t* d, uint16_t l, uint32_t t)
{
    (void) c;
    (void) d;
    (void) t;
    return l;
}
static bool uart_txa(void* c, const uint8_t* d, uint16_t l)
{
    (void) c;
    (void) d;
    (void) l;
    return true;
}
static void uart_attach(void* c, IMPL_UART_RxCb r, IMPL_UART_TxCb t, IMPL_UART_ErrCb e, void* a)
{
    (void) c;
    (void) r;
    (void) t;
    (void) e;
    (void) a;
    uart_attaches++;
}
static bool uart_start(void* c, uint8_t* b, uint16_t s)
{
    (void) c;
    (void) b;
    (void) s;
    return true;
}
static void             uart_stop(void* c) { (void) c; }
static const UART_Ops_s uart_ops = {uart_tx, uart_rx, uart_txa, uart_attach, uart_start, uart_stop};
static bool             pwm_start(void* c)
{
    (void) c;
    return true;
}
static void pwm_stop(void* c) { (void) c; }
static void pwm_compare(void* c, uint32_t v)
{
    (void) c;
    (void) v;
}
static uint32_t pwm_period(void* c)
{
    (void) c;
    return 1000u;
}
static uint32_t pwm_freq(void* c, uint32_t f)
{
    (void) c;
    return f;
}
static const PWM_Ops_s pwm_ops = {pwm_start, pwm_stop, pwm_compare, pwm_period, pwm_freq};
static bool            flash_read(void* c, uint32_t o, uint8_t* d, size_t l)
{
    (void) c;
    (void) o;
    (void) d;
    (void) l;
    return true;
}
static bool flash_write(void* c, uint32_t o, const uint8_t* d, size_t l)
{
    (void) c;
    (void) o;
    (void) d;
    (void) l;
    return true;
}
static bool flash_erase(void* c, uint32_t o)
{
    (void) c;
    (void) o;
    return true;
}
static uint32_t flash_sector(void* c, uint32_t o)
{
    (void) c;
    (void) o;
    return 7u;
}
static uint32_t flash_base(void* c, uint32_t o)
{
    (void) c;
    (void) o;
    return 0u;
}
static uint32_t flash_sector_size(void* c, uint32_t o)
{
    (void) c;
    (void) o;
    return 131072u;
}
static uint32_t flash_size(void* c)
{
    (void) c;
    return 131072u;
}
static bool flash_erased(void* c, uint32_t o, size_t l)
{
    (void) c;
    (void) o;
    (void) l;
    return true;
}
static uint32_t flash_gran(void* c)
{
    (void) c;
    return 32u;
}
static const Flash_Ops_s flash_ops = {flash_read,   flash_write,  flash_erase,
                                      flash_sector, flash_base,   flash_sector_size,
                                      flash_size,   flash_erased, flash_gran};
static bool              can_send(void* c, const uint8_t* d, uint8_t l)
{
    (void) c;
    (void) d;
    (void) l;
    return true;
}
static bool can_send_to(void* c, uint32_t i, const uint8_t* d, uint8_t l)
{
    (void) c;
    (void) i;
    (void) d;
    (void) l;
    return true;
}
static void can_attach(void* c, IMPL_CAN_RxCb r, IMPL_CAN_ErrCb e, void* a)
{
    (void) c;
    (void) r;
    (void) e;
    TEST_ASSERT_NOT_NULL(a);
    can_attaches++;
}
static bool can_start(void* c)
{
    (void) c;
    return true;
}
static uint32_t can_free(void* c)
{
    (void) c;
    return 3u;
}
static const CAN_Ops_s can_ops = {can_send, can_send_to, can_attach, can_start, can_free};
static void*           alloc_cb(size_t n, int calls)
{
    (void) calls;
    return TEST_TrackedMalloc(n);
}
static void free_cb(void* p, int calls)
{
    (void) calls;
    TEST_TrackedFree(p);
}

void setUp(void)
{
    mock_board_backend_contract_Init();
    spi_attaches = uart_attaches = can_attaches = 0u;
    PLAT_malloc_StubWithCallback(alloc_cb);
    PLAT_free_StubWithCallback(free_cb);
}
void tearDown(void)
{
    mock_board_backend_contract_Verify();
    mock_board_backend_contract_Destroy();
}
/**
 * @brief Expect the reverse-order teardown that opens every Board_Init call.
 *
 * Board_Init tears down before it builds, on every call including the first,
 * so both tests below must expect these before their bring-up expectations —
 * enforce_strict_ordering fails the test otherwise. NULL is what a fresh
 * Board_Init actually tears down: nothing has been created yet in either
 * test, so every DestroyCtx here runs against a never-populated context.
 */
static void expect_teardown(void)
{
    HOST_FLASH_DestroyCtx_Expect(NULL);
    HOST_PWM_DestroyCtx_Expect(NULL);
    HOST_PWM_DestroyCtx_Expect(NULL);
    HOST_UART_DestroyCtx_Expect(NULL);
    HOST_SPI_DestroyCtx_Expect(NULL);
    HOST_SPI_DestroyCtx_Expect(NULL);
    HOST_SPI_DestroyCtx_Expect(NULL);
    HOST_DWT_DestroyCtx_Expect(NULL);
}
static void expect_all(void)
{
    expect_teardown();
    HOST_DWT_CreateCtx_ExpectAndReturn(SystemCoreClock, &contexts[0]);
    HOST_DWT_GetOps_ExpectAndReturn(&dwt_ops);
    HOST_SPI_CreateCtx_ExpectAndReturn(&hspi2, ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, SPI_XFER_IT,
                                       &contexts[1]);
    HOST_SPI_GetOps_ExpectAndReturn(&spi_ops);
    HOST_SPI_CreateCtx_ExpectAndReturn(&hspi2, GYRO_CS_GPIO_Port, GYRO_CS_Pin, SPI_XFER_IT,
                                       &contexts[2]);
    HOST_SPI_GetOps_ExpectAndReturn(&spi_ops);
    HOST_SPI_CreateCtx_ExpectAndReturn(&hspi6, NULL, 0u, SPI_XFER_IT, &contexts[3]);
    HOST_SPI_GetOps_ExpectAndReturn(&spi_ops);
    HOST_UART_CreateCtx_ExpectAndReturn(&huart10, UART_XFER_IT, &contexts[4]);
    HOST_UART_GetOps_ExpectAndReturn(&uart_ops);
    HOST_PWM_CreateCtx_ExpectAndReturn(&htim12, TIM_CHANNEL_2, &contexts[5]);
    HOST_PWM_GetOps_ExpectAndReturn(&pwm_ops);
    HOST_PWM_CreateCtx_ExpectAndReturn(&htim3, TIM_CHANNEL_4, &contexts[6]);
    HOST_PWM_GetOps_ExpectAndReturn(&pwm_ops);
    HOST_FLASH_CreateCtx_ExpectAndReturn(7u, 1u, &contexts[7]);
    HOST_FLASH_GetOps_ExpectAndReturn(&flash_ops);
}
static void test_real_platform_init_order_accessors_and_can_factory(void)
{
    expect_all();
    TEST_ASSERT_TRUE(Board_Init());
    TEST_ASSERT_NULL(Board_FailedDevice());
    TEST_ASSERT_NOT_NULL(Board_Timebase());
    TEST_ASSERT_NOT_NULL(Board_ImuAccel());
    TEST_ASSERT_NOT_NULL(Board_ImuGyro());
    TEST_ASSERT_NOT_NULL(Board_StatusLed());
    TEST_ASSERT_NOT_NULL(Board_DebugUart());
    TEST_ASSERT_NOT_NULL(Board_BuzzerPWM());
    TEST_ASSERT_NOT_NULL(Board_ImuHeater());
    TEST_ASSERT_NOT_NULL(Board_ParamFlash());
    TEST_ASSERT_EQUAL_UINT(3u, spi_attaches);
    TEST_ASSERT_EQUAL_UINT(1u, uart_attaches);
    HOST_CAN_CreateCtx_ExpectAndReturn(&hfdcan2, 0x200u, 0x205u, &contexts[8]);
    HOST_CAN_GetOps_ExpectAndReturn(&can_ops);
    CAN_Instance_s* can = Board_CANCreate(BOARD_CAN2, 0x200u, 0x205u);
    TEST_ASSERT_NOT_NULL(can);
    TEST_ASSERT_EQUAL_UINT(1u, can_attaches);
}
static void test_first_context_error_short_circuits_remaining_definitions(void)
{
    /* This runs second in the same process, so board_devices.c's static state
     * still holds what the previous test built — its teardown must release
     * those exact pointers, not NULL, mirroring test_reinit_destroys_each_
     * previous_context_exactly_once in the unit suite. */
    HOST_FLASH_DestroyCtx_Expect(&contexts[7]);
    HOST_PWM_DestroyCtx_Expect(&contexts[6]);
    HOST_PWM_DestroyCtx_Expect(&contexts[5]);
    HOST_UART_DestroyCtx_Expect(&contexts[4]);
    HOST_SPI_DestroyCtx_Expect(&contexts[3]);
    HOST_SPI_DestroyCtx_Expect(&contexts[2]);
    HOST_SPI_DestroyCtx_Expect(&contexts[1]);
    HOST_DWT_DestroyCtx_Expect(&contexts[0]);
    HOST_DWT_CreateCtx_ExpectAndReturn(SystemCoreClock, &contexts[0]);
    HOST_DWT_GetOps_ExpectAndReturn(&dwt_ops);
    HOST_SPI_CreateCtx_ExpectAndReturn(&hspi2, ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, SPI_XFER_IT, NULL);
    TEST_ASSERT_FALSE(Board_Init());
    TEST_ASSERT_EQUAL_STRING("imu_accel", Board_FailedDevice());
    TEST_ASSERT_NOT_NULL(Board_Timebase());
    TEST_ASSERT_NULL(Board_ImuAccel());
    TEST_ASSERT_NULL(Board_DebugUart());
}
int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_real_platform_init_order_accessors_and_can_factory);
    RUN_TEST(test_first_context_error_short_circuits_remaining_definitions);
    return UNITY_END();
}

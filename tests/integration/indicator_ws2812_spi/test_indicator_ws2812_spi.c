/**
 * @file test_indicator_ws2812_spi.c
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */
#define PLAT_ALLOW_CONSTRUCTION
#include "app_indicator.h"
#include "dev_ws2812.h"
#include "mock_indicator_contract.h"
#include "plat_pwm.h"
#include "plat_spi.h"
#include "unity.h"

#include <setjmp.h>
#include <string.h>

static SPI_Instance_s  spi;
static int             backend_ctx;
static PLAT_Task_Entry task_entry;
static void*           task_arg;
static jmp_buf         done;
static unsigned        delay_calls;
static unsigned        loop_limit;
static unsigned        tx_calls;
static bool            fail_first;
static uint8_t         wire[DEV_WS2812_BUF_BYTES(1u)];
static uint8_t         first_wire[DEV_WS2812_BUF_BYTES(1u)];
static IMPL_SPI_TxCb   backend_tx_cb;
static IMPL_SPI_RxCb   backend_rx_cb;
static IMPL_SPI_ErrCb  backend_err_cb;
static void*           backend_cb_arg;
static SPI_Instance_s* user_tx_spi;
static SPI_Instance_s* user_rx_spi;
static SPI_Instance_s* user_err_spi;
static const uint8_t*  user_rx_data;
static uint16_t        user_rx_len;
static uint32_t        user_err_bits;
static unsigned        callback_sequence;
static PWM_Instance_s  pwm;
static bool            pwm_running;
static uint32_t        pwm_period;
static uint32_t        pwm_compare;
static uint32_t        pwm_freq;

static bool pwm_start(void* ctx)
{
    (void) ctx;
    pwm_running = true;
    return true;
}
static void pwm_stop(void* ctx)
{
    (void) ctx;
    pwm_running = false;
}
static void pwm_set_compare(void* ctx, uint32_t ccr)
{
    (void) ctx;
    pwm_compare = ccr;
}
static uint32_t pwm_get_period(void* ctx)
{
    (void) ctx;
    return pwm_period;
}
static uint32_t pwm_set_frequency(void* ctx, uint32_t freq_hz)
{
    (void) ctx;
    pwm_freq   = freq_hz;
    pwm_period = 1000u;
    return pwm_period;
}
static const PWM_Ops_s pwm_ops = {pwm_start, pwm_stop, pwm_set_compare, pwm_get_period,
                                  pwm_set_frequency};

static bool transmit(void* ctx, const uint8_t* data, uint16_t len, uint32_t timeout)
{
    (void) ctx;
    TEST_ASSERT_EQUAL_UINT16(sizeof wire, len);
    TEST_ASSERT_EQUAL_UINT32(50u, timeout);
    if (tx_calls == 0u)
    {
        memcpy(first_wire, data, len);
    }
    memcpy(wire, data, len);
    tx_calls++;
    return !(fail_first && tx_calls == 1u);
}
static bool receive(void* c, uint8_t* d, uint16_t l, uint32_t t)
{
    (void) c;
    (void) d;
    (void) l;
    (void) t;
    return true;
}
static bool txrx(void* c, const uint8_t* t, uint8_t* r, uint16_t l, uint32_t o)
{
    (void) c;
    (void) t;
    (void) r;
    (void) l;
    (void) o;
    return true;
}
static bool txa(void* c, const uint8_t* d, uint16_t l)
{
    (void) c;
    (void) d;
    (void) l;
    return true;
}
static bool rxa(void* c, uint8_t* d, uint16_t l)
{
    (void) c;
    (void) d;
    (void) l;
    return true;
}
static bool txrxa(void* c, const uint8_t* t, uint8_t* r, uint16_t l)
{
    (void) c;
    (void) t;
    (void) r;
    (void) l;
    return true;
}
static void attach(void* c, IMPL_SPI_TxCb t, IMPL_SPI_RxCb r, IMPL_SPI_ErrCb e, void* a)
{
    TEST_ASSERT_EQUAL_PTR(&backend_ctx, c);
    backend_tx_cb  = t;
    backend_rx_cb  = r;
    backend_err_cb = e;
    backend_cb_arg = a;
}
static bool select(void* c)
{
    (void) c;
    return true;
}
static void deselect(void* c) { (void) c; }
static bool busy(void* c)
{
    (void) c;
    return false;
}
static const SPI_Ops_s ops = {transmit, receive, txrx,   txa,      rxa,
                              txrxa,    attach,  select, deselect, busy};

static bool capture_create(Task_s* t, PLAT_Task_Entry entry, void* arg, const char* name,
                           void* stack, size_t bytes, uint8_t priority, int calls)
{
    (void) t;
    (void) stack;
    (void) calls;
    TEST_ASSERT_EQUAL_STRING("indicator", name);
    TEST_ASSERT_EQUAL_size_t(1024u, bytes);
    TEST_ASSERT_EQUAL_UINT8(0u, priority);
    task_entry = entry;
    task_arg   = arg;
    return true;
}
static uint32_t tick_cb(int calls) { return (uint32_t) (calls + 1) * 25u; }
static bool     delay_cb(uint32_t* cursor, uint32_t period, int calls)
{
    (void) cursor;
    (void) calls;
    TEST_ASSERT_EQUAL_UINT32(25u, period);
    delay_calls++;
    if (delay_calls >= loop_limit)
    {
        longjmp(done, 1);
    }
    return true;
}

void setUp(void)
{
    mock_indicator_contract_Init();
    memset(&spi, 0, sizeof spi);
    memset(wire, 0, sizeof wire);
    memset(first_wire, 0, sizeof first_wire);
    task_entry        = NULL;
    task_arg          = NULL;
    delay_calls       = 0;
    loop_limit        = 1;
    tx_calls          = 0;
    fail_first        = false;
    backend_tx_cb     = NULL;
    backend_rx_cb     = NULL;
    backend_err_cb    = NULL;
    backend_cb_arg    = NULL;
    user_tx_spi       = NULL;
    user_rx_spi       = NULL;
    user_err_spi      = NULL;
    user_rx_data      = NULL;
    user_rx_len       = 0u;
    user_err_bits     = 0u;
    callback_sequence = 0u;
    pwm_running       = false;
    pwm_period        = 1000u;
    pwm_compare       = 0u;
    pwm_freq          = 0u;
    TEST_ASSERT_TRUE(PLAT_SPI_Init(&spi, &ops, &backend_ctx));
    TEST_ASSERT_TRUE(PLAT_PWM_Init(&pwm, &pwm_ops, &pwm));
    PLAT_Task_Create_StubWithCallback(capture_create);
    PLAT_Task_TickNow_StubWithCallback(tick_cb);
    PLAT_Task_DelayUntil_StubWithCallback(delay_cb);
    UTIL_Log_Write_Ignore();
}
void tearDown(void)
{
    mock_indicator_contract_Verify();
    mock_indicator_contract_Destroy();
}
static void run_task(unsigned loops)
{
    loop_limit = loops;
    TEST_ASSERT_TRUE(App_Indicator_StartTask(0u));
    TEST_ASSERT_NOT_NULL(task_entry);
    Board_StatusLed_ExpectAndReturn(&spi);
    Board_BuzzerPWM_ExpectAndReturn(&pwm);
    if (setjmp(done) == 0)
    {
        task_entry(task_arg);
        TEST_FAIL_MESSAGE("indicator task returned");
    }
}
static void assert_channel_80(const uint8_t* frame, unsigned off)
{
    const uint8_t expected[8] = {0x60u, 0x78u, 0x60u, 0x78u, 0x60u, 0x60u, 0x60u, 0x60u};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame + off, 8u);
}
static void on_spi_tx(SPI_Instance_s* instance)
{
    TEST_ASSERT_EQUAL_UINT(0u, callback_sequence);
    callback_sequence++;
    user_tx_spi = instance;
}
static void on_spi_rx(SPI_Instance_s* instance, const uint8_t* data, uint16_t len)
{
    TEST_ASSERT_EQUAL_UINT(1u, callback_sequence);
    callback_sequence++;
    user_rx_spi  = instance;
    user_rx_data = data;
    user_rx_len  = len;
}
static void on_spi_error(SPI_Instance_s* instance, uint32_t err)
{
    TEST_ASSERT_EQUAL_UINT(2u, callback_sequence);
    callback_sequence++;
    user_err_spi  = instance;
    user_err_bits = err;
}
static void test_spi_callback_trampolines(void)
{
    const uint8_t  rx_data[] = {0x12u, 0x34u, 0x56u, 0x78u};
    const uint32_t err_bits  = IMPL_SPI_ERR_OVERRUN | IMPL_SPI_ERR_DMA;

    TEST_ASSERT_NOT_NULL(backend_tx_cb);
    TEST_ASSERT_NOT_NULL(backend_rx_cb);
    TEST_ASSERT_NOT_NULL(backend_err_cb);
    TEST_ASSERT_EQUAL_PTR(&spi, backend_cb_arg);

    PLAT_SPI_OnSendComplete(&spi, on_spi_tx);
    PLAT_SPI_OnReceive(&spi, on_spi_rx);
    PLAT_SPI_OnError(&spi, on_spi_error);

    backend_tx_cb(backend_cb_arg);
    backend_rx_cb(backend_cb_arg, rx_data, (uint16_t) sizeof rx_data);
    backend_err_cb(backend_cb_arg, err_bits);

    TEST_ASSERT_EQUAL_UINT(3u, callback_sequence);
    TEST_ASSERT_EQUAL_PTR(&spi, user_tx_spi);
    TEST_ASSERT_EQUAL_PTR(&spi, user_rx_spi);
    TEST_ASSERT_EQUAL_PTR(rx_data, user_rx_data);
    TEST_ASSERT_EQUAL_UINT16(sizeof rx_data, user_rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(rx_data, user_rx_data, sizeof rx_data);
    TEST_ASSERT_EQUAL_PTR(&spi, user_err_spi);
    TEST_ASSERT_EQUAL_HEX32(err_bits, user_err_bits);
}
static void test_heartbeat_wire(void)
{
    run_task(1u);
    TEST_ASSERT_EQUAL_UINT(1u, tx_calls);
    assert_channel_80(wire, 0u);
    for (unsigned i = 8u; i < 24u; i++)
    {
        TEST_ASSERT_EQUAL_HEX8(0x60u, wire[i]);
    }
    for (unsigned i = 24u; i < sizeof wire; i++)
    {
        TEST_ASSERT_EQUAL_HEX8(0u, wire[i]);
    }
}
static void test_fault_priority_and_send_failure(void)
{
    App_Indicator_Set(INDICATOR_CAN_LOST, true);
    App_Indicator_Set(INDICATOR_LOW_BATTERY, true);
    App_Indicator_SetFault(12u);
    TEST_ASSERT_EQUAL(INDICATOR_FAULT, App_Indicator_Active());
    fail_first = true;
    run_task(2u);
    TEST_ASSERT_TRUE(tx_calls >= 2u);
    for (unsigned i = 0u; i < 8u; i++)
    {
        TEST_ASSERT_EQUAL_HEX8(0x60u, first_wire[i]);
    }
    assert_channel_80(first_wire, 8u);
}
int main(int argc, char** argv)
{
    if (argc != 2)
    {
        return 2;
    }
    UNITY_BEGIN();
    if (strcmp(argv[1], "spi_callback_trampolines") == 0)
    {
        RUN_TEST(test_spi_callback_trampolines);
    }
    else if (strcmp(argv[1], "heartbeat_wire") == 0)
    {
        RUN_TEST(test_heartbeat_wire);
    }
    else if (strcmp(argv[1], "fault_priority_and_send_failure") == 0)
    {
        RUN_TEST(test_fault_priority_and_send_failure);
    }
    else
    {
        return 2;
    }
    return UNITY_END();
}

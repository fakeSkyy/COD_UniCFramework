/**
 * @file test_plat_gpio.c
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#define PLAT_ALLOW_CONSTRUCTION
#include "plat_gpio.h"

#include "platform_bsp_test_support.h"

static void*            backend_ctx = (void*) 0x6100u;
static const GPIO_Ops_s gpio_ops    = {
       .set    = PBSP_GPIO_Set,
       .reset  = PBSP_GPIO_Reset,
       .toggle = PBSP_GPIO_Toggle,
       .write  = PBSP_GPIO_Write,
       .read   = PBSP_GPIO_Read,
};

void setUp(void) { PlatformBsp_Test_MockInit(); }

void tearDown(void) { PlatformBsp_Test_MockVerify(); }

static void test_init_and_create_cover_allocator_paths(void)
{
    GPIO_Instance_s storage = {0};

    TEST_ASSERT_FALSE(PLAT_GPIO_Init(NULL, &gpio_ops, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_GPIO_Init(&storage, NULL, backend_ctx));
    TEST_ASSERT_FALSE(PLAT_GPIO_Init(&storage, &gpio_ops, NULL));
    PLAT_malloc_ExpectAndReturn(sizeof(GPIO_Instance_s), NULL);
    TEST_ASSERT_NULL(PLAT_GPIO_Create(&gpio_ops, backend_ctx));
    PLAT_malloc_ExpectAndReturn(sizeof(GPIO_Instance_s), &storage);
    PLAT_free_Expect(&storage);
    TEST_ASSERT_NULL(PLAT_GPIO_Create(NULL, backend_ctx));
    PLAT_malloc_ExpectAndReturn(sizeof(GPIO_Instance_s), &storage);
    TEST_ASSERT_EQUAL_PTR(&storage, PLAT_GPIO_Create(&gpio_ops, backend_ctx));
    TEST_ASSERT_EQUAL_PTR(&gpio_ops, storage.ops);
    TEST_ASSERT_EQUAL_PTR(backend_ctx, storage.ctx);
    TEST_ASSERT_NULL(storage.callback);
    TEST_ASSERT_NULL(storage.id);
}

static void test_all_gpio_ops_forward_exact_arguments_and_return(void)
{
    GPIO_Instance_s gpio;

    TEST_ASSERT_TRUE(PLAT_GPIO_Init(&gpio, &gpio_ops, backend_ctx));
    PBSP_GPIO_Set_Expect(backend_ctx);
    PLAT_GPIO_Set(&gpio);
    PBSP_GPIO_Reset_Expect(backend_ctx);
    PLAT_GPIO_Reset(&gpio);
    PBSP_GPIO_Toggle_Expect(backend_ctx);
    PLAT_GPIO_Toggle(&gpio);
    PBSP_GPIO_Write_Expect(backend_ctx, 0u);
    PLAT_GPIO_Write(&gpio, 0u);
    PBSP_GPIO_Write_Expect(backend_ctx, 7u);
    PLAT_GPIO_Write(&gpio, 7u);
    PBSP_GPIO_Read_ExpectAndReturn(backend_ctx, 1u);
    TEST_ASSERT_EQUAL_UINT8(1u, PLAT_GPIO_Read(&gpio));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_and_create_cover_allocator_paths);
    RUN_TEST(test_all_gpio_ops_forward_exact_arguments_and_return);
    return UNITY_END();
}

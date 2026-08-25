/**
 * @file test_impl_freertos_fault.c
 * @author Gao Xing
 * @date 2026/8/24
 * @version 1.0
 */

#include "rtos_test_support.h"

#include <string.h>
#include <sys/mman.h>

#include "rtos_fault.h"

void rtos_fault_dispatch(uint32_t exc_return, uint32_t kind);

typedef struct
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
} Test_Fault_Frame_s;

void setUp(void) { RTOS_Test_MockInit(); }
void tearDown(void) { RTOS_Test_MockVerify(); }

void test_fault_init_enables_configurable_faults_divide_trap_and_barriers(void)
{
    SCB->SHCSR = 0x10u;
    SCB->CCR   = 0x20u;

    RTOS_FaultInit();

    TEST_ASSERT_BITS_HIGH(SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk |
                              SCB_SHCSR_USGFAULTENA_Msk,
                          SCB->SHCSR);
    TEST_ASSERT_BITS_HIGH(SCB_CCR_DIV_0_TRP_Msk, SCB->CCR);
    TEST_ASSERT_BITS_HIGH(0x10u, SCB->SHCSR);
    TEST_ASSERT_BITS_HIGH(0x20u, SCB->CCR);
    TEST_ASSERT_EQUAL_UINT(1u, RTOS_Test_GetDSBCount());
    TEST_ASSERT_EQUAL_UINT(1u, RTOS_Test_GetISBCount());
}

void test_dispatch_selects_psp_and_prints_frame_and_fault_status(void)
{
    const uintptr_t     address = 0x20000000u;
    Test_Fault_Frame_s* frame   = mmap((void*) address, 4096u, PROT_READ | PROT_WRITE,
                                       MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);

    TEST_ASSERT_EQUAL_PTR((void*) address, frame);
    *frame = (Test_Fault_Frame_s){
        .r0  = 1u,
        .r1  = 2u,
        .r2  = 3u,
        .r3  = 4u,
        .r12 = 12u,
        .lr  = 0x11112222u,
        .pc  = 0x33334444u,
        .psr = 0x21000000u,
    };

    SCB->CFSR = SCB_CFSR_INVSTATE_Msk | SCB_CFSR_PRECISERR_Msk | SCB_CFSR_DACCVIOL_Msk |
                SCB_CFSR_DIVBYZERO_Msk;
    SCB->HFSR  = SCB_HFSR_FORCED_Msk | SCB_HFSR_VECTTBL_Msk;
    SCB->BFAR  = 0xDEADBEEFu;
    SCB->MMFAR = 0x20001234u;
    RTOS_Test_SetStackPointers(0u, (uint32_t) address);
    RTOS_Test_ArmFatalJump();

    if (setjmp(rtos_test_fatal_jump) == 0)
    {
        rtos_fault_dispatch(0x4u, 2u);
        TEST_FAIL_MESSAGE("fault reporter returned");
    }

    TEST_ASSERT_EQUAL_UINT(1u, RTOS_Test_GetDisableCount());
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "BusFault"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "33334444"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "11112222"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "INVSTATE"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "DIVBYZERO"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "DEADBEEF"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "20001234"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "escalated"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "VECTTBL"));

    TEST_ASSERT_EQUAL_INT(0, munmap(frame, 4096u));
}

void test_dispatch_handles_unknown_kind_null_frame_and_remaining_status_bits(void)
{
    SCB->CFSR = SCB_CFSR_UNDEFINSTR_Msk | SCB_CFSR_INVPC_Msk | SCB_CFSR_UNALIGNED_Msk |
                SCB_CFSR_IMPRECISERR_Msk | SCB_CFSR_IBUSERR_Msk | SCB_CFSR_IACCVIOL_Msk |
                SCB_CFSR_MSTKERR_Msk | SCB_CFSR_MUNSTKERR_Msk;
    RTOS_Test_SetStackPointers(0u, 0u);
    RTOS_Test_ArmFatalJump();

    if (setjmp(rtos_test_fatal_jump) == 0)
    {
        rtos_fault_dispatch(0u, UINT32_MAX);
        TEST_FAIL_MESSAGE("fault reporter returned");
    }

    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "unknown fault"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "UNDEFINSTR"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "INVPC"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "UNALIGNED"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "IMPRECISERR"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "IBUSERR"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "IACCVIOL"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "MSTKERR"));
    TEST_ASSERT_NOT_NULL(strstr(RTOS_Test_GetOutput(), "MUNSTKERR"));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_fault_init_enables_configurable_faults_divide_trap_and_barriers);
    RUN_TEST(test_dispatch_selects_psp_and_prints_frame_and_fault_status);
    RUN_TEST(test_dispatch_handles_unknown_kind_null_frame_and_remaining_status_bits);
    return UNITY_END();
}

#ifndef HOST_STM32H7XX_H
#define HOST_STM32H7XX_H

#include <stdint.h>

typedef struct
{
    uint32_t CPUID;
    uint32_t ICSR;
    uint32_t VTOR;
    uint32_t AIRCR;
    uint32_t SCR;
    uint32_t CCR;
    uint8_t  reserved0[4];
    uint32_t SHPR[3];
    uint32_t SHCSR;
    uint32_t CFSR;
    uint32_t HFSR;
    uint32_t DFSR;
    uint32_t MMFAR;
    uint32_t BFAR;
} SCB_Type;

extern SCB_Type* SCB;

#define SCB_HFSR_FORCED_Msk (1u << 30)
#define SCB_HFSR_VECTTBL_Msk (1u << 1)
#define SCB_CFSR_IACCVIOL_Msk (1u << 0)
#define SCB_CFSR_DACCVIOL_Msk (1u << 1)
#define SCB_CFSR_MUNSTKERR_Msk (1u << 3)
#define SCB_CFSR_MSTKERR_Msk (1u << 4)
#define SCB_CFSR_IBUSERR_Msk (1u << 8)
#define SCB_CFSR_PRECISERR_Msk (1u << 9)
#define SCB_CFSR_IMPRECISERR_Msk (1u << 10)
#define SCB_CFSR_UNDEFINSTR_Msk (1u << 16)
#define SCB_CFSR_INVSTATE_Msk (1u << 17)
#define SCB_CFSR_INVPC_Msk (1u << 18)
#define SCB_CFSR_UNALIGNED_Msk (1u << 24)
#define SCB_CFSR_DIVBYZERO_Msk (1u << 25)
#define SCB_SHCSR_MEMFAULTENA_Msk (1u << 16)
#define SCB_SHCSR_BUSFAULTENA_Msk (1u << 17)
#define SCB_SHCSR_USGFAULTENA_Msk (1u << 18)
#define SCB_CCR_DIV_0_TRP_Msk (1u << 4)

uint32_t RTOS_Test_GetMSP(void);
uint32_t RTOS_Test_GetPSP(void);
void     RTOS_Test_DisableIRQ(void);
void     RTOS_Test_DSB(void);
void     RTOS_Test_ISB(void);

#define __get_MSP() RTOS_Test_GetMSP()
#define __get_PSP() RTOS_Test_GetPSP()
#define __disable_irq() RTOS_Test_DisableIRQ()
#define __DSB() RTOS_Test_DSB()
#define __ISB() RTOS_Test_ISB()

#endif /* HOST_STM32H7XX_H */

#ifndef INTEGRATION_FREERTOS_H
#define INTEGRATION_FREERTOS_H
#include <stddef.h>
#include <stdint.h>
typedef int32_t  BaseType_t;
typedef uint32_t UBaseType_t;
typedef uint32_t TickType_t;
typedef uint32_t StackType_t;
typedef void*    TaskHandle_t;
typedef void*    SemaphoreHandle_t;
typedef struct
{
    uint8_t opaque[128];
} StaticTask_t;
typedef struct
{
    uint8_t opaque[96];
} StaticSemaphore_t;
#define pdFALSE ((BaseType_t) 0)
#define pdTRUE ((BaseType_t) 1)
#define pdPASS pdTRUE
#define portMAX_DELAY UINT32_MAX
#define taskSCHEDULER_NOT_STARTED ((BaseType_t) 0)
#define taskSCHEDULER_RUNNING ((BaseType_t) 1)
#define taskSCHEDULER_SUSPENDED ((BaseType_t) 2)
#define configTICK_RATE_HZ ((TickType_t) 1000)
#define configMAX_PRIORITIES 7u
#define configMINIMAL_STACK_SIZE ((uint16_t) 256)
#define pdMS_TO_TICKS(ms) ((TickType_t) (ms))
#define portYIELD_FROM_ISR(woken) vPortYieldFromISR(woken)
#define taskYIELD() vPortYield()
BaseType_t xPortIsInsideInterrupt(void);
void       vPortYieldFromISR(BaseType_t woken);
void       vPortYield(void);
#endif

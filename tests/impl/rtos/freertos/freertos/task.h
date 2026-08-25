#ifndef HOST_TASK_H
#define HOST_TASK_H

#include "FreeRTOS.h"

TaskHandle_t xTaskCreateStatic(void (*entry)(void*), const char* name, uint32_t stack_depth,
                               void* arg, UBaseType_t priority, StackType_t* stack,
                               StaticTask_t* tcb);
BaseType_t   xTaskGetSchedulerState(void);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
void         xTaskNotifyGive(TaskHandle_t task);
void         vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t* woken);
uint32_t     ulTaskNotifyTake(BaseType_t clear_on_exit, TickType_t timeout);
BaseType_t   xTaskDelayUntil(TickType_t* previous_wake, TickType_t increment);
TickType_t   xTaskGetTickCount(void);
UBaseType_t  uxTaskGetStackHighWaterMark(TaskHandle_t task);
void         vTaskSuspend(TaskHandle_t task);
void         vTaskResume(TaskHandle_t task);
void         vTaskDelete(TaskHandle_t task);
void         xPortSysTickHandler(void);
void         vTaskStartScheduler(void);

#endif /* HOST_TASK_H */

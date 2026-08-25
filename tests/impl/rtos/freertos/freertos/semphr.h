#ifndef HOST_SEMPHR_H
#define HOST_SEMPHR_H

#include "FreeRTOS.h"

SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t* storage);
SemaphoreHandle_t xSemaphoreCreateRecursiveMutexStatic(StaticSemaphore_t* storage);
SemaphoreHandle_t xSemaphoreCreateCountingStatic(UBaseType_t max, UBaseType_t initial,
                                                 StaticSemaphore_t* storage);
SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t* storage);
BaseType_t        xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout);
BaseType_t        xSemaphoreGive(SemaphoreHandle_t sem);
BaseType_t        xSemaphoreTakeRecursive(SemaphoreHandle_t sem, TickType_t timeout);
BaseType_t        xSemaphoreGiveRecursive(SemaphoreHandle_t sem);
BaseType_t        xSemaphoreGiveFromISR(SemaphoreHandle_t sem, BaseType_t* woken);
UBaseType_t       uxSemaphoreGetCount(SemaphoreHandle_t sem);
UBaseType_t       uxSemaphoreGetCountFromISR(SemaphoreHandle_t sem);

#endif /* HOST_SEMPHR_H */

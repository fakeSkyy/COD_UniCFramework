#ifndef INTEGRATION_SEMPHR_H
#define INTEGRATION_SEMPHR_H
#include "FreeRTOS.h"
SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t* storage);
SemaphoreHandle_t xSemaphoreCreateRecursiveMutexStatic(StaticSemaphore_t* storage);
BaseType_t        xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout);
BaseType_t        xSemaphoreGive(SemaphoreHandle_t sem);
BaseType_t        xSemaphoreTakeRecursive(SemaphoreHandle_t sem, TickType_t timeout);
BaseType_t        xSemaphoreGiveRecursive(SemaphoreHandle_t sem);
#endif

/**
 * @file platform_rtos_backend.h
 * @author Gao Xing
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef PLATFORM_RTOS_BACKEND_H
#define PLATFORM_RTOS_BACKEND_H

#include "impl_memory.h"
#include "impl_mutex.h"
#include "impl_sem.h"
#include "impl_task.h"

void* PRTOS_Memory_Alloc(size_t size);
void  PRTOS_Memory_Free(void* ptr);

bool PRTOS_Mutex_Init(void* storage, size_t bytes);
bool PRTOS_Mutex_Lock(void* storage, uint32_t timeout_ms);
void PRTOS_Mutex_Unlock(void* storage);
bool PRTOS_Mutex_LockForbidden(void);
bool PRTOS_Mutex_InitRecursive(void* storage, size_t bytes);
bool PRTOS_Mutex_LockRecursive(void* storage, uint32_t timeout_ms);
void PRTOS_Mutex_UnlockRecursive(void* storage);

bool     PRTOS_Sem_InitCounting(void* storage, size_t bytes, uint32_t max, uint32_t initial);
bool     PRTOS_Sem_InitBinary(void* storage, size_t bytes);
bool     PRTOS_Sem_Take(void* storage, uint32_t timeout_ms);
bool     PRTOS_Sem_Give(void* storage);
uint32_t PRTOS_Sem_Count(void* storage);

bool     PRTOS_Task_Create(void (*entry)(void*), void* arg, const char* name, void* stack,
                           size_t stack_bytes, void* tcb, size_t tcb_bytes, uint8_t priority,
                           void** out_handle);
void     PRTOS_Task_Notify(void* task);
void*    PRTOS_Task_Current(void);
bool     PRTOS_Task_NotifyWait(uint32_t timeout_ms);
bool     PRTOS_Task_DelayUntil(uint32_t* prev_tick, uint32_t period_ms);
uint32_t PRTOS_Task_TickNow(void);
void     PRTOS_Task_Yield(void);
bool     PRTOS_Task_BlockingForbidden(void);
size_t   PRTOS_Task_StackFree(void* task);
void     PRTOS_Task_Suspend(void* task);
void     PRTOS_Task_Resume(void* task);
void     PRTOS_Task_Destroy(void* task);
bool     PRTOS_Task_StartScheduler(void);
void     PRTOS_Task_FaultInit(void);

#endif /* PLATFORM_RTOS_BACKEND_H */
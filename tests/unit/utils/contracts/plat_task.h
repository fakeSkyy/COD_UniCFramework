#ifndef UTILS_CMOCK_PLAT_TASK_H
#define UTILS_CMOCK_PLAT_TASK_H

#include <stdbool.h>
#include <stdint.h>

void* PLAT_Task_Current(void);
void  PLAT_Task_Notify(void* task);
bool  PLAT_Task_Wait(uint32_t timeout_ms);

#endif /* UTILS_CMOCK_PLAT_TASK_H */

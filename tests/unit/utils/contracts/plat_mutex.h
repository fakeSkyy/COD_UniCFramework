#ifndef UTILS_CMOCK_PLAT_MUTEX_H
#define UTILS_CMOCK_PLAT_MUTEX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PLAT_MUTEX_WAIT_FOREVER UINT32_MAX

typedef union
{
    max_align_t align;
    uint8_t     bytes[128];
} Mutex_s;

bool PLAT_Mutex_Init(Mutex_s* mutex);
bool PLAT_Mutex_Lock(Mutex_s* mutex, uint32_t timeout_ms);
void PLAT_Mutex_Unlock(Mutex_s* mutex);
bool PLAT_Mutex_LockRequired(void);

#endif /* UTILS_CMOCK_PLAT_MUTEX_H */

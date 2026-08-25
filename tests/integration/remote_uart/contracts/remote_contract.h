/**
 * @file remote_contract.h
 * @author Kiro
 * @date 2026/8/24
 * @version 1.0
 */
#ifndef REMOTE_CONTRACT_H
#define REMOTE_CONTRACT_H
#include <stddef.h>
void* PLAT_malloc(size_t size);
void  PLAT_free(void* ptr);
#endif

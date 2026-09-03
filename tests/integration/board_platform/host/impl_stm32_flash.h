/**
 * @file impl_stm32_flash.h
 * @author Gao Xing
 * @date 2026/9/2
 * @version 1.0
 */

#ifndef INTEGRATION_HOST_IMPL_STM32_FLASH_H
#define INTEGRATION_HOST_IMPL_STM32_FLASH_H

/* Host stand-in for the real backend header. The composition root under test names
 * each backend header directly, so this build needs a file by that name; the
 * declarations live in board_backend_contract.h, which CMock generates the mocks
 * from. */
#include "board_backend_contract.h"

/* The real backend header defines this; the composition root names it directly now
 * rather than through a bind-header alias. */
#define IMPL_STM32_FLASH_PARAM_SECTOR 7u

#endif /* INTEGRATION_HOST_IMPL_STM32_FLASH_H */

/**
 * @file impl_stm32_flash.h
 * @author Gao Xing
 * @date 2026/9/2
 * @version 1.0
 */

#ifndef APPLICATION_HOST_IMPL_STM32_FLASH_H
#define APPLICATION_HOST_IMPL_STM32_FLASH_H

/* Host stand-in for the real backend header. The composition root under test names
 * each backend header directly, so the host build needs a file by that name; the
 * declarations themselves live in board_deps.h, which is what CMock generates the
 * IMPL_STM32_* mocks from. One stub per backend the root includes. */
#include "board_deps.h"

/* The real backend header defines this; the composition root names it directly now
 * rather than through a bind-header alias, so the host stand-in must supply it too.
 * The value only has to be consistent -- the tests assert the argument reaching
 * IMPL_STM32_FLASH_CreateCtx, not which sector the part actually has. */
#define IMPL_STM32_FLASH_PARAM_SECTOR 7u

#endif /* APPLICATION_HOST_IMPL_STM32_FLASH_H */

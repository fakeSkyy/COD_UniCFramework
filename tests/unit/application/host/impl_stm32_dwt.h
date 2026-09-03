/**
 * @file impl_stm32_dwt.h
 * @author Gao Xing
 * @date 2026/9/2
 * @version 1.0
 */

#ifndef APPLICATION_HOST_IMPL_STM32_DWT_H
#define APPLICATION_HOST_IMPL_STM32_DWT_H

/* Host stand-in for the real backend header. The composition root under test names
 * each backend header directly, so the host build needs a file by that name; the
 * declarations themselves live in board_deps.h, which is what CMock generates the
 * IMPL_STM32_* mocks from. One stub per backend the root includes. */
#include "board_deps.h"

#endif /* APPLICATION_HOST_IMPL_STM32_DWT_H */

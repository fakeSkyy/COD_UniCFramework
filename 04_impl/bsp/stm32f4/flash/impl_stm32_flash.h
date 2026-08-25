/**
 * @file impl_stm32_flash.h
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 */

#ifndef IMPL_STM32_FLASH_H
#define IMPL_STM32_FLASH_H

#include "impl_flash.h"
#include "stm32f4xx_hal.h"

/**
 * @brief Sector index of the last 128 KB sector on a 1 MB STM32F407.
 *
 * The furthest sector from the vector table, which makes it the safest place for
 * parameter storage: the firmware would have to grow past 896 KB before it could
 * reach this. Sector 3 sits only 14 KB past the current image and would be
 * overwritten by a moderately larger build.
 */
#define IMPL_STM32_FLASH_PARAM_SECTOR 11u

/**
 * @brief Create the opaque context for a range of internal flash sectors.
 *
 * @par This is a loaded gun, and the bounds check is the safety
 * Internal flash holds the running firmware. The context fixes a sector range
 * once, here, and every later call is bounds-checked against it — so a wrong
 * offset from a caller fails rather than erasing the vector table. Choosing the
 * range correctly is therefore the one thing that cannot be checked later, and
 * it is why this takes sector indices rather than raw addresses.
 *
 * Verify against the linker map before changing it. On the current build the
 * image ends at 0x0800885C, i.e. inside sector 2.
 *
 * @par Erase stalls the whole CPU
 * An STM32F4 cannot fetch instructions from flash while a sector is erasing, so
 * the core halts for the duration — the datasheet gives 1 to 2 seconds for a
 * 128 KB sector at 2.7-3.6 V. Interrupts do not run. Never erase from a control
 * loop or with motors enabled; do it at bring-up or in an explicit maintenance
 * state.
 *
 * @param first_sector  First sector in the region, 0 to 11 on a 1 MB F407.
 *                      MUST be past the end of the firmware image.
 * @param sector_count  Sectors in the region, at least 1.
 * @return Opaque context to hand to PLAT_Flash_Create, or NULL if the range is
 *         invalid or would extend past the end of flash.
 */
void* IMPL_STM32_FLASH_CreateCtx(uint32_t first_sector, uint32_t sector_count);

/**
 * @brief Get the STM32 internal-flash ops (vtable) for PLAT_Flash_Create.
 * @return Pointer to a read-only ops struct.
 */
const Flash_Ops_s* IMPL_STM32_FLASH_GetOps(void);

/**
 * @brief Release the context returned by IMPL_STM32_FLASH_CreateCtx.
 *
 * A no-op: the context names a sector range rather than owning an
 * allocation, so there is nothing to release. It exists so a generated
 * teardown loop can call it unconditionally alongside the allocating
 * backends, with no per-class exception.
 *
 * @param ctx  Context to release; unused, NULL is accepted.
 */
void IMPL_STM32_FLASH_DestroyCtx(void* ctx);

#endif /* IMPL_STM32_FLASH_H */

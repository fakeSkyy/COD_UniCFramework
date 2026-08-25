/**
 * @file impl_stm32_flash.h
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 */

#ifndef IMPL_STM32_FLASH_H
#define IMPL_STM32_FLASH_H

#include "impl_flash.h"
#include "stm32h7xx_hal.h"

/**
 * @brief Sector index of the last 128 KB sector on a 1 MB STM32H723xG.
 *
 * The furthest sector from the vector table, which makes it the safest place for
 * parameter storage: sectors are a uniform 128 KB here (FLASH_SECTOR_SIZE), so
 * the firmware would have to grow past 896 KB before it could reach this one.
 *
 * Note this is 7, not the 11 the STM32F407 used. An H723xG divides the same 1 MB
 * into 8 uniform sectors instead of 12 unequal ones (FLASH_SECTOR_TOTAL is 8),
 * so the F4 value is not merely suboptimal here — it is out of range and
 * IMPL_STM32_FLASH_CreateCtx rejects it.
 *
 * @warning Verify against the linker map before trusting it. No H7 image has been
 *          linked yet, so unlike the F407 build there is no measured image end to
 *          compare against; 896 KB of headroom is the geometry's promise, not an
 *          observation.
 */
#define IMPL_STM32_FLASH_PARAM_SECTOR 7u

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
 * @par Writes into this region are 32-byte quantised
 * The backend reports a write granularity of 32 because an H7 programs one
 * 256-bit flash word at a time (FLASH_NB_32BITWORD_IN_FLASHWORD is 8), and each
 * word may be programmed only once between erases. There is no read-modify-write
 * escape from that, so every offset and length handed to the write op must be a
 * multiple of 32. PLAT_Flash_Write enforces it; see PLAT_Flash_WriteGranularity.
 *
 * @par Erase stalls the whole CPU
 * An STM32H7 cannot fetch instructions from the bank being erased, so the core
 * halts for the duration of a 128 KB sector erase and interrupts do not run.
 * Never erase from a control loop or with motors enabled; do it at bring-up or in
 * an explicit maintenance state. The exact duration is a datasheet figure for
 * this part and has not been measured on this board.
 *
 * @param first_sector  First sector in the region, 0 to 7 on a 1 MB H723xG.
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

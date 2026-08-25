/**
 * @file impl_flash.h
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 */

#ifndef IMPL_FLASH_H
#define IMPL_FLASH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Non-volatile storage operations vtable — the vendor-neutral contract
 *        between the platform layer and any flash backend.
 *
 * Every member receives an opaque @p ctx describing one storage region. The
 * platform layer never inspects @p ctx; only the implementation that produced it
 * knows its real type. On an STM32F4 that is a range of internal flash sectors,
 * but the same ops can back an external SPI NOR chip or a RAM-backed stub for
 * host testing.
 *
 * @par Offsets are region-relative, never absolute
 * Every @p off is a byte offset from the start of the region, so a caller cannot
 * name an address outside it. Translating that to a physical address is the
 * backend's job, and it is the single place where a bounds check protects the
 * running firmware from being overwritten.
 *
 * Contract that every implementation MUST honor:
 *   - read:        Copy @p len bytes from @p off. No alignment requirement, and
 *                  no erase state assumed — reading erased flash yields 0xFF.
 *   - write:       Program @p len bytes at @p off. The target MUST already be
 *                  erased; flash can only clear bits, so writing over existing
 *                  data yields the AND of old and new. Both @p off and @p len
 *                  MUST be multiples of write_granularity — see below.
 *   - erase:       Erase the sector containing @p off, returning it to 0xFF.
 *                  Blocking, and on internal flash it stalls the CPU (see the
 *                  backend's docs) — never call from a control loop.
 *   - sector_of:   Index of the sector containing @p off, so a caller can tell
 *                  whether two objects share an erase unit.
 *   - sector_base: Region-relative offset where the sector containing @p off
 *                  starts, and sector_size gives its length. Both are needed
 *                  because sectors are NOT uniformly sized on every device.
 *   - size:        Total bytes in the region.
 *   - is_erased:   True when every byte in the range reads as erased. Cheaper
 *                  than reading it out to check, and lets a caller skip a
 *                  needless erase.
 *   - write_granularity: Smallest programmable unit in bytes, always a power of
 *                  two. Constant after creation, so callers may cache it.
 *
 * @par Why write_granularity has to be part of the contract
 * The programmable unit is not a detail a backend can hide. An STM32F4 programs
 * a single byte at a time, so 1 is honest there and any offset works. An
 * STM32H7 programs a 256-bit flash word and **each word may be written only
 * once between erases**, so a 32-byte granularity is not a performance
 * preference — a backend cannot emulate byte writes by read-modify-write,
 * because the second write to the same word fails in hardware.
 *
 * A contract that promised "backends handle any alignment" would therefore be
 * unimplementable on an H7, and the failure would appear as a corrupt record
 * rather than a compile error. Exposing the number instead lets a caller pad its
 * record once and stay correct on both.
 *
 * None of these members may be called from ISR context: erase and write block
 * for milliseconds and, on internal flash, halt instruction fetch.
 */
typedef struct
{
    bool (*read)(void* ctx, uint32_t off, uint8_t* dst, size_t len);
    bool (*write)(void* ctx, uint32_t off, const uint8_t* src, size_t len);
    bool (*erase)(void* ctx, uint32_t off);

    uint32_t (*sector_of)(void* ctx, uint32_t off);
    uint32_t (*sector_base)(void* ctx, uint32_t off);
    uint32_t (*sector_size)(void* ctx, uint32_t off);

    uint32_t (*size)(void* ctx);
    bool (*is_erased)(void* ctx, uint32_t off, size_t len);

    uint32_t (*write_granularity)(void* ctx);
} Flash_Ops_s;

#endif /* IMPL_FLASH_H */

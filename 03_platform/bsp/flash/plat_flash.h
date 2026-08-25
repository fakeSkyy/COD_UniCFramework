/**
 * @file plat_flash.h
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 */

#ifndef PLAT_FLASH_H
#define PLAT_FLASH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "impl_flash.h"

/**
 * @brief A vendor-neutral handle to one non-volatile storage region.
 *
 * Carries only an ops vtable and an opaque @c ctx produced by some backend. This
 * layer never dereferences @c ctx, so it stays fully decoupled from any chip
 * vendor — the same handle can front internal flash, an external SPI NOR, or a
 * RAM stub in a host test.
 */
typedef struct Flash_Instance_s Flash_Instance_s;

struct Flash_Instance_s
{
    const Flash_Ops_s* ops; /**< Backend vtable (from *_GetOps).            */
    void*              ctx; /**< Opaque, backend-owned region descriptor.   */
    void*              id;  /**< Optional owner tag for registry use.       */
};

/* ------------------------------------------------------------------------- */
/*  Lifecycle                                                                */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/*  Access                                                                   */
/* ------------------------------------------------------------------------- */

/**
 * @brief Read bytes from the region.
 *
 * Cheap and always safe: on internal flash this is a memory-mapped copy with no
 * unlock and no CPU stall, so it may be called freely.
 *
 * @param f    Flash instance.
 * @param off  Region-relative byte offset. Offsets are relative so a caller
 *             cannot name an address outside the region.
 * @param dst  Destination buffer for @p len bytes.
 * @param len  Bytes to read; must be non-zero.
 * @return true on success; false if any argument is NULL or the range extends
 *         past the end of the region. Erased bytes read as 0xFF.
 */
bool PLAT_Flash_Read(Flash_Instance_s* f, uint32_t off, uint8_t* dst, size_t len);

/**
 * @brief Program bytes into the region.
 *
 * @par The target must already be erased
 * Flash can only clear bits, so programming over existing data yields the
 * bitwise AND of old and new rather than the new value. The backend verifies by
 * reading back, so this call reports the failure — but the data is still lost.
 * Erase first, or check with PLAT_Flash_IsErased.
 *
 * @par Offset and length must be granularity-aligned
 * Both must be multiples of PLAT_Flash_WriteGranularity(). Where that is 1 any
 * offset works; where it is larger the requirement is **not negotiable**, because
 * such a part typically allows a flash word to be programmed only once between
 * erases — so a backend cannot emulate a narrower write by reading, modifying and
 * rewriting. Round your record size up to the granularity once and the same code
 * is correct on every backend.
 *
 * @param f    Flash instance.
 * @param off  Region-relative byte offset; must be granularity-aligned.
 * @param src  Source bytes.
 * @param len  Bytes to write; must be non-zero and a granularity multiple.
 * @return true when every byte was programmed AND read back identical; false on
 *         a bad argument, a misaligned offset or length, an out-of-range span, a
 *         hardware failure, or a verification mismatch.
 */
bool PLAT_Flash_Write(Flash_Instance_s* f, uint32_t off, const uint8_t* src, size_t len);

/**
 * @brief Erase the whole sector containing @p off, returning it to 0xFF.
 *
 * @par This stalls the CPU, it does not merely block
 * On internal flash the core cannot fetch instructions while a sector erases, so
 * everything stops — including interrupts — for 1 to 2 seconds on a 128 KB
 * sector. Call this at bring-up or in an explicit maintenance state, never from
 * a control loop and never with motors enabled.
 *
 * @par It erases a whole sector, not a range
 * Sectors are the erase unit, and they are not necessarily uniformly sized — do
 * not compute a sector index by dividing. Anything else sharing the sector is
 * destroyed too, so use PLAT_Flash_SectorOf to check whether two objects collide
 * before assuming they are independent.
 *
 * @param f    Flash instance.
 * @param off  Region-relative offset anywhere inside the sector to erase.
 * @return true when the sector was erased and verified blank.
 */
bool PLAT_Flash_EraseSector(Flash_Instance_s* f, uint32_t off);

/* ------------------------------------------------------------------------- */
/*  Geometry                                                                 */
/* ------------------------------------------------------------------------- */

/**
 * @brief Total size of the region in bytes.
 * @param f  Flash instance.
 * @return Size in bytes, or 0 if @p f is NULL.
 */
uint32_t PLAT_Flash_Size(Flash_Instance_s* f);

/**
 * @brief Index of the sector containing @p off.
 *
 * Use it to tell whether two stored objects share an erase unit: if they do,
 * rewriting one destroys the other.
 *
 * @param f    Flash instance.
 * @param off  Region-relative offset.
 * @return Sector index, or 0 if @p f is NULL or @p off is out of range. Since 0
 *         is also a valid index, validate the offset separately when the
 *         distinction matters.
 */
uint32_t PLAT_Flash_SectorOf(Flash_Instance_s* f, uint32_t off);

/**
 * @brief Region-relative offset at which @p off's sector begins.
 * @param f    Flash instance.
 * @param off  Region-relative offset.
 * @return Sector start offset, or 0 on a bad argument.
 */
uint32_t PLAT_Flash_SectorBase(Flash_Instance_s* f, uint32_t off);

/**
 * @brief Size in bytes of the sector containing @p off.
 *
 * Needed because sectors are not uniform: assuming a fixed size is how a caller
 * ends up erasing the wrong span.
 *
 * @param f    Flash instance.
 * @param off  Region-relative offset.
 * @return Sector size in bytes, or 0 on a bad argument.
 */
uint32_t PLAT_Flash_SectorSize(Flash_Instance_s* f, uint32_t off);

/**
 * @brief Smallest programmable unit, in bytes.
 *
 * Round every record size up to a multiple of this and place records at
 * multiples of it. It is 1 on a part that programs single bytes, and larger where
 * the controller writes a wide flash word that may not be revisited before the
 * next erase. Constant for the life of the instance, so it is safe to cache or to
 * use in a compile-time layout when the target is known.
 *
 * @param f  Flash instance.
 * @return Granularity in bytes, always a power of two; 0 if @p f is NULL.
 */
uint32_t PLAT_Flash_WriteGranularity(Flash_Instance_s* f);

/**
 * @brief Test whether a range is entirely erased.
 *
 * Cheaper than reading it back to check, and lets a caller skip an erase that
 * would otherwise cost seconds of CPU stall.
 *
 * @param f    Flash instance.
 * @param off  Region-relative offset.
 * @param len  Bytes to test; must be non-zero.
 * @return true when every byte in the range reads as 0xFF.
 */
bool PLAT_Flash_IsErased(Flash_Instance_s* f, uint32_t off, size_t len);

/* ========================================================================= */
/*  Construction (composition root only)                                     */
/* ========================================================================= */

/* Building an instance needs an ops vtable and a backend context, and both are
 * vendor symbols — so any caller of these is, by definition, naming a specific
 * chip. That is the composition root's job and nowhere else's.
 *
 * The gate makes that a compile error rather than a convention: an application or
 * device file that reaches for one of these has not defined
 * PLAT_ALLOW_CONSTRUCTION, so the declaration is not visible and the call fails
 * to compile. It gets its handles from the board layer instead, which is the only
 * place allowed to open this.
 *
 * Everything above this line takes an already-built handle and never mentions a
 * vendor, so it stays available to every layer. */
#ifdef PLAT_ALLOW_CONSTRUCTION

/**
 * @brief Initialize a Flash instance over caller-provided storage.
 *
 * The counterpart of PLAT_Flash_Create for storage the caller already owns — a
 * static, or a member of a larger struct. Preferred wherever the number of
 * instances is known at build time, because it removes the only way bringing a
 * fixed peripheral up can fail for a reason unrelated to the hardware: the heap
 * being short. PLAT_Flash_Create is now a thin wrapper around this.
 *
 * @param inst  Storage to initialize. Must outlive every use of the instance.
 * @param ops   Backend ops vtable (must not be NULL).
 * @param ctx   Opaque backend context.
 * @return true on success; false on a NULL argument or a context the backend
 *         reports as unusable, in which case @p inst is left untouched.
 */
bool PLAT_Flash_Init(Flash_Instance_s* inst, const Flash_Ops_s* ops, void* ctx);

/**
 * @brief Create a flash instance from a backend-provided ops and context.
 *
 * @param ops  Backend ops vtable (must not be NULL).
 * @param ctx  Opaque backend context describing one storage region.
 * @return Pointer to the created instance, or NULL on allocation failure.
 */
Flash_Instance_s* PLAT_Flash_Create(const Flash_Ops_s* ops, void* ctx);

#endif /* PLAT_ALLOW_CONSTRUCTION */

#endif /* PLAT_FLASH_H */

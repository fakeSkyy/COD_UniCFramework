/**
 * @file impl_stm32_flash.c
 * @author Gao Xing
 * @date 2026/8/7
 * @version 1.0
 *
 * Internal-flash backend for the STM32H723xG.
 *
 * Four properties of this hardware shape the whole implementation, and three of
 * them differ from the STM32F407 this was ported from:
 *
 *   - Sectors ARE uniform here: 8 of them, 128 KB each (FLASH_SECTOR_TOTAL and
 *     FLASH_SECTOR_SIZE). The F4 needed a lookup table because its 12 sectors ran
 *     16/64/128 KB; on this part the geometry is a shift, so the tables are gone
 *     and the constants are tied to the CMSIS header by static assertion.
 *   - The programmable unit is a 256-bit flash word, not a byte, and each word may
 *     be programmed only once between erases. That is why write() refuses a
 *     misaligned offset or length instead of emulating byte writes.
 *   - Programming can only clear bits. Writing 0x00 over 0xFF works; the reverse
 *     needs a sector erase. Write() therefore verifies rather than assuming.
 *   - Erase halts instruction fetch from the bank, stalling the CPU for the
 *     duration of a 128 KB sector erase. Nothing here can hide that, so it is
 *     documented at every level.
 */

#include "impl_stm32_flash.h"

/* ========================================================================= */
/*  Geometry                                                                 */
/* ========================================================================= */

/** @brief Sectors on a 1 MB STM32H723xG. */
#define SECTOR_COUNT 8u

/** @brief Bytes per sector. Uniform on this part, unlike the F407. */
#define SECTOR_BYTES 0x20000u

/** @brief Base address of internal flash. */
#define FLASH_ORIGIN 0x08000000u

/** @brief 32-bit words in one flash word — the hardware programming unit. */
#define FLASH_WORD_WORDS 8u

/** @brief Bytes in one flash word, i.e. the write granularity. */
#define FLASH_WORD_BYTES (FLASH_WORD_WORDS * 4u)

/** @brief Value every byte reads as after an erase. */
#define ERASED_BYTE 0xFFu

/* The constants above are duplicated from the CMSIS device header so the hot
 * paths can use plain literals, which means a header that disagreed would be a
 * silent wrong-sector bug. Bind them instead: swapping in a different H7 part
 * whose geometry differs becomes a build failure here rather than an erase of the
 * vector table at run time. */
_Static_assert(FLASH_SECTOR_TOTAL == SECTOR_COUNT, "sector count must match the device header");
_Static_assert(FLASH_SECTOR_SIZE == SECTOR_BYTES, "sector size must match the device header");
_Static_assert(FLASH_NB_32BITWORD_IN_FLASHWORD == FLASH_WORD_WORDS,
               "flash word width must match the device header");
_Static_assert(IMPL_STM32_FLASH_PARAM_SECTOR < SECTOR_COUNT,
               "the parameter sector must exist on this part");

/* ========================================================================= */
/*  Context                                                                  */
/* ========================================================================= */

/**
 * @brief One contiguous range of internal flash sectors.
 *
 * @c base and @c bytes are the bounds every operation is checked against, which
 * is what stops a bad offset from reaching the firmware image.
 */
typedef struct
{
    uint32_t first_sector; /**< First sector index in the region.        */
    uint32_t last_sector;  /**< Last sector index, inclusive.            */
    uint32_t base;         /**< Absolute address of the region start.    */
    uint32_t bytes;        /**< Total size of the region.                */
    bool     in_use;       /**< True once handed out.                    */
} IMPL_STM32_FLASH_Context_s;

/**
 * @brief Contexts are static, one per possible region.
 *
 * Internal flash is a single peripheral, so there is nothing to allocate; a small
 * fixed pool avoids a dependency on the heap in a module that has to work at
 * board-setup time.
 */
#define MAX_REGIONS 2u

static IMPL_STM32_FLASH_Context_s regions[MAX_REGIONS];

/* ========================================================================= */
/*  Helpers                                                                  */
/* ========================================================================= */

/**
 * @brief Sector index containing an absolute address.
 *
 * @param addr  Absolute flash address.
 * @return Sector index, or SECTOR_COUNT when the address is outside flash.
 */
static uint32_t sector_for_addr(uint32_t addr)
{
    if (addr < FLASH_ORIGIN)
    {
        return SECTOR_COUNT;
    }

    uint32_t index = (addr - FLASH_ORIGIN) / SECTOR_BYTES;

    return (index < SECTOR_COUNT) ? index : SECTOR_COUNT;
}

/**
 * @brief Absolute address at which a sector starts.
 *
 * @param sector  Sector index, which the callers have already bounds-checked.
 * @return Absolute address of the sector start.
 */
static uint32_t addr_for_sector(uint32_t sector) { return FLASH_ORIGIN + (sector * SECTOR_BYTES); }

/**
 * @brief Test whether a region-relative range lies wholly inside the region.
 *
 * The overflow term matters: a caller passing a huge @p len could otherwise wrap
 * @c off+len back into range and pass a naive comparison.
 *
 * @param c    Region to check against.
 * @param off  Region-relative start offset.
 * @param len  Length in bytes.
 * @return true when the whole range is inside the region.
 */
static bool range_ok(const IMPL_STM32_FLASH_Context_s* c, uint32_t off, size_t len)
{
    if (len == 0u)
    {
        return false;
    }
    if (off >= c->bytes)
    {
        return false;
    }
    if (len > (size_t) (c->bytes - off))
    {
        return false;
    }

    return true;
}

/**
 * @brief Drop any cached copies of a flash range so the next read hits the array.
 *
 * @par Why this exists at all, and why the F4's FLASH_FlushCaches did not survive
 * The F4 backend called FLASH_FlushCaches, which resets the flash controller's own
 * ART accelerator. That function does not exist on an H7 and the problem it solved
 * is not the problem here: the H7 hazard is the Cortex-M7 L1 D-Cache, which
 * main.c enables unconditionally (SCB_EnableDCache). Neither HAL_FLASH_Program nor
 * HAL_FLASHEx_Erase performs any cache maintenance — checked, there is not a
 * single SCB_ reference in either HAL source — so a line cached before an erase or
 * a program still returns its old contents afterwards. The verify below would then
 * compare against stale bytes: it would fail a write that succeeded and, worse,
 * pass one that did not when the stale value happens to match.
 *
 * @par Why invalidate is enough, rather than clean-and-invalidate
 * Invalidating discards a cache line without writing it back, which would lose
 * data if the line could be dirty. It cannot be. MPU_Config in main.c installs
 * exactly one region, at 0x24000000, and enables the MPU with MPU_HFNMI_PRIVDEF,
 * so flash at 0x08000000 is governed by the ARMv7-M default map: Normal,
 * write-through, no write-allocate. Write-through means the stores HAL_FLASH_Program
 * issues into the flash write buffer reach the controller rather than lingering in
 * cache, so there is never a dirty flash line to lose.
 *
 * @param addr  Absolute start address; need not be cache-line aligned, the CMSIS
 *              helper widens to lines itself.
 * @param len   Bytes to invalidate.
 */
static void invalidate_cached_range(uint32_t addr, size_t len)
{
    SCB_InvalidateDCache_by_Addr((volatile void*) addr, (int32_t) len);
}

/**
 * @brief Clear the error and end-of-operation flags left by a previous operation.
 *
 * @par Why the flag set is not the F4's
 * The F407 code cleared PGAERR, which does not exist on an H7 — the controller has
 * a different error set (PGSERR, STRBERR, INCERR, the two ECC flags, and more).
 * Naming them individually invites the same drift on the next port, so this uses
 * the header's own aggregate. H723xG is single-bank (DUAL_BANK is not defined for
 * it), which is why the bank-1 masks are the whole story here.
 *
 * Clearing matters because FLASH_WaitForLastOperation reads FLASH->SR1 and reports
 * any error bit it finds as belonging to the operation just issued, so a stale
 * flag turns the next program or erase into a spurious failure.
 */
static void clear_flash_flags(void)
{
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK1);
}

/* ========================================================================= */
/*  Ops implementation (private)                                             */
/* ========================================================================= */

static bool stm32_flash_read(void* ctx, uint32_t off, uint8_t* dst, size_t len)
{
    IMPL_STM32_FLASH_Context_s* c = (IMPL_STM32_FLASH_Context_s*) ctx;

    if (c == NULL || dst == NULL || !range_ok(c, off, len))
    {
        return false;
    }

    /* Flash is memory-mapped for reads, so this is a plain copy — no unlock, no
     * wait states to manage, and safe to do at any time.
     *
     * One H7-only caveat the F4 did not have: every flash word carries ECC, and a
     * word left half-programmed by a power loss can raise a double-detection ECC
     * error on read. That surfaces as a bus fault, not as wrong data, so it cannot
     * be reported through this return value. It is the reason a stored record wants
     * its own integrity check on top of this (dev_bmi088_store's magic and CRC). */
    const uint8_t* src = (const uint8_t*) (c->base + off);

    for (size_t i = 0u; i < len; i++)
    {
        dst[i] = src[i];
    }

    return true;
}

static bool stm32_flash_write(void* ctx, uint32_t off, const uint8_t* src, size_t len)
{
    IMPL_STM32_FLASH_Context_s* c = (IMPL_STM32_FLASH_Context_s*) ctx;

    if (c == NULL || src == NULL || !range_ok(c, off, len))
    {
        return false;
    }

    /* PLAT_Flash_Write already rejects a misaligned offset or length, but the ops
     * table is reachable directly and this is the one check that cannot be relaxed:
     * an unaligned or partial flash word makes the controller raise PGSERR, and a
     * second program of a word already written between erases is what leaves it
     * ECC-inconsistent. Fail here rather than corrupt it there. */
    if ((off & (FLASH_WORD_BYTES - 1u)) != 0u || (len & (size_t) (FLASH_WORD_BYTES - 1u)) != 0u)
    {
        return false;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }

    clear_flash_flags();

    uint32_t addr = c->base + off;
    bool     ok   = true;

    /* One 256-bit flash word per iteration. This is not a tuning choice as the F4's
     * byte loop was: FLASH_TYPEPROGRAM_FLASHWORD is the only user-flash mode this
     * part offers, and the loop step has to be its width.
     *
     * The staging array is not redundant either. HAL_FLASH_Program takes the ADDRESS
     * of the data, not the value the F4 signature took, and dereferences it as
     * uint32_t* — so the source must be 4-byte aligned, which a caller's
     * const uint8_t* is not required to be. Copying through an aligned local costs
     * 32 bytes of stack and removes an unaligned-access fault that would only
     * appear for some callers. */
    for (size_t i = 0u; i < len && ok; i += FLASH_WORD_BYTES)
    {
        uint32_t word[FLASH_WORD_WORDS];
        uint8_t* staging = (uint8_t*) word;

        for (uint32_t b = 0u; b < FLASH_WORD_BYTES; b++)
        {
            staging[b] = src[i + b];
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t) (addr + i),
                              (uint32_t) (uintptr_t) word) != HAL_OK)
        {
            ok = false;
        }
    }

    /* Unlock is what can fail meaningfully; a failing Lock means the peripheral is
     * already locked, which is the state being asked for. The write result below is
     * what the caller is told about. */
    HAL_FLASH_Lock();

    if (!ok)
    {
        return false;
    }

    invalidate_cached_range(addr, len);

    /* Verify by reading back. Flash can only clear bits, so programming over
     * un-erased data silently yields the AND of old and new — a corruption that
     * looks like a successful write to the HAL. This is the only way to catch it. */
    const uint8_t* check = (const uint8_t*) addr;

    for (size_t i = 0u; i < len; i++)
    {
        if (check[i] != src[i])
        {
            return false;
        }
    }

    return true;
}

static bool stm32_flash_erase(void* ctx, uint32_t off)
{
    IMPL_STM32_FLASH_Context_s* c = (IMPL_STM32_FLASH_Context_s*) ctx;

    if (c == NULL || !range_ok(c, off, 1u))
    {
        return false;
    }

    uint32_t sector = sector_for_addr(c->base + off);

    /* Belt and braces: range_ok already proved the offset is inside the region,
     * so this can only fail if the geometry and the context disagree. */
    if (sector < c->first_sector || sector > c->last_sector)
    {
        return false;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }

    clear_flash_flags();

    FLASH_EraseInitTypeDef er;
    er.TypeErase = FLASH_TYPEERASE_SECTORS;

    /* H723xG has a single bank, so this is the only legal value — IS_FLASH_BANK
     * reduces to a comparison against FLASH_BANK_1 when DUAL_BANK is undefined. */
    er.Banks     = FLASH_BANK_1;
    er.Sector    = sector; /* HAL sector IDs are the plain indices, 0 to 7. */
    er.NbSectors = 1u;

    /* Read this field's name as it behaves on an H7 rather than as it read on an
     * F4. Here it lands in FLASH_CR.PSIZE, which selects the controller's access
     * width, not a supply-voltage band; RANGE_3 is the 32-bit setting, matching the
     * 32-bit accesses HAL_FLASH_Program makes on the program path. Getting it wrong
     * misconfigures erase parallelism and can leave the sector partly erased. */
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t          failed_sector = 0u;
    HAL_StatusTypeDef st            = HAL_FLASHEx_Erase(&er, &failed_sector);

    /* See the note at the other Lock: a failing Lock means already locked, and the
     * erase result in st is what the caller is told about. */
    HAL_FLASH_Lock();

    if (st != HAL_OK || failed_sector != 0xFFFFFFFFu)
    {
        return false;
    }

    /* The erase changed 128 KB behind the D-Cache's back; drop the stale lines
     * before the blank check reads any of it. */
    invalidate_cached_range(addr_for_sector(sector), SECTOR_BYTES);

    /* Confirm the sector really did come back as 0xFF. A reported success with a
     * partly-erased sector would otherwise surface later as a write that cannot
     * set bits. */
    const uint8_t* p = (const uint8_t*) addr_for_sector(sector);

    for (uint32_t i = 0u; i < SECTOR_BYTES; i++)
    {
        if (p[i] != ERASED_BYTE)
        {
            return false;
        }
    }

    return true;
}

static uint32_t stm32_flash_sector_of(void* ctx, uint32_t off)
{
    IMPL_STM32_FLASH_Context_s* c = (IMPL_STM32_FLASH_Context_s*) ctx;

    if (c == NULL || !range_ok(c, off, 1u))
    {
        return 0u;
    }

    return sector_for_addr(c->base + off);
}

static uint32_t stm32_flash_sector_base(void* ctx, uint32_t off)
{
    IMPL_STM32_FLASH_Context_s* c = (IMPL_STM32_FLASH_Context_s*) ctx;

    if (c == NULL || !range_ok(c, off, 1u))
    {
        return 0u;
    }

    uint32_t sector = sector_for_addr(c->base + off);

    if (sector >= SECTOR_COUNT)
    {
        return 0u;
    }

    /* Region-relative, matching the offsets the caller passes in. */
    return addr_for_sector(sector) - c->base;
}

static uint32_t stm32_flash_sector_size(void* ctx, uint32_t off)
{
    IMPL_STM32_FLASH_Context_s* c = (IMPL_STM32_FLASH_Context_s*) ctx;

    if (c == NULL || !range_ok(c, off, 1u))
    {
        return 0u;
    }

    uint32_t sector = sector_for_addr(c->base + off);

    return (sector < SECTOR_COUNT) ? SECTOR_BYTES : 0u;
}

static uint32_t stm32_flash_size(void* ctx)
{
    IMPL_STM32_FLASH_Context_s* c = (IMPL_STM32_FLASH_Context_s*) ctx;

    return (c != NULL) ? c->bytes : 0u;
}

static bool stm32_flash_is_erased(void* ctx, uint32_t off, size_t len)
{
    IMPL_STM32_FLASH_Context_s* c = (IMPL_STM32_FLASH_Context_s*) ctx;

    if (c == NULL || !range_ok(c, off, len))
    {
        return false;
    }

    const uint8_t* p = (const uint8_t*) (c->base + off);

    for (size_t i = 0u; i < len; i++)
    {
        if (p[i] != ERASED_BYTE)
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief Smallest programmable unit on an STM32H723xG: one 256-bit flash word.
 *
 * 32, not the F4's 1, and the difference is not a performance detail a backend
 * could paper over. A flash word may be programmed only once between erases, so
 * read-modify-write is unavailable: the second program of a word does not merely
 * fail, it can leave the word's ECC inconsistent so that reading it faults.
 * Reporting the real number is what lets PLAT_Flash_Write reject a misaligned
 * request instead of producing a record that is corrupt on the next boot.
 */
static uint32_t stm32_flash_write_granularity(void* ctx)
{
    (void) ctx;
    return FLASH_WORD_BYTES;
}

/* ========================================================================= */
/*  Ops table                                                                */
/* ========================================================================= */

static const Flash_Ops_s stm32_flash_ops = {
    .read              = stm32_flash_read,
    .write             = stm32_flash_write,
    .erase             = stm32_flash_erase,
    .sector_of         = stm32_flash_sector_of,
    .sector_base       = stm32_flash_sector_base,
    .sector_size       = stm32_flash_sector_size,
    .size              = stm32_flash_size,
    .is_erased         = stm32_flash_is_erased,
    .write_granularity = stm32_flash_write_granularity,
};

/* ========================================================================= */
/*  Public                                                                   */
/* ========================================================================= */

void* IMPL_STM32_FLASH_CreateCtx(uint32_t first_sector, uint32_t sector_count)
{
    if (sector_count == 0u || first_sector >= SECTOR_COUNT)
    {
        return NULL;
    }
    if ((first_sector + sector_count) > SECTOR_COUNT)
    {
        return NULL;
    }

    const uint32_t last = first_sector + sector_count - 1u;

    /* Refuse a region that overlaps one already handed out.
     *
     * Without this, two calls naming the same sectors each get their own context and
     * each believes it owns them. Nothing detects the conflict, and the failure is
     * silent and asymmetric: one region writes a record, the other erases the sector
     * it lives in, and the first region's data is simply gone at the next read. A
     * parameter store is exactly where that is most costly, because the loss shows up
     * as a device that has forgotten its calibration rather than as a failed call.
     *
     * MAX_REGIONS is 2, so this can only happen once — which is not a defence, since
     * once is enough to lose the store. */
    IMPL_STM32_FLASH_Context_s* c = NULL;

    for (uint32_t i = 0u; i < MAX_REGIONS; i++)
    {
        if (!regions[i].in_use)
        {
            if (c == NULL)
            {
                c = &regions[i];
            }
            continue;
        }

        if (first_sector <= regions[i].last_sector && last >= regions[i].first_sector)
        {
            return NULL; /* overlaps a live region */
        }
    }

    if (c == NULL)
    {
        return NULL;
    }

    c->first_sector = first_sector;
    c->last_sector  = last;
    c->base         = addr_for_sector(first_sector);
    c->bytes        = sector_count * SECTOR_BYTES;
    c->in_use       = true;

    return c;
}

const Flash_Ops_s* IMPL_STM32_FLASH_GetOps(void) { return &stm32_flash_ops; }

/* No-op: the context is a slot inside the static regions[] table, not an
 * allocation, so DestroyCtx exists only for symmetry with the allocating
 * backends and has nothing to release. */
void IMPL_STM32_FLASH_DestroyCtx(void* ctx) { (void) ctx; }

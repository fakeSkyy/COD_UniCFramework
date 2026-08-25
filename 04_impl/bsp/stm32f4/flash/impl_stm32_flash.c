/**
 * @file impl_stm32_flash.c
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 *
 * Internal-flash backend for the STM32F4.
 *
 * Three properties of this hardware shape the whole implementation:
 *
 *   - Sectors are NOT uniform. On a 1 MB F407 they run 4x16 KB, 1x64 KB, then
 *     7x128 KB, so every offset-to-sector question needs a table rather than a
 *     shift. A backend that assumed a fixed size would erase the wrong region.
 *   - Programming can only clear bits. Writing 0x00 over 0xFF works; the reverse
 *     needs a sector erase. Write() therefore verifies rather than assuming.
 *   - Erase halts instruction fetch, stalling the CPU for 1-2 s on a 128 KB
 *     sector. Nothing here can hide that, so it is documented at every level.
 */

#include "impl_stm32_flash.h"

/* ========================================================================= */
/*  Sector table                                                             */
/* ========================================================================= */

/** @brief Sectors on a 1 MB STM32F407. */
#define SECTOR_COUNT 12u

/** @brief Base address of internal flash. */
#define FLASH_ORIGIN 0x08000000u

/**
 * @brief Size of each sector, in bytes, indexed by sector number.
 *
 * The non-uniformity is the reason this table exists: sector 4 is four times
 * sector 3, and sector 5 twice sector 4.
 */
static const uint32_t sector_bytes[SECTOR_COUNT] = {
    16u * 1024u,  16u * 1024u,  16u * 1024u,  16u * 1024u,  64u * 1024u,  128u * 1024u,
    128u * 1024u, 128u * 1024u, 128u * 1024u, 128u * 1024u, 128u * 1024u, 128u * 1024u,
};

/**
 * @brief Absolute address where each sector starts.
 *
 * Derived from @ref sector_bytes, but held explicitly so the hot paths do not
 * re-accumulate it on every call.
 */
static const uint32_t sector_addr[SECTOR_COUNT] = {
    0x08000000u, 0x08004000u, 0x08008000u, 0x0800C000u, 0x08010000u, 0x08020000u,
    0x08040000u, 0x08060000u, 0x08080000u, 0x080A0000u, 0x080C0000u, 0x080E0000u,
};

/** @brief HAL sector identifiers, which happen to be the plain indices. */
static const uint32_t hal_sector[SECTOR_COUNT] = {
    FLASH_SECTOR_0, FLASH_SECTOR_1, FLASH_SECTOR_2,  FLASH_SECTOR_3,
    FLASH_SECTOR_4, FLASH_SECTOR_5, FLASH_SECTOR_6,  FLASH_SECTOR_7,
    FLASH_SECTOR_8, FLASH_SECTOR_9, FLASH_SECTOR_10, FLASH_SECTOR_11,
};

/** @brief Value every byte reads as after an erase. */
#define ERASED_BYTE 0xFFu

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
    for (uint32_t i = 0u; i < SECTOR_COUNT; i++)
    {
        if (addr >= sector_addr[i] && addr < (sector_addr[i] + sector_bytes[i]))
        {
            return i;
        }
    }

    return SECTOR_COUNT;
}

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
     * wait states to manage, and safe to do at any time. */
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

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }

    /* Clear the error flags left by any previous operation. HAL_FLASH_Program
     * reports a stale PGSERR as a fresh failure otherwise. */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR);

    uint32_t addr = c->base + off;
    bool     ok   = true;

    /* Byte-at-a-time. Word programming would be roughly four times faster, but it
     * requires both the address and the length to be word-aligned, and a
     * parameter blob is neither in general. Correctness first: this path runs
     * once per save, not in a loop. */
    for (size_t i = 0u; i < len && ok; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr + i, (uint64_t) src[i]) != HAL_OK)
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

    /* The ART data cache is enabled (DATA_CACHE_ENABLE), and HAL_FLASH_Program does
     * not flush it — only the erase paths do. Any line cached before the program
     * still returns its old contents, so the verify below would compare against
     * stale bytes: it would fail a write that succeeded, and, worse, pass one that
     * did not when the stale value happens to match. Reachable today from
     * dev_bmi088_store, which reads the region with IsErased and then skips the
     * erase when it is already blank, so nothing in that path flushes. */
    FLASH_FlushCaches();

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
     * so this can only fail if the tables and the context disagree. */
    if (sector < c->first_sector || sector > c->last_sector)
    {
        return false;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef er;
    er.TypeErase = FLASH_TYPEERASE_SECTORS;
    er.Banks     = FLASH_BANK_1;
    er.Sector    = hal_sector[sector];
    er.NbSectors = 1u;

    /* Range 3 is 2.7-3.6 V, which is where this board runs. A wrong voltage range
     * selects the wrong erase parallelism and can leave the sector only partly
     * erased. */
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

    /* Confirm the sector really did come back as 0xFF. A reported success with a
     * partly-erased sector would otherwise surface later as a write that cannot
     * set bits. */
    const uint8_t* p = (const uint8_t*) sector_addr[sector];

    for (uint32_t i = 0u; i < sector_bytes[sector]; i++)
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
    return sector_addr[sector] - c->base;
}

static uint32_t stm32_flash_sector_size(void* ctx, uint32_t off)
{
    IMPL_STM32_FLASH_Context_s* c = (IMPL_STM32_FLASH_Context_s*) ctx;

    if (c == NULL || !range_ok(c, off, 1u))
    {
        return 0u;
    }

    uint32_t sector = sector_for_addr(c->base + off);

    return (sector < SECTOR_COUNT) ? sector_bytes[sector] : 0u;
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
 * @brief Smallest programmable unit on an STM32F4: one byte.
 *
 * The F4 flash controller supports byte, half-word, word and double-word
 * programming, so any offset and length are valid. Reporting 1 tells the platform
 * layer there is nothing to align, which keeps the granularity check free here
 * while the same code stays correct on a device that needs 32.
 */
static uint32_t stm32_flash_write_granularity(void* ctx)
{
    (void) ctx;
    return 1u;
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

    /* Refuse a region that overlaps one already handed out. Without this, two calls
     * naming the same sectors each get their own context and each believes it owns
     * them: one writes a record, the other erases the sector it lives in, and the
     * first region's data is gone with nothing reporting it. */
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

        if (first_sector <= regions[i].last_sector &&
            (first_sector + sector_count - 1u) >= regions[i].first_sector)
        {
            return NULL; /* overlaps a live region */
        }
    }

    if (c == NULL)
    {
        return NULL;
    }

    uint32_t last  = first_sector + sector_count - 1u;
    uint32_t bytes = 0u;

    for (uint32_t s = first_sector; s <= last; s++)
    {
        bytes += sector_bytes[s];
    }

    c->first_sector = first_sector;
    c->last_sector  = last;
    c->base         = sector_addr[first_sector];
    c->bytes        = bytes;
    c->in_use       = true;

    return c;
}

const Flash_Ops_s* IMPL_STM32_FLASH_GetOps(void) { return &stm32_flash_ops; }

/* No-op: the context names a sector range rather than owning an allocation,
 * so DestroyCtx exists only for symmetry with the allocating backends and has
 * nothing to release. */
void IMPL_STM32_FLASH_DestroyCtx(void* ctx) { (void) ctx; }

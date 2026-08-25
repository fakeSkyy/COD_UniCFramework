/**
 * @file plat_flash.c
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 */

/* This file defines the construction functions, so it opens the gate on itself.
 * See the gate in the header for why they are not visible by default. */
#define PLAT_ALLOW_CONSTRUCTION

#include "plat_flash.h"

#include "plat_memory.h"

/* ========================================================================= */
/*  Lifecycle                                                                */
/* ========================================================================= */

bool PLAT_Flash_Init(Flash_Instance_s* inst, const Flash_Ops_s* ops, void* ctx)
{
    if (inst == NULL || ops == NULL || ctx == NULL)
    {
        return false;
    }

    /* Every member is reached unconditionally by the calls below, so a partial
     * vtable would fault on first use rather than here. Check once instead. */
    if (ops->read == NULL || ops->write == NULL || ops->erase == NULL || ops->sector_of == NULL ||
        ops->sector_base == NULL || ops->sector_size == NULL || ops->size == NULL ||
        ops->is_erased == NULL || ops->write_granularity == NULL)
    {
        return false;
    }

    /* A zero-length region would make every offset out of range, i.e. an instance
     * that can never do anything. Reject it at creation. */
    if (ops->size(ctx) == 0u)
    {
        return false;
    }

    /* The alignment test in Write masks with (granularity - 1), which is only
     * valid for a power of two. Verify it here rather than trusting each backend. */
    uint32_t gran = ops->write_granularity(ctx);

    if (gran == 0u || (gran & (gran - 1u)) != 0u)
    {
        return false;
    }

    inst->ops = ops;
    inst->ctx = ctx;
    inst->id  = NULL;

    return true;
}

Flash_Instance_s* PLAT_Flash_Create(const Flash_Ops_s* ops, void* ctx)
{
    Flash_Instance_s* inst = PLAT_malloc(sizeof(Flash_Instance_s));

    if (inst == NULL)
    {
        return NULL;
    }

    if (!PLAT_Flash_Init(inst, ops, ctx))
    {
        PLAT_free(inst);
        return NULL;
    }

    return inst;
}

/**
 * @brief Test whether a range lies wholly inside the region.
 *
 * @par Why this is checked here as well as in the backend
 * The contract makes the backend responsible for bounds, and the current backends
 * are correct. But the naive form of this test — @c off+len>size — silently passes a
 * huge @p len that wraps the addition, and a backend written that way turns a bad
 * argument into a write outside the region. On internal flash that region is
 * adjacent to the running firmware.
 *
 * The check costs two comparisons on a path that already blocks for
 * milliseconds, and it means one careless backend cannot reach past its bounds.
 * Written with a subtraction so the arithmetic cannot overflow.
 *
 * @param f    Instance whose region bounds are used.
 * @param off  Region-relative start offset.
 * @param len  Length in bytes.
 * @return true when the whole range is inside the region.
 */
static bool range_inside(Flash_Instance_s* f, uint32_t off, size_t len)
{
    uint32_t size = f->ops->size(f->ctx);

    if (len == 0u || off >= size)
    {
        return false;
    }

    return len <= (size_t) (size - off);
}

/* ========================================================================= */
/*  Access                                                                   */
/* ========================================================================= */

bool PLAT_Flash_Read(Flash_Instance_s* f, uint32_t off, uint8_t* dst, size_t len)
{
    if (f == NULL || dst == NULL || !range_inside(f, off, len))
    {
        return false;
    }

    return f->ops->read(f->ctx, off, dst, len);
}

bool PLAT_Flash_Write(Flash_Instance_s* f, uint32_t off, const uint8_t* src, size_t len)
{
    if (f == NULL || src == NULL || !range_inside(f, off, len))
    {
        return false;
    }

    /* Enforce granularity here rather than in each backend. On a device with a
     * write-once flash word a misaligned write cannot be emulated at all, so
     * catching it at the boundary turns a silently corrupt record into a plain
     * false — and the check is free on an F4, where the granularity is 1. */
    uint32_t gran = f->ops->write_granularity(f->ctx);

    if ((off & (gran - 1u)) != 0u || (len & (size_t) (gran - 1u)) != 0u)
    {
        return false;
    }

    return f->ops->write(f->ctx, off, src, len);
}

bool PLAT_Flash_EraseSector(Flash_Instance_s* f, uint32_t off)
{
    if (f == NULL || !range_inside(f, off, 1u))
    {
        return false;
    }

    return f->ops->erase(f->ctx, off);
}

/* ========================================================================= */
/*  Geometry                                                                 */
/* ========================================================================= */

uint32_t PLAT_Flash_Size(Flash_Instance_s* f) { return (f != NULL) ? f->ops->size(f->ctx) : 0u; }

uint32_t PLAT_Flash_SectorOf(Flash_Instance_s* f, uint32_t off)
{
    return (f != NULL) ? f->ops->sector_of(f->ctx, off) : 0u;
}

uint32_t PLAT_Flash_SectorBase(Flash_Instance_s* f, uint32_t off)
{
    return (f != NULL) ? f->ops->sector_base(f->ctx, off) : 0u;
}

uint32_t PLAT_Flash_SectorSize(Flash_Instance_s* f, uint32_t off)
{
    return (f != NULL) ? f->ops->sector_size(f->ctx, off) : 0u;
}

uint32_t PLAT_Flash_WriteGranularity(Flash_Instance_s* f)
{
    return (f != NULL) ? f->ops->write_granularity(f->ctx) : 0u;
}

bool PLAT_Flash_IsErased(Flash_Instance_s* f, uint32_t off, size_t len)
{
    if (f == NULL || !range_inside(f, off, len))
    {
        return false;
    }

    return f->ops->is_erased(f->ctx, off, len);
}

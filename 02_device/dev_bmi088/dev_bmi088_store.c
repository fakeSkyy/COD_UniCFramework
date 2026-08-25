/**
 * @file dev_bmi088_store.c
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 */

#include "dev_bmi088_store.h"

#include <stddef.h>

#include "util_crc.h"
#include "util_fast_math.h"

/* ========================================================================= */
/*  Record format                                                            */
/* ========================================================================= */

/**
 * @brief Marks a written record. Chosen so erased flash (all 0xFF) cannot match.
 *
 * This is the first line of defence: without it, a blank sector's 0xFF bytes read
 * back as NaN floats and would be installed as a bias, poisoning every sample.
 */
#define STORE_MAGIC 0xB1A5C0DEu

/** @brief Format version, so an old record is rejected rather than misread. */
#define STORE_VERSION 1u

/**
 * @brief On-flash layout of a stored bias.
 *
 * Fields are ordered largest-first so the struct needs no padding on any ABI —
 * padding bytes would be uninitialised, which would make the checksum depend on
 * whatever the compiler left in the gaps.
 *
 * @par Why it is padded to 32 bytes
 * The natural size is 20, but an STM32H7 programs a 256-bit flash word and each
 * word may be written only once between erases, so its granularity is 32 and a
 * 20-byte write is not expressible there. Padding to 32 makes one layout valid on
 * both an F4 (granularity 1) and an H7, at the cost of 12 wasted bytes in a
 * 128 KB sector. The pad is explicit and checksummed rather than implicit, so it
 * cannot carry indeterminate values into the CRC.
 */
typedef struct
{
    uint32_t magic;   /**< STORE_MAGIC when written.                */
    float    bias[3]; /**< Gyro bias, rad/s, sensor frame.          */
    uint16_t version; /**< STORE_VERSION.                           */
    uint16_t crc;     /**< CRC-16 over magic..pad (see record_crc). */
    uint8_t  pad[12]; /**< Zeroed; brings the record to 32 bytes.   */
} Record_s;

/* The header advertises a size to callers sizing their flash budget; bind it to
 * the real layout so adding a field cannot silently leave the two disagreeing. */
_Static_assert(sizeof(Record_s) == DEV_BMI088_STORE_SIZE,
               "DEV_BMI088_STORE_SIZE must match the on-flash record");

/* 32 bytes covers every granularity this code is expected to meet: 1 on an F4,
 * 32 on an H7. A device needing more would need a larger record, which this
 * assertion turns into a build failure rather than a runtime write that fails. */
_Static_assert(sizeof(Record_s) % 32u == 0u,
               "record must be a multiple of the largest supported write granularity");

/**
 * @brief CRC over everything in the record except the CRC field itself.
 *
 * The length is @c offsetof(crc), not @c sizeof minus the trailing fields. The
 * pad sits after the CRC precisely so that the checksummed region is the whole
 * prefix of the struct with no gap: an earlier layout put the pad before the CRC,
 * which left two bytes of implicit tail padding and made
 * @c sizeof-sizeof(uint16_t) overshoot into the CRC field itself. That folded the
 * checksum into its own input — matching at save time, when crc is still zero,
 * and never at load time.
 *
 * @param r  Record to checksum.
 * @return CRC-16 of the bytes preceding the CRC field.
 */
static uint16_t record_crc(const Record_s* r)
{
    return UTIL_CRC16_Calc((const uint8_t*) r, offsetof(Record_s, crc), UTIL_CRC16_INIT);
}

/* ========================================================================= */
/*  Public                                                                   */
/* ========================================================================= */

bool DEV_BMI088_LoadBias(DEV_BMI088_s* imu, Flash_Instance_s* flash, uint32_t off)
{
    if (imu == NULL || flash == NULL)
    {
        return false;
    }

    Record_s r;

    if (!PLAT_Flash_Read(flash, off, (uint8_t*) &r, sizeof r))
    {
        return false;
    }

    /* Three independent checks. The magic rejects a blank sector, the version
     * rejects a record this build cannot interpret, and the CRC rejects one that
     * was corrupted or half-written by a power loss mid-save. */
    if (r.magic != STORE_MAGIC)
    {
        return false;
    }
    if (r.version != STORE_VERSION)
    {
        return false;
    }
    if (r.crc != record_crc(&r))
    {
        return false;
    }

    /* Even a CRC-valid record gets a sanity check on the values. A bias larger
     * than a MEMS gyro can plausibly have means the record is from a different
     * sensor or a bad calibration, and installing it would be worse than having
     * none. */
    for (uint8_t i = 0u; i < 3u; i++)
    {
        if (!UTIL_IsFinitef(r.bias[i]) || UTIL_Absf(r.bias[i]) > 0.5f)
        {
            return false;
        }
    }

    DEV_BMI088_SetGyroBias(imu, r.bias);

    return true;
}

bool DEV_BMI088_SaveBias(DEV_BMI088_s* imu, Flash_Instance_s* flash, uint32_t off)
{
    if (imu == NULL || flash == NULL)
    {
        return false;
    }

    /* Refuse to store an unmeasured bias. Without this a caller who forgot to
     * calibrate would persist all zeros, and the next boot would load them as if
     * they were a real measurement — silently worse than finding nothing. */
    if (!DEV_BMI088_IsCalibrated(imu))
    {
        return false;
    }

    Record_s r;

    /* Zero the whole record first: the pad bytes feed the CRC, so leaving them
     * indeterminate would make the checksum depend on stack contents and a
     * reload could fail on a record that was written correctly. */
    for (uint32_t i = 0u; i < sizeof r; i++)
    {
        ((uint8_t*) &r)[i] = 0u;
    }

    r.magic   = STORE_MAGIC;
    r.version = STORE_VERSION;

    DEV_BMI088_GetGyroBias(imu, r.bias);

    for (uint8_t i = 0u; i < 3u; i++)
    {
        if (!UTIL_IsFinitef(r.bias[i]))
        {
            return false;
        }
    }

    r.crc = record_crc(&r);

    /* Skip the erase when the target is already blank. Worth the check: an erase
     * costs 1-2 seconds of stalled CPU, and on a first save the sector usually is
     * blank. */
    if (!PLAT_Flash_IsErased(flash, off, sizeof r))
    {
        if (!PLAT_Flash_EraseSector(flash, off))
        {
            return false;
        }
    }

    /* Write verifies by reading back, so a partial program is reported here
     * rather than discovered at the next load. */
    return PLAT_Flash_Write(flash, off, (const uint8_t*) &r, sizeof r);
}

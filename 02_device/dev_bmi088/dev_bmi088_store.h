/**
 * @file dev_bmi088_store.h
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 */

#ifndef DEV_BMI088_STORE_H
#define DEV_BMI088_STORE_H

#include "dev_bmi088.h"
#include "plat_flash.h"

/**
 * @brief Persist a BMI088 gyro bias across power cycles.
 *
 * @par Why this is worth having
 * DEV_BMI088_CalibrateGyro blocks for a second or two and refuses to run unless
 * the robot is genuinely still — neither of which is convenient at every boot.
 * Storing the result means a normal start-up loads a known bias in microseconds
 * and only an explicit recalibration pays the wait.
 *
 * It also covers what UTIL_AHRS cannot. That filter estimates the x and y bias
 * online, but an accelerometer cannot observe rotation about gravity, so the z
 * bias is unobservable and yaw drifts at whatever it happens to be. A stored z
 * bias is the only thing that removes it.
 *
 * @par Validation, and why a blank sector must not read as a bias
 * Erased flash reads as 0xFF everywhere, which as a float is NaN — and as a bias
 * would poison every sample. The record therefore carries a magic number, a
 * version and a checksum, and Load reports failure unless all three agree. A
 * first boot on a blank sector is then indistinguishable from no stored data,
 * which is exactly right.
 */

/**
 * @brief Bytes of flash one record occupies.
 *
 * 32 rather than the 20 the fields need, so that one layout satisfies the write
 * granularity of every target: 1 byte on an STM32F4, but 32 on an STM32H7, whose
 * 256-bit flash word may be programmed only once between erases. Two compile-time
 * assertions in the .c tie this to the real record and to that granularity.
 */
#define DEV_BMI088_STORE_SIZE 32u

/**
 * @brief Load a stored gyro bias and install it.
 *
 * Reads are cheap and never stall the CPU, so this is safe to call during normal
 * start-up.
 *
 * @param imu    Instance to configure. Must be initialized.
 * @param flash  Region holding the record.
 * @param off    Region-relative offset of the record. Must leave
 *               @ref DEV_BMI088_STORE_SIZE bytes inside the region.
 * @return true when a valid record was found and its bias installed; false when
 *         the region is blank, the record is corrupt, or its version is unknown —
 *         in which case the instance's bias is left untouched and the caller
 *         should calibrate instead.
 */
bool DEV_BMI088_LoadBias(DEV_BMI088_s* imu, Flash_Instance_s* flash, uint32_t off);

/**
 * @brief Write the instance's current gyro bias to flash.
 *
 * @warning This erases a flash sector, which stalls the CPU for 1-2 seconds with
 *          interrupts disabled — the core cannot fetch instructions from flash
 *          while an erase is in progress. Call it at bring-up or in an explicit
 *          maintenance state, never from a control loop and never with motors
 *          enabled.
 *
 * @warning The erase destroys the ENTIRE sector containing @p off, not just the
 *          record. On an STM32F407 that is up to 128 KB. Anything else stored in
 *          the same sector is lost — check with PLAT_Flash_SectorOf first.
 *
 * @param imu    Instance whose bias is saved. Must be calibrated, so that a
 *               default of all zeros cannot be stored as if it were measured.
 * @param flash  Region to write to.
 * @param off    Region-relative offset for the record.
 * @return true when the record was written and read back identical.
 */
bool DEV_BMI088_SaveBias(DEV_BMI088_s* imu, Flash_Instance_s* flash, uint32_t off);

#endif /* DEV_BMI088_STORE_H */

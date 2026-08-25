/**
 * @file dev_bmi088.h
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 */

#ifndef DEV_BMI088_H
#define DEV_BMI088_H

#include <stdbool.h>
#include <stdint.h>

#include "dev_watchdog.h"
#include "plat_dwt.h"
#include "plat_spi.h"

/* ========================================================================= */
/*  Ranges                                                                   */
/* ========================================================================= */

/**
 * @brief Accelerometer full-scale range.
 *
 * The enum is the single source of truth: the register value and the sensitivity
 * are both derived from it internally, so the two cannot disagree. The legacy
 * header declared the range with one macro, wrote a different one to the
 * register, and picked the sensitivity from a third place — currently consistent
 * only by luck, and a silent factor-of-two error waiting for whoever edits one
 * of the three.
 */
typedef enum
{
    DEV_BMI088_ACC_RANGE_3G = 0, /**< +/-3 g, finest resolution.   */
    DEV_BMI088_ACC_RANGE_6G,     /**< +/-6 g, the usual choice.     */
    DEV_BMI088_ACC_RANGE_12G,    /**< +/-12 g.                      */
    DEV_BMI088_ACC_RANGE_24G,    /**< +/-24 g, for heavy impacts.   */
} DEV_BMI088_AccRange_e;

/**
 * @brief Gyroscope full-scale range, in degrees per second.
 *
 * As with the accelerometer range, this drives both the register value and the
 * sensitivity.
 */
typedef enum
{
    DEV_BMI088_GYRO_RANGE_2000 = 0, /**< +/-2000 dps, the usual choice. */
    DEV_BMI088_GYRO_RANGE_1000,     /**< +/-1000 dps.                   */
    DEV_BMI088_GYRO_RANGE_500,      /**< +/-500 dps.                    */
    DEV_BMI088_GYRO_RANGE_250,      /**< +/-250 dps.                    */
    DEV_BMI088_GYRO_RANGE_125,      /**< +/-125 dps, finest resolution. */
} DEV_BMI088_GyroRange_e;

/* ========================================================================= */
/*  Status                                                                   */
/* ========================================================================= */

/**
 * @brief Outcome of an initialization attempt.
 *
 * Returned so a failure at bring-up names the step that failed instead of
 * looping silently. The legacy code accumulated its status with @c |= inside a
 * @c do-while, which made the value monotonically non-decreasing: one failure
 * and the loop never terminated even when every later attempt succeeded.
 */
typedef enum
{
    DEV_BMI088_OK = 0,          /**< Both sensors configured and verified.   */
    DEV_BMI088_ERR_ARG,         /**< NULL pointer or bad argument.           */
    DEV_BMI088_ERR_ACC_ID,      /**< Accelerometer chip ID did not read back. */
    DEV_BMI088_ERR_GYRO_ID,     /**< Gyroscope chip ID did not read back.    */
    DEV_BMI088_ERR_ACC_CONFIG,  /**< An accelerometer register failed verify. */
    DEV_BMI088_ERR_GYRO_CONFIG, /**< A gyroscope register failed verify.     */
    DEV_BMI088_ERR_SPI,         /**< A bus transfer failed outright.         */
} DEV_BMI088_Status_e;

/* ========================================================================= */
/*  Configuration                                                            */
/* ========================================================================= */

/**
 * @brief Everything needed to bring up one BMI088.
 *
 * @par The two dies need two SPI instances
 * A BMI088 is an accelerometer die and a gyroscope die in one package, with
 * independent register maps and separate chip selects. They are NOT
 * interchangeable — in particular the accelerometer inserts a dummy byte in
 * every SPI read and the gyroscope does not, so the read paths genuinely differ.
 */
typedef struct
{
    SPI_Instance_s* spi_accel; /**< Device handle for the accelerometer die. */
    SPI_Instance_s* spi_gyro;  /**< Device handle for the gyroscope die.     */
    DWT_Instance_s* timebase;  /**< Used for the delays the datasheet requires. */

    DEV_BMI088_AccRange_e  acc_range;  /**< Accelerometer full-scale range. */
    DEV_BMI088_GyroRange_e gyro_range; /**< Gyroscope full-scale range.     */

    /**
     * @brief How many bring-up attempts before giving up. 0 selects 3.
     *
     * Each attempt begins with a full soft reset, so a retry is a genuine fresh
     * start rather than a repeat of the half-configured state that failed.
     */
    uint8_t max_attempts;

    /**
     * @brief Read the temperature only once every N calls to Read. 0 selects 100.
     *
     * The temperature register has 0.125 degC resolution and a thermal time
     * constant measured in seconds, so sampling it at the gyro rate spends a
     * whole extra SPI transaction per cycle to observe nothing. At 1 kHz the
     * default gives 10 Hz, which is far faster than the sensor can actually
     * change. Pass 1 to read it every cycle.
     */
    uint16_t temp_divider;
} DEV_BMI088_Cfg_s;

/* ========================================================================= */
/*  Instance                                                                 */
/* ========================================================================= */

/**
 * @brief BMI088 six-axis IMU over caller-owned storage.
 *
 * Reads calibrated angular rate, acceleration and die temperature. The module
 * touches no HAL and no vendor header — it talks to the platform SPI and delay
 * interfaces only, so it compiles and can be exercised on a host with a stub
 * backend.
 *
 * @par Axis and sign convention
 * Values come out in the sensor's own frame, unrotated: whatever the datasheet
 * says about the package is what appears here. Mapping that onto the vehicle
 * frame is the caller's job, because only the caller knows how the board is
 * mounted. The legacy driver mixed the two by indexing its output arrays with
 * pitch/roll/yaw macros from an application header.
 *
 * @par Units
 * Angular rate in rad/s, acceleration in m/s^2, temperature in degrees Celsius.
 * The legacy driver returned the gyro in rad/s but the accelerometer in g, which
 * is a trap for anything that treats the two as one measurement set.
 *
 * @par Allocation and concurrency
 * Caller-owned storage, no dynamic allocation. An instance must be read from one
 * context only — the two dies are read in sequence and share no lock.
 */
typedef struct
{
    DEV_BMI088_Cfg_s cfg; /**< Configuration, copied at Init. */

    float gyro[3];  /**< Bias-corrected angular rate, rad/s. */
    float accel[3]; /**< Acceleration, m/s^2.                */
    float temp_c;   /**< Die temperature, degrees Celsius.   */

    float gyro_bias[3]; /**< Subtracted from every gyro sample, rad/s. */

    float acc_sensitivity;  /**< Raw LSB to m/s^2.  */
    float gyro_sensitivity; /**< Raw LSB to rad/s.  */

    uint16_t temp_countdown; /**< Cycles until the next temperature read. */

    uint32_t read_count;      /**< Successful Read calls since Init.      */
    uint32_t spi_error_count; /**< Failed bus transfers since Init.       */

    /**
     * @brief Liveness node, kicked by every Read that returns a verified sample.
     *
     * Kicked on success only, so a bus that keeps failing stops the kicks rather
     * than reporting a live sensor. Pass it to DEV_Watchdog_Register to have it
     * supervised; a driver that is never registered simply keeps a node nobody
     * reads.
     */
    DEV_Watchdog_s wd;

    bool initialized; /**< False until Init succeeds.               */
    bool calibrated;  /**< True once a gyro bias has been applied.  */
} DEV_BMI088_s;

/* ========================================================================= */
/*  API                                                                      */
/* ========================================================================= */

/**
 * @brief Bring up both dies and verify every register written.
 *
 * Blocks for roughly 165 ms per attempt, of which 160 ms is the two soft-reset
 * waits the datasheet mandates — no implementation can shorten those. This is
 * intended to be called from Board_Init, before the scheduler starts, where
 * there is no other task to starve.
 *
 * Every configuration register is read back and compared. A mismatch fails the
 * attempt rather than leaving a sensor half-configured, and the returned status
 * names which group failed.
 *
 * @param imu  Instance to initialize.
 * @param cfg  Configuration to copy.
 * @return DEV_BMI088_OK on success. On any other value the instance is left
 *         uninitialized and DEV_BMI088_Read returns false with zeroed outputs,
 *         so a missing IMU cannot masquerade as a stationary one.
 */
DEV_BMI088_Status_e DEV_BMI088_Init(DEV_BMI088_s* imu, const DEV_BMI088_Cfg_s* cfg);

/**
 * @brief Read one sample of rate, acceleration and (periodically) temperature.
 *
 * @param imu  Instance to read.
 * @return true when the sample is valid. On false the cached values are left
 *         untouched — holding the previous sample for one cycle is better than
 *         handing a control loop a zero that reads as "level and stationary".
 *
 * @note The gyro read verifies the chip ID that precedes the data in the same
 *       burst. A mismatch means the bus is desynchronised, so the sample is
 *       rejected rather than decoded from bytes that may be shifted.
 */
bool DEV_BMI088_Read(DEV_BMI088_s* imu);

/**
 * @brief Measure the gyro bias, refusing to run unless the sensor is still.
 *
 * Averages @p samples readings and adopts the mean as the bias. Before
 * committing it, the spread of those readings is checked: if the sensor moved
 * during the window, the mean is contaminated with real rotation and adopting it
 * would make the gyro permanently report that motion as zero. The legacy version
 * ran 5000 blocking one-millisecond samples with no such check.
 *
 * All three axes are measured, including z. UTIL_AHRS estimates only the x and y
 * bias — an accelerometer cannot observe rotation about gravity — so the z bias
 * has to be removed here or yaw drifts at whatever it happens to be.
 *
 * Blocks for approximately @p samples milliseconds. Intended for start-up or for
 * an explicit operator-requested recalibration, not for the control loop.
 *
 * @param imu      Instance to calibrate. Must be initialized.
 * @param samples  Readings to average; clamped to [100, 5000]. 2000 at 1 kHz is
 *                 two seconds and averages the noise down by about 45x.
 * @return true when a new bias was adopted; false if the instance is
 *         uninitialized, the bus failed, or the sensor was moving — in which
 *         case the previous bias is left in place.
 */
bool DEV_BMI088_CalibrateGyro(DEV_BMI088_s* imu, uint16_t samples);

/**
 * @brief Install a known gyro bias, e.g. one restored from flash.
 *
 * Faster than calibrating at every boot, and usable when the vehicle cannot be
 * guaranteed still at start-up. Combine the two: load the stored value here, then
 * refine with DEV_BMI088_CalibrateGyro once the robot is known to be at rest.
 *
 * @param imu   Instance to configure.
 * @param bias  Three bias values in rad/s, in the sensor frame. A non-finite
 *              entry leaves that axis unchanged.
 */
void DEV_BMI088_SetGyroBias(DEV_BMI088_s* imu, const float* bias);

/* ========================================================================= */
/*  Output                                                                   */
/* ========================================================================= */

/**
 * @brief Bias-corrected angular rate, rad/s, in the sensor frame.
 * @param imu  Instance to query.
 * @return Pointer to three floats, x/y/z.
 */
static inline const float* DEV_BMI088_GetGyro(const DEV_BMI088_s* imu) { return imu->gyro; }

/**
 * @brief Acceleration, m/s^2, in the sensor frame.
 *
 * At rest the axis pointing up reads +9.8, matching what UTIL_AHRS_Update
 * expects.
 *
 * @param imu  Instance to query.
 * @return Pointer to three floats, x/y/z.
 */
static inline const float* DEV_BMI088_GetAccel(const DEV_BMI088_s* imu) { return imu->accel; }

/**
 * @brief Die temperature in degrees Celsius.
 *
 * Refreshed once every @c temp_divider calls to Read, not every call. Feed this
 * to a heater loop where one exists — a BMI088 held at a constant temperature
 * has a far more stable bias than one left to follow ambient.
 *
 * @param imu  Instance to query.
 * @return Temperature, degC.
 */
static inline float DEV_BMI088_GetTemperature(const DEV_BMI088_s* imu) { return imu->temp_c; }

/**
 * @brief The gyro bias currently being subtracted, rad/s.
 * @param imu  Instance to query.
 * @param out  Destination for three floats. Store these to flash to skip the
 *             calibration wait on the next boot.
 */
void DEV_BMI088_GetGyroBias(const DEV_BMI088_s* imu, float* out);

/**
 * @brief Test whether a gyro bias has been established.
 * @param imu  Instance to query.
 * @return true after a successful CalibrateGyro or SetGyroBias.
 */
static inline bool DEV_BMI088_IsCalibrated(const DEV_BMI088_s* imu) { return imu->calibrated; }

/**
 * @brief Number of failed SPI transfers since Init.
 *
 * Should stay at zero. A rising count means wiring, clock rate or contention on
 * a shared bus — worth surfacing rather than silently retrying forever.
 *
 * @param imu  Instance to query.
 * @return Failure count.
 */
static inline uint32_t DEV_BMI088_GetSpiErrorCount(const DEV_BMI088_s* imu)
{
    return imu->spi_error_count;
}

#endif /* DEV_BMI088_H */

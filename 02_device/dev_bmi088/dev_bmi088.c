/**
 * @file dev_bmi088.c
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 */

#include "dev_bmi088.h"

#include "dev_bmi088_reg.h"
#include <math.h>

#include "util_fast_math.h"

/* ========================================================================= */
/*  Timing and limits                                                        */
/* ========================================================================= */

/** @brief Soft-reset settling time, milliseconds. Mandated by the datasheet. */
#define RESET_DELAY_MS 80u

/** @brief Settling time after touching a configuration register, microseconds. */
#define REG_DELAY_US 150u

/** @brief SPI transfer timeout handed to the platform layer. */
#define SPI_TIMEOUT 10u

/** @brief Default bring-up attempts before reporting failure. */
#define DEFAULT_MAX_ATTEMPTS 3u

/** @brief Default temperature read divider; 10 Hz when Read runs at 1 kHz. */
#define DEFAULT_TEMP_DIVIDER 100u

/** @brief Bounds on the calibration sample count. */
#define CALIB_MIN_SAMPLES 100u
#define CALIB_MAX_SAMPLES 5000u

/**
 * @brief Largest sample standard deviation, in rad/s, that still counts as still.
 *
 * Standard deviation rather than peak-to-peak, because peak-to-peak grows with the
 * sample count while the underlying noise does not: the extremes of 2000 draws sit
 * near +/-3.5 sigma, so a spread test that passes a 200-sample window fails a 2000
 * one for no physical reason. That is not hypothetical — it is what this file did.
 * Measured on the bench at 0.0132 rad/s of gyro noise, every axis produced a
 * peak-to-peak of 0.065 to 0.088 against a 0.05 ceiling and every calibration was
 * rejected, while the means came in two orders of magnitude inside their own limit
 * and proved the board had been sitting still the whole time.
 *
 * 0.05 rad/s is about 3.8x that measured noise, so stillness passes with real
 * margin, while a nudge (0.1 rad/s and up) still fails. Unlike the old test, this
 * threshold means the same thing whatever IMU_CALIB_SAMPLES is set to.
 */
#define CALIB_STILL_STD 0.05f

/**
 * @brief Largest mean magnitude, in rad/s, that a bias may plausibly have.
 *
 * A real BMI088 bias is well under 0.1 rad/s. A larger mean means the sensor was
 * rotating steadily through the whole window — which the spread test cannot
 * catch, because a constant rotation has a small spread.
 */
#define CALIB_MAX_BIAS 0.15f

/** @brief Standard gravity, for converting the accelerometer from g to m/s^2. */
#define GRAVITY_MSS 9.80665f

/** @brief Degrees to radians, for converting the gyro from dps. */
#define DEG_TO_RAD 0.01745329252f

/** @brief Temperature scale, degC per LSB, and its offset. */
#define TEMP_SCALE 0.125f
#define TEMP_OFFSET 23.0f

/* ========================================================================= */
/*  Range tables                                                             */
/* ========================================================================= */

/**
 * @brief Register value for each accelerometer range, indexed by the enum.
 *
 * Keeping the register value and the sensitivity in two arrays indexed by the
 * same enum is what makes them impossible to mismatch: adding a range means
 * adding one entry to each, and the compiler will not let the enum drift.
 */
static const uint8_t acc_range_reg[] = {
    BMI088_ACC_RANGE_3G,
    BMI088_ACC_RANGE_6G,
    BMI088_ACC_RANGE_12G,
    BMI088_ACC_RANGE_24G,
};

/**
 * @brief Sensitivity in g per LSB for each accelerometer range.
 *
 * From the datasheet: full scale spans 32768 counts, so 3 g gives
 * 3 * 2 / 65536 = 9.1553e-5 g/LSB... but Bosch specifies these directly as
 * 1/10920, 1/5460, 1/2730 and 1/1365 LSB per g respectively.
 */
static const float acc_sens_g[] = {
    1.0f / 10920.0f,
    1.0f / 5460.0f,
    1.0f / 2730.0f,
    1.0f / 1365.0f,
};

/** @brief Register value for each gyroscope range, indexed by the enum. */
static const uint8_t gyro_range_reg[] = {
    BMI088_GYRO_RANGE_2000, BMI088_GYRO_RANGE_1000, BMI088_GYRO_RANGE_500,
    BMI088_GYRO_RANGE_250,  BMI088_GYRO_RANGE_125,
};

/** @brief Full scale in degrees per second for each gyroscope range. */
static const float gyro_fs_dps[] = {2000.0f, 1000.0f, 500.0f, 250.0f, 125.0f};

/* ========================================================================= */
/*  Bus primitives                                                           */
/* ========================================================================= */

/**
 * @brief Write one register.
 *
 * @param spi  Device handle.
 * @param reg  Register address.
 * @param val  Value to write.
 * @return true when the transfer succeeded.
 */
static bool reg_write(SPI_Instance_s* spi, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {(uint8_t) (reg & 0x7Fu), val};

    /* The transaction is one held selection, and the hold reserves the bus — the
     * other die sits on this same peripheral, so an interleaved transfer would have
     * both chip selects low at once. A refusal means that die is mid-transaction. */
    if (!PLAT_SPI_Select(spi))
    {
        return false;
    }

    bool ok = PLAT_SPI_Send(spi, tx, 2u, SPI_TIMEOUT);
    PLAT_SPI_Deselect(spi);

    return ok;
}

/**
 * @brief Read a burst of registers, discarding a leading dummy byte if asked.
 *
 * @par Why the dummy byte is a parameter
 * The accelerometer die returns one throwaway byte between the address and the
 * data; the gyroscope die does not. Bosch confirms this asymmetry, and it is the
 * reason the two dies need separate read paths. The legacy driver used one path
 * for both and discarded the byte in neither, so every accelerometer sample was
 * assembled from bytes shifted by one position — which is exactly why its gyro
 * worked and its accelerometer did not.
 *
 * @param spi    Device handle.
 * @param reg    First register address.
 * @param dst    Destination for @p len bytes.
 * @param len    Payload bytes wanted.
 * @param dummy  True for the accelerometer die, false for the gyroscope.
 * @return true when the transfer succeeded.
 */
static bool reg_read(SPI_Instance_s* spi, uint8_t reg, uint8_t* dst, uint8_t len, bool dummy)
{
    uint8_t addr = (uint8_t) (reg | BMI088_SPI_READ_BIT);

    /* Address then data must be one uninterrupted selection, which is also what
     * reserves the bus against the other die — see reg_write. */
    if (!PLAT_SPI_Select(spi))
    {
        return false;
    }

    bool ok = PLAT_SPI_Send(spi, &addr, 1u, SPI_TIMEOUT);

    if (ok && dummy)
    {
        uint8_t throwaway;
        ok = PLAT_SPI_Receive(spi, &throwaway, 1u, SPI_TIMEOUT);
    }

    if (ok)
    {
        ok = PLAT_SPI_Receive(spi, dst, len, SPI_TIMEOUT);
    }

    PLAT_SPI_Deselect(spi);

    return ok;
}

/**
 * @brief Read one register from the accelerometer die.
 * @param imu  Instance holding the handles.
 * @param reg  Register address.
 * @param out  Destination for one byte.
 * @return true on success.
 */
static bool acc_read1(DEV_BMI088_s* imu, uint8_t reg, uint8_t* out)
{
    return reg_read(imu->cfg.spi_accel, reg, out, 1u, true);
}

/**
 * @brief Read one register from the gyroscope die.
 * @param imu  Instance holding the handles.
 * @param reg  Register address.
 * @param out  Destination for one byte.
 * @return true on success.
 */
static bool gyro_read1(DEV_BMI088_s* imu, uint8_t reg, uint8_t* out)
{
    return reg_read(imu->cfg.spi_gyro, reg, out, 1u, false);
}

/**
 * @brief Write a register and verify it reads back.
 *
 * @param imu    Instance holding the handles and timebase.
 * @param accel  True to address the accelerometer die.
 * @param reg    Register address.
 * @param val    Value to write and expect back.
 * @return true when the write took effect.
 */
static bool reg_write_verify(DEV_BMI088_s* imu, bool accel, uint8_t reg, uint8_t val)
{
    SPI_Instance_s* spi = accel ? imu->cfg.spi_accel : imu->cfg.spi_gyro;

    if (!reg_write(spi, reg, val))
    {
        return false;
    }

    PLAT_DWT_Delay_us(imu->cfg.timebase, REG_DELAY_US);

    uint8_t readback = 0u;
    bool    ok       = accel ? acc_read1(imu, reg, &readback) : gyro_read1(imu, reg, &readback);

    if (!ok)
    {
        return false;
    }

    PLAT_DWT_Delay_us(imu->cfg.timebase, REG_DELAY_US);

    return (readback == val);
}

/* ========================================================================= */
/*  Bring-up                                                                 */
/* ========================================================================= */

/**
 * @brief Configure the accelerometer die once.
 *
 * @param imu  Instance to configure.
 * @return DEV_BMI088_OK, or the step that failed.
 */
static DEV_BMI088_Status_e accel_setup(DEV_BMI088_s* imu)
{
    uint8_t id = 0u;

    /* The accelerometer powers up in I2C mode and needs one throwaway SPI read
     * to switch the bus over, so the first ID read is expected to return
     * rubbish. Only the second one is meaningful. */
    (void) acc_read1(imu, BMI088_ACC_CHIP_ID, &id);
    PLAT_DWT_Delay_us(imu->cfg.timebase, REG_DELAY_US);

    if (!acc_read1(imu, BMI088_ACC_CHIP_ID, &id))
    {
        return DEV_BMI088_ERR_SPI;
    }
    if (id != BMI088_ACC_CHIP_ID_VALUE)
    {
        return DEV_BMI088_ERR_ACC_ID;
    }

    if (!reg_write(imu->cfg.spi_accel, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE))
    {
        return DEV_BMI088_ERR_SPI;
    }
    PLAT_DWT_Delay_ms(imu->cfg.timebase, RESET_DELAY_MS);

    /* The reset returns the die to I2C mode, so the bus-switching read is needed
     * again before the ID is trustworthy. */
    (void) acc_read1(imu, BMI088_ACC_CHIP_ID, &id);
    PLAT_DWT_Delay_us(imu->cfg.timebase, REG_DELAY_US);

    if (!acc_read1(imu, BMI088_ACC_CHIP_ID, &id))
    {
        return DEV_BMI088_ERR_SPI;
    }
    if (id != BMI088_ACC_CHIP_ID_VALUE)
    {
        return DEV_BMI088_ERR_ACC_ID;
    }

    /* Order matters: power on before leaving suspend, and set the range before
     * relying on any reading. */
    if (!reg_write_verify(imu, true, BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE))
    {
        return DEV_BMI088_ERR_ACC_CONFIG;
    }
    if (!reg_write_verify(imu, true, BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE))
    {
        return DEV_BMI088_ERR_ACC_CONFIG;
    }
    if (!reg_write_verify(imu, true, BMI088_ACC_CONF, BMI088_ACC_CONF_VALUE))
    {
        return DEV_BMI088_ERR_ACC_CONFIG;
    }
    if (!reg_write_verify(imu, true, BMI088_ACC_RANGE, acc_range_reg[imu->cfg.acc_range]))
    {
        return DEV_BMI088_ERR_ACC_CONFIG;
    }

    return DEV_BMI088_OK;
}

/**
 * @brief Configure the gyroscope die once.
 *
 * @param imu  Instance to configure.
 * @return DEV_BMI088_OK, or the step that failed.
 */
static DEV_BMI088_Status_e gyro_setup(DEV_BMI088_s* imu)
{
    uint8_t id = 0u;

    /* No bus switching here: the gyroscope die selects its interface from the
     * chip-select pin rather than latching a mode. */
    if (!gyro_read1(imu, BMI088_GYRO_CHIP_ID, &id))
    {
        return DEV_BMI088_ERR_SPI;
    }
    if (id != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return DEV_BMI088_ERR_GYRO_ID;
    }

    if (!reg_write(imu->cfg.spi_gyro, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE))
    {
        return DEV_BMI088_ERR_SPI;
    }
    PLAT_DWT_Delay_ms(imu->cfg.timebase, RESET_DELAY_MS);

    if (!gyro_read1(imu, BMI088_GYRO_CHIP_ID, &id))
    {
        return DEV_BMI088_ERR_SPI;
    }
    if (id != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return DEV_BMI088_ERR_GYRO_ID;
    }

    if (!reg_write_verify(imu, false, BMI088_GYRO_RANGE, gyro_range_reg[imu->cfg.gyro_range]))
    {
        return DEV_BMI088_ERR_GYRO_CONFIG;
    }
    if (!reg_write_verify(imu, false, BMI088_GYRO_BANDWIDTH, BMI088_GYRO_BANDWIDTH_VALUE))
    {
        return DEV_BMI088_ERR_GYRO_CONFIG;
    }
    if (!reg_write_verify(imu, false, BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE))
    {
        return DEV_BMI088_ERR_GYRO_CONFIG;
    }

    return DEV_BMI088_OK;
}

DEV_BMI088_Status_e DEV_BMI088_Init(DEV_BMI088_s* imu, const DEV_BMI088_Cfg_s* cfg)
{
    if (imu == NULL)
    {
        return DEV_BMI088_ERR_ARG;
    }

    for (uint8_t i = 0u; i < 3u; i++)
    {
        imu->gyro[i]      = 0.0f;
        imu->accel[i]     = 0.0f;
        imu->gyro_bias[i] = 0.0f;
    }

    imu->temp_c          = 0.0f;
    imu->temp_countdown  = 1u;
    imu->read_count      = 0u;
    imu->spi_error_count = 0u;
    imu->initialized     = false;
    imu->calibrated      = false;

    if (cfg == NULL || cfg->spi_accel == NULL || cfg->spi_gyro == NULL || cfg->timebase == NULL)
    {
        return DEV_BMI088_ERR_ARG;
    }

    if ((unsigned) cfg->acc_range >= (sizeof acc_range_reg / sizeof acc_range_reg[0]) ||
        (unsigned) cfg->gyro_range >= (sizeof gyro_range_reg / sizeof gyro_range_reg[0]))
    {
        return DEV_BMI088_ERR_ARG;
    }

    imu->cfg = *cfg;

    if (imu->cfg.max_attempts == 0u)
    {
        imu->cfg.max_attempts = DEFAULT_MAX_ATTEMPTS;
    }
    if (imu->cfg.temp_divider == 0u)
    {
        imu->cfg.temp_divider = DEFAULT_TEMP_DIVIDER;
    }

    /* Sensitivity is derived from the same enum that picks the register value, so
     * the scale always matches the configured range. */
    imu->acc_sensitivity  = acc_sens_g[cfg->acc_range] * GRAVITY_MSS;
    imu->gyro_sensitivity = (gyro_fs_dps[cfg->gyro_range] / 32768.0f) * DEG_TO_RAD;

    /* A bounded retry loop, each attempt starting from a fresh soft reset. The
     * legacy version accumulated its status with |=, which never cleared once
     * set, so a single failure meant an infinite loop. */
    DEV_BMI088_Status_e status = DEV_BMI088_ERR_SPI;

    for (uint8_t attempt = 0u; attempt < imu->cfg.max_attempts; attempt++)
    {
        status = accel_setup(imu);

        if (status == DEV_BMI088_OK)
        {
            status = gyro_setup(imu);
        }

        if (status == DEV_BMI088_OK)
        {
            imu->initialized    = true;
            imu->temp_countdown = 1u;

            /* 100 ms against the 1 kHz the attitude loop reads at: long enough
             * that a single rejected sample does not trip it, short enough to
             * notice before anything downstream has acted on a stale attitude for
             * long. Prepared but not registered — see dev_watchdog.h. */
            DEV_Watchdog_Init(&imu->wd, "imu", 100u);

            return DEV_BMI088_OK;
        }

        PLAT_DWT_Delay_ms(imu->cfg.timebase, 2u);
    }

    return status;
}

/* ========================================================================= */
/*  Reading                                                                  */
/* ========================================================================= */

bool DEV_BMI088_Read(DEV_BMI088_s* imu)
{
    if (imu == NULL || !imu->initialized)
    {
        return false;
    }

    uint8_t buf[8] = {0u};

    /* ---- Accelerometer: six bytes, little-endian per axis ---- */

    if (!reg_read(imu->cfg.spi_accel, BMI088_ACC_XOUT_L, buf, 6u, true))
    {
        imu->spi_error_count++;
        return false;
    }

    int16_t ax = (int16_t) ((uint16_t) buf[1] << 8 | buf[0]);
    int16_t ay = (int16_t) ((uint16_t) buf[3] << 8 | buf[2]);
    int16_t az = (int16_t) ((uint16_t) buf[5] << 8 | buf[4]);

    /* ---- Gyroscope: read from the ID so the burst carries its own check ---- */

    if (!reg_read(imu->cfg.spi_gyro, BMI088_GYRO_CHIP_ID, buf, 8u, false))
    {
        imu->spi_error_count++;
        return false;
    }

    /* A wrong ID here means the bus is desynchronised, so the rate bytes cannot
     * be trusted either. Reject the whole sample rather than decode bytes that
     * may be shifted — the legacy code kept the previous values silently in this
     * case, which hides a wiring fault as a frozen gyro. */
    if (buf[0] != BMI088_GYRO_CHIP_ID_VALUE)
    {
        imu->spi_error_count++;
        return false;
    }

    int16_t gx = (int16_t) ((uint16_t) buf[3] << 8 | buf[2]);
    int16_t gy = (int16_t) ((uint16_t) buf[5] << 8 | buf[4]);
    int16_t gz = (int16_t) ((uint16_t) buf[7] << 8 | buf[6]);

    /* ---- Temperature, but only every temp_divider cycles ---- */

    if (--imu->temp_countdown == 0u)
    {
        imu->temp_countdown = imu->cfg.temp_divider;

        uint8_t tbuf[2] = {0u, 0u};

        if (reg_read(imu->cfg.spi_accel, BMI088_ACC_TEMP_MSB, tbuf, 2u, true))
        {
            /* 11-bit two's complement: 8 bits in the MSB register and 3 more in
             * the top bits of the LSB register. */
            int16_t code = (int16_t) ((uint16_t) tbuf[0] << 3 | (tbuf[1] >> 5));

            if (code > 1023)
            {
                code = (int16_t) (code - 2048);
            }

            imu->temp_c = (float) code * TEMP_SCALE + TEMP_OFFSET;
        }
        else
        {
            /* Not fatal: the temperature is advisory, so keep the previous value
             * and let the sample through rather than dropping good rate data. */
            imu->spi_error_count++;
        }
    }

    /* ---- Commit ---- */

    imu->accel[0] = (float) ax * imu->acc_sensitivity;
    imu->accel[1] = (float) ay * imu->acc_sensitivity;
    imu->accel[2] = (float) az * imu->acc_sensitivity;

    imu->gyro[0] = (float) gx * imu->gyro_sensitivity - imu->gyro_bias[0];
    imu->gyro[1] = (float) gy * imu->gyro_sensitivity - imu->gyro_bias[1];
    imu->gyro[2] = (float) gz * imu->gyro_sensitivity - imu->gyro_bias[2];

    imu->read_count++;

    /* Kicked here rather than at the top of Read: reaching this point means both
     * dies answered and their chip IDs verified, which is the only evidence that
     * the sensor is actually present. Kicking on entry would report a sensor that
     * has been unplugged as alive for as long as the loop keeps calling. */
    DEV_Watchdog_Kick(&imu->wd, (uint32_t) PLAT_DWT_GetTimeline_ms(imu->cfg.timebase));

    return true;
}

/* ========================================================================= */
/*  Calibration                                                              */
/* ========================================================================= */

bool DEV_BMI088_CalibrateGyro(DEV_BMI088_s* imu, uint16_t samples)
{
    if (imu == NULL || !imu->initialized)
    {
        return false;
    }

    uint32_t n = samples;

    if (n < CALIB_MIN_SAMPLES)
    {
        n = CALIB_MIN_SAMPLES;
    }
    if (n > CALIB_MAX_SAMPLES)
    {
        n = CALIB_MAX_SAMPLES;
    }

    /* Measure the raw rate, so temporarily ignore whatever bias is installed. */
    float saved[3] = {imu->gyro_bias[0], imu->gyro_bias[1], imu->gyro_bias[2]};

    imu->gyro_bias[0] = 0.0f;
    imu->gyro_bias[1] = 0.0f;
    imu->gyro_bias[2] = 0.0f;

    float sum[3]   = {0.0f, 0.0f, 0.0f};
    float sumsq[3] = {0.0f, 0.0f, 0.0f};

    uint32_t taken = 0u;

    for (uint32_t k = 0u; k < n; k++)
    {
        if (DEV_BMI088_Read(imu))
        {
            for (uint8_t i = 0u; i < 3u; i++)
            {
                float v = imu->gyro[i];

                sum[i] += v;
                sumsq[i] += v * v;
            }
            taken++;
        }

        PLAT_DWT_Delay_ms(imu->cfg.timebase, 1u);
    }

    /* Demand most of the window: a handful of samples is not an average, and a
     * bus that dropped half the reads is not one to trust a calibration to. */
    if (taken < (n / 2u) || taken == 0u)
    {
        imu->gyro_bias[0] = saved[0];
        imu->gyro_bias[1] = saved[1];
        imu->gyro_bias[2] = saved[2];
        return false;
    }

    float inv = 1.0f / (float) taken;
    float mean[3];

    for (uint8_t i = 0u; i < 3u; i++)
    {
        mean[i] = sum[i] * inv;
    }

    /* Two independent stillness tests, because each catches what the other misses.
     * A large deviation means the sensor was shaken; a large mean with a small
     * deviation means it was turning steadily, which no deviation test can see.
     * Either way the average is contaminated with real rotation, and adopting it
     * would make the gyro report that motion as zero from then on. */
    for (uint8_t i = 0u; i < 3u; i++)
    {
        /* var = E[v^2] - E[v]^2. Clamped at zero before the root because that
         * identity is the cancellation-prone form of the variance: with a mean far
         * larger than the spread, the two terms are nearly equal and rounding can
         * land the difference just below zero, where sqrtf would return NaN and the
         * comparison below would silently pass. */
        const float var = sumsq[i] * inv - mean[i] * mean[i];
        const float std = (var > 0.0f) ? sqrtf(var) : 0.0f;

        if (!UTIL_IsFinitef(mean[i]) || !UTIL_IsFinitef(std) || std > CALIB_STILL_STD ||
            UTIL_Absf(mean[i]) > CALIB_MAX_BIAS)
        {
            imu->gyro_bias[0] = saved[0];
            imu->gyro_bias[1] = saved[1];
            imu->gyro_bias[2] = saved[2];
            return false;
        }
    }

    imu->gyro_bias[0] = mean[0];
    imu->gyro_bias[1] = mean[1];
    imu->gyro_bias[2] = mean[2];
    imu->calibrated   = true;

    return true;
}

void DEV_BMI088_SetGyroBias(DEV_BMI088_s* imu, const float* bias)
{
    if (imu == NULL || bias == NULL)
    {
        return;
    }

    for (uint8_t i = 0u; i < 3u; i++)
    {
        if (UTIL_IsFinitef(bias[i]))
        {
            imu->gyro_bias[i] = bias[i];
        }
    }

    imu->calibrated = true;
}

void DEV_BMI088_GetGyroBias(const DEV_BMI088_s* imu, float* out)
{
    if (imu == NULL || out == NULL)
    {
        return;
    }

    out[0] = imu->gyro_bias[0];
    out[1] = imu->gyro_bias[1];
    out[2] = imu->gyro_bias[2];
}

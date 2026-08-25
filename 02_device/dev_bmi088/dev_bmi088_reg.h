/**
 * @file dev_bmi088_reg.h
 * @author Gao Xing
 * @date 2026/8/5
 * @version 1.0
 *
 * Register addresses and field values for the BMI088, from the Bosch datasheet
 * (BST-BMI088-DS001). Only the registers this driver actually touches are here;
 * the legacy header carried 224 lines covering registers nothing read.
 */

#ifndef DEV_BMI088_REG_H
#define DEV_BMI088_REG_H

/* ========================================================================= */
/*  Accelerometer die                                                        */
/* ========================================================================= */

#define BMI088_ACC_CHIP_ID 0x00u       /**< Who-am-I register.               */
#define BMI088_ACC_CHIP_ID_VALUE 0x1Eu /**< Expected who-am-I value.         */

#define BMI088_ACC_ERR_REG 0x02u /**< Error flags; bit 0 is a fatal error.   */

#define BMI088_ACC_XOUT_L 0x12u   /**< First of six acceleration data bytes.   */
#define BMI088_ACC_TEMP_MSB 0x22u /**< Temperature, high 8 bits.             */

#define BMI088_ACC_CONF 0x40u  /**< Bandwidth and output data rate.          */
#define BMI088_ACC_RANGE 0x41u /**< Full-scale range.                        */

#define BMI088_ACC_PWR_CONF 0x7Cu  /**< Suspend / active mode.                */
#define BMI088_ACC_PWR_CTRL 0x7Du  /**< Accelerometer on / off.               */
#define BMI088_ACC_SOFTRESET 0x7Eu /**< Write the reset value to restart.    */

#define BMI088_ACC_SOFTRESET_VALUE 0xB6u /**< Soft-reset magic value.        */
#define BMI088_ACC_PWR_ACTIVE 0x00u      /**< Active (not suspended).        */
#define BMI088_ACC_ENABLE 0x04u          /**< Accelerometer enabled.         */

/**
 * @brief ACC_CONF: normal bandwidth, 800 Hz ODR, plus the reserved must-set bit.
 *
 * Bit 7 reads back as 1 regardless of what is written, so it has to be included
 * in the expected value or the read-back verification fails on a correctly
 * configured device.
 */
#define BMI088_ACC_CONF_VALUE 0xABu

/* Range register values; the sensitivity that pairs with each is derived in the
 * .c from the same enum, so the two cannot drift apart. */
#define BMI088_ACC_RANGE_3G 0x00u
#define BMI088_ACC_RANGE_6G 0x01u
#define BMI088_ACC_RANGE_12G 0x02u
#define BMI088_ACC_RANGE_24G 0x03u

/* ========================================================================= */
/*  Gyroscope die                                                            */
/* ========================================================================= */

#define BMI088_GYRO_CHIP_ID 0x00u       /**< Who-am-I register.              */
#define BMI088_GYRO_CHIP_ID_VALUE 0x0Fu /**< Expected who-am-I value.        */

#define BMI088_GYRO_RANGE 0x0Fu     /**< Full-scale range.                   */
#define BMI088_GYRO_BANDWIDTH 0x10u /**< Output rate and filter bandwidth.   */
#define BMI088_GYRO_LPM1 0x11u      /**< Power mode.                         */
#define BMI088_GYRO_SOFTRESET 0x14u /**< Write the reset value to restart.   */

#define BMI088_GYRO_SOFTRESET_VALUE 0xB6u /**< Soft-reset magic value.       */
#define BMI088_GYRO_NORMAL_MODE 0x00u     /**< Normal power mode.            */

/**
 * @brief GYRO_BANDWIDTH: 2000 Hz ODR / 230 Hz filter, plus the must-set bit.
 *
 * As with ACC_CONF, bit 7 always reads back set.
 */
#define BMI088_GYRO_BANDWIDTH_VALUE 0x81u

#define BMI088_GYRO_RANGE_2000 0x00u
#define BMI088_GYRO_RANGE_1000 0x01u
#define BMI088_GYRO_RANGE_500 0x02u
#define BMI088_GYRO_RANGE_250 0x03u
#define BMI088_GYRO_RANGE_125 0x04u

/* ========================================================================= */
/*  Protocol                                                                 */
/* ========================================================================= */

/** @brief OR into a register address to make it a read rather than a write. */
#define BMI088_SPI_READ_BIT 0x80u

/** @brief Byte clocked out during a read; the value itself is irrelevant. */
#define BMI088_SPI_DUMMY 0x55u

#endif /* DEV_BMI088_REG_H */

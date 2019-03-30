/*******************************************************************************
 * This file prototypes the LSM9DS1 class, implemented in SFE_LSM9DS1.cpp. In
 * addition, it defines every register in the LSM9DS1 (both the Gyro and Accel/
 * Magnetometer registers).
 *
 * Written by Jim Lindblom @ SparkFun Electronics
 * Re-written in C by Richard Chen <ryc5@sfu.ca>
 *******************************************************************************
 *
 * Copyright Movit Technologies Inc., 2018
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Movit Technologies Inc.
 *
 * ========================================
 */

#ifndef __LSM9DS1_H__
#define __LSM9DS1_H__

#include <stdint.h>
#include <stdbool.h>
#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"
#include "common.h"
#include "i2c.h"

/*******************************************************************************
* Macros
*******************************************************************************/

#define LSM9DS1_AG_ADDR(sa0)    ((sa0) == 0 ? 0x6A : 0x6B)
#define LSM9DS1_M_ADDR(sa1)     ((sa1) == 0 ? 0x1C : 0x1E)

/*******************************************************************************
* Typedefs
*******************************************************************************/

/**
 * @brief Axis enum.
 */
typedef enum lsm9ds1_axis_e {
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    ALL_AXIS
} lsm9ds1_axis_t;

/**
 * @brief Struct containing all device information and raw readings.
 */
typedef struct imu_s {
    IMUSettings_t settings;

    /* Raw, signed 16-bit readings from the sensors */
    int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
    int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
    int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
    int16_t temperature; // Chip temperature

    /* Store initial averaged calibration values */
    int32_t g_bias_raw[3], a_bias_raw[3], m_bias_raw[3];

    /* Calibration values for zeroing and generating model */
    int32_t calib_posture[CALIB_ARRAY_SIZE][3];

    /* Current resolution for each sensor */
    float gRes, aRes, mRes;
} imu_t;

/**
 * @brief Configuration struct.
 */
typedef struct imu_config_s {
    bool enable_accel; // Enable accelerometer
    bool enable_gyro; // Enable gyro
    bool enable_mag; // Enable magnetometer
    uint8_t ag_addr; // I2C address for accelerometer and gyro
    uint8_t mag_addr; // I2C address for magnetometer
    bool low_power_mode; // enable low power mode
} imu_config_t;

/*******************************************************************************
* Public functions
*******************************************************************************/

/**
 * @brief LSM9DS1_init() is the class constructor.
 * @param config Configuration struct.
 * @param dev Struct containing device information.
 * @return imu Struct that will contain raw readings and device info.
 */
uint16_t LSM9DS1_init(imu_t* imu, const imu_config_t* config, dev_t* dev);

/**
 * @brief LSM9DS1_init() is the class constructor.
 * @param imu Struct that will contain raw readings and device info for IMU1
 * @param imu_2 Struct that will contain raw readings and device info for IMU2
 * @param dev Struct containing device information.
 * @return void.
 */
void LSM9DS1_step(imu_t* imu, imu_t* imu_2, dev_t* dev);

/**
 * @brief LSM9DS1_sleep() puts both IMUs to sleep.
 * @param imu Struct containing device info for IMU1.
 * @param imu_2 Struct containing device info for IMU2.
 * @return void.
 */
void LSM9DS1_sleep(imu_t* imu, imu_t* imu_2);

/**
 * @brief LSM9DS1_calibratePosture() samples and averages both accelerometers
 *        and stores into respective variables for posture calibration.
 * @param imu Struct containing device info for IMU1.
 * @param imu_2 Struct containing device info for IMU2.
 * @param posture Defines the posture and specifies where to store data.
 * @param num_samples Specifies the number of samples to be averaged.
 * @return void.
 */
void LSM9DS1_calibratePosture(imu_t* imu, imu_t* imu_2, calib_t posture, uint16_t num_samples);

/**
 * @brief LSM9DS1_xgAverage() collects and averages accelerometer and gyroscope
 *        samples using built-in FIFO.
 * @param imu Struct containing device information.
 * @return void.
 */
void LSM9DS1_xgAverage(imu_t* imu);

/**
 * @brief LSM9DS1_mAverage() collects and averages magnetometer samples.
 * @param imu Struct containing device information.
 * @param loadIn Loads the offset into device if true.
 * @return void.
 */
void LSM9DS1_mAverage(imu_t* imu, bool loadIn);

/**
 * @brief LSM9DS1_magOffset() loads an axis offset into device.
 * @param imu Struct containing device information.
 * @param axis Specifies the axis.
 * @param offset Value to be loaded into the device
 * @return void.
 */
void LSM9DS1_magOffset(imu_t* imu, uint8_t axis, int16_t offset);

/**
 * @brief accelAvailable() polls the accelerometer status register to check
 *        if new data is available.
 * @param imu Struct containing device information.
 * @return 1 if new data available.
 */
uint8_t LSM9DS1_accelAvailable(imu_t* imu);

/**
 * @brief LSM9DS1_gyroAvailable() polls the gyroscope status register to check
 *        if new data is available.
 * @param imu Struct containing device information.
 * @return 1 if new data available.
 */
uint8_t LSM9DS1_gyroAvailable(imu_t* imu);

/**
 * @brief LSM9DS1_tempAvailable() polls the temperature status register to check
 *        if new data is available.
 * @param imu Struct containing device information.
 * @return 1 if new data available.
 */
uint8_t LSM9DS1_tempAvailable(imu_t* imu);

/**
 * @brief LSM9DS1_magAvailable() polls the magnetometer status register to check
 *        if new data is available.
 * @param imu Struct containing device information.
 * @return 1 if new data available.
 */
uint8_t LSM9DS1_magAvailable(imu_t* imu);

/**
 * @brief LSM9DS1_readGyro() reads all six gyroscope output registers
 *        and stored back into imu struct.
 * @param imu Struct containing device information.
 * @return imu Raw data stored back into original struct.
 */
void LSM9DS1_readGyro(imu_t* imu);

/**
 * @brief LSM9DS1_readAccel() reads all six accelerometer output registers
 *        and stored back into imu struct.
 * @param imu Struct containing device information.
 * @return imu Raw data stored back into original struct.
 */
void LSM9DS1_readAccel(imu_t* imu);

/**
 * @brief LSM9DS1_readMag() reads all six magnetometer output registers
 *        and stored back into imu struct.
 * @param imu Struct containing device information.
 * @return imu Raw data stored back into original struct.
 */
void LSM9DS1_readMag(imu_t* imu);

/**
 * @brief LSM9DS1_readTemp() reads both temperature output registers
 *        and stored back into imu struct.
 * @param imu Struct containing device information.
 * @return imu Raw data stored back into original struct.
 */
void LSM9DS1_readTemp(imu_t* imu);

/**
 * @brief LSM9DS1_calcTemp() calculates temperature from raw value.
 * @param imu Struct containing device information.
 * @return imu Raw data stored back into original struct.
 */
float LSM9DS1_calcTemp(const imu_t* imu);

/**
 * @brief LSM9DS1_setGyroScale() sets the full-scale range of the gyroscope.
 *
 * This function can be called to set the scale of the gyroscope to
 * 245, 500, or 2000 degrees per second.
 *
 * @param imu Struct containing device information.
 * @param gScl New full-scale range to be set for the gyroscope.
 * @return void.
 */
void LSM9DS1_setGyroScale(imu_t* imu, uint16_t gScl);

/**
 * @brief LSM9DS1_setAccelScale() sets the full-scale range of the accelerometer.
 *
 * This function can be called to set the scale of the accelerometer to
 * 2, 4, 6, 8, or 16 g's.
 *
 * @param imu Struct containing device information.
 * @param aScl New full-scale range to be set for the accelerometer.
 * @return void.
 */
void LSM9DS1_setAccelScale(imu_t* imu, uint8_t aScl);

/**
 * @brief LSM9DS1_setMagScale() sets the full-scale range of the magnetometer.
 *
 * This function can be called to set the scale of the magnetometer to
 * 4, 8, 12, or 16 Gs.
 *
 * @param imu Struct containing device information.
 * @param aScl New full-scale range to be set for the magnetometer.
 * @return void.
 */
void LSM9DS1_setMagScale(imu_t* imu, uint8_t mScl);

/**
 * @brief LSM9DS1_setGyroODR() sets the output data rate of the gyroscope.
 * @param imu Struct containing device information.
 * @param gRate New output data rate to be set.
 * @return void.
 */
void LSM9DS1_setGyroODR(imu_t* imu, uint8_t gRate);

/**
 * @brief LSM9DS1_setAccelODR() sets the output data rate of the accelerometer.
 * @param imu Struct containing device information.
 * @param aRate New output data rate to be set.
 * @return void.
 */
void LSM9DS1_setAccelODR(imu_t* imu, uint8_t aRate);

/**
 * @brief LSM9DS1_setMagODR() sets the output data rate of the magnetometer.
 * @param imu Struct containing device information.
 * @param mRate New output data rate to be set.
 * @return void.
 */
void LSM9DS1_setMagODR(imu_t* imu, uint8_t mRate);

/**
 * @brief LSM9DS1_configInactivity() configures the inactivity interrupt parameters.
 * @param imu Struct containing device information.
 * @param duration Inactivty duration, actual value depends on gyro ODR.
 * @param threshold Activity threshold.
 * @param sleepOn Gyroscope operating mode, true sets sleep mode, false sets power-down.
 * @return void.
 */
void LSM9DS1_configInactivity(imu_t* imu, uint8_t duration, uint8_t threshold, bool sleepOn);

/**
 * @brief LSM9DS1_configAccelInt() configures accelerometer interrupt generator.
 * @param imu Struct containing device information.
 * @param generator Interrupt axis/high-low events
 *        any OR'd combination of ZHIE_XL, ZLIE_XL, YHIE_XL, YLIE_XL, XHIE_XL, XLIE_XL
 * @param andInterrupts AND/OR combination of interrupt events
 *        true: AND combination
 *        false: OR combination
 * @return void.
 */
void LSM9DS1_configAccelInt(imu_t* imu, uint8_t generator, bool andInterrupts);

/**
 * @brief LSM9DS1_configAccelThs() configures the threshold of an accelerometer axis.
 * @param imu Struct containing device information.
 * @param threshold Interrupt threshold. Possible values: 0-255.
 *        Multiply by 128 to get the actual raw accel value.
 * @param axis Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
 * @param duration Duration value must be above or below threshold to trigger interrupt
 * @param wait Wait function on duration counter
 *        true: Wait for duration samples before exiting interrupt
 *        false: Wait function off
 * @return void.
 */
void LSM9DS1_configAccelThs(imu_t* imu, uint8_t threshold, lsm9ds1_axis_t axis, uint8_t duration, bool wait);

/**
 * @brief LSM9DS1_configGyroInt() configures gyroscope interrupt generator.
 * @param imu Struct containing device information.
 * @param generator Interrupt axis/high-low events
 *        Any OR'd combination of ZHIE_G, ZLIE_G, YHIE_G, YLIE_G, XHIE_G, XLIE_G
 * @param aoi AND/OR combination of interrupt events
 *        true: AND combination
 *        false: OR combination
 * @param latch: latch gyroscope interrupt request.
 * @return void.
 */
void LSM9DS1_configGyroInt(imu_t* imu, uint8_t generator, bool aoi, bool latch);

/**
 * @brief LSM9DS1_configGyroThs() configures the threshold of a gyroscope axis.
 * @param imu Struct containing device information.
 * @param threshold Interrupt threshold. Possible values: 0-0x7FF.
 *        Value is equivalent to raw gyroscope value.
 * @param axis Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
 * @param duration Duration value must be above or below threshold to trigger interrupt
 * @param wait Wait function on duration counter
 *        true: Wait for duration samples before exiting interrupt
 *        false: Wait function off
 * @return void.
 */
void LSM9DS1_configGyroThs(imu_t* imu, int16_t threshold, lsm9ds1_axis_t axis, uint8_t duration, bool wait);

/**
 * @brief LSM9DS1_configInt() configures INT1 or INT2 (gyro and accel interrupts only).
 * @param imu Struct containing device information.
 * @param interrupt Select INT1 or INT2
 *        Possible values: XG_INT1 or XG_INT2
 * @param generator OR'd combination of interrupt generators.
 *        Possible values: INT_DRDY_XL, INT_DRDY_G, INT1_BOOT (INT1 only), INT2_DRDY_TEMP (INT2 only)
 *        INT_FTH, INT_OVR, INT_FSS5, INT_IG_XL (INT1 only), INT1_IG_G (INT1 only), INT2_INACT (INT2 only)
 * @param activeLow Interrupt active configuration
 *        Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
 * @param pushPull Push-pull or open drain interrupt configuration
 *        Can be either INT_PUSH_PULL or INT_OPEN_DRAIN
 * @return void.
 */
void LSM9DS1_configInt(imu_t* imu, interrupt_select_t interupt, uint8_t generator,
               h_lactive_t activeLow /* INT_ACTIVE_LOW */, pp_od_t pushPull /* INT_PUSH_PULL*/);

/**
 * @brief LSM9DS1_configMagInt() configures magnetometer interrupt generator.
 * @param imu Struct containing device information.
 * @param generator Interrupt axis/high-low events
 *        Any OR'd combination of ZIEN, YIEN, XIEN
 * @param activeLow Interrupt active configuration
 *        Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
 * @param latch Latch gyroscope interrupt request.
 * @return void.
 */
void LSM9DS1_configMagInt(imu_t* imu, uint8_t generator, h_lactive_t activeLow, bool latch);

/**
 * @brief LSM9DS1_configMagThs() configures the threshold of a gyroscope axis.
 * @param imu Struct containing device information.
 * @param threshold Interrupt threshold. Possible values: 0-0x7FF.
 *        Value is equivalent to raw magnetometer value.
 * @return void.
 */
void LSM9DS1_configMagThs(imu_t* imu, uint16_t threshold);

/**
 * @brief LSM9DS1_getGyroIntSrc() gets contents of gyroscope interrupt source register.
 * @param imu Struct containing device information.
 * @return interrupt source register.
 */
uint8_t LSM9DS1_getGyroIntSrc(imu_t* imu);

/**
 * @brief LSM9DS1_getGyroIntSrc() gets contents of accelerometer interrupt source register.
 * @param imu Struct containing device information.
 * @return interrupt source register.
 */
uint8_t LSM9DS1_getAccelIntSrc(imu_t* imu);

/**
 * @brief LSM9DS1_getGyroIntSrc() gets contents of magnetometer interrupt source register.
 * @param imu Struct containing device information.
 * @return interrupt source register.
 */
uint8_t LSM9DS1_getMagIntSrc(imu_t* imu);

/**
 * @brief LSM9DS1_getGyroIntSrc() gets status of inactivity interrupt
 * @param imu Struct containing device information.
 * @return status of inactivity interrupt.
 */
uint8_t LSM9DS1_getInactivity(imu_t* imu);

/**
 * @brief LSM9DS1_sleepGyro() sleeps or wakes the gyroscope
 * @param imu Struct containing device information.
 * @param enable True = sleep gyro. False = wake gyro.
 * @return void.
 */
void LSM9DS1_sleepGyro(imu_t* imu, bool enable);

/**
 * @brief LSM9DS1_enableFIFO() enables or disables the FIFO.
 * @param imu Struct containing device information.
 * @param enable true = enable, false = disable.
 * @return void.
 */
void LSM9DS1_enableFIFO(imu_t* imu, bool enable);

/**
 * @brief LSM9DS1_setFIFO() vonfigures FIFO mode and threshold.
 * @param imu Struct containing device information.
 * @param fifoMode Set FIFO mode to off, FIFO (stop when full), continuous, bypass
 *        Possible inputs FIFO_OFF, FIFO_THS, FIFO_CONT_TRIGGER, FIFO_OFF_TRIGGER, FIFO_CONT
 * @param fifoThs FIFO threshold level setting
 *        Any value from 0-0x1F is acceptable.
 * @return void.
 */
void LSM9DS1_setFIFO(imu_t* imu, fifoMode_type_t fifoMode, uint8_t fifoThs);

/**
 * @brief LSM9DS1_getFIFOSamples() gets number of FIFO samples.
 * @param imu Struct containing device information.
 * @return number of FIFO samples.
 */
uint8_t LSM9DS1_getFIFOSamples(imu_t* imu);

/**
 * @brief LSM9DS1_initGyro() sets up the gyroscope to begin reading.
 *
 * This function steps through all five gyroscope control registers.
 * Upon exit, the following parameters will be set:
 *      CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled.
 *                          95 Hz ODR, 12.5 Hz cutoff frequency.
 *      CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
 *                          set to 7.2 Hz (depends on ODR).
 *      CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
 *                          active high). Data-ready output enabled on DRDY_G.
 *      CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
 *                          address. Scale set to 245 DPS. SPI mode set to 4-wire.
 *      CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
 *
 * @param none.
 * @return void.
 */
void LSM9DS1_initGyro(imu_t* imu);

/**
 * @brief LSM9DS1_initAccel() sets up the accelerometer to begin reading.
 *
 * This function steps through all accelerometer related control registers.
 * Upon exit these registers will be set as:
 *      CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
 *      CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
 *                           all axes enabled.
 *      CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
 *      CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
 *
 * @param imu Struct containing device information.
 * @return void.
 */
void LSM9DS1_initAccel(imu_t* imu);

/**
 * @brief LSM9DS1_initMag() sets up the magnetometer to begin reading.
 *
 * This function steps through all magnetometer-related control registers.
 * Upon exit these registers will be set as:
 *      CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
 *      CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
 *                           requests don't latch. Temperature sensor disabled.
 *      CTRL_REG6_XM = 0x00: 2 Gs scale.
 *      CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
 *      INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
 *
 * @param imu Struct containing device information.
 * @return void.
 */
void LSM9DS1_initMag(imu_t* imu);

/**
 * @brief LSM9DS1_gReadByte() reads a byte from a specified gyroscope register.
 * @param imu Struct containing device information.
 * @param subAddress Register to be read from.
 * @return An 8-bit value read from the requested address.
 */
uint8_t LSM9DS1_mReadByte(imu_t* imu, uint8_t subAddress);

/**
 * @brief LSM9DS1_gReadBytes() reads a number of bytes beginning at an address.
 *        and incrementing from there from the gyroscope.
 * @param imu Struct containing device information.
 * @param subAddress Register to be read from.
 * @param count The number of bytes to be read.
 * @return *dest A pointer to an array of uint8_t's. Values read will be
 *         stored in here on return.
 */
void LSM9DS1_mReadBytes(imu_t* imu, uint8_t subAddress, uint8_t * dest, uint8_t count);

/**
 * @brief LSM9DS1_gWriteByte() writes a byte to a register in the gyroscope.
 * @param imu Struct containing device information.
 * @param subAddress Register to be written to.
 * @param data Data to be written to the register.
 * @return void.
 */
void LSM9DS1_mWriteByte(imu_t* imu, uint8_t subAddress, uint8_t data);

/**
 * @brief LSM9DS1_xmReadByte() reads a byte from a register in the accel/mag sensor.
 * @param imu Struct containing device information.
 * @param subAddress Register to be read from.
 * @return An 8-bit value read from the requested register.
 */
uint8_t LSM9DS1_xgReadByte(imu_t* imu, uint8_t subAddress);

/**
 * @brief LSM9DS1_xmReadBytes() reads a number of bytes beginning at an address.
 *        and incrementing from there from the accelerometer/magnetometer.
 * @param imu Struct containing device information.
 * @param subAddress Register to be read from.
 * @param count The number of bytes to be read.
 * @return *dest A pointer to an array of uint8_t's. Values read will be
 *         stored in here on return.
 */
void LSM9DS1_xgReadBytes(imu_t* imu, uint8_t subAddress, uint8_t * dest, uint8_t count);

/**
 * @brief LSM9DS1_xmWriteByte() writes a byte to a register in the accel/mag sensor.
 * @param imu Struct containing device information.
 * @param subAddress Register to be written to.
 * @param data Data to be written to the register.
 * @return void.
 */
void LSM9DS1_xgWriteByte(imu_t* imu, uint8_t subAddress, uint8_t data);

/**
 * @brief LSM9DS1_calcgRes() calculates the resolution of the gyroscope.
 *
 * This function will set the value of the gRes variable. gScale must
 * be set prior to calling this function.
 *
 * @param imu Struct containing device information.
 * @return void.
 */
void LSM9DS1_calcgRes(imu_t* imu);

/**
 * @brief LSM9DS1_calcmRes() calculates the resolution of the magnetometer.
 *
 * This function will set the value of the mRes variable. mScale must
 * be set prior to calling this function.
 *
 * @param imu Struct containing device information.
 * @return void.
 */
void LSM9DS1_calcmRes(imu_t* imu);

/**
 * @brief LSM9DS1_calcaRes() calculates the resolution of the accelerometer.
 *
 * This function will set the value of the aRes variable. aScale must
 * be set prior to calling this function.
 *
 * @param imu Struct containing device information.
 * @return void.
 */
void LSM9DS1_calcaRes(imu_t* imu);

/**
 * @brief LSM9DS1_constrainScales() limits to allowable full-range settings.
 * @param imu Struct containing device information.
 * @return void.
 */
void LSM9DS1_constrainScales(imu_t* imu);

/**
 * @brief LSM9DS1_I2CwriteByte() writes a byte out of I2C to a register in the device.
 * @param address The 7-bit I2C address of the slave device.
 * @param subAddress The register to be written to.
 * @param data Byte to be written to the register.
 * @return void.
 */
void LSM9DS1_I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);

/**
 * @brief LSM9DS1_I2CreadByte() reads a single byte from a register over I2C.
 * @param address The 7-bit I2C address of the slave device.
 * @param subAddress The register to be read from.
 * @return The byte read from the requested address.
 */
uint8_t LSM9DS1_I2CreadByte(uint8_t address, uint8_t subAddress);

/**
 * @brief LSM9DS1_I2CreadBytes() reads a series of bytes, starting at a register via I2C.
 * @param address The 7-bit I2C address of the slave device.
 * @param subAddress The register to begin reading.
 * @param count Number of registers to be read.
 * @return *dest Pointer to an array where we'll store the readings.
 */
void LSM9DS1_I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);

#endif /* __LSM9DS1_H__ */

/*******************************************************************************
 * This file prototypes the LSM9DS1 class, implemented in SFE_LSM9DS1.cpp. In
 * addition, it defines every register in the LSM9DS1 (both the Gyro and Accel/
 * Magnetometer registers).
 *
 * Written by Jim Lindblom @ SparkFun Electronics
 * Re-written in C by Richard Chen <ryc5@sfu.ca>
 * Re-written for Tiva-C by Rick Lin <rick_lin@sfu.ca>
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

#include "LSM9DS1.h"

/*******************************************************************************
* Constants
*******************************************************************************/

const float magSensitivity[4] = {0.00014, 0.00029, 0.00043, 0.00058};
const uint8_t NUM_RETRIES = 200; // Arbitrarily set for now
const float PI = 3.14159265358979323846;

/*******************************************************************************
* Global Variables
*******************************************************************************/

imu_error_t imu_error_flag;

/*******************************************************************************
* Public functions
*******************************************************************************/

uint16_t LSM9DS1_init(imu_t* imu, const imu_config_t* config, dev_t* dev)
{
    /* Store slave address */
    imu->settings.device.agAddress = config->ag_addr;
    imu->settings.device.mAddress = config->mag_addr;

    /* Gyro enable axes */
    imu->settings.gyro.enabled = config->enable_gyro;
    imu->settings.gyro.enableX = config->enable_gyro;
    imu->settings.gyro.enableY = config->enable_gyro;
    imu->settings.gyro.enableZ = config->enable_gyro;

    /* Gyro scale can be 245, 500, or 2000 */
    imu->settings.gyro.scale = 245;

    /**
     * Gyro sample rate: value between 1-6
     * 1 = 14.9    4 = 238
     * 2 = 59.5    5 = 476
     * 3 = 119     6 = 952
     */
    imu->settings.gyro.sampleRate = 6;

    /**
     * Gyro cutoff frequency: value between 0-3
     * Actual value of cutoff frequency depends on sample rate.
     */
    imu->settings.gyro.bandwidth = 0;
    imu->settings.gyro.lowPowerEnable = config->low_power_mode;
    imu->settings.gyro.HPFEnable = false;

    /**
     * Gyro HPF cutoff frequency: value between 0-9
     * Actual value depends on sample rate. Only applies
     * if gyroHPFEnable is true.
     */
    imu->settings.gyro.HPFCutoff = 0;
    imu->settings.gyro.flipX = false;
    imu->settings.gyro.flipY = false;
    imu->settings.gyro.flipZ = false;
    imu->settings.gyro.latchInterrupt = true;

    /* Accel enable axes */
    imu->settings.accel.enabled = config->enable_accel;
    imu->settings.accel.enableX = config->enable_accel;
    imu->settings.accel.enableY = config->enable_accel;
    imu->settings.accel.enableZ = config->enable_accel;

    /* Accel scale can be 2, 4, 8, or 16 */
    imu->settings.accel.scale = 2;

    /**
     * Accel sample rate can be 1-6
     * 1 = 10 Hz    4 = 238 Hz
     * 2 = 50 Hz    5 = 476 Hz
     * 3 = 119 Hz   6 = 952 Hz
     */
    imu->settings.accel.sampleRate = 6;

    /**
     * Accel cutoff freqeuncy can be any value between -1 - 3.
     * -1 = bandwidth determined by sample rate
     *  0 = 408 Hz   2 = 105 Hz
     *  1 = 211 Hz   3 = 50 Hz
     */
    imu->settings.accel.bandwidth = -1;
    imu->settings.accel.highResEnable = false;

    /**
     * Accel high res bandwidth can be any value between 0-3
     * LP cutoff is set to a factor of sample rate
     * 0 = ODR/50    2 = ODR/9
     * 1 = ODR/100   3 = ODR/400
     */
    imu->settings.accel.highResBandwidth = 0;
    imu->settings.mag.enabled = config->enable_mag;

    /* Mag scale can be 4, 8, 12, or 16 */
    imu->settings.mag.scale = 4;

    /**
     * Mag data rate can be 0-7
     * 0 = 0.625 Hz  4 = 10 Hz
     * 1 = 1.25 Hz   5 = 20 Hz
     * 2 = 2.5 Hz    6 = 40 Hz
     * 3 = 5 Hz      7 = 80 Hz
     */
    imu->settings.mag.sampleRate = 7;
    imu->settings.mag.tempCompensationEnable = false;

    /**
     * Mag performance can be any value between 0-3
     * 0 = Low power mode      2 = high performance
     * 1 = medium performance  3 = ultra-high performance
     */
    imu->settings.mag.XYPerformance = config->low_power_mode ? 0 : 3;
    imu->settings.mag.ZPerformance = config->low_power_mode ? 0 : 3;
    imu->settings.mag.lowPowerEnable = config->low_power_mode;

    /**
     * Mag operating mode can be 0-2
     * 0 = continuous conversion
     * 1 = single-conversion
     * 2 = power down
     */
    imu->settings.mag.operatingMode = 0;
    imu->settings.temp.enabled = true;

    /* Limit full-range scale to allowable values */
    LSM9DS1_constrainScales(imu);

    /**
     * Once we have the scale values, we can calculate the resolution
     * of each sensor. That's what these functions are for. One for each sensor
     */
    LSM9DS1_calcgRes(imu); // Calculate DPS / ADC tick, stored in gRes variable
    LSM9DS1_calcmRes(imu); // Calculate Gs / ADC tick, stored in mRes variable
    LSM9DS1_calcaRes(imu); // Calculate g / ADC tick, stored in aRes variable

    /**
     * To verify communication, we can read from the WHO_AM_I register of
     * each device. Store those in a variable so we can return them.
     */
    uint8_t mTest = LSM9DS1_mReadByte(imu, WHO_AM_I_M);     // Read the gyro WHO_AM_I
    uint8_t xgTest = LSM9DS1_xgReadByte(imu, WHO_AM_I_XG);  // Read the accel/mag WHO_AM_I
    uint16_t whoAmICombined = (xgTest << 8) | mTest;

    /* Handle error flags to identify source of failure */
    if (mTest != WHO_AM_I_M_RSP && xgTest != WHO_AM_I_AG_RSP) {
        imu_error_flag = IMU_MXG_INIT_ERR;
    }
    else if (mTest != WHO_AM_I_M_RSP) {
        imu_error_flag = IMU_M_INIT_ERR;
    }
    else if (xgTest != WHO_AM_I_AG_RSP) {
        imu_error_flag = IMU_XG_INIT_ERR;
    }

    /* Determine which IMU is being initialized */
    if (imu_error_flag) {
        if (imu->settings.device.agAddress == LSM9DS1_AG_ADDR(0)) {
            dev->status.imu1_error = imu_error_flag;
        }
        else {
            dev->status.imu2_error = imu_error_flag;
        }
        return 0;
    }

    /* Gyro initialization */
    LSM9DS1_initGyro(imu);  // This will "turn on" the gyro. Setting up interrupts, etc.

    /* Accelerometer initialization */
    LSM9DS1_initAccel(imu); // "Turn on" all axes of the accel. Set up interrupts, etc.

    /* Magnetometer initialization */
    LSM9DS1_initMag(imu); // "Turn on" all axes of the mag. Set up interrupts, etc.

    /* Set error flags if any arise from initialization */
    if (imu->settings.device.agAddress == LSM9DS1_AG_ADDR(0)) {
        dev->status.imu1_error = imu_error_flag;
    }
    else {
        dev->status.imu2_error = imu_error_flag;
    }

    /* Once everything is initialized, return the WHO_AM_I registers we read */
    return whoAmICombined;
}

void LSM9DS1_step(imu_t* imu, dev_t* dev)
{
    /* Guard against any IMU error */
    if ((dev->status.imu1_error == IMU_NO_ERR || dev->status.imu1_error == IMU_M_INIT_ERR)
        && (dev->status.imu2_error == IMU_NO_ERR || dev->status.imu2_error == IMU_M_INIT_ERR)) {

        /* Read sensor data */
        LSM9DS1_readAccel(imu);
        LSM9DS1_calcAttitude(imu, dev);
        dev->status.imu1_error = imu_error_flag;
    }
}

void LSM9DS1_sleep(imu_t* imu, imu_t* imu_2)
{
    /* Set gyro to power down mode, default 0x00 */
    LSM9DS1_xgWriteByte(imu, CTRL_REG1_G, 0x00);
    LSM9DS1_xgWriteByte(imu_2, CTRL_REG1_G, 0x00);

    /* Set accel to power down mode, default 0x00 */
    LSM9DS1_xgWriteByte(imu, CTRL_REG6_XL, 0x00);
    LSM9DS1_xgWriteByte(imu_2, CTRL_REG6_XL, 0x00);

    /* Set mag to power down mode, default 0x03 */
    LSM9DS1_mWriteByte(imu, CTRL_REG3_M, 0x03);
    LSM9DS1_mWriteByte(imu_2, CTRL_REG3_M, 0x03);
}

void LSM9DS1_calcAttitude(imu_t* imu, dev_t* dev)
{
    dev->data.roll = atan2(imu->ay, imu->az);
    dev->data.pitch = atan2(-imu->ax, sqrt(imu->ay * imu->ay + imu->az * imu->az));

    // Convert everything from radians to degrees:
    dev->data.pitch *= 180.0 / PI;
    dev->data.roll  *= 180.0 / PI;
}

void LSM9DS1_initGyro(imu_t* imu)
{
    uint8_t tempRegValue = 0;

    /**
     * CTRL_REG1_G (Default value: 0x00)
     * [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
     * ODR_G[2:0] - Output data rate selection
     * FS_G[1:0] - Gyroscope full-scale selection
     * BW_G[1:0] - Gyroscope bandwidth selection
     */

    /**
     * To disable gyro, set sample rate bits to 0. We'll only set sample
     * rate if the gyro is enabled since it is 0 if not set.
     */
    if (imu->settings.gyro.enabled)
    {
        tempRegValue = (imu->settings.gyro.sampleRate & 0x07) << 5;
    }
    switch (imu->settings.gyro.scale)
    {
        case 500:
            tempRegValue |= (0x1 << 3);
            break;
        case 2000:
            tempRegValue |= (0x3 << 3);
            break;
        // Otherwise we'll set it to 245 dps (0x0 << 4)
    }
    tempRegValue |= (imu->settings.gyro.bandwidth & 0x3);
    LSM9DS1_xgWriteByte(imu, CTRL_REG1_G, tempRegValue);

    /**
     * CTRL_REG2_G (Default value: 0x00)
     * [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
     * INT_SEL[1:0] - INT selection configuration
     * OUT_SEL[1:0] - Out selection configuration
     */
    LSM9DS1_xgWriteByte(imu, CTRL_REG2_G, 0x00);

    /**
     * CTRL_REG3_G (Default value: 0x00)
     * [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
     * LP_mode - Low-power mode enable (0: disabled, 1: enabled)
     * HP_EN - HPF enable (0:disabled, 1: enabled)
     * HPCF_G[3:0] - HPF cutoff frequency
     */
    tempRegValue = imu->settings.gyro.lowPowerEnable ? (1<<7) : 0;
    if (imu->settings.gyro.HPFEnable)
    {
        tempRegValue |= (1<<6) | (imu->settings.gyro.HPFCutoff & 0x0F);
    }
    LSM9DS1_xgWriteByte(imu, CTRL_REG3_G, tempRegValue);

    /**
     * CTRL_REG4 (Default value: 0x38)
     * [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
     * Zen_G - Z-axis output enable (0:disable, 1:enable)
     * Yen_G - Y-axis output enable (0:disable, 1:enable)
     * Xen_G - X-axis output enable (0:disable, 1:enable)
     * LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
     * 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
     */
    tempRegValue = 0;
    if (imu->settings.gyro.enableZ) tempRegValue |= (1<<5);
    if (imu->settings.gyro.enableY) tempRegValue |= (1<<4);
    if (imu->settings.gyro.enableX) tempRegValue |= (1<<3);
    if (imu->settings.gyro.latchInterrupt) tempRegValue |= (1<<1);
    LSM9DS1_xgWriteByte(imu, CTRL_REG4, tempRegValue);

    /**
     * ORIENT_CFG_G (Default value: 0x00)
     * [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
     * SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
     * Orient [2:0] - Directional user orientation selection
     */
    tempRegValue = 0;
    if (imu->settings.gyro.flipX) tempRegValue |= (1<<5);
    if (imu->settings.gyro.flipY) tempRegValue |= (1<<4);
    if (imu->settings.gyro.flipZ) tempRegValue |= (1<<3);
    LSM9DS1_xgWriteByte(imu, ORIENT_CFG_G, tempRegValue);
}

void LSM9DS1_initAccel(imu_t* imu)
{
    uint8_t tempRegValue = 0;

    /**
     * CTRL_REG5_XL (0x1F) (Default value: 0x38)
     * [DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
     * DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
     *          00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
     * Zen_XL - Z-axis output enabled
     * Yen_XL - Y-axis output enabled
     * Xen_XL - X-axis output enabled
     */
    if (imu->settings.accel.enableZ) tempRegValue |= (1<<5);
    if (imu->settings.accel.enableY) tempRegValue |= (1<<4);
    if (imu->settings.accel.enableX) tempRegValue |= (1<<3);

    LSM9DS1_xgWriteByte(imu, CTRL_REG5_XL, tempRegValue);

    /**
     * CTRL_REG6_XL (0x20) (Default value: 0x00)
     * [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
     * ODR_XL[2:0] - Output data rate & power mode selection
     * FS_XL[1:0] - Full-scale selection
     * BW_SCAL_ODR - Bandwidth selection
     * BW_XL[1:0] - Anti-aliasing filter bandwidth selection
     */
    tempRegValue = 0;

    /* To disable the accel, set the sampleRate bits to 0 */
    if (imu->settings.accel.enabled)
    {
        tempRegValue |= (imu->settings.accel.sampleRate & 0x07) << 5;
    }
    switch (imu->settings.accel.scale)
    {
        case 4:
            tempRegValue |= (0x2 << 3);
            break;
        case 8:
            tempRegValue |= (0x3 << 3);
            break;
        case 16:
            tempRegValue |= (0x1 << 3);
            break;
        // Otherwise it'll be set to 2g (0x0 << 3)
    }
    if (imu->settings.accel.bandwidth >= 0)
    {
        tempRegValue |= (1<<2); // Set BW_SCAL_ODR
        tempRegValue |= (imu->settings.accel.bandwidth & 0x03);
    }
    LSM9DS1_xgWriteByte(imu, CTRL_REG6_XL, tempRegValue);

    /**
     * CTRL_REG7_XL (0x21) (Default value: 0x00)
     * [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
     * HR - High resolution mode (0: disable, 1: enable)
     * DCF[1:0] - Digital filter cutoff frequency
     * FDS - Filtered data selection
     * HPIS1 - HPF enabled for interrupt function
     */
    tempRegValue = 0;
    if (imu->settings.accel.highResEnable)
    {
        tempRegValue |= (1<<7); // Set HR bit
        tempRegValue |= (imu->settings.accel.highResBandwidth & 0x3) << 5;
    }
    LSM9DS1_xgWriteByte(imu, CTRL_REG7_XL, tempRegValue);
}

void LSM9DS1_calibratePosture(imu_t* imu, imu_t* imu_2, calib_t posture, uint16_t num_samples)
{
    float g1[3] = {0.0};
    float g2[3] = {0.0};
    int i = 0;

    for (i = 0; i < num_samples; i++) {
        LSM9DS1_readAccel(imu);
        LSM9DS1_readAccel(imu_2);

        g1[0] += imu->ax;
        g1[1] += imu->ay;
        g1[2] += imu->az;
        g2[0] += imu_2->ax;
        g2[1] += imu_2->ay;
        g2[2] += imu_2->az;
    }
    g1[0] *= 1.0 / num_samples;
    g1[1] *= 1.0 / num_samples;
    g1[2] *= 1.0 / num_samples;
    g2[0] *= 1.0 / num_samples;
    g2[1] *= 1.0 / num_samples;
    g2[2] *= 1.0 / num_samples;

    for (i = 0; i < 3; i++) {
        imu->calib_posture[posture][i] = g1[i];
        imu_2->calib_posture[posture][i] = g2[i];
    }
}

void LSM9DS1_xgAverage(imu_t* imu)
{
    int ii;
    uint8_t samples = 0;
    int32_t a_bias_raw_temp[3] = {0, 0, 0};
    int32_t g_bias_raw_temp[3] = {0, 0, 0};

    /* Turn on FIFO and set threshold to 32 samples */
    LSM9DS1_enableFIFO(imu, true);
    LSM9DS1_setFIFO(imu, FIFO_THS, 0x1F);
    while (samples < 0x1F)
    {
        samples = (LSM9DS1_xgReadByte(imu, FIFO_SRC) & 0x3F); // Read number of stored samples
    }

    /* Read and accumulate FIFO */
    for(ii = 0; ii < samples ; ii++)
    {
        LSM9DS1_readGyro(imu);
        g_bias_raw_temp[0] += imu->gx;
        g_bias_raw_temp[1] += imu->gy;
        g_bias_raw_temp[2] += imu->gz;
        LSM9DS1_readAccel(imu);
        a_bias_raw_temp[0] += imu->ax;
        a_bias_raw_temp[1] += imu->ay;
        a_bias_raw_temp[2] += imu->az;
    }

    /* Average and store to imu struct */
    for(ii = 0; ii < 3; ii++)
    {
        imu->g_bias_raw[ii] = g_bias_raw_temp[ii] / samples;
        imu->a_bias_raw[ii] = a_bias_raw_temp[ii] / samples;
    }

    LSM9DS1_enableFIFO(imu, false);
    LSM9DS1_setFIFO(imu, FIFO_OFF, 0x00);
}

void LSM9DS1_mAverage(imu_t* imu, bool loadIn)
{
    int i, j;
    int16_t magMin[3] = {0, 0, 0};
    int16_t magMax[3] = {0, 0, 0}; // The road warrior

    for (i=0; i<128; i++)
    {
        while (!LSM9DS1_magAvailable(imu))
            ;
        LSM9DS1_readMag(imu);
        int16_t magTemp[3] = {0, 0, 0};
        magTemp[0] = imu->mx;
        magTemp[1] = imu->my;
        magTemp[2] = imu->mz;
        for (j = 0; j < 3; j++)
        {
            if (magTemp[j] > magMax[j]) magMax[j] = magTemp[j];
            if (magTemp[j] < magMin[j]) magMin[j] = magTemp[j];
        }
    }
    for (j = 0; j < 3; j++)
    {
        imu->m_bias_raw[j] = (magMax[j] + magMin[j]) / 2;
        if (loadIn)
            LSM9DS1_magOffset(imu, j, imu->m_bias_raw[j]);
    }
}

void LSM9DS1_magOffset(imu_t* imu, uint8_t axis, int16_t offset)
{
    if (axis > 2)
        return;
    uint8_t msb, lsb;
    msb = (offset & 0xFF00) >> 8;
    lsb = offset & 0x00FF;
    LSM9DS1_mWriteByte(imu, OFFSET_X_REG_L_M + (2 * axis), lsb);
    LSM9DS1_mWriteByte(imu, OFFSET_X_REG_H_M + (2 * axis), msb);
}

void LSM9DS1_initMag(imu_t* imu)
{
    uint8_t tempRegValue = 0;

    /**
     * CTRL_REG1_M (Default value: 0x10)
     * [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
     * TEMP_COMP - Temperature compensation
     * OM[1:0] - X & Y axes op mode selection
     *          00:low-power, 01:medium performance
     *          10: high performance, 11:ultra-high performance
     * DO[2:0] - Output data rate selection
     * ST - Self-test enable
     */
    if (imu->settings.mag.tempCompensationEnable) tempRegValue |= (1<<7);
    tempRegValue |= (imu->settings.mag.XYPerformance & 0x3) << 5;
    tempRegValue |= (imu->settings.mag.sampleRate & 0x7) << 2;
    LSM9DS1_mWriteByte(imu, CTRL_REG1_M, tempRegValue);

    /**
     * CTRL_REG2_M (Default value 0x00)
     * [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
     * FS[1:0] - Full-scale configuration
     * REBOOT - Reboot memory content (0:normal, 1:reboot)
     * SOFT_RST - Reset config and user registers (0:default, 1:reset)
     */
    tempRegValue = 0;
    switch (imu->settings.mag.scale)
    {
    case 8:
        tempRegValue |= (0x1 << 5);
        break;
    case 12:
        tempRegValue |= (0x2 << 5);
        break;
    case 16:
        tempRegValue |= (0x3 << 5);
        break;
    // Otherwise we'll default to 4 gauss (00)
    }
    LSM9DS1_mWriteByte(imu, CTRL_REG2_M, tempRegValue); // +/-4Gauss

    /**
     * CTRL_REG3_M (Default value: 0x03)
     * [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
     * I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
     * LP - Low-power mode cofiguration (1:enable)
     * SIM - SPI mode selection (0:write-only, 1:read/write enable)
     * MD[1:0] - Operating mode
     *          00:continuous conversion, 01:single-conversion,
     *          10,11: Power-down
     */
    tempRegValue = 0;
    if (imu->settings.mag.lowPowerEnable) tempRegValue |= (1<<5);
    tempRegValue |= (imu->settings.mag.operatingMode & 0x3);
    LSM9DS1_mWriteByte(imu, CTRL_REG3_M, tempRegValue); // Continuous conversion mode

    /**
     * CTRL_REG4_M (Default value: 0x00)
     * [0][0][0][0][OMZ1][OMZ0][BLE][0]
     * OMZ[1:0] - Z-axis operative mode selection
     *          00:low-power mode, 01:medium performance
     *          10:high performance, 10:ultra-high performance
     * BLE - Big/little endian data
     */
    tempRegValue = 0;
    tempRegValue = (imu->settings.mag.ZPerformance & 0x3) << 2;
    LSM9DS1_mWriteByte(imu, CTRL_REG4_M, tempRegValue);

    /**
     * CTRL_REG5_M (Default value: 0x00)
     * [0][BDU][0][0][0][0][0][0]
     * BDU - Block data update for magnetic data
     *      0:continuous, 1:not updated until MSB/LSB are read
     */
    tempRegValue = 0;
    LSM9DS1_mWriteByte(imu, CTRL_REG5_M, tempRegValue);
}

uint8_t LSM9DS1_accelAvailable(imu_t* imu)
{
    uint8_t status = LSM9DS1_xgReadByte(imu, STATUS_REG_1);

    return (status & (1<<0));
}

uint8_t LSM9DS1_gyroAvailable(imu_t* imu)
{
    uint8_t status = LSM9DS1_xgReadByte(imu, STATUS_REG_1);

    return ((status & (1<<1)) >> 1);
}

uint8_t LSM9DS1_tempAvailable(imu_t* imu)
{
    uint8_t status = LSM9DS1_xgReadByte(imu, STATUS_REG_1);

    return ((status & (1<<2)) >> 2);
}

uint8_t LSM9DS1_magAvailable(imu_t* imu)
{
    lsm9ds1_axis_t axis = ALL_AXIS;
    uint8_t status;
    status = LSM9DS1_mReadByte(imu, STATUS_REG_M);

    return ((status & (1<<axis)) >> axis);
}

void LSM9DS1_readAccel(imu_t* imu)
{
    uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp
    LSM9DS1_xgReadBytes(imu, OUT_X_L_XL, temp, 6); // Read 6 bytes, beginning at OUT_X_L_XL
    imu->ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
    imu->ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
    imu->az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
}

void LSM9DS1_readMag(imu_t* imu)
{
    uint8_t temp[6]; // We'll read six bytes from the mag into temp
    LSM9DS1_mReadBytes(imu, OUT_X_L_M, temp, 6); // Read 6 bytes, beginning at OUT_X_L_M
    imu->mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
    imu->my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
    imu->mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
}

void LSM9DS1_readTemp(imu_t* imu)
{
    uint8_t low = LSM9DS1_xgReadByte(imu, OUT_TEMP_L);
    uint8_t high = LSM9DS1_xgReadByte(imu, OUT_TEMP_H);

    imu->temperature = (int16_t)(high << 8) | low;
}

void LSM9DS1_readGyro(imu_t* imu)
{
    uint8_t temp[6]; // We'll read six bytes from the gyro into temp
    LSM9DS1_xgReadBytes(imu, OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G

    imu->gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
    imu->gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
    imu->gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
}

#define TEMP_SCALE 16.0
#define TEMP_BIAS 27.5

float LSM9DS1_calcTemp(const imu_t* imu) {
    return (imu->temperature / TEMP_SCALE) + TEMP_BIAS;
}

void LSM9DS1_setGyroScale(imu_t* imu, uint16_t gScl)
{
    /* Read current value of CTRL_REG1_G */
    uint8_t ctrl1RegValue = LSM9DS1_xgReadByte(imu, CTRL_REG1_G);

    /* Mask out scale bits (3 & 4) */
    ctrl1RegValue &= 0xE7;
    switch (gScl)
    {
        case 500:
            ctrl1RegValue |= (0x1 << 3);
            imu->settings.gyro.scale = 500;
            break;
        case 2000:
            ctrl1RegValue |= (0x3 << 3);
            imu->settings.gyro.scale = 2000;
            break;
        default: // Otherwise we'll set it to 245 dps (0x0 << 4)
            imu->settings.gyro.scale = 245;
            break;
    }
    LSM9DS1_xgWriteByte(imu, CTRL_REG1_G, ctrl1RegValue);

    /* Then calculate a new gRes, which relies on gScale being set correctly */
    LSM9DS1_calcgRes(imu);
}

void LSM9DS1_setAccelScale(imu_t* imu, uint8_t aScl)
{
    /* We need to preserve the other bytes in CTRL_REG6_XL. So, first read it */
    uint8_t tempRegValue = LSM9DS1_xgReadByte(imu, CTRL_REG6_XL);

    /* Mask out accel scale bits */
    tempRegValue &= 0xE7;

    switch (aScl)
    {
        case 4:
            tempRegValue |= (0x2 << 3);
            imu->settings.accel.scale = 4;
            break;
        case 8:
            tempRegValue |= (0x3 << 3);
            imu->settings.accel.scale = 8;
            break;
        case 16:
            tempRegValue |= (0x1 << 3);
            imu->settings.accel.scale = 16;
            break;
        default: // Otherwise it'll be set to 2g (0x0 << 3)
            imu->settings.accel.scale = 2;
            break;
    }
    LSM9DS1_xgWriteByte(imu, CTRL_REG6_XL, tempRegValue);

    /* Then calculate a new aRes, which relies on aScale being set correctly */
    LSM9DS1_calcaRes(imu);
}

void LSM9DS1_setMagScale(imu_t* imu, uint8_t mScl)
{
    /* We need to preserve the other bytes in CTRL_REG6_XM. So, first read it */
    uint8_t temp = LSM9DS1_mReadByte(imu, CTRL_REG2_M);

    /* Then mask out the mag scale bits */
    temp &= 0xFF^(0x3 << 5);

    switch (mScl)
    {
        case 8:
            temp |= (0x1 << 5);
            imu->settings.mag.scale = 8;
            break;
        case 12:
            temp |= (0x2 << 5);
            imu->settings.mag.scale = 12;
            break;
        case 16:
            temp |= (0x3 << 5);
            imu->settings.mag.scale = 16;
            break;
        default: // Otherwise we'll default to 4 gauss (00)
            imu->settings.mag.scale = 4;
            break;
    }

    /* And write the new register value back into CTRL_REG6_XM */
    LSM9DS1_mWriteByte(imu, CTRL_REG2_M, temp);

    /* Then calculate a new mRes, which relies on mScale being set correctly */
    LSM9DS1_calcmRes(imu);
}

void LSM9DS1_setGyroODR(imu_t* imu, uint8_t gRate)
{
    /* Only do this if gRate is not 0 (which would disable the gyro) */
    if ((gRate & 0x07) != 0)
    {
        /* We need to preserve the other bytes in CTRL_REG1_G. So, first read it */
        uint8_t temp = LSM9DS1_xgReadByte(imu, CTRL_REG1_G);

        /* Then mask out the gyro ODR bits */
        temp &= 0xFF^(0x7 << 5);
        temp |= (gRate & 0x07) << 5;

        /* Update our settings struct */
        imu->settings.gyro.sampleRate = gRate & 0x07;

        /* And write the new register value back into CTRL_REG1_G */
        LSM9DS1_xgWriteByte(imu, CTRL_REG1_G, temp);
    }
}

void LSM9DS1_setAccelODR(imu_t* imu, uint8_t aRate)
{
    /* Only do this if aRate is not 0 (which would disable the accel) */
    if ((aRate & 0x07) != 0)
    {
        /* We need to preserve the other bytes in CTRL_REG1_XM. So, first read it */
        uint8_t temp = LSM9DS1_xgReadByte(imu, CTRL_REG6_XL);

        /* Then mask out the accel ODR bits */
        temp &= 0x1F;

        /* Then shift in our new ODR bits */
        temp |= ((aRate & 0x07) << 5);
        imu->settings.accel.sampleRate = aRate & 0x07;

        /* And write the new register value back into CTRL_REG1_XM */
        LSM9DS1_xgWriteByte(imu, CTRL_REG6_XL, temp);
    }
}

void LSM9DS1_setMagODR(imu_t* imu, uint8_t mRate)
{
    /* We need to preserve the other bytes in CTRL_REG5_XM. So, first read it */
    uint8_t temp = LSM9DS1_mReadByte(imu, CTRL_REG1_M);

    /* Then mask out the mag ODR bits */
    temp &= 0xFF^(0x7 << 2);

    /* Then shift in our new ODR bits */
    temp |= ((mRate & 0x07) << 2);
    imu->settings.mag.sampleRate = mRate & 0x07;

    /* And write the new register value back into CTRL_REG5_XM */
    LSM9DS1_mWriteByte(imu, CTRL_REG1_M, temp);
}

void LSM9DS1_calcgRes(imu_t* imu)
{
    imu->gRes = ((float) imu->settings.gyro.scale) / 32768.0;
}

void LSM9DS1_calcaRes(imu_t* imu)
{
    imu->aRes = ((float) imu->settings.accel.scale) / 32768.0;
}

void LSM9DS1_calcmRes(imu_t* imu)
{
    // mRes = ((float) settings.mag.scale) / 32768.0;
    switch (imu->settings.mag.scale)
    {
    case 4:
        imu->mRes = magSensitivity[0];
        break;
    case 8:
        imu->mRes = magSensitivity[1];
        break;
    case 12:
        imu->mRes = magSensitivity[2];
        break;
    case 16:
        imu->mRes = magSensitivity[3];
        break;
    }
}

//void LSM9DS1_configInt(imu_t* imu, interrupt_select_t interrupt, uint8_t generator,
//                       h_lactive_t activeLow, pp_od_t pushPull)
//{
//    /**
//     * Write to INT1_CTRL or INT2_CTRL. [interupt] should already be one of
//     * those two values.
//     * [generator] should be an OR'd list of values from the interrupt_generators enum
//     */
//    LSM9DS1_xgWriteByte(imu, interrupt, generator);
//
//    /* Configure CTRL_REG8 */
//    uint8_t temp;
//    temp = LSM9DS1_xgReadByte(imu, CTRL_REG8);
//
//    if (activeLow) temp |= (1<<5);
//    else temp &= ~(1<<5);
//
//    if (pushPull) temp &= ~(1<<4);
//    else temp |= (1<<4);
//
//    LSM9DS1_xgWriteByte(imu, CTRL_REG8, temp);
//}

void LSM9DS1_configInactivity(imu_t* imu, uint8_t duration, uint8_t threshold, bool sleepOn)
{
    uint8_t temp = 0;
    temp = threshold & 0x7F;
    if (sleepOn) temp |= (1<<7);
    LSM9DS1_xgWriteByte(imu, ACT_THS, temp);
    LSM9DS1_xgWriteByte(imu, ACT_DUR, duration);
}

uint8_t LSM9DS1_getInactivity(imu_t* imu)
{
    uint8_t temp = LSM9DS1_xgReadByte(imu, STATUS_REG_0);
    temp &= (0x10);
    return temp;
}

void LSM9DS1_configAccelInt(imu_t* imu, uint8_t generator, bool andInterrupts)
{
    /**
     * Use variables from accel_interrupt_generator, OR'd together to create
     * the [generator]value.
     */
    uint8_t temp = generator;
    if (andInterrupts) temp |= 0x80;
    LSM9DS1_xgWriteByte(imu, INT_GEN_CFG_XL, temp);
}

void LSM9DS1_configAccelThs(imu_t* imu, uint8_t threshold, lsm9ds1_axis_t axis, uint8_t duration, bool wait)
{
    /**
     * Write threshold value to INT_GEN_THS_?_XL.
     * axis will be 0, 1, or 2 (x, y, z respectively)
     */
    LSM9DS1_xgWriteByte(imu, INT_GEN_THS_X_XL + axis, threshold);

    /* Write duration and wait to INT_GEN_DUR_XL */
    uint8_t temp;
    temp = (duration & 0x7F);
    if (wait) temp |= 0x80;
    LSM9DS1_xgWriteByte(imu, INT_GEN_DUR_XL, temp);
}

uint8_t LSM9DS1_getAccelIntSrc(imu_t* imu)
{
    uint8_t intSrc = LSM9DS1_xgReadByte(imu, INT_GEN_SRC_XL);

    /* Check if the IA_XL (interrupt active) bit is set */
    if (intSrc & (1<<6))
    {
        return (intSrc & 0x3F);
    }

    return 0;
}

void LSM9DS1_configGyroInt(imu_t* imu, uint8_t generator, bool aoi, bool latch)
{
    /**
     * Use variables from accel_interrupt_generator, OR'd together to create
     * the [generator]value.
     */
    uint8_t temp = generator;
    if (aoi) temp |= 0x80;
    if (latch) temp |= 0x40;
    LSM9DS1_xgWriteByte(imu, INT_GEN_CFG_G, temp);
}

void LSM9DS1_configGyroThs(imu_t* imu, int16_t threshold, lsm9ds1_axis_t axis, uint8_t duration, bool wait)
{
    uint8_t buffer[2];
    buffer[0] = (threshold & 0x7F00) >> 8;
    buffer[1] = (threshold & 0x00FF);

    /**
     * Write threshold value to INT_GEN_THS_?H_G and  INT_GEN_THS_?L_G.
     * axis will be 0, 1, or 2 (x, y, z respectively)
     */
    LSM9DS1_xgWriteByte(imu, INT_GEN_THS_XH_G + (axis * 2), buffer[0]);
    LSM9DS1_xgWriteByte(imu, INT_GEN_THS_XH_G + 1 + (axis * 2), buffer[1]);

    /* Write duration and wait to INT_GEN_DUR_XL */
    uint8_t temp;
    temp = (duration & 0x7F);
    if (wait) temp |= 0x80;
    LSM9DS1_xgWriteByte(imu, INT_GEN_DUR_G, temp);
}

uint8_t LSM9DS1_getGyroIntSrc(imu_t* imu)
{
    uint8_t intSrc = LSM9DS1_xgReadByte(imu, INT_GEN_SRC_G);

    /* Check if the IA_G (interrupt active) bit is set */
    if (intSrc & (1<<6))
    {
        return (intSrc & 0x3F);
    }

    return 0;
}

void LSM9DS1_configMagInt(imu_t* imu, uint8_t generator, h_lactive_t activeLow, bool latch)
{
    /* Mask out non-generator bits (0-4) */
    uint8_t config = (generator & 0xE0);
    /* IEA bit is 0 for active-low, 1 for active-high. */
    if (activeLow == INT_ACTIVE_HIGH) config |= (1<<2);
    /* IEL bit is 0 for latched, 1 for not-latched */
    if (!latch) config |= (1<<1);
    /* As long as we have at least 1 generator, enable the interrupt */
    if (generator != 0) config |= (1<<0);

    LSM9DS1_mWriteByte(imu, INT_CFG_M, config);
}

void LSM9DS1_configMagThs(imu_t* imu, uint16_t threshold)
{
    /* Write high eight bits of [threshold] to INT_THS_H_M */
    LSM9DS1_mWriteByte(imu, INT_THS_H_M, (uint8_t)((threshold & 0x7F00) >> 8));
    /* Write low eight bits of [threshold] to INT_THS_L_M */
    LSM9DS1_mWriteByte(imu, INT_THS_L_M, (uint8_t)(threshold & 0x00FF));
}

uint8_t LSM9DS1_getMagIntSrc(imu_t* imu)
{
    uint8_t intSrc = LSM9DS1_mReadByte(imu, INT_SRC_M);

    /* Check if the INT (interrupt active) bit is set */
    if (intSrc & (1<<0))
    {
        return (intSrc & 0xFE);
    }

    return 0;
}

void LSM9DS1_sleepGyro(imu_t* imu, bool enable)
{
    uint8_t temp = LSM9DS1_xgReadByte(imu, CTRL_REG9);
    if (enable) temp |= (1<<6);
    else temp &= ~(1<<6);
    LSM9DS1_xgWriteByte(imu, CTRL_REG9, temp);
}

void LSM9DS1_enableFIFO(imu_t* imu, bool enable)
{
    uint8_t temp = LSM9DS1_xgReadByte(imu, CTRL_REG9);
    if (enable) temp |= (1<<1);
    else temp &= ~(1<<1);
    LSM9DS1_xgWriteByte(imu, CTRL_REG9, temp);
}

void LSM9DS1_setFIFO(imu_t* imu, fifoMode_type_t fifoMode, uint8_t fifoThs)
{
    /**
     * Limit threshold - 0x1F (31) is the maximum. If more than that was asked
     * limit it to the maximum.
     */
    uint8_t threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
    LSM9DS1_xgWriteByte(imu, FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
}

uint8_t LSM9DS1_getFIFOSamples(imu_t* imu)
{
    return (LSM9DS1_xgReadByte(imu, FIFO_SRC) & 0x3F);
}

void LSM9DS1_constrainScales(imu_t* imu)
{
    if ((imu->settings.gyro.scale != 245) && (imu->settings.gyro.scale != 500) &&
        (imu->settings.gyro.scale != 2000))
    {
        imu->settings.gyro.scale = 245;
    }

    if ((imu->settings.accel.scale != 2) && (imu->settings.accel.scale != 4) &&
        (imu->settings.accel.scale != 8) && (imu->settings.accel.scale != 16))
    {
        imu->settings.accel.scale = 2;
    }

    if ((imu->settings.mag.scale != 4) && (imu->settings.mag.scale != 8) &&
        (imu->settings.mag.scale != 12) && (imu->settings.mag.scale != 16))
    {
        imu->settings.mag.scale = 4;
    }
}

void LSM9DS1_xgWriteByte(imu_t* imu, uint8_t subAddress, uint8_t data)
{
    LSM9DS1_I2CwriteByte(imu->settings.device.agAddress, subAddress, data);
}

void LSM9DS1_mWriteByte(imu_t* imu, uint8_t subAddress, uint8_t data)
{
    LSM9DS1_I2CwriteByte(imu->settings.device.mAddress, subAddress, data);
}

uint8_t LSM9DS1_xgReadByte(imu_t* imu, uint8_t subAddress)
{
    return LSM9DS1_I2CreadByte(imu->settings.device.agAddress, subAddress);
}

void LSM9DS1_xgReadBytes(imu_t* imu, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
    LSM9DS1_I2CreadBytes(imu->settings.device.agAddress, subAddress, dest, count);
}

uint8_t LSM9DS1_mReadByte(imu_t* imu, uint8_t subAddress)
{
    return LSM9DS1_I2CreadByte(imu->settings.device.mAddress, subAddress);
}

void LSM9DS1_mReadBytes(imu_t* imu, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
    LSM9DS1_I2CreadBytes(imu->settings.device.mAddress, subAddress, dest, count);
}

void LSM9DS1_I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    I2C_Write(I2C3_BASE, address, &data, 1, subAddress);
}

uint8_t LSM9DS1_I2CreadByte(uint8_t address, uint8_t subAddress)
{
    unsigned char read_byte;
    I2C_Read(I2C3_BASE, address, &read_byte, 1, subAddress, false);
    return read_byte;
}

void LSM9DS1_I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
    I2C_Read(I2C3_BASE, address, dest, count, subAddress, false);
}

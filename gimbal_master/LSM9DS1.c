/******************************************************************************
LSM9DS1.c
LSM9DS1 Library Source File
Nghia Nguyen @ PayItForward Club
Created On: Oct 1, 2016

This file implements basic functions of the LSM9DS1.

Basic Functions:
- Initialize module
- Select running mode & range
- Output RAW data of 9DoF

Development environment specifics:
IDE: TI CCS 6.2.0
Hardware Platform: TM4C123GH6PM (Tiva C Launchpad)

Reference: Sparkfun Arduino Library.
Distributed as-is; no warranty is given.
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"
#include "LSM9DS1.h"

#define LSM9DS1_BASE		I2C3_BASE

struct deviceSettings device;
struct gyroSettings gyro;
struct accelSettings accel;
struct magSettings mag;
struct temperatureSettings temp;

enum lsm9ds1_axis{
	X_AXIS,
	Y_AXIS,
	Z_AXIS,
	ALL_AXIS
};

/*
 * Global variable
 */
// We'll store the gyro, accel, and magnetometer readings in a series of
// public class variables. Each sensor gets three variables -- one for each
// axis. Call readGyro(), readAccel(), and readMag() first, before using
// these variables!
// These values are the RAW signed 16-bit readings from the sensors.
int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
int16_t temperature; // Chip temperature

float gBias[3], aBias[3], mBias[3];
int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];

// gRes, aRes, and mRes store the current resolution for each sensor.
// Units of these values would be DPS (or g's or Gs's) per ADC tick.
// This value is calculated as (sensor scale) / (2^15).
float gRes, aRes, mRes;
float magSensitivity[4] = {0.00014, 0.00029, 0.00043, 0.00058};



void IMU_readWHOAMI_AG(uint8_t *ui8data){
	I2C_Read(LSM9DS1_BASE,device.agAddress, ui8data,1, WHO_AM_I_XG, false);
}

void IMU_readWHOAMI_M(uint8_t *ui8data){
	I2C_Read(LSM9DS1_BASE,device.mAddress, ui8data,1, WHO_AM_I_M, false);
}

void IMU_init(void)
{
	I2C_Init(LSM9DS1_BASE, true);		//fast mode
	{
		device.commInterface = IMU_MODE_I2C;
		//! TODO
		//! Change I2C address here (if needed)
		device.agAddress = LSM9DS1_AG_ADDR;
		device.mAddress = LSM9DS1_M_ADDR;

		gyro.enabled = true;
		gyro.enableX = true;
		gyro.enableY = true;
		gyro.enableZ = true;
		// gyro scale can be 245, 500, or 2000
		gyro.scale = 245;
		// gyro sample rate: value between 1-6
		// 1 = 14.9    4 = 238
		// 2 = 59.5    5 = 476
		// 3 = 119     6 = 952
		gyro.sampleRate = 6;
		// gyro cutoff frequency: value between 0-3
		// Actual value of cutoff frequency depends
		// on sample rate.
		gyro.bandwidth = 0;
		gyro.lowPowerEnable = false;
		gyro.HPFEnable = false;
		// Gyro HPF cutoff frequency: value between 0-9
		// Actual value depends on sample rate. Only applies
		// if gyroHPFEnable is true.
		gyro.HPFCutoff = 0;
		gyro.flipX = false;
		gyro.flipY = false;
		gyro.flipZ = false;
		gyro.orientation = 0;
		gyro.latchInterrupt = true;

		accel.enabled = true;
		accel.enableX = true;
		accel.enableY = true;
		accel.enableZ = true;
		// accel scale can be 2, 4, 8, or 16
		accel.scale = 2;
		// accel sample rate can be 1-6
		// 1 = 10 Hz    4 = 238 Hz
		// 2 = 50 Hz    5 = 476 Hz
		// 3 = 119 Hz   6 = 952 Hz
		accel.sampleRate = 6;
		// Accel cutoff freqeuncy can be any value between -1 - 3.
		// -1 = bandwidth determined by sample rate
		// 0 = 408 Hz   2 = 105 Hz
		// 1 = 211 Hz   3 = 50 Hz
		accel.bandwidth = -1;
		accel.highResEnable = false;
		// accelHighResBandwidth can be any value between 0-3
		// LP cutoff is set to a factor of sample rate
		// 0 = ODR/50    2 = ODR/9
		// 1 = ODR/100   3 = ODR/400
		accel.highResBandwidth = 0;

		mag.enabled = true;
		// mag scale can be 4, 8, 12, or 16
		mag.scale = 4;
		// mag data rate can be 0-7
		// 0 = 0.625 Hz  4 = 10 Hz
		// 1 = 1.25 Hz   5 = 20 Hz
		// 2 = 2.5 Hz    6 = 40 Hz
		// 3 = 5 Hz      7 = 80 Hz
		mag.sampleRate = 7;
		mag.tempCompensationEnable = false;
		// magPerformance can be any value between 0-3
		// 0 = Low power mode      2 = high performance
		// 1 = medium performance  3 = ultra-high performance
		mag.XYPerformance = 3;
		mag.ZPerformance = 3;
		mag.lowPowerEnable = false;
		// magOperatingMode can be 0-2
		// 0 = continuous conversion
		// 1 = single-conversion
		// 2 = power down
		mag.operatingMode = 0;

		temp.enabled = true;
		int i;
		for (i=0; i<3; i++)
		{
			gBias[i] = 0;
			aBias[i] = 0;
			mBias[i] = 0;
			gBiasRaw[i] = 0;
			aBiasRaw[i] = 0;
			mBiasRaw[i] = 0;
		}
		//				_autoCalc = false;
	}
}
void calcgRes()
{
	gRes = (gyro.scale) / 32768.0;
}

void calcaRes()
{
	aRes = (accel.scale) / 32768.0;
}

void calcmRes()
{
	//mRes = ((float) settings.mag.scale) / 32768.0;
	switch (mag.scale)
	{
	case 4:
		mRes = magSensitivity[0];
		break;
	case 8:
		mRes = magSensitivity[1];
		break;
	case 12:
		mRes = magSensitivity[2];
		break;
	case 16:
		mRes = magSensitivity[3];
		break;
	}
}
bool LSM9DS1_begin(void)
{
	constrainScales();
	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
	calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable
	calcaRes(); // Calculate g / ADC tick, stored in aRes variable

	//	if (whoAmICombined != ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP))
	//		return 0;

	// Gyro initialization stuff:
	initGyro();	// This will "turn on" the gyro. Setting up interrupts, etc.

	// Accelerometer initialization stuff:
	initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.

	// Magnetometer initialization stuff:
	initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.

	// Once everything is initialized, return the WHO_AM_I registers we read:
	return true;
}

void initGyro()
{
	uint8_t tempRegValue = 0;

	// CTRL_REG1_G (Default value: 0x00)
	// [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
	// ODR_G[2:0] - Output data rate selection
	// FS_G[1:0] - Gyroscope full-scale selection
	// BW_G[1:0] - Gyroscope bandwidth selection

	// To disable gyro, set sample rate bits to 0. We'll only set sample
	// rate if the gyro is enabled.
	if (gyro.enabled)
	{
		tempRegValue = (gyro.sampleRate & 0x07) << 5;
	}
	switch (gyro.scale)
	{
	case 500:
		tempRegValue |= (0x1 << 3);
		break;
	case 2000:
		tempRegValue |= (0x3 << 3);
		break;
		// Otherwise we'll set it to 245 dps (0x0 << 4)
	}
	tempRegValue |= (gyro.bandwidth & 0x3);
	xgWriteByte(CTRL_REG1_G, tempRegValue);

	// CTRL_REG2_G (Default value: 0x00)
	// [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
	// INT_SEL[1:0] - INT selection configuration
	// OUT_SEL[1:0] - Out selection configuration
	xgWriteByte(CTRL_REG2_G, 0x00);

	// CTRL_REG3_G (Default value: 0x00)
	// [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
	// LP_mode - Low-power mode enable (0: disabled, 1: enabled)
	// HP_EN - HPF enable (0:disabled, 1: enabled)
	// HPCF_G[3:0] - HPF cutoff frequency
	tempRegValue = gyro.lowPowerEnable ? (1<<7) : 0;
	if (gyro.HPFEnable)
	{
		tempRegValue |= (1<<6) | (gyro.HPFCutoff & 0x0F);
	}
	xgWriteByte(CTRL_REG3_G, tempRegValue);

	// CTRL_REG4 (Default value: 0x38)
	// [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
	// Zen_G - Z-axis output enable (0:disable, 1:enable)
	// Yen_G - Y-axis output enable (0:disable, 1:enable)
	// Xen_G - X-axis output enable (0:disable, 1:enable)
	// LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
	// 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
	tempRegValue = 0;
	if (gyro.enableZ) tempRegValue |= (1<<5);
	if (gyro.enableY) tempRegValue |= (1<<4);
	if (gyro.enableX) tempRegValue |= (1<<3);
	if (gyro.latchInterrupt) tempRegValue |= (1<<1);
	xgWriteByte(CTRL_REG4, tempRegValue);

	// ORIENT_CFG_G (Default value: 0x00)
	// [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
	// SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
	// Orient [2:0] - Directional user orientation selection
	tempRegValue = 0;
	if (gyro.flipX) tempRegValue |= (1<<5);
	if (gyro.flipY) tempRegValue |= (1<<4);
	if (gyro.flipZ) tempRegValue |= (1<<3);
	xgWriteByte(ORIENT_CFG_G, tempRegValue);
}

void initAccel()
{
	uint8_t tempRegValue = 0;

	//	CTRL_REG5_XL (0x1F) (Default value: 0x38)
	//	[DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
	//	DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
	//		00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
	//	Zen_XL - Z-axis output enabled
	//	Yen_XL - Y-axis output enabled
	//	Xen_XL - X-axis output enabled
	if (accel.enableZ) tempRegValue |= (1<<5);
	if (accel.enableY) tempRegValue |= (1<<4);
	if (accel.enableX) tempRegValue |= (1<<3);

	xgWriteByte(CTRL_REG5_XL, tempRegValue);

	// CTRL_REG6_XL (0x20) (Default value: 0x00)
	// [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
	// ODR_XL[2:0] - Output data rate & power mode selection
	// FS_XL[1:0] - Full-scale selection
	// BW_SCAL_ODR - Bandwidth selection
	// BW_XL[1:0] - Anti-aliasing filter bandwidth selection
	tempRegValue = 0;
	// To disable the accel, set the sampleRate bits to 0.
	if (accel.enabled)
	{
		tempRegValue |= (accel.sampleRate & 0x07) << 5;
	}
	switch (accel.scale)
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
	if (accel.bandwidth >= 0)
	{
		tempRegValue |= (1<<2); // Set BW_SCAL_ODR
		tempRegValue |= (accel.bandwidth & 0x03);
	}
	xgWriteByte(CTRL_REG6_XL, tempRegValue);

	// CTRL_REG7_XL (0x21) (Default value: 0x00)
	// [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
	// HR - High resolution mode (0: disable, 1: enable)
	// DCF[1:0] - Digital filter cutoff frequency
	// FDS - Filtered data selection
	// HPIS1 - HPF enabled for interrupt function
	tempRegValue = 0;
	if (accel.highResEnable)
	{
		tempRegValue |= (1<<7); // Set HR bit
		tempRegValue |= (accel.highResBandwidth & 0x3) << 5;
	}
	xgWriteByte(CTRL_REG7_XL, tempRegValue);
}

void initMag()
{
	uint8_t tempRegValue = 0;

	// CTRL_REG1_M (Default value: 0x10)
	// [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
	// TEMP_COMP - Temperature compensation
	// OM[1:0] - X & Y axes op mode selection
	//	00:low-power, 01:medium performance
	//	10: high performance, 11:ultra-high performance
	// DO[2:0] - Output data rate selection
	// ST - Self-test enable
	if (mag.tempCompensationEnable) tempRegValue |= (1<<7);
	tempRegValue |= (mag.XYPerformance & 0x3) << 5;
	tempRegValue |= (mag.sampleRate & 0x7) << 2;
	mWriteByte(CTRL_REG1_M, tempRegValue);

	// CTRL_REG2_M (Default value 0x00)
	// [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
	// FS[1:0] - Full-scale configuration
	// REBOOT - Reboot memory content (0:normal, 1:reboot)
	// SOFT_RST - Reset config and user registers (0:default, 1:reset)
	tempRegValue = 0;
	switch (mag.scale)
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
	mWriteByte(CTRL_REG2_M, tempRegValue); // +/-4Gauss

	// CTRL_REG3_M (Default value: 0x03)
	// [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
	// I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
	// LP - Low-power mode cofiguration (1:enable)
	// SIM - SPI mode selection (0:write-only, 1:read/write enable)
	// MD[1:0] - Operating mode
	//	00:continuous conversion, 01:single-conversion,
	//  10,11: Power-down
	tempRegValue = 0;
	if (mag.lowPowerEnable) tempRegValue |= (1<<5);
	tempRegValue |= (mag.operatingMode & 0x3);
	mWriteByte(CTRL_REG3_M, tempRegValue); // Continuous conversion mode

	// CTRL_REG4_M (Default value: 0x00)
	// [0][0][0][0][OMZ1][OMZ0][BLE][0]
	// OMZ[1:0] - Z-axis operative mode selection
	//	00:low-power mode, 01:medium performance
	//	10:high performance, 10:ultra-high performance
	// BLE - Big/little endian data
	tempRegValue = 0;
	tempRegValue = (mag.ZPerformance & 0x3) << 2;
	mWriteByte(CTRL_REG4_M, tempRegValue);

	// CTRL_REG5_M (Default value: 0x00)
	// [0][BDU][0][0][0][0][0][0]
	// BDU - Block data update for magnetic data
	//	0:continuous, 1:not updated until MSB/LSB are read
	tempRegValue = 0;
	mWriteByte(CTRL_REG5_M, tempRegValue);
}

uint8_t accelAvailable()
{
	uint8_t status = xgReadByte(STATUS_REG_1);

	return (status & (1<<0));
}

uint8_t gyroAvailable()
{
	uint8_t status = xgReadByte(STATUS_REG_1);

	return ((status & (1<<1)) >> 1);
}

uint8_t tempAvailable()
{
	uint8_t status = xgReadByte(STATUS_REG_1);

	return ((status & (1<<2)) >> 2);
}
//! TODO
//////////////////////////////////

void readAccel()
{
	uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp
	xgReadBytes(OUT_X_L_XL, temp, 6); // Read 6 bytes, beginning at OUT_X_L_XL
	ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
	ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
	az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
	//	if (_autoCalc)
	//	{
	//		ax -= aBiasRaw[X_AXIS];
	//		ay -= aBiasRaw[Y_AXIS];
	//		az -= aBiasRaw[Z_AXIS];
	//	}
}

void readMag()
{
	uint8_t temp[6]; // We'll read six bytes from the mag into temp
	mReadBytes(OUT_X_L_M, temp, 6); // Read 6 bytes, beginning at OUT_X_L_M
	mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
	my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
	mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
}

void readTemp()
{
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp
	xgReadBytes(OUT_TEMP_L, temp, 2); // Read 2 bytes, beginning at OUT_TEMP_L
	temperature = ((int16_t)temp[1] << 8) | temp[0];
}

void readGyro()
{
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	xgReadBytes(OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
	gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
	gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
	gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
	//	if (_autoCalc)
	//	{
	//		gx -= gBiasRaw[X_AXIS];
	//		gy -= gBiasRaw[Y_AXIS];
	//		gz -= gBiasRaw[Z_AXIS];
	//	}
}

float calcGyro(int16_t gyro)
{
	// Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
	return gRes * gyro;
}

float calcAccel(int16_t accel)
{
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return aRes * accel;
}

float calcMag(int16_t mag)
{
	// Return the mag raw reading times our pre-calculated Gs / (ADC tick):
	return mRes * mag;
}

void setGyroScale(uint16_t gScl)
{
	// Read current value of CTRL_REG1_G:
	uint8_t ctrl1RegValue = xgReadByte(CTRL_REG1_G);
	// Mask out scale bits (3 & 4):
	ctrl1RegValue &= 0xE7;
	switch (gScl)
	{
	case 500:
		ctrl1RegValue |= (0x1 << 3);
		gyro.scale = 500;
		break;
	case 2000:
		ctrl1RegValue |= (0x3 << 3);
		gyro.scale = 2000;
		break;
	default: // Otherwise we'll set it to 245 dps (0x0 << 4)
		gyro.scale = 245;
		break;
	}
	xgWriteByte(CTRL_REG1_G, ctrl1RegValue);

	calcgRes();
}

void setAccelScale(uint8_t aScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
	uint8_t tempRegValue = xgReadByte(CTRL_REG6_XL);
	// Mask out accel scale bits:
	tempRegValue &= 0xE7;

	switch (aScl)
	{
	case 4:
		tempRegValue |= (0x2 << 3);
		accel.scale = 4;
		break;
	case 8:
		tempRegValue |= (0x3 << 3);
		accel.scale = 8;
		break;
	case 16:
		tempRegValue |= (0x1 << 3);
		accel.scale = 16;
		break;
	default: // Otherwise it'll be set to 2g (0x0 << 3)
		accel.scale = 2;
		break;
	}
	xgWriteByte(CTRL_REG6_XL, tempRegValue);

	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes();
}

void setMagScale(uint8_t mScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG2_M);
	// Then mask out the mag scale bits:
	temp &= 0xFF^(0x3 << 5);

	switch (mScl)
	{
	case 8:
		temp |= (0x1 << 5);
		mag.scale = 8;
		break;
	case 12:
		temp |= (0x2 << 5);
		mag.scale = 12;
		break;
	case 16:
		temp |= (0x3 << 5);
		mag.scale = 16;
		break;
	default: // Otherwise we'll default to 4 gauss (00)
		mag.scale = 4;
		break;
	}

	// And write the new register value back into CTRL_REG6_XM:
	mWriteByte(CTRL_REG2_M, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	//mScale = mScl;
	// Then calculate a new mRes, which relies on mScale being set correctly:
	calcmRes();
}

void setGyroODR(uint8_t gRate)
{
	// Only do this if gRate is not 0 (which would disable the gyro)
	if ((gRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
		uint8_t temp = xgReadByte(CTRL_REG1_G);
		// Then mask out the gyro ODR bits:
		temp &= 0xFF^(0x7 << 5);
		temp |= (gRate & 0x07) << 5;
		// Update our settings struct
		gyro.sampleRate = gRate & 0x07;
		// And write the new register value back into CTRL_REG1_G:
		xgWriteByte(CTRL_REG1_G, temp);
	}
}

void setAccelODR(uint8_t aRate)
{
	// Only do this if aRate is not 0 (which would disable the accel)
	if ((aRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
		uint8_t temp = xgReadByte(CTRL_REG6_XL);
		// Then mask out the accel ODR bits:
		temp &= 0x1F;
		// Then shift in our new ODR bits:
		temp |= ((aRate & 0x07) << 5);
		accel.sampleRate = aRate & 0x07;
		// And write the new register value back into CTRL_REG1_XM:
		xgWriteByte(CTRL_REG6_XL, temp);
	}
}

void setMagODR(uint8_t mRate)
{
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG1_M);
	// Then mask out the mag ODR bits:
	temp &= 0xFF^(0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= ((mRate & 0x07) << 2);
	mag.sampleRate = mRate & 0x07;
	// And write the new register value back into CTRL_REG5_XM:
	mWriteByte(CTRL_REG1_M, temp);
}


//! TODO
//////////////////////////////////
void constrainScales()
{
	if ((gyro.scale != 245) && (gyro.scale != 500) &&
			(gyro.scale != 2000))
	{
		gyro.scale = 245;
	}

	if ((accel.scale != 2) && (accel.scale != 4) &&
			(accel.scale != 8) && (accel.scale != 16))
	{
		accel.scale = 2;
	}

	if ((mag.scale != 4) && (mag.scale != 8) &&
			(mag.scale != 12) && (mag.scale != 16))
	{
		mag.scale = 4;
	}
}

void xgWriteByte(uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// gyro-specific I2C address or SPI CS pin.
	if (device.commInterface == IMU_MODE_I2C)
		//		I2CwriteByte(_xgAddress, subAddress, data);
		I2C_Write(LSM9DS1_BASE, LSM9DS1_AG_ADDR, &data, 1, subAddress);
	//	else if (settings.device.commInterface == IMU_MODE_SPI)
	//		SPIwriteByte(_xgAddress, subAddress, data);
}

void mWriteByte(uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (device.commInterface == IMU_MODE_I2C)
		//		return I2CwriteByte(_mAddress, subAddress, data);
		return I2C_Write(LSM9DS1_BASE, LSM9DS1_M_ADDR, &data, 1, subAddress);
	//	else if (settings.device.commInterface == IMU_MODE_SPI)
	//		return SPIwriteByte(_mAddress, subAddress, data);
}

uint8_t xgReadByte(uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// gyro-specific I2C address or SPI CS pin.
	unsigned char read_byte;
	if (device.commInterface == IMU_MODE_I2C)
		//		return I2CreadByte(_xgAddress, subAddress);
		I2C_Read(LSM9DS1_BASE, LSM9DS1_AG_ADDR, &read_byte, 1, subAddress, false);
	return read_byte;

	//	else if (settings.device.commInterface == IMU_MODE_SPI)
	//		return SPIreadByte(_xgAddress, subAddress);
}

void xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// gyro-specific I2C address or SPI CS pin.
	if (device.commInterface == IMU_MODE_I2C)
		//		I2CreadBytes(_xgAddress, subAddress, dest, count);
		I2C_Read(LSM9DS1_BASE, LSM9DS1_AG_ADDR, dest, count, subAddress, false);
	//	else if (settings.device.commInterface == IMU_MODE_SPI)
	//		SPIreadBytes(_xgAddress, subAddress, dest, count);
}

uint8_t mReadByte(uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	unsigned char read_byte;
	if (device.commInterface == IMU_MODE_I2C)
		//		return I2CreadByte(_xgAddress, subAddress);
		I2C_Read(LSM9DS1_BASE, LSM9DS1_M_ADDR, &read_byte, 1, subAddress, false);
	return read_byte;
	//		return I2CreadByte(_mAddress, subAddress);
	//	else if (settings.device.commInterface == IMU_MODE_SPI)
	//		return SPIreadByte(_mAddress, subAddress);
}

void mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (device.commInterface == IMU_MODE_I2C)
		//		I2CreadBytes(_mAddress, subAddress, dest, count);
		I2C_Read(LSM9DS1_BASE, LSM9DS1_M_ADDR, dest, count, subAddress, false);
	//	else if (settings.device.commInterface == IMU_MODE_SPI)
	//		SPIreadBytes(_mAddress, subAddress, dest, count);
}


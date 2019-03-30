/******************************************************************************
LSM9DS1.h
LSM9DS1 Library Header File
Nghia Nguyen @ PayItForward Club
Created On: Oct 1, 2016

This file prototypes the LSM9DS1 functions, implemented in LSM9DS1.c.
In addition, it defines every registers in the LSM9DS1
(Gyro/Accel/Magnetometer registers).

Basic Functions:
- Initialize module
- Select operating mode & range
- Output RAW data of 9DoF

Development environment specifics:
- IDE: TI CCS 6.2.0
- Hardware Platform: TM4C123GH6PM (Tiva C Launchpad)
- I2C Interface Only

Referenced from Sparkfun Arduino Library.
 ******************************************************************************/

#ifndef LSM9DS1_H_
#define LSM9DS1_H_

#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"

/*
 * Global variable
 */
// We'll store the gyro, accel, and magnetometer readings in a series of
// public class variables. Each sensor gets three variables -- one for each
// axis. Call readGyro(), readAccel(), and readMag() first, before using
// these variables!
// These values are the RAW signed 16-bit readings from the sensors.
extern int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
extern int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
extern int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
extern int16_t temperature; // Chip temperature



///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M_ADDR		0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG_ADDR		0x6B // Would be 0x6A if SDO_AG is LOW

//****************************Function prototypes******************************
void IMU_readWHOAMI_AG(uint8_t *ui8data);
void IMU_readWHOAMI_M(uint8_t *ui8data);

void IMU_init(void);
void calcgRes();
void calcaRes();
void calcmRes();
bool LSM9DS1_begin(void);


void initGyro();
void initAccel();
void initMag();

uint8_t accelAvailable();
uint8_t gyroAvailable();
uint8_t tempAvailable();

void readAccel();
void readMag();
void readTemp();
void readGyro();

float calcGyro(int16_t gyro);
float calcAccel(int16_t accel);
float calcMag(int16_t mag);

void setGyroScale(uint16_t gScl);
void setAccelScale(uint8_t aScl);
void setMagScale(uint8_t mScl);
void setGyroODR(uint8_t gRate);
void setAccelODR(uint8_t aRate);
void setMagODR(uint8_t mRate);

void constrainScales();
void xgWriteByte(uint8_t subAddress, uint8_t data);
void mWriteByte(uint8_t subAddress, uint8_t data);
uint8_t xgReadByte(uint8_t subAddress);
void xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
uint8_t mReadByte(uint8_t subAddress);
void mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
//*****************************************************************************



#endif /* LSM9DS1_H_ */

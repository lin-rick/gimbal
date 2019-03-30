/*
 * I2C.c - I2C Library Header File
 *
 * Date: 01/10/2016
 * Revision: 1.05
 * Author: PIFers
 *
 * This library supports the communication between TIVA and
 * devices via I2C interface.
 *
 * This library supports 4 I2C modules (from 0 to 3).
 *
 *		Hardware connections:
 * 		I2Cx |  SDA  |  SCL
 * 		---- + ----- + -----
 * 		I2C0 |  PB3	 |  PB2
 * 		I2C1	 |  PA7	 |  PA6
 * 		I2C2	 |  PE5	 |  PE4
 * 		I2C3	 |  PD1	 |  PD0
 *
 */
#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/i2c.h"
#include "inc/hw_i2c.h"
#include "global_include.h"


//****************************Function prototypes******************************
void I2C_Init(uint32_t ui32Base, bool bFast);
void I2C_Write(uint32_t ui32Base, unsigned char uiSlave_add, unsigned char *ucData,
		uint16_t uiCount, unsigned char ucStart_add );
void I2C_Read(uint32_t ui32Base, unsigned char uiSlave_add, unsigned char *ucRec_Data,
		uint16_t uiCount, unsigned char ucStart_add, bool bDummyRead);
//*****************************************************************************
#endif	/* I2C_H_ */

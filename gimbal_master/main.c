/*******************************************************************************
* Included headers
*******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "global_include.h"
#include "basic_functions.h"
#include "LSM9DS1.h"

/*******************************************************************************
* Global Variables
*******************************************************************************/

/* Device information */
dev_t dev;

/* IMU Configuration */
static imu_t imu;
imu_config_t imu_config = {
    .enable_accel = true,
    .enable_gyro = false,
    .enable_mag = false,
    .ag_addr = LSM9DS1_AG_ADDR(1), // Argument selects pull up or down
    .mag_addr = LSM9DS1_M_ADDR(1),
    .low_power_mode = true
};

/*******************************************************************************
* Function Definitions
*******************************************************************************/

void printAccel();
void printAttitude();

/*******************************************************************************
* Main Loop
*******************************************************************************/

int main(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	UART_init();
	I2C_Init(I2C3_BASE, true);
	LSM9DS1_init(&imu, &imu_config, &dev);

	while(1)
	{
	    LSM9DS1_step(&imu, &dev);
		printAccel(); // Print "A: ax, ay, az"
		printAttitude();
		SysCtlDelay(1000000);
	}
}

void printAccel()
{
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  UARTprintf("A: ");
  UARTprintf("x:%5d ",imu.ax);
  UARTprintf("y:%5d ",imu.ay);
  UARTprintf("z:%5d ",imu.az);
}


// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude()
{
  int16_t i16roll, i16pitch;
  i16roll = dev.data.roll;
  i16pitch = dev.data.pitch;

  UARTprintf("Pitch: %3d,",i16pitch);
  UARTprintf("Roll: %3d,",i16roll);
  UARTprintf(" @degree\n");
}

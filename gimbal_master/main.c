#include <stdint.h>
#include <stdbool.h>
#include "global_include.h"
#include "basic_functions.h"
#include "LSM9DS1.h"
#include <math.h>
uint8_t rev_data[10];
int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
int16_t temperature; // Chip temperature

const float PI = 3.14159265358979323846;
/*
 * Subfunction Prototypes
 */

void printAccel();
void printGyro();
void printMag();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);

/*
 * Main Control Loop
 */
int main(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	UART_init();
	IMU_init();
	while(!(LSM9DS1_begin())){};
//	initAccel();

	IMU_readWHOAMI_AG(&rev_data[0]);
	IMU_readWHOAMI_M(&rev_data[1]);
	UARTprintf("who_am_i: %x %x\n",rev_data[0],rev_data[1]);

	while(1)
	{
		printAccel(); // Print "A: ax, ay, az"
		printGyro();  // Print "G: gx, gy, gz"
		printMag();   // Print "M: mx, my, mz"

		printAttitude(ax, ay, az, -my, -mx, mz);
		SysCtlDelay(1000000);
	}
}

void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  readAccel();

  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  UARTprintf("A: ");
  UARTprintf("x:%5d ",ax);
  UARTprintf("y:%5d ",ay);
  UARTprintf("z:%5d ",az);
//  UARTprintf("\n");
}

void printGyro()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  readGyro();

  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  UARTprintf("G: ");
  UARTprintf("x:%5d ",gx);
  UARTprintf("y:%5d ",gy);
  UARTprintf("z:%5d ",gz);
//  UARTprintf("\n");
}

void printMag()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  readMag();

  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  UARTprintf("M: ");
  UARTprintf("x:%5d ",mx);
  UARTprintf("y:%5d ",my);
  UARTprintf("z:%5d ",mz);
  UARTprintf("\n");
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);

//  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  int16_t i16roll, i16pitch, i16heading;
  i16roll = roll;
  i16pitch = pitch;
  i16heading = heading;

  UARTprintf("Pitch: %3d,",i16pitch);
  UARTprintf("Roll: %3d,",i16roll);
  UARTprintf("Heading: %3d",i16heading);
  UARTprintf(" @degree\n");
}

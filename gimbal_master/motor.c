/*******************************************************************************
* Included headers
*******************************************************************************/

#include "motor.h"

/*******************************************************************************
* Public functions
*******************************************************************************/

void Motor_init() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,0);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,0);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_6,0);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_7,0);
}

void DriveMotor1(int step,int dir)
{
    int i =0;
    if (dir==1)
    {
        for (i=0; i< step; i++)
        {
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,GPIO_PIN_0); //direction pin
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,GPIO_PIN_1); //step pin
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,0);
        }
    }
    else
    {
        for (i=0; i< step; i++)
        {
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,0); //direction pin
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,GPIO_PIN_1); //step pin
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,0);
        }
    }
}
void DriveMotor2(int step,int dir)
{
    int i =0;
    if (dir==1)
    {
        for (i=0; i< step; i++)
        {
            GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_6,GPIO_PIN_6); //direction pin
            GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,GPIO_PIN_7); //step pin
            GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
        }
    }
    else
    {
        for (i=0; i< step; i++)
        {
            GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_6,0); //direction pin
            GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,GPIO_PIN_7); //step pin
            GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7,0);
        }
    }
}

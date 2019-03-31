/*******************************************************************************
* Included headers
*******************************************************************************/

#include "motor.h"

/*******************************************************************************
* Public functions
*******************************************************************************/

void Motor_init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,0);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,0);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,0);
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3,0);
}

void Motor_step(dev_t* dev) {
    if (dev->axis_lock == 0)
    {

        if (dev->data.roll > THRESHOLD)
        {
            _driveMotor2(1,C_CLOCKWISE);
        }
        else if (dev->data.roll < -THRESHOLD)
        {
            _driveMotor2(1,CLOCKWISE);
        }
        if(dev->data.pitch > THRESHOLD)
        {
            _driveMotor1(1,C_CLOCKWISE);
        }
        else if(dev->data.pitch < -THRESHOLD)
        {
            _driveMotor1(1,CLOCKWISE);
        }

    }
    else
    {
        _driveMotor1(STEP_SIZE, dev->motor1_dir);
        _driveMotor2(STEP_SIZE, dev->motor2_dir);
        dev->motor1_dir = 0;
        dev->motor2_dir = 0;

    }
}


/*******************************************************************************
* Private functions
*******************************************************************************/
void _driveMotor1(int step,int dir)
{
    int i =0;
    if (dir == 1)
    {
        for (i=0; i< step; i++)
        {
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,GPIO_PIN_0); //direction pin
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,GPIO_PIN_1); //step pin
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,0);
            SysCtlDelay(1000000/50);
        }
    }
    else if (dir == 2)
    {
        for (i=0; i< step; i++)
        {
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,0); //direction pin
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,GPIO_PIN_1); //step pin
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,0);
            SysCtlDelay(1000000/50);
        }
    }
}
void _driveMotor2(int step,int dir)
{
    int i = 0;
    if (dir == 1)
    {
        for (i=0; i< step; i++)
        {
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,GPIO_PIN_2); //direction pin
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3,GPIO_PIN_3); //step pin
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3,0);
            SysCtlDelay(1000000/70);
        }
    }
    else if (dir == 2)
    {
        for (i=0; i< step; i++)
        {
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,0); //direction pin
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3,GPIO_PIN_3); //step pin
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3,0);
            SysCtlDelay(1000000/70);
        }
    }
}

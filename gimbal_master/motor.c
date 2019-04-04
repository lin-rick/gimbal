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
    int step_size_p = 1;
    int step_size_r = 1;
    if (dev->axis_lock == 0)
    {
        step_size_r = abs(dev->data.roll/10)*15+5;
        step_size_p = abs(dev->data.pitch/10)*10+5;
//        if(abs(dev->data.roll) > 20) {
//            step_size_r = 3;
//        }
//        if(abs(dev->data.pitch) > 20) {
//            step_size_p = 3;
//        }


        if (dev->data.roll > THRESHOLD)
        {
            _driveMotor2(step_size_r,C_CLOCKWISE);
        }
        else if (dev->data.roll < -THRESHOLD)
        {
            _driveMotor2(step_size_r,CLOCKWISE);
        }
        if(dev->data.pitch > THRESHOLD)
        {
            _driveMotor1(step_size_p,C_CLOCKWISE);
        }
        else if(dev->data.pitch < -THRESHOLD)
        {
            _driveMotor1(step_size_p,CLOCKWISE);
        }

    }
    else
    {
        _driveMotor1(MANUAL_STEP_SIZE, dev->motor1_dir);
        _driveMotor2(MANUAL_STEP_SIZE, dev->motor2_dir);
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
            SysCtlDelay(MOTOR_DELAY);
        }
    }
    else if (dir == 2)
    {
        for (i=0; i< step; i++)
        {
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,0); //direction pin
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,GPIO_PIN_1); //step pin
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,0);
            SysCtlDelay(MOTOR_DELAY);
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
            SysCtlDelay(MOTOR_DELAY);
        }
    }
    else if (dir == 2)
    {
        for (i=0; i< step; i++)
        {
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,0); //direction pin
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3,GPIO_PIN_3); //step pin
            GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_3,0);
            SysCtlDelay(MOTOR_DELAY);
        }
    }
}

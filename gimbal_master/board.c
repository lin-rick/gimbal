/*******************************************************************************
* Included headers
*******************************************************************************/

#include <board.h>

/*******************************************************************************
* Global variables
*******************************************************************************/
int axis_lock = 1;
int button_pressed = 0;
int motor1_dir = 0;
int motor2_dir = 0;

/*******************************************************************************
* Public functions
*******************************************************************************/

void UART_init(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	UARTStdioConfig(0, 115200, SysCtlClockGet());

	IntMasterEnable(); //enable processor interrupts
	IntEnable(INT_UART0); //enable the UART interrupt
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interrupts
}

void button_init(void)
{
    // declare interupt pin - for button
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6); // setup pin C4, C5, C6, C7 as input
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    SysCtlDelay(3);
    GPIOIntDisable(GPIO_PORTC_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);
    GPIOIntRegister(GPIO_PORTC_BASE, buttonDown);     // Register our handler function for port
    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_FALLING_EDGE); // fall edge detect
    GPIOIntEnable(GPIO_PORTC_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOInput (GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6); // setup pin C4, C5, C6, C7 as input
    GPIOPadConfigSet(GPIO_PORTD_BASE,  GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    SysCtlDelay(3);
    GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6);
    GPIOIntClear(GPIO_PORTD_BASE,  GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6);
    GPIOIntRegister(GPIO_PORTD_BASE, buttonDown);     // Register our handler function for port
    GPIOIntTypeSet(GPIO_PORTD_BASE,  GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6, GPIO_FALLING_EDGE); // fall edge detect
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6);
}

void buttonDown()
{
    if (GPIOIntStatus(GPIO_PORTC_BASE,false)&GPIO_PIN_4) //motor 1 cw
        {
            GPIOIntRegister(GPIO_PORTC_BASE, buttonUp);
            GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);
            GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4);
        }
    if (GPIOIntStatus(GPIO_PORTC_BASE,false)&GPIO_PIN_5) // motor 1 ccw
        {
        GPIOIntRegister(GPIO_PORTC_BASE, buttonUp);
        GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_RISING_EDGE);
        GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_5);
        }
    if (GPIOIntStatus(GPIO_PORTC_BASE,false)&GPIO_PIN_6) // motor 2 cw
        {
        GPIOIntRegister(GPIO_PORTC_BASE, buttonUp);
        GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_RISING_EDGE);
        GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_6);
        }
    if (GPIOIntStatus(GPIO_PORTD_BASE,false)&GPIO_PIN_6) // motor 2 ccw
        {
        GPIOIntRegister(GPIO_PORTD_BASE, buttonUp);
        GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_RISING_EDGE);
        GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_6);
        }
    if (GPIOIntStatus(GPIO_PORTD_BASE,false)&GPIO_PIN_2) // mode select auto adjust
        {
        GPIOIntRegister(GPIO_PORTD_BASE, buttonUp);
        GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_RISING_EDGE);
        GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_2);
        }
    if (GPIOIntStatus(GPIO_PORTD_BASE,false)&GPIO_PIN_3) // mode select user control
        {
        GPIOIntRegister(GPIO_PORTD_BASE, buttonUp);
        GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_RISING_EDGE);
        GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_3);
        }

}
void buttonUp()
{
    if (GPIOIntStatus(GPIO_PORTC_BASE,false)&GPIO_PIN_4)
    {
        GPIOIntRegister(GPIO_PORTC_BASE, buttonDown);   // Register our handler function for port F
        GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);          // Configure PF4 for rising edge trigger
        GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4);
        button_pressed = 1;
    }
    if (GPIOIntStatus(GPIO_PORTC_BASE,false)&GPIO_PIN_5)
    {
        GPIOIntRegister(GPIO_PORTC_BASE, buttonDown);
        GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
        GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_5);
        button_pressed = 2;
    }
    if (GPIOIntStatus(GPIO_PORTC_BASE,false)&GPIO_PIN_6)
    {
        GPIOIntRegister(GPIO_PORTC_BASE, buttonDown);
        GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_FALLING_EDGE);
        GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_6);
        button_pressed = 3;
    }
    if (GPIOIntStatus(GPIO_PORTD_BASE,false)&GPIO_PIN_6)
    {
        GPIOIntRegister(GPIO_PORTD_BASE, buttonDown);
        GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_FALLING_EDGE);
        GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_6);
        button_pressed = 4;
    }
    if (GPIOIntStatus(GPIO_PORTD_BASE,false)&GPIO_PIN_2)
    {
        GPIOIntRegister(GPIO_PORTD_BASE, buttonDown);
        GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
        GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_2);
        button_pressed = 5;
    }
    if (GPIOIntStatus(GPIO_PORTD_BASE,false)&GPIO_PIN_3)
    {
        GPIOIntRegister(GPIO_PORTD_BASE, buttonDown);
        GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_FALLING_EDGE);
        GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_3);
        button_pressed = 6;
    }
}

void Button_step(dev_t* dev)
{
    switch(button_pressed)
    {
    case 0:
        break;
    case 1:
        // Reset axis lock
        dev->axis_lock = 0;
        button_pressed = 0;
        break;
    case 2:
        // Lock axis for manual control
        dev->axis_lock = 1;
        button_pressed = 0;
        break;
    case 3:
        // Pitch down
        dev->motor1_dir = 1;
        button_pressed = 0;
        break;
    case 4:
        // Pitch up
        dev->motor1_dir = 2;
        button_pressed = 0;
        break;
    case 5:
        // Roll left
        dev->motor2_dir = 1;
        button_pressed = 0;
        break;
    case 6:
        // Roll_right
        dev->motor2_dir = 2;
        button_pressed = 0;
        break;
    }
}

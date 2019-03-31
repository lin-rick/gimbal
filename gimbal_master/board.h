/*******************************************************************************
* Included headers
*******************************************************************************/

#ifndef BOARD_H_
#define BOARD_H_

#include "global_include.h"

/*******************************************************************************
* Function Definitions
*******************************************************************************/

void UART_init(void);
void button_init(void);
void buttonDown();
void buttonUp();
void Button_step(dev_t* dev);

#endif /* BOARD_H_ */

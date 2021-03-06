/*******************************************************************************
* Macros
*******************************************************************************/
#define MANUAL_STEP_SIZE   (10)
#define AUTO_STEP_SIZE     (1)
#define CLOCKWISE   (2)
#define C_CLOCKWISE (1)
#define THRESHOLD   (10)

/*******************************************************************************
* Included headers
*******************************************************************************/

#include "global_include.h"

/*******************************************************************************
* Function Definitions
*******************************************************************************/
void Motor_init();

void Motor_step(dev_t* dev);

void _driveMotor1(int step,int dir);

void _driveMotor2(int step,int dir);

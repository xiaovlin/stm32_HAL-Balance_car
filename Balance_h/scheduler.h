#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "stm32f1xx_hal.h"

#define NumOfTask 10       //优先组10

extern unsigned int Task_Delay[NumOfTask];  //优先组时间

void Task_main(void);


#endif

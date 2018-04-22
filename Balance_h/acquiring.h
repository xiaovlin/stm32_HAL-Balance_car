#ifndef __ACQUIRING_H_
#define __ACQUIRING_H_

#include "stm32f1xx_hal.h"

void initalize(void);

void Mpu6050_Date(float *f_angle, float *f_angle_dot, 
	                float *gr_z);

#endif

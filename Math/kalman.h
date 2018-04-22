#ifndef __KALMAN_H_
#define __KALMAN_H_

#include "stm32f1xx_hal.h"

void kalman_filter(float angle_m, float gyro_m, 
float *angle_f, float *angle_dot_f);

#endif

#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "stm32f1xx_hal.h"

void Protocol(void);
void ProtocolCpyData(void);
void UART_IRQHandle(void);

void balance(float *Angle_Cnt, float *Angle_dot);
void IncPIDCalc(void);
void T_balance(float *gyor_z);
void acquiring_int(void);
void IncPIDCalc_L(void);
void IncPIDCalc_R(void);
void velocity(void);
void Balancing_Loop(float *Angle_Cnt, float *Angle_dot1);


#endif


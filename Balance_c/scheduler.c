#include "scheduler.h"
#include "mpuiic.h"
#include "mpu6050.h"
#include "math.h" 
#include "tim.h"
#include "acquiring.h"
#include "control.h"
#include "ANO_DT.h"
#include "math.h" 
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "hc05.h"
#include "usart2.h"


unsigned int Task_Delay[NumOfTask];   //优先组

static float angle_data = 0, angle_dot_data = 0, Gr_z = 0; //函数变量


//函数声明 
void Task_5ms(void);
void Task_10ms(void);
void Task_20ms(void);
void Task_25ms(void);
void Task_40ms(void);
void Task_100ms(void);
void Task_200ms(void);


/**
  * 函数功能: 任务.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */

void Task_main(void)
{
	Task_5ms();               //执行的周期是5ms
	Task_10ms();              //执行的周期是10ms
	Task_20ms();              //执行的周期是20ms
    Task_25ms();              //执行的周期是25ms
	Task_40ms();              //执行的周期是40ms
	Task_100ms();             //执行的周期是100ms
	Task_200ms();             //执行的周期是200ms
}


void Task_5ms(void)
{
	if (Task_Delay[0] == 0)
	{
		Balancing_Loop(&angle_data, &angle_dot_data);  
		
		Task_Delay[0] = 5;             //此值每1ms会减1，减到0才可以重新进来这里，所以执行的周期是5ms
	}
}


void Task_10ms(void)
{
	if (Task_Delay[1] == 0)
	{	
		Mpu6050_Date(&angle_data, &angle_dot_data, &Gr_z);
		
		balance(&angle_data, &angle_dot_data);
		
		Task_Delay[1] = 10;           //此值每1ms会减1，减到0才可以重新进来这里，所以执行的周期是10ms
	}
}


void Task_20ms(void)
{
	if (Task_Delay[2] == 0)
	{
//		T_balance(&Gr_z);
		Task_Delay[2] = 20;          //此值每1ms会减1，减到0才可以重新进来这里，所以执行的周期是20ms
	}
}


void Task_25ms(void)
{
	if (Task_Delay[3] == 0)
	{
		velocity();
		
		Task_Delay[3] = 25;          //此值每1ms会减1，减到0才可以重新进来这里，所以执行的周期是25ms
	}
}


void Task_40ms(void)
{
	if (Task_Delay[5] == 0)
	{
		acquiring_int();
		
		Task_Delay[5] = 40;         //此值每1ms会减1，减到0才可以重新进来这里，所以执行的周期是40ms
	}
}


void Task_100ms(void)
{
	if (Task_Delay[6] == 0)
	{
		Protocol();
		
		ProtocolCpyData();

		Task_Delay[6] = 100;        //此值每1ms会减1，减到0才可以重新进来这里，所以执行的周期是100ms
	}
}


void Task_200ms(void)
{
	if (Task_Delay[7] == 0)
	{
		//		sprintf(sendData, "{B%d:%d:%d:%d:%d}$",(int16_t)angle_data*100, L_cnt1, R_cnt1, (int16_t)angle_dot_data* 100, Turn);
		//HC05_SendString(sendData);
		Task_Delay[7] = 200;                  //此值每1ms会减1，减到0才可以重新进来这里，所以执行的周期是200ms
	}
}

#include "control.h"
#include "tim.h"
#include "ANO_DT.h"
#include "usart2.h"
#include "hc05.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>


#define FILTER_COUNT     20

#define FPosition_MAX    5000
#define FPosition_MIN   -5000             //积分上限设限

#define FPWM_MAX         900
#define FPWM_MIN        -900              //速度上限设限

#define Amplitude_MAX    55              //加速度上限设限
#define Amplitude_MIN   -55


#define UART_BUFF_SIZE   80


__IO double fPosition = 0;														//位移
__IO float Velocity;
__IO float balance1;

__IO float iIncpid = 0;
__IO float iIncpid_L = 0, iIncpid_R = 0;
__IO float Target_speed_L = 0, Target_speed_R = 0;

__IO float G_Turn = 0;
__IO uint8_t control = 0;

volatile float fTarget_Speed = 0;										    //速度调节量
volatile float fTarget_Turn = 0;											//旋转调节量


char piddisplay[50] = "{";


__IO float Ap1 = 0, Ad1 = 0;
__IO float Ap2 = 0, Ad2 = 0;
__IO float Ap3 = 0, Ad3 = 0;
__IO float Sp = 0, Si = 0;
__IO float gr = 0;

int g_newcarstate = 0;            

__IO uint16_t num = 0;
__IO uint8_t startBit = 0;
__IO int int9num = 0;
__IO uint8_t Flag_angle = 1;

int16_t cnt_L, cnt_R;

uint8_t inputString[UART_BUFF_SIZE] = {0};
uint8_t ProtocolString[UART_BUFF_SIZE] = {0};


/**
  * 函数功能: 角度环控制函数
  * 输入参数: mpu6050的加速度Y轴和陀螺仪x轴
  * 返 回 值: 无
  * 说    明：无
  */
void balance(float *Angle_Cnt, float *Angle_dot)
{
	const float kp = 96.38f, kd = 6.16f;
	const float Bias = 0.0;
	
	balance1 = kp * (*Angle_Cnt - Bias) - *Angle_dot * kd;
	
//	if (Flag_angle != 0)        
//	{
//		
//	}
//	else
//	{
//		if (3 + gr  > fabsf(*Angle_Cnt) && 0.0 <= fabsf(*Angle_Cnt))         //滑动pid
//		{
//			balance1 = Ap1 * (*Angle_Cnt - Bias) - *Angle_dot * Ad1;
//		}
//		if (9 + gr > fabsf(*Angle_Cnt) && 3 + gr <= fabsf(*Angle_Cnt))
//		{
//			balance1 = Ap2 * (*Angle_Cnt - Bias) - *Angle_dot * Ad2;
//		}
//		if ( 45 > fabsf(*Angle_Cnt) && 9 + gr <= fabsf(*Angle_Cnt))
//		{
//			balance1 = Ap3 * (*Angle_Cnt - Bias) - *Angle_dot * Ad3;
//		}
//	}
	//		ANO_DT_Send_Senser(balance1, 0, 0, *Angle_dot, 0, 0, 0, 0, 0, 0);   //给电脑上位机发送数据
//		ANO_DT_Send_Status(0, *Angle_Cnt, 0, *Angle_dot, 0, 0);
}


/**
  * 函数功能: 采集电机速度脉冲
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void acquiring_int(void)
{
	cnt_L += TIM2->CNT;     //脉冲值叠加
	cnt_R += -TIM3->CNT;
	
	TIM3->CNT = 0;
	TIM2->CNT = 0;
}


/**
  * 函数功能: 作为参考
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：增量控制
  */
void IncPIDCalc(void) 
{
	const float Kp = 0, Ki = 0, Kd = 0;
	float PError = 0, DError = 0, IError = 0;
	float fSpeed_Vechile = 0;												//小车整体速度
  static float fSpeed_Vechile_F = 0;
	static float LastError = 0, PrevError = 0;
	
	fSpeed_Vechile = cnt_R - cnt_L;
	
	fSpeed_Vechile_F = fSpeed_Vechile_F * 0.8 + fSpeed_Vechile * 0.2;
	
	PError = Kp * (fSpeed_Vechile_F - LastError);
	DError = Kd * (fSpeed_Vechile_F - 2 * LastError + PrevError);
	IError = Ki * fSpeed_Vechile_F;
	LastError = fSpeed_Vechile_F;
	PrevError = LastError;
	
//	ANO_DT_Send_Senser(fSpeed_Vechile, 0, 0, 0, 0, 0, 0, 0, 0, 0);  //给电脑上位机发送数据
	iIncpid = PError + DError + IError;
}


/**
  * 函数功能: 作为参考
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：增量控制
  */
void IncPIDCalc_L(void) 
{
	const float Kp = 0, Ki = 0, Kd = 0;
	float PError = 0, DError = 0, IError = 0;
	float fSpeed_Vechile = 0;												//小车整体速度
  static float fSpeed_Vechile_F = 0;
	static float LastError = 0, PrevError = 0;
	fSpeed_Vechile = Target_speed_R - fSpeed_Vechile_F;
	
	fSpeed_Vechile_F = fSpeed_Vechile_F * 0.8 + fSpeed_Vechile * 0.2;

	PError = Kp * (fSpeed_Vechile_F - LastError);
	DError = Kd * (fSpeed_Vechile_F - 2 * LastError + PrevError);
	IError = Ki * fSpeed_Vechile_F;
	LastError = fSpeed_Vechile_F;
	PrevError = LastError;
	
	iIncpid_L = PError + DError + IError;
}


/**
  * 函数功能: 作为参考
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：增量控制
  */
void IncPIDCalc_R(void) 
{
	const float Kp = 0, Ki = 0, Kd = 0;
	float PError = 0, DError = 0, IError = 0;
	float fSpeed_Vechile = 0;												//小车整体速度
  static float fSpeed_Vechile_F = 0;
	static float LastError = 0, PrevError = 0;
	
	fSpeed_Vechile = Target_speed_L - fSpeed_Vechile_F;
	fSpeed_Vechile_F = fSpeed_Vechile_F * 0.8 + fSpeed_Vechile * 0.2;
	Target_speed_R = fSpeed_Vechile_F;
	
	PError = Kp * (fSpeed_Vechile_F - LastError);
	DError = Kd * (fSpeed_Vechile_F - 2 * LastError + PrevError);
	IError = Ki * fSpeed_Vechile_F;
	
	LastError = fSpeed_Vechile_F;
	PrevError = LastError;
	
	//	ANO_DT_Send_Senser(Error, 0, 0, 0, 0, 0, 0, 0, 0, 0);        //给电脑上位机发送数据
	iIncpid_R = PError + DError + IError;
}


/**
  * 函数功能: 作为参考
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void T_balance(float *gyor_z)
{
	static float Turn_Target = 0;
	float Turn_Amplitude = 900;
	static float G_turn = 0;
	
	G_turn = G_turn * 0.7 + *gyor_z * 0.3;
//	ANO_DT_Send_Senser(G_turn, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	
	if (control == 2)
	{
		control = 0;
		G_Turn = -8.5 * G_turn;
	}
		
	if (control == 1)
	{
		control = 0;
		
		Turn_Target += fTarget_Turn;
		if (Turn_Amplitude < Turn_Target) 
		{
			Turn_Target = Turn_Amplitude;
		}			
			  
	  if (-Turn_Amplitude > Turn_Target) 
		{
			Turn_Target = -Turn_Amplitude;
		}
		
		G_Turn = 0.65 * G_turn;
		G_Turn += Turn_Target;
		fTarget_Turn = 0;
	}
}


/**
  * 函数功能: 速度环PID控制
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：速度环PID控制	 记住，
  *速度反馈是正反馈，就是小车快的时候要
  *慢下来就需要再跑快一点
  */
void velocity(void)
{
	const float kp = 27.93f, ki = 0.313f;//0.265f; // kp * 1/75 = ki   23.50 * 1/75 = 0.313  
	float fSpeed_Vechile = 0;												
  static float fSpeed_Vechile_F = 0;											
	
	fSpeed_Vechile = (cnt_L + cnt_R) * 0.5f;
	cnt_L = cnt_R = 0;
	
	fSpeed_Vechile_F = (fSpeed_Vechile_F * 0.8 + fSpeed_Vechile * 0.2);
	fPosition += fTarget_Speed;                 //融合蓝牙给定速度
	fPosition += fSpeed_Vechile_F * ki;         //路程即速度积分
	
	/*位移限制,上下限待调节*/
	if (FPosition_MAX < fPosition)
	{															
		fPosition = FPosition_MAX;
	}
	else if (FPosition_MIN > fPosition)
	{
		fPosition = FPosition_MIN;
	}
	
	Velocity = kp * fSpeed_Vechile_F + fPosition;
	fTarget_Speed = 0;                           //全局变量 注意及时清零	
}


/**
  * 函数功能: 转向环控制
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：计算左右轮电机最终PWM
  */
void Balancing_Loop(float *Angle_Cnt, float *Angle_dot1)
{												
	float PWM = 0;
  float fPWM_L, fPWM_R;
	
	if (Amplitude_MAX > *Angle_Cnt && Amplitude_MIN < *Angle_Cnt)
	{
		PWM = balance1 - Velocity; 	   //PWM控制融合平衡角度、速度输出
		fPWM_L = PWM + fTarget_Turn;
		fPWM_R = PWM - fTarget_Turn;     

		if (FPWM_MAX < fPWM_L) 
		{
			fPWM_L = FPWM_MAX;
		}
		
		if (FPWM_MIN > fPWM_L) 
		{
			fPWM_L = FPWM_MIN;
		}
		
		if (FPWM_MAX < fPWM_R) 
		{	
		  fPWM_R = FPWM_MAX;
		}
		
		if (FPWM_MIN > fPWM_R) 
		{
			fPWM_R = FPWM_MIN;
		}
		
		L298N_DCMOTOR_Contrl(fPWM_L, fPWM_R);
	}
	else 
	{	
		L298N_DCMOTOR_Contrl(0, 0);
		
		fPosition = 0;
	}
	
}


/**
  * 函数功能: 接收蓝牙数据
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：蓝牙调试pid和控制平衡小车转向函数
  */
void UART_IRQHandle(void)
{
  __IO uint8_t uartvalue = 0;
	
	if (USART2->SR & (1 << 5))
	{
		uartvalue = USART2->DR;
		
		if (uartvalue == '{')
		{
			startBit = 1;
			num = 0;
		}
		
		if (startBit == 1) 
		{
			inputString[num] = uartvalue;
		}

		if (startBit == 1 && uartvalue == '}')
		{
			startBit = 0;
			USART2->SR = 0;
			
			int9num = num;
		}
		
		if (uartvalue == 'A')
		{
			control = 1;
			fTarget_Speed = -110;
			fTarget_Turn = 0;
		}
			
		if (uartvalue == 'E')
		{
			control = 1; 
			fTarget_Speed = 110;
			fTarget_Turn = 0;
		}
		
		if (uartvalue == 'C' || uartvalue == 'B')  
		{
			control = 1;
			fTarget_Speed = -60;
			fTarget_Turn  = -150;
		}
		
		if (uartvalue == 'G' || uartvalue == 'H')
		{
			control = 1;
			fTarget_Speed = -60;
			fTarget_Turn  = 150;
		}
		
		if (uartvalue == 'D')  
		{
			control = 1;
			fTarget_Speed = 60;
			fTarget_Turn  = 150;
		}
		
		if (uartvalue == 'F')
		{
			control = 1;
			fTarget_Speed = 60;
			fTarget_Turn  = -150;
		}
		
		num++;
		
		if (num >= 80)
		{
			num = 0;
			startBit = 0;
			USART2->SR = 0;
		}
	}
	
	USART2->SR = 0;
}


/**
  * 函数功能: 计算上报给蓝牙的数据
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：暂时不用
  */
void ProtocolGetPID(void)
{
	char charkp1[10] = {0},charkd1[10] = {0};
	char charkp2[10] = {0},charkd2[10] = {0};
	char charkp3[10] = {0},charkd3[10] = {0};
	char charksp[10] = {0},charksi[10] = {0};

	if (Ap1 >= 0 && Ap1 <= 100)
	{
		sprintf(charkp1, "%d", (int16_t)Ap1 * 100);
	}
	
	if (Ad1 >= 0 && Ad1 <= 100)
	{
		sprintf(charkd1, "%d", (int16_t)Ad1 * 100);
	}
	
	if (Ap2 >= 0 && Ap2 <= 100)
	{
		sprintf(charkp2, "%d", (int16_t)Ap2 * 100);
		
	}
	
	if (Ad2 >= 0 && Ad2 <= 100)
	{
		sprintf(charkd2, "%d", (int16_t)Ad2 * 100);
	}
	
	if (Ap3 >= 0 && Ap3 <= 100)
	{
		sprintf(charkp3, "%d", (int16_t)Ap3 * 100);
	}
	
	if (Ad3 >= 0 && Ad3 <= 100)
	{
		sprintf(charkd3, "%d", (int16_t)Ad3 * 100);
	}
	
	if (Sp >= 0 && Sp <= 100)
	{
		sprintf(charksp, "%d", (int16_t)Sp * 100);
	}
	
	if (Si >= 0 && Si <= 100)
	{
		sprintf(charksi, "%d", (int16_t)Si * 100);
	}
	
	strcat(piddisplay, "C");
	strcat(piddisplay, charkp1);
	strcat(piddisplay, ":");
	strcat(piddisplay, charkd1);
	strcat(piddisplay, ":");
	strcat(piddisplay, charkp2);
	strcat(piddisplay, ":");
	strcat(piddisplay, charkd2);
	strcat(piddisplay, ":");
	strcat(piddisplay, charkp3);
	strcat(piddisplay, ":");
	strcat(piddisplay, charkd3);
	strcat(piddisplay, ":");
	strcat(piddisplay, charksp);
	strcat(piddisplay, ":");
	strcat(piddisplay, "89");
	strcat(piddisplay, "}$");
	HC05_SendString(piddisplay);
}


/**
  * 函数功能: 作为参考
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int StringFind(const char *pSrc, const char *pDst)  
{  
    int i, j;  
	
    for (i = 0; pSrc[i] != '\0'; i++)  
    {  
        if (pSrc[i] != pDst[0])
				{					
            continue; 
				}				
				
        j = 0;
				
        while (pDst[j] != '\0' && pSrc[i+j] != '\0')  
        {  
            j++;  
					
            if (pDst[j] != pSrc[i+j]) 
						{							
              break;
						}							
        }
				
        if (pDst[j] == '\0')
				{
					return i; 
				}					  
    }  
		
    return -1;  
}  


/**
  * 函数功能: 蓝牙接收数据进行保存
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void ProtocolCpyData(void)
{
	memcpy(ProtocolString, inputString, num + 1);
	memset(inputString, 0x00, sizeof(inputString));
}


/**
  * 函数功能: 蓝牙调试pid
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void Protocol(void)
{
	char apad[20] = {0};
	static uint8_t i = 0;
	
	if (ProtocolString[3] == 'P')
	{
//		memset(apad, 0x00, sizeof(apad));
//		memset(ProtocolString, 0x00, sizeof(ProtocolString)); 
//		ProtocolGetPID();	
	}
	
	if (ProtocolString[3] == 'W')
	{
		i++;
		
		if (i == 2)
		{
			Flag_angle = 1;
		}
		
		Flag_angle = 0;
	}
	
  if (ProtocolString[1] == '0')
	{
		memcpy(apad, ProtocolString + 3, int9num - 3);
		Ap1 = atof(apad) * 0.01;
		memset(ProtocolString, 0x00, sizeof(ProtocolString)); 
	}
	
	if (ProtocolString[1] == '1')
	{
		memcpy(apad, ProtocolString + 3, int9num - 3);
		Ad1 = atof(apad) * 0.01;
		memset(ProtocolString, 0x00, sizeof(ProtocolString)); 
	}
	
	if (ProtocolString[1] == '2')
	{
		memcpy(apad, ProtocolString + 3, int9num - 3);
		Ap2 = atof(apad) * 0.01;
		memset(ProtocolString, 0x00, sizeof(ProtocolString)); 
	}
	
	if (ProtocolString[1] == '3')
	{
		memcpy(apad, ProtocolString + 3, int9num - 3);
		Ad2 = atof(apad) * 0.01;
		memset(ProtocolString, 0x00, sizeof(ProtocolString)); 
	}
	
	if (ProtocolString[1] == '4')
	{
		memcpy(apad, ProtocolString + 3, int9num - 3);
		Ap3 = atof(apad) * 0.01;
		memset(ProtocolString, 0x00, sizeof(ProtocolString)); 
	}
	
	if (ProtocolString[1] == '5')
	{
		memcpy(apad, ProtocolString + 3, int9num - 3);
		Ad3 = atof(apad) * 0.01;
		memset(ProtocolString, 0x00, sizeof(ProtocolString)); 
	}
	
	if (ProtocolString[1] == '6')
	{
		memcpy(apad, ProtocolString + 3, int9num - 3);
		Sp = atof(apad) * 0.01;
		memset(ProtocolString, 0x00, sizeof(ProtocolString)); 
	}
	
	if (ProtocolString[1] == '7')
	{
		memcpy(apad, ProtocolString + 3, int9num - 3);
		Si = atof(apad) * 0.001;
		memset(ProtocolString, 0x00, sizeof(ProtocolString)); 
	}
	
	if (ProtocolString[1] == '8')
	{
		memcpy(apad, ProtocolString + 3, int9num - 3);
		gr = atof(apad) * 0.01;
		memset(ProtocolString, 0x00, sizeof(ProtocolString)); 
	}
}




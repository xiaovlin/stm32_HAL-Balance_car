#include "acquiring.h"
#include "tim.h"
#include "mpu6050.h"
#include "math.h"
#include "kalman.h"
#include "mpuiic.h"
#include "math.h" 
#include "usart2.h"
#include "hc05.h"


#define FILTER_COUNT  20

#define GX_OFFSET 0x01
#define AX_OFFSET 0x01
#define AY_OFFSET 0x01
#define AZ_OFFSET 0x01

#define	GYRO_SCALE 0.060975609f  //+-2000   //16.4'/LSB


int16_t gx, gy, gz, ax, ay, az;

uint8_t aRxBuffer;
BLTDev bltDevList;


/**
  * 函数功能: 作为参考
  * 输入参数: MPU6050的原始温度数据
  * 返 回 值: 无
  * 说    明：无
  */
void  MPU6050_newValues(int16_t ax1, int16_t ay1, 
                        int16_t az1, int16_t gx1, 
                        int16_t gy1, int16_t gz1)
{
	unsigned char i ;
	static int16_t  MPU6050_FIFO[6][11];
	int32_t sum = 0;

	for (i = 1; i < 10; i++)
	{	
		MPU6050_FIFO[0][i-1] = MPU6050_FIFO[0][i];
		MPU6050_FIFO[1][i-1] = MPU6050_FIFO[1][i];
		MPU6050_FIFO[2][i-1] = MPU6050_FIFO[2][i];
		MPU6050_FIFO[3][i-1] = MPU6050_FIFO[3][i];
		MPU6050_FIFO[4][i-1] = MPU6050_FIFO[4][i];
		MPU6050_FIFO[5][i-1] = MPU6050_FIFO[5][i];
	}
		ax = MPU6050_FIFO[0][9] = ax1;
		ay = MPU6050_FIFO[1][9] = ay1;
		az = MPU6050_FIFO[2][9] = az1;
		gx = MPU6050_FIFO[3][9] = gx1;
		gy = MPU6050_FIFO[4][9] = gy1;
		gz = MPU6050_FIFO[5][9] = gz1;

	sum = 0;
	
	for (i = 0; i < 10; i++)
	{	
		 sum += MPU6050_FIFO[0][i];
	}
	
	MPU6050_FIFO[0][10] = sum / 10;
	sum = 0;
	
	for (i = 0; i < 10; i++)
	{
		 sum += MPU6050_FIFO[1][i];
	}
	
	MPU6050_FIFO[1][10] = sum / 10;
	sum = 0;
	
	for (i = 0; i < 10; i++)
	{
		 sum += MPU6050_FIFO[2][i];
	}
	
	MPU6050_FIFO[2][10] = sum / 10;
	sum = 0;
	
	for (i = 0; i < 10; i++)
	{
		 sum += MPU6050_FIFO[3][i];
	}
	
	MPU6050_FIFO[3][10] = sum / 10;
	sum=0;
	
	for (i = 0; i < 10; i++)
	{
		 sum+=MPU6050_FIFO[4][i];
	}
	
	MPU6050_FIFO[4][10] = sum / 10;
	sum = 0;
	
	for (i = 0; i < 10; i++)
	{
		 sum += MPU6050_FIFO[5][i];
	}
	
	MPU6050_FIFO[5][10] = sum / 10;
}


/**
  * 函数功能: mpu6050的加速度Y轴和陀螺仪x轴融合并滤波
  * 输入参数: 获取mpu6050的加速度Y轴和陀螺仪x轴
  * 返 回 值: 无
  * 说    明：无
  */
void Mpu6050_Date(float *f_angle, float *f_angle_dot, float *gr_z)
{
	static float angle = 0, angle_dot = 0;
  static int16_t gx0, gy0, gz0, ax0 ,ay0, 
	az0, temperature;
	
	mpu6050_get_data(&gx0, &gy0, &gz0, 
	&ax0, &ay0, &az0, &temperature);
	
//	MPU6050_newValues(ax0, ay0, az0, gx0, gy0, gz0);
	
	gx0 -= GX_OFFSET;
	gz0 -= GX_OFFSET;
	ax0 -= AX_OFFSET; 
	ay0 -=	AY_OFFSET;
	az0 -= AZ_OFFSET;
	*gr_z = gz0 * 0.060975609f;
	                               //16.4'/LSB
	angle_dot = gx0 * GYRO_SCALE;  //+-2000  0.060975 */LSB   
	angle = atan(ay0 / sqrt(ax0 * ax0 + az0 * az0 ));
	 
	angle = angle * 57.2957795f;    //180/pi
	
	kalman_filter(angle, angle_dot, f_angle, f_angle_dot);     //滤波
}


/**
  * 函数功能: 初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void initalize(void)
{
	HC05_Init();                    //蓝牙初始化
	HAL_TIM_Base_Start(&htim1);     //pwm始动
	
	i2c_CfgGpio();                  //i2c初始化
	MPU6050_Init();                 //mpu6050初始化
	
	HAL_UART_Receive_IT(&husartx_HC05, &aRxBuffer, 1);
	
	L298N_DCMOTOR_Contrl(0, 0);                     //pwm初始化0
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);    //编码器始动
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
	
  while (MPU6050ReadID() == 0)                      //mpu6050检测
	{	
    printf("mpu6050 error!!");
		
		L298N_DCMOTOR_Contrl(0, 0);
  }
}



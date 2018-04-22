#ifndef __MPUIIC_H_
#define __MPUIIC_H_

#include "stm32f1xx_hal.h"

#define I2C_WR	        0		/* 写控制bit */
#define I2C_RD	        1		/* 读控制bit */

/* 定义I2C总线连接的GPIO端口, 用户只需要修改下面4行代码即可任意改变SCL和SDA的引脚 */
#define GPIO_PORT_I2C	    GPIOB			              /* GPIO端口 */
#define RCC_I2C_PORT() 	  __HAL_RCC_GPIOB_CLK_ENABLE()		/* GPIO端口时钟 */
#define I2C_SCL_PIN		    GPIO_PIN_10			        /* 连接到SCL时钟线的GPIO */
#define I2C_SDA_PIN		    GPIO_PIN_11			        /* 连接到SDA数据线的GPIO */

#define I2C_SCL_1()  GPIO_PORT_I2C->BSRR = I2C_SCL_PIN				     /* SCL = 1 */
#define I2C_SCL_0()  GPIO_PORT_I2C->BRR = I2C_SCL_PIN				       /* SCL = 0 */

#define I2C_SDA_1()  GPIO_PORT_I2C->BSRR = I2C_SDA_PIN				     /* SDA = 1 */
#define I2C_SDA_0()  GPIO_PORT_I2C->BRR = I2C_SDA_PIN				       /* SDA = 0 */
	
#define I2C_SDA_READ()  ((GPIO_PORT_I2C->IDR & I2C_SDA_PIN) != 0)	 /* 读SDA口线状态 */


void i2c_Start(void);
void i2c_Stop(void);
void i2c_Ack(void);
void i2c_NAck(void);

void i2c_SendByte(uint8_t _ucByte);
uint8_t i2c_ReadByte(uint8_t ack);
uint8_t i2c_WaitAck(void);

void i2c_CfgGpio(void);
uint8_t i2c_CheckDevice(uint8_t _Address);

#endif

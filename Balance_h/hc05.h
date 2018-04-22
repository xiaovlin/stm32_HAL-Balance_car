#ifndef __HC05_H_
#define __HC05_H_

#include "stm32f1xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/*蓝牙地址，数字形式，分NAP，UAP，LAP段*/			
#define BLTDEV_MAX_NUM         5

typedef  struct 
{
	uint16_t  NAP;
	uint8_t 	UAP;
	uint32_t  LAP;
}BLTAddr;


typedef  struct 
{
	uint8_t num;		//扫描到的蓝牙设备数量	

	BLTAddr addr[BLTDEV_MAX_NUM];	//蓝牙设备地址，数字形式
	char unpraseAddr[BLTDEV_MAX_NUM][50];	//蓝牙设备地址，字符串形式，方便扫描时和连接时使用
	char name[BLTDEV_MAX_NUM][50];	//蓝牙设备的名字
}BLTDev;


enum
{
  HC05_DEFAULT_TIMEOUT = 200,
  HC05_INQUIRY_DEFAULT_TIMEOUT = 10000,
  HC05_PAIRING_DEFAULT_TIMEOUT = 10000,
  HC05_PASSWORD_MAXLEN = 16,
  HC05_PASSWORD_BUFSIZE = HC05_PASSWORD_MAXLEN + 1,
  HC05_NAME_MAXLEN = 32,
  HC05_NAME_BUFSIZE = HC05_NAME_MAXLEN + 1,
  HC05_ADDRESS_MAXLEN = 14,
  HC05_ADDRESS_BUFSIZE = HC05_ADDRESS_MAXLEN + 1,
};

		
/* 宏定义 --------------------------------------------------------------------*/
#define HC05_USART          	          USART2

#define HC05_EN_GPIO_CLK() 	            __HAL_RCC_GPIOF_CLK_ENABLE()		/* GPIO端口时钟 */
#define HC05_EN_GPIO_PORT    	          GPIOF			              /* GPIO端口 */
#define HC05_EN_GPIO_PIN		            GPIO_PIN_11		          /* 连接到HC05 EN引脚的GPIO */
#define HC05_EN_HIGHT()		              HAL_GPIO_WritePin(HC05_EN_GPIO_PORT, HC05_EN_GPIO_PIN, GPIO_PIN_SET);	
#define HC05_EN_LOW()				            HAL_GPIO_WritePin(HC05_EN_GPIO_PORT, HC05_EN_GPIO_PIN, GPIO_PIN_RESET);	

/*信息输出*/
#define HC05_DEBUG_ON                   1
#define HC05_DEBUG_FUNC_ON              0

#define HC05_INFO(fmt,arg...)           printf("<<-HC05-INFO->> "fmt"\n", ##arg)
#define HC05_ERROR(fmt,arg...)          printf("<<-HC05-ERROR->> "fmt"\n", ##arg)
#define HC05_DEBUG(fmt,arg...)          do{\
                                          if(HC05_DEBUG_ON)\
                                          printf("<<-HC05-DEBUG->> [%d]"fmt"\n", __LINE__, ##arg);\
                                          }while(0)

#define HC05_DEBUG_FUNC()               do{\
                                         if(HC05_DEBUG_FUNC_ON)\
                                         printf("<<-HC05-FUNC->> Func:%s@Line:%d\n", __func__, __LINE__);\
                                       }while(0)  


														 
/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/																	 
void HC05_Init(void);
uint8_t HC05_Send_CMD(char *cmd, uint8_t clean);
void HC05_SendString(char *str);																			 
void strBLTAddr(BLTDev *bltDev, char delimiter);
uint8_t getRemoteDeviceName(BLTDev *bltDev);
void printBLTInfo(BLTDev *bltDev);
uint8_t linkHC05(void);
int get_line(char *line, char *stream, int max_size);


#endif

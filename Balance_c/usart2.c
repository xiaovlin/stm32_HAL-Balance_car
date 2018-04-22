#include "usart2.h"
#include "control.h"
#include <stdarg.h>

//中断缓存串口数据
#define UART_BUFF_SIZE      1024
__IO  uint16_t uart_p = 0;
uint8_t   uart_buff[UART_BUFF_SIZE];

UART_HandleTypeDef husartx_HC05;
static __IO uint32_t TimingDelay = 0;

/* 扩展变量 ------------------------------------------------------------------*/
extern uint8_t aRxBuffer;

/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: HC05通信功能引脚GPIO初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void HC05_GPIO_Init(void)
{
   GPIO_InitTypeDef GPIO_InitStruct;

    /* 串口外设时钟使能 */
    HC05_USART_RCC_CLK_ENABLE();
  
    /* 串口外设功能GPIO配置 */
    GPIO_InitStruct.Pin = HC05_USARTx_Tx_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(HC05_USARTx_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = HC05_USARTx_Rx_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(HC05_USARTx_PORT, &GPIO_InitStruct);
}


/**
  * 函数功能: 串口参数配置.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void HC05_USARTx_Init(void)
{ 
  /* HC05通信功能引脚GPIO初始化 */
  HC05_GPIO_Init();
  __HAL_RCC_USART2_CLK_ENABLE();
  husartx_HC05.Instance = HC05_USARTx;
  husartx_HC05.Init.BaudRate = HC05_USARTx_BAUDRATE;
  husartx_HC05.Init.WordLength = UART_WORDLENGTH_8B;
  husartx_HC05.Init.StopBits = UART_STOPBITS_1;
  husartx_HC05.Init.Parity = UART_PARITY_NONE;
  husartx_HC05.Init.Mode = UART_MODE_TX_RX;
  husartx_HC05.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husartx_HC05.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&husartx_HC05);
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}


/**
  * 函数功能: 串口发送一个字节数据 
  * 输入参数: ch：待发送字符
  * 返 回 值: 无
  * 说    明：无
  */
void Usart_SendByte(uint8_t ch)
{
  	while (__HAL_UART_GET_FLAG(&husartx_HC05, UART_FLAG_TXE) == 0)  //循环发送,直到发送完毕
		{
		}			
		
	/* 发送一个字节数据到USART2 */
	HAL_UART_Transmit(&husartx_HC05, (uint8_t*)&ch, 1, 0xffff);
		
}


/**
  * 函数功能: 串口发送指定长度的字符串
  * 输入参数: str：待发送字符串缓冲器
  *           strlen:指定字符串长度
  * 返 回 值: 无
  * 说    明：无
  */
void Usart_SendStr_length(uint8_t *str, uint32_t strlen )
{
	unsigned int k = 0;
	
	do 
	{
			Usart_SendByte(*(str + k));
		
			k++;
	} while (k < strlen);
}


/**
  * 函数功能: 串口发送字符串，直到遇到字符串结束符
  * 输入参数: str：待发送字符串缓冲器
  * 返 回 值: 无
  * 说    明：无
  */
void Usart_SendString(uint8_t *str)
{
	unsigned int k = 0;
	
	do 
	{
			Usart_SendByte(*(str + k) );
			k++;
	} while (*(str + k) != '\0');
}


/**
  * 函数功能: 接收中断回调函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  if(uart_p < UART_BUFF_SIZE)
  {
    uart_buff[uart_p] = aRxBuffer; 
		
    uart_p++; 
		
    HAL_UART_Receive_IT(&husartx_HC05, &aRxBuffer, 1);
  }
  else
  {
    clean_rebuff();       
  }
}


/**
  * 函数功能: 获取接收到的数据和长度 
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
char *get_rebuff(uint16_t *len) 
{
    *len = uart_p;
	
    return (char*)&uart_buff;
}


/**
  * 函数功能: 清空缓冲区
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void clean_rebuff(void)
{
  uint16_t i = UART_BUFF_SIZE + 1;
  
  uart_p = 0;
	
	while (i)
	{
		uart_buff[--i] = 0;
	}
}



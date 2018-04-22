#ifndef  __USART2_H_
#define  __USART2_H_

#include "stm32f1xx_hal.h"


#define HC05_USARTx                                 USART2
#define HC05_USARTx_BAUDRATE                        9600
#define HC05_USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART2_CLK_ENABLE()
#define HC05_USART_RCC_CLK_DISABLE()                __HAL_RCC_USART2_CLK_DISABLE()

#define HC05_USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
#define HC05_USARTx_PORT                            GPIOA
#define HC05_USARTx_Tx_PIN                          GPIO_PIN_2
#define HC05_USARTx_Rx_PIN                          GPIO_PIN_3

/* 扩展变量 ------------------------------------------------------------------*/
extern UART_HandleTypeDef husartx_HC05;

/* 函数声明 ------------------------------------------------------------------*/
void HC05_USARTx_Init(void);
char *get_rebuff(uint16_t *len);
void clean_rebuff(void);
void Usart_SendString(uint8_t *str);
void Usart_SendByte(uint8_t ch);
void Usart_SendStr_length(uint8_t *str, uint32_t strlen);



#endif


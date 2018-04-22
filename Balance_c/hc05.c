#include "hc05.h"
#include "usart2.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>


/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
extern  BLTDev bltDevList;   //蓝牙设备列表，在main文件中定义

/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/


/**
  * 函数功能: 向HC05模块发送命令并检查OK。只适用于具有OK应答的命令
  * 输入参数: cmd：待发送命令
  *           clean：1：清除接收缓冲区内容
  *                  0：保留接收缓冲区内容
  * 返 回 值: 命令应答状态：1：无OK应答
  *                         0：成功发送并接收到OK应答
  * 说    明：无
  */
uint8_t HC05_Send_CMD(char *cmd, uint8_t clean)
{	 		 
	uint8_t retry = 5;
	uint8_t i;
	
	while (retry--)
	{
		//HAL_UART_Transmit(&husartx_rs485,(uint8_t *)cmd,strlen(cmd),1000);
		Usart_SendString((uint8_t *)cmd);
		
    for (i = 0; i < 20; i++)
    { 
      uint16_t len;
      char *redata;
      
      HAL_Delay(10);
      
      redata = get_rebuff(&len);
			
      if (len > 0)
      {
        if (redata[0] != 0)
        {
          HC05_DEBUG("send CMD: %s", cmd);
          HC05_DEBUG("receive %s", redata);
        }
				
        if (strstr(redata, "OK"))				
        {          
          if(clean == 1)
					{
						clean_rebuff();
					}
					
          return 0;
        }
      }
      else
      {					
        HAL_Delay(100);
      }		
    }
		
    HC05_DEBUG("HC05 send CMD fail %d times",retry);
  }	
	
	HC05_DEBUG("HC05 send CMD fail ");
	
	if (clean == 1)
	{
		clean_rebuff();
	}
	
	return 1 ;
}


/**
  * 函数功能: 使用HC05透传字符串数据
  * 输入参数: str,要传输的字符串
  * 返 回 值: 无
  * 说    明：无
  */
void HC05_SendString(char *str)
{
	Usart_SendString((uint8_t*)str);
//HAL_UART_Transmit(&husartx_rs485,(uint8_t *)str,strlen(str),1000);
}


/**
  * 函数功能: 初始化GPIO及检测HC05模块
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void HC05_Init(void)
{
	uint8_t i;
	
	HC05_USARTx_Init();	
	
	for (i = 0; i < BLTDEV_MAX_NUM; i++)
	{
		sprintf(bltDevList.unpraseAddr[i], " ");
		sprintf(bltDevList.name[i], " ");
	}	
	
	bltDevList.num = 0;
}

/**
  * 函数功能: 把接收到的字符串转化成16进制形式的数字变量(主要用于转化蓝牙地址)
  * 输入参数: str：待转换字符串
  * 返 回 值: 无
  * 说    明：无
  */
unsigned long htoul(const char *str)
{
  long result = 0;

  if (!str)
	{
		return 0;
	}

  while (*str)
  {
    uint8_t value;

    if (*str >= 'a' && *str <= 'f')
		{
			value = (*str - 'a') + 10;
		}
    else if (*str >= 'A' && *str <= 'F')
		{
			value = (*str - 'A') + 10;
		}
    else if (*str >= '0' && *str <= '9')
		{
			value = *str - '0';
		}
    else
		{
			break;
		}
     
    result = (result * 16) + value;
		
    ++str;
  }
	
  return result;
}


/**
  * 函数功能: 在str中，跳过它前面的prefix字符串,
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
char *skipPrefix(char *str, size_t str_length, const char *prefix)
{
  uint16_t prefix_length = strlen(prefix);
	
  if (!str || str_length == 0 || !prefix)
	{
    return 0;
	}
	
  if (str_length >= prefix_length && strncmp(str, prefix, prefix_length) == 0)
	{
		return str + prefix_length;
	}
    
  return 0;
}


/**
  * 函数功能: 从stream中获取一行字符串到line中
  * 输入参数: line,存储获得行的字符串数组
  *           stream，原字符串数据流       max_size，stream的大小   
  * 返 回 值: line的长度，若stream中没有‘\0’，'\r'，'\n'，则返回0
  * 说    明：无
  */
int get_line(char *line, char *stream, int max_size)  
{  
  char *p;	
  int len = 0;  
	
  p = stream;
	
  while ( *p != '\0' && len < max_size )
  {  
    line[len++] = *p;  
    p++;
		
    if ('\n' == *p || '\r'==*p)
		{
			break; 
		}			
  }
	
  if (*p != '\0' && *p != '\n' && *p != '\r')
	{
		return 0;
	}
    
  line[len] = '\0';  
	
  return len;  
} 



/**
  * 函数功能: 向HC05写入命令，不检查模块的响应
  * 输入参数: arg，命令参数，为0时不带参数，若command也为0时，发送"AT"命令
  * 返 回 值: 无
  * 说    明：无
  */
void writeCommand(const char *command, const char *arg)
{
  char str_buf[50];

  if (arg && arg[0] != 0)
	{
		sprintf(str_buf, "AT+%s%s\r\n", command, arg);
	}
    
  else if (command && command[0] != 0)
  {
    sprintf(str_buf, "AT+%s\r\n", command);
  }
  else
	{
		sprintf(str_buf, "AT\r\n");
	}		
  
  HC05_DEBUG("CMD send:%s", str_buf);
  Usart_SendString((uint8_t*)str_buf);
 // HAL_UART_Transmit(&husartx_rs485,(uint8_t *)str_buf,strlen(str_buf),1000);
}


/**
  * 函数功能: 扫描周边的蓝牙设备，并存储到设备列表中
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
uint8_t parseBluetoothAddress(BLTDev *bltDev)
{
  /* Address should look like "+ADDR:<NAP>:<UAP>:<LAP>",
   * where actual address will look like "1234:56:abcdef".
   */
	char *redata;
	uint16_t len;
	char linebuff[50];
	uint16_t linelen;
	uint16_t getlen = 0;
	uint8_t linenum = 0;	
	uint8_t i;
	char *p;

	HC05_Send_CMD("AT+INQ\r\n", 0);
	redata = get_rebuff(&len);
	
	if (redata[0] != 0 && strstr(redata, "+INQ:") != 0)
	{
		HC05_DEBUG("rebuf =%s", redata);

getNewLine:
		while (getlen < len - 2 * linenum )
		{	
			linelen = get_line(linebuff, redata + getlen + 2 * linenum, len);
			
			if (linelen > 50 && linelen != 0)
			{
				HC05_Send_CMD("AT+INQC\r\n", 1);//退出前中断查询
				
				return 1;
			}
			
			getlen += linelen;
			linenum++;			
			p = skipPrefix(linebuff, linelen, "+INQ:");
			
			if (p != 0)
			{
				uint8_t num;
				num = bltDev->num;
				strBLTAddr(bltDev, ':');
				for (i = 0; i <= num; i++)
				{
					if (strstr(linebuff, bltDev->unpraseAddr[i]) != NULL)	
					{
						goto getNewLine;	//!=null时，表示该地址与解码语句的地址相同
					}
				}							
				
				/*若蓝牙设备不在列表中，对地址进行解码*/	
				bltDev->addr[num].NAP = htoul(p);			
				p = strchr(p, ':');

				if (p == 0)
				{
					HC05_Send_CMD("AT+INQC\r\n", 1);//退出前中断查询
					
					return 1;
				}
				
				bltDev->addr[num].UAP = htoul(++p);
				p = strchr(p, ':');
				
				if (p == 0)
				{
					HC05_Send_CMD("AT+INQC\r\n", 1);//退出前中断查询
					
					return 1;
				}
				
				bltDev->addr[num].LAP = htoul(++p);
				/*存储蓝牙地址(字符串形式)*/
				sprintf(bltDev->unpraseAddr[num], "%X:%X:%X", bltDev->addr[num].NAP, 
				bltDev->addr[num].UAP, bltDev->addr[num].LAP);
				
				bltDev->num++;
			}
		}
		
		clean_rebuff();
		HC05_Send_CMD("AT+INQC\r\n", 1);//退出前中断查询
		
		return 0;
	}	
	else
	{
		clean_rebuff();
		
		HC05_Send_CMD("AT+INQC\r\n", 1);//退出前中断查询
		
		return 1;	
	}
}


/**
  * 函数功能: 把蓝牙地址转化成字符串形式
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void strBLTAddr(BLTDev *bltDev, char delimiter)  
{
	uint8_t i;
	
	if (bltDev->num == 0)
	{
		HC05_DEBUG("/*******No other BLT Device********/");
	}
	else
	{
		for (i = 0; i < bltDev->num; i++)
		{
			sprintf(bltDev->unpraseAddr[i], "%X%c%X%c%X", bltDev->addr[i].NAP,
			        delimiter, bltDev->addr[i].UAP, delimiter, bltDev->addr[i].LAP);
		}
	}
}


/**
  * 函数功能: 获取远程蓝牙设备的名称
  * 输入参数: bltDev ，蓝牙设备列表指针
  * 返 回 值: 0获取成功，非0不成功
  * 说    明：无
  */
uint8_t getRemoteDeviceName(BLTDev *bltDev)
{
	uint8_t i;
	char *redata;
	uint16_t len;
	
	char linebuff[50];
	uint16_t linelen;
	char *p;
	
	char cmdbuff[100];
	
	strBLTAddr(bltDev, ',');

	HC05_DEBUG("device num =%d", bltDev->num);
	
	for (i = 0; i < bltDev->num; i++)
	{
		sprintf(cmdbuff, "AT+RNAME?%s\r\n", bltDev->unpraseAddr[i]);
		HC05_Send_CMD(cmdbuff,0);
		redata = get_rebuff(&len);
		
		if (redata[0] != 0 && strstr(redata, "OK") != 0)
		{
			linelen = get_line(linebuff, redata, len);
			
			if (linelen > 50 && linelen !=0 ) 
			{
				linebuff[linelen] = '\0';	//超长截断
			}
				
			p = skipPrefix(linebuff, linelen, "+RNAME:");
			
			if (p != 0)
			{
				strcpy(bltDev->name[i], p);
			}
		}
		else
		{
			clean_rebuff();
			
			return 1;	
		}
		clean_rebuff();
	}
	
	return 0;
}


/**
  * 函数功能: 输出蓝牙设备列表
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void printBLTInfo(BLTDev *bltDev)  
{
	uint8_t i;
	
	if (bltDev->num == 0)
	{
		HC05_DEBUG("/*******No remote BLT Device or in SLAVE mode********/");
	}
	else
	{
		//扫描到 %d 个蓝牙设备
		
		HC05_DEBUG("Scan to %d bluetooth devices", bltDev->num);

		for (i = 0; i < bltDev->num; i++)
		{
			HC05_INFO("/*******Device[%d]********/", i);	
			HC05_INFO("Device Addr: %s", bltDev->unpraseAddr[i]);
			HC05_INFO("Device name: %s", bltDev->name[i]);
		}
	}
}


/**
  * 函数功能: 扫描蓝牙设备，并连接名称中含有"HC05"的设备
  * 输入参数: 无
  * 返 回 值: 0获取成功，非0不成功
  * 说    明：无
  */
uint8_t linkHC05(void)
{
	uint8_t i = 0;
	char cmdbuff[100];
	
	parseBluetoothAddress(&bltDevList);
	getRemoteDeviceName(&bltDevList);
	printBLTInfo(&bltDevList);
	
	for (i = 0; i <= bltDevList.num; i++)
	{
		if (strstr(bltDevList.name[i], "HC05") != NULL) //非NULL表示找到有名称部分为HC05的设备
		{
			//搜索到远程HC05模块，即将进行配对连接
			
			HC05_INFO("Search to the remote HC05 module, which is about to be matched...");
			
			strBLTAddr(&bltDevList, ',');		
			//配对
			sprintf(cmdbuff,"AT+PAIR=%s,20\r\n", bltDevList.unpraseAddr[i]);
			HC05_Send_CMD(cmdbuff, 0);
			//连接	
			sprintf(cmdbuff, "AT+LINK=%s\r\n", bltDevList.unpraseAddr[i]);
			
			return HC05_Send_CMD(cmdbuff, 0);		
		}
	}
	
	return 1;
}



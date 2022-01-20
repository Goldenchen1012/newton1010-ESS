/**
  ******************************************************************************
  * @file        HalSpiRom.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2022/01/06
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 SMP</center></h2>
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "define.h"
#include "main.h"
#include "LibSwTimer.h"
#include "ApiFu.h"
#include "HalEeprom.h"
#include "smp_MX25L_Driver.h"


void appSerialCanDavinciSendTextMessage(char *str);
#define	halSpiromDebugMsg(str)		appSerialCanDavinciSendTextMessage(str);

/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void halSpiromOpen(void)
{
	char	str[100];
	smp_mx25l_ID	mx25l_ID;
	smp_mx25l_flash_init();
	
	smp_mx25l_flash_read_ID(&mx25l_ID);
	sprintf(str,"SPIID = %.2X %.4X",mx25l_ID.Manufacture_ID, mx25l_ID.Device_ID);
	halSpiromDebugMsg(str);

}

/************************ (C) COPYRIGHT SMP *****END OF FILE****/    








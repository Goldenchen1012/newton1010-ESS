/**
  ******************************************************************************
  * @file        HalSpiSTM32L4.c
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/7
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"
#include "LibDebug.h"
#include "halspi.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
tErrCode HalSpiOpen(tHalSpi *pHalSpi)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	
	if(pHalSpi->PortNo == 1)
	{
		;
	}
	else 
		return RES_ERROR_INVALID_PARAM;
	return RES_SUCCESS;
}


tErrCode HalSpiClose(tHalSpi *pHalSpi)
{
	if(pHalSpi->PortNo == 1)
	{
		
	}
	else 
		return RES_ERROR_INVALID_PARAM;	
	return RES_SUCCESS;

}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



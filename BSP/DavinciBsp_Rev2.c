/**
  ******************************************************************************
  * @file        DavinciBsp_Rev2.c
  * @author      Johnny/Golden Chen
  * @version     v0.0.1
  * @date        2022/01/06
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021</center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "DavinciBsp_Rev2.h"
  
/* Private macro -------------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/

/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/

void HalBspSetGpio(GPIO_TypeDef  *GPIOx, uint16_t Pin, uint32_t mode,uint32_t pull, uint32_t speed)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = Pin;
	GPIO_InitStructure.Mode = mode;
	GPIO_InitStructure.Pull = pull;
	GPIO_InitStructure.Speed = speed;
	
	#if 0
	GPIO_InitStructure.Alternate = 0;
	#endif
	
	HAL_GPIO_Init(GPIOx, &GPIO_InitStructure); 

}



/************************ (C) COPYRIGHT *****END OF FILE****/    

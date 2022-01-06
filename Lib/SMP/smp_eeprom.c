/**
  ******************************************************************************
  * @file    smp_eeprom.c
  * @author  Randy Huang
  * @version V0.0.1
  * @date    23-December-2016
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 SMP</center></h2>
  *
  * 
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "smp_eeprom.h"
#include "smp_debug.h"
#include <stdbool.h>
#include "eeprom.h"
#include "stm32l4xx_hal.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int8_t smp_eeprom_open(void)
{
	bool init_state = false;
	   /* Unlock the Flash Program Erase controller */
	HAL_FLASH_Unlock();
	init_state = (EE_Init() == EE_OK) ? true : false;
	HAL_FLASH_Lock();
	
	return (init_state==true) ? SMP_SUCCESS : SMP_ERROR_NOT_FOUND;
}

int8_t smp_eeprom_write(uint16_t addr, uint16_t val)
{
	bool write_state = false;
	HAL_FLASH_Unlock();
	write_state = (EE_WriteVariable(addr,val)==HAL_OK) ? true : false;
	HAL_FLASH_Unlock();
	
	return (write_state==true) ? SMP_SUCCESS : SMP_ERROR_NOT_FOUND;
}

int8_t smp_eeprom_read(uint16_t addr, uint16_t *val)
{
	bool read_state = false;
	HAL_FLASH_Unlock();	
	read_state = (EE_ReadVariable(addr,val)==HAL_OK) ? true : false;	
	HAL_FLASH_Unlock();
	
	return (read_state==true) ? SMP_SUCCESS : SMP_ERROR_NOT_FOUND;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Simplo all right reserved *****END OF FILE****/

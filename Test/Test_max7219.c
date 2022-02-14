/**
  ******************************************************************************
  * @file    Test_max7219.c
  * @author  Golden Chen
  * @version V0.0.1
  * @date    2022/02/11
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 SMP</center></h2>
  *
  * 
  *
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "smp_max7219.h"
#include "SEGGER_RTT.h"
#include "RTT_Log.h"

void Test_MAX7219_LCD_Maxtrix_Func(void){
  maxInit(4,0);
	HAL_Delay(500);
	MAX7219_All_Display(0xFF);
	HAL_Delay(500);
	MAX7219_All_Display(0x00);
	HAL_Delay(500);
	MAX7219_Display_1();
	HAL_Delay(1000);
}
//--------FILE END------------------------------------------------------------------------------------------

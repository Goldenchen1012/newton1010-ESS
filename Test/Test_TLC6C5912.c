/**
  ******************************************************************************
  * @file    Test_TLC6C5912.c
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
#include "Bsp.h"
#include "stm32l4xx_hal.h"
#include "smp_TLC6C5912.h"
#include "SEGGER_RTT.h"
#include "RTT_Log.h"

LED_Display_Indicator_Type led_status_temp;
uint32_t test_led_display_cnt=0; 

void Test_TLC6C5912_FUNC(void){
	smp_TLC6C5912_Init();
	for(int i=0;i<10;i++){
	    if((i%2)==1)
	        smp_TLC6C5912_All_LED_Off();
	    else
				  smp_TLC6C5912_All_LED_On();
			HAL_Delay(50);
	}
	
	for(int i=0;i<10;i++){
		  led_status_temp.w = 0x0555;
      smp_TLC6C5912_Set_Ledstauts(led_status_temp);
		  smp_TLC6C5912_Display_Ledstauts(); 
			HAL_Delay(50);
	}	

	for(int i=0;i<10;i++){
		  led_status_temp.w = 0x0AAA;
      smp_TLC6C5912_Set_Ledstauts(led_status_temp);
		  smp_TLC6C5912_Display_Ledstauts(); 
			HAL_Delay(50);
	}	
	
	smp_TLC6C5912_All_LED_Off();
	
	for(int i=0;i<10;i++){
      smp_TLC6C5912_Percent20_LED(i%2);
			HAL_Delay(50);
	}	
	
	for(int i=0;i<10;i++){
      smp_TLC6C5912_Percent40_LED(i%2);
			HAL_Delay(50);
	}
	
	for(int i=0;i<10;i++){
      smp_TLC6C5912_Percent60_LED(i%2);
			HAL_Delay(50);
	}	
	
	for(int i=0;i<10;i++){
      smp_TLC6C5912_Percent80_LED(i%2);
			HAL_Delay(50);
	}	

	for(int i=0;i<10;i++){
      smp_TLC6C5912_Percent100_LED(i%2);
			HAL_Delay(50);
	}	
	
	for(int i=0;i<10;i++){
      smp_TLC6C5912_Discharge_LED(i%2);
			HAL_Delay(50);
	}		
	
	for(int i=0;i<10;i++){
      smp_TLC6C5912_Charge_LED(i%2);
			HAL_Delay(50);
	}
	
	for(int i=0;i<10;i++){
      smp_TLC6C5912_Alarm_LED(i%2);
			HAL_Delay(50);
	}

	for(int i=0;i<10;i++){
      smp_TLC6C5912_Comm_LED(i%2);
			HAL_Delay(50);
	}	
	
	smp_TLC6C5912_All_LED_Off();
	smp_TLC6C5912_Percent100_LED(LED_ON);
	smp_TLC6C5912_Comm_LED(LED_ON);
}

//--------FILE END------------------------------------------------------------------------------------------

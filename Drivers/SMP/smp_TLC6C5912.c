/**
  ******************************************************************************
  * @file    smp_TLC6C5912.c
  * @author  Golden Chen
  * @version V0.0.1
  * @date    2022/01/21
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

//defalut LED display  all = LED OFF 
LED_Display_Indicator_Type Led_display={LED_OFF};

void smp_TLC6C5912_Init(void){
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = BSP_TLC6C5912_SER_IN_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BSP_TLC6C5912_SER_IN_PORT, &GPIO_InitStructure); 
	
    GPIO_InitStructure.Pin = BSP_TLC6C5912_SRCK_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BSP_TLC6C5912_SRCK_PORT, &GPIO_InitStructure); 
		
    GPIO_InitStructure.Pin = BSP_TLC6C5912_RCK_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BSP_TLC6C5912_RCK_PORT, &GPIO_InitStructure); 	
}

LED_Display_Indicator_Type smp_TLC6C5912_Get_Ledstauts(void){
    return(Led_display);
}

void smp_TLC6C5912_Set_Ledstauts(LED_Display_Indicator_Type led_status){
    Led_display = led_status;
}

void smp_TLC6C5912_Display_Ledstauts(void){
    smp_TLC6C5912_SendData(Led_display.w);
}

void smp_TLC6C5912_small_delay(uint8_t nop_num){
  	for(int i= 0 ; i< nop_num ;i++){
		    __NOP();
		}
}

void smp_TLC6C5912_OutputLed_Open(void){
   BSP_TLC6C5912_CLK_0();                  // bring CLK low
   BSP_TLC6C5912_CS_1();
	
	#if 0
	 #ifdef TLC6C5912_CLK_DELAY_WITH_NOP
	 smp_TLC6C5912_small_delay(TLC6C5912_CLK_DELAY_NOP);
	 #endif 
	 
	 BSP_TLC6C5912_CLK_1();	 
	 
	 #ifdef TLC6C5912_CLK_DELAY_WITH_NOP
	 smp_TLC6C5912_small_delay(TLC6C5912_CLK_DELAY_NOP);
	 #endif
	
	 BSP_TLC6C5912_CLK_0();                  // bring CLK low
	 
	 #ifdef TLC6C5912_CLK_DELAY_WITH_NOP
   smp_TLC6C5912_small_delay(TLC6C5912_CLK_DELAY_NOP);
   #endif
	 #endif
	
	 BSP_TLC6C5912_CS_0();
	 BSP_TLC6C5912_DATA_0();
}

void smp_TLC6C5912_SendData(uint16_t dataout){
    int i;
    uint16_t mask;    
  
	  BSP_TLC6C5912_CS_0();
	  BSP_TLC6C5912_DATA_0();
    BSP_TLC6C5912_CLK_0();	
	
    for (i=16; i>0; i--) {
        mask = 1 << (i - 1);                    // calculate bitmask
			
			  if (dataout & mask)                     // output one data bit
          BSP_TLC6C5912_DATA_1();                     
        else                                    
          BSP_TLC6C5912_DATA_0();                     
			 
				BSP_TLC6C5912_CLK_1();	 
				
				#ifdef TLC6C5912_CLK_DELAY_WITH_NOP
			  smp_TLC6C5912_small_delay(TLC6C5912_CLK_DELAY_NOP);
        #endif
				
				BSP_TLC6C5912_CLK_0();                  
        
				#ifdef TLC6C5912_CLK_DELAY_WITH_NOP
				smp_TLC6C5912_small_delay(TLC6C5912_CLK_DELAY_NOP);
        #endif				
	   }	
				
    smp_TLC6C5912_OutputLed_Open();
		 
		#ifdef TLC6C5912_CLK_DELAY_WITH_NOP 
 		smp_TLC6C5912_small_delay(TLC6C5912_CLK_DELAY_NOP+2); 
    #endif
}

void smp_TLC6C5912_All_LED_On(void){
    for (int i=0; i< 16; i++) { 
			  Led_display.w <<= 1;
			  Led_display.w |= LED_ON;
    }
		smp_TLC6C5912_SendData(Led_display.w);
}

void smp_TLC6C5912_All_LED_Off(void){
    for (int i=0; i< 16; i++) {
		    Led_display.w <<= 1;
		    Led_display.w &= ~LED_ON;
    }
    smp_TLC6C5912_SendData(Led_display.w);
}

void smp_TLC6C5912_Percent20_LED(bsp_tlc6c5912_led_type led_status){

	Led_display.b.percent20 = led_status;
		
	smp_TLC6C5912_SendData(Led_display.w);
}

void smp_TLC6C5912_Percent40_LED(bsp_tlc6c5912_led_type led_status){

	Led_display.b.percent40 = led_status;
		
	smp_TLC6C5912_SendData(Led_display.w);
}

void smp_TLC6C5912_Percent60_LED(bsp_tlc6c5912_led_type led_status){

	Led_display.b.percent60 = led_status;
		
	smp_TLC6C5912_SendData(Led_display.w);
}

void smp_TLC6C5912_Percent80_LED(bsp_tlc6c5912_led_type led_status){

	Led_display.b.percent80 = led_status;
		
	smp_TLC6C5912_SendData(Led_display.w);
}

void smp_TLC6C5912_Percent100_LED(bsp_tlc6c5912_led_type led_status){

	Led_display.b.percent100 = led_status;
		
	smp_TLC6C5912_SendData(Led_display.w);
}

void smp_TLC6C5912_Discharge_LED(bsp_tlc6c5912_led_type led_status){

	Led_display.b.dsg = led_status;
		
	smp_TLC6C5912_SendData(Led_display.w);
}

void smp_TLC6C5912_Charge_LED(bsp_tlc6c5912_led_type led_status){

	Led_display.b.chg = led_status;
		
	smp_TLC6C5912_SendData(Led_display.w);
}

void smp_TLC6C5912_Alarm_LED(bsp_tlc6c5912_led_type led_status){

	Led_display.b.alarm = led_status;
		
	smp_TLC6C5912_SendData(Led_display.w);
}

void smp_TLC6C5912_Comm_LED(bsp_tlc6c5912_led_type led_status){

	Led_display.b.comm = led_status;
		
	smp_TLC6C5912_SendData(Led_display.w);
}
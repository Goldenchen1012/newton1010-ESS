/**
  ******************************************************************************
  * @file    Test_ADC.c
  * @author  Golden Chen
  * @version V0.0.1
  * @date    2022/02/14
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 SMP</center></h2>
  *
  * 
  *
  ******************************************************************************
  */

#include "stm32l4xx_hal.h"
#include "smp_adc.h"
#include "Test_ADC.h"
#include "main.h"
#include "SEGGER_RTT.h"
#include "RTT_Log.h"

void Test_Internal_ADC(void){	
    static	uint16_t	app_adc_temp[10];
    extern bsp_adc_init_io_config bsp_in_adc_ch[5];
	  static uint32_t test_cont;  
	
	  test_cont = 0;
	  smp_adc_adc_para_init(adc1);                            //Initial ADC 
	  while(1){
		    hal_internal_adc_get(&app_adc_temp[0] ,adc1 , bsp_in_adc_ch[0]);
		    hal_internal_adc_get(&app_adc_temp[1] ,adc1 , bsp_in_adc_ch[1]);
		    hal_internal_adc_get(&app_adc_temp[2] ,adc1 , bsp_in_adc_ch[2]);
		    hal_internal_adc_get(&app_adc_temp[3] ,adc1 , bsp_in_adc_ch[3]);
		    hal_internal_adc_get(&app_adc_temp[4] ,adc1 , bsp_in_adc_ch[4]);
		
		    LOG_BLUE("TEST ADC#%04d %04d,%04d,%04d,%04d,%04d\r\n", test_cont,app_adc_temp[0],app_adc_temp[1],app_adc_temp[2],app_adc_temp[3],app_adc_temp[4]);
		
		    HAL_Delay(50);
		
		    test_cont++;
		    if(test_cont >= TEST_INT_ADC_TEST_CYCLE_NUM) break;
    }	
}	
//--------FILE END------------------------------------------------------------------------------------------

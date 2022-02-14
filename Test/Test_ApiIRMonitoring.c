/**
  ******************************************************************************
  * @file    Test_ApiIRMonitoring.c
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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "define.h"
#include "main.h"
#include "ApiIRMonitoring.h"
#include "Test_ApiIRMonitoring.h"
#include "smp_ADS7946_Driver.h"
#include "HalAfeADS7946.h"
#include "LibSwTimer.h"
#include "LibHwTimer.h"
#include "RTT_Log.h"

long int irm_get_vstack_cont = 0;
uint16_t test_get_vatck_cnt = 0;
apiIRMonitoring_cb_t app_irm_event_cb;

extern void ads7946_callBack(uint8_t *pDat, uint8_t size);

//  Test use IRM calllback function to rx IRM data.
//--------------------------------------------------
uint8_t app_irm_rxdata_cb(IRMonitoring_Resistor_t *irm_res_data, IRMonitoring_event_cb_type irm_event){

  static IRMonitoring_Resistor_t temp_irm_data;
  static float temp_irm_vstack;
	
 	switch(irm_event)
 	{
 	case IRM_EVENT_BALANCE:
    temp_irm_data = *irm_res_data;
		break;
 	case IRM_EVENT_UNBALANCE:
    temp_irm_data = *irm_res_data;
		break;
 	case IRM_EVENT_GET_VSTACK:
    temp_irm_vstack = irm_res_data->V_stack;
		break;	
 	case IRM_EVENT_OTHER_ERR:

		break;
  }
	
	#if 1
	SEGGER_RTT_printf(0,"IRM EVENT=%d, Vstack=%d, Rn=%d, Rp=%d\r\n", irm_event, (uint16_t)(irm_res_data->V_stack), irm_res_data->Rn_kohm, irm_res_data->Rp_kohm);
	#endif
	
	return 0;
}
//--------------------------------------------------

void app_irm_sw_gpio_init_cb(void){
    GPIO_InitTypeDef GPIO_InitStructure;
	
	  // Control SW1~SW3 GPIO setting
	  //-------------------------------------------------
	  GPIO_InitStructure.Pin   = BSP_IRM_SW1_PIN;
	  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStructure.Pull  = GPIO_PULLUP;
	  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(BSP_IRM_SW1_PORT, &GPIO_InitStructure); 	

 	  GPIO_InitStructure.Pin   = BSP_IRM_SW2_PIN;
	  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStructure.Pull  = GPIO_PULLUP;
	  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(BSP_IRM_SW2_PORT, &GPIO_InitStructure); 	
	
 	  GPIO_InitStructure.Pin   = BSP_IRM_SW3_PIN;
	  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStructure.Pull  = GPIO_PULLUP;
	  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(BSP_IRM_SW3_PORT, &GPIO_InitStructure); 	
	  //-------------------------------------------------
}

void app_irm_sw1_gpio(IRMonitoring_SW_enum on_off){
    if(on_off == IRM_SW_ON){
	      BSP_IRM_SW1_ON();
	  }else{
		    BSP_IRM_SW1_OFF();
		}
}

void app_irm_sw2_gpio(IRMonitoring_SW_enum on_off){
    if(on_off == IRM_SW_ON){
	      BSP_IRM_SW2_ON();
	  }else{
		    BSP_IRM_SW2_OFF();
		}
}

void app_irm_sw3_gpio(IRMonitoring_SW_enum on_off){
    if(on_off == IRM_SW_ON){
	      BSP_IRM_SW3_ON();
	  }else{
		    BSP_IRM_SW3_OFF();
		}
}

IRM_Recv_CB_t irm_fun_ptr = 0;
static void app_imr_ads7946_callBack(uint8_t *pDat, uint8_t size)
{
	tIbyte	AdcValue;
	static float volt_data;
	static float volt_data2;
	static float adc_bits;
  adc_bits = (float)(1<<ADS7946_RESOLUTION_B); 	
	
	AdcValue.b[1] = pDat[2];
	AdcValue.b[0] = pDat[3];
	AdcValue.i >>= 2;
	volt_data = (float)(AdcValue.i)/adc_bits * ADS7946_VREF;
	//volt_data = (volt_data - VO_CAL_V_P1);
	volt_data = volt_data/2;
	
	if(volt_data< 0){
	    volt_data = 0;
	}
	
	if((BSP_IRM_SW1_PORT->ODR & BSP_IRM_SW1_PIN)== BSP_IRM_SW1_PIN){
	    if((BSP_IRM_SW2_PORT->ODR & BSP_IRM_SW2_PIN)== 0x0000){
			    if((BSP_IRM_SW3_PORT->ODR & BSP_IRM_SW3_PIN)== 0x0000){
						  volt_data2 = ((volt_data-VO_CAL_V_P1)*VSTACK_CAL_P1) +VSTACK_CAL_P2;       //Compensate for actual voltage(unit: V) 
	            LOG_GREEN("Vstack=%d mV\r\n",(uint16_t)(volt_data2*1000));
	        }
			}
	}
 
	LOG_GREEN("Vo=%d mV\r\n",(uint16_t)(volt_data*1000));
	
	if(irm_fun_ptr !=NULL){
		  irm_fun_ptr(&volt_data);
	}
	
}

void irm_data_ready_cb(IRM_Recv_CB_t rcv_ptr){
	irm_fun_ptr = rcv_ptr;
}

IRMonitoring_event_read_cb_type app_irm_trigger_voltage_data_cb(void){
	  static int8_t res;
		
	  //Test 2022.01.10
		GPIOD->ODR ^= GPIO_PIN_15;
	
    res = smp_ADS7946_get_data(channel_0,CS_0,app_imr_ads7946_callBack);
	  
	  if(res == SMP_SUCCESS){
		    return(IRM_OK);	
		}else{
		    return(IRM_BUSY);
		}
	
}

void app_irm_get_device_init_cb(void){
    smp_ADS7946_init();
}
//--------------------------------------------------

void Test_IRM_Function_Init(void){
	
	//Test IRM SW1 mesaure Vstack(Total Battreary Voltage)
	#if 0
	app_irm_sw_gpio_init_cb();
	BSP_IRM_SW1_ON();
	HAL_Delay(500);
	BSP_IRM_SW1_OFF();
	HAL_Delay(500);
	BSP_IRM_SW1_ON();
	HAL_Delay(500);	
	BSP_IRM_SW2_ON();
	HAL_Delay(500);	
	BSP_IRM_SW2_OFF();	
	HAL_Delay(500);
	BSP_IRM_SW3_ON();
	HAL_Delay(500);	
	BSP_IRM_SW3_OFF();	
	HAL_Delay(500);	
	while(1){
		 BSP_IRM_SW1_OFF();
     BSP_IRM_SW2_ON();
     BSP_IRM_SW3_OFF();			
	   app_irm_trigger_voltage_data_cb();
	   HAL_Delay(500);
	}
	#endif

}

void Test_IRM_Function_Exe(void){
	uint8_t res;
	//Register All callbasck function for IRM
	app_irm_event_cb.SW_gpioinit_cb = app_irm_sw_gpio_init_cb;
	app_irm_event_cb.SW_gpio_crtl_cb[0]=app_irm_sw1_gpio; 
	app_irm_event_cb.SW_gpio_crtl_cb[1]=app_irm_sw2_gpio;
	app_irm_event_cb.SW_gpio_crtl_cb[2]=app_irm_sw3_gpio; 
	app_irm_event_cb.GetVoltDeviceInit_cb = app_irm_get_device_init_cb;
	app_irm_event_cb.TriggerData_cb = app_irm_trigger_voltage_data_cb;
	app_irm_event_cb.irm_outdata = app_irm_rxdata_cb;
	app_irm_event_cb.DataReady_cb = irm_data_ready_cb;
	
	//Open IRM function, Setting 5s= IRM detection period, 200ms = IRM switch SW1~Sw3 waitting time. 
	res = apiIRMonitoringOpen(ITRM_TESTPARAM_DET_CYCLE_TIME, ITRM_TESTPARAM_RELAY_WAIT_TIME, app_irm_event_cb);
	
	//Get current Vstack, IRM use callback notification Vstack.
	while(test_get_vatck_cnt < TEST_IRM_FUNCTION_TEST_CYCLE_NUM){
		  test_get_vatck_cnt++;
		
	    LOG_BLUE("IRM Get Vstack...\r\n");
	    apiIRMonitoringGetVstack();
		  LibSwTimerHandle();
		  HAL_Delay(TEST_IRM_FUNCTION_TEST_CYCLE_DELAY_TIME); 
	}
}

//--------FILE END------------------------------------------------------------------------------------------

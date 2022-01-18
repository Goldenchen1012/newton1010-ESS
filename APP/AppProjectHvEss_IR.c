/**
  ******************************************************************************
  * @file        AppProjectHvEss_IR.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2022/1/10
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 SMP </center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "LibDebug.h"
#include "LibSwTimer.h"
#include "LibHwTimer.h"
#include "halafe.h"
#include "halUart.h"
#include "HalCan.h"
#include "HalBsp.h"
#include "HalTimer.h"
#include "HalRtc.h"
#include "HalEeprom.h"
#include "AppProject.h"
#include "ApiSysPar.h"
#include "AppProtect.h"
#include "AppSerialCanDavinci.h"
#include "AppSerialUartDavinci.h"
#include "AppBalance.h"
#include "AppGauge.h"
#include "ApiSignalFeedback.h"
#include "LibNtc.h"
#include "LibData.h"
#include "ApiRamData.h"
#include "HalRtc.h"
#include "ApiSysPar.h"
#include "ApiRelayControl.h"
#include "AppBms.h"
#include "ApiScuTemp.h"
#include "ApiSystemFlag.h"
#include "HalAfeADS7946.h"
#include "ApiEventLog.h"
#include "SmpEventType.h"
#include "AppButton.h"
#include "smp_w5500_DMA.h"
#include "AppTcpipSmp.h"
#include "HalSpirom.h"
#include "ApiIRMonitoring.h"
#include "smp_ADS7946_Driver.h"


void appSerialCanDavinciSendTextMessage(char *msg);
#define	projectIrDbgMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private define ------------------------------------------------------------*/
#define ADS7946_VREF                           4.096f
#define ADS7946_RESOLUTION_B                   14


/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static apiIRMonitoring_cb_t app_irm_event_cb;

/* Private function prototypes -----------------------------------------------*/
static void app_irm_sw_gpio_init_cb(void){
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
static void app_irm_sw1_gpio(IRMonitoring_SW_enum on_off){
    if(on_off == IRM_SW_ON){
	      BSP_IRM_SW1_ON();
	  }else{
		    BSP_IRM_SW1_OFF();
		}
}
static void app_irm_sw2_gpio(IRMonitoring_SW_enum on_off){
    if(on_off == IRM_SW_ON){
	      BSP_IRM_SW2_ON();
	  }else{
		    BSP_IRM_SW2_OFF();
		}
}
static void app_irm_sw3_gpio(IRMonitoring_SW_enum on_off){
    if(on_off == IRM_SW_ON){
	      BSP_IRM_SW3_ON();
	  }else{
		    BSP_IRM_SW3_OFF();
		}
}

static IRM_Recv_CB_t irm_fun_ptr = 0;
static void app_imr_ads7946_callBack(uint8_t *pDat, uint8_t size)
{
	tIbyte	AdcValue;
	static float volt_data;
	static float adc_bits;
	
	AdcValue.b[1] = pDat[2];
	AdcValue.b[0] = pDat[3];
	AdcValue.i >>= 2;
#if 1	
	volt_data = doCalibration(&SysCalPar.RamPar.VBat[0], AdcValue.i);
#else	
  	adc_bits = (float)(1<<ADS7946_RESOLUTION_B); 	
	volt_data = (float)(AdcValue.i)/adc_bits * ADS7946_VREF;
#endif	
	
	if(irm_fun_ptr !=NULL){
		  irm_fun_ptr(&volt_data);
	}
	
}
static void irm_data_ready_cb(IRM_Recv_CB_t rcv_ptr){
	irm_fun_ptr = rcv_ptr;
}

static IRMonitoring_event_read_cb_type app_irm_trigger_voltage_data_cb(void){
	  static int8_t res;
		
    res = smp_ADS7946_get_data(channel_0,CS_0,app_imr_ads7946_callBack);
	  
	  if(res == SMP_SUCCESS){
		    return(IRM_OK);	
		}else{
		    return(IRM_BUSY);
		}
	
}
static void app_irm_get_device_init_cb(void){
    smp_ADS7946_init();
}
static uint8_t app_irm_rxdata_cb(IRMonitoring_Resistor_t *irm_res_data, IRMonitoring_event_cb_type irm_event){

  static IRMonitoring_Resistor_t temp_irm_data;
  static float temp_irm_vstack;
	
	char	str[100];
	
	/*
	uint16_t Rp_kohm;
	uint16_t Rn_kohm;
	float V_stack;   
	*/
 	switch(irm_event)
 	{
 	case IRM_EVENT_BALANCE:
    	temp_irm_data = *irm_res_data;
 //   	projectIrDbgMsg("IRM_EVENT_BALANCE");
		break;
 	case IRM_EVENT_UNBALANCE:
    	temp_irm_data = *irm_res_data;
//    	projectIrDbgMsg("IRM_EVENT_UNBALANCE");
		break;
 	case IRM_EVENT_GET_VSTACK:
    	temp_irm_vstack = irm_res_data->V_stack;
    	projectIrDbgMsg("IRM_EVENT_GET_VSTACK");
    	sprintf(str,"Vbat = %.3f", temp_irm_vstack);
    	projectIrDbgMsg(str);
		break;	
 	case IRM_EVENT_OTHER_ERR:
		projectIrDbgMsg("IRM_EVENT_OTHER_ERR");
		break;
	default:
		projectIrDbgMsg("default");
		break;		
  }
	
	#if 1
	SEGGER_RTT_printf(0,"IRM EVENT=%d, Vstack=%d, Rn=%d, Rp=%d\r\n", irm_event, (int)(temp_irm_data.V_stack), temp_irm_data.Rn_kohm, temp_irm_data.Rp_kohm);
	#endif
	
	return 0;
}
/* Public function prototypes -----------------------------------------------*/
void IrFunctionOpen(void)
{
	uint8_t	res;
	tScuProtectPar	Par;
	
	apiSysParGetInsulationResistance(&Par);
		
	//Register All callbasck function for IRM
	app_irm_event_cb.SW_gpioinit_cb = app_irm_sw_gpio_init_cb;
	app_irm_event_cb.SW_gpio_crtl_cb[0] = app_irm_sw1_gpio; 
	app_irm_event_cb.SW_gpio_crtl_cb[1] = app_irm_sw2_gpio;
	app_irm_event_cb.SW_gpio_crtl_cb[2] = app_irm_sw3_gpio; 
	app_irm_event_cb.GetVoltDeviceInit_cb = app_irm_get_device_init_cb;
	app_irm_event_cb.TriggerData_cb = app_irm_trigger_voltage_data_cb;
	app_irm_event_cb.irm_outdata = app_irm_rxdata_cb;
	app_irm_event_cb.DataReady_cb = irm_data_ready_cb;
	
	//Open IRM function, Setting 2s= IRM detection period, 100ms = IRM switch SW1~Sw3 waitting time. 
	res = apiIRMonitoringOpen(Par.SetValue.l, Par.STime.l, app_irm_event_cb);
	
	//Get current Vstack, IRM use callback notification Vstack.
	//apiIRMonitoringGetVstack();


}
/************************ (C) COPYRIGHT SMP *****END OF FILE****/    


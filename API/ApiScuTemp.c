/**
  ******************************************************************************
  * @file        AppScuTemp.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/15
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
#include <stdio.h>
#include <stdlib.h>
#include "define.h"
#include "main.h"
#include "halafe.h"
#include "ApiSysPar.h"
#include "AppGauge.h"
#include "LibSwTimer.h"
#include "LibHwTimer.h"
#include "HalRtc.h"
//#include "ApiRamData.h"
//#include "ApiSysPar.h"
#include "AppBms.h"
#include "ApiSystemFlag.h"
//#include "ApiRelayControl.h"
#include "AppSerialCanDavinci.h"
#include "ApiSignalFeedback.h"
#include "smp_adc.h"
//#include "

void appSerialUartSendMessage(char *str);
void appSerialCanDavinciSendTextMessage(char *str);

#define	appScuTempDebugMsg(str)		appSerialCanDavinciSendTextMessage(str);
//appSerialCanDavinciSendTextMessage(str)

//appSerialUartSendMessage

/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint16_t app_adc_temp[5]={0};

//Select BSP_ADC Channel use
bsp_adc_init_io_config bsp_in_adc_ch[5]={BSP_TMP_RLY_1,
                                         BSP_TMP_RLY_2,
                                         BSP_TMP_RLY_AMB,
                                         BSP_TMP_RLY_BBP,
                                         BSP_TMP_RLY_BBN,
                                        };

/* Private function prototypes -----------------------------------------------*/
#define	REAL_NTC_VOLTAGE	3300
static uint16_t cvtMcuNtcTemp(uint16_t adc)
{
	char	str[100];
	
	uint32_t	voltage;
	voltage = adc;
	voltage *= 3300;
	voltage /= 4096;
	
	voltage *= 5000;
	voltage /= REAL_NTC_VOLTAGE;
	
//	sprintf(str,"V= %d", voltage);
//	appScuTempDebugMsg(str);
	
	return LibNtcVoltageToTemperature(voltage);
}

static void getScuTempValue(void)
{
	int32_t	temp;
	char	str[100];
	hal_internal_adc_get(&app_adc_temp[0] ,adc1 , bsp_in_adc_ch[0]);
	hal_internal_adc_get(&app_adc_temp[1] ,adc1 , bsp_in_adc_ch[1]);
	hal_internal_adc_get(&app_adc_temp[2] ,adc1 , bsp_in_adc_ch[2]);
	hal_internal_adc_get(&app_adc_temp[3] ,adc1 , bsp_in_adc_ch[3]);
	hal_internal_adc_get(&app_adc_temp[4] ,adc1 , bsp_in_adc_ch[4]);
	
#if	0
	sprintf(str,"RLY1 ADC = %d %.2lfC", app_adc_temp[0],
			(double)((double)cvtMcuNtcTemp(app_adc_temp[0])-4000.0)/100.0);
	appScuTempDebugMsg(str);

	sprintf(str,"RLY2 ADC = %d %.2lfC", app_adc_temp[1],
			(double)((double)cvtMcuNtcTemp(app_adc_temp[1])-4000.0)/100.0);	
	appScuTempDebugMsg(str);
	
	sprintf(str,"Abm ADC = %d %.2lfC", app_adc_temp[2],
			(double)((double)cvtMcuNtcTemp(app_adc_temp[2])-4000.0)/100.0);	
	appScuTempDebugMsg(str);
	
	sprintf(str,"BBP ADC = %d %.2lfC", app_adc_temp[3],
			(double)((double)cvtMcuNtcTemp(app_adc_temp[3])-4000.0)/100.0);	
	appScuTempDebugMsg(str);
	
	sprintf(str,"BBN ADC = %d %.2lfC", app_adc_temp[4],
			(double)((double)cvtMcuNtcTemp(app_adc_temp[4])-4000.0)/100.0);	
	appScuTempDebugMsg(str);
#endif	
	
}
static void scuTempSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	char	str[100];
	static uint8_t	count = 0;
	
	//GPIOD->ODR |= GPIO_PIN_14;
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
	
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_5)
	{	
		count++;
		if(count >= 100)
		{
			count = 0;
			getScuTempValue();
//			GPIOD->ODR ^= GPIO_PIN_14;

//			appSerialCanDavinciSendTextMessage("scutemp 1sec");
//			appSerialUartSendMessage("scutemp 1sec");
		}
	}
	//GPIOD->ODR &= ~GPIO_PIN_14;
}

/* Public function prototypes -----------------------------------------------*/
uint16_t apiScuTempGetTemperature(uint8_t index)
{
	return cvtMcuNtcTemp(app_adc_temp[index]);
}
void apiScuTempOpen(void)
{
	uint32_t		d;
	
	//return;
//	appScuTempDebugMsg("appScuTempOpen..0");

//	for(d=0; d<0xfff0000;d++);
	smp_adc_adc_para_init(adc1);
	/*
	while(1){
		app_adc_temp[0] = smp_adc_get(adc1, Tmp_Rly_1);
		app_adc_temp[1] = smp_adc_get(adc1, Tmp_Rly_2);
		app_adc_temp[2] = smp_adc_get(adc1, Tmp_Rly_Amb);
		app_adc_temp[3] = smp_adc_get(adc1, Tmp_BBP);
		app_adc_temp[4] = smp_adc_get(adc1, Tmp_BBN);
		drv_bq796xx_delay_ms(10);
		
		test_cont++;
		if(test_cont>=20) break;
	}
	*/
	LibSwTimerOpen(scuTempSwTimerHandler, 0);
//	appScuTempDebugMsg("appScuTempOpen...1");

}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    















/**
  ******************************************************************************
  * @file        HalAfeADS7946.c
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/28
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
#include "halafe.h"
#include "LibSwTimer.h"
#include "LibNtc.h"
#include "HalAfeBq796xx.h"
#include "HalAfeADS7946.h"
#include "smp_ADS7946_Driver.h"
#include "ApiSysPar.h"
#include "AppProject.h"

void appSerialUartSendMessage(char *str);
void appSerialCanDavinciSendTextMessage(char *str);

#define	halAfeADS7946DebugMsg(str)		appSerialCanDavinciSendTextMessage(str);


/* Private define ------------------------------------------------------------*/
enum{
	ADC_CURR_P = 0,
	ADC_CURR_N,
	ADC_VB_INT,
	ADC_VB_EXT
};
/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t	adcChannel = 4;

static uint8_t	Callback_flag = 0;
static uint8_t	TimeOutCoiunt = 0;

//int32_t	ads_7945_adcValue[4];

/* Private function prototypes -----------------------------------------------*/


static void updateInternalVbatVoltage(void)
{
	if(!appProjectIsInSimuMode())
	{
		halAfeSetVBatVoltage(0, doCalibration(&SysCalPar.RamPar.VBat[0], halAfeGetVBatAdcValue(0)));
		//HalAfeSetCurrentValue(1, doCalibration(&SysCalPar.RamPar.Current[1], halAfeGetCurrentAdcValue(1)));		
	}
	
}

static void updateExternalVbatVoltage(void)
{
	if(!appProjectIsInSimuMode())
	{
		halAfeSetVBatVoltage(1, doCalibration(&SysCalPar.RamPar.VBat[1], halAfeGetVBatAdcValue(1)));
		//HalAfeSetCurrentValue(1, doCalibration(&SysCalPar.RamPar.Current[1], halAfeGetCurrentAdcValue(1)));		
	}
}

static void ads7946_callBack(uint8_t *pDat, uint8_t size)
{
	char	str[100];
	tIbyte	AdcValue;
		
	AdcValue.b[1] = pDat[2];
	AdcValue.b[0] = pDat[3];
	AdcValue.i >>= 3;
	
//	sprintf(str,"Adc 7946 =0x %.2X %.2X %.2x %.2X ==> %d", 
//				pDat[0], pDat[1], pDat[2], pDat[3], 
//				AdcValue.i);
//	halAfeADS7946DebugMsg(str);//"ads7946_cb");
	Callback_flag = 1;

	if(appProjectIsInSimuMode() != 0)
		return;
	switch(adcChannel)
	{
	case ADC_CURR_P:
		halAfeSetCurrentAdcValue(0, AdcValue.i);
		break;
	case ADC_CURR_N:
		halAfeSetCurrentAdcValue(1, AdcValue.i);
		break;
	case ADC_VB_INT:
		halAfeSetVBatAdcValue(0, AdcValue.i);
		updateInternalVbatVoltage();
		break;
	case ADC_VB_EXT:
		halAfeSetVBatAdcValue(1, AdcValue.i);
		updateExternalVbatVoltage();
		break;
	}	
}


//void HalAfeSetCurrentValue(int32_t	current)
static void halAfeCurrentGetCurrentValue(void)
{
	if(!appProjectIsInSimuMode())
	{
		HalAfeSetCurrentValue(0, doCalibration(&SysCalPar.RamPar.Current[0], halAfeGetCurrentAdcValue(0)));
		HalAfeSetCurrentValue(1, doCalibration(&SysCalPar.RamPar.Current[1], halAfeGetCurrentAdcValue(1)));
	}
}


static void currentSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	static	uint8_t	count =0;
	static uint8_t	step =0;
//	GPIOD->ODR |= GPIO_PIN_14;
	
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
		
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_3)
	{
		halAfeCurrentGetCurrentValue();
		TimeOutCoiunt++;
		//if(Callback_flag || TimeOutCoiunt >= 3)
		count++;
		if(count >= 50)
		{
			count = 0;
			//step = 1;
			adcChannel++;
			if(adcChannel >= 4)
				adcChannel = 0;
				
			switch(adcChannel)
			{
			case ADC_CURR_P:
				smp_ADS7946_get_data(channel_1,CS_0,ads7946_callBack);
			//	halAfeADS7946DebugMsg("ch 1 CS 0");
				break;
			case ADC_CURR_N:
				smp_ADS7946_get_data(channel_1,CS_1,ads7946_callBack);	
			//	halAfeADS7946DebugMsg("ch 1 CS 1");
				break;
			case ADC_VB_INT:
				smp_ADS7946_get_data(channel_0,CS_0,ads7946_callBack);
			//	halAfeADS7946DebugMsg("ch 0 CS 0");
				break;
			case ADC_VB_EXT:
				smp_ADS7946_get_data(channel_0,CS_1,ads7946_callBack);
			//	halAfeADS7946DebugMsg("ch 0 CS 1");
				break;
			}
		}
		//GPIOD->ODR ^= GPIO_PIN_14;
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
	///	 smp_ADS7946_get_data(channel_0,CS_0);
	//	HAL_Delay(10);
	//	smp_ADS7946_get_data(channel_1,CS_0);
	//	HAL_Delay(10);
	//	smp_ADS7946_get_data(channel_0,CS_1);
	//	HAL_Delay(10);
	//	smp_ADS7946_get_data(channel_1,CS_1);
		
	//	halAfeADS7946DebugMsg("ADS7946 1sec");
	}
	
//	GPIOD->ODR &= ~GPIO_PIN_14;
}
/* Public function prototypes -----------------------------------------------*/

void halAfeCurrentOpen(void)
{
//	MX_DMA_Init();
	Callback_flag = 1;
	TimeOutCoiunt = 0;
	smp_ADS7946_init();
		
	LibSwTimerOpen(currentSwTimerHandler, 0);
}


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    





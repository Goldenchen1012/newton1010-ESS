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
#include "ApiIRMonitoring.h"


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
#define	CURR_P_FLAG		0x01
#define	CURR_N_FLAG		0x02
#define	VPACK_FLAG		0x04
#define	VBAT_FLAG		0x08

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
void (*adc7946FunctionProcessor)(void) = {0};

static uint8_t	adcChannel = 0;

static uint8_t	WaitAdcCallbackCount;
static uint8_t	AdcFlag = 0;


//int32_t	ads_7945_adcValue[4];

/* Private function prototypes -----------------------------------------------*/


static void updateInternalVbatVoltage(void)
{
	if(!appProjectIsInSimuMode())
	{
		;//halAfeSetVBatVoltage(0, doCalibration(&SysCalPar.RamPar.VBat[0], halAfeGetVBatAdcValue(0)));
	}
	
}

static void updateExternalVbatVoltage(void)
{
	int32_t		voltage;
	if(!appProjectIsInSimuMode())
	{		
		voltage = doCalibration(&SysCalPar.RamPar.VBat[1], halAfeGetVBatAdcValue(1));
		if(voltage <=  1000)
			voltage = 0;
		halAfeSetVBatVoltage(AFE_VPACK_INDEX, voltage);
	}
}
static void getCurrentP(void);
static void getCurrentN(void);
static void ads7946_callBack(uint8_t *pDat, uint8_t size);
static void getVpack(void);
static void getVbat(void);


static void getCurrentPWaitResponse(void)
{
	WaitAdcCallbackCount++;
	if(WaitAdcCallbackCount >= 5)
		adc7946FunctionProcessor = getCurrentP;
}

static void getCurrentP(void)
{
	int8_t	res;
//	GPIOD->ODR |= GPIO_PIN_14;	
	res = smp_ADS7946_get_data(channel_1, CS_0, ads7946_callBack);
	if(res == SMP_SUCCESS)
	{
		adc7946FunctionProcessor = getCurrentPWaitResponse;
		WaitAdcCallbackCount = 0;
	}
}

static void getCurrentNWaitResponse(void)
{
	WaitAdcCallbackCount++;
	if(WaitAdcCallbackCount >= 5)
		adc7946FunctionProcessor = getCurrentN;
}

static void getCurrentN(void)
{
	int8_t	res;
//	GPIOD->ODR |= GPIO_PIN_14;	
	res = smp_ADS7946_get_data(channel_1, CS_1, ads7946_callBack);	
	if(res == SMP_SUCCESS)
	{
		adc7946FunctionProcessor = getCurrentNWaitResponse;
		WaitAdcCallbackCount = 0;
	}
}
static void getVpackWaitResponse(void)
{
	WaitAdcCallbackCount++;
	if(WaitAdcCallbackCount >= 5)
	{
		adc7946FunctionProcessor = getVpack;
		//halAfeADS7946DebugMsg("get VPack-2");
	}
}

static void getVpack(void)
{
	int8_t	res;
//	GPIOD->ODR |= GPIO_PIN_14;	
	//halAfeADS7946DebugMsg("get VPack-1");
	res = smp_ADS7946_get_data(channel_0, CS_1, ads7946_callBack);
	if(res == SMP_SUCCESS)
	{
		adc7946FunctionProcessor = getVpackWaitResponse;
		WaitAdcCallbackCount = 0;
	}
}

static void getVbatWaitResponse(void)
{
	WaitAdcCallbackCount++;
	if(WaitAdcCallbackCount >= 5)
	{
		adc7946FunctionProcessor = getVbat;
		//halAfeADS7946DebugMsg("get Vbat-2");
	}
}

static void getVbat(void)
{
	int8_t	res;
//	GPIOD->ODR |= GPIO_PIN_14;	
	//halAfeADS7946DebugMsg("get Vbat-1");
	res = smp_ADS7946_get_data(channel_0, CS_0, ads7946_callBack);
	if(res == SMP_SUCCESS)
	{
		adc7946FunctionProcessor = getVbatWaitResponse;
		WaitAdcCallbackCount = 0;
	}
}
static void ads7946_callBack(uint8_t *pDat, uint8_t size)
{
	char	str[100];
	tIbyte	AdcValue;
		
	AdcValue.b[1] = pDat[2];
	AdcValue.b[0] = pDat[3];
	AdcValue.i >>= 2;
	
//	sprintf(str,"Adc 7946 =0x %.2X %.2X %.2x %.2X ==> %d", 
//				pDat[0], pDat[1], pDat[2], pDat[3], 
//				AdcValue.i);
//	halAfeADS7946DebugMsg(str);//"ads7946_cb");
//	GPIOD->ODR &= ~GPIO_PIN_14;	

	if(adc7946FunctionProcessor == getCurrentPWaitResponse)
	{
		AdcFlag &= ~CURR_P_FLAG;
		if(appProjectIsInSimuMode() == 0)
			halAfeSetCurrentAdcValue(0, AdcValue.i);	
	}
	else if(adc7946FunctionProcessor == getCurrentNWaitResponse)
	{
		AdcFlag &= ~CURR_N_FLAG;
		if(appProjectIsInSimuMode() == 0)
			halAfeSetCurrentAdcValue(1, AdcValue.i);
	}
	else if(adc7946FunctionProcessor == getVpackWaitResponse)
	{
		AdcFlag &= ~VPACK_FLAG;
		if(appProjectIsInSimuMode() == 0)
		{
			halAfeSetVBatAdcValue(1, AdcValue.i);
			updateExternalVbatVoltage();
		}
#if	0		
		if((AdcFlag & VBAT_FLAG) == 0)
		{
			halAfeADS7946DebugMsg("Get Vbat...");
			apiIRMonitoringGetVstack();
		}
#endif		
	}
	else if(adc7946FunctionProcessor == getVbatWaitResponse)
	{
		AdcFlag &= ~VBAT_FLAG;
		if(appProjectIsInSimuMode() == 0)
		{
//			halAfeSetVBatAdcValue(0, AdcValue.i);
//			updateInternalVbatVoltage();
		}
	}
	
	if(AdcFlag & CURR_P_FLAG)
	{
		adc7946FunctionProcessor = getCurrentP;
	}
	else if(AdcFlag & CURR_N_FLAG)
	{
		adc7946FunctionProcessor = getCurrentN;
	}
	else if(AdcFlag & VPACK_FLAG)
	{
		adc7946FunctionProcessor = getVpack;
	}
	else if(AdcFlag & VBAT_FLAG)
	{
		adc7946FunctionProcessor = getVbat;
		//halAfeADS7946DebugMsg("get Vbat-0");
	}
	else
		adc7946FunctionProcessor = 0;
	
	//if(AdcFlag)
	//	GPIOD->ODR |= GPIO_PIN_14;	
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

//adc7946FunctionProcessor

static void currentSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	static	uint16_t	count =0;
	
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
//		GPIOD->ODR ^= GPIO_PIN_15;	
		if(adc7946FunctionProcessor)
			adc7946FunctionProcessor();
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_3)
	{
		halAfeCurrentGetCurrentValue();
		//GPIOD->ODR ^= GPIO_PIN_14;
		if(adc7946FunctionProcessor == 0)
			adc7946FunctionProcessor = getCurrentP;
		AdcFlag |= (CURR_P_FLAG + CURR_N_FLAG);
		//AdcFlag |= (CURR_P_FLAG + 0);

	}
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_8)
	{
		count++;
		if(count >= 50)
		{
			count = 0;
			#if	0
			if(appProjectIsInEngMode())
			{
				AdcFlag |= (VPACK_FLAG + VBAT_FLAG);
				//count = 30;	
				//halAfeADS7946DebugMsg("Read Int Ext");
			}
			else
			#endif
			{
				AdcFlag |= (VPACK_FLAG);
				//halAfeADS7946DebugMsg("Read Ext");
			}
			if(adc7946FunctionProcessor == 0)
				adc7946FunctionProcessor = getVpack;
		}	
	}
}
/* Public function prototypes -----------------------------------------------*/

void halAfeCurrentOpen(void)
{
	AdcFlag = 0;
	smp_ADS7946_init();
	adc7946FunctionProcessor = 0;
	LibSwTimerOpen(currentSwTimerHandler, 0);
}


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    





/**
  ******************************************************************************
  * @file        AppProjectHvEss.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/12/1
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
#include "AppProjectHvEss_IR.h"

void appSerialCanDavinciSendTextMessage(char *msg);
#define	appProjectDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private define ------------------------------------------------------------*/
#define	saveEventLog(type, par)		apiEventLogSaveLogData(type, par)

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


tHalTimer	mHalTimer3={3, 1000};
static uint8_t	RtcValid;
static uint8_t	ModeFlag = 0;
static uint8_t	ScuId = 1;
static uint8_t	SystemReadyFlag = 0;
static uint8_t	RelayOnFlag = 0;

/* Private function prototypes -----------------------------------------------*/
static void releaseOCP(void);

static void relayOff(void)
{	
	apiRelayControlMainRelayOff();
	RelayOnFlag = 0;
}
static void protectEventHandler(void *pDest, uint16_t evt, void *pData)
{
	switch(evt)
	{
	case APP_PROTECT_OVP_L1_SET:
		saveEventLog(EVENT_TYPE_OVP_L1_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_OVP_L2_SET:
		saveEventLog(EVENT_TYPE_OVP_L2_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_OVP_L3_SET:
		relayOff();
		//appProjectDebugMsg("OVP L3 set");
		saveEventLog(EVENT_TYPE_OVP_L3_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_OVP_L1_RELEASE:
		saveEventLog(EVENT_TYPE_OVP_L1_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_OVP_L2_RELEASE:
		saveEventLog(EVENT_TYPE_OVP_L2_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_OVP_L3_RELEASE:
		saveEventLog(EVENT_TYPE_OVP_L3_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_UVP_L1_SET:
		saveEventLog(EVENT_TYPE_UVP_L1_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_UVP_L2_SET:
		saveEventLog(EVENT_TYPE_UVP_L2_SET, libGetUint16((uint8_t *)pData));	
		break;
	case APP_PROTECT_UVP_L3_SET:
		saveEventLog(EVENT_TYPE_UVP_L3_SET, libGetUint16((uint8_t *)pData));
		relayOff();
		break;
	case APP_PROTECT_UVP_L1_RELEASE:
		saveEventLog(EVENT_TYPE_UVP_L1_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_UVP_L2_RELEASE:
		saveEventLog(EVENT_TYPE_UVP_L2_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_UVP_L3_RELEASE:
		saveEventLog(EVENT_TYPE_UVP_L3_RELEASE, libGetUint16((uint8_t *)pData));
		relayOff();
		break;
	case APP_PROTECT_COTP_L1_SET:
		saveEventLog(EVENT_TYPE_COTP_L1_SET, libGetUint16((uint8_t *)pData));
		break;	
	case APP_PROTECT_COTP_L2_SET:
		saveEventLog(EVENT_TYPE_COTP_L2_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COTP_L3_SET:
		saveEventLog(EVENT_TYPE_COTP_L3_SET, libGetUint16((uint8_t *)pData));
		relayOff();
		break;
	case APP_PROTECT_COTP_L4_SET:
		saveEventLog(EVENT_TYPE_COTP_L4_SET, libGetUint16((uint8_t *)pData));
		relayOff();
		break;
	case APP_PROTECT_COTP_L1_RELEASE:
		saveEventLog(EVENT_TYPE_COTP_L1_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COTP_L2_RELEASE:
		saveEventLog(EVENT_TYPE_COTP_L2_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COTP_L3_RELEASE:
		saveEventLog(EVENT_TYPE_COTP_L3_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COTP_L4_RELEASE:
		saveEventLog(EVENT_TYPE_COTP_L4_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_CUTP_L1_SET:
		saveEventLog(EVENT_TYPE_CUTP_L1_SET, libGetUint16((uint8_t *)pData));
		break;	
	case APP_PROTECT_CUTP_L2_SET:
		saveEventLog(EVENT_TYPE_CUTP_L2_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_CUTP_L3_SET:
		saveEventLog(EVENT_TYPE_CUTP_L3_SET, libGetUint16((uint8_t *)pData));
		relayOff();
		break;
	case APP_PROTECT_CUTP_L4_SET:
		saveEventLog(EVENT_TYPE_CUTP_L4_SET, libGetUint16((uint8_t *)pData));
		relayOff();
		break;
	case APP_PROTECT_CUTP_L1_RELEASE:
		saveEventLog(EVENT_TYPE_CUTP_L1_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_CUTP_L2_RELEASE:
		saveEventLog(EVENT_TYPE_CUTP_L2_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_CUTP_L3_RELEASE:
		saveEventLog(EVENT_TYPE_CUTP_L3_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_CUTP_L4_RELEASE:
		saveEventLog(EVENT_TYPE_CUTP_L4_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOTP_L1_SET:
		saveEventLog(EVENT_TYPE_DOTP_L1_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOTP_L2_SET:		
		saveEventLog(EVENT_TYPE_DOTP_L2_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOTP_L3_SET:
		relayOff();
		saveEventLog(EVENT_TYPE_DOTP_L3_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOTP_L1_RELEASE:
		saveEventLog(EVENT_TYPE_DOTP_L1_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOTP_L2_RELEASE:
		saveEventLog(EVENT_TYPE_DOTP_L2_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOTP_L3_RELEASE:
		saveEventLog(EVENT_TYPE_DOTP_L3_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DUTP_L1_SET:
		saveEventLog(EVENT_TYPE_DUTP_L1_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DUTP_L2_SET:		
		saveEventLog(EVENT_TYPE_DUTP_L2_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DUTP_L3_SET:
		relayOff();
		saveEventLog(EVENT_TYPE_DUTP_L3_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DUTP_L1_RELEASE:
		saveEventLog(EVENT_TYPE_DUTP_L1_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DUTP_L2_RELEASE:
		saveEventLog(EVENT_TYPE_DUTP_L2_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DUTP_L3_RELEASE:
		saveEventLog(EVENT_TYPE_DUTP_L3_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOCP_L1_SET:
		saveEventLog(EVENT_TYPE_DOCP_L1_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOCP_L2_SET:
		saveEventLog(EVENT_TYPE_DOCP_L2_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOCP_L3_SET:
		relayOff();
		saveEventLog(EVENT_TYPE_DOCP_L3_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOCP_L4_SET:
		relayOff();
		saveEventLog(EVENT_TYPE_DOCP_L4_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOCP_L1_RELEASE:
		saveEventLog(EVENT_TYPE_DOCP_L1_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOCP_L2_RELEASE:
		saveEventLog(EVENT_TYPE_DOCP_L2_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOCP_L3_RELEASE:
		saveEventLog(EVENT_TYPE_DOCP_L3_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOCP_L4_RELEASE:
		saveEventLog(EVENT_TYPE_DOCP_L4_RELEASE, libGetUint16((uint8_t *)pData));
		break;	
	case APP_PROTECT_COCP_L1_SET:
		saveEventLog(EVENT_TYPE_COCP_L1_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COCP_L2_SET:	
		saveEventLog(EVENT_TYPE_COCP_L2_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COCP_L3_SET:
		relayOff();
		saveEventLog(EVENT_TYPE_COCP_L3_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COCP_L4_SET:
		relayOff();
		saveEventLog(EVENT_TYPE_COCP_L4_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COCP_L1_RELEASE:
		saveEventLog(EVENT_TYPE_COCP_L1_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COCP_L2_RELEASE:
		saveEventLog(EVENT_TYPE_COCP_L2_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COCP_L3_RELEASE:
		saveEventLog(EVENT_TYPE_COCP_L3_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COCP_L4_RELEASE:
		saveEventLog(EVENT_TYPE_COCP_L4_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_OVP_PF:
		relayOff();
		apiSysParOvpPfSet();
		saveEventLog(EVENT_TYPE_OVP_PF, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_UVP_PF:
		relayOff();
		apiSysParUvpPfSet();
		saveEventLog(EVENT_TYPE_UVP_PF, libGetUint16((uint8_t *)pData));
		break;
	}
}

static void gaugeEventHandler(void *pDest, uint16_t evt, void *pData)
{
	switch(evt)
	{
	case APP_GAUGE_EVENT_START_CHG_SOC:
	case APP_GAUGE_EVENT_START_DHG_SOC:
	case APP_GAUGE_EVENT_IDLE_OVER_5HR:
	case APP_GAUGE_EVENT_CANNOT_GET_5HR_SOC:
	case APP_GAUGE_EVENT_UPDATE_QMAX_1ST:
	case APP_GAUGE_EVENT_UPDATE_QMAX:
	case APP_GAUGE_EVENT_GET_5HR_SOC:
		break;
	}
	if(evt >= APP_GAUGE_EVENT_CAL_RA1 &&
	   evt <= (APP_GAUGE_EVENT_CAL_RA1+30))
	{
		;
	}

}

static void appProjectDavinciBalanceEventHandler(void *pDest, uint16_t evt, void *pData)
{
	char	str[100];
	uint16_t	cell;
	
	cell = libGetUint16((uint8_t *)pData);
	switch(evt)
	{
	case APP_CHG_BALANCE_SET:
		sprintf(str,"CHG Balance SET %d",cell);
		break;
	case APP_CHG_BALANCE_RELEASE:
		sprintf(str,"CHG Balance Release %d",cell);
		break;
	case APP_DHG_BALANCE_SET:
		sprintf(str,"DHG Balance SET %d",cell);
		break;
	case APP_DHG_BALANCE_RELEASE:
		sprintf(str,"DHG Balance Release %d",cell);
		break;
	case APP_RLX_BALANCE_SET:
		sprintf(str,"RLX Balance SET %d",cell);
		break;
	case APP_RLX_BALANCE_RELEASE:
		sprintf(str,"RLX Balance Release %d",cell);
		break;
	}
	
	appSerialCanDavinciSendTextMessage(str);
}
static void signalFeedbackEventHandler(void *pDest, uint16_t evt, void *pData)
{
	switch(evt)
	{
	case APP_SIGNAL_FB_EVT_DI1_HI:
	//	appProjectDebugMsg("DI1 Hi");
		break;
	case APP_SIGNAL_FB_EVT_DI1_LO:
	//	appProjectDebugMsg("DI1 Low");
		break;
	case APP_SIGNAL_FB_EVT_DI2_HI:
	//	appProjectDebugMsg("DI2 Hi");
		break;
	case APP_SIGNAL_FB_EVT_DI2_LO:
	//	appProjectDebugMsg("DI2 Low");
		break;
	case APP_SIGNAL_FB_EVT_EPO_HI:
		//appProjectDebugMsg("..............EPO Hi");
		break;
	case APP_SIGNAL_FB_EVT_EPO_LO:
		//appProjectDebugMsg("..............EOP Low");
		relayOff();
		break;
	case APP_SIGNAL_FB_EVT_SP_HI:
		//appProjectDebugMsg("SP Hi");
		break;
	case APP_SIGNAL_FB_EVT_SP_LO:
		//appProjectDebugMsg("SP Low");
		break;
	case APP_SIGNAL_FB_EVT_PS1_HI:
		//appProjectDebugMsg("PS1 Hi");
		break;
	case APP_SIGNAL_FB_EVT_PS1_LO:
		//appProjectDebugMsg("PS1 Low");
		break;
	case APP_SIGNAL_FB_EVT_PS2_HI:
	//	appProjectDebugMsg("PS2 Hi");
		break;
	case APP_SIGNAL_FB_EVT_PS2_LO:
	//	appProjectDebugMsg("PS2 Low");
		break;
	case APP_SIGNAL_FB_EVT_PS3_HI:
	//	appProjectDebugMsg("PS3 Hi");
		break;
	case APP_SIGNAL_FB_EVT_PS3_LO:
	//	appProjectDebugMsg("PS3 Low");
		break;
	case APP_SIGNAL_FB_EVT_BUTTON_HI:
		//appProjectDebugMsg("Button Hi");
		break;
	case APP_SIGNAL_FB_EVT_BUTTON_LO:
		//appProjectDebugMsg("Button Low");
		break;

	case APP_SIGNAL_FB_EVT_K1_HI:
	//	appProjectDebugMsg("K1 Hi");
		break;
	case APP_SIGNAL_FB_EVT_K1_LO:
	//	appProjectDebugMsg("K1 Low");
		break;
	case APP_SIGNAL_FB_EVT_K2_HI:
	//	appProjectDebugMsg("K2 Hi");
		break;
	case APP_SIGNAL_FB_EVT_K2_LO:
	//	appProjectDebugMsg("K2 Low");
		break;
	case APP_SIGNAL_FB_EVT_K3_HI:
	//	appProjectDebugMsg("K3 Hi");
		break;
	case APP_SIGNAL_FB_EVT_K3_LO:
	//	appProjectDebugMsg("K3 Low");
		break;
	case APP_SIGNAL_FB_EVT_K4_HI:
	//	appProjectDebugMsg("K4 Hi");
		break;
	case APP_SIGNAL_FB_EVT_K4_LO:
	//	appProjectDebugMsg("K4 Low");
		break;
	case APP_SIGNAL_FB_EVT_DOCP_HI:
		//appProjectDebugMsg("DOCP Hi");
		//releaseOCP();
		break;
	case APP_SIGNAL_FB_EVT_DOCP_LO:
		//appProjectDebugMsg("DOCP Lo");
		break;
	case APP_SIGNAL_FB_EVT_COCP_HI:
		//appProjectDebugMsg("COCP Hi");
		//releaseOCP();
		break;
	case APP_SIGNAL_FB_EVT_COCP_LO:
		//appProjectDebugMsg("COCP Low");
		break;
	case APP_SIGNAL_FB_EVT_OD_IN_HI:
		//appProjectDebugMsg("OD IN Hi");
		break;
	case APP_SIGNAL_FB_EVT_OD_IN_LO:
		//appProjectDebugMsg("OD IN Low");
		break;
	case APP_SIGNAL_FB_EVT_NFAULT_HI:
		//appProjectDebugMsg("Nfault Hi");
		break;
	case APP_SIGNAL_FB_EVT_NFAULT_LO:
		//appProjectDebugMsg("Nfault Lo");
		break;
	}
}
static void afeEventHandler(void *pDest, uint16_t evt, void *pData)
{
	switch(evt)
	{
	case AFE_EVT_COMM_L1_SET:
		appProjectDebugMsg("AFE_EVT_COMM_L1_SET");
		saveEventLog(EVENT_TYPE_AFE_COMM_L1_SET, 0);
		break;
	case AFE_EVT_COMM_L1_RELEASE:
		appProjectDebugMsg("AFE_EVT_COMM_L1_RELEASE");
		saveEventLog(EVENT_TYPE_AFE_COMM_L1_RELEASE, 0);
		break;
	case AFE_EVT_COMM_L2_SET:
		appProjectDebugMsg("AFE_EVT_COMM_L2_SET");
		saveEventLog(EVENT_TYPE_AFE_COMM_L2_SET, 0);
		relayOff();
		break;
	case AFE_EVT_COMM_L2_RELEASE:
		appProjectDebugMsg("AFE_EVT_COMM_L2_RELEASE");
		saveEventLog(EVENT_TYPE_AFE_COMM_L2_RELEASE, 0);
		
		break;
	}
}
static void buttonEventHandler(void *pDest, uint16_t evt, void *pData)
{
	switch(evt)
	{
	case BUTTON_EVT_CLICK_KEY_0:
		appProjectDebugMsg("BUTTON_EVT_CLICK_KEY_0");
		break;
	case BUTTON_EVT_LONG_PRESS_KEY_0:
		appProjectDebugMsg("BUTTON_EVT_LONG_PRESS_KEY_0");
		break;
	case BUTTON_EVT_PRESS_1S_KEY_0:
		appProjectDebugMsg("BUTTON_EVT_PRESS_1S_KEY_0");
		break;
	case BUTTON_EVT_PRESS_KEY_0:
		appProjectDebugMsg("BUTTON_EVT_PRESS_KEY_0");
		break;
	case BUTTON_EVT_RELEASE_KEY_0:	
		appProjectDebugMsg("BUTTON_EVT_RELEASE_KEY_0");
		break;
	}
}


static void releaseOCP(void);

static void appProjectSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	static	uint8_t		SystemReadyCount = 10;
//	tHalRtcDateTime	mHalRtcDateTime;
	char	str[100];

    if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
//		GPIOD->ODR ^= GPIO_PIN_14;
			;
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		if(SystemReadyCount)
		{
			SystemReadyCount--;
			if(!SystemReadyCount)
				SystemReadyFlag = 1;
		}
		apiRamSaveRtcDateTime();
		
		//sprintf(str,"EPO Status ......%d",	HalBspGetEpoStatus());
		//appProjectDebugMsg(str);
	}
}

static void appProjectHwTimerHandler(void *pin, uint16_t evt, void *pData)
{
	LibSwTimerHwHandler(LIB_SW_TIMER_EVT_HW_1MS, 0);
	LibHwTimerHandle();
}
void DumpBuffer(uint8_t *pBuf,uint16_t len)
{
	uint16_t	i;
	char	str[100];
	char	str1[10];
	
	str[0] = 0;
	sprintf(str,"0000 ");
	for(i=0; i<len; i++)
	{
		if((i & 0x0f) == 0)
		{
			sprintf(str,"%.4X ", i);
		}
		sprintf(str1,"%.2X ",pBuf[i]);
		strcat(str, str1);
		
		if((i&0x0f) == 0x0f)
		{
			appProjectDebugMsg(str);
			str[0] = 0;	
		}
		
	}
	if(str[0])
		appProjectDebugMsg(str);
}

void EepromFunctionTest(void)
{
	tHalEeProm	mHalEeProm;
	uint8_t		buffer[512];
	uint16_t	i;
	
	mHalEeProm.StartAddress = 0x08000000L + 510L * 1024L;
	mHalEeProm.Length = 32;
	mHalEeProm.pDataBuffer = buffer;

	appProjectDebugMsg("----Erase");
	HalEePromRead(&mHalEeProm);
	DumpBuffer(buffer, 32);

	if(HalEePromErase(&mHalEeProm) != 0)
	{
		appProjectDebugMsg("Erase Error");
	}
	HalEePromRead(&mHalEeProm);
	DumpBuffer(buffer, 32);
  	appProjectDebugMsg("----Write");
	for(i=0; i<100; i++)
		buffer[i] = i;	
	HalEePromWrite(&mHalEeProm);
	//	 HAL_FLASH_Lock();
  	appProjectDebugMsg("----Read"); 
	HalEePromRead(&mHalEeProm);
	DumpBuffer(buffer, 32);
}

void NtcTest(void)
{
	int 	t;
	int16_t	temp;
	double 	v,R;
	char	str[100];
	uint16_t	d;
	
	for(t=-40; t<= 105; t++)
	{
		temp = t + 40;
		v = LibTemperatureToVoltage(temp);
		R = (10000.0 * v) / (5000 - v);
		temp = LibNtcVoltageToTemperature(v);

	
		sprintf(str,"T=%d =>%f %f %f", t, v, R, ((double)temp-4000.0)/100);
		appSerialCanDavinciSendTextMessage(str);
		

		for(d=0; d<50000;d++);
	}
}



static uint8_t	ocpCount;
static void ocpReleaseSwTimer(__far void *dest, uint16_t evt, void *vDataPtr)
{
//	GPIOD->ODR |= GPIO_PIN_14;
    if(evt == LIB_SW_TIMER_EVT_SW_1MS)
    {
    	ocpCount++;
    	if(ocpCount >= 20)
    	{
			HalBspReleaseCtrl(0);
	  		LibSwTimerClose(ocpReleaseSwTimer, 0);
	  	}
	}
//	GPIOD->ODR &= ~GPIO_PIN_14;
}
static void releaseOCP(void)
{
	ocpCount = 0;
	HalBspReleaseCtrl(1);
  	LibSwTimerOpen(ocpReleaseSwTimer, 0);
}

/* Public function prototypes -----------------------------------------------*/
uint8_t appProjectIsSystemReadyFlag(void)
{
	return SystemReadyFlag;
}
uint8_t appProjectGetRelayOnFlag(void)
{
	return RelayOnFlag;
}
void appProjectSetRelayOnFlag(void)
{
	RelayOnFlag = 1;
}

uint8_t appProjectGetScuId(void)
{
	//return ScuId;
	return appBmsGetScuId();
}

uint8_t appProjectIsInSimuMode(void)
{
	if(ModeFlag & APP_PROJECT_SIMU_MODE)
		return 1;
	else
		return 0;
}
void appProjectEnableSimuMode(void)
{
	ModeFlag |= APP_PROJECT_SIMU_MODE;
}
void appProjectDisableSimuMode(void)
{
	ModeFlag &= ~APP_PROJECT_SIMU_MODE;
}
uint8_t appProjectIsInEngMode(void)
{
	if(ModeFlag & APP_PROJECT_ENG_MODE)
		return 1;
	else
		return 0;
}
void appProjectEnableEngMode(void)
{
	if((ModeFlag & APP_PROJECT_ENG_MODE) == 0)
	{
	
		
	}	
	ModeFlag |= APP_PROJECT_ENG_MODE;
}
void appProjectDisableEngMode(void)
{
	if((ModeFlag & APP_PROJECT_ENG_MODE) != 0)
	{
		
		
	}
	ModeFlag &= ~APP_PROJECT_ENG_MODE;
}


uint8_t	appProjectIsRtcValid(void)
{
	return RtcValid;
}

uint16_t appProjectGetTimerCount(void)
{
	return halTimerGetCountValue(&mHalTimer3);
}

void appProjectOpen(void){
	char	str[100];
	uint32_t	len;
	 
	HalRtcOpen();
	apiRamOpen();
	HalTimerOpen(&mHalTimer3, appProjectHwTimerHandler);
	appSerialUartDavinciOpen();
	appSerialCanDavinciOpen();
	appTcpipSmpOpen();
  	//------------------------------------------
	appProjectDebugMsg("--------- Start Run -----------...9");
	len = apiSysParOpen();
	
	appProtectOpen(protectEventHandler);
	appGaugeOpen(gaugeEventHandler);
	halafeOpen(afeEventHandler);
	halAfeCurrentOpen();
	Hal_W5500_Open();

	halSpiromOpen();
	
  	//------------------------------------------
	//appProjectDebugMsg("Start Run !!");

  	//-----------------------
  	//	rtc test
	//EepromFunctionTest();

	sprintf(str,"Par Len = %d", len);
	appProjectDebugMsg(str);
	appBalanceOpen(appProjectDavinciBalanceEventHandler);
	apiSignalFeedbackOpen(signalFeedbackEventHandler);
  	LibSwTimerOpen(appProjectSwTimerHandler, 0);
  	releaseOCP(); 
  	apiRelayControlOpen();
  	appBmsOpen();
  	apiScuTempOpen();
  	apiSystemFlagOpen();
  	apiEventLogOpen();
	appButtonOpen(buttonEventHandler);
	
	appTcpipSmpOpen();
	IrFunctionOpen();
	
//HalBspReleaseCtrl
//	NtcTest();
	{
		uint32_t	sec,rtc;
		tHalRtcDateTime	mRtcDateTime;

		sec = apiRamLoadPowerOffDateTime();
		HalRtcSmpUnixTimeToDateTime(sec, &mRtcDateTime);

		rtc = HalRtcGetSmpUnixTime();
		if(rtc >= sec)
		{
			rtc -= sec;
		}
		else
			rtc = 0;
		sprintf(str,"距上次關機 %d Sec %d %.4d/%.2d/%.2d %.2d:%.2d:%.2d",
							rtc, sec,
							mRtcDateTime.Year,
							mRtcDateTime.Month,
							mRtcDateTime.Day,
							mRtcDateTime.Hour,
							mRtcDateTime.Minute,
							mRtcDateTime.Second
							);
		appProjectDebugMsg(str);		
		
		sec = apiRamLoadReleaseTime();
		sprintf(str,"距上次充放電時間 %d Sec",
							sec	);
		appProjectDebugMsg(str);			
		
		
		sprintf(str,"累計放電 %d mAh",apiRamLoadTotalDisChargeCount());
		appProjectDebugMsg(str);
	}
	
	if(apiRamLoadRtcMagicCode() != 0x5ACD)
	{
		apiRamSaveRtcMagicCode(0x5ACD);
		appProjectDebugMsg("Rtc Invalid");
		RtcValid = 0;
	}
	else
	{
		appProjectDebugMsg("Rtc valid");
		RtcValid = 1;
	}

}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


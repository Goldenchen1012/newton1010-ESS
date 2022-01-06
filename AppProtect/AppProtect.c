/**
  ******************************************************************************
  * @file        AppProtech.c
  * @author      Johnny
  * @version     v0.0
  * @date        2021/9/7
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
#include "define.h"
#include "main.h"
#include "halafe.h"
#include "AppProtect.h"
#include "ApiProtectUvp.h"
#include "ApiProtectOvp.h"
#include "ApiProtectCotp.h"
#include "ApiProtectCutp.h"
#include "ApiProtectDotp.h"
#include "ApiProtectDutp.h"
#include "ApiProtectCocp.h"
#include "ApiProtectDocp.h"
#include "LibSwTimer.h"

/* Private define ------------------------------------------------------------*/
#define	PROTECT_INTERVAL_10MS	50

//#define	USE_TEMP_VALUE_FOR_TEMP_COMPARE
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef void (* tProtectRunTable)(void);

/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t	ProtectFunIndex = 0;

/* Private function prototypes -----------------------------------------------*/
static void checkNextProtectFunction(void)
{
	ProtectFunIndex++;
}

static void appProtectNone(void)
{
	checkNextProtectFunction();
}

static void appProtectOvp_L1(void)
{
	if(apiProtectOvpHandler(0))
		checkNextProtectFunction();
}
static void appProtectOvp_L2(void)
{
	if(apiProtectOvpHandler(1))
		checkNextProtectFunction();
}

static void appProtectOvp_L3(void)
{
	if(apiProtectOvpHandler(2))
		checkNextProtectFunction();
}
static void appProtectOvPf(void)
{
	apiProtectOvpPfHandler();
	checkNextProtectFunction();
}

static void appProtectUvp_L1(void)
{
	if(apiProtectUvpHandler(0))
		checkNextProtectFunction();
}
static void appProtectUvp_L2(void)
{
	if(apiProtectUvpHandler(1))
		checkNextProtectFunction();
}

static void appProtectUvp_L3(void)
{
	if(apiProtectUvpHandler(2))
		checkNextProtectFunction();
}
static void appProtectUvpPf(void)
{
	apiProtectUvpPfHandler();
	checkNextProtectFunction();
}

//---------------------------------
//	cotp
static void appProtectCotp_L1(void)
{
	if(apiProtectCotpHandler(0))
		checkNextProtectFunction();
}
static void appProtectCotp_L2(void)
{
	if(apiProtectCotpHandler(1))
		checkNextProtectFunction();
}

static void appProtectCotp_L3(void)
{
	if(apiProtectCotpHandler(2))
		checkNextProtectFunction();
}

//---------------------------------
//	cutp
static void appProtectCutp_L1(void)
{
	if(apiProtectCutpHandler(0))
		checkNextProtectFunction();
}
static void appProtectCutp_L2(void)
{
	if(apiProtectCutpHandler(1))
		checkNextProtectFunction();
}

static void appProtectCutp_L3(void)
{
	if(apiProtectCutpHandler(2))
		checkNextProtectFunction();
}

//---------------------------------
//	dotp
static void appProtectDotp_L1(void)
{
	if(apiProtectDotpHandler(0))
		checkNextProtectFunction();
}
static void appProtectDotp_L2(void)
{
	if(apiProtectDotpHandler(1))
		checkNextProtectFunction();
}

static void appProtectDotp_L3(void)
{
	if(apiProtectDotpHandler(2))
		checkNextProtectFunction();
}
//---------------------------------
//	dutp
static void appProtectDutp_L1(void)
{
	if(apiProtectDutpHandler(0))
		checkNextProtectFunction();
}
static void appProtectDutp_L2(void)
{
	if(apiProtectDutpHandler(1))
		checkNextProtectFunction();
}

static void appProtectDutp_L3(void)
{
	if(apiProtectDutpHandler(2))
		checkNextProtectFunction();
}
//---------------------------------
//	cocp
static void appProtectCocp_L1(void)
{
	apiProtectCocpHandler(0);
	checkNextProtectFunction();
}
static void appProtectCocp_L2(void)
{
	apiProtectCocpHandler(1);
	checkNextProtectFunction();
}

static void appProtectCocp_L3(void)
{
	apiProtectCocpHandler(2);
	checkNextProtectFunction();
}
//---------------------------------
//	docp
static void appProtectDocp_L1(void)
{
	apiProtectDocpHandler(0);
	checkNextProtectFunction();
}
static void appProtectDocp_L2(void)
{
	apiProtectDocpHandler(1);
	checkNextProtectFunction();
}

static void appProtectDocp_L3(void)
{
	apiProtectDocpHandler(2);
	checkNextProtectFunction();
}
//---------------------------------
static void appProtectEnd(void)
{
	ProtectFunIndex	= 0;
}


const tProtectRunTable	ProtectRunTable[]={
	appProtectNone,
#if	1	
	appProtectOvp_L1,
	appProtectOvp_L2,
	appProtectOvp_L3,
	appProtectOvPf,
#endif
#if	1	
	appProtectUvp_L1,
	appProtectUvp_L2,
	appProtectUvp_L3,
	appProtectUvpPf,
#endif	
#if	1
	appProtectCotp_L1,
	appProtectCotp_L2,
	appProtectCotp_L3,
#endif	
#if 1
	appProtectCutp_L1,
	appProtectCutp_L2,
	appProtectCutp_L3,
#endif
#if	1
	appProtectDotp_L1,
	appProtectDotp_L2,
	appProtectDotp_L3,
#endif	
#if 1
	appProtectDutp_L1,
	appProtectDutp_L2,
	appProtectDutp_L3,
#endif
#if 1
	appProtectCocp_L1,
	appProtectCocp_L2,
	appProtectCocp_L3,
#endif
#if 1
	appProtectDocp_L1,
	appProtectDocp_L2,
	appProtectDocp_L3,
#endif
	appProtectEnd
};

/* Public function prototypes -----------------------------------------------*/
void appProtectGetLevelMask(uint8_t Level, tProtectFlagValue *pProtectFlagValue)
{
	const uint8_t LevelTable[][4]={
			{PROTECT_FLAG_L1_MASK,	PROTECT_FLAG_L1_SETTING,	PROTECT_FLAG_L1_SETTED,	PROTECT_FLAG_L1_REALSING},
			{PROTECT_FLAG_L2_MASK,	PROTECT_FLAG_L2_SETTING,	PROTECT_FLAG_L2_SETTED,	PROTECT_FLAG_L2_REALSING},
			{PROTECT_FLAG_L3_MASK,	PROTECT_FLAG_L3_SETTING,	PROTECT_FLAG_L3_SETTED,	PROTECT_FLAG_L3_REALSING},
			{PROTECT_FLAG_L4_MASK,	PROTECT_FLAG_L4_SETTING,	PROTECT_FLAG_L4_SETTED,	PROTECT_FLAG_L4_REALSING}
		};
	
	pProtectFlagValue->Mask = LevelTable[Level][0];
	pProtectFlagValue->ClearMask = ~ pProtectFlagValue->Mask;
	pProtectFlagValue->Setting = LevelTable[Level][1];
	pProtectFlagValue->Setted = LevelTable[Level][2];
	pProtectFlagValue->Releasing = LevelTable[Level][3];
}

uint8_t	appProtectIsUnderTemperter(tNtcAdcData NtcAdcValue, tNtcAdcData CompareAdcValue)
{
#ifdef USE_TEMP_VALUE_FOR_TEMP_COMPARE	
	if(NtcAdcValue < CompareAdcValue)
#else
	if(NtcAdcValue > CompareAdcValue)
#endif	
	{
		return 1;
	}
	else
		return 0;
}

uint8_t	appProtectIsOverTemperter(tNtcAdcData NtcAdcValue, tNtcAdcData CompareAdcValue)
{
#ifdef USE_TEMP_VALUE_FOR_TEMP_COMPARE	
	if(NtcAdcValue > CompareAdcValue)
#else	
	if(NtcAdcValue < CompareAdcValue)
#endif
	{	
		return 1;
	}
	else
		return 0;
}

void appProtectSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	uint8_t	state;
	static uint8_t	count = 0;
	
    if(evt == LIB_SW_TIMER_EVT_SW_1MS)
    {
    	if(ProtectFunIndex)
		{
			//GPIOD->ODR |= GPIO_PIN_14;
			(*ProtectRunTable[ProtectFunIndex])();	
			//ProtectFunIndex++;
			//GPIOD->ODR &= ~GPIO_PIN_14;
		}
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_8)
	{
		count++;
		if(count >= PROTECT_INTERVAL_10MS)
		{
			count = 0;
			if(appProjectIsInSimuMode()==0 &&  halAfeGetState() != AFE_STATE_NORMAL)
				return;
			else			
				ProtectFunIndex = 1;
		}
	}
}
void appProtectOpen(tAppProtectEvtHandler evtHandler)
{	
  	LibSwTimerOpen(appProtectSwTimerHandler, 0);
	apiProtectOvpOpen(evtHandler);
	apiProtectUvpOpen(evtHandler);
	apiProtectCotpOpen(evtHandler);
	apiProtectCutpOpen(evtHandler);
	apiProtectDotpOpen(evtHandler);
	apiProtectCocpOpen(evtHandler);
	apiProtectDocpOpen(evtHandler);
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



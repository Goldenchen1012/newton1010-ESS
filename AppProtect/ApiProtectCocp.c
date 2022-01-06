/**
  ******************************************************************************
  * @file        ApiProtechCocp.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/23
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
#include "halafe.h"
#include "AppProtect.h"
#include "ApiProtectCocp.h"
#include "ApiSysPar.h"
#include "AppGauge.h"

void appSerialCanDavinciSendTextMessage(uint8_t *str);
#define	appProtectCocpDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t	Flag;
	uint8_t	SetCount[PROTECT_LEVEL];
	uint8_t	ReleaseCount[PROTECT_LEVEL];
	tAppProtectEvtHandler  EvtHandler;
}tCocpProtect;

static tCocpProtect	mCocpProtect={0};
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
uint8_t	apiProtectCocpGetFlag(void)
{
	return mCocpProtect.Flag;
}
uint8_t apiProtectCocpHandler(uint8_t ProtectLevel)
{
	BYTE	str[200];

	static 	uint8_t	flag = 0;
	uint16_t		value;
	tCurrent		CurrentValue;
	tProtectFlagValue	ProtectFlagValue;
	tScuProtectPar	ProtectPar;
	
	
	apiSysParGetCocpPar(ProtectLevel, &ProtectPar);
	appProtectGetLevelMask(ProtectLevel, &ProtectFlagValue);
	CurrentValue = abs(appGaugeGetCurrentValue()) / 1000;


	if(appGaugeGetCurrentMode() != APP_SCU_GAUGE_CHARGE_MODE)
		CurrentValue = 0;

	if(CurrentValue > ProtectPar.SetValue.l)
	{
		if((mCocpProtect.Flag & ProtectFlagValue.Mask) == 0)
		{
			mCocpProtect.Flag &= ProtectFlagValue.ClearMask;
			mCocpProtect.Flag |= ProtectFlagValue.Setting;
			mCocpProtect.SetCount[ProtectLevel] = 1;			
		}
		else if((mCocpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
		{
			mCocpProtect.SetCount[ProtectLevel]++;
			if(mCocpProtect.SetCount[ProtectLevel] >= ProtectPar.STime.l)
			{
				if(mCocpProtect.EvtHandler)
				{
					value = CurrentValue;
					mCocpProtect.EvtHandler(0, APP_PROTECT_COCP_L1_SET + ProtectLevel, &value);	
				}	
				mCocpProtect.Flag &= ProtectFlagValue.ClearMask;
				mCocpProtect.Flag |= ProtectFlagValue.Setted;
				mCocpProtect.SetCount[ProtectLevel] = 0;
				//appProtectCocpDebugMsg("COCP Set");
			}
		}
	}
	else if((mCocpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
	{
		mCocpProtect.Flag &= ProtectFlagValue.ClearMask;
	}
	//-----------------------------------------
	//	Level1	Release
//	if(level==2)
//		continue;
	if(CurrentValue < ProtectPar.RelValue.l)
	{
		if((mCocpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Setted)	//Seted!!
		{
			mCocpProtect.Flag &= ProtectFlagValue.ClearMask;
			mCocpProtect.Flag |= ProtectFlagValue.Releasing;
			mCocpProtect.ReleaseCount[ProtectLevel] = 1;
		}
		else if((mCocpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
		{
			mCocpProtect.ReleaseCount[ProtectLevel] ++;
			if(mCocpProtect.ReleaseCount[ProtectLevel]  >=  ProtectPar.RTime.l)
			{
				if(mCocpProtect.EvtHandler)
				{
					value = CurrentValue;
					mCocpProtect.EvtHandler(0, APP_PROTECT_COCP_L1_RELEASE + ProtectLevel, &value);
				}	
				mCocpProtect.Flag &= ProtectFlagValue.ClearMask;
				mCocpProtect.ReleaseCount[ProtectLevel] = 0;
				//appProtectCocpDebugMsg("COCP Release");

			}		
		}
	}
	else if((mCocpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
	{
		mCocpProtect.Flag &= ProtectFlagValue.ClearMask;
		mCocpProtect.Flag |= ProtectFlagValue.Setted;
 	}	
	return 1;
}

void apiProtectCocpOpen(tAppProtectEvtHandler evtHandler)
{
	mCocpProtect.EvtHandler = evtHandler;
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


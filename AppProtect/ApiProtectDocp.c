/**
  ******************************************************************************
  * @file        ApiProtechDocp.c
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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "define.h"
#include "halafe.h"
#include "AppProtect.h"
#include "ApiProtectDocp.h"
#include "ApiSysPar.h"
#include "AppGauge.h"

void appSerialCanDavinciSendTextMessage(uint8_t *str);
#define	appProtectDocpDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t	Flag;
	uint8_t	SetCount[PROTECT_LEVEL];
	uint8_t	ReleaseCount[PROTECT_LEVEL];
	tAppProtectEvtHandler  EvtHandler;
}tDocpProtect;

static tDocpProtect	mDocpProtect={0};
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
uint8_t	apiProtectDocpGetFlag(void)
{
	return mDocpProtect.Flag;
}

uint8_t apiProtectDocpHandler(uint8_t ProtectLevel)
{
	char		str[100];
	uint16_t		value;
	tCurrent		CurrentValue;
	tProtectFlagValue	ProtectFlagValue;
	tScuProtectPar		ProtectPar;
	
	apiSysParGetDocpPar(ProtectLevel, &ProtectPar);
	appProtectGetLevelMask(ProtectLevel, &ProtectFlagValue);

	CurrentValue = abs(appGaugeGetCurrentValue()) / 1000;

	if(appGaugeGetCurrentMode() != APP_SCU_GAUGE_DISCHARGE_MODE)
		CurrentValue = 0;

//	sprintf(str,"DOCP %d %d %d %d %d %d",ProtectLevel,
//					ProtectPar.SetValue.l,	ProtectPar.STime.l,
//					ProtectPar.RelValue.l,ProtectPar.RTime.l,
//					CurrentValue);
	//appProtectDocpDebugMsg(str);

//	if(InEngineerModeFlag)
//		return;	
//	GetMaskData(level);
	
		//if(ChargeMode[area]==CHARGE_MODE)		//¥¿¦b¥R¹q
		//	Current=0;
		//else
		//	Current=(SystemParameter.Current_uA[area*2].l/1000000L);
		
	if(CurrentValue > ProtectPar.SetValue.l)
	{
		if((mDocpProtect.Flag & ProtectFlagValue.Mask) == 0)
		{
			mDocpProtect.Flag &= ProtectFlagValue.ClearMask;
			mDocpProtect.Flag |= ProtectFlagValue.Setting;
			mDocpProtect.SetCount[ProtectLevel] = 1;			
		}
		else if((mDocpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
		{
			mDocpProtect.SetCount[ProtectLevel]++;
			if(mDocpProtect.SetCount[ProtectLevel] >= ProtectPar.STime.l)
			{
				if(mDocpProtect.EvtHandler)
				{
					value = CurrentValue;
					mDocpProtect.EvtHandler(0, APP_PROTECT_DOCP_L1_SET + ProtectLevel, &value);
				}
				mDocpProtect.Flag &= ProtectFlagValue.ClearMask;
				mDocpProtect.Flag |= ProtectFlagValue.Setted;
				mDocpProtect.SetCount[ProtectLevel] = 0;
			}
		}
	}
	else if((mDocpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
	{
		mDocpProtect.Flag &= ProtectFlagValue.ClearMask;
	}	
	//-----------------------------------------
	//	Level1	Release
	if(CurrentValue < ProtectPar.RelValue.l)
	{
		if((mDocpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Setted)
		{
			mDocpProtect.Flag &= ProtectFlagValue.ClearMask;
			mDocpProtect.Flag |= ProtectFlagValue.Releasing;
			mDocpProtect.ReleaseCount[ProtectLevel] = 1;		
		}
		else if((mDocpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
		{
			mDocpProtect.ReleaseCount[ProtectLevel] ++;
			if(mDocpProtect.ReleaseCount[ProtectLevel]  >=  ProtectPar.RTime.l)
			{
				if(mDocpProtect.EvtHandler)
				{
					value = CurrentValue;
					mDocpProtect.EvtHandler(0, APP_PROTECT_DOCP_L1_RELEASE + ProtectLevel, &value);	
				}	
				mDocpProtect.Flag &= ProtectFlagValue.ClearMask;
				mDocpProtect.ReleaseCount[ProtectLevel] = 0;
			}		
		}
	}
	else if((mDocpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
	{
		mDocpProtect.Flag &= ProtectFlagValue.ClearMask;
		mDocpProtect.Flag |= ProtectFlagValue.Setted;
 	}	
	
	return 1;
}

void apiProtectDocpOpen(tAppProtectEvtHandler evtHandler)
{
	mDocpProtect.EvtHandler = evtHandler;
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


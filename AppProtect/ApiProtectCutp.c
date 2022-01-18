/**
  ******************************************************************************
  * @file        ApiProtechCutp.c
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "define.h"
#include "halafe.h"
#include "AppProtect.h"
#include "ApiProtectCutp.h"
#include "ApiSysPar.h"

void appSerialCanDavinciSendTextMessage(uint8_t *str);
#define	appProtectCutpDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private define ------------------------------------------------------------*/
#define	CUTP_CHECK_COUNT	20
#define	cutpNtcNumber()		apiSysParGetNtcNumber()
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t	Flag[MAX_NTC_NUMBER];
	uint8_t	SetCount[PROTECT_LEVEL][MAX_NTC_NUMBER];
	uint8_t	ReleaseCount[PROTECT_LEVEL][MAX_NTC_NUMBER];
	tAppProtectEvtHandler  EvtHandler;
}tCutpProtect;

static tCutpProtect	mCutpProtect={0};
static	uint16_t	CutpNtcIndex;
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void cutpProtectIni(void)
{
	CutpNtcIndex = 0;
}
/* Public function prototypes -----------------------------------------------*/
uint8_t	apiProtectCutpGetFlag(uint16_t NtcIndex)
{
	return mCutpProtect.Flag[NtcIndex];
}


uint8_t apiProtectCutpHandler(uint8_t ProtectLevel)
{
//	char	str[200];
//	static uint8_t	flag = 0;
//	if(InEngineerModeFlag)
//		return;	
	uint8_t		checkcount = 0;
	tNtcAdcData		NtcAdcValue;
	tProtectFlagValue	ProtectFlagValue;
	tScuProtectPar		ProtectPar;

	apiSysParGetCutpProtectPar(ProtectLevel, &ProtectPar);
	appProtectGetLevelMask(ProtectLevel, &ProtectFlagValue);
	
//	sprintf(str,"CUTP %d %d %d %d %d",ProtectLevel,
//					ProtectPar.SetValue.l,	ProtectPar.STime.l,
//					ProtectPar.RelValue.l,ProtectPar.RTime.l);
//	if(!flag)					
//		appProtectCutpDebugMsg(str);

	while(1)
	{			
		NtcAdcValue = HalAfeGetNtcAdc(CutpNtcIndex);

//		sprintf(str,"%d %d",CutpNtcIndex, NtcAdcValue);
//		if(!flag)
//			appProtectCutpDebugMsg(str);
//		flag = 1;
		
		if(appProtectIsUnderTemperter(NtcAdcValue, ProtectPar.SetValue.l) && ProtectPar.STime.l)
		{
			if((mCutpProtect.Flag[CutpNtcIndex] & ProtectFlagValue.Mask) == 0)
			{
				mCutpProtect.Flag[CutpNtcIndex] &= ProtectFlagValue.ClearMask;
				mCutpProtect.Flag[CutpNtcIndex] |= ProtectFlagValue.Setting;
				mCutpProtect.SetCount[ProtectLevel][CutpNtcIndex] = 1;
			}	
			else if((mCutpProtect.Flag[CutpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
			{
				mCutpProtect.SetCount[ProtectLevel][CutpNtcIndex]++;
				if(mCutpProtect.SetCount[ProtectLevel][CutpNtcIndex] >= ProtectPar.STime.l)
				{
					mCutpProtect.Flag[CutpNtcIndex] &= ProtectFlagValue.ClearMask;
					mCutpProtect.Flag[CutpNtcIndex] |= ProtectFlagValue.Setted;
					mCutpProtect.SetCount[ProtectLevel][CutpNtcIndex] = 0;

					if(mCutpProtect.EvtHandler)
					{
						mCutpProtect.EvtHandler(0, APP_PROTECT_CUTP_L1_SET + ProtectLevel, &CutpNtcIndex);	
					}
				}
			}
		}
		else if((mCutpProtect.Flag[CutpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
		{
			mCutpProtect.Flag[CutpNtcIndex] &= ProtectFlagValue.ClearMask;
		}
		//--------------------------------------------------------------------------
		//	Level	Release
		if(appProtectIsOverTemperter(NtcAdcValue, ProtectPar.RelValue.l) && ProtectPar.RTime.l)
		{
			if((mCutpProtect.Flag[CutpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setted)
			{
				mCutpProtect.Flag[CutpNtcIndex] &= ProtectFlagValue.ClearMask;
				mCutpProtect.Flag[CutpNtcIndex] |= ProtectFlagValue.Releasing;
				mCutpProtect.ReleaseCount[ProtectLevel][CutpNtcIndex] = 1;
			}	
			else if((mCutpProtect.Flag[CutpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
			{
				mCutpProtect.ReleaseCount[ProtectLevel][CutpNtcIndex]++;
				if(mCutpProtect.ReleaseCount[ProtectLevel][CutpNtcIndex] >= ProtectPar.RTime.l)
				{
					mCutpProtect.Flag[CutpNtcIndex] &= ProtectFlagValue.ClearMask;
					mCutpProtect.ReleaseCount[ProtectLevel][CutpNtcIndex] = 0;
					if(mCutpProtect.EvtHandler)
					{
						mCutpProtect.EvtHandler(0, APP_PROTECT_CUTP_L1_RELEASE + ProtectLevel, &CutpNtcIndex);	
					}
				}
			}
		}
		else if((mCutpProtect.Flag[CutpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
		{
			mCutpProtect.Flag[CutpNtcIndex] &= ProtectFlagValue.ClearMask;
			mCutpProtect.Flag[CutpNtcIndex] |= ProtectFlagValue.Setted;
		}
		CutpNtcIndex++;
		if(CutpNtcIndex >= cutpNtcNumber())
		{
			CutpNtcIndex = 0;
			return 1;
		}
		checkcount++;
		if(checkcount >= CUTP_CHECK_COUNT)
			break;
	}
	return 0;
}

void apiProtectCutpOpen(tAppProtectEvtHandler evtHandler)
{
	cutpProtectIni();
	
	mCutpProtect.EvtHandler = evtHandler;
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


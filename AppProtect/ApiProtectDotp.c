/**
  ******************************************************************************
  * @file        ApiProtechDotp.c
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
#include "ApiProtectDotp.h"
#include "ApiSysPar.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	appProtectDotpDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)


/* Private define ------------------------------------------------------------*/
#define	DOTP_CHECK_COUNT	20
#define	dotpNtcNumber()		apiSysParGetNtcNumber()
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t	Flag[MAX_NTC_NUMBER];
	uint8_t	SetCount[PROTECT_LEVEL][MAX_NTC_NUMBER];
	uint8_t	ReleaseCount[PROTECT_LEVEL][MAX_NTC_NUMBER];
	tAppProtectEvtHandler  EvtHandler;
}tDotpProtect;

static tDotpProtect	mDotpProtect={0};
static	uint16_t	DotpNtcIndex;
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static void dotpProtectIni(void)
{
	DotpNtcIndex = 0;	
}
/* Private function prototypes -----------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
uint8_t	apiProtectDotpGetFlag(uint16_t NtcIndex)
{
	return mDotpProtect.Flag[NtcIndex];
}


uint8_t apiProtectDotpHandler(uint8_t ProtectLevel)
{
	BYTE	str[200];
	uint8_t	checkcount;
//	if(InEngineerModeFlag)
//		return;	
	static 	uint8_t		flag = 0;
	uint16_t		NtcAdcValue;
	tProtectFlagValue	ProtectFlagValue;
	tScuProtectPar		ProtectPar;
	
	apiSysParGetDotpProtectPar(ProtectLevel, &ProtectPar);
	appProtectGetLevelMask(ProtectLevel, &ProtectFlagValue);


//	sprintf(str,"DOTP %d %d %d %d %d",ProtectLevel,
//					ProtectPar.SetValue.l,	ProtectPar.STime.l,
//					ProtectPar.RelValue.l,ProtectPar.RTime.l);
//	if(!flag)					
//		appProtectDotpDebugMsg(str);

	while(1)
	{			
		NtcAdcValue = HalAfeGetNtcAdc(DotpNtcIndex);
//		sprintf(str,"%d %d",DotpNtcIndex, NtcAdcValue);
//		if(!flag)
//			appProtectDotpDebugMsg(str);
//		flag = 1;

		if(appProtectIsOverTemperter(NtcAdcValue, ProtectPar.SetValue.l))
		{
			if((mDotpProtect.Flag[DotpNtcIndex] & ProtectFlagValue.Mask) == 0)
			{
				mDotpProtect.Flag[DotpNtcIndex] &= ProtectFlagValue.ClearMask;
				mDotpProtect.Flag[DotpNtcIndex] |= ProtectFlagValue.Setting;
				mDotpProtect.SetCount[ProtectLevel][DotpNtcIndex] = 1;
			}	
			else if((mDotpProtect.Flag[DotpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
			{
				mDotpProtect.SetCount[ProtectLevel][DotpNtcIndex]++;
				if(mDotpProtect.SetCount[ProtectLevel][DotpNtcIndex] >= ProtectPar.STime.l)
				{
					mDotpProtect.Flag[DotpNtcIndex] &= ProtectFlagValue.ClearMask;
					mDotpProtect.Flag[DotpNtcIndex] |= ProtectFlagValue.Setted;
					mDotpProtect.SetCount[ProtectLevel][DotpNtcIndex] = 0;

					if(mDotpProtect.EvtHandler)
					{
						mDotpProtect.EvtHandler(0, APP_PROTECT_DOTP_L1_SET + ProtectLevel, &DotpNtcIndex);
					}
				}
			}
		}
		else if((mDotpProtect.Flag[DotpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
		{
			mDotpProtect.Flag[DotpNtcIndex] &= ProtectFlagValue.ClearMask;
		}
		//--------------------------------------------------------------------------
		//	Level	Release
		if(appProtectIsUnderTemperter(NtcAdcValue, ProtectPar.RelValue.l))
		{
			if((mDotpProtect.Flag[DotpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setted)
			{
				mDotpProtect.Flag[DotpNtcIndex] &= ProtectFlagValue.ClearMask;
				mDotpProtect.Flag[DotpNtcIndex] |= ProtectFlagValue.Releasing;
				mDotpProtect.ReleaseCount[ProtectLevel][DotpNtcIndex] = 1;
			}	
			else if((mDotpProtect.Flag[DotpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
			{
				mDotpProtect.ReleaseCount[ProtectLevel][DotpNtcIndex]++;
				if(mDotpProtect.ReleaseCount[ProtectLevel][DotpNtcIndex] >= ProtectPar.RTime.l)
				{
					mDotpProtect.Flag[DotpNtcIndex] &= ProtectFlagValue.ClearMask;
					mDotpProtect.ReleaseCount[ProtectLevel][DotpNtcIndex] = 0;
					if(mDotpProtect.EvtHandler)
					{
						mDotpProtect.EvtHandler(0, APP_PROTECT_DOTP_L1_RELEASE + ProtectLevel, &DotpNtcIndex);
					}
				}
			}
		}
		else if((mDotpProtect.Flag[DotpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
		{
			mDotpProtect.Flag[DotpNtcIndex] &= ProtectFlagValue.ClearMask;
			mDotpProtect.Flag[DotpNtcIndex] |= ProtectFlagValue.Setted;
		}
		DotpNtcIndex++;
		if(DotpNtcIndex >= dotpNtcNumber())
		{
			DotpNtcIndex = 0;
			return 1;
		}
		checkcount++;
		if(checkcount >= DOTP_CHECK_COUNT)
			break;
	}

	return 0;
}

void apiProtectDotpOpen(tAppProtectEvtHandler evtHandler)
{
	dotpProtectIni();
	
	mDotpProtect.EvtHandler = evtHandler;
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


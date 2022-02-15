/**
  ******************************************************************************
  * @file        ApiProtechIrRup.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2022/02/15
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "define.h"
#include "halafe.h"
#include "AppProtect.h"
#include "ApiProtectDutp.h"
#include "ApiSysPar.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	apiProtectRupDebugMsg(str)	//appSerialCanDavinciSendTextMessage(str)

/* Private define ------------------------------------------------------------*/
#define	DUTP_CHECK_COUNT	20
#define	dutpNtcNumber()		apiSysParGetNtcNumber()

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t	Flag[MAX_NTC_NUMBER];
	uint8_t	SetCount[PROTECT_LEVEL][MAX_NTC_NUMBER];
	uint8_t	ReleaseCount[PROTECT_LEVEL][MAX_NTC_NUMBER];
	tAppProtectEvtHandler  EvtHandler;
}tDutpProtect;

static tDutpProtect	mDutpProtect={0};
static	uint16_t	DutpNtcIndex;
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static void dutpProtectIni(void)
{
	DutpNtcIndex = 0;
}
/* Private function prototypes -----------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
uint8_t	apiProtectDutpGetFlag(uint16_t NtcIndex)
{
	return mDutpProtect.Flag[NtcIndex];
}


uint8_t apiProtectDutpHandler(uint8_t ProtectLevel)
{
	BYTE	str[200];
	
	uint8_t	checkcount = 0;
	BYTE	ntc;
//	if(InEngineerModeFlag)
//		return;	
	tNtcVoltage		NtcVoltage;
	tProtectFlagValue	ProtectFlagValue;
	tScuProtectPar		ProtectPar;

	apiSysParGetDutpProtectPar(ProtectLevel, &ProtectPar);
	appProtectGetLevelMask(ProtectLevel, &ProtectFlagValue);

//	sprintf(str,"DUTP %d %d %d %d %d",ProtectLevel,
//					ProtectPar.SetValue.l,	ProtectPar.STime.l,
//					ProtectPar.RelValue.l,ProtectPar.RTime.l);
//	appProtectDutpDebugMsg(str);


	while(1)
	{			
		NtcVoltage = HalAfeGetNtcVoltage(DutpNtcIndex);

//		sprintf(str,"%d %d",DutpNtcIndex, NtcVoltage);
//		appProtectDutpDebugMsg(str);
		
		if(appProtectIsUnderTemperter(NtcVoltage, ProtectPar.SetValue.l) && ProtectPar.STime.l)
		{
			if((mDutpProtect.Flag[DutpNtcIndex] & ProtectFlagValue.Mask) == 0)
			{
				mDutpProtect.Flag[DutpNtcIndex] &= ProtectFlagValue.ClearMask;
				mDutpProtect.Flag[DutpNtcIndex] |= ProtectFlagValue.Setting;
				mDutpProtect.SetCount[ProtectLevel][DutpNtcIndex] = 1;
			}	
			else if((mDutpProtect.Flag[DutpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
			{
				mDutpProtect.SetCount[ProtectLevel][DutpNtcIndex]++;
				if(mDutpProtect.SetCount[ProtectLevel][DutpNtcIndex] >= ProtectPar.STime.l)
				{
					mDutpProtect.Flag[DutpNtcIndex] &= ProtectFlagValue.ClearMask;
					mDutpProtect.Flag[DutpNtcIndex] |= ProtectFlagValue.Setted;
					mDutpProtect.SetCount[ProtectLevel][DutpNtcIndex] = 0;

					if(mDutpProtect.EvtHandler)
					{
						mDutpProtect.EvtHandler(0, APP_PROTECT_DUTP_L1_SET + ProtectLevel, &DutpNtcIndex);	
					}
				}
			}
		}
		else if((mDutpProtect.Flag[DutpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
		{
			mDutpProtect.Flag[DutpNtcIndex] &= ProtectFlagValue.ClearMask;
		}
		//--------------------------------------------------------------------------
		//	Level	Release
		if(appProtectIsOverTemperter(NtcVoltage, ProtectPar.RelValue.l) && ProtectPar.RTime.l)
		{
			if((mDutpProtect.Flag[DutpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setted)
			{
				mDutpProtect.Flag[DutpNtcIndex] &= ProtectFlagValue.ClearMask;
				mDutpProtect.Flag[DutpNtcIndex] |= ProtectFlagValue.Releasing;
				mDutpProtect.ReleaseCount[ProtectLevel][DutpNtcIndex] = 1;
			}	
			else if((mDutpProtect.Flag[DutpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
			{
				mDutpProtect.ReleaseCount[ProtectLevel][DutpNtcIndex]++;
				if(mDutpProtect.ReleaseCount[ProtectLevel][DutpNtcIndex] >= ProtectPar.RTime.l)
				{
					mDutpProtect.Flag[DutpNtcIndex] &= ProtectFlagValue.ClearMask;
					mDutpProtect.ReleaseCount[ProtectLevel][DutpNtcIndex] = 0;
					if(mDutpProtect.EvtHandler)
					{
						mDutpProtect.EvtHandler(0, APP_PROTECT_DUTP_L1_RELEASE + ProtectLevel, &DutpNtcIndex);
					}
				}
			}
		}
		else if((mDutpProtect.Flag[DutpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
		{
			mDutpProtect.Flag[DutpNtcIndex] &= ProtectFlagValue.ClearMask;
			mDutpProtect.Flag[DutpNtcIndex] |= ProtectFlagValue.Setted;
		}
		DutpNtcIndex++;
		if(DutpNtcIndex >= dutpNtcNumber())
		{
			DutpNtcIndex = 0;
			return 1;
		}
		checkcount++;
		if(checkcount >= DUTP_CHECK_COUNT)
			break;
	}
	return 0;
}

void apiProtectDutpOpen(tAppProtectEvtHandler evtHandler)
{
	dutpProtectIni();
	
	mDutpProtect.EvtHandler = evtHandler;
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


/**
  ******************************************************************************
  * @file        ApiProtechCotp.c
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
#include "ApiProtectCotp.h"
#include "ApiSysPar.h"

void appSerialCanDavinciSendTextMessage(uint8_t *str);
#define	appProtectCotpDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)


/* Private define ------------------------------------------------------------*/
#define	COTP_CHECK_COUNT	20
#define	cotpNtcNumber()		apiSysParGetNtcNumber()
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t	Flag[MAX_NTC_NUMBER];
	uint8_t	SetCount[PROTECT_LEVEL][MAX_NTC_NUMBER];
	uint8_t	ReleaseCount[PROTECT_LEVEL][MAX_NTC_NUMBER];
	tAppProtectEvtHandler  EvtHandler;
}tCotpProtect;

static tCotpProtect	mCotpProtect={0};
static	uint16_t	CotpNtcIndex;

/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void cotpProtectIni(void)
{
	CotpNtcIndex = 0;
}
/* Public function prototypes -----------------------------------------------*/
uint8_t	apiProtectCotpGetFlag(uint16_t NtcIndex)
{
	return mCotpProtect.Flag[NtcIndex];
}


uint8_t apiProtectCotpHandler(uint8_t ProtectLevel)
{
	BYTE	str[200];
	//static	uint8_t	flag = 0;
	uint8_t		count = 0;
	
	tNtcVoltage		NtcVoltage;
	tProtectFlagValue	ProtectFlagValue;
	tScuProtectPar		ProtectPar;

	apiSysParGetCotpProtectPar(ProtectLevel, &ProtectPar);
	appProtectGetLevelMask(ProtectLevel, &ProtectFlagValue);
	
	
	//sprintf(str,"COTP %d %d %d %d %d",ProtectLevel,
	//				ProtectPar.SetValue.l,	ProtectPar.STime.l,
	//				ProtectPar.RelValue.l,ProtectPar.RTime.l);
	//if(!flag)
	//	appProtectCotpDebugMsg(str);

	while(1)
	{			
		NtcVoltage = HalAfeGetNtcVoltage(CotpNtcIndex);
		
		//sprintf(str,"%d %d",CotpNtcIndex, NtcVoltage);
		//if(!flag)
		//	appProtectCotpDebugMsg(str);
		//flag = 1;
		if(appProtectIsOverTemperter(NtcVoltage, ProtectPar.SetValue.l) && ProtectPar.STime.l)
		{
			if((mCotpProtect.Flag[CotpNtcIndex] & ProtectFlagValue.Mask) == 0)
			{
				mCotpProtect.Flag[CotpNtcIndex] &= ProtectFlagValue.ClearMask;
				mCotpProtect.Flag[CotpNtcIndex] |= ProtectFlagValue.Setting;
				mCotpProtect.SetCount[ProtectLevel][CotpNtcIndex] = 1;
			}	
			else if((mCotpProtect.Flag[CotpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
			{
				mCotpProtect.SetCount[ProtectLevel][CotpNtcIndex]++;
				if(mCotpProtect.SetCount[ProtectLevel][CotpNtcIndex] >= ProtectPar.STime.l)
				{
					mCotpProtect.Flag[CotpNtcIndex] &= ProtectFlagValue.ClearMask;
					mCotpProtect.Flag[CotpNtcIndex] |= ProtectFlagValue.Setted;
					mCotpProtect.SetCount[ProtectLevel][CotpNtcIndex] = 0;

					if(mCotpProtect.EvtHandler)
					{
						mCotpProtect.EvtHandler(0, APP_PROTECT_COTP_L1_SET + ProtectLevel, &CotpNtcIndex);	
					}
					//sprintf(str,"COTP Set %d %.2X", CotpNtcIndex, mCotpProtect.Flag[CotpNtcIndex]);
					//appProtectCotpDebugMsg(str);
				}
			}
		}
		else if((mCotpProtect.Flag[CotpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
		{
			mCotpProtect.Flag[CotpNtcIndex] &= ProtectFlagValue.ClearMask;
		}
		//--------------------------------------------------------------------------
		//	Level	Release
		if(appProtectIsUnderTemperter(NtcVoltage, ProtectPar.RelValue.l) && ProtectPar.RTime.l)
		{
			if((mCotpProtect.Flag[CotpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setted)
			{
				mCotpProtect.Flag[CotpNtcIndex] &= ProtectFlagValue.ClearMask;
				mCotpProtect.Flag[CotpNtcIndex] |= ProtectFlagValue.Releasing;
				mCotpProtect.ReleaseCount[ProtectLevel][CotpNtcIndex] = 1;
			}	
			else if((mCotpProtect.Flag[CotpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
			{
				mCotpProtect.ReleaseCount[ProtectLevel][CotpNtcIndex]++;
				if(mCotpProtect.ReleaseCount[ProtectLevel][CotpNtcIndex] >= ProtectPar.RTime.l)
				{
					mCotpProtect.Flag[CotpNtcIndex] &= ProtectFlagValue.ClearMask;
					mCotpProtect.ReleaseCount[ProtectLevel][CotpNtcIndex] = 0;
					if(mCotpProtect.EvtHandler)
					{
						mCotpProtect.EvtHandler(0, APP_PROTECT_COTP_L1_RELEASE + ProtectLevel, &CotpNtcIndex);
					}
					//sprintf(str,"COTP Release %d %.2X", CotpNtcIndex, mCotpProtect.Flag[CotpNtcIndex]);
					//appProtectCotpDebugMsg(str);
				}
			}
		}
		else if((mCotpProtect.Flag[CotpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
		{
			mCotpProtect.Flag[CotpNtcIndex] &= ProtectFlagValue.ClearMask;
			mCotpProtect.Flag[CotpNtcIndex] |= ProtectFlagValue.Setted;
		}
		CotpNtcIndex++;
		if(CotpNtcIndex >= cotpNtcNumber())
		{
			CotpNtcIndex = 0;
			return 1;
		}
		count++;
		if(count >= COTP_CHECK_COUNT)
			break;
	}
	return 0;
}

void apiProtectCotpOpen(tAppProtectEvtHandler evtHandler)
{
	cotpProtectIni();
	
	mCotpProtect.EvtHandler = evtHandler;
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


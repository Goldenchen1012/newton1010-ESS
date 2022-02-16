/**
  ******************************************************************************
  * @file        ApiProtechIrUrp.c
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
#include "ApiSysPar.h"
#include "AppProject.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	apiProtectUrpDebugMsg(str)	//appSerialCanDavinciSendTextMessage(str)

/* Private define ------------------------------------------------------------*/
#define IR_RP_INDEX	0
#define	IR_RN_INDEX	1	

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t	Flag[2];
	uint8_t	SetCount[PROTECT_LEVEL][2];
	uint8_t	ReleaseCount[PROTECT_LEVEL][2];
	tAppProtectEvtHandler  EvtHandler;
}tUrpProtect;

static tUrpProtect	mUrpProtect={0};
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
uint8_t	apiProtectIrUrpGetRpFlag(void)
{
	return mUrpProtect.Flag[IR_RP_INDEX];
}
uint8_t	apiProtectIrUrpGetRnFlag(void)
{
	return mUrpProtect.Flag[IR_RN_INDEX];
}

uint8_t apiProtectIrUrpHandler(uint8_t ProtectLevel)
{
	BYTE	str[200];
	
	uint8_t		RpRnSel;
	uint32_t	RpValue;
	uint32_t	RnValue;
	tProtectFlagValue	ProtectFlagValue;
	tScuProtectPar		ProtectPar;

	apiSysParGetIrUrpSetPar(IR_RP_INDEX, ProtectLevel, &ProtectPar);
	apiSysParGetIrUrpRlxPar(IR_RP_INDEX, ProtectLevel, &ProtectPar);

	appProtectGetLevelMask(ProtectLevel, &ProtectFlagValue);

	appProjcetGetIrValue(&RpValue, &RnValue);

	sprintf(str,"ID %d %d  %d %d %d %d %d",ProtectLevel,
					RpValue, RnValue, 
					ProtectPar.SetValue.l,	ProtectPar.STime.l,
					ProtectPar.RelValue.l,ProtectPar.RTime.l);
	apiProtectUrpDebugMsg(str);
	
	for(RpRnSel=0; RpRnSel<2; RpRnSel++)
	{		
		if(RpValue > ProtectPar.SetValue.l && ProtectPar.STime.l)	 	
		{
			if((mUrpProtect.Flag[RpRnSel] & ProtectFlagValue.Mask) == 0)
			{
				mUrpProtect.Flag[RpRnSel] &= ProtectFlagValue.ClearMask;
				mUrpProtect.Flag[RpRnSel] |= ProtectFlagValue.Setting;
				mUrpProtect.SetCount[ProtectLevel][RpRnSel] = 1;
			}	
			else if((mUrpProtect.Flag[RpRnSel] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
			{
				mUrpProtect.SetCount[ProtectLevel][RpRnSel]++;
				if(mUrpProtect.SetCount[ProtectLevel][RpRnSel] >= ProtectPar.STime.l)
				{
					mUrpProtect.Flag[RpRnSel] &= ProtectFlagValue.ClearMask;
					mUrpProtect.Flag[RpRnSel] |= ProtectFlagValue.Setted;
					mUrpProtect.SetCount[ProtectLevel][RpRnSel] = 0;

					if(mUrpProtect.EvtHandler)
					{
						//mUrpProtect.EvtHandler(0, APP_PROTECT_DUTP_L1_SET + ProtectLevel, &DutpNtcIndex);	
					}
				}
			}
		}
		else if((mUrpProtect.Flag[RpRnSel] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
		{
			mUrpProtect.Flag[RpRnSel] &= ProtectFlagValue.ClearMask;
		}
		//--------------------------------------------------------------------------
		//	Level	Release
		//if(appProtectIsOverTemperter(NtcVoltage, ProtectPar.RelValue.l) && ProtectPar.RTime.l)
		if(RpValue < ProtectPar.RelValue.l && ProtectPar.RTime.l)	 	
		{
			if((mUrpProtect.Flag[RpRnSel] & ProtectFlagValue.Mask) == ProtectFlagValue.Setted)
			{
				mUrpProtect.Flag[RpRnSel] &= ProtectFlagValue.ClearMask;
				mUrpProtect.Flag[RpRnSel] |= ProtectFlagValue.Releasing;
				mUrpProtect.ReleaseCount[ProtectLevel][RpRnSel] = 1;
			}	
			else if((mUrpProtect.Flag[RpRnSel] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
			{
				mUrpProtect.ReleaseCount[ProtectLevel][RpRnSel]++;
				if(mUrpProtect.ReleaseCount[ProtectLevel][RpRnSel] >= ProtectPar.RTime.l)
				{
					mUrpProtect.Flag[RpRnSel] &= ProtectFlagValue.ClearMask;
					mUrpProtect.ReleaseCount[ProtectLevel][RpRnSel] = 0;
					if(mUrpProtect.EvtHandler)
					{
						//mUrpProtect.EvtHandler(0, APP_PROTECT_DUTP_L1_RELEASE + ProtectLevel, &DutpNtcIndex);
					}
				}
			}
		}
		else if((mUrpProtect.Flag[RpRnSel] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
		{
			mUrpProtect.Flag[RpRnSel] &= ProtectFlagValue.ClearMask;
			mUrpProtect.Flag[RpRnSel] |= ProtectFlagValue.Setted;
		}
	}
	return 0;
}

void apiProtectIrUrpOpen(tAppProtectEvtHandler evtHandler)
{
	mUrpProtect.EvtHandler = evtHandler;
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


/**
  ******************************************************************************
  * @file        ApiProtechDtp.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2022/1/27
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
#include "ApiProtectDtp.h"
#include "ApiSysPar.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	appProtectDtpDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t	Flag;
	uint8_t	SetCount[PROTECT_LEVEL];
	uint8_t	ReleaseCount[PROTECT_LEVEL];
	tAppProtectEvtHandler  EvtHandler;
}tDtpProtect;

static tDtpProtect	mDtpProtect={0};
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
uint8_t	apiProtectDtpGetFlag(void)
{
	return mDtpProtect.Flag;
}


uint8_t apiProtectDtpHandler(void)
{

	uint16_t	dT;
	uint8_t		ProtectLevel;
	tScuProtectPar		ProtectPar;
	tProtectFlagValue	ProtectFlagValue;
	char	str[100];
	
	dT = HalAfeGetMaxNtcTemp(0,0) - HalAfeGetMinNtcTemp(0,0);
	dT /= 100;
	
	for(ProtectLevel=0; ProtectLevel<3; ProtectLevel++)
	{
		apiSysParGetDtpPar(ProtectLevel, &ProtectPar);
		appProtectGetLevelMask(ProtectLevel, &ProtectFlagValue);
		
		sprintf(str,"DTP %d %d %d %d",ProtectLevel,
			 dT,ProtectPar.SetValue.l,
			 ProtectPar.RelValue.l);
		
		
		appProtectDtpDebugMsg(str);
		if(dT > ProtectPar.SetValue.l && ProtectPar.STime.l)
		{
			if((mDtpProtect.Flag & ProtectFlagValue.Mask) == 0)
			{
				mDtpProtect.Flag &= ProtectFlagValue.ClearMask;
				mDtpProtect.Flag |= ProtectFlagValue.Setting;
				mDtpProtect.SetCount[ProtectLevel] = 1;
			}	
			else if((mDtpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
			{
				mDtpProtect.SetCount[ProtectLevel]++;
				if(mDtpProtect.SetCount[ProtectLevel] >= ProtectPar.STime.l)
				{
					mDtpProtect.Flag &= ProtectFlagValue.ClearMask;
					mDtpProtect.Flag |= ProtectFlagValue.Setted;
					mDtpProtect.SetCount[ProtectLevel] = 0;

					if(mDtpProtect.EvtHandler)
					{
						mDtpProtect.EvtHandler(0, APP_PROTECT_DTP_L1_SET + ProtectLevel, &dT);	
					}
				}
			}
		}
		else if((mDtpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
		{
			mDtpProtect.Flag &= ProtectFlagValue.ClearMask;
		}
		//--------------------------------------------------------------------------
		//	Level	Release
		if(dT < ProtectPar.RelValue.l && ProtectPar.RTime.l)
		{
			if((mDtpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Setted)
			{
				mDtpProtect.Flag &= ProtectFlagValue.ClearMask;
				mDtpProtect.Flag |= ProtectFlagValue.Releasing;
				mDtpProtect.ReleaseCount[ProtectLevel] = 1;
			}	
			else if((mDtpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
			{
				mDtpProtect.ReleaseCount[ProtectLevel]++;
				if(mDtpProtect.ReleaseCount[ProtectLevel] >= ProtectPar.RTime.l)
				{
					mDtpProtect.Flag &= ProtectFlagValue.ClearMask;
					mDtpProtect.ReleaseCount[ProtectLevel] = 0;
					if(mDtpProtect.EvtHandler)
					{
						mDtpProtect.EvtHandler(0, APP_PROTECT_DTP_L1_RELEASE + ProtectLevel, &dT);
					}
				}
			}
		}
		else if((mDtpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
		{
			mDtpProtect.Flag &= ProtectFlagValue.ClearMask;
			mDtpProtect.Flag |= ProtectFlagValue.Setted;
		}
	}
	return 0;
}

void apiProtectDtpOpen(tAppProtectEvtHandler evtHandler)
{
	mDtpProtect.EvtHandler = evtHandler;
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


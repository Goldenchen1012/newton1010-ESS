/**
  ******************************************************************************
  * @file        ApiProtechDvp.c
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
#include <string.h>
#include <stdio.h>
#include "define.h"
#include "halafe.h"
#include "AppProtect.h"
#include "ApiProtectUvp.h"
#include "ApiSysPar.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	appProtectDvpDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t	Flag;
	uint8_t	SetCount[PROTECT_LEVEL];
	uint8_t	ReleaseCount[PROTECT_LEVEL];
	tAppProtectEvtHandler  EvtHandler;
}tDvpProtect;

static tDvpProtect	mDvpProtect={0};
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
uint8_t	apiProtectDvpGetFlag(void)
{
	return mDvpProtect.Flag;
}

uint8_t apiProtectDvpHandler(void)
{
	uint16_t			dV;
	tProtectFlagValue	ProtectFlagValue;
	tScuProtectPar		ProtectPar;
	uint8_t				ProtectLevel;
	
	char	str[100];
	
	dV = halAfeGetMaxCellVoltage(0, 0) - halAfeGetMinCellVoltage(0, 0);

	for(ProtectLevel=0; ProtectLevel<3; ProtectLevel++)
	{
		apiSysParGetDvpPar(ProtectLevel, &ProtectPar);
		appProtectGetLevelMask(ProtectLevel, &ProtectFlagValue);
#if 0		
		sprintf(str,"DVP %d %d %d %d %.2X %.2X",ProtectLevel,
			 dV,
			 ProtectPar.SetValue.l,
			 ProtectPar.RelValue.l,
			 mDvpProtect.Flag,
			 ProtectFlagValue.ClearMask
			 );
		appProtectDvpDebugMsg(str);
#endif
		if(dV > ProtectPar.SetValue.l && ProtectPar.STime.l)
		{
			if((mDvpProtect.Flag & ProtectFlagValue.Mask) == 0)
			{
				mDvpProtect.Flag &= ProtectFlagValue.ClearMask;
				mDvpProtect.Flag |= ProtectFlagValue.Setting;
				mDvpProtect.SetCount[ProtectLevel] = 1;
			}
			else if((mDvpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
			{
				mDvpProtect.SetCount[ProtectLevel]++;
				if(mDvpProtect.SetCount[ProtectLevel] >= ProtectPar.STime.l)
				{
					appProtectDvpDebugMsg("Set");
					if(mDvpProtect.EvtHandler)
					{
						mDvpProtect.EvtHandler(0, APP_PROTECT_DVP_L1_SET + ProtectLevel, &dV);
					}
					mDvpProtect.Flag &= ProtectFlagValue.ClearMask;
					mDvpProtect.Flag |= ProtectFlagValue.Setted;
					mDvpProtect.SetCount[ProtectLevel] = 0;
				}
			}
		}
		else  if((mDvpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
		{
			mDvpProtect.Flag &= ProtectFlagValue.ClearMask;
		}
		//-------------------------------------------
		//release 
		if(dV < ProtectPar.RelValue.l && ProtectPar.RTime.l)
		{
			if((mDvpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Setted)
			{
				mDvpProtect.Flag &= ProtectFlagValue.ClearMask;
				mDvpProtect.Flag |= ProtectFlagValue.Releasing;
				mDvpProtect.ReleaseCount[ProtectLevel] = 1;
			}
			else if((mDvpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
			{
				mDvpProtect.ReleaseCount[ProtectLevel]++;
				if(mDvpProtect.ReleaseCount[ProtectLevel] >= ProtectPar.RTime.l)
				{
					mDvpProtect.Flag &= ProtectFlagValue.ClearMask;
					mDvpProtect.ReleaseCount[ProtectLevel] = 0;
					appProtectDvpDebugMsg("Release");
					if(mDvpProtect.EvtHandler)
					{
						mDvpProtect.EvtHandler(0, APP_PROTECT_DVP_L1_RELEASE + ProtectLevel, &dV);	
					}
				}
			}
		}
		else if((mDvpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
		{
			mDvpProtect.Flag &= ProtectFlagValue.ClearMask;
			mDvpProtect.Flag |= ProtectFlagValue.Setted;
		}
	}
	return 0;
}

void apiProtectDvpOpen(tAppProtectEvtHandler evtHandler)
{
	mDvpProtect.EvtHandler = evtHandler;
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


/**
  ******************************************************************************
  * @file        ApiProtechScuOt.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2022/01/26
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
#include "ApiProtectScuOt.h"
#include "ApiSysPar.h"

void appSerialCanDavinciSendTextMessage(uint8_t *str);
#define	scuOtDebugMsg(str)	//appSerialCanDavinciSendTextMessage(str)


/* Private define ------------------------------------------------------------*/
#define	MAX_SCU_TEMP_NUMBER		5
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t	Flag[MAX_SCU_TEMP_NUMBER];
	uint8_t	SetCount[PROTECT_LEVEL][MAX_SCU_TEMP_NUMBER];
	uint8_t	ReleaseCount[PROTECT_LEVEL][MAX_SCU_TEMP_NUMBER];
	tAppProtectEvtHandler  EvtHandler;
}tScuProtect;

/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const uint8_t ScuOtSetEvent[MAX_SCU_TEMP_NUMBER][3]={
	{APP_PROTECT_RLY1_OT_L1_SET, 
	 APP_PROTECT_RLY1_OT_L2_SET,	
	 APP_PROTECT_RLY1_OT_L3_SET},
	{APP_PROTECT_RLY2_OT_L1_SET,
	 APP_PROTECT_RLY2_OT_L2_SET,
	 APP_PROTECT_RLY2_OT_L3_SET},
	{APP_PROTECT_AMBI_OT_L1_SET,
	 APP_PROTECT_AMBI_OT_L2_SET,
	 APP_PROTECT_AMBI_OT_L3_SET},
	{APP_PROTECT_BUSBAR_P_OT_L1_SET,
	 APP_PROTECT_BUSBAR_P_OT_L2_SET,
	 APP_PROTECT_BUSBAR_P_OT_L3_SET},
	{APP_PROTECT_BUSBAR_N_OT_L1_SET,
	 APP_PROTECT_BUSBAR_N_OT_L2_SET,
	 APP_PROTECT_BUSBAR_N_OT_L3_SET}};
	 
static const uint8_t ScuOtReleaseEvent[MAX_SCU_TEMP_NUMBER][3]={
	{APP_PROTECT_RLY1_OT_L1_RELEASE,
	 APP_PROTECT_RLY1_OT_L2_RELEASE,
	 APP_PROTECT_RLY1_OT_L3_RELEASE},
	{APP_PROTECT_RLY2_OT_L1_RELEASE,
	 APP_PROTECT_RLY2_OT_L2_RELEASE,
	 APP_PROTECT_RLY2_OT_L3_RELEASE},
	{APP_PROTECT_AMBI_OT_L1_RELEASE,
	 APP_PROTECT_AMBI_OT_L2_RELEASE,
	 APP_PROTECT_AMBI_OT_L3_RELEASE},
	{APP_PROTECT_BUSBAR_P_OT_L1_RELEASE,
	 APP_PROTECT_BUSBAR_P_OT_L2_RELEASE,
	 APP_PROTECT_BUSBAR_P_OT_L3_RELEASE},
	{APP_PROTECT_BUSBAR_N_OT_L1_RELEASE,
	 APP_PROTECT_BUSBAR_N_OT_L2_RELEASE,
	 APP_PROTECT_BUSBAR_N_OT_L3_RELEASE}};
	

static tScuProtect	mScuOtProtect = {0};
/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
uint8_t	apiProtectScuOtFlag(uint16_t NtcIndex)
{
	return mScuOtProtect.Flag[NtcIndex];
}


uint8_t apiProtectScuOtHandler(uint8_t ProtectLevel)
{
//#define	SHOW_SCU_OT_DEBUG_MSG	

#ifdef SHOW_SCU_OT_DEBUG_MSG
	BYTE	str[200];
#endif	
	uint8_t		ntcindex;
	uint16_t	temp;
	
	tProtectFlagValue	ProtectFlagValue;
	tScuProtectPar		ProtectPar;

	appProtectGetLevelMask(ProtectLevel, &ProtectFlagValue);
	
	for(ntcindex=0; ntcindex<MAX_SCU_TEMP_NUMBER; ntcindex++)
	{
		apiSysParGetScuOtPar(ntcindex, ProtectLevel, &ProtectPar);
		temp = apiScuTempGetTemperature(ntcindex);
		temp /= 100;
#ifdef SHOW_SCU_OT_DEBUG_MSG
		sprintf(str, "ScuOT I=%d L=%d %d ,%d %d %d %d",
						ntcindex, ProtectLevel, 
						temp,
						ProtectPar.SetValue.l,
						ProtectPar.STime.l,
						ProtectPar.RelValue.l,
						ProtectPar.RTime.l);
		scuOtDebugMsg(str);		
#endif		
		if(temp > ProtectPar.SetValue.l && ProtectPar.STime.l)
		{
			if((mScuOtProtect.Flag[ntcindex] & ProtectFlagValue.Mask) == 0)
			{
				mScuOtProtect.Flag[ntcindex] &= ProtectFlagValue.ClearMask;
				mScuOtProtect.Flag[ntcindex] |= ProtectFlagValue.Setting;
				mScuOtProtect.SetCount[ProtectLevel][ntcindex] = 1;
			}	
			else if((mScuOtProtect.Flag[ntcindex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
			{
				mScuOtProtect.SetCount[ProtectLevel][ntcindex]++;
				if(mScuOtProtect.SetCount[ProtectLevel][ntcindex] >= ProtectPar.STime.l)
				{
					mScuOtProtect.Flag[ntcindex] &= ProtectFlagValue.ClearMask;
					mScuOtProtect.Flag[ntcindex] |= ProtectFlagValue.Setted;
					mScuOtProtect.SetCount[ProtectLevel][ntcindex] = 0;

					if(mScuOtProtect.EvtHandler)
					{
						mScuOtProtect.EvtHandler(0, ScuOtSetEvent[ntcindex][ProtectLevel], 0);
					}
				}
			}
		}
		else if((mScuOtProtect.Flag[ntcindex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
		{
			mScuOtProtect.Flag[ntcindex] &= ProtectFlagValue.ClearMask;
		}
		//--------------------------------------------------------------------------
		//	Level	Release
		if(temp < ProtectPar.SetValue.l && ProtectPar.RTime.l)
		{
			if((mScuOtProtect.Flag[ntcindex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setted)
			{
				mScuOtProtect.Flag[ntcindex] &= ProtectFlagValue.ClearMask;
				mScuOtProtect.Flag[ntcindex] |= ProtectFlagValue.Releasing;
				mScuOtProtect.ReleaseCount[ProtectLevel][ntcindex] = 1;
			}	
			else if((mScuOtProtect.Flag[ntcindex] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
			{
				mScuOtProtect.ReleaseCount[ProtectLevel][ntcindex]++;
				if(mScuOtProtect.ReleaseCount[ProtectLevel][ntcindex] >= ProtectPar.RTime.l)
				{
					mScuOtProtect.Flag[ntcindex] &= ProtectFlagValue.ClearMask;
					mScuOtProtect.ReleaseCount[ProtectLevel][ntcindex] = 0;
					if(mScuOtProtect.EvtHandler)
					{
						mScuOtProtect.EvtHandler(0, ScuOtReleaseEvent[ntcindex][ProtectLevel], 0);
					}
				}
			}
		}
		else if((mScuOtProtect.Flag[ntcindex] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
		{
			mScuOtProtect.Flag[ntcindex] &= ProtectFlagValue.ClearMask;
			mScuOtProtect.Flag[ntcindex] |= ProtectFlagValue.Setted;
		}
	}
	return 0;
}

void apiProtectScuOtOpen(tAppProtectEvtHandler evtHandler)
{
	mScuOtProtect.EvtHandler = evtHandler;
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


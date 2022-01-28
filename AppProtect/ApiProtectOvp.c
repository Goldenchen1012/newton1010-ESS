/**
  ******************************************************************************
  * @file        ApiProtechOvp.c
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
#include "ApiProtectOvp.h"
#include "ApiSysPar.h"

void appSerialCanDavinciSendTextMessage(uint8_t *str);
#define	apiProtectOvpDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private define ------------------------------------------------------------*/
#define	OVP_CHECK_COUNT			20
#define	ovpCellNumber()			apiSysParGetCellNumber()
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t	Flag[MAX_CELL_NUMBER];
	uint8_t	SetCount[PROTECT_LEVEL][MAX_CELL_NUMBER];
	uint8_t	ReleaseCount[PROTECT_LEVEL][MAX_CELL_NUMBER];
	uint8_t	PfSetCount;
	uint8_t	PfFlag;
	tAppProtectEvtHandler  EvtHandler;
}tOvpProtect;

static tOvpProtect	mOvpProtect={0};
static	uint16_t	OvpCellIndex;
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void ovpProtectIni(void)
{
	OvpCellIndex = 0;
	if(apiSysParIsOvpPfSet())
		mOvpProtect.PfFlag = 1;
}


/* Public function prototypes -----------------------------------------------*/
void apiProtectOvpPfClean(void)
{
	mOvpProtect.PfFlag = 0;
}

uint8_t	apiProtectOvpPfGetFlag(void)
{
	return mOvpProtect.PfFlag;
}

uint8_t	apiProtectOvpGetFlag(uint16_t CellIndex)
{
	return mOvpProtect.Flag[CellIndex];
}

uint8_t apiProtectOvpHandler(uint8_t ProtectLevel)
{
	uint8_t	count = 0;
	WORD	CellVoltage;
	tProtectFlagValue	ProtectFlagValue;
	tScuProtectPar	ProtectPar;
	char	str[100];
	
	apiSysParGetOvpPar(ProtectLevel, &ProtectPar);
	appProtectGetLevelMask(ProtectLevel, &ProtectFlagValue);

	while(1)
	{			
		CellVoltage = halAfeGetCellVoltage(OvpCellIndex);
		//sprintf(str,"C%d = %d",OvpCellIndex, CellVoltage);
		//appProtectOvpDebugMsg(str);
		
		if(CellVoltage > ProtectPar.SetValue.l && ProtectPar.STime.l)
		{
			if((mOvpProtect.Flag[OvpCellIndex] & ProtectFlagValue.Mask) == 0)
			{
				mOvpProtect.Flag[OvpCellIndex] &= ProtectFlagValue.ClearMask;
				mOvpProtect.Flag[OvpCellIndex] |= ProtectFlagValue.Setting;
				mOvpProtect.SetCount[ProtectLevel][OvpCellIndex] = 1;
			}
			else if((mOvpProtect.Flag[OvpCellIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
			{
				mOvpProtect.SetCount[ProtectLevel][OvpCellIndex]++;
				if(mOvpProtect.SetCount[ProtectLevel][OvpCellIndex] >= ProtectPar.STime.l)
				{
					mOvpProtect.Flag[OvpCellIndex] &= ProtectFlagValue.ClearMask;
					mOvpProtect.Flag[OvpCellIndex] |= ProtectFlagValue.Setted;
					mOvpProtect.SetCount[ProtectLevel][OvpCellIndex] = 0;
					
					if(mOvpProtect.EvtHandler)
					{
						mOvpProtect.EvtHandler(0, APP_PROTECT_OVP_L1_SET + ProtectLevel, &OvpCellIndex);	
					}
				}
			}
		}
		else  if((mOvpProtect.Flag[OvpCellIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
		{
			mOvpProtect.Flag[OvpCellIndex] &= ProtectFlagValue.ClearMask;
		}
		//-------------------------------------------
		//release 
		if(CellVoltage < ProtectPar.RelValue.l && ProtectPar.RTime.l)
		{
			if((mOvpProtect.Flag[OvpCellIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setted)
			{
				mOvpProtect.Flag[OvpCellIndex] &= ProtectFlagValue.ClearMask;
				mOvpProtect.Flag[OvpCellIndex] |= ProtectFlagValue.Releasing;
				mOvpProtect.ReleaseCount[ProtectLevel][OvpCellIndex] = 1;
			}
			else if((mOvpProtect.Flag[OvpCellIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
			{
				mOvpProtect.ReleaseCount[ProtectLevel][OvpCellIndex]++;
				if(mOvpProtect.ReleaseCount[ProtectLevel][OvpCellIndex] >= ProtectPar.RTime.l)
				{
					mOvpProtect.Flag[OvpCellIndex] &= ProtectFlagValue.ClearMask;
					mOvpProtect.ReleaseCount[ProtectLevel][OvpCellIndex] = 0;
					if(mOvpProtect.EvtHandler)
					{
						mOvpProtect.EvtHandler(0, APP_PROTECT_OVP_L1_RELEASE + ProtectLevel, &OvpCellIndex);
					}
				}
			}
		}
		else if((mOvpProtect.Flag[OvpCellIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
		{
			mOvpProtect.Flag[OvpCellIndex] &= ProtectFlagValue.ClearMask;
			mOvpProtect.Flag[OvpCellIndex] |= ProtectFlagValue.Setted;
		}
		OvpCellIndex++;
		if(OvpCellIndex >= ovpCellNumber())
		{
			OvpCellIndex = 0;
			return 1;
		}
		count++;
		if(count >= OVP_CHECK_COUNT)
			break;
	}
	return 0;
}

void apiProtectOvpPfHandler(void)
{
	uint8_t		bmu,posi;
	tScuProtectPar	ProtectPar;
	uint16_t	voltage;
	
	if(mOvpProtect.PfFlag)
		return;
	apiSysParGetOvpPfPar(&ProtectPar);
	voltage = halAfeGetMaxCellVoltage(&bmu, &posi);
	if(voltage > ProtectPar.SetValue.l && ProtectPar.STime.l)
	{
		mOvpProtect.PfSetCount++;
		if(mOvpProtect.PfSetCount >= ProtectPar.STime.l)
		{
			mOvpProtect.PfFlag = 1;	
			if(mOvpProtect.EvtHandler)
			{
				mOvpProtect.EvtHandler(0, APP_PROTECT_OVP_PF, 0);
			}
		}
	}
	else
		mOvpProtect.PfSetCount = 0;
}


void apiProtectOvpOpen(tAppProtectEvtHandler evtHandler)
{
	ovpProtectIni();
	
	mOvpProtect.EvtHandler = evtHandler;
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


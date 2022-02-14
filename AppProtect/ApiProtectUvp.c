/**
  ******************************************************************************
  * @file        ApiProtechUvp.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/29
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
#include <string.h>
#include <stdio.h>
#include "define.h"
#include "halafe.h"
#include "AppProtect.h"
#include "ApiProtectUvp.h"
#include "ApiSysPar.h"

void appSerialUartSendMessage(uint8_t *str);
#define	appProtectUvpDebugMsg(str)	//appSerialUartSendMessage(str)


/* Private define ------------------------------------------------------------*/
#define	UVP_CHECK_COUNT			20
#define	uvpCellNumber()			apiSysParGetCellNumber()
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t	Flag[MAX_CELL_NUMBER];
	uint8_t	SetCount[PROTECT_LEVEL][MAX_CELL_NUMBER];
	uint8_t	ReleaseCount[PROTECT_LEVEL][MAX_CELL_NUMBER];
	uint8_t	PfSetCount;
	uint8_t	PfFlag;
	tAppProtectEvtHandler  EvtHandler;
}tUvpProtect;

static tUvpProtect	mUvpProtect={0};
static	uint16_t	UvpCellIndex;

/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void uvpProtectIni(void)
{
	UvpCellIndex = 0;
	if(apiSysParIsUvpPfSet())
		mUvpProtect.PfFlag = 1;
}

/* Public function prototypes -----------------------------------------------*/
void apiProtectUvpPfClean(void)
{
	mUvpProtect.PfFlag = 0;
}

uint8_t	apiProtectUvpPfGetFlag(void)
{
	return mUvpProtect.PfFlag;
}

uint8_t	apiProtectUvpGetFlag(uint16_t CellIndex)
{
	return mUvpProtect.Flag[CellIndex];
}

uint8_t apiProtectUvpHandler(uint8_t ProtectLevel)
{
	uint8_t		checkcount = 0;
	WORD	CellVoltage;
	tProtectFlagValue	ProtectFlagValue;
	tScuProtectPar	ProtectPar;
	
	apiSysParGetUvpPar(ProtectLevel, &ProtectPar);
	appProtectGetLevelMask(ProtectLevel, &ProtectFlagValue);

	while(1)
	{			
		CellVoltage = halAfeGetCellVoltage(UvpCellIndex);

		if(CellVoltage < ProtectPar.SetValue.l && ProtectPar.STime.l)
		{
			if((mUvpProtect.Flag[UvpCellIndex] & ProtectFlagValue.Mask) == 0)
			{
				mUvpProtect.Flag[UvpCellIndex] &= ProtectFlagValue.ClearMask;
				mUvpProtect.Flag[UvpCellIndex] |= ProtectFlagValue.Setting;
				mUvpProtect.SetCount[ProtectLevel][UvpCellIndex] = 1;
			}
			else if((mUvpProtect.Flag[UvpCellIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
			{
				mUvpProtect.SetCount[ProtectLevel][UvpCellIndex]++;
				if(mUvpProtect.SetCount[ProtectLevel][UvpCellIndex] >= ProtectPar.STime.l)
				{
					if(mUvpProtect.EvtHandler)
					{
						mUvpProtect.EvtHandler(0, APP_PROTECT_UVP_L1_SET + ProtectLevel, &UvpCellIndex);	
					}
					mUvpProtect.Flag[UvpCellIndex] &= ProtectFlagValue.ClearMask;
					mUvpProtect.Flag[UvpCellIndex] |= ProtectFlagValue.Setted;
					mUvpProtect.SetCount[ProtectLevel][UvpCellIndex] = 0;
				}
			}
		}
		else  if((mUvpProtect.Flag[UvpCellIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
		{
			mUvpProtect.Flag[UvpCellIndex] &= ProtectFlagValue.ClearMask;
		}
		//-------------------------------------------
		//release 
		if(CellVoltage > ProtectPar.RelValue.l && ProtectPar.RTime.l)
		{
			if((mUvpProtect.Flag[UvpCellIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setted)
			{
				mUvpProtect.Flag[UvpCellIndex] &= ProtectFlagValue.ClearMask;
				mUvpProtect.Flag[UvpCellIndex] |= ProtectFlagValue.Releasing;
				mUvpProtect.ReleaseCount[ProtectLevel][UvpCellIndex] = 1;
			}
			else if((mUvpProtect.Flag[UvpCellIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
			{
				mUvpProtect.ReleaseCount[ProtectLevel][UvpCellIndex]++;
				if(mUvpProtect.ReleaseCount[ProtectLevel][UvpCellIndex] >= ProtectPar.RTime.l)
				{
					mUvpProtect.Flag[UvpCellIndex] &= ProtectFlagValue.ClearMask;
					mUvpProtect.ReleaseCount[ProtectLevel][UvpCellIndex] = 0;
					if(mUvpProtect.EvtHandler)
					{
						mUvpProtect.EvtHandler(0, APP_PROTECT_UVP_L1_RELEASE + ProtectLevel, &UvpCellIndex);	
					}
				}
			}
		}
		else if((mUvpProtect.Flag[UvpCellIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
		{
			mUvpProtect.Flag[UvpCellIndex] &= ProtectFlagValue.ClearMask;
			mUvpProtect.Flag[UvpCellIndex] |= ProtectFlagValue.Setted;
		}
		UvpCellIndex++;
		if(UvpCellIndex >= uvpCellNumber())
		{
			UvpCellIndex = 0;
			return 1;
		}
		checkcount++;
		if(checkcount >= UVP_CHECK_COUNT)
			break;
	}
	return 0;
}


void apiProtectUvpPfHandler(void)
{
	tScuProtectPar	ProtectPar;
	
	if(mUvpProtect.PfFlag)
		return;
	apiSysParGetUvpPfPar(&ProtectPar);
	if(halAfeGetMinCellVoltage() < ProtectPar.SetValue.l)
	{
		mUvpProtect.PfSetCount++;
		if(mUvpProtect.PfSetCount >= ProtectPar.STime.l)
		{
			mUvpProtect.PfFlag = 1;	
			if(mUvpProtect.EvtHandler)
			{
				mUvpProtect.EvtHandler(0, APP_PROTECT_UVP_PF, 0);
			}
		}
	}
	else
		mUvpProtect.PfSetCount = 0;
}

void apiProtectUvpOpen(tAppProtectEvtHandler evtHandler)
{
	uvpProtectIni();
	
	mUvpProtect.EvtHandler = evtHandler;
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


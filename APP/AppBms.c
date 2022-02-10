/**
  ******************************************************************************
  * @file        appBms.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/11
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
#include "define.h"
#include "main.h"
#include "halafe.h"
#include "LibSwTimer.h"
#include "HalRtc.h"
#include "ApiRamData.h"
#include "ApiSysPar.h"
#include "AppBms.h"
#include "ApiSystemFlag.h"
#include "ApiRelayControl.h"
#include "AppSerialCanDavinci.h"
#include "ApiSignalFeedback.h"


void appSerialCanDavinciSendTextMessage(char *str);
#define	appBmsDebugMsg(str)	//appSerialCanDavinciSendTextMessage(str)

/* Private define ------------------------------------------------------------*/
#define	BMS_DATA_VALID_DATA_VB				0x0001
#define	BMS_DATA_VALID_DATA_CURRENT			0x0002
#define	BMS_DATA_VALIE_DATA_SYS_FLAG1_2		0x0004
#define	BMS_DATA_VALIE_DATA_SYS_FLAG3_4		0x0008

#define	BMS_DATA_VALID_FLAG_ALL				0x000F


/* Private typedef -----------------------------------------------------------*/
#define	BMS_IDLE_COUNT			5
typedef struct{
	uint8_t		IdleCount;
	uint16_t	DataValidFlag;
	uint32_t	VbInt;
	uint32_t	VbExt;
	int32_t		CurrentP;
	int32_t		CurrentN;
	uint32_t	RM;					//mAh	甧秖 RM
	uint32_t	FCC;				//mAh	骸甧秖
	uint16_t	MaxVoltage;			//cell 程筿溃 1mV
	uint8_t		MaxVoltageBmu;		
	uint8_t		MaxVoltagePosition;
	uint16_t	MinVoltage;			//cell 程筿溃 1mV
	uint8_t		MinVoltageBmu;
	uint8_t		MinVoltagePosition;	
	uint16_t	MaxNtcTemp;		//Ntcs 程蔼放
	uint8_t		MaxNtcBmu;
	uint8_t		MaxNtcPosition;
	uint16_t	MinNtcTemp;
	uint8_t		MinNtcBmu;
	uint8_t		MinNtcPosition;
	uint32_t	SystemFlag1;		//SCU1 & SCU2
	uint32_t	SystemFlag2;
	uint32_t	SystemFlag3;
	uint32_t	SystemFlag4;
	uint16_t	CellVoltage[MAX_CELL_NUMBER];
	uint16_t	NtcVoltage[MAX_NTC_NUMBER];
}tBmsBuf;

static tBmsBuf	mBmsBuf[SYSTEM_MAX_SCU_NUM];


/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void updateScuIdleCount(uint8_t scuid)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return;
	mBmsBuf[scuid - 1].IdleCount = BMS_IDLE_COUNT;
}

static uint8_t isScuProtect(tBmsBuf *pBmsBuf)
{
	if(pBmsBuf->SystemFlag1 & FLAG1_PROTECT_MASK)
		return 1;
	if(pBmsBuf->SystemFlag2 & FLAG2_PROTECT_MASK)
		return 1;
	if(pBmsBuf->SystemFlag3 & FLAG3_PROTECT_MASK)
		return 1;
	if(pBmsBuf->SystemFlag4 & FLAG4_PROTECT_MASK)
		return 1;

	return 0;
}

static void bmsSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	uint8_t		ScuIndex;

	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
		return;
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		for(ScuIndex=0; ScuIndex<SYSTEM_MAX_SCU_NUM; ScuIndex++)
		{
			if(mBmsBuf[ScuIndex].IdleCount)
			{
				mBmsBuf[ScuIndex].IdleCount--;
				if(mBmsBuf[ScuIndex].IdleCount == 0)
				{
					memset(&mBmsBuf[ScuIndex], 0, sizeof(tBmsBuf));
				}
			}
		}
		putSelfDataToBmsBuffer();	//jjjjjj
	}
}

/* Public function prototypes -----------------------------------------------*/


uint8_t appBmsIsScuDataValid(uint8_t scuid)
{
	tBmsBuf 	*pBmsBuf;
	
	if(appBmsIsValidScuid(scuid) == false)
		return 0;
		
	pBmsBuf = &mBmsBuf[scuid - 1];

	if((pBmsBuf->DataValidFlag & BMS_DATA_VALID_FLAG_ALL) == BMS_DATA_VALID_FLAG_ALL)
		return 1;
	else
		return 0;
}

uint8_t appBmsIsValidScuid(uint8_t scuid)
{
	if(scuid == 0 || scuid >= SYSTEM_MAX_SCU_NUM)
		return 0;
	return 1;
}


uint8_t appBmsIsScuCanTurnOnRelay(uint8_t scuid)
{
	tBmsBuf 	*pBmsBuf;

	if(appBmsIsValidScuid(scuid) == false)
		return 0;
	pBmsBuf = &mBmsBuf[scuid - 1];
	
	if(pBmsBuf->IdleCount == 0)
		return 0;
	if(appBmsIsScuDataValid(scuid) == 0)
		return 0;
	if(isScuProtect(pBmsBuf))
		return 0;
	
	if((pBmsBuf->SystemFlag2 & SYSTEM_FLAG2_RELAY_ON)== 0)
		return 1;
	return 0;
}
uint8_t appBmsIsScuRelayOn(uint8_t scuid)
{
	tBmsBuf 	*pBmsBuf;

//	if(appBmsIsScudIdReady() == false)
//		return 0;
	
	if(appBmsIsValidScuid(scuid) == false)
		return 0;
	pBmsBuf = &mBmsBuf[scuid - 1];
	
	if(pBmsBuf->IdleCount == 0)
		return 0;
	if(appBmsIsScuDataValid(scuid) == 0)
		return 0;
	if(isScuProtect(pBmsBuf))
		return 0;
	
	if(pBmsBuf->SystemFlag2 & SYSTEM_FLAG2_RELAY_ON)
		return 1;
	else
		return 0;
}
void putSelfDataToBmsBuffer(void)
{
	uint8_t		scuid;
	uint8_t		bmu;
	uint8_t		posi;	
	tBmsBuf		*pBmsBuf;
	
	if(appBmsIsScudIdReady() == false)
		return;
	scuid = appBmsGetScuId();
	pBmsBuf = &mBmsBuf[scuid - 1];
	pBmsBuf->IdleCount = BMS_IDLE_COUNT;
	pBmsBuf->DataValidFlag = BMS_DATA_VALID_FLAG_ALL;
	pBmsBuf->SystemFlag1 = apiSystemFlagGetFlag1();
	pBmsBuf->SystemFlag2 = apiSystemFlagGetFlag2();
	pBmsBuf->SystemFlag3 = apiSystemFlagGetFlag3();
	pBmsBuf->SystemFlag4 = apiSystemFlagGetFlag4();

	pBmsBuf->MaxVoltage = halAfeGetMaxCellVoltage(&bmu, &posi);
	pBmsBuf->MaxVoltageBmu = bmu;
	pBmsBuf->MaxVoltagePosition = posi;

	pBmsBuf->MinVoltage = halAfeGetMinCellVoltage(&bmu, &posi);
	pBmsBuf->MinVoltageBmu = bmu;
	pBmsBuf->MinVoltagePosition = posi;

	pBmsBuf->MaxNtcTemp = HalAfeGetMaxNtcTemp(&bmu, &posi);
	pBmsBuf->MaxNtcBmu = bmu;
	pBmsBuf->MaxNtcPosition = posi;
	
	pBmsBuf->MinNtcTemp = HalAfeGetMinNtcTemp(&bmu, &posi);
	pBmsBuf->MinNtcBmu = bmu;
	pBmsBuf->MinNtcPosition = posi;

	pBmsBuf->VbInt = halAfeGetVBatVoltage(0);
}

uint8_t appBmsSetScuSystemFlag1_2(uint8_t scuid, uint32_t flag1, uint32_t flag2)
{
//	appBmsDebugMsg("Rcv system flag..0");
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	scuid--;
	mBmsBuf[scuid].SystemFlag1 = flag1;
	mBmsBuf[scuid].SystemFlag2 = flag2;
	mBmsBuf[scuid].DataValidFlag |= BMS_DATA_VALIE_DATA_SYS_FLAG1_2;
	updateScuIdleCount(scuid);
//	appBmsDebugMsg("Rcv system flag");
	return true;
}
uint8_t appBmsGetScuSystemFlag1_2(uint8_t scuid, uint32_t *flag1, uint32_t *flag2)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	scuid--;
	*flag1 = mBmsBuf[scuid].SystemFlag1;
	*flag2 = mBmsBuf[scuid].SystemFlag2;
	return true;
}

uint8_t appBmsSetScuSystemFlag3_4(uint8_t scuid, uint32_t flag3, uint32_t flag4)
{
//	appBmsDebugMsg("Rcv system flag..0");
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	scuid--;
	mBmsBuf[scuid].SystemFlag3 = flag3;
	mBmsBuf[scuid].SystemFlag4 = flag4;
	mBmsBuf[scuid].DataValidFlag |= BMS_DATA_VALIE_DATA_SYS_FLAG3_4;
	updateScuIdleCount(scuid);
//	appBmsDebugMsg("Rcv system flag");
	return true;
}

uint8_t appBmsGetScuSystemFlag3_4(uint8_t scuid, uint32_t *flag3, uint32_t *flag4)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	scuid--;
	*flag3 = mBmsBuf[scuid].SystemFlag3;
	*flag4 = mBmsBuf[scuid].SystemFlag4;
	return true;
}

uint8_t appBmsSetScuCurrent(uint8_t scuid, int32_t CurrentP, int32_t CurrentN)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	scuid--;
	mBmsBuf[scuid].CurrentP = CurrentP;
	mBmsBuf[scuid].CurrentN = CurrentN;
	mBmsBuf[scuid].DataValidFlag |= BMS_DATA_VALID_DATA_CURRENT;
	updateScuIdleCount(scuid);
//	appBmsDebugMsg("Rcv current");
	return true;
}

uint8_t appBmsGetScuCurrent(uint8_t scuid, int32_t *CurrentP, int32_t *CurrentN)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	scuid--;
	*CurrentP = mBmsBuf[scuid].CurrentP;
	*CurrentN = mBmsBuf[scuid].CurrentN;
	return true;
}

uint8_t appBmsSetScuVbat(uint8_t scuid, uint32_t VbInt, uint32_t VbExt)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	scuid--;
	mBmsBuf[scuid].VbInt = VbInt;
	mBmsBuf[scuid].VbExt = VbExt;
	mBmsBuf[scuid].DataValidFlag |= BMS_DATA_VALID_DATA_VB;
	updateScuIdleCount(scuid);
//	appBmsDebugMsg("Rcv Vb");
	return true;
}
uint8_t appBmsGetScuVbat(uint8_t scuid, uint32_t *VbInt, uint32_t *VbExt)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	scuid--;
	*VbInt = mBmsBuf[scuid].VbInt;
	*VbExt = mBmsBuf[scuid].VbExt;
	return true;
}

uint8_t appBmsSetMinCellVoltage(uint8_t scuid, uint16_t MinV, uint8_t bmu, uint8_t posi)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	scuid--;
	mBmsBuf[scuid].MinVoltage = MinV;
	mBmsBuf[scuid].MinVoltageBmu = bmu;
	mBmsBuf[scuid].MinVoltagePosition = posi;
	return true;
}
uint8_t appBmsGetMinCellVoltage(uint8_t scuid, uint16_t *MinV, uint8_t *bmu, uint8_t *posi)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	scuid--;
	*MinV = mBmsBuf[scuid].MinVoltage;
	*bmu = mBmsBuf[scuid].MinVoltageBmu;
	*posi = mBmsBuf[scuid].MinVoltagePosition;
	return true;
}

uint8_t appBmsSetMaxCellVoltage(uint8_t scuid, uint16_t MaxV, uint8_t bmu, uint8_t posi)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	scuid--;
	mBmsBuf[scuid].MaxVoltage = MaxV;
	mBmsBuf[scuid].MaxVoltageBmu = bmu;
	mBmsBuf[scuid].MaxVoltagePosition = posi;
	return true;
}

uint8_t appBmsGetMaxCellVoltage(uint8_t scuid, uint16_t *MaxV, uint8_t *bmu, uint8_t *posi)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	scuid--;
	*MaxV = mBmsBuf[scuid].MaxVoltage;
	*bmu = mBmsBuf[scuid].MaxVoltageBmu;
	*posi = mBmsBuf[scuid].MaxVoltagePosition;
	return true;
}
uint8_t appBmsSetMinNtcTemp(uint8_t scuid, uint16_t MinT, uint8_t bmu, uint8_t posi)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	scuid--;
	mBmsBuf[scuid].MinNtcTemp = MinT;
	mBmsBuf[scuid].MinNtcBmu = bmu;
	mBmsBuf[scuid].MinNtcPosition = posi;
	return true;
}
uint8_t appBmsGetMinNtcTemp(uint8_t scuid, uint16_t *MinT, uint8_t *bmu, uint8_t *posi)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	scuid--;
	*MinT = mBmsBuf[scuid].MinNtcTemp;
	*bmu = mBmsBuf[scuid].MinNtcBmu;
	*posi = mBmsBuf[scuid].MinNtcPosition;
	return true;
}

uint8_t appBmsSetMaxNtcTemp(uint8_t scuid, uint16_t MaxT, uint8_t bmu, uint8_t posi)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	scuid--;
	mBmsBuf[scuid].MaxNtcTemp = MaxT;
	mBmsBuf[scuid].MaxNtcBmu = bmu;
	mBmsBuf[scuid].MaxNtcPosition = posi;		
	return true;
}

uint8_t appBmsGetMaxNtcTemp(uint8_t scuid, uint16_t *MaxT, uint8_t *bmu, uint8_t *posi)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	scuid--;
	*MaxT = mBmsBuf[scuid].MaxNtcTemp;
	*bmu = mBmsBuf[scuid].MaxNtcBmu;
	*posi = mBmsBuf[scuid].MaxNtcPosition ;
	return true;
}

uint8_t appBmsSetCellVoltage(uint8_t scuid, uint16_t cells, uint16_t voltage)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	if(cells >= MAX_CELL_NUMBER)
		return false;
	scuid--;
	mBmsBuf[scuid].CellVoltage[cells] = voltage;
	return true;
}

uint8_t appBmsGetCellVoltage(uint8_t scuid, uint16_t cells, uint16_t *voltage)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	if(cells >= MAX_CELL_NUMBER)
		return false;
	scuid--;
	*voltage = mBmsBuf[scuid].CellVoltage[cells];
	return true;
}


uint8_t appBmsSetNtcVoltage(uint8_t scuid, uint16_t ntcs, uint16_t voltage)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	if(ntcs >= MAX_NTC_NUMBER)
		return false;
	scuid--;
	mBmsBuf[scuid].NtcVoltage[ntcs] = voltage;
	return true;
}

uint8_t appBmsGetNtcVoltage(uint8_t scuid, uint16_t ntcs, uint16_t *voltage)
{
	if(appBmsIsValidScuid(scuid) == 0)
		return false;
	if(ntcs >= MAX_NTC_NUMBER)
		return false;
	scuid--;
	*voltage = mBmsBuf[scuid].NtcVoltage[ntcs];
	return true;
}

void appBmsOpen(void)
{
	memset(&mBmsBuf, 0, sizeof(mBmsBuf));
	LibSwTimerOpen(bmsSwTimerHandler, 0);
}
/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    







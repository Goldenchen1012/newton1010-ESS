/**
  ******************************************************************************
  * @file        AppBalance.h
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/12
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
#include "main.h"
#include "halafe.h"
#include "LibSwTimer.h"
#include "AppBalance.h"
#include "LibNtc.h"
#include "AppGauge.h"
#include "ApiSysPar.h"

void appSerialCanDavinciSendTextMessage(char *msg);

#define	appBalanceDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)


/* Private define ------------------------------------------------------------*/
#define	balanceCellNumber()		apiSysParGetCellNumber()
#define	balanceBmuNumber()		apiSysParGetBmuNumber()
/* Private macro -------------------------------------------------------------*/
static const uint8_t BitsTab[]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
static const uint8_t BitsTabMask[]={0xFE,0xFD,0xFB,0xF7,0xEF,0xDF,0xBF,0x7F};

/* Private typedef -----------------------------------------------------------*/

static uint8_t	BalanceFlag[(MAX_CELL_NUMBER / 8) + 1]={0};
static uint8_t	BalanceFlagTemp[(MAX_CELL_NUMBER / 8) + 1]={0};


/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
void (*balanceFunProcessor)(void) = {0};

static tAppBalanceEvtHandler EvtHandler;
static uint16_t	BalanceDutyCount = 0;
static tScuProtectPar	mDutyCondition;

/* Private function prototypes -----------------------------------------------*/
static uint8_t getPhysicalCellPosition(uint16_t CellIndex, uint8_t *BmuIndex,uint8_t *Position)
{
	uint8_t		bmu;
	uint32_t	CellFlag;
	uint8_t		i;
	uint16_t	CellNum = 0;

	for(bmu=0; bmu<balanceBmuNumber(); bmu++)
	{
		CellFlag = apiSysParGetCellFlag(bmu);
		for(i=0; i<32; i++)
		{
			if(CellFlag & 0x01)
			{
				if(CellNum == CellIndex)
				{
					*BmuIndex = bmu;
					*Position = i;
					return 1;
				}
				CellNum++;
			}
			CellFlag >>= 1;
		}
	}
	return 0;
}
static void setupPhysicalBalancePosition(void)
{
	uint8_t		Bmu,Position;
	uint16_t	index, cell;
	uint8_t		B,b;
	uint8_t		diff,dat;
	uint32_t	PhysicialPosition[64];
	char	str[100];
	uint8_t		flag = 0;
	memset(&PhysicialPosition, 0, sizeof(PhysicialPosition));

	//appBalanceDebugMsg("setupPhysicalBalancePosition");
	
	B = 0;
	for(cell=0; cell<balanceCellNumber(); B++)
	{
		for(b=0; b<8; b++,cell++)
		{
			if(cell >= balanceCellNumber())
				break;
			if(!getPhysicalCellPosition(cell, &Bmu, &Position))
				continue;
			if(!flag)
			{
				//sprintf(str,"chk balance %d %d %d",cell, Bmu, Position);
				//appBalanceDebugMsg(str);
			}
			flag = 1;
			if(BalanceFlag[B] & BitsTab[b])
			{
				PhysicialPosition[Bmu] |= (1 << Position);
				//sprintf(str,"Balance %d %d",Bmu, Position);
				//appBalanceDebugMsg(str);
			}
		}
	}	
	for(Bmu=0; Bmu < balanceBmuNumber(); Bmu++)
	{
		uint32_t	d;
		halAfeSetPhysicalBalancePosition(Bmu, PhysicialPosition[Bmu]);
		//sprintf(str,"Physical: %d %.8lX",Bmu, PhysicialPosition[Bmu]);
		//appBalanceDebugMsg(str);
		//for(d=0; d<5000; d++);
	}
	//HalAfeSetPhysicalBalancePosition
	balanceFunProcessor = 0;
}

static void appBalanceCheckEvent(void)
{
	uint16_t	index, cell;
	uint8_t		B,b;
	uint8_t		diff,dat;
	
	B = 0;
	for(cell=0; cell<balanceCellNumber(); cell+=8,B++)
	{  
		diff = BalanceFlag[B] ^ BalanceFlagTemp[B];
		if(diff)
		{
			dat = BalanceFlag[B];
			index = cell;
			for(b=0; b<8; b++,index++)
			{
				if(index >= balanceCellNumber())
					break;
								
				if(diff & BitsTab[b])
				{
					if(dat & BitsTab[b])
					{
						if(EvtHandler)
						{
							switch(appGaugeGetCurrentMode())
							{
							case APP_SCU_GAUGE_CHARGE_MODE:
								EvtHandler(0,APP_CHG_BALANCE_SET, &index);
								break;
							case APP_SCU_GAUGE_DISCHARGE_MODE:
								EvtHandler(0,APP_DHG_BALANCE_SET, &index);
								break;
							case APP_SCU_GAUGE_RELEASE_MODE:
								EvtHandler(0,APP_RLX_BALANCE_SET, &index);
								break;
							}
						}
					}
					else			//Balance Release
					{
						if(EvtHandler)
						{
							switch(appGaugeGetCurrentMode())
							{
							case APP_SCU_GAUGE_CHARGE_MODE:
								EvtHandler(0,APP_CHG_BALANCE_RELEASE, &index);
								break;
							case APP_SCU_GAUGE_DISCHARGE_MODE:
								EvtHandler(0,APP_DHG_BALANCE_RELEASE, &index);
								break;
							case APP_SCU_GAUGE_RELEASE_MODE:
								EvtHandler(0,APP_RLX_BALANCE_RELEASE, &index);
								break;
							}
						}
					}	//if(dat&0x01)
				}	//if(diff&0x01)
			}
		}
		BalanceFlagTemp[B] = BalanceFlag[B];
	}
	
	balanceFunProcessor = setupPhysicalBalancePosition;
}

static void appBalanceCheckSet(uint16_t MinCellVoltage, uint16_t DeltaSet)
{
	uint16_t	cell;
	uint16_t	CellVoltage,DeltaVoltage;
	char		str[100];

	for(cell=0; cell<balanceCellNumber(); cell++)
	{		
		CellVoltage = halAfeGetCellVoltage(cell);
		if(CellVoltage >= MinCellVoltage)
			DeltaVoltage = CellVoltage - MinCellVoltage;		
		else
			DeltaVoltage = 0;
		if(DeltaVoltage > DeltaSet)
		{
			BYTE	B,b;

			B = cell/8;
			b = cell & 0x07;
			BalanceFlag[B] |= BitsTab[b];
			if(cell == 10)
			{					
				sprintf(str,"Set Balance %d %d %d",cell+1, CellVoltage, MinCellVoltage);
				//appBalanceDebugMsg(str);
			}
		}		
	}
}
static void appBalanceCheckRelease(uint16_t MinCellVoltage, uint16_t DeltaRelease)
{
	uint16_t	cell;
	uint16_t	CellVoltage,DeltaVoltage;
	
	for(cell=0; cell<balanceCellNumber(); cell++)
	{
		//---------------------------------------------------
		//	check release
		CellVoltage = halAfeGetCellVoltage(cell);
		if(CellVoltage >= MinCellVoltage)
			DeltaVoltage = CellVoltage - MinCellVoltage;
		else
			CellVoltage = 0;	

		if(DeltaVoltage < DeltaRelease)
		{	
			BYTE	B,b;	
			B = cell / 8;
			b = cell & 0x07;
			BalanceFlag[B] &= BitsTabMask[b];
			
		//	sprintf(str,"Release Balance %d",cell+1);
		//	appBalanceDebugMsg(str);
		}
	}	
}
static void appBalanceClearAllFlag(void)
{
	memset(&BalanceFlag, 0, sizeof(BalanceFlag));
}
static void nextBalanceCheckProcess(void)
{
	balanceFunProcessor = appBalanceCheckEvent;
}

static void appBalanceCheck(void)
{	
	tScuProtectPar	mVoltageCondition;
	uint8_t		MaxNtcTemp_1C;
	uint16_t	DutySet;
	uint16_t	DutyRest;
	uint16_t	TempSet;
	uint16_t	TempRelease;
	
	uint16_t	CellVoltage,DeltaVoltage;
	uint16_t	MinCellVoltage;
	uint16_t	MaxCellVoltage;
	
	uint16_t	SetVoltage,ReleaseVoltage;
	uint16_t	DeltaSet,DeltaRelease;
	
	uint16_t	cell,index,i;
	BYTE	B,b;
	BYTE	diff;
	BYTE	dat;
	char	str[100];
	
	if(appGaugeGetCurrentMode() == APP_SCU_GAUGE_CHARGE_MODE)
	{
		apiSysParGetBalanceChg(&mVoltageCondition);
	}
	else if(appGaugeGetCurrentMode() == APP_SCU_GAUGE_RELEASE_MODE)
	{
		apiSysParGetBalanceRlx(&mVoltageCondition);
	}
	else
	{
		nextBalanceCheckProcess();
		return;
	}
	
	SetVoltage = mVoltageCondition.SetValue.l;
	ReleaseVoltage = mVoltageCondition.STime.l;
	DeltaSet = mVoltageCondition.RelValue.l;
	DeltaRelease = mVoltageCondition.RTime.l;
	
	apiSysParGetBalanceDuty(&mDutyCondition);

	DutySet = mDutyCondition.SetValue.l;
	DutyRest = mDutyCondition.STime.l;
	TempSet = mDutyCondition.RelValue.l;
	TempRelease = mDutyCondition.RTime.l;

	MaxNtcTemp_1C = LibNtcVoltageToTemperature(HalAfeGetMaxNtcTempAdc()) / 100;
	MinCellVoltage = halAfeGetMinCellVoltage();
	MaxCellVoltage = halAfeGetMaxCellVoltage();


	sprintf(str,"%d %d %d %d %d %d %d %d %d %d %d",
			SetVoltage,
			ReleaseVoltage,
			DeltaSet,
			DeltaRelease,
			DutySet,
			DutyRest,
			TempSet,
			TempRelease,
			MaxNtcTemp_1C,
			MinCellVoltage,
			MaxCellVoltage
			);
	//appBalanceDebugMsg(str);
	//---------------------------------------------
	//
	if(MinCellVoltage <= SetVoltage || 
	   MaxNtcTemp_1C >= TempSet)
	 {
	 	if(MaxCellVoltage < ReleaseVoltage || 
   	   		MaxNtcTemp_1C > TempRelease)
		{	
			appBalanceClearAllFlag();
			nextBalanceCheckProcess();
			return;
		}
		appBalanceCheckRelease(MinCellVoltage, DeltaRelease);
		nextBalanceCheckProcess();
		return;
	}
	appBalanceCheckSet(MinCellVoltage, DeltaSet);
	
	//-------------------------------------------------------------
	if(MaxCellVoltage < ReleaseVoltage || 
   	   MaxNtcTemp_1C > TempRelease)
	{	
		appBalanceClearAllFlag();
		nextBalanceCheckProcess();
		return;
	}
	appBalanceCheckRelease(MinCellVoltage, DeltaRelease);
	nextBalanceCheckProcess();
}


static void appBalanceSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
//	GPIOD->ODR |= GPIO_PIN_14;
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
		if(balanceFunProcessor)
			balanceFunProcessor();
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		apiSysParGetBalanceDuty(&mDutyCondition);

		if(!balanceFunProcessor)
			balanceFunProcessor = appBalanceCheck;

		if(mDutyCondition.SetValue.l == 0)
			halAfeSetBalanceOnFlag(0);
		else if(mDutyCondition.SetValue.l !=0 && mDutyCondition.STime.l == 0)
			halAfeSetBalanceOnFlag(1);
		else
		{
			BalanceDutyCount++;
			if(BalanceDutyCount ==  mDutyCondition.SetValue.l)
			{
				halAfeSetBalanceOnFlag(0);
				//appBalanceDebugMsg("Balance Switch = 0");
			}
			else if(BalanceDutyCount >= (mDutyCondition.SetValue.l + mDutyCondition.STime.l))
			{
				BalanceDutyCount = 0;
				halAfeSetBalanceOnFlag(1);
				//appBalanceDebugMsg("Balance Switch = 1");
			}
		}
	}
//	GPIOD->ODR &= ~GPIO_PIN_14;
}


/* Public function prototypes -----------------------------------------------*/
uint8_t	appBalanceIsBalanceSet(uint16_t cell)
{
	uint8_t	B,b;

	B = cell / 8;
	b = cell & 0x07;
	if(BalanceFlag[B] & BitsTab[b])
		return 1;
	else
		return 0;
}

void appBalanceOpen(tAppBalanceEvtHandler evtHandler)
{  	
	EvtHandler = evtHandler;
	balanceFunProcessor = 0;
	LibSwTimerOpen(appBalanceSwTimerHandler, 0);
}


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



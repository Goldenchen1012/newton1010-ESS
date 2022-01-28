/**
  ******************************************************************************
  * @file        AppRelayControl.c
  * @author      Johnny
  * @version     v0.0.0
  * @date        2021/11/10
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
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "define.h"
#include "LibDebug.h"
#include "halafe.h"
#include "smp_drv_bq796xx.h"
#include "smp_uart.h"
#include "LibSwTimer.h"
#include "LibHwTimer.h"
#include "ApiSysPar.h"
#include "AppProject.h"
#include "ApiSystemFlag.h"
#include "HalBsp.h"
#include "ApiRelayControl.h"

void appSerialCanDavinciSendTextMessage(char *msg);
#define	appRelayControlDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private define ------------------------------------------------------------*/
#define	PreRelayOnTime10Ms()	(apiSysParGetPreDischargeTime() * 10)
#ifdef DEBUG_MODE1
	#define	RELAY_POWER_ON()	//HalBspRelayPsCtrl(1)
	#define	RELAY_POWER_OFF()	//HalBspRelayPsCtrl(0)
#else
	#define	RELAY_POWER_ON()	HalBspRelayPsCtrl(1)
	#define	RELAY_POWER_OFF()	HalBspRelayPsCtrl(0)
#endif

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
enum{
	RELAY_CTRL_INI =0,
	RELAY_CTRL_TURN_ON_PRE_RELAY,
	RELAY_CTRL_TURN_OFF_PRE_RELAY,
	RELAY_CTRL_PRE_DISCHARGE_ON,
	RELAY_CTRL_PRE_DISCHARGE_DELAY,
	RELAY_CTRL_CHECK_BUS_VOLTAGE,
	RELAY_CTRL_TURN_ON_MAIN_RELAY,
	RELAY_CTRL_END
};
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
void (*apiRelayControlFunctionProcessor)(void) = {0};
static uint8_t MasterTurnOnRelayFlag = 0;
static uint8_t RelayOnStep = RELAY_CTRL_INI;
static uint16_t DelayCount;	
/* Private function prototypes -----------------------------------------------*/

static void turnOnRelayProcess(void)
{
	switch(RelayOnStep)
	{
	case RELAY_CTRL_TURN_ON_PRE_RELAY:
	 	DelayCount = 0;
	 	halBspPreDischargeRelayOn();
	 	RelayOnStep = RELAY_CTRL_PRE_DISCHARGE_DELAY;
		break;
	case RELAY_CTRL_PRE_DISCHARGE_DELAY:
		DelayCount++;
		if(DelayCount >= PreRelayOnTime10Ms())
		{
			RelayOnStep = RELAY_CTRL_TURN_ON_MAIN_RELAY;
			appRelayControlDebugMsg("Main On");
		}
		break;
	case RELAY_CTRL_TURN_ON_MAIN_RELAY:
		DelayCount = 0;
		RelayOnStep = RELAY_CTRL_TURN_OFF_PRE_RELAY;
		halBspPostiveRelayOn();
		halBspNegtiveRelayOn();	
		appProjectSetRelayOnFlag();
		break;
	case RELAY_CTRL_TURN_OFF_PRE_RELAY:
		DelayCount++;
//		if(DelayCount >= 500)
		if(DelayCount >= 100)
		{
			halBspPreDischargeRelayOff();
			apiRelayControlFunctionProcessor = 0;
			RelayOnStep = RELAY_CTRL_INI;
			MasterTurnOnRelayFlag = 0;
		}
		break;
	}
}

static uint8_t isSystemRelay(void)
{
	uint32_t	SystemFlag1;
	SystemFlag1 = apiSystemFlagGetFlag1();
	if((SystemFlag1 & SYSTEM_FLAG1_SYSTEM_READY))
		return 1;
	return 0;
}

static uint8_t isRelayOn(void)
{
	uint32_t	SystemFlag2;
	SystemFlag2 = apiSystemFlagGetFlag2();
	if(SystemFlag2 & SYSTEM_FLAG2_RELAY_ON)
		return 1;
	return 0;
}
static uint8_t isInProtectState(void)
{
	uint32_t	SystemFlag1, SystemFlag2;
	uint32_t	SystemFlag3, SystemFlag4;
	
	SystemFlag1 = apiSystemFlagGetFlag1();
	SystemFlag2 = apiSystemFlagGetFlag2();
	SystemFlag3 = apiSystemFlagGetFlag3();
	SystemFlag4 = apiSystemFlagGetFlag4();


	if(SystemFlag1 & FLAG1_PROTECT_MASK)
		return 1;
	if(SystemFlag2 & FLAG2_PROTECT_MASK)
		return 1;
	if(SystemFlag3 & FLAG3_PROTECT_MASK)
		return 1;
	if(SystemFlag4 & FLAG4_PROTECT_MASK)
		return 1;
	return 0;
}

static void checkRelayOn(void)
{
	uint32_t	SystemFlag1, SystemFlag2;
	
	SystemFlag1 = apiSystemFlagGetFlag1();
	SystemFlag2 = apiSystemFlagGetFlag2();
	
	if(isRelayOn())
		return;

	if(appProjectIsInEngMode())
		return;
		
	if(MasterTurnOnRelayFlag == 0)		
		return;
		
	if(isSystemRelay() == 0)
		return;
	
	if(isInProtectState())
		return;

	if(RelayOnStep != RELAY_CTRL_INI)
		return;
		
	appRelayControlDebugMsg("On Pre..0");
	if(apiRelayControlFunctionProcessor == 0)
	{
		RelayOnStep = RELAY_CTRL_TURN_ON_PRE_RELAY;
		apiRelayControlFunctionProcessor = turnOnRelayProcess;
		RELAY_POWER_ON();
		appRelayControlDebugMsg("On Pre..1");
	}
}

static void relayControlSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
//	GPIOD->ODR |= GPIO_PIN_14;
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_3)
	{
		if(apiRelayControlFunctionProcessor)
			apiRelayControlFunctionProcessor();
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		checkRelayOn();
		//appRelayControlDebugMsg("Relay Control");
	}
//	GPIOD->ODR &= ~GPIO_PIN_14;
}

/* Public function prototypes -----------------------------------------------*/
void apiRelayControlSetMasterTurnOnFlag(void)
{
	MasterTurnOnRelayFlag = 1;
}

void apiRelayControlMainRelayOff(void)
{
	RELAY_POWER_OFF();
	
	halBspPostiveRelayOff();
	halBspNegtiveRelayOff();
	
	halBspPreDischargeRelayOff();
	apiRelayControlFunctionProcessor = 0;
	RelayOnStep = RELAY_CTRL_INI;
	MasterTurnOnRelayFlag = 0;
}

void apiRelayControlOpen(void)
{
	MasterTurnOnRelayFlag = 0;
	LibSwTimerOpen(relayControlSwTimerHandler, 0);
}
/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


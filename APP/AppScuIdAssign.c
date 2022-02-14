/**
  ******************************************************************************
  * @file        AppScuIdAddign.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2022/2/9
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
#include "AppScuIdAssign.h"
#include "ApiSystemFlag.h"
#include "ApiRelayControl.h"
#include "AppSerialCanDavinci.h"
#include "ApiSignalFeedback.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	appScuIdAssignDebugMsg(str)	//appSerialCanDavinciSendTextMessage(str)

#define	CHECK_SCU_ID

/* Private define ------------------------------------------------------------*/
#define	relayOnDiffVoltage()		apiSysParGetRelayOnDiffVoltage()
#define	MASTER_BTN_CLICK_COUNT		1500
#define	SIGNAL_T_BASE_COUNT			15
#define	ASSIGN_ID_HI_BASE_NUM		2
#define	ASSIGN_ID_LO_BASE_NUM		2
#define	FIND_SCUID_HI_BASE_NUM		3
#define	FIND_SCUID_LO_BASE_NUM		1
#define	VALID_CYCLE_COUNT			((SIGNAL_T_BASE_COUNT * 4) * 9 / 10)
#define	MAX_CYCLE_COUNT				( 4 * SIGNAL_T_BASE_COUNT)

#define	VALID_MODE_COUNT			3

#define	FIND_FIRST_SCU_COUNT		2000
#define	ASSIGN_NEXT_SCUID_COUNT		5000

#define	FIND_FIRST_SCUID_DUTY_L		65
#define	FIND_FIRST_SCUID_DUTY_H		85

#define HOST_SETUP_ID_MODE_DUTY_L	40
#define	HOST_SETUP_ID_MODE_DUTY_H	60
#define	STANDBY_MODE_DUTY			5

#define	BMS_DATA_VALID_DATA_VB				0x0001
#define	BMS_DATA_VALID_DATA_CURRENT			0x0002
#define	BMS_DATA_VALIE_DATA_SYS_FLAG1_2		0x0004
#define	BMS_DATA_VALIE_DATA_SYS_FLAG3_4		0x0008

#define	BMS_DATA_VALID_FLAG_ALL				0x000F

#define	WAIT_LAST_OD_IN_COUNT_10MS			10
#define	NEXT_SCU_ID		(ScuID + 1)

enum{
	OD_IN_MODE_STANDBY = 1,
	OD_IN_MODE_HOST_SETUP_ID,
	OD_IN_MODE_MSATER,
	OD_IN_MODE_FIND_FIRST_SCUID,
	OD_IN_MODE_UNKNOW
};

/* Private typedef -----------------------------------------------------------*/

#ifdef CHECK_SCU_ID
	static uint8_t	ScuID = 0xff;
#else
	static uint8_t	ScuID = 0x01;
#endif
static uint8_t	MasterFlag = 0;
static uint8_t	WaitTime;
static uint8_t  OdInHiCount = 0;
static uint8_t	OdInLoCount = 0;
static uint8_t	OdInSignalMode = OD_IN_MODE_UNKNOW;
static void (*OdOutSignalProcessor)(void) = {0};

static uint8_t	AssignNextScuIdModeFlag = 0;
static uint8_t	ScuIdRequestModeFlag = 0;
static uint16_t	OdOutOutputCount = 0;

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void assignSlaveIdSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr);
static void masterSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr);

static void checkWhichScuRelayOn(void)
{
	uint8_t		ScuIndex;
	uint8_t		RelayOnNum = 0;
	uint8_t		MaxVbScuID = 0xff;
	uint32_t	MaxVbVoltage = 0;
	uint32_t	VbIntAvg = 0;
	uint32_t	vb;
	uint32_t	diffv;
	uint32_t	VbInt,VbExt;
//	tBmsBuf		*pBmsBuf;
	smp_can_package_t CanPkg;
	
	char	str[100];
	//----------------------------------------------
	//	get	
	for(ScuIndex=1; ScuIndex<=SYSTEM_MAX_SCU_NUM; ScuIndex++)
	{
#if 0 //jjjjjjjjjjj
		pBmsBuf = &mBmsBuf[ScuIndex];

		if(pBmsBuf->IdleCount == 0)
			continue;
		if(isDataValid(pBmsBuf) == 0)
			continue;
		if(isScuProtect(pBmsBuf))
			continue;
		if(isRelayOn(pBmsBuf))
#endif		
		if(appBmsIsScuRelayOn(ScuIndex))
		{
			RelayOnNum++;
			appBmsGetScuVbat(ScuIndex, &VbInt, &VbExt);
			VbIntAvg += VbInt;
		}
	}
	
	//sprintf(str,"Turn On Relay Num=%d",RelayOnNum);
	//appScuIdAssignDebugMsg(str);
	
	if(RelayOnNum == 0)
	{		
		for(ScuIndex=1; ScuIndex<=SYSTEM_MAX_SCU_NUM; ScuIndex++)
		{
#if 0	//jjjjjjjjjjjjjjjjjjjjjjjjjj
			pBmsBuf = &mBmsBuf[ScuIndex];
			if(pBmsBuf->IdleCount == 0)
				continue;
			if(isDataValid(pBmsBuf) == 0)
				continue;
			if(isScuProtect(pBmsBuf))
				continue;
			if(isRelayOn(pBmsBuf))
				continue;
#endif			
				/// 
#if 0			
			!!!
			sum all cell to VBat value
			!!!
#endif
			if(appBmsIsScuCanTurnOnRelay(ScuIndex) == true)
			{
				appBmsGetScuVbat(ScuIndex, &VbInt, &VbExt);

				if(VbInt > MaxVbVoltage)	//jjjjjjj
				{
					MaxVbVoltage = VbInt;	//jjjjjjj
					MaxVbScuID = ScuIndex;
				}
			}
		}
		if(MaxVbScuID != 0xff)	//有relay 需要被打開
		{
			sprintf(str,"有要開啟的 SCU:%d / %d", MaxVbScuID, ScuID);
			appScuIdAssignDebugMsg(str);
			if(MaxVbScuID == ScuID)
			{
				apiRelayControlSetMasterTurnOnFlag();
				appScuIdAssignDebugMsg("Turn On Self Relay");
				//TurnOnMos();
			}
			else
			{
				CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, MaxVbScuID,
									SMP_CMD_RELAY_ON_OBJ_INDEX,
									ScuID);
				CanPkg.dlc = 7;
				memcpy(&CanPkg.dat[0], "RelayOn", 7);
				appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
				appScuIdAssignDebugMsg("Turn On Other Relay");
			}
		}
	}
	else
	{
		VbIntAvg /= RelayOnNum;
		VbIntAvg /= 100L;
		
		for(ScuIndex=1; ScuIndex<=SYSTEM_MAX_SCU_NUM; ScuIndex++)
		{
#if 0 //jjjjjjjjjjjjjjjj
			pBmsBuf = &mBmsBuf[ScuIndex];
			if(pBmsBuf->IdleCount == 0)
				continue;
			if(isDataValid(pBmsBuf) == 0)
				continue;
			if(isScuProtect(pBmsBuf))
				continue;
			if(isRelayOn(pBmsBuf))
				continue;
			vb = pBmsBuf->VbInt;
#endif
			if(appBmsIsScuCanTurnOnRelay(ScuIndex) == false)
				continue;
			appBmsGetScuVbat(ScuIndex, &VbInt, &VbExt);
			VbInt /= 100L;
			diffv = abs(VbIntAvg - vb);
			sprintf(str,"壓差 = %d", diffv);
			appScuIdAssignDebugMsg(str);
			if(diffv < relayOnDiffVoltage())	//parameter diff v resolution:0.1V
			{
				if((ScuIndex + 1) == ScuID)
				{	
					apiRelayControlSetMasterTurnOnFlag();
					appScuIdAssignDebugMsg("Turn On Self Relay");
				}
				else
				{
					CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, ScuIndex,
										SMP_CMD_RELAY_ON_OBJ_INDEX,
										ScuID);
					CanPkg.dlc = 7;
					memcpy(&CanPkg.dat[0], "RelayOn", 7);
					appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
					appScuIdAssignDebugMsg("Turn On Other Relay");
				}
			}
			else
			{
				appScuIdAssignDebugMsg("壓差過大，無法並上 BUS");
			}
		}
	}
}


static void outputNextScuCanRequestIdSignal(void)	//2T+2T
{
//	static uint16_t	outputCount = 0;
	static	uint8_t	count = MAX_CYCLE_COUNT;
	
	count++;
	OdOutOutputCount++;
	
	if(count >= ((ASSIGN_ID_HI_BASE_NUM + ASSIGN_ID_LO_BASE_NUM) * SIGNAL_T_BASE_COUNT))
	{
		count = 0;
		if(OdOutOutputCount >= ASSIGN_NEXT_SCUID_COUNT)
		{
			OdOutOutputCount = 0;
			HalBspOdOutCtrl(0);
			//appScuIdAssignDebugMsg("Stop OD Out");
			OdOutSignalProcessor = 0;
		}
		else
			HalBspOdOutCtrl(1);
	}
	else if(count == (ASSIGN_ID_HI_BASE_NUM * SIGNAL_T_BASE_COUNT))
		HalBspOdOutCtrl(0);
	
}
static void sendResetScuIdCanPackage(void)
{
	smp_can_package_t	CanPkg;
	uint8_t	u8;

	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_COMMON_RX, 0,
									SMP_COMMON_RESET_SCU_ID_OBJ_INDEX,
									0);

	CanPkg.dlc = 8;
	memcpy(&CanPkg.dat[0], "RstScuID", 8);
	appScuIdAssignDebugMsg("Reset All ScuID");

	for(u8=0; u8<5; u8++)
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}

static void enterMasterMode(void)
{
	ScuID = 1;	
	sendResetScuIdCanPackage();
	saveScuIdPar(1);
	MasterFlag = 1;
	appBmsExitScuIdRequestMode();
	LibSwTimerOpen(masterSwTimerHandler, 0);
	appBmsEnterScuIdAssignMode();
	appScuIdAssignDebugMsg("I am Master");
}

static void outputFindFirstScuSignal(void)	//3T+1T
{
//	static uint16_t	outputCount = 0;
	static	uint8_t	count = MAX_CYCLE_COUNT;
	
	count++;
	OdOutOutputCount++;
	if(count >= ((FIND_SCUID_HI_BASE_NUM + FIND_SCUID_LO_BASE_NUM) * SIGNAL_T_BASE_COUNT))
	{
		count = 0;
		if(OdOutOutputCount >= FIND_FIRST_SCU_COUNT)
		{
			OdOutOutputCount = 0;
			HalBspOdOutCtrl(0);
			//appScuIdAssignDebugMsg("Stop OD Out");
			OdOutSignalProcessor = 0;
		
			if(OdInSignalMode == OD_IN_MODE_UNKNOW)	
			{
				appScuIdAssignDebugMsg("I am first SCU !!");			
				enterMasterMode();
			}
		}
		else
			HalBspOdOutCtrl(1);
	}
	else if(count == (FIND_SCUID_HI_BASE_NUM * SIGNAL_T_BASE_COUNT))
		HalBspOdOutCtrl(0);
}

static void masterProcessor(void)
{
	if(ScuID == 0 || ScuID >= SYSTEM_MAX_SCU_NUM)
		return;
	if(appProjectIsSystemReadyFlag() == 0)
		return;

	if(!MasterFlag)
		return;
		
	checkWhichScuRelayOn();
}

static uint8_t getOdInMode(void)
{
	uint16_t 	duty,cycle;
	char	str[100];
	
	cycle = OdInHiCount + OdInLoCount;
	
	if(cycle < VALID_CYCLE_COUNT)
		return OD_IN_MODE_UNKNOW;

	duty = (uint16_t)OdInHiCount * 100;
	duty /= cycle;
	
//	sprintf(str,"OD in Duty = %d", duty);
//	appScuIdAssignDebugMsg(str);

	if(duty >= FIND_FIRST_SCUID_DUTY_L && duty <= FIND_FIRST_SCUID_DUTY_H)
		return OD_IN_MODE_FIND_FIRST_SCUID;
	else if(duty >= HOST_SETUP_ID_MODE_DUTY_L && duty <= HOST_SETUP_ID_MODE_DUTY_H)
		return OD_IN_MODE_HOST_SETUP_ID;
	else if(duty < STANDBY_MODE_DUTY)
		return OD_IN_MODE_STANDBY;
	else 
		return OD_IN_MODE_UNKNOW;
}

static uint8_t getOdInSignalDuty(void)
{
	static uint8_t	odin_status = 2;
	static	uint8_t	count = 0;
	static	uint16_t	btn_count = 0;
	static	uint8_t		wait_release_flag = 0;
	
	if(count < 255)
		count++;
		
	if(apiSignalFeedbackGetStatus(APP_SIGNAL_ID_BUTTON) == 0)
	{
		if(wait_release_flag == 0)
		{
			btn_count++;
			if(btn_count == MASTER_BTN_CLICK_COUNT)
			{
				wait_release_flag  = 20;
				return OD_IN_MODE_MSATER;
			}
		}
	}
	else
	{
		if(wait_release_flag)
			wait_release_flag--;
		btn_count = 0;	
	}

	if(apiSignalFeedbackGetStatus(APP_SIGNAL_ID_OD_IN) == 0)
	{
		if(odin_status != 0)
		{
			OdInHiCount = count;
			count = 0;
			odin_status = 0;
		}

		if(count == MAX_CYCLE_COUNT)
		{
			OdInHiCount = 0;
			OdInLoCount = MAX_CYCLE_COUNT;
			count = 0;
			return getOdInMode();
		}
	}
	else
	{
		if(odin_status != 1)
		{
			OdInLoCount = count;
			count = 0;
			odin_status = 1;
			return getOdInMode();
		}
		if(count == MAX_CYCLE_COUNT)
		{
			OdInHiCount = MAX_CYCLE_COUNT;
			OdInLoCount = 0;
			count = 0;
			//return getOdInMode();		
		}
	}
	return 0;
}



static void sendRequestIdCanPackage(void)
{
	smp_can_package_t	CanPkg;

	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_COMMON_RX, 0,
									SMP_COMMON_GET_SCUID_OBJ_INDEX,
									0);

	CanPkg.dlc = 8;
	ScuIdRequestModeFlag = 1;
	memcpy(&CanPkg.dat[0], "GetScuID", 8);
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	appScuIdAssignDebugMsg("ID Request");
}

static void checkOdInSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	char	str[100];
	static uint8_t mode_count = 0;
	static uint8_t old_mode = 0xff;
	uint8_t	mode;
	
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
		if(OdOutSignalProcessor)
			OdOutSignalProcessor();
		
		mode = getOdInSignalDuty();
		if(mode == 0)
			return;
		if(old_mode != mode)
		{
			old_mode = mode;
			mode_count = 0;
		}
		
		if(mode == OD_IN_MODE_MSATER)// && ScuID == 0xff)
		{
			appBmsSendFindFirstScuCanPackage();
			appBmsFindFirstScu();
		//	enterMasterMode();
			return; 
		}
		mode_count++;
		if(mode_count < VALID_MODE_COUNT)
			return;
		if(mode == OD_IN_MODE_HOST_SETUP_ID)
		{
			OdInSignalMode = OD_IN_MODE_HOST_SETUP_ID;
			sendRequestIdCanPackage();
		}
		else if(mode == OD_IN_MODE_FIND_FIRST_SCUID)
		{
			OdInSignalMode = OD_IN_MODE_FIND_FIRST_SCUID;
		}
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		mode = getOdInMode();
		switch(mode)
		{
		case OD_IN_MODE_STANDBY:
			appScuIdAssignDebugMsg("OD in Standby Mode");
			break;
		case OD_IN_MODE_HOST_SETUP_ID:
			appScuIdAssignDebugMsg("OD in Setup ID Mode");
			break;
		case OD_IN_MODE_FIND_FIRST_SCUID:
			appScuIdAssignDebugMsg("OD in Find First SCU");
			break;
		case OD_IN_MODE_MSATER:
			appScuIdAssignDebugMsg("OD in Master Mode");
			break;
		case OD_IN_MODE_UNKNOW:
			appScuIdAssignDebugMsg("OD in Unknow Mode");
			break;
		}
	}
}

static void assignSlaveIdSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
		outputNextScuCanRequestIdSignal();
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		appScuIdAssignDebugMsg("Out assign id signal");	
	}
}

static void masterSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
		return;
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		appScuIdAssignDebugMsg("Run Master Processor");
		masterProcessor();
	}
}


/* Public function prototypes -----------------------------------------------*/
void appBmsSendFindFirstScuCanPackage(void)
{
	smp_can_package_t	CanPkg;
	uint8_t	u8;

	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_COMMON_RX, 0,
									SMP_COMMON_FIND_FIRST_SCU_OBJ_INDEX,
									0);

	CanPkg.dlc = 8;
	memcpy(&CanPkg.dat[0], "FirstScu", 8);
	for(u8=0; u8<5; u8++)
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	appScuIdAssignDebugMsg("Find First Scu");
}


void appBmsRcvScuIdBrocast(uint8_t scuid)
{
	char	str[100];
	
	sprintf(str, "My ID = %d RcvId = %d", ScuID, scuid);
	//appScuIdAssignDebugMsg(str);
	appSerialCanDavinciSendTextMessage(str);
	if(scuid == ScuID)
	{
		appScuIdAssignDebugMsg("=========== ID Error ============");
		appBmsSendFindFirstScuCanPackage();
		appBmsFindFirstScu();
	}
}

void appBmsResetScuId(void)
{
	ScuID = 0xff;
	LibSwTimerClose(masterSwTimerHandler, 0);
	AssignNextScuIdModeFlag = 0;
	OdOutSignalProcessor = 0;
	OdOutOutputCount = 0;
	ScuIdRequestModeFlag = 1;
}

void appBmsFindFirstScu(void)
{
	ScuID = 0xff;
	LibSwTimerClose(masterSwTimerHandler, 0);
	AssignNextScuIdModeFlag = 0;
	ScuIdRequestModeFlag = 0;
	OdInSignalMode = OD_IN_MODE_UNKNOW;
	OdOutOutputCount = 0;
	OdOutSignalProcessor = outputFindFirstScuSignal;
}

void appBmsStopOutputAssignIdSignal(void)
{
//	if(AssignNextScuIdModeFlag == 0)
//		return;
	OdOutSignalProcessor = 0;
	
	//LibSwTimerClose(assignSlaveIdSwTimerHandler, 0);
	HalBspOdOutCtrl(0);
}

uint8_t appBmsIsInAssignIdMode(void)
{
	if(AssignNextScuIdModeFlag)
		return 1;
	else
		return 0;
}
uint8_t appBmsIsInScuIdRequestMode(void)
{
	if(ScuIdRequestModeFlag == 0)
		return 0;
		
	if(getOdInMode() == OD_IN_MODE_HOST_SETUP_ID)
		return 1;
	else
		return 0;
}

void  appBmsExitScuIdRequestMode(void)
{
	ScuIdRequestModeFlag = 0;
	//LibSwTimerClose(checkOdInSwTimerHandler, 0);
}
void appBmsEnterScuIdAssignMode(void)
{
	AssignNextScuIdModeFlag = 1;
	OdOutOutputCount = 0;
	OdOutSignalProcessor = outputNextScuCanRequestIdSignal;
	//LibSwTimerOpen(assignSlaveIdSwTimerHandler, 0);
}
uint8_t appBmsGetScuId(void)
{
	return ScuID;
}

void appBmsAssignScuId(uint8_t id)
{
	if(ScuID != 0xff)
		return;
	ScuID = id;
	saveScuIdPar(ScuID);
}

uint8_t appBmsIsScudIdReady(void)
{
	if(ScuID >= 1 && ScuID < SYSTEM_MAX_SCU_NUM)
		return 1;
	else
		return 0;
}

uint8_t appBmsIsMaster(void)
{
	return MasterFlag;
}
void appScuIdAssignOpen(void)
{
#ifdef CHECK_SCU_ID	
	ScuIdRequestModeFlag = 1;
	ScuID = apiSysParGetScuId();
	if(ScuID == 1)
	{
		MasterFlag = 1;
		LibSwTimerOpen(masterSwTimerHandler, 0);
		ScuIdRequestModeFlag = 0;
	}
	{
		char	str[100];
		sprintf(str, "SCUID = %d", ScuID);
		appSerialCanDavinciSendTextMessage(str);
	}
	if(appBmsIsValidScuid(ScuID) == 0)
	{
		appBmsSendFindFirstScuCanPackage();
		appBmsFindFirstScu();
	}
	LibSwTimerOpen(checkOdInSwTimerHandler, 0);
	
#else
	ScuID = 1;	
	MasterFlag = 1;
	appBmsEnterScuIdAssignMode();
	LibSwTimerOpen(masterSwTimerHandler, 0);
#endif	
}
/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    







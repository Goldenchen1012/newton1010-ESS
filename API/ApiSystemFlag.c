/**
  ******************************************************************************
  * @file        ApiSystemFlag.c
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
#include "main.h"
#include "LibDebug.h"
#include "LibSwTimer.h"
#include "LibHwTimer.h"
#include "halafe.h"
#include "halUart.h"
#include "HalCan.h"
#include "HalTimer.h"
#include "HalRtc.h"
#include "HalEeprom.h"
#include "AppProject.h"
#include "ApiSysPar.h"
#include "AppProtect.h"
#include "AppSerialCanDavinci.h"
#include "AppBalance.h"
#include "AppGauge.h"
#include "ApiSignalFeedback.h"
#include "LibNtc.h"
#include "ApiSystemFlag.h"
#include "ApiProtectOvp.h"
#include "ApiProtectUvp.h"
#include "ApiProtectCotp.h"
#include "ApiProtectCutp.h"
#include "ApiProtectDotp.h"
#include "ApiProtectDutp.h"
#include "ApiProtectCocp.h"
#include "ApiProtectDocp.h"
#include "ApiSignalFeedback.h"
#include "HalRtc.h"
#include "AppBms.h"
#include "ApiProtectScuOt.h"
#include "ApiProtectDvp.h"
#include "ApiProtectDtp.h"

void appSerialCanDavinciSendTextMessage(char *msg);

#define	systemFlagDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private define ------------------------------------------------------------*/
#define		sysFlagNtcNumber()		apiSysParGetNtcNumber()
#define		sysFlagCellNumber()		apiSysParGetCellNumber()

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef void (* tSystemFlagCheckFunTable)(void);
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint16_t	SystemFlagSubIndex = 0;
static uint32_t	SystemFlag1Temp;
static uint32_t	SystemFlag2Temp;
static uint32_t	SystemFlag3Temp;
static uint32_t	SystemFlag4Temp;

static uint8_t 	SystemFlagFunIndex = 0;
static uint32_t	SystemFlag1 = 0;
static uint32_t	SystemFlag2 = 0;
static uint32_t	SystemFlag3 = 0;
static uint32_t	SystemFlag4 = 0;

/* Private function prototypes -----------------------------------------------*/
static void systemFlagNextFunction(void)
{
	SystemFlagSubIndex = 0;
	SystemFlagFunIndex++;
}
static void checkFlagNone(void)
{
	systemFlagNextFunction();
}
static void systemFlagCheckFinish(void)
{
	SystemFlag1 = SystemFlag1Temp;
	SystemFlag2 = SystemFlag2Temp;
	SystemFlag3 = SystemFlag3Temp;
	SystemFlag4 = SystemFlag4Temp;

	SystemFlagFunIndex = 0;
	SystemFlagSubIndex = 0;
}
static void checkMinMaxNtcTempVoltage(void)
{
	halAfeUpdateMinMaxNtcTempVoltage();
	systemFlagNextFunction();
}

static void checkMinMaxCellVoltage(void)
{
	halAfeUpdateMinMaxCellVoltage();	
	systemFlagNextFunction();
}

static void checkOtherFlag(void)
{
	if(halAfeIsL1Protect())
		SystemFlag2Temp |= SYSTEM_FLAG2_AFE_L1;
	if(halAfeIsL2Protect())
		SystemFlag2Temp |= SYSTEM_FLAG2_AFE_L2;

	if(appProjectIsRtcValid())
		SystemFlag2Temp |= SYSTEM_FLAG2_RTC_VALID;
		
	if(appBmsIsScudIdReady())
		SystemFlag1Temp |= SYSTEM_FLAG1_CANID_READY;

	if(appProjectIsSystemReadyFlag())
		SystemFlag1Temp |= SYSTEM_FLAG1_SYSTEM_READY;
	
	if(appProjectGetRelayOnFlag())
		SystemFlag2Temp |= SYSTEM_FLAG2_RELAY_ON;
	if(appBmsIsMaster())
		SystemFlag2Temp |= SYSTEM_FLAG2_MASTER;
	
	if(apiProtectOvpPfGetFlag())
		SystemFlag2Temp |= SYSTEM_FLAG2_OVP_PF;
	if(apiProtectUvpPfGetFlag())
		SystemFlag2Temp |= SYSTEM_FLAG2_UVP_PF;		
		
		
	systemFlagNextFunction();
}

static void checkFbSignal(void)
{
	if(apiSignalFeedbackGetStatus(APP_SIGNAL_ID_DI1))
		SystemFlag2Temp |= SYSTEM_FLAG2_DI1;
	if(apiSignalFeedbackGetStatus(APP_SIGNAL_ID_DI2))
		SystemFlag2Temp |= SYSTEM_FLAG2_DI2;
	if(!apiSignalFeedbackGetStatus(APP_SIGNAL_ID_EPO))
		SystemFlag2Temp |= SYSTEM_FLAG2_EPO_ENABLE;
	if(apiSignalFeedbackGetStatus(APP_SIGNAL_ID_SP))
		SystemFlag2Temp |= SYSTEM_FLAG2_SP_FB;
	if(apiSignalFeedbackGetStatus(APP_SIGNAL_ID_PS1))
		SystemFlag2Temp |= SYSTEM_FLAG2_PS1;
	if(apiSignalFeedbackGetStatus(APP_SIGNAL_ID_PS2))
		SystemFlag2Temp |= SYSTEM_FLAG2_PS2;
	if(apiSignalFeedbackGetStatus(APP_SIGNAL_ID_PS3))
		SystemFlag2Temp |= SYSTEM_FLAG2_PS3;
	if(apiSignalFeedbackGetStatus(APP_SIGNAL_ID_K1))
		SystemFlag2Temp |= SYSTEM_FLAG2_K1;
	if(apiSignalFeedbackGetStatus(APP_SIGNAL_ID_K2))
		SystemFlag2Temp |= SYSTEM_FLAG2_K2;
	if(apiSignalFeedbackGetStatus(APP_SIGNAL_ID_K3))
		SystemFlag2Temp |= SYSTEM_FLAG2_K3;
	if(apiSignalFeedbackGetStatus(APP_SIGNAL_ID_K4))
		SystemFlag2Temp |= SYSTEM_FLAG2_K4;
	if(apiSignalFeedbackGetStatus(APP_SIGNAL_ID_DOCP))
		SystemFlag1Temp |= SYSTEM_FLAG1_DOCP_LATCH;
	if(apiSignalFeedbackGetStatus(APP_SIGNAL_ID_COCP))
		SystemFlag1Temp |= SYSTEM_FLAG1_COCP_LATCH;
	if(apiSignalFeedbackGetStatus(APP_SIGNAL_ID_OD_IN))
		SystemFlag2Temp |= SYSTEM_FLAG2_OD_IN;
	if(appProjectIsInEngMode())
		SystemFlag2Temp |= SYSTEM_FLAG2_ENG_MODE;
	if(halAfeGetState() != AFE_STATE_NORMAL)
		SystemFlag2Temp |= SYSTEM_FLAG2_AFE_INI_STATE;
	
	systemFlagNextFunction();
}

static void checkCOCP_DOCP_Flag(void)
{
	uint8_t	flag;
	
	flag = apiProtectCocpGetFlag();
	if( (flag & PROTECT_FLAG_L1_MASK) >= PROTECT_FLAG_L1_SETTED)
	{
		SystemFlag1Temp |= SYSTEM_FLAG1_COCP_L1;
	}

	if( (flag & PROTECT_FLAG_L2_MASK) >= PROTECT_FLAG_L2_SETTED)
	{
		SystemFlag1Temp |= SYSTEM_FLAG1_COCP_L2;
	}

	if( (flag & PROTECT_FLAG_L3_MASK) >= PROTECT_FLAG_L3_SETTED)
	{
		SystemFlag1Temp |= SYSTEM_FLAG1_COCP_L3;
	}
			
	flag = apiProtectDocpGetFlag();
	if( (flag & PROTECT_FLAG_L1_MASK) >= PROTECT_FLAG_L1_SETTED)
	{
		SystemFlag1Temp |= SYSTEM_FLAG1_DOCP_L1;
	}

	if( (flag & PROTECT_FLAG_L2_MASK) >= PROTECT_FLAG_L2_SETTED)
	{
		SystemFlag1Temp |= SYSTEM_FLAG1_DOCP_L2;
	}

	if( (flag & PROTECT_FLAG_L3_MASK) >= PROTECT_FLAG_L3_SETTED)
	{
		SystemFlag1Temp |= SYSTEM_FLAG1_DOCP_L3;
	}
	systemFlagNextFunction();
}

static void checkDUTP_L3(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagNtcNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectDutpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L3_MASK) >= PROTECT_FLAG_L3_SETTED)
		{
			SystemFlag1Temp |= SYSTEM_FLAG1_DUTP_L3;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagNtcNumber())
	{
		systemFlagNextFunction();
	}
}


static void checkDUTP_L2(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagNtcNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectDutpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L2_MASK) >= PROTECT_FLAG_L2_SETTED)
		{
			SystemFlag1Temp |= SYSTEM_FLAG1_DUTP_L2;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagNtcNumber())
	{
		systemFlagNextFunction();
	}
}


static void checkDUTP_L1(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagNtcNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectDutpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L1_MASK) >= PROTECT_FLAG_L1_SETTED)
		{
			SystemFlag1Temp |= SYSTEM_FLAG1_DUTP_L1;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagNtcNumber())
	{
		systemFlagNextFunction();
	}
}

static void checkDOTP_L3(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagNtcNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectDotpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L3_MASK) >= PROTECT_FLAG_L3_SETTED)
		{
			SystemFlag1Temp |= SYSTEM_FLAG1_DOTP_L3;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagNtcNumber())
	{
		systemFlagNextFunction();
	}
}


static void checkDOTP_L2(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagNtcNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectDotpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L2_MASK) >= PROTECT_FLAG_L2_SETTED)
		{
			SystemFlag1Temp |= SYSTEM_FLAG1_DOTP_L2;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagNtcNumber())
	{
		systemFlagNextFunction();
	}
}



static void checkDOTP_L1(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagNtcNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectDotpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L1_MASK) >= PROTECT_FLAG_L1_SETTED)
		{
			SystemFlag1Temp |= SYSTEM_FLAG1_DOTP_L1;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagNtcNumber())
	{
		systemFlagNextFunction();
	}
}



static void checkCUTP_L3(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagNtcNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectCutpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L3_MASK) >= PROTECT_FLAG_L3_SETTED)
		{
			SystemFlag1Temp |= SYSTEM_FLAG1_CUTP_L3;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagNtcNumber())
	{
		systemFlagNextFunction();
	}
}

static void checkCUTP_L2(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagNtcNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectCutpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L2_MASK) >= PROTECT_FLAG_L2_SETTED)
		{
			SystemFlag1Temp |= SYSTEM_FLAG1_CUTP_L2;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagNtcNumber())
	{
		systemFlagNextFunction();
	}
}

static void checkCUTP_L1(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagNtcNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectCutpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L1_MASK) >= PROTECT_FLAG_L1_SETTED)
		{
			SystemFlag1Temp |= SYSTEM_FLAG1_CUTP_L1;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagNtcNumber())
	{
		systemFlagNextFunction();
	}
}

static void checkCOTP_L3(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagNtcNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectCotpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L3_MASK) >= PROTECT_FLAG_L3_SETTED)
		{
			
			SystemFlag1Temp |= SYSTEM_FLAG1_COTP_L3;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagNtcNumber())
	{
		systemFlagNextFunction();
	}
}


static void checkCOTP_L2(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagNtcNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectCotpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L2_MASK) >= PROTECT_FLAG_L2_SETTED)
		{
			SystemFlag1Temp |= SYSTEM_FLAG1_COTP_L2;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagNtcNumber())
	{
		systemFlagNextFunction();
	}
}


static void checkCOTP_L1(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagNtcNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectCotpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L1_MASK) >= PROTECT_FLAG_L1_SETTED)
		{
			//appSystemFlagDebugMsg("COTP L1");
			SystemFlag1Temp |= SYSTEM_FLAG1_COTP_L1;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagNtcNumber())
	{
		systemFlagNextFunction();
	}
}


static void checkUVP_L3(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagCellNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectUvpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L3_MASK) >= PROTECT_FLAG_L3_SETTED)
		{
			SystemFlag1Temp |= SYSTEM_FLAG1_UVP_L3;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagCellNumber())
	{
		systemFlagNextFunction();
	}
}
static void checkUVP_L2(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagCellNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectUvpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L2_MASK) >= PROTECT_FLAG_L2_SETTED)
		{
			SystemFlag1Temp |= SYSTEM_FLAG1_UVP_L2;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagCellNumber())
	{
		systemFlagNextFunction();
	}
}

static void checkUVP_L1(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagCellNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectUvpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L1_MASK) >= PROTECT_FLAG_L1_SETTED)
		{
			SystemFlag1Temp |= SYSTEM_FLAG1_UVP_L1;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagCellNumber())
	{
		systemFlagNextFunction();
	}
}

static void checkOVP_L3(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagCellNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectOvpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L3_MASK) >= PROTECT_FLAG_L3_SETTED)
		{
			SystemFlag1Temp |= SYSTEM_FLAG1_OVP_L3;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagCellNumber())
	{
		systemFlagNextFunction();
	}
}

static void checkOVP_L2(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagCellNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectOvpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L2_MASK) >= PROTECT_FLAG_L2_SETTED)
		{
			SystemFlag1Temp |= SYSTEM_FLAG1_OVP_L2;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagCellNumber())
	{
		systemFlagNextFunction();
	}
}

static void checkOVP_L1(void)
{
	uint8_t	flag;
	
	for(; SystemFlagSubIndex < sysFlagCellNumber(); SystemFlagSubIndex++)
	{
		flag = apiProtectOvpGetFlag(SystemFlagSubIndex);
		if( (flag & PROTECT_FLAG_L1_MASK) >= PROTECT_FLAG_L1_SETTED)
		{
			SystemFlag1Temp |= SYSTEM_FLAG1_OVP_L1;
			systemFlagNextFunction();
			return;
		}
	}
	if(SystemFlagSubIndex >= sysFlagCellNumber())
	{
		systemFlagNextFunction();
	}
}

static void checkScuOt(void)
{
	uint8_t	flag;
	
	flag = apiProtectScuOtFlag(0);
	if( (flag & PROTECT_FLAG_L1_MASK) >= PROTECT_FLAG_L1_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_RLY1_OT_L1;
	if( (flag & PROTECT_FLAG_L2_MASK) >= PROTECT_FLAG_L2_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_RLY1_OT_L2;
	if( (flag & PROTECT_FLAG_L3_MASK) >= PROTECT_FLAG_L3_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_RLY1_OT_L3;
		
	flag = apiProtectScuOtFlag(1);
	if( (flag & PROTECT_FLAG_L1_MASK) >= PROTECT_FLAG_L1_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_RLY2_OT_L1;
	if( (flag & PROTECT_FLAG_L2_MASK) >= PROTECT_FLAG_L2_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_RLY2_OT_L2;
	if( (flag & PROTECT_FLAG_L3_MASK) >= PROTECT_FLAG_L3_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_RLY2_OT_L3;

	flag = apiProtectScuOtFlag(2);
	if( (flag & PROTECT_FLAG_L1_MASK) >= PROTECT_FLAG_L1_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_AMBI_OT_L1;
	if( (flag & PROTECT_FLAG_L2_MASK) >= PROTECT_FLAG_L2_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_AMBI_OT_L2;
	if( (flag & PROTECT_FLAG_L3_MASK) >= PROTECT_FLAG_L3_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_AMBI_OT_L3;

	flag = apiProtectScuOtFlag(3);
	if( (flag & PROTECT_FLAG_L1_MASK) >= PROTECT_FLAG_L1_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_BUSBARP_OT_L1;
	if( (flag & PROTECT_FLAG_L2_MASK) >= PROTECT_FLAG_L2_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_BUSBARP_OT_L2;
	if( (flag & PROTECT_FLAG_L3_MASK) >= PROTECT_FLAG_L3_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_BUSBARP_OT_L3;

	flag = apiProtectScuOtFlag(4);
	if( (flag & PROTECT_FLAG_L1_MASK) >= PROTECT_FLAG_L1_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_BUSBARN_OT_L1;
	if( (flag & PROTECT_FLAG_L2_MASK) >= PROTECT_FLAG_L2_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_BUSBARN_OT_L2;
	if( (flag & PROTECT_FLAG_L3_MASK) >= PROTECT_FLAG_L3_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_BUSBARN_OT_L3;
		
	systemFlagNextFunction();
}

static void checkDvp(void)
{
	uint8_t	flag;
	
	flag = apiProtectDvpGetFlag();
	if( (flag & PROTECT_FLAG_L1_MASK) >= PROTECT_FLAG_L1_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_DVP_L1;
	if( (flag & PROTECT_FLAG_L2_MASK) >= PROTECT_FLAG_L2_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_DVP_L2;
	if( (flag & PROTECT_FLAG_L3_MASK) >= PROTECT_FLAG_L3_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_DVP_L3;
		
	systemFlagNextFunction();
}


static void checkDtp(void)
{
	uint8_t	flag;
	
	flag = apiProtectDtpGetFlag();
	if( (flag & PROTECT_FLAG_L1_MASK) >= PROTECT_FLAG_L1_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_DTP_L1;
	if( (flag & PROTECT_FLAG_L2_MASK) >= PROTECT_FLAG_L2_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_DTP_L2;
	if( (flag & PROTECT_FLAG_L3_MASK) >= PROTECT_FLAG_L3_SETTED)
		SystemFlag3Temp |= SYSTEM_FLAG3_DTP_L3;
		
	systemFlagNextFunction();
}


static void systemFlagIni(void)
{
	SystemFlagFunIndex = 1;
	SystemFlagSubIndex = 0;
	SystemFlag1Temp = 0;
	SystemFlag2Temp = 0;
	SystemFlag3Temp = 0;
	SystemFlag4Temp = 0;
}

const tSystemFlagCheckFunTable mSystemFlagCheckFunTable[]={
		checkFlagNone,
		checkOVP_L1,
		checkOVP_L2,
		checkOVP_L3,
		checkUVP_L1,
		checkUVP_L2,
		checkUVP_L3,
		checkCOTP_L1,
		checkCOTP_L2,
		checkCOTP_L3,
		checkCUTP_L1,
		checkCUTP_L2,
		checkCUTP_L3,
		checkDOTP_L1,
		checkDOTP_L2,
		checkDOTP_L3,
		checkDUTP_L1,
		checkDUTP_L2,
		checkDUTP_L3,
		checkCOCP_DOCP_Flag,
		checkFbSignal,
		checkOtherFlag,
		checkMinMaxCellVoltage,
		checkMinMaxNtcTempVoltage,
		checkScuOt,
		checkDvp,
		checkDtp,
		
		systemFlagCheckFinish
};
	
static void systemFlagSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
		if(SystemFlagFunIndex)
			mSystemFlagCheckFunTable[SystemFlagFunIndex]();
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		if(!SystemFlagFunIndex)
		{
			systemFlagIni();
		}
	}
}

/* Public function prototypes -----------------------------------------------*/
uint32_t apiSystemFlagGetFlag1(void)
{
	return SystemFlag1;
}
uint32_t apiSystemFlagGetFlag2(void)
{
	return SystemFlag2;
}
uint32_t apiSystemFlagGetFlag3(void)
{
	return SystemFlag3;
}
uint32_t apiSystemFlagGetFlag4(void)
{
	return SystemFlag4;
}

void apiSystemFlagOpen(void)
{
	LibSwTimerOpen(systemFlagSwTimerHandler, 0);
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


/**
  ******************************************************************************
  * @file        LibProtect.c
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/14
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "battery.h"
#include "readadci.h"
#include "KgFaultType.h"
//#include "fault.h"
#include "TimeShare.h"
#include "uart.h"
#include "gpio.h"
#include "uvpcheck.h"
#include "ovpcheck.h"
#include "cotpcheck.h"
#include "cutpcheck.h"
#include "dotpcheck.h"
#include "dtpcheck.h"
#include "cocpcheck.h"
#include "docpcheck.h"
#include "soccheck.h"
#include "dvpcheck.h"
#include "ovppfcheck.h"
#include "mosotcheck.h"
#include "eeprom.h"

#include "sdk_config.h"
#include "LibSwTimer.h"
#include "LibRegister.h"
#include "LibFaultLog.h"
#include "LibProtect.h"
#include "ebikebmuproxy.h"

static tLibRegister  evtHandlerTab;

#if 0
/* Includes ------------------------------------------------------------------*/
#include "datatype.h"
#include "fault.h"
#include "LibProtectDef.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct {
  tLibRegister  evtHandlerTab;
} tLibProtect;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
tLibProtect mProtect = {0};
/* Private function prototypes -----------------------------------------------*/

static void LibProtectEvtDispatchHandler(uint16_t evt, void *data){
  RegisterTypeHandlerExe(&mProtect.evtHandlerTab, evt, data);
}


void LibProtectOpen(tLibProtectEvtHandler evtHandler){
  if(RegisterIsMemberNull(&mProtect.evtHandlerTab) == true){
#if (LIB_PROTECT_OV_ENABLE == 1)
     LibProtectOvOpen(LibProtectEvtDispatchHandler);
#endif
  }
  RegisterAdd(&mProtect.evtHandlerTab, evtHandler);
}

void LibProtectClose(tLibProtectEvtHandler evtHandler){
  RegisterRm(&mProtect.evtHandlerTab, evtHandler);
  if(RegisterIsMemberNull(&mProtect.evtHandlerTab) == true){
    
  }  
}

#endif

__IO	BYTE	CheckProtectFunIndex;


//----------------------------------------
//----------------------------------------
//	保護點檢查
#ifndef DISABLE_OVP_FUN	
BYTE CheckFun_OVP_L1_0(void)
{
	OvpLxCheck();
	return 1;
}
#endif

//-------------------------------------
BYTE CheckFun_UVP_L1_0(void)
{
	UvpLxCheck();
	return 1;
}

//-----------------------------------------
BYTE CheckFun_DVP_OVPPF(void)
{
#ifndef DISABLE_DVP_FUN
	DvpCheck();
#endif		
#ifndef DISABLE_OVPPF_UVPPF_FUN
	OvpUvpPfCheck(); 
#endif
	return 1;
}
//------------------------------------------------
	
BYTE CheckFun_COTP_L1(void)
{
	CotpLxCheck();
	return 1;
}
		
//------------------------------------------------
			
BYTE CheckFun_DOTP_L1(void)
{
	DotpLxCheck();	
	return 1;
}

//------------------------------------------------
BYTE CheckFun_CUTP_L1(void)
{
	CutpLxCheck();
	return 1;
}

//------------------------------------------------
BYTE CheckFun_COCP_L1(void)
{
	CocpLxCheck(0);	//COCP CHECK level 0
	CocpLxCheck(1);	//COCP CHECK level 1

	return 1;
}

//------------------------------------------------
BYTE CheckFun_DOCP_L1(void)
{
	DocpLxCheck(0);	//COCP CHECK level 0
	DocpLxCheck(1);	//COCP CHECK level 1

	return 1;
}


//------------------------------------------------
#ifndef DISABLE_DTP_FUN	
BYTE CheckFun_DTP_L1(void)
{
	DtpLxCheck(0);
	return 0;
}
BYTE CheckFun_DTP_L2(void)
{
	DtpLxCheck(1);
	return 0;
}
#endif
	//--------------------------------
BYTE CheckFun_MOSOT(void)
{
//	if(CanCheckTemp200msEvent())
		MosotLxCheck();
	return 0;
}
//------------------------------------------------
BYTE CheckFun_SystemFlag_1(void)
{
	CheckSystemFlag(0);		//每100ms檢查一次系統Flag
	return 1;
}
BYTE CheckFun_SystemFlag_2(void)
{
	CheckSystemFlag(1);		//每100ms檢查一次系統Flag
	return 1;
}
BYTE CheckFun_SystemFlag_3(void)
{
	CheckSystemFlag(2);		//每100ms檢查一次系統Flag
	return 1;
}
BYTE CheckFun_SystemFlag_4(void)
{
	CheckSystemFlag(3);		//每100ms檢查一次系統Flag
	return 1;
}

BYTE CheckMosOnOffTimer=0;
static BYTE	CocpL4RCount=0;
static BYTE	DocpL4RCount=0;
static BYTE	ShortCircuitRCount=0;

BYTE CheckFun_MosControl(void)	//95ms ~ 100ms 
{
	char	str[100];
	WORD	Current_mA;
	
	if(InEngineerModeFlag)
		return 0;
	//---------------------------------------
	//	charge mode change !!
	if(ChargeModeChangeFlagForMOS_Ctrl)
	{
		ChargeModeChangeFlagForMOS_Ctrl=FALSE;
		CocpL4RCount=0;
		DocpL4RCount=0;
		ShortCircuitRCount=0;
	}
	if(SystemReadyFlag)
	{
		if((SystemParameter.COCPFlag&L4_FLAG_MASK)>=L4_FLAG_SETTED && 
			ChargeMode==DISCHARGE_MODE)
		{
			CocpL4RCount++;
			if(CocpL4RCount>=20)	//j++ 20191007 over 10 sec
			{
				SaveFaultData(FAULT_TYPE_COCP_L4_RELEASE,0);
				SystemParameter.COCPFlag&=~L4_FLAG_MASK;	//discharge, release COCP L4	
				SendUartMessage("Release COCP L4");
				CocpL4RCount=0;
			}
		}
		else if((SystemParameter.DOCPFlag&L4_FLAG_MASK)>=L4_FLAG_SETTED &&
			     ChargeMode==CHARGE_MODE)
		{
			Current_mA=SystemParameter.Current_uA.l/1000L;			
			if((Current_mA>=1800 &&		//release condition I<2A+-10%
	//	   SystemParameter.AbsCurrentValue.l<=2200) ||
	//	   (SystemParameter.AbsCurrentValue.l>=3600 &&		//release condition I<4A+-10%
			   Current_mA<=4400))
			{
				DocpL4RCount++;
				if(DocpL4RCount>=20)					//j++ 20191007 over 10 sec
				{
					SaveFaultData(FAULT_TYPE_DOCP_L4_RELEASE,0);
					SystemParameter.DOCPFlag&=~L4_FLAG_MASK;	//charge, release DOCP L4
					SendUartMessage("Release DOCP L4");
					DocpL4RCount=0;
				}
			}
			else
				DocpL4RCount=0;
		}
		else if((SystemParameter.SystemFlag.l&SYSTEMFLAG_SHORT_CIRCUIT_FLAG_SET) &&
				ChargeMode==CHARGE_MODE)
		{
			Current_mA=SystemParameter.Current_uA.l/1000L;	
			if((Current_mA>=1800 &&		//release condition I<2A+-10%
//			    SystemParameter.AbsCurrentValue.l<=2200) ||
//			   (SystemParameter.AbsCurrentValue.l>=3600 &&		//release condition I<2A+-10%
			    Current_mA<=4400))
			{
				ShortCircuitRCount++;
				if(ShortCircuitRCount>=20)		//j++ 20191007 over 10 sec
				{
			//		AFE_WR(AFE_OCDRSTRT,0x01);		//release short circuit status
					SendUartMessage("Release Short Circuit ");
					ShortCircuitRCount=0;
				}
			}
		}
		else
		{
			CocpL4RCount=0;
			DocpL4RCount=0;
			ShortCircuitRCount=0;
		}
	}
	
//	CheckMosOnOffTimer++;
//	if(CheckMosOnOffTimer>=10)	//over 1sec check relay
	{
		CheckMosOnOffTimer=0;
		if(SystemReadyFlag)
		{
			//CheckSystemFlag();		//check last flag !!!
			//------------------------
			//	chk system flag 
			//	cotp L4 mos off
			//	check mos-->Mos on
			//	chk system flag -->set mos off flag
			//	check mos-->Mos off
			//------------------------------------
			//	Check Charge Mos event
			SystemParameter.IsMosOnFlag=0;
			if(((SystemParameter.SystemFlag.l&
					(
						SYSTEMFLAG_OVP_MOSOFF_FLAG_SET+
				 		//SYSTEMFLAG_UVP_MOSOFF_FLAG_SET+
				 		SYSTEMFLAG_COTP_MOSOFF_FLAG_SET+
				 		SYSTEMFLAG_CUTP_MOSOFF_FLAG_SET+
				 		//SYSTEMFLAG_DOTP_MOSOFF_FLAG_SET+
				 		//SYSTEMFLAG_DOCP_MOSOFF_FLAG_SET+
				 		SYSTEMFLAG_COCP_MOSOFF_FLAG_SET+
				 		SYSTEMFLAG_DVP_MOSOFF_FLAG_SET+                                                                                                                                                                        				 		
				 		SYSTEMFLAG_AFE_MOSOFF_COMM_FAIL+
				 		SYSTEMFLAG_MOS_PF+
				 		SYSTEMFLAG_OVPPF+
				 		SYSTEMFLAG_UVPPF+
				 		SYSTEMFLAG_MOSOT_MOSOFF_FLAG_SET+
				 		//SYSTEMFLAG_C_MOS_OFF+
						//SYSTEMFLAG_D_MOS_OFF+
				 		SYSTEMFLAG_SYSTEM_READY))==
					(SYSTEMFLAG_SYSTEM_READY)))
					//(SYSTEMFLAG_C_MOS_OFF+SYSTEMFLAG_D_MOS_OFF+SYSTEMFLAG_SYSTEM_READY)))
			{
				SystemParameter.IsMosOnFlag|=CHG_FET;
			}
			
			//------------------------------------
			//	Check DisCharge Mos event
			if(((SystemParameter.SystemFlag.l&
					(
						//SYSTEMFLAG_OVP_MOSOFF_FLAG_SET+
				 		SYSTEMFLAG_UVP_MOSOFF_FLAG_SET+
				 		//SYSTEMFLAG_COTP_MOSOFF_FLAG_SET+
				 		//SYSTEMFLAG_CUTP_MOSOFF_FLAG_SET+
				 		SYSTEMFLAG_DOTP_MOSOFF_FLAG_SET+
				 		SYSTEMFLAG_DOCP_MOSOFF_FLAG_SET+
				 		//SYSTEMFLAG_COCP_MOSOFF_FLAG_SET+
				 		SYSTEMFLAG_DVP_MOSOFF_FLAG_SET+                                                                                                                                                                        				 		
				 		SYSTEMFLAG_AFE_MOSOFF_COMM_FAIL+
				 		SYSTEMFLAG_MOS_PF+
				 		SYSTEMFLAG_OVPPF+
				 		SYSTEMFLAG_UVPPF+
				 		SYSTEMFLAG_MOSOT_MOSOFF_FLAG_SET+
				 		SYSTEMFLAG_SHORT_CIRCUIT_FLAG_SET+
				 		//SYSTEMFLAG_C_MOS_OFF+
						//SYSTEMFLAG_D_MOS_OFF+
				 		SYSTEMFLAG_SYSTEM_READY))==
					(SYSTEMFLAG_SYSTEM_READY)))
					//(SYSTEMFLAG_C_MOS_OFF+SYSTEMFLAG_D_MOS_OFF+SYSTEMFLAG_SYSTEM_READY)))
			{
				SystemParameter.IsMosOnFlag|=DSG_FET;
			}
			//-------------------------------------
			//	when mos off 
			//	charge mos off,when discharge,must be turn on charge mos 
			if(!(SystemParameter.IsMosOnFlag&CHG_FET))	//charge mos off
			{
				if(((SystemParameter.SystemFlag.l&(
					SYSTEMFLAG_DOTP_MOSOFF_FLAG_SET+
					SYSTEMFLAG_DOCP_MOSOFF_FLAG_SET+
					SYSTEMFLAG_MOS_PF+
				 	SYSTEMFLAG_OVPPF+
				 	SYSTEMFLAG_UVPPF+
				 	SYSTEMFLAG_MOSOT_MOSOFF_FLAG_SET+
				 	SYSTEMFLAG_SHORT_CIRCUIT_FLAG_SET+
				 	SYSTEMFLAG_AFE_MOSOFF_COMM_FAIL+	
					SYSTEMFLAG_DVP_MOSOFF_FLAG_SET+                
			 		SYSTEMFLAG_SYSTEM_READY))==
					(SYSTEMFLAG_SYSTEM_READY)) &&  
					ChargeMode==DISCHARGE_MODE)
				{
					SystemParameter.IsMosOnFlag|=CHG_FET;
				}
			}
			if(!(SystemParameter.IsMosOnFlag&DSG_FET) )	//discharge mos off
			{
				if(((SystemParameter.SystemFlag.l&(
					//SYSTEMFLAG_DOTP_MOSOFF_FLAG_SET+
					SYSTEMFLAG_COCP_MOSOFF_FLAG_SET+
					SYSTEMFLAG_MOS_PF+
				 	SYSTEMFLAG_OVPPF+
				 	SYSTEMFLAG_UVPPF+
				 	SYSTEMFLAG_MOSOT_MOSOFF_FLAG_SET+
				 	SYSTEMFLAG_SHORT_CIRCUIT_FLAG_SET+
				 	SYSTEMFLAG_AFE_MOSOFF_COMM_FAIL+	
					SYSTEMFLAG_DVP_MOSOFF_FLAG_SET+                
			 		SYSTEMFLAG_SYSTEM_READY))==
					(SYSTEMFLAG_SYSTEM_READY)) && 
					ChargeMode==CHARGE_MODE)
				{
					SystemParameter.IsMosOnFlag|=DSG_FET;
				}
			}
			//-------------------------------------
			TurnOnMos();
		}			
		sprintf(str,"Mos=%.2X %.2X %.2X %.2X",
					SystemParameter.IsMosOnFlagTemp,
					SystemParameter.IsMosOnFlag,
					BatteryCapInfo.PF_Flag,
					SystemParameter.MosFailOvpUvpPFFlag
					);		
	}

	return 0;
}

//------------------------------------------------
//------------------------------------------------
	
#ifndef DISABLE_SOC_FUN
BYTE CheckFun_Soc(void)
{
	SocCheck();
	return 0;
}
#endif
//=========================================================

//=========================================================
BYTE CheckFun_End(void)
{
	CheckProtectFunIndex=0xfe;
	return 1;
}


BYTE (*CheckProtectFunTable[])(void) = {
	//------------------------------------
	CheckFun_OVP_L1_0,	
//--------------------------------------	
	CheckFun_UVP_L1_0,	
//--------------------------------------	12
	CheckFun_DVP_OVPPF,		//200ms
	CheckFun_COTP_L1,
//--------------------------------------	17		
	CheckFun_DOTP_L1,
//--------------------------------------	
	CheckFun_CUTP_L1,
//--------------------------------------	
	CheckFun_COCP_L1,
//--------------------------------------	
	CheckFun_DOCP_L1,
//--------------------------------------	24
#ifndef DISABLE_DTP_FUN		//200ms
	CheckFun_DTP_L1,	CheckFun_DTP_L2,
#endif
//--------------------------------------	
	CheckFun_MOSOT,

//--------------------------------------	28
	CheckFun_SystemFlag_1,CheckFun_SystemFlag_2,
	CheckFun_SystemFlag_3,CheckFun_SystemFlag_4,
	CheckFun_MosControl,
//--------------------------------------		
#ifndef 	DISABLE_SOC_FUN
	CheckFun_Soc,
#endif	
//---------------------------------
CheckFun_End
};
//----------------------------------------------------------------------------
void RunCheckProtectFun(void)
{
	BYTE	result,i;	
	for(i=0;i<40;i++)
	{
		result=(*CheckProtectFunTable[CheckProtectFunIndex])();
		CheckProtectFunIndex++;
		if(result)
			break;
		if(CheckProtectFunIndex>=0xf0)
			break;
	}
}

void LibProtectCheckUVBAT(void){
	if((SystemParameter.VBAT.l / 100) < EEPromParameter.ShutdownVol.i){
		if(SystemParameter.UnderVoltTimeCount < 60){
		    SystemParameter.UnderVoltTimeCount++ ;	
			if(SystemParameter.UnderVoltTimeCount == 60){
			    SaveFaultData(EVENT_LOW_POWER_OFF,65535);
				LibRegisterTypeHandlerExe(&evtHandlerTab, LIB_PROTECT_EVT_LOW_VBAT, 0);
			}
		}
	}else{	
		SystemParameter.UnderVoltTimeCount=0 ;
	}		
}
	


void LibProtectTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr){
    if(evt == LIB_SW_TIMER_EVT_SW_1MS){
		if(CheckProtectFunIndex<=0xf0){
		    RunCheckProtectFun();
		}
    }else if(evt == LIB_SW_TIMER_EVT_SW_500MS){
		CheckProtectFunIndex=0;
    }else if(evt == LIB_SW_TIMER_EVT_SW_1S){
		LibProtectCheckUVBAT();
    }
}


void LibProtectOpen(tLibSwTimerEvtHandler handler){
    if(LibRegisterIsMemberNull(&evtHandlerTab) == true){
	  CheckProtectFunIndex=0xff;
      LibSwTimerOpen(LibProtectTimerHandler, 0);
    }
    LibRegisterAdd(&evtHandlerTab, handler, 0);
}

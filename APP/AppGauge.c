/**
  ******************************************************************************
  * @file        AppGauge.c
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
#include <stdio.h>
#include <stdlib.h>
#include "define.h"
#include "main.h"
#include "halafe.h"
#include "ApiSysPar.h"
#include "AppGauge.h"
#include "LibSwTimer.h"
#include "LibHwTimer.h"
#include "HalRtc.h"
#include "ApiRamData.h"
#include "ApiSysPar.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	scuDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t			GaugeTickMs;
	uint8_t			ChargeMode;
	uint8_t			ChargeModeTemp;
	WORD			ReadSocTime;		//欲更新Soc 的時間
	WORD			RelaxSec;			//電池在Relax 的總時間
	WORD			NowSoc;				//根據Cell 電壓讀到的Soc值
	WORD			SocA_Voltage;		//Relax 30 min 之後第一次取樣的電壓值
	WORD			SocB_Voltage;		//Relax 30 min 之後第二次取樣的電壓值
	WORD			Soc0;
	WORD			StarCalQmaxSoc;
	WORD			StartCalRaSoc;
	WORD			RamSoc;				//儲存在RTC Ram 的Soc值
	WORD			SocBase;			//計算 RamSoc 的基準值,當RamSoc更新時,此值設為RamSoc
	tIbyte			QmaxUpdateTimes;
	tIbyte			CycleCount;
	BYTE			PF_Flag;
	BYTE			SocReadNum;
	BYTE			QmaxStatus;
	BYTE			NowSocValid;
	BYTE			Soc0Valid;
	BYTE			RaSocValid;
	BYTE			UpdateStstus;
	WORD			RaLastSoc;
	WORD			CutOffSoc;			//截止電壓點上的SOC
	tLbyte			Qstart;
	BYTE			RmUpdateLastLevel;
	tLbyte			FCC;	//unit mAh 4 
	tLbyte			Qmax;
	tLLbyte			QmaxuAms;
	tLLbyte			TotalDisChargeCount;
	tLLbyte			QmaxPassCharge;
	tLLbyte			RaPassCharge;
	tLLbyte			RM;				//unit ma,ms	8
	tLbyte			MaxInputPower;		//根據實際情況計算出的Power(考慮溫度的影響)	
	tLbyte			MaxOutputPower;
	tLbyte			CalMaxInputPower;	//理論上輸的Power
	tLbyte			CalMaxOutputPower;
	WORD			Table_RaValue;	//根據查表得到的Ra電阻值
	WORD			Cal_RaValue;	//根據實際情況計算出的Ra電阻值
	BYTE			TerminateVoltageCount;	//連續低於截止電壓的計時器
	BYTE			FullChargeTimerCount;	//滿充計時器
	BYTE			CalRaIdleTimer;			//在RA 有發生校正時，讓輸出的SOC 顯示為199.99 做為Debug 用
	BYTE			Over5HrStatus;
	tLLbyte			AccChgPower;		//j++ 20190219 累計充電電量
	tLLbyte			AccDhgPower;		//j++ 20190219 累計放電電量	
	uint8_t			OcvTableNum;
	uint8_t			RaTableNum;
	tOcvRaTable		OcvTable[25];
	tOcvRaTable		RaTable[25];
	
	tIbyte			DisplaySocValue;
	tIbyte			RsocValue;
		
	uint16_t		TerminateVoltage;
	
	struct{
		uint16_t	Voltage;
		uint16_t	Current;
		uint8_t		Second;
	}FullCharge;
	
	WORD			LastRsocValue;
	DWORD			AbsAvggCurrent;
	uint8_t			GaugeState;
	uint8_t			FirstReadCurrentFlag;
	
	tAppGaugeEvtHandler	EvtHandler;
}tAppScuGauage;


/* Private define ------------------------------------------------------------*/
#define	gaugeCellNumber()		apiSysParGetCellNumber()
/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//static	
tAppScuGauage	mAppScuGauage ={0};

/* Private function prototypes -----------------------------------------------*/
//uint8_t	appGaugeGetCurrentMode(void);

void setupOcvTable(void)
{
	uint8_t	i;
	char	str[100];
	
	mAppScuGauage.OcvTableNum = 0;
	for(i=0; i<25; i++)
	{
		apiSysParGetOcvTable(i, &mAppScuGauage.OcvTable[i]);
//		sprintf(str,"OCV %d %d",
//					mAppScuGauage.OcvTable[i].Level,
//					mAppScuGauage.OcvTable[i].Value);
//		scuDebugMsg(str);
		
		if(mAppScuGauage.OcvTable[i].Level >= 100)
		{
			mAppScuGauage.OcvTableNum = i + 1;
			break;
		}
	}
}
static void setupRaTable(void)
{
	uint8_t	i;
	char	str[100];
	mAppScuGauage.RaTableNum = 0;
	
	for(i=0; i<25; i++)
	{
		apiSysParGetRaTable(i, &mAppScuGauage.RaTable[i]);
//		sprintf(str,"RA %d %d",
//					mAppScuGauage.RaTable[i].Level,
//					mAppScuGauage.RaTable[i].Value);
//		scuDebugMsg(str);
		
		if(mAppScuGauage.RaTable[i].Level >= 100)
		{
			mAppScuGauage.RaTableNum = i + 1;
			break;
		}
	}
}

static void chargeModeIni(void)
{
	mAppScuGauage.ChargeMode = APP_SCU_GAUGE_RELEASE_MODE;
	mAppScuGauage.ChargeModeTemp = APP_SCU_GAUGE_UNKNOW_MODE;
}

static void gaugeQPassChargeIni(void)
{
	mAppScuGauage.QmaxPassCharge.sll = 0;
}

static void gaugeRPassChargeIni(void)
{
	mAppScuGauage.RaPassCharge.sll = 0;
}

DWORD CvtQmaxPassChargeTomAh(void)
{
	tLLbyte		LLbyte;
	LLbyte.sll = mAppScuGauage.QmaxPassCharge.sll;
	LLbyte.sll /= (DLONG)(3600u * 1000u * 1000u);    //uA , ms-->sec-->h
	return	LLbyte.l[0];
}
DWORD CvtRaPassChargeTomAh(void)
{
	tLLbyte		LLbyte;
	LLbyte.sll = mAppScuGauage.RaPassCharge.ll;
	LLbyte.sll /= (DLONG)(3600u * 1000u * 1000u);    //uA ms-->sec-->h
	return	LLbyte.l[0];
}

DWORD appScuGaugeCvtRmTomAh(void)
{
	tLLbyte		LLbyte;
	LLbyte.ll = mAppScuGauage.RM.ll;
	LLbyte.ll /= (DDWORD)(3600u * 1000u * 1000u);    //uA->mA ms-->sec-->h
	return	LLbyte.l[0];
}

//------------------------------------
static void gaugeCalSoc(void)
{
	DWORD	n1;
	n1 = appScuGaugeCvtRmTomAh();//...RM_mAh.l;
	n1 *= 10000L;
	n1 /= mAppScuGauage.FCC.l;
	if(n1 > 10000)
		n1 = 10000;
	mAppScuGauage.RsocValue.i = n1;		//真正的Soc
}
uint16_t GetAverageCellVoltage(void)
{
	WORD	no,cell;
	uint32_t	TotalCellVoltage = 0;
	
	if(gaugeCellNumber() == 0)
		return 3500;
	
	for(cell=0; cell < gaugeCellNumber(); cell++)
	{
		TotalCellVoltage += halAfeGetCellVoltage(cell);
	}
	TotalCellVoltage /= gaugeCellNumber();
	
	return (uint16_t)TotalCellVoltage;
}

	
//------------------------------------
void appScuGaugeSetFcc(void)
{
	DWORD	n1;
	LONG	slong;
	n1 = appScuGaugeCvtRmTomAh();	
	
	mAppScuGauage.FCC.l = mAppScuGauage.Qstart.l + n1;
	slong = CvtRaPassChargeTomAh();
	if(slong < 0)
	{
		n1 = slong * (-1);
		mAppScuGauage.FCC.l += n1;
	}
	else
	{
		if(mAppScuGauage.FCC.l >= slong)
			mAppScuGauage.FCC.l -= slong;
	}
	gaugeCalSoc();
}

static void gaugeSetRmAndFcc(uint16_t soc)
{
	DDWORD	d,n;
	if(soc >= mAppScuGauage.CutOffSoc)
		d = (DDWORD)(soc - mAppScuGauage.CutOffSoc);
	else
		d = 0;
			
	n = (DDWORD)mAppScuGauage.Qmax.l;
	n *= d;
	n /= (DDWORD)10000;		//mAh
	n *= (DDWORD)1000L;		//uAhH	
	mAppScuGauage.RM.ll = (DDWORD)n*(DDWORD)3600*(DDWORD)1000;
	
	appScuGaugeSetFcc();
}

//------------------------------------
//	根據SOC 回傳OCV 電壓值
WORD GetOcvVoltage(WORD SOC)
{
	BYTE	i,soc;
    DWORD	v;
    DWORD   dV,dR;
    BYTE	negflag;
    
    if(mAppScuGauage.OcvTableNum<1)
    	return 0;
    soc = SOC / 100;
 	if(soc < mAppScuGauage.OcvTable[0].Level)
        return mAppScuGauage.OcvTable[0].Value;
	else if(soc >= mAppScuGauage.OcvTable[mAppScuGauage.OcvTableNum-1].Level)
        return mAppScuGauage.OcvTable[mAppScuGauage.OcvTableNum-1].Value;
	else
	{
		for(i=0; i<(mAppScuGauage.OcvTableNum-1); i++)
		{
			if(soc >= mAppScuGauage.OcvTable[i].Level &&
		   	   soc < mAppScuGauage.OcvTable[i+1].Level)
		   	{
		   		v = SOC-((WORD)mAppScuGauage.OcvTable[i].Level * 100);
		   		//----------------------------------
		   		//	j++ 20190618 新增判斷負電壓值!!!
		   		//		某些 OCV table 會有高SOC 比低SOC 電壓還低的情況!!!
		   		if(mAppScuGauage.OcvTable[i+1].Value >= mAppScuGauage.OcvTable[i].Value)
		   		{
			   		dV = mAppScuGauage.OcvTable[i+1].Value-
					   mAppScuGauage.OcvTable[i].Value;
					negflag = 0;
				}
				else
				{
					dV = mAppScuGauage.OcvTable[i].Value-
					   mAppScuGauage.OcvTable[i+1].Value;
					negflag=1;
				}
				//-----------------------------
		   		dR = mAppScuGauage.OcvTable[i+1].Level - mAppScuGauage.OcvTable[i].Level;
		   		dR *= 100L;
		   		v *= dV;
		   		v /= dR;
		   		if(negflag)
		   			v = mAppScuGauage.OcvTable[i].Value - v;
		   		else
		   			v = mAppScuGauage.OcvTable[i].Value + v;	
		 		break;
		   	}
		}
	}
    return v;
}
//------------------------------------
//	根據SOC 回傳Ra 值
//	解析度 0.01 mR
WORD GetRaFromSoc(WORD SOC)
{
	BYTE	i;
	DWORD	dR,dS,S;
	WORD	Ra;
	BYTE	plus_flag;
	BYTE	soc;
	
	Ra = mAppScuGauage.RaTable[0].Value;	//設定初始值
	
	soc = SOC / 100;
	if(mAppScuGauage.RaTableNum < 1)
		return 0;
	for(i=0; i<(mAppScuGauage.RaTableNum - 1); i++)
	{
		if(soc >= mAppScuGauage.RaTable[i].Level &&
		   soc < mAppScuGauage.RaTable[i+1].Level)
		{
			if(mAppScuGauage.RaTable[i].Value==
			   mAppScuGauage.RaTable[i+1].Value)
			{
				Ra = mAppScuGauage.RaTable[i].Value;
				break;	
			}
			
			if(mAppScuGauage.RaTable[i].Value>
			   mAppScuGauage.RaTable[i+1].Value)
			{
				dR = mAppScuGauage.RaTable[i].Value-
					mAppScuGauage.RaTable[i+1].Value;
				plus_flag = 0;
				S=((WORD)mAppScuGauage.RaTable[i+1].Level * 100) - SOC;
			}
			else
			{
				dR = mAppScuGauage.RaTable[i+1].Value-
					mAppScuGauage.RaTable[i].Value;
				plus_flag = 1;
				S = SOC-((WORD)mAppScuGauage.RaTable[i].Level * 100);
			}
			dS = mAppScuGauage.RaTable[i+1].Level - mAppScuGauage.RaTable[i].Level;
			S *= dR;
			S /= dS;
			S /= 100;
			if(plus_flag)
				S += mAppScuGauage.RaTable[i].Value;
			else
				S += mAppScuGauage.RaTable[i+1].Value;
			Ra = (WORD)S;
			break;
		}
	}
	return Ra;
}

//--------------------------------------------
//	j++ 20190709 新增計算電池內阻功能
//	根據 BatteryCapInfo[area].RamSoc 的值取得目前Ocv Table 電壓值
static void gaugeCalCellRValue(void)
{
	WORD		SocTabVoltage;
	WORD		RealVoltage;
	double		cur,newRa;
	char		str[100];
	
	//j-- if(SystemParameter.CalRTimer[area]<50)
//		return;
	
//j--	if(SystemParameter.CellRAvgCurrentTimer[area]<10)	//等待電流穩定 1 Sec 之後才可計算內阻	
//		return;
	
//j--	SystemParameter.CalRTimer[area]=0;
	
	SocTabVoltage = GetOcvVoltage(mAppScuGauage.RamSoc);
	RealVoltage = GetAverageCellVoltage();
	
	//sprintf(str,"Cal RValue =%d %d ",
	//				SocTabVoltage,
	//				RealVoltage);
	//scuDebugMsg(str);
	if(appGaugeGetCurrentMode() == APP_SCU_GAUGE_DISCHARGE_MODE)	//放電模式
	{
		//scuDebugMsg("Discharge ");
		if(SocTabVoltage > RealVoltage)
		{
			newRa = (double)(SocTabVoltage - RealVoltage);
		}
		else
		{
//			strcat(str," Last R1 ");
			//scuDebugMsg("放電條件錯誤,無法計算內阻");
			return;
			//goto _exit;
		}
	}	
	else if(appGaugeGetCurrentMode() == APP_SCU_GAUGE_CHARGE_MODE)
	{
		//-------------------------------
		//	實際電壓值		
		if(RealVoltage>SocTabVoltage)
		{
			newRa=(double)(RealVoltage-SocTabVoltage);
		}
		else
		{
	//		strcat(str," Last R2 ");
			//SendMessageToCan("充電條件錯誤,無法計算內阻");
			return;
			//goto _exit;
		}
	}
	else	//無充放電模式,無法計算內阻,改以Ra Table 為主?
	{
		//strcat(str," Last R3 ");
		return;
		//goto _exit;
	}
	//----------------------------------
	//	計算實際內阻
	;//j-- cur = (double)SystemParameter.CellRAvgCurrent[area].l;	//mA
//	sprintf(str,"RA1= %d",(DWORD) newRa);
//	scuDebugMsg(str);
	
	cur = abs(appGaugeGetCurrentValue());
//	sprintf(str,"curr= %d",(DWORD) cur);
//	scuDebugMsg(str);
	
	newRa *= 100000.0;	//0.01mR
	newRa /= cur;		
	if(newRa > 65535.0)
		newRa = 65535.0;
//	sprintf(str,"RA= %d",(DWORD) newRa);
//	scuDebugMsg(str);
	//j-- SystemParameter.CellRValue[area]=(WORD)newRa;
}


//===========================================================================
void appScuGaugeUpdateQmax(DWORD passcharge,BYTE Times)
{
	WORD		soc;
	DWORD		n1;
	tLLbyte		LLbyte;
	
	if(mAppScuGauage.StarCalQmaxSoc >= mAppScuGauage.Soc0)
		soc = mAppScuGauage.StarCalQmaxSoc - mAppScuGauage.Soc0;
	else
		soc = mAppScuGauage.Soc0 - mAppScuGauage.StarCalQmaxSoc;
	//	由放電的百分比及容量，反推Qmax 的值
	LLbyte.ll = (DDWORD)passcharge;
	LLbyte.ll *= (DDWORD)10000;	
	LLbyte.ll /= (DDWORD)soc;
					
	mAppScuGauage.QmaxUpdateTimes.i++;
	mAppScuGauage.Qmax.l = LLbyte.l[0];
	//------------------------------------------------------
	//	20180328 限制Qmax 不可大於 DC 
	if(mAppScuGauage.Qmax.l >= apiSysParGetDesignedCapacity())
		mAppScuGauage.Qmax.l = apiSysParGetDesignedCapacity();	
	//----------------------------------------------
	mAppScuGauage.QmaxuAms.ll = mAppScuGauage.Qmax.l;				
	mAppScuGauage.QmaxuAms.ll *= 1000;	//mA-->uA
	mAppScuGauage.QmaxuAms.ll *= 3600;	//h-->sec
	mAppScuGauage.QmaxuAms.ll *= 1000;	//sec-->ms
	//----------------------------------------------
	;//j-- SaveBatteryInfo();
	mAppScuGauage.QmaxStatus = 0;	//設定已更新過Qmax
	gaugeQPassChargeIni();			//重新設定PassCharge的基本值
	//--------------
	//計算QStart
	n1 = (10000L - mAppScuGauage.Soc0);
	n1 *= mAppScuGauage.Qmax.l;
	n1 /= 10000L;
	mAppScuGauage.Qstart.l = n1;
	//-------------
	//	必需先更新Qstart 再更新RM,因為更新RM 時會一併更新FCC
	gaugeSetRmAndFcc(mAppScuGauage.Soc0);
	if(Times==0)
	{
		if(mAppScuGauage.EvtHandler)
			mAppScuGauage.EvtHandler(0, APP_GAUGE_EVENT_UPDATE_QMAX_1ST, 0);
		;//j-- SaveFaultData(EVENT_UPDATE_QMAX_1ST,area);
	}
	else
	{
		if(mAppScuGauage.EvtHandler)
			mAppScuGauage.EvtHandler(0, APP_GAUGE_EVENT_UPDATE_QMAX, 0);

		;//j-- SaveFaultData(EVENT_UPDATE_QMAX,area);
	}	
	apiSysParSetQmaxUpdateTimes(mAppScuGauage.QmaxUpdateTimes.i);
	apiSysParSetQmax(mAppScuGauage.Qmax.l);
}

static uint8_t	gaugeUpdateCurrentMode(void)
{
	tCurrent	Current;
	tCurrent	AbsCurrentValue;
	Current = 	halAfeGetCurrentValue(0);
	AbsCurrentValue = abs(Current);
	
	if(AbsCurrentValue < apiSysParGetZeroCurrentValue())
	{
		mAppScuGauage.ChargeMode = APP_SCU_GAUGE_RELEASE_MODE;
	}
	else if(AbsCurrentValue > apiSysParGetMinChargeCurrentValue())
	{
		if(Current < 0)
			mAppScuGauage.ChargeMode = APP_SCU_GAUGE_CHARGE_MODE;
		else
			mAppScuGauage.ChargeMode = APP_SCU_GAUGE_DISCHARGE_MODE;
	}
	if(mAppScuGauage.ChargeModeTemp != mAppScuGauage.ChargeMode)
	{
		mAppScuGauage.ChargeModeTemp = mAppScuGauage.ChargeMode;
		mAppScuGauage.FirstReadCurrentFlag = 1;
	}
	return mAppScuGauage.ChargeMode;
}
static void gaugeCalAvgCurrent(void)
{
	double	d1,d2;
	DWORD	curmA;
	DWORD	diff;
	DWORD	dI;
	char	str[100];
	static	double	avg;
			
	if(mAppScuGauage.FirstReadCurrentFlag)
	{
		//scuDebugMsg("First");
		mAppScuGauage.FirstReadCurrentFlag = 0;
		avg = (double)abs(appGaugeGetCurrentValue());
		//mAppScuGauage.AbsAvggCurrent= (double)abs(appGaugeGetCurrentValue());
		//CellRAvgCurrent[area]=(double)curmA;
	}
	else
	{
		d1 = avg;//mAppScuGauage.AbsAvggCurrent;
		d1 *= 19.0;
		d1 /= 20.0;
		d2 = (double)abs(appGaugeGetCurrentValue());
		d2 /= (double)(20.0);
		//sprintf(str, "Avg %f %f", d1, d2);
		//scuDebugMsg(str);
		
		d1 += d2;
		avg = d1;
		mAppScuGauage.AbsAvggCurrent = (DWORD)d1;
		//------------------------------
		//d1=CellRAvgCurrent[area]*((double)9.0/(double)10.0);
		//d2=(double)curmA/(double)(10.0);
		//d1+=d2;
		//CellRAvgCurrent[area]=d1;
	}
}		

static void gaugeCalCapacity(void)
{
	char	str[100];
	uint64_t	CalRmValue;

	if(mAppScuGauage.GaugeTickMs == 0)
		return;
	CalRmValue = mAppScuGauage.GaugeTickMs;
	CalRmValue *= (uint64_t)abs(appGaugeGetCurrentValue());
	CalRmValue *= 1000u;	//uA
	
	mAppScuGauage.GaugeTickMs = 0;
	//sprintf(str, "%u", CalRmValue); 
	//appScuGaugeDebugMsg(str);

//	mAppScuGauage.TimeTickMs = 0;
	gaugeUpdateCurrentMode();
	gaugeCalAvgCurrent();
	
	if(appGaugeGetCurrentMode() == APP_SCU_GAUGE_CHARGE_MODE)
	{
		mAppScuGauage.RM.ll += CalRmValue; 
		if(mAppScuGauage.RM.ll > mAppScuGauage.QmaxuAms.ll)
			mAppScuGauage.RM.ll = mAppScuGauage.QmaxuAms.ll;

		//sprintf(str,"Charge Mode %u %d",
		//			mAppScuGauage.RM.ll,
		//			appScuGaugeCvtRmTomAh());
		
		//appScuGaugeDebugMsg(str);

		//----------------------------------
		if(mAppScuGauage.Soc0Valid)	//有抓到有效的Soc0,可以計算Qmax PassCharge
		{
			if(mAppScuGauage.QmaxStatus==0)
			{
				if(mAppScuGauage.EvtHandler)
					mAppScuGauage.EvtHandler(0,APP_GAUGE_EVENT_START_CHG_SOC,0);
				
			//		SaveFaultData(EVENT_START_CHG_SOC,BatteryCapInfo[area].Soc0);	//記錄休息五小時之後開始充電的Soc
				
				mAppScuGauage.StarCalQmaxSoc = mAppScuGauage.Soc0;
				mAppScuGauage.QmaxStatus = 1;
			}	
			mAppScuGauage.QmaxPassCharge.sll += CalRmValue;	
		}
		mAppScuGauage.RaPassCharge.sll += CalRmValue;
	}
	else if(appGaugeGetCurrentMode() == APP_SCU_GAUGE_DISCHARGE_MODE)
	{
		//appScuGaugeDebugMsg("DisCharge Mode");
		
		if(mAppScuGauage.RM.ll >= CalRmValue)
			mAppScuGauage.RM.ll -= CalRmValue; 
		else
			mAppScuGauage.RM.ll = 0;	
			
		
		//------------------------------
		if(mAppScuGauage.Soc0Valid)	//有抓到有效的Soc0,可以計算Qmax PassCharge
		{
			if(mAppScuGauage.QmaxStatus==0)
			{
//					char	str[100];
				if(mAppScuGauage.EvtHandler)
					mAppScuGauage.EvtHandler(0,APP_GAUGE_EVENT_START_DHG_SOC, &mAppScuGauage.Soc0);

				//SaveFaultData(EVENT_START_DHG_SOC,BatteryCapInfo[area].Soc0);	//記錄休息五小時之後開始放電的Soc

				mAppScuGauage.StarCalQmaxSoc = mAppScuGauage.Soc0;
				mAppScuGauage.QmaxStatus = 1;
//					sprintf(str,"設定 StarCalQmaxSoc=%d",BatteryCapInfo.Soc0);
//					SendUartMessage((BYTE *)str);
			}	
			mAppScuGauage.QmaxPassCharge.sll -= CalRmValue;	//累計開始放電的容量			
		}
		mAppScuGauage.TotalDisChargeCount.ll += CalRmValue;	//累計放電的容量,用於計算 Cycle count
		mAppScuGauage.RaPassCharge.sll -= CalRmValue;
	}
}
//------------------------------------
uint16_t getSocUseVoltage(uint32_t voltage)
{
	DWORD	dV,dR,y;
	BYTE	i;
	WORD	soc;
	
	if(mAppScuGauage.OcvTableNum < 1)
		return 0;
	//-----------------------------------------------
	soc = 5000;	//default value
	if(voltage < mAppScuGauage.OcvTable[0].Value)
		soc = 0;	//0%
	else if(voltage >= mAppScuGauage.OcvTable[mAppScuGauage.OcvTableNum - 1].Value)
		soc = (uint16_t)mAppScuGauage.OcvTable[mAppScuGauage.OcvTableNum - 1].Level * 100;		//100%
	else
	{
		for(i=0; i<(mAppScuGauage.OcvTableNum - 1); i++)
		{
			if(voltage >= mAppScuGauage.OcvTable[i].Value && 
		   	   voltage < mAppScuGauage.OcvTable[i+1].Value)
		   	{
		   		y = voltage - mAppScuGauage.OcvTable[i].Value;
		   		dV = mAppScuGauage.OcvTable[i+1].Value - mAppScuGauage.OcvTable[i].Value;
		   		dR = mAppScuGauage.OcvTable[i+1].Level - mAppScuGauage.OcvTable[i].Level;
		   		y *= dR;
		   		y *= 100L;
		   		y /= dV;		//換算出百分比
		 		y += ((DWORD)mAppScuGauage.OcvTable[i].Level*100L);
		 		soc = (WORD)y;
#if	0
				{
			 		char	str[100];
			 		sprintf(str,"%d %d %d %d %d %d %d",i,voltage,soc,
			 			EEPromParameter.OcvTable[i].Value.i,
		 				EEPromParameter.OcvTable[i].Level,
		 				EEPromParameter.OcvTable[i+1].Value.i,
		 				EEPromParameter.OcvTable[i+1].Level		 					 			
		 			);
			 		SendMessageToCan(str);
				}
#endif
		 		break;
		   	}
		}
	}
	return soc;
}

//------------------------------------
//	根據Cell電壓值,回傳SOC 
void getSocTableUseAvgCellVoltage(void)
{
	WORD	voltage;
	char	str[100];
	
	voltage = GetAverageCellVoltage();
	sprintf(str,"Avg CV=%d", voltage);
	scuDebugMsg(str);
	//-----------------------------------------------
	//	判斷是否在平坦區
	if(voltage > apiSysParGetMinFlatVoltage() &&
	   voltage < apiSysParGetMaxFlatVoltage())
	{
		mAppScuGauage.NowSocValid = 0;
		scuDebugMsg("平坦區");
		return;
	}
	mAppScuGauage.NowSoc = getSocUseVoltage(voltage);
	mAppScuGauage.NowSocValid = 1;
}

//--------------------------------------------
static uint16_t getTerminateSocFromOcvTable(void)
{
	DWORD	dS,dV,v;
	BYTE	i;
	if(mAppScuGauage.OcvTableNum < 1)
		return 0;
	if(mAppScuGauage.TerminateVoltage <= mAppScuGauage.OcvTable[0].Value)
		return 0;
	else if(mAppScuGauage.TerminateVoltage >= mAppScuGauage.OcvTable[mAppScuGauage.OcvTableNum-1].Value)
		return 10000;
	for(i=0; i<(mAppScuGauage.OcvTableNum-1); i++)
	{
		if((mAppScuGauage.TerminateVoltage >= mAppScuGauage.OcvTable[i].Value) &&
		   (mAppScuGauage.TerminateVoltage < mAppScuGauage.OcvTable[i+1].Value))
		{
			if(mAppScuGauage.TerminateVoltage == mAppScuGauage.OcvTable[i].Value)
				return ((WORD)mAppScuGauage.OcvTable[i].Level * 100);
			dS = mAppScuGauage.OcvTable[i+1].Level - mAppScuGauage.OcvTable[i].Level;
			dS *= 100L;
			dV = mAppScuGauage.OcvTable[i+1].Value- mAppScuGauage.OcvTable[i].Value;
			v = mAppScuGauage.TerminateVoltage - mAppScuGauage.OcvTable[i].Value;
			v *= dS;
			v /= dV;
			v += ((WORD)mAppScuGauage.OcvTable[i].Level * 100);
			return (WORD)v;
		}
	}
	return 0;
}

static void gaugeReleaseMode(void)
{
	long		slong;
	DWORD		n1,n2;
	BYTE		Over5HrFlag;
	BYTE		PlusFlag;
	char	str[100];

	if(mAppScuGauage.RelaxSec < 0xfff0)
		mAppScuGauage.RelaxSec++;
	mAppScuGauage.ReadSocTime++;
	//scuDebugMsg("gaugeReleaseMode....0");
	if(mAppScuGauage.ReadSocTime >= (30*60) )		//Relax 超過30分鐘
	{
		//scuDebugMsg("gaugeReleaseMode....1");
		mAppScuGauage.ReadSocTime = (30*60-100);	//下次在100 sec 之後更新Soc 0
		mAppScuGauage.SocB_Voltage = GetAverageCellVoltage();
		if(mAppScuGauage.SocReadNum == 0)
		{
			mAppScuGauage.SocA_Voltage = mAppScuGauage.SocB_Voltage;
			mAppScuGauage.SocReadNum = 1;
//				SendUartMessage("第一次取樣SOC");
		}			
		if(mAppScuGauage.RelaxSec >= (5L*3600L))	//總Relax時間超過5小時,無條件設為Soc0 或更新Qmax
		{
			if(!mAppScuGauage.Over5HrStatus)
			{
				if(mAppScuGauage.EvtHandler)
					mAppScuGauage.EvtHandler(0,APP_GAUGE_EVENT_IDLE_OVER_5HR, 0);

				;//j--	SaveFaultData(EVENT_IDLE_OVER_5HR,area);
					//BatteryCapInfo.Over5HrStatus=1;
			}
				//BatteryCapInfo.RelaxSec=0;
			;//j-- Over5HrFlag=TRUE;				
			//j--	goto _Update_Soc0;
		}
		else
			Over5HrFlag = 0;
				
			//if(BatteryCapInfo.SocB_Voltage>=BatteryCapInfo.SocA_Voltage)
			//	SocDeltaVoltage=BatteryCapInfo.SocB_Voltage-BatteryCapInfo.SocA_Voltage;
			//else
			//	SocDeltaVoltage=BatteryCapInfo.SocA_Voltage-BatteryCapInfo.SocB_Voltage;
			mAppScuGauage.SocA_Voltage = mAppScuGauage.SocB_Voltage;
			//if(SocDeltaVoltage>40)	//dv/dt <40uV 才可以識為正確的Soc
		if(1)						//6813 最低解析度為100uV
		{
				//SendUartMessage("Soc 壓差大於40uV");
			;//j--	goto _exit;
		}
//			SendUartMessage("Soc 壓差小於40uV");
//_Update_Soc0:
		//scuDebugMsg("_Update_Soc0");
		getSocTableUseAvgCellVoltage();
		if(mAppScuGauage.NowSocValid)	//有效的 SOC
		{
			//scuDebugMsg("gaugeReleaseMode....2");
			if(Over5HrFlag && (!mAppScuGauage.Over5HrStatus ||
				mAppScuGauage.Over5HrStatus==2))
			{
				if(mAppScuGauage.EvtHandler)
					mAppScuGauage.EvtHandler(0, APP_GAUGE_EVENT_GET_5HR_SOC, &mAppScuGauage.NowSoc);
				;//j--	SaveFaultData(EVENT_GET_5HR_SOC,BatteryCapInfo[area].NowSoc);				
			}
			mAppScuGauage.Over5HrStatus = 1;
			//sprintf(str,"讀取到有效的Soc 0 = %d", mAppScuGauage.NowSoc);
			scuDebugMsg(str);
			mAppScuGauage.Soc0Valid = 1;
			mAppScuGauage.Soc0 = mAppScuGauage.NowSoc;
			mAppScuGauage.StartCalRaSoc = mAppScuGauage.Soc0;	//設定為有效的RA Soc
			mAppScuGauage.RaSocValid = 1;
			gaugeRPassChargeIni();							//設定Ra PassCharge 初始值
			mAppScuGauage.RamSoc = mAppScuGauage.NowSoc;
			mAppScuGauage.SocBase = mAppScuGauage.RamSoc;
			//---------------------------------
			//	抓取新的截止點
			mAppScuGauage.CutOffSoc = getTerminateSocFromOcvTable();
			//--------------
			//計算QStart
			n1 = (10000L - mAppScuGauage.NowSoc);
			n1 *= mAppScuGauage.Qmax.l;
			n1 /= 10000L;
			mAppScuGauage.Qstart.l = n1;
			gaugeSetRmAndFcc(mAppScuGauage.NowSoc);
			mAppScuGauage.DisplaySocValue.i = mAppScuGauage.RsocValue.i;
		}
		else
		{
			if(Over5HrFlag && !mAppScuGauage.Over5HrStatus)
			{
				if(mAppScuGauage.EvtHandler)
					mAppScuGauage.EvtHandler(0, APP_GAUGE_EVENT_CANNOT_GET_5HR_SOC, 0);

				;//j--		SaveFaultData(EVENT_CANNOT_GET_5HR_SOC,0);		//無法讀取到正確的Soc
			}
			mAppScuGauage.Over5HrStatus = 2;
			;//j--	goto _exit;;
		}
		//----------------------------
		//Relax-->DisCharge-->Relax,	Update Qmax
		if(mAppScuGauage.QmaxStatus == 1)	
		{
			slong = (LONG)CvtQmaxPassChargeTomAh();
			if(slong < 0)
				n1 = slong * (-1);
			else
				n1 = slong;
			//sprintf(str,"PassCharge 總容量=%d",n1);			
			//SendUartMessage(str);		
			if(mAppScuGauage.QmaxUpdateTimes.i == 0)	//第一次更新
			{
				n2 = (apiSysParGetDesignedCapacity() * 9L) / 10L;
					//sprintf(str,"第一次更新允許容量:%d",n2);
					//SendMessageToCan(str);
				if(n1 >= n2)	
				{
					appScuGaugeUpdateQmax(n1,0);
				}
				else if(Over5HrFlag)	//第一次更新條件不成立而且Relax 總時間超過5小時，重設更新條件
				{
					mAppScuGauage.QmaxStatus = 0;	//重新設定更新條件
					gaugeQPassChargeIni();
				}
			}
			else
			{
				n2=(apiSysParGetDesignedCapacity() * 37L) / 100L;
				if(n1 >= n2)
				{
					appScuGaugeUpdateQmax(n1, 2);
				}
				else if(Over5HrFlag)	//第一次更新條件不成立而且Relax 總時間超過5小時，重設更新條件
				{
					mAppScuGauage.QmaxStatus = 0;	//重新設定更新條件
					gaugeQPassChargeIni();
				}
			}
		}	//if(BatteryCapInfo.QmaxStatus==1)	
	}		
	//---------
//j--	CheckNotFullSoc();
//	}	//if(ChargeMode==RELEASE_MODE)
}

//--------------------------------------------------------
//--------------------------------------------

//--------------------------------------------
WORD CheckTerminateSocUseRa(void)
{
	const	WORD StepTab[]={500,250,100,50,20,10,5,1,0};
	WORD	basesoc,soc,lastsoc,endsoc,SocStep;
	WORD	Vbat,v;
	BYTE	i;
	DWORD	realRa,vRa;
	WORD	RunTimes=0;
	
	basesoc = getTerminateSocFromOcvTable();	//??OCV table ??????? SOC
	lastsoc = basesoc;
	endsoc = 10000;
	SocStep = 1000;

	soc = lastsoc;
	for(i=0; i<10; i++)		//10%, 1% 0.1% 0.01%
	{
		for(;soc<endsoc; soc += SocStep)
		{
			RunTimes++;
			Vbat = GetOcvVoltage(soc);	//????
			realRa = (DWORD)GetRaFromSoc(soc);		//????Ra?
			//-------------------------------			
			realRa *= (DWORD)mAppScuGauage.Cal_RaValue;
			realRa /= (DWORD)mAppScuGauage.Table_RaValue;
			//-------------------------------
			vRa = abs(appGaugeGetCurrentValue());
			vRa *= (DWORD)realRa;

			vRa /= 100000L;			//???0.1mV ??
			if(vRa > 65535)			//j++ 20180328 ????overflow !!!
				vRa = 65535;
			v = (WORD)vRa;
			//-------------------------------			
			if(Vbat < v)
			{
				continue;
			}
			Vbat -= v;
			if(Vbat == mAppScuGauage.TerminateVoltage)
				return soc;
			if(Vbat >= mAppScuGauage.TerminateVoltage)
				break;
			lastsoc = soc;
		}
		endsoc = lastsoc + SocStep;
		SocStep = StepTab[i+1];
		if(SocStep == 0)
		{
			if(soc == 1)
				soc = 0;
			break;
		}
		soc = lastsoc + SocStep;
	}

	return soc;
}

//----------------------------------------------------
void appScuGaugeUpdateRm(WORD lastsoc,WORD nowsoc)
{
	BYTE		i;
	WORD		st,ed;
	WORD		SocTabVoltage;
	WORD		RealVoltage;
	double		cur,newRa;
	char	str[100];
	
	if(mAppScuGauage.RaTableNum<1)
		return;
		
	//scuDebugMsg("I.T...................");

	for(i=mAppScuGauage.RaTableNum-1; i>=1; i--)
	{
		st = mAppScuGauage.RaTable[i-1].Level * 100;
		ed = mAppScuGauage.RaTable[i].Level * 100;
				
		if(lastsoc>=ed && nowsoc>st && nowsoc<=ed && i != mAppScuGauage.RmUpdateLastLevel)
		{
			//sprintf(str,"RA Level=%d", i);
			//scuDebugMsg(str);
			mAppScuGauage.RmUpdateLastLevel = i;
			//-------------------------------
			SocTabVoltage = GetOcvVoltage(nowsoc);
			
			//-------------------------------
			RealVoltage = GetAverageCellVoltage();
			if(SocTabVoltage > RealVoltage)
			{
				mAppScuGauage.Table_RaValue = GetRaFromSoc(nowsoc);
//				cur = (double)mAppScuGauage.AvggCurrent.sl;
				cur = abs(appGaugeGetCurrentValue());
				newRa =(double)(SocTabVoltage - RealVoltage);
				newRa *= 100000.0;	//0.01mR
				newRa /= cur;
				mAppScuGauage.Cal_RaValue = (WORD)newRa;
				
				mAppScuGauage.CutOffSoc = CheckTerminateSocUseRa();

				//sprintf(str,"CalRa= %d RA CutSoc = %d AvgI=%f", 
				//		mAppScuGauage.Cal_RaValue,
				///		mAppScuGauage.CutOffSoc,
				//		cur
				//		);
				//scuDebugMsg(str);
				
				gaugeSetRmAndFcc(nowsoc);
				if(mAppScuGauage.EvtHandler)
				{
					mAppScuGauage.EvtHandler(0, APP_GAUGE_EVENT_CAL_RA1+i, &mAppScuGauage.Cal_RaValue);
					mAppScuGauage.EvtHandler(0, APP_GAUGE_EVENT_CAL_RA1+i, &mAppScuGauage.CutOffSoc);
					
					//j--SaveFaultData(EVENT_CAL_RA1+i,BatteryCapInfo[area].Cal_RaValue);	//
					//j--SaveFaultData(EVENT_CAL_RA1+i,BatteryCapInfo[area].CutOffSoc);	//
				}
				mAppScuGauage.CalRaIdleTimer = 10;
			}
			else
			{
				//scuDebugMsg("soc Vtable << Real Voltage !!!!!!!!");
			}
			break;
		}
		if(lastsoc>ed && nowsoc>ed)
		{
			if(mAppScuGauage.RmUpdateLastLevel != (i+1))
			{
				//sprintf(str,"???Leval=%d",i+1);
				//scuDebugMsg(str);
				;
			}
			mAppScuGauage.RmUpdateLastLevel = i + 1;
			break;
		}
	}
}

void gaugeCheckCycleCount(void)
{
	tLLbyte		LLbyte;
	char	str[100];
	uint32_t	mAh;
	
	mAh = mAppScuGauage.TotalDisChargeCount.ll /(3600u * 1000u * 1000u);
	
	LLbyte.ll = apiSysParGetDesignedCapacity();
	LLbyte.ll *= (DDWORD)(3600000u * 800u);		//uA sec
/*
	sprintf(str,"Cycle Count:%u %d / %d %.8lX%.8lX / %.8lX%.8lX",
			mAppScuGauage.CycleCount.i,
			mAh,
			apiSysParGetDesignedCapacity(),
			mAppScuGauage.TotalDisChargeCount.l[1],
			mAppScuGauage.TotalDisChargeCount.l[0],
			LLbyte.l[1],
			LLbyte.l[0]		
			);
		
	scuDebugMsg(str);
*/	
	if(mAppScuGauage.TotalDisChargeCount.ll >= LLbyte.ll)
	{
		mAppScuGauage.TotalDisChargeCount.ll -= LLbyte.ll;
		mAppScuGauage.CycleCount.i++;
		apiSysParSetCycleCount(mAppScuGauage.CycleCount.i);
	}
}


//-----------------------------------------------
static void scuGaugeUnReleasedMode(void)
{
	LONG		slong;
	DWORD		n1,n2;
	BYTE		Over5HrFlag;
	BYTE		PlusFlag;
	char	str[100];
	
	apiRamSaveLastChgDhgTime();
	//--------------------------
	//	計算最新的Soc值
	slong = CvtRaPassChargeTomAh();
	if(slong < 0)	//放電
	{
		n1 = slong * (-1);
		PlusFlag = 0;
	}
	else
	{
		PlusFlag = 1;
		n1 = slong;
	}
	n1 *= 10000L;
	n1 /= mAppScuGauage.Qmax.l;		//充放電百分比
	if(PlusFlag)					//充電
	{
		mAppScuGauage.RamSoc = mAppScuGauage.SocBase + n1;
		if(mAppScuGauage.RamSoc > 10000L)
			mAppScuGauage.RamSoc = 10000L;
	}
	else
	{
		if(mAppScuGauage.SocBase >= n1)
			mAppScuGauage.RamSoc = mAppScuGauage.SocBase - n1;
		else
			mAppScuGauage.RamSoc = 0;
	}
	//--------------------------
	if(appGaugeGetCurrentMode() == APP_SCU_GAUGE_DISCHARGE_MODE)	//在放電模式，可以判斷是否到到Rtable 的參考值
	{	
		mAppScuGauage.FullChargeTimerCount = 0;
		//DryCtrl_NoFull_Chg();
			
		//--------------------
		gaugeCheckCycleCount();
				
		//--------------------------
		//	判斷是否已到到截止電壓	
		{
			WORD	v;
			v = GetAverageCellVoltage();
			if(v < mAppScuGauage.TerminateVoltage)	//已放到截止電壓
			{
				mAppScuGauage.TerminateVoltageCount++;
				if(mAppScuGauage.TerminateVoltageCount >= 40)	//已連續低於截止電壓40 sec, 設定容量為0
				{
					mAppScuGauage.TerminateVoltageCount = 40;
					mAppScuGauage.RM.ll = 0;
					//scuDebugMsg("...............EDV");
				}
			}
			else
				mAppScuGauage.TerminateVoltageCount = 0;
		}
		//-------------------------	
		//	j++
		//	20171121 修改為直接以 RamSoc 做為Ra 校正的參考點,
		//	不再另外以 	StartCalRaSoc 為參考點
		if(mAppScuGauage.RaLastSoc != 0xffff)
		{						
			appScuGaugeUpdateRm(mAppScuGauage.RaLastSoc, mAppScuGauage.RamSoc);
		}
		mAppScuGauage.RaLastSoc = mAppScuGauage.RamSoc;
		//---------------------------
		//	放電模式,校正SOC 顯示值
		if(mAppScuGauage.DisplaySocValue.i != mAppScuGauage.RsocValue.i)
		{				
			if(mAppScuGauage.DisplaySocValue.i <= mAppScuGauage.RsocValue.i)
			{
				#if	0
				WORD	n;
				n=SystemParameter.RsocValue.i-SystemParameter.DisplaySocValue.i;
				n*=105;
				n/=100;	//20秒
				if(n==0)
					SystemParameter.DisplaySocValue.i=SystemParameter.RsocValue.i;
				else
					SystemParameter.DisplaySocValue.i+=n;	
				#endif
				;
			}
			else
			{
				WORD	n,n1;
				if(mAppScuGauage.LastRsocValue > mAppScuGauage.RsocValue.i)
					n1 = mAppScuGauage.LastRsocValue - mAppScuGauage.RsocValue.i;
				else
					n1 = 0;
				n1 /= 30;
				n = mAppScuGauage.DisplaySocValue.i - mAppScuGauage.RsocValue.i;
				n /= 30;	//20秒
				n1 += n;
				if(n1 >= mAppScuGauage.DisplaySocValue.i || n1 == 0)
					mAppScuGauage.DisplaySocValue.i = mAppScuGauage.RsocValue.i;
				else
					mAppScuGauage.DisplaySocValue.i -= n1;
			}
		}
	}	//if(ChargeMode==DISCHARGE_MODE)
	//------------------------------------------------------
	else		//充電模式
	{
		//------------------
		//	充電模式，判斷是否到達taper current & voltage 的條件
		{
			WORD	v;
			DWORD	current;
			DDWORD	d;
			current = abs(appGaugeGetCurrentValue());
				
			v = GetAverageCellVoltage();
			
			if(v >= mAppScuGauage.FullCharge.Voltage && 
			   current < mAppScuGauage.FullCharge.Current)	//已頂到充飽的條件
			{
				if(mAppScuGauage.FullChargeTimerCount < 200)
					mAppScuGauage.FullChargeTimerCount++;
				if(mAppScuGauage.FullChargeTimerCount >= mAppScuGauage.FullCharge.Second)
				{			
					#if 0					
					if(SystemParameter.BmuNumber==1)
						LedChargeMode(TRUE);	//充飽電
					else
					{
						if(BatteryCapInfo[0].FullChargeTimerCount>=40 &&
						   BatteryCapInfo[1].FullChargeTimerCount>=40)
						LedChargeMode(TRUE);	//充飽電   
					}
					#endif
					//BatteryCapInfo[area].FullChargeTimerCount=40;
					//----------------------------
					mAppScuGauage.RamSoc = 10000;
					mAppScuGauage.SocBase = 10000;
					mAppScuGauage.Qstart.l = 0;
					gaugeRPassChargeIni();		//設定Ra PassCharge 初始值
					mAppScuGauage.FCC.l = mAppScuGauage.Qmax.l;
					d = mAppScuGauage.Qmax.l;	//mAh
					d *= 1000;	//uA
					mAppScuGauage.RM.ll = (DDWORD)d*(DDWORD)3600*(DDWORD)1000;	//H-->ms
						//----------------------------
				}
			}
			else
			{
				//LedChargeMode(FALSE);
				mAppScuGauage.FullChargeTimerCount = 0;
			}
		}			
		//------------------				
#ifdef RE_CHARGE_TEST			
		old_soc=255;
#endif			
		mAppScuGauage.RaLastSoc = 0xffff;		//重新設定 Ra Last Soc 
		//---------------------------
		//	充電模式,校正SOC 顯示值
		if(mAppScuGauage.DisplaySocValue.i != mAppScuGauage.RsocValue.i)
		{
			if(mAppScuGauage.DisplaySocValue.i < mAppScuGauage.RsocValue.i)
			{
				WORD	n;
				n = mAppScuGauage.RsocValue.i - mAppScuGauage.DisplaySocValue.i;
				n /= 30;	//30秒
				if(n == 0)
					mAppScuGauage.DisplaySocValue.i = mAppScuGauage.RsocValue.i;
				else
					mAppScuGauage.DisplaySocValue.i += n;	
			}			
		}			
	}
	mAppScuGauage.RelaxSec = 0;
	mAppScuGauage.ReadSocTime = 0;
	mAppScuGauage.SocReadNum = 0;
	mAppScuGauage.Over5HrStatus = 0;
	mAppScuGauage.LastRsocValue = mAppScuGauage.RsocValue.i;
}

static void gaugeCorrectCapacity(void)
{
	gaugeCalSoc();	
	gaugeCalCellRValue();
	
	if(appGaugeGetCurrentMode() == APP_SCU_GAUGE_RELEASE_MODE)
		gaugeReleaseMode();
	else
	{
		scuGaugeUnReleasedMode();
	}
	
}

static void loadTotalDischargeCountFromRam(void)
{
	mAppScuGauage.TotalDisChargeCount.ll = apiRamLoadTotalDisChargeCount();	//mAh
	mAppScuGauage.TotalDisChargeCount.ll *= (3600u * 1000u *1000u);	//ms, uA
}

static void setupIdleTime(void)
{
	uint32_t	sec;
	sec = apiRamLoadReleaseTime();
	if(sec >= 0xfff0)
		sec = 0xfff0;
	mAppScuGauage.RelaxSec = sec;
	mAppScuGauage.ReadSocTime = sec;
}

static void baseInfoIni(void)
{
	uint32_t	n1;
	char	str[100];

	mAppScuGauage.RamSoc = apiRamLoadSoc();
	if(mAppScuGauage.RamSoc == 0xffff)
	{
		//scuDebugMsg("Ram Soc error");
		mAppScuGauage.RamSoc = 5000;
	}
	appGaugeSetQmax(apiSysParGetQmax());
	
	mAppScuGauage.QmaxUpdateTimes.i = apiSysParGetQmaxUpdateTimes();
	mAppScuGauage.CycleCount.i = apiSysParGetCycleCount();

	loadTotalDischargeCountFromRam();
	setupIdleTime();
	
//	mAppScuGauage.Qmax.l = apiSysParGetQmax();
//	mAppScuGauage.QmaxuAms.ll = mAppScuGauage.Qmax.l;
//	mAppScuGauage.QmaxuAms.ll *= (1000u * 3600u * 1000u);
	
	mAppScuGauage.SocBase = mAppScuGauage.RamSoc;
	n1 = (DWORD)(10000L - mAppScuGauage.RamSoc);
	n1 *= mAppScuGauage.Qmax.l;
	n1 /= 10000L;
	mAppScuGauage.Qstart.l = n1;
	gaugeSetRmAndFcc(mAppScuGauage.RamSoc);
	mAppScuGauage.DisplaySocValue.i = mAppScuGauage.RsocValue.i;
	mAppScuGauage.LastRsocValue = mAppScuGauage.RsocValue.i;
	mAppScuGauage.TerminateVoltage = apiSysParGetTerminateVoltage();

	mAppScuGauage.CutOffSoc = getTerminateSocFromOcvTable();
	sprintf(str,"CutSoc = %d", mAppScuGauage.CutOffSoc);
	scuDebugMsg(str);
//	mAppScuGauage.FCC.l = 280u * 1000u;
}


static void saveTotalDischargeCountToRam(void)
{
	uint32_t	mAh;
	if(appGaugeGetCurrentMode() != APP_SCU_GAUGE_DISCHARGE_MODE)		
		return;
	
	mAh = mAppScuGauage.TotalDisChargeCount.ll /(3600u * 1000u * 1000u);
	apiRamSaveTotalDisChargeCount(mAh);
}

static void gaugeSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	static uint16_t	count1 = 0;
	static uint8_t	count=0;
	char	str[100];
	
	//GPIOD->ODR |= GPIO_PIN_14;
	count1++;
	if(appProjectIsInSimuMode()==0 &&  halAfeGetState() != AFE_STATE_NORMAL)
	{
		if(count1 >= 2000)
		{
			//sprintf(str,"ret %d %d", appProjectIsSimuMode(), halAfeGetState());
			//appSerialCanDavinciSendTextMessage(str);
			count1 = 0;
		}
		if(evt == LIB_SW_TIMER_EVT_SW_10MS_2)
		{
			gaugeUpdateCurrentMode();
			gaugeCalAvgCurrent();
		}
		
		return;
	}
	if(count1 >= 2000)
	{
		//sprintf(str,"run %d %d", appProjectIsSimuMode(), halAfeGetState());
		//appSerialCanDavinciSendTextMessage(str);
		count1 = 0;
	}
	
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
		return;
	//GPIOD->ODR |= GPIO_PIN_14;
	if(evt == LIB_SW_TIMER_EVT_SW_10MS_1)
	{
		gaugeCalCapacity();	
		count++;
		if(count>=100)
		{
			count = 0;
		}
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		//gaugeCalAvgCurrent();
		
		//sprintf(str,"I = %d AvgI = %d %d", 
		//		abs(appGaugeGetCurrentValue()),
		//		mAppScuGauage.AbsAvggCurrent,
		//		mAppScuGauage.FirstReadCurrentFlag
		//		);
		//scuDebugMsg(str);		
		
		apiRamSaveSoc(mAppScuGauage.RamSoc);
		saveTotalDischargeCountToRam();

		gaugeCorrectCapacity();
	}
	//GPIOD->ODR &= ~GPIO_PIN_14;
}
static void gaugeHwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	mAppScuGauage.GaugeTickMs++;
}

static void setupFullChargeCondition(void)
{
	tScuProtectPar	mScuProtectPar;
	apiSysParGetFullChargeCondition(&mScuProtectPar);
	
	mAppScuGauage.FullCharge.Voltage = mScuProtectPar.SetValue.i[0];
	mAppScuGauage.FullCharge.Current = mScuProtectPar.STime.i[0];
	mAppScuGauage.FullCharge.Second = mScuProtectPar.RelValue.b[0];
}
/* Public function prototypes ----------------------------------------------- */
void appGaugeSetSoc0(uint16_t soc)
{
	uint32_t	n1;
	mAppScuGauage.Soc0Valid = 1;
	mAppScuGauage.Soc0 = soc;
	mAppScuGauage.StartCalRaSoc = mAppScuGauage.Soc0;	//設定為有效的RA Soc
	mAppScuGauage.RaSocValid = 1;
	gaugeRPassChargeIni();							//設定Ra PassCharge 初始值
	mAppScuGauage.RamSoc = mAppScuGauage.Soc0;
	mAppScuGauage.SocBase = mAppScuGauage.Soc0;
	//---------------------------------
	//	抓取新的截止點
	mAppScuGauage.CutOffSoc = getTerminateSocFromOcvTable();
	//--------------
	//計算QStart
	n1 = (10000L - mAppScuGauage.Soc0);
	n1 *= mAppScuGauage.Qmax.l;
	n1 /= 10000L;
	mAppScuGauage.Qstart.l = n1;
	gaugeSetRmAndFcc(mAppScuGauage.Soc0);
	mAppScuGauage.DisplaySocValue.i = mAppScuGauage.RsocValue.i;
}
void appGaugeUpdateSoc0(void)
{
	mAppScuGauage.RelaxSec = 5 * 3600;
	mAppScuGauage.ReadSocTime = 1800;
	mAppScuGauage.SocReadNum = 1;
	mAppScuGauage.SocA_Voltage = GetAverageCellVoltage();
	
//	BatteryCapInfo.RelaxSec = 5*3600;
//	BatteryCapInfo.ReadSocTime = 1800;
//	BatteryCapInfo.SocA_Voltage = GetAverageCellVoltage();
//	BatteryCapInfo.SocReadNum = 1;
}
void appGaugeCleanCycleCount(void)
{
	mAppScuGauage.CycleCount.i = 0;	
	mAppScuGauage.TotalDisChargeCount.ll = 0;
	apiSysParSetCycleCount(0);
	apiRamSaveTotalDisChargeCount(0);
}

uint16_t appGaugeGetCyleCount(void)
{
	return mAppScuGauage.CycleCount.i;
}


uint16_t appGaugeGetRSoc(void)
{
	return mAppScuGauage.RsocValue.i;
}

uint16_t appGaugeGetSoc0(void)
{
	return mAppScuGauage.Soc0;
}
 
uint16_t appGaugeGetSOH(void)
{
	uint32_t	n;
	
	n = mAppScuGauage.Qmax.l;
	n *= 10000L;
	n /= apiSysParGetDesignedCapacity();
	return n;
}

uint32_t appGaugeGetQmax(void)
{
	return mAppScuGauage.Qmax.l;
}

void appGaugeSetQmax(uint32_t qmax)
{
	mAppScuGauage.Qmax.l = qmax;
	mAppScuGauage.QmaxuAms.ll = mAppScuGauage.Qmax.l;
	mAppScuGauage.QmaxuAms.ll *= (1000u * 3600u * 1000u);
}

uint32_t appGaugeGetQStart(void)
{
	return mAppScuGauage.Qstart.l;
}
uint32_t appGaugeGetRM(void)
{
	return appScuGaugeCvtRmTomAh();
}

void appGaugeSetRM(uint32_t rm)
{
	mAppScuGauage.RM.ll = rm;
	mAppScuGauage.RM.ll *= (DDWORD)(3600u * 1000u * 1000u);    //uA->mA ms-->sec-->h
}

int32_t appGaugeGetQPassCharge(void)
{
	return CvtQmaxPassChargeTomAh();
}
int32_t appGaugeGetRPassCharge(void)
{
	return CvtRaPassChargeTomAh();
}
uint32_t appGaugeGetFCC(void)
{
	return mAppScuGauage.FCC.l;
}

uint8_t	appGaugeGetCurrentMode(void)
{
	return mAppScuGauage.ChargeMode;
}

tCurrent appGaugeGetCurrentValue(void)
{
	if(appGaugeGetCurrentMode() == APP_SCU_GAUGE_RELEASE_MODE)
		return 0;
	else 
		return halAfeGetCurrentValue(0);
}

uint16_t appGaugeGetRamSoc(void)
{
	return mAppScuGauage.RamSoc;
}
uint16_t appGaugeGetEndOfSoc(void)
{
	return mAppScuGauage.CutOffSoc;
}
uint16_t appGaugeGetDisplaySoc(void)
{
	return mAppScuGauage.DisplaySocValue.i;
}

void appGaugeOpen(tAppGaugeEvtHandler eventHandler)
{
	chargeModeIni();
	gaugeQPassChargeIni();
	gaugeRPassChargeIni();
	setupOcvTable();
	setupRaTable();
	baseInfoIni();
	setupFullChargeCondition();

	mAppScuGauage.EvtHandler = eventHandler;
	mAppScuGauage.GaugeTickMs = 0;	
  	LibSwTimerOpen(gaugeSwTimerHandler, 0);
  	LibHwTimerOpen(gaugeHwTimerHandler, 0);
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


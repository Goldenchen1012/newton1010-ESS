/**
  ******************************************************************************
  * @file        AppSystemParameter.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/8
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
#include "halafe.h"
#include "ApiSysPar.h"
#include "HalEeprom.h"
#include "LibSwTimer.h"
#include "LibNtc.h"
#include "LibCalibration.h"

void appSerialCanDavinciSendTextMessage(char *msg);
#define	appSysParDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)


/* Private define ------------------------------------------------------------*/
#define	SYSPAR_HEAD_INFO	"SysPar02"
#define	SYSPAR_DATE_CODE	0x1102
#define	PAR_ADDR			(0x08000000L + 510L * 1024L)

#define	CAL_PAR_HEAD_INFO	"CalPar01"
#define	CAL_PAR_DATE_CODE	0x1102
#define	CAL_PAR_ADDR		(0x08000000L + 508L * 1024L)

#define	BATINFO_PAR_ADDR	(0x08000000L + 506L * 1024L)

static	uint8_t SystemParIdleCount = 0;

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/


typedef struct{
	uint8_t			HeadInfo[8];
	uint16_t		ParLeng;
	uint16_t		DateCode;
	uint32_t		Checksum;
	uint32_t		HwVersion;
	uint32_t		DesignedCapacity;
	uint16_t		ZeroCurrent;
	uint16_t		MinChargeDischargeCurrent;
	uint16_t		MinFlatVoltage;
	uint16_t		MaxFlatVoltage;
	uint32_t		CellFlag[64];
	uint32_t		NtcFlag[64];
	uint8_t			BmuNumber;
	uint16_t		TerminateVoltage;

	uint16_t		OtHwSetValue;
	uint16_t		UtHwSetValue;
	uint16_t		OvpHwSetValue;
	uint16_t		UvpHwSetValue;
	
	struct{
		uint16_t	Voltage;
		uint16_t	Current;
		uint8_t		Time;
	}FullCharge;
	
	struct{
		uint16_t	SetValue;
		uint8_t		SetTime;
		uint16_t	ReleaseValue;
		uint8_t		ReleaseTime;
	}Ovp[3];	
		
	struct{
		uint16_t	SetValue;
		uint8_t		SetTime;
		uint16_t	ReleaseValue;
		uint8_t		ReleaseTime;
	}Uvp[3];
	
	struct{
		uint16_t	SetValue;
		uint8_t		SetTime;
		uint16_t	ReleaseValue;
		uint8_t		ReleaseTime;
	}Cotp[3];
	struct{
		uint16_t	SetValue;
		uint8_t		SetTime;
		uint16_t	ReleaseValue;
		uint8_t		ReleaseTime;
	}Cutp[3];
		struct{
		uint16_t	SetValue;
		uint8_t		SetTime;
		uint16_t	ReleaseValue;
		uint8_t		ReleaseTime;
	}Dotp[3];

	struct{
		uint16_t	SetValue;
		uint8_t		SetTime;
		uint16_t	ReleaseValue;
		uint8_t		ReleaseTime;
	}Dutp[3];
	
	struct{
		uint16_t	SetValue;
		uint8_t		SetTime;
		uint16_t	ReleaseValue;
		uint8_t		ReleaseTime;
	}Dtp[3];

	struct{
		uint16_t	SetValue;
		uint8_t		SetTime;
		uint16_t	ReleaseValue;
		uint8_t		ReleaseTime;
	}Cocp[3];
	
	struct{
		uint16_t	SetValue;
		uint8_t		SetTime;
		uint16_t	ReleaseValue;
		uint8_t		ReleaseTime;
	}Docp[3];
	
	tOcvRaTable	OcvTable[25];
	
	tOcvRaTable	RaTable[25];
	
	struct{
		uint16_t	SetValue;
		uint8_t		SetTime;
	}OvpPf;
	
	struct{
		uint16_t	SetValue;
		uint8_t		SetTime;
	}UvpPf;

//------------------------------------
	struct{
		uint8_t			DutySet;
		uint8_t			DutyRest;
		uint8_t			TempSet;
		uint8_t			TempRelease;
		//------------------------------------
		uint16_t		ChgSet;
		uint16_t		ChgRelease;
		uint16_t		ChgDeltaSet;
		uint16_t		ChgDeltaRelease;
		//------------------------------------
		uint16_t		DhgSet;
		uint16_t		DhgRelease;
		uint16_t		DhgDeltaSet;
		uint16_t		DhgDeltaRelease;
		//------------------------------------
		uint16_t		RlxSet;
		uint16_t		RlxRelease;
		uint16_t		RlxDeltaSet;
		uint16_t		RlxDeltaRelease;
	}Balance;
	uint8_t		NoteMessage[MAX_NOTE_MESSAGE_STRING_ITEM + 2];
	uint32_t	Reserved;
}tSysRomPar;
typedef struct{
	uint16_t		CellNumber;
	uint16_t		NtcNumber;

	struct{
		uint16_t	SetAdcValue;
		uint16_t	ReleaseAdcValue;
	}Cotp[3];
	struct{
		uint16_t	SetAdcValue;
		uint16_t	ReleaseAdcValue;
	}Cutp[3];
	struct{
		uint16_t	SetAdcValue;
		uint16_t	ReleaseAdcValue;
	}Dotp[3];

	struct{
		uint16_t	SetAdcValue;
		uint16_t	ReleaseAdcValue;
	}Dutp[3];
	struct{
		uint16_t		TempSetAdcValue;
		uint16_t		TempReleaseAdcValue;	
	}Balance;
}tSysRamPar;

typedef struct{
	
	tSysRomPar	RomPar;
	tSysRamPar	RamPar;

	uint32_t		Qmax;
	uint16_t		RelayOnDiffVoltage;
}tSysPar;

static tSysPar	SystemParemater;

tSysCalPar	SysCalPar;
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
//---------------------------------------
uint16_t appCalParSave(void);
void appCalParSetRamValue(void);


int32_t appCurrDebug(uint8_t CurrentIndex, int32_t adc)
{
	return doCalibration(&SysCalPar.RamPar.Current[CurrentIndex], adc);
}
int32_t appVbatDebug(uint8_t VbatIndex, int32_t adc)
{
	return doCalibration(&SysCalPar.RamPar.VBat[VbatIndex], adc);
}

void appCaliParSetCurrentValue(uint8_t CurrentIndex, uint8_t PointIndex,int32_t Value, int32_t Adc)
{
	if(PointIndex == CALI_P1_INDEX)
	{
		SysCalPar.RomPar.Currentt[CurrentIndex].valL = Value;
		SysCalPar.RomPar.Currentt[CurrentIndex].adcL = Adc;
	}
	else if(PointIndex == CALI_P2_INDEX)
	{
		SysCalPar.RomPar.Currentt[CurrentIndex].valH = Value;
		SysCalPar.RomPar.Currentt[CurrentIndex].adcH = Adc;
		appCalParSetRamValue();
		appCalParSave();
	}
}
void appCaliParGetCurrentValue(uint8_t CurrentIndex, uint8_t PointIndex,int32_t *Value, int32_t *Adc)
{
	if(PointIndex == CALI_P1_INDEX)
	{
		*Value = SysCalPar.RomPar.Currentt[CurrentIndex].valL;
		*Adc = SysCalPar.RomPar.Currentt[CurrentIndex].adcL;
	}
	else if(PointIndex == CALI_P2_INDEX)
	{
		*Value = SysCalPar.RomPar.Currentt[CurrentIndex].valH;
		*Adc = SysCalPar.RomPar.Currentt[CurrentIndex].adcH;
	}		
}

void appCaliParSetVbatValue(uint8_t VbatIndex, uint8_t PointIndex, int32_t Value, int32_t Adc)
{
	if(PointIndex == CALI_P1_INDEX)
	{
		SysCalPar.RomPar.VBat[VbatIndex].valL = Value;
		SysCalPar.RomPar.VBat[VbatIndex].adcL = Adc;
	}
	else if(PointIndex == CALI_P2_INDEX)
	{
		SysCalPar.RomPar.VBat[VbatIndex].valH = Value;
		SysCalPar.RomPar.VBat[VbatIndex].adcH = Adc;
		appCalParSetRamValue();
		appCalParSave();
	}
}
void appCaliParGetVbatValue(uint8_t VbatIndex, uint8_t PointIndex,int32_t *Value, int32_t *Adc)
{
	if(PointIndex == CALI_P1_INDEX)
	{
		*Value = SysCalPar.RomPar.VBat[VbatIndex].valL;
		*Adc = SysCalPar.RomPar.VBat[VbatIndex].adcL;
	}
	else if(PointIndex == CALI_P2_INDEX)
	{
		*Value = SysCalPar.RomPar.VBat[VbatIndex].valH;
		*Adc = SysCalPar.RomPar.VBat[VbatIndex].adcH;
	}		
}

//System parameter
//uint16_t AppSysParGetOvpSetValue(uint8_t level)
uint16_t appSysParSave(void);

static void appSysParSwTimerHanlder(__far void *dest, uint16_t evt, void *vDataPtr)
{
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
		
	}
	if(evt == LIB_SW_TIMER_EVT_SW_10MS_2)
	{
//		GPIOD->ODR ^= GPIO_PIN_15;	
		if(SystemParIdleCount)
		{
			SystemParIdleCount--;
			if(!SystemParIdleCount)
			{
				appSysParSave();
				LibSwTimerClose(appSysParSwTimerHanlder, 0);
			}
		}
	}
}
static void appSysParResetIdleCount(void)
{
	if(SystemParIdleCount == 0)
	{
	  	LibSwTimerOpen(appSysParSwTimerHanlder, 0);
	}
	SystemParIdleCount  = 50;
}
static void appSysParSetDefaultOvpValue(void)
{
	SystemParemater.RomPar.Ovp[0].SetValue = 3400;
	SystemParemater.RomPar.Ovp[0].SetTime = 10;
	SystemParemater.RomPar.Ovp[0].ReleaseValue = 3350;
	SystemParemater.RomPar.Ovp[0].ReleaseTime = 10;
         
	SystemParemater.RomPar.Ovp[1].SetValue = 3500;
	SystemParemater.RomPar.Ovp[1].SetTime = 10;
	SystemParemater.RomPar.Ovp[1].ReleaseValue = 3450;
	SystemParemater.RomPar.Ovp[1].ReleaseTime = 10;
         
	SystemParemater.RomPar.Ovp[2].SetValue = 3600;
	SystemParemater.RomPar.Ovp[2].SetTime = 10;
	SystemParemater.RomPar.Ovp[2].ReleaseValue = 3550;
	SystemParemater.RomPar.Ovp[2].ReleaseTime = 10;
}
static void appSysParSetDefaultUvpValue(void)
{
	SystemParemater.RomPar.Uvp[0].SetValue =3000;
	SystemParemater.RomPar.Uvp[0].SetTime = 10;
	SystemParemater.RomPar.Uvp[0].ReleaseValue = 2950;
	SystemParemater.RomPar.Uvp[0].ReleaseTime = 10;
	
	SystemParemater.RomPar.Uvp[1].SetValue = 2700;
	SystemParemater.RomPar.Uvp[1].SetTime = 10;
	SystemParemater.RomPar.Uvp[1].ReleaseValue = 2750;
	SystemParemater.RomPar.Uvp[1].ReleaseTime = 10;
                   
	SystemParemater.RomPar.Uvp[2].SetValue = 2600;
	SystemParemater.RomPar.Uvp[2].SetTime = 10;
	SystemParemater.RomPar.Uvp[2].ReleaseValue = 2650;
	SystemParemater.RomPar.Uvp[2].ReleaseTime = 10;	
	
}
static void appSysParSetDefaultCotpValue(void)
{
	SystemParemater.RomPar.Cotp[0].SetValue = 
			LibSetRealTemperatureToInternalValue(45);
	SystemParemater.RomPar.Cotp[0].SetTime = 10;
	SystemParemater.RomPar.Cotp[0].ReleaseValue = 
			LibSetRealTemperatureToInternalValue(40);
	SystemParemater.RomPar.Cotp[0].ReleaseTime = 10;
                   
	SystemParemater.RomPar.Cotp[1].SetValue = 
			LibSetRealTemperatureToInternalValue(50);
	SystemParemater.RomPar.Cotp[1].SetTime = 10;
	SystemParemater.RomPar.Cotp[1].ReleaseValue = 
			LibSetRealTemperatureToInternalValue(45);
	SystemParemater.RomPar.Cotp[1].ReleaseTime = 10;
                   
	SystemParemater.RomPar.Cotp[2].SetValue = 
			LibSetRealTemperatureToInternalValue(53);
	SystemParemater.RomPar.Cotp[2].SetTime = 10;
	SystemParemater.RomPar.Cotp[2].ReleaseValue = 
			LibSetRealTemperatureToInternalValue(45);
	SystemParemater.RomPar.Cotp[2].ReleaseTime = 10;
}
static void appSysParSetDefaultCutpValue(void)
{
	SystemParemater.RomPar.Cutp[0].SetValue = 
			LibSetRealTemperatureToInternalValue(5);
	SystemParemater.RomPar.Cutp[0].SetTime = 10;
	SystemParemater.RomPar.Cutp[0].ReleaseValue = 
			LibSetRealTemperatureToInternalValue(10);
	SystemParemater.RomPar.Cutp[0].ReleaseTime = 10;
                    
	SystemParemater.RomPar.Cutp[1].SetValue = 
			LibSetRealTemperatureToInternalValue(2);
	SystemParemater.RomPar.Cutp[1].SetTime = 10;
	SystemParemater.RomPar.Cutp[1].ReleaseValue = 
			LibSetRealTemperatureToInternalValue(5);
	SystemParemater.RomPar.Cutp[1].ReleaseTime = 10;
                    
	SystemParemater.RomPar.Cutp[2].SetValue = 
			LibSetRealTemperatureToInternalValue(0);
	SystemParemater.RomPar.Cutp[2].SetTime = 10;
	SystemParemater.RomPar.Cutp[2].ReleaseValue = 
			LibSetRealTemperatureToInternalValue(5);
	SystemParemater.RomPar.Cutp[2].ReleaseTime = 10;
}
static void appSysParSetDefaultDotpValue(void)
{
	SystemParemater.RomPar.Dotp[0].SetValue = 
			LibSetRealTemperatureToInternalValue(45);
	SystemParemater.RomPar.Dotp[0].SetTime = 10;
	SystemParemater.RomPar.Dotp[0].ReleaseValue = 
			LibSetRealTemperatureToInternalValue(40);
	SystemParemater.RomPar.Dotp[0].ReleaseTime = 10;
                    
	SystemParemater.RomPar.Dotp[1].SetValue = 
			LibSetRealTemperatureToInternalValue(50);
	SystemParemater.RomPar.Dotp[1].SetTime = 10;
	SystemParemater.RomPar.Dotp[1].ReleaseValue = 
			LibSetRealTemperatureToInternalValue(45);
	SystemParemater.RomPar.Dotp[1].ReleaseTime = 10;
                    
	SystemParemater.RomPar.Dotp[2].SetValue = 
			LibSetRealTemperatureToInternalValue(53);
	SystemParemater.RomPar.Dotp[2].SetTime = 10;
	SystemParemater.RomPar.Dotp[2].ReleaseValue = 
			LibSetRealTemperatureToInternalValue(45);
	SystemParemater.RomPar.Dotp[2].ReleaseTime = 10;
	
}
static void appSysParSetDefaultDutpValue(void)
{
	SystemParemater.RomPar.Dutp[0].SetValue = 
			LibSetRealTemperatureToInternalValue(-10);
	SystemParemater.RomPar.Dutp[0].SetTime = 10;
	SystemParemater.RomPar.Dutp[0].ReleaseValue = 
			LibSetRealTemperatureToInternalValue(-5);
	SystemParemater.RomPar.Dutp[0].ReleaseTime = 10;
                    
	SystemParemater.RomPar.Dutp[1].SetValue = 
			LibSetRealTemperatureToInternalValue(-15);
	SystemParemater.RomPar.Dutp[1].SetTime = 10;
	SystemParemater.RomPar.Dutp[1].ReleaseValue = 
			LibSetRealTemperatureToInternalValue(-12);
	SystemParemater.RomPar.Dutp[1].ReleaseTime = 10;
                    
	SystemParemater.RomPar.Dutp[2].SetValue = 
			LibSetRealTemperatureToInternalValue(-25);
	SystemParemater.RomPar.Dutp[2].SetTime = 10;
	SystemParemater.RomPar.Dutp[2].ReleaseValue = 
			LibSetRealTemperatureToInternalValue(-18);
	SystemParemater.RomPar.Dutp[2].ReleaseTime = 10;
}

static void appSysParSetDefaultCocpValue(void)
{
	SystemParemater.RomPar.Cocp[0].SetValue = 95;
	SystemParemater.RomPar.Cocp[0].SetTime = 8;
	SystemParemater.RomPar.Cocp[0].ReleaseValue = 90;
	SystemParemater.RomPar.Cocp[0].ReleaseTime = 9;
                    
	SystemParemater.RomPar.Cocp[1].SetValue = 100;
	SystemParemater.RomPar.Cocp[1].SetTime = 10;
	SystemParemater.RomPar.Cocp[1].ReleaseValue = 97;
	SystemParemater.RomPar.Cocp[1].ReleaseTime = 11;
                    
	SystemParemater.RomPar.Cocp[2].SetValue = 105;
	SystemParemater.RomPar.Cocp[2].SetTime = 12;
	SystemParemater.RomPar.Cocp[2].ReleaseValue = 102;
	SystemParemater.RomPar.Cocp[2].ReleaseTime = 13;
}

static void appSysParSetDefaultDocpValue(void)
{
	SystemParemater.RomPar.Docp[0].SetValue = 96;
	SystemParemater.RomPar.Docp[0].SetTime = 10;
	SystemParemater.RomPar.Docp[0].ReleaseValue = 91;
	SystemParemater.RomPar.Docp[0].ReleaseTime = 11;
                    
	SystemParemater.RomPar.Docp[1].SetValue = 101;
	SystemParemater.RomPar.Docp[1].SetTime = 12;
	SystemParemater.RomPar.Docp[1].ReleaseValue = 98;
	SystemParemater.RomPar.Docp[1].ReleaseTime = 13;
                    
	SystemParemater.RomPar.Docp[2].SetValue = 106;
	SystemParemater.RomPar.Docp[2].SetTime = 14;
	SystemParemater.RomPar.Docp[2].ReleaseValue = 103;
	SystemParemater.RomPar.Docp[2].ReleaseTime = 15;
}
static void appSysParSetDefaultBalanceValue(void)
{
	SystemParemater.RomPar.Balance.DutySet = 10;
	SystemParemater.RomPar.Balance.DutyRest = 5;
	SystemParemater.RomPar.Balance.TempSet = 
				LibSetRealTemperatureToInternalValue(50);
	SystemParemater.RomPar.Balance.TempRelease = 
				LibSetRealTemperatureToInternalValue(53);
	SystemParemater.RomPar.Balance.ChgSet = 3350;
	SystemParemater.RomPar.Balance.ChgRelease = 3350;
	SystemParemater.RomPar.Balance.ChgDeltaSet = 20;
	SystemParemater.RomPar.Balance.ChgDeltaRelease = 10;
	SystemParemater.RomPar.Balance.DhgSet = 3300;
	SystemParemater.RomPar.Balance.DhgRelease = 3300;
	SystemParemater.RomPar.Balance.DhgDeltaSet = 5000;
	SystemParemater.RomPar.Balance.DhgDeltaRelease = 2000;

	SystemParemater.RomPar.Balance.RlxSet = 3300;
	SystemParemater.RomPar.Balance.RlxRelease = 3300;
	SystemParemater.RomPar.Balance.RlxDeltaSet = 20;
	SystemParemater.RomPar.Balance.RlxDeltaRelease = 10;
}
static void appSysParSetDefaultOcvTable(void)
{
	uint8_t		i;
	tOcvRaTable	Ocv[]={
		{0,		2706},
		{2,		3051},
		{4,		3159},
		{5,		3196},
		{8,		3224},
		{10,	3226},
		{15,	3249},
		{20,	3278},
		{25,	3300},
		{30,	3307},
		{35,	3308},
		{40,	3308},
		{45,	3309},
		{50,	3311},
		{60,	3316},
		{75,	3343},
		{85,	3344},
		{94,	3344},
		{95,	3344},
		{98,	3345},
		{99,	3346},
		{100,	3360},
		{255,	65535}};
	
	for(i=0; i<25; i++)
	{
		if(Ocv[i].Level == 255)
			break;
		SystemParemater.RomPar.OcvTable[i].Level = Ocv[i].Level;
		SystemParemater.RomPar.OcvTable[i].Value = Ocv[i].Value;
	}

}
static void appSysParSetDefaultRaTable(void)
{
	uint8_t		i;
	tOcvRaTable	Ra[]={
		{0,		119},
		{2,		114},
		{4,		110},
		{5,		108},
		{8,		97},
		{10,	97},
		{15,	105},
		{20,	98},
		{25,	96},
		{30,	95},
		{35,	97},
		{40,	102},
		{45,	106},
		{60,	102},
		{75,	102},
		{85,	98},
		{94,	101},
		{98,	103},
		{99,	103},
		{100,	104},
		{255,	65535}};
	for(i=0; i<25; i++)
	{
		if(Ra[i].Level == 255)
			break;
		SystemParemater.RomPar.RaTable[i].Level = Ra[i].Level;
		SystemParemater.RomPar.RaTable[i].Value = Ra[i].Value;
	}
}

static void setDefaultOvpPfValue(void)
{
	SystemParemater.RomPar.OvpPf.SetValue = 4500;
	SystemParemater.RomPar.OvpPf.SetTime = 20;
}

static void setDefaultUvpPfValue(void)
{
	SystemParemater.RomPar.UvpPf.SetValue = 1500;
	SystemParemater.RomPar.UvpPf.SetTime = 20;
}

static void appSysParSetDefaultRomValue(void)
{
	uint8_t	i;
	
	
	SystemParemater.RomPar.HwVersion=0x0102;
	SystemParemater.RomPar.ZeroCurrent = 200;
	SystemParemater.RomPar.MinChargeDischargeCurrent = 500;
	SystemParemater.RomPar.MinFlatVoltage = 3278;
	SystemParemater.RomPar.MaxFlatVoltage = 3344;
	SystemParemater.RomPar.TerminateVoltage = 3000;
	SystemParemater.RomPar.OtHwSetValue = (40+60);
	SystemParemater.RomPar.UtHwSetValue = (40-10);
	SystemParemater.RomPar.OvpHwSetValue = 3800;
	SystemParemater.RomPar.UvpHwSetValue = 2500;
	SystemParemater.RomPar.FullCharge.Voltage = 3400;
	SystemParemater.RomPar.FullCharge.Current = 5250;
	SystemParemater.RomPar.FullCharge.Time = 10;
	
	SystemParemater.RomPar.DesignedCapacity = 280000L;
	SystemParemater.Qmax = 280000L;
	SystemParemater.RomPar.ZeroCurrent = 200;
	SystemParemater.RomPar.MinChargeDischargeCurrent = 300;
	SystemParemater.RomPar.BmuNumber = 2;
	for(i=0;i<32;i++)
	{
		SystemParemater.RomPar.CellFlag[i] = 0xffff;
		SystemParemater.RomPar.NtcFlag[i] = 0xffff;		
	}

	appSysParSetDefaultOvpValue();
	appSysParSetDefaultUvpValue();
	appSysParSetDefaultCotpValue();
    appSysParSetDefaultCutpValue();
	appSysParSetDefaultDotpValue();
	appSysParSetDefaultDutpValue();
	appSysParSetDefaultCocpValue();
	appSysParSetDefaultDocpValue();
	appSysParSetDefaultBalanceValue();
	appSysParSetDefaultOcvTable();
	appSysParSetDefaultRaTable();	
	
	setDefaultOvpPfValue();
	setDefaultUvpPfValue();
}

void appSysParSetRamValue(void)
{
	SystemParemater.RamPar.Cotp[0].SetAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Cotp[0].SetValue);
	SystemParemater.RamPar.Cotp[0].ReleaseAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Cotp[0].ReleaseValue);
	SystemParemater.RamPar.Cotp[1].SetAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Cotp[1].SetValue);
	SystemParemater.RamPar.Cotp[1].ReleaseAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Cotp[1].ReleaseValue);
	SystemParemater.RamPar.Cotp[2].SetAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Cotp[2].SetValue);
	SystemParemater.RamPar.Cotp[2].ReleaseAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Cotp[2].ReleaseValue);

	SystemParemater.RamPar.Cutp[0].SetAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Cutp[0].SetValue);
	SystemParemater.RamPar.Cutp[0].ReleaseAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Cutp[0].ReleaseValue);
	SystemParemater.RamPar.Cutp[1].SetAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Cutp[1].SetValue);
	SystemParemater.RamPar.Cutp[1].ReleaseAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Cutp[1].ReleaseValue);
	SystemParemater.RamPar.Cutp[2].SetAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Cutp[2].SetValue);
	SystemParemater.RamPar.Cutp[2].ReleaseAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Cutp[2].ReleaseValue);

	SystemParemater.RamPar.Dotp[0].SetAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Dotp[0].SetValue);
	SystemParemater.RamPar.Dotp[0].ReleaseAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Dotp[0].ReleaseValue);
	SystemParemater.RamPar.Dotp[1].SetAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Dotp[1].SetValue);
	SystemParemater.RamPar.Dotp[1].ReleaseAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Dotp[1].ReleaseValue);
	SystemParemater.RamPar.Dotp[2].SetAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Dotp[2].SetValue);
	SystemParemater.RamPar.Dotp[2].ReleaseAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Dotp[2].ReleaseValue);

	SystemParemater.RamPar.Dutp[0].SetAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Dutp[0].SetValue);
	SystemParemater.RamPar.Dutp[0].ReleaseAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Dutp[0].ReleaseValue);
	SystemParemater.RamPar.Dutp[1].SetAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Dutp[1].SetValue);
	SystemParemater.RamPar.Dutp[1].ReleaseAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Dutp[1].ReleaseValue);
	SystemParemater.RamPar.Dutp[2].SetAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Dutp[2].SetValue);
	SystemParemater.RamPar.Dutp[2].ReleaseAdcValue = 
				LibTemperatureToVoltage(SystemParemater.RomPar.Dutp[2].ReleaseValue);

	SystemParemater.RamPar.Balance.TempSetAdcValue = 
					LibTemperatureToVoltage(SystemParemater.RomPar.Balance.TempSet);
	SystemParemater.RamPar.Balance.TempReleaseAdcValue = 
					LibTemperatureToVoltage(SystemParemater.RomPar.Balance.TempRelease);
}

void appSysParUpdateCellNumber(void)
{
	uint32_t	flag;
	uint8_t		bmuindex,b;
	
	SystemParemater.RamPar.CellNumber = 0;
	for(bmuindex=0; bmuindex<SystemParemater.RomPar.BmuNumber;  bmuindex++)
	{
		flag = SystemParemater.RomPar.CellFlag[bmuindex];
		for(b=0; b<32; b++)
		{
			if(flag & 0x01)
			{
				SystemParemater.RamPar.CellNumber++;	
			}
			flag >>= 1;
		}
	}	
}
void appSysParUpdateNtcNumber(void)
{
	uint32_t	flag;
	uint8_t		bmuindex,b;
	
	SystemParemater.RamPar.NtcNumber = 0;
	for(bmuindex=0; bmuindex<SystemParemater.RomPar.BmuNumber;  bmuindex++)
	{
		flag = SystemParemater.RomPar.NtcFlag[bmuindex];
		for(b=0; b<32; b++)
		{
			if(flag & 0x01)
			{
				SystemParemater.RamPar.NtcNumber++;	
			}
			flag >>= 1;
		}
	}	
}

//-----------------------------------------------------------


uint32_t appSysParGetHwVersion(void)
{
	return SystemParemater.RomPar.HwVersion;
}
void appSysParSetHwVersion(uint32_t version)
{
	SystemParemater.RomPar.HwVersion = version;
	appSysParResetIdleCount();
}
uint32_t appSysParGetFwVersion(void)
{
	//SystemParemater.FwVersion = 0x010203;
	return 0;//SystemParemater.FwVersion;
}

uint8_t appSysParGetBmuNumber(void)
{
	return SystemParemater.RomPar.BmuNumber;	
}
void appSysParSetBmuNumber(uint8_t num)
{
	SystemParemater.RomPar.BmuNumber = num;
	appSysParResetIdleCount();
}

uint32_t appSysParGetCellFlag(uint8_t BmuIndex)
{
	return SystemParemater.RomPar.CellFlag[BmuIndex];	
}
void appSysParSetCellFlag(uint8_t BmuIndex,uint32_t CellFlag)
{
	SystemParemater.RomPar.CellFlag[BmuIndex] = CellFlag;
	appSysParResetIdleCount();
}

uint32_t appSysParGetNtcFlag(uint8_t BmuIndex)
{
	return SystemParemater.RomPar.NtcFlag[BmuIndex];	
}

void appSysParSetNtcFlag(uint8_t BmuIndex, uint32_t NtcFlag)
{
	SystemParemater.RomPar.NtcFlag[BmuIndex] = NtcFlag;
	appSysParResetIdleCount();
}

uint32_t appSysParGetQmax(void)
{
	return 1234;//SystemParemater.Qmax;	
}


uint16_t appSysParGetZeroCurrentValue(void)
{
	return SystemParemater.RomPar.ZeroCurrent;	
}
void appSysParSetZeroCurrentValue(uint16_t current)
{
	SystemParemater.RomPar.ZeroCurrent = current;
	appSysParResetIdleCount();
}

uint16_t appSysParGetMinChargeCurrentValue(void)
{
	return SystemParemater.RomPar.MinChargeDischargeCurrent;	
}

void appSysParSetMinChargeCurrentValue(uint16_t current)
{
	SystemParemater.RomPar.MinChargeDischargeCurrent = current;	
	appSysParResetIdleCount();
}

uint32_t appSysParGetDesignedCapacity(void)
{
	return SystemParemater.RomPar.DesignedCapacity;
}

void appSysParSetDesignedCapacity(uint32_t dc)
{
	SystemParemater.RomPar.DesignedCapacity = dc;
	appSysParResetIdleCount();
}

uint16_t appSysParGetMinFlatVoltage(void)
{
	return SystemParemater.RomPar.MinFlatVoltage;
}

uint16_t appSysParGetMaxFlatVoltage(void)
{
	return SystemParemater.RomPar.MaxFlatVoltage;
}
void appSysParGetFlatVoltage(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.MinFlatVoltage;
	pPar->STime.l = SystemParemater.RomPar.MaxFlatVoltage;
}

void appSysParSetFlatVoltage(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.MinFlatVoltage = pPar->SetValue.l;
	SystemParemater.RomPar.MaxFlatVoltage = pPar->STime.l;
	appSysParResetIdleCount();
}

void appSysParGetFullChargeCondition(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.FullCharge.Voltage;
	pPar->STime.l = SystemParemater.RomPar.FullCharge.Current;
	pPar->RelValue.l = SystemParemater.RomPar.FullCharge.Time;
}
void appSysParSetFullChargeCondition(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.FullCharge.Voltage = pPar->SetValue.l;
	SystemParemater.RomPar.FullCharge.Current = pPar->STime.l;
	SystemParemater.RomPar.FullCharge.Time = pPar->RelValue.l;
	appSysParResetIdleCount();
}	
uint16_t appSysParGetTerminateVoltage(void)
{
	return SystemParemater.RomPar.TerminateVoltage;
}
void appSysParSetTerminateVoltage(uint16_t voltage)
{
	SystemParemater.RomPar.TerminateVoltage = voltage;
	appSysParResetIdleCount();
}

uint16_t appSysParGetCellNumber(void)
{
	return SystemParemater.RamPar.CellNumber;
}

uint16_t appSysParGetNtcNumber(void)
{
	return SystemParemater.RamPar.NtcNumber;
}
//---------------------------------------
void appSysParGetOcvTable(uint8_t index ,tOcvRaTable *pOcvTable)
{
	pOcvTable->Level =  SystemParemater.RomPar.OcvTable[index].Level;
	pOcvTable->Value =  SystemParemater.RomPar.OcvTable[index].Value;
}

void appSysParGetRaTable(uint8_t index ,tOcvRaTable *pOcvTable)
{
	pOcvTable->Level =  SystemParemater.RomPar.RaTable[index].Level;
	pOcvTable->Value =  SystemParemater.RomPar.RaTable[index].Value;
}

void appSysParSetOcvTable(uint8_t index ,tOcvRaTable *pOcvTable)
{
	SystemParemater.RomPar.OcvTable[index].Level = pOcvTable->Level;
	SystemParemater.RomPar.OcvTable[index].Value = pOcvTable->Value;
	appSysParResetIdleCount();
}

void appSysParSetRaTable(uint8_t index ,tOcvRaTable *pOcvTable)
{
	SystemParemater.RomPar.RaTable[index].Level = pOcvTable->Level;
	SystemParemater.RomPar.RaTable[index].Value = pOcvTable->Value;
	appSysParResetIdleCount();
}

//---------------------------------------
//Protect parameter
//OVP
void appSysParGetOvpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
	{
		if(level == 0x10)
			pPar->SetValue.l = SystemParemater.RomPar.OvpHwSetValue;
		else
		{
			pPar->SetValue.l = 0;
			pPar->STime.l = 0;
			pPar->RelValue.l = 0;
			pPar->RTime.l = 0;
		}
		return;
	}
	pPar->SetValue.l = SystemParemater.RomPar.Ovp[level].SetValue;
	pPar->STime.l = SystemParemater.RomPar.Ovp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RomPar.Ovp[level].ReleaseValue;
	pPar->RTime.l = SystemParemater.RomPar.Ovp[level].ReleaseTime;
}

void appSysParSetOvpPar(uint8_t level, tScuProtectPar *pPar)
{	
	if(level >= 3)
	{
		if(level == 0x10)
			SystemParemater.RomPar.OvpHwSetValue = pPar->SetValue.l;
		return;
	}
	SystemParemater.RomPar.Ovp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Ovp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Ovp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Ovp[level].ReleaseTime = pPar->RTime.l;
	appSysParResetIdleCount();
}

// UVP

void appSysParGetUvpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
	{
		if(level == 0x10)
			pPar->SetValue.l = SystemParemater.RomPar.UvpHwSetValue;
		else
		{
			pPar->SetValue.l = 0;
			pPar->STime.l = 0;
			pPar->RelValue.l = 0;
			pPar->RTime.l = 0;
		}
		return;
	}
	pPar->SetValue.l = SystemParemater.RomPar.Uvp[level].SetValue;
	pPar->STime.l = SystemParemater.RomPar.Uvp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RomPar.Uvp[level].ReleaseValue;
	pPar->RTime.l = SystemParemater.RomPar.Uvp[level].ReleaseTime;
}

void appSysParSetUvpPar(uint8_t level, tScuProtectPar *pPar)
{	
	if(level >= 3)
	{
		if(level == 0x10)
			SystemParemater.RomPar.UvpHwSetValue = pPar->SetValue.l;
		return;
	}
	SystemParemater.RomPar.Uvp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Uvp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Uvp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Uvp[level].ReleaseTime = pPar->RTime.l;
	appSysParResetIdleCount();
}

void appSysParGet2ndOtProtectPar(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.OtHwSetValue;
}
void appSysParSet2ndOtProtectPar(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.OtHwSetValue = pPar->SetValue.l; 
}
void appSysParGet2ndUtProtectPar(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.UtHwSetValue;
}
void appSysParSet2ndUtProtectPar(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.UtHwSetValue = pPar->SetValue.l;
}

//---------------------------------------------------
//	Dotp
void appSysParGetDotpProtectPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	pPar->SetValue.l = SystemParemater.RamPar.Dotp[level].SetAdcValue;
	pPar->STime.l = SystemParemater.RomPar.Dotp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RamPar.Dotp[level].ReleaseAdcValue;
	pPar->RTime.l = SystemParemater.RomPar.Dotp[level].ReleaseTime;
}


void appSysParGetDotpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	pPar->SetValue.l = SystemParemater.RomPar.Dotp[level].SetValue;
	pPar->STime.l = SystemParemater.RomPar.Dotp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RomPar.Dotp[level].ReleaseValue;
	pPar->RTime.l = SystemParemater.RomPar.Dotp[level].ReleaseTime;
}
void appSysParSetDotpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	SystemParemater.RomPar.Dotp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Dotp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Dotp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Dotp[level].ReleaseTime = pPar->RTime.l;	
	appSysParResetIdleCount();
}
//----------- DUTP ---------------------------
void appSysParGetDutpProtectPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	pPar->SetValue.l = SystemParemater.RamPar.Dutp[level].SetAdcValue;
	pPar->STime.l = SystemParemater.RomPar.Dutp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RamPar.Dutp[level].ReleaseAdcValue;
	pPar->RTime.l = SystemParemater.RomPar.Dutp[level].ReleaseTime;
}

void appSysParGetDutpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	pPar->SetValue.l = SystemParemater.RomPar.Dutp[level].SetValue;
	pPar->STime.l = SystemParemater.RomPar.Dutp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RomPar.Dutp[level].ReleaseValue;
	pPar->RTime.l = SystemParemater.RomPar.Dutp[level].ReleaseTime;
}
void appSysParSetDutpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	SystemParemater.RomPar.Dutp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Dutp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Dutp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Dutp[level].ReleaseTime = pPar->RTime.l;	
	appSysParResetIdleCount();
}
//----------- DTP ---------------------------
void appSysParGetDtpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	pPar->SetValue.l = SystemParemater.RomPar.Dtp[level].SetValue;
	pPar->STime.l = SystemParemater.RomPar.Dtp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RomPar.Dtp[level].ReleaseValue;
	pPar->RTime.l = SystemParemater.RomPar.Dtp[level].ReleaseTime;
}
void appSysParSetDtpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	SystemParemater.RomPar.Dtp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Dtp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Dtp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Dtp[level].ReleaseTime = pPar->RTime.l;	
	appSysParResetIdleCount();
}
//------------------------------------------------------
//	Cotp
void appSysParGetCotpProtectPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	pPar->SetValue.l = SystemParemater.RamPar.Cotp[level].SetAdcValue;
	pPar->STime.l = SystemParemater.RomPar.Cotp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RamPar.Cotp[level].ReleaseAdcValue;
	pPar->RTime.l = SystemParemater.RomPar.Cotp[level].ReleaseTime;
}

void appSysParGetCotpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
	{
		if(level == 0x10)
			 pPar->SetValue.l = SystemParemater.RomPar.OtHwSetValue;
		return;
	}
	pPar->SetValue.l = SystemParemater.RomPar.Cotp[level].SetValue;
	pPar->STime.l = SystemParemater.RomPar.Cotp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RomPar.Cotp[level].ReleaseValue;
	pPar->RTime.l = SystemParemater.RomPar.Cotp[level].ReleaseTime;
}
void appSysParSetCotpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
	{
		if(level == 0x10)
			SystemParemater.RomPar.OtHwSetValue = pPar->SetValue.l;
		return;
	}
	SystemParemater.RomPar.Cotp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Cotp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Cotp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Cotp[level].ReleaseTime = pPar->RTime.l;
	appSysParResetIdleCount();
}
	
//-----------------------------------------------	
//	Cutp
void appSysParGetCutpProtectPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
	{
		return;
	}
	pPar->SetValue.l = SystemParemater.RamPar.Cutp[level].SetAdcValue;
	pPar->STime.l = SystemParemater.RomPar.Cutp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RamPar.Cutp[level].ReleaseAdcValue;
	pPar->RTime.l = SystemParemater.RomPar.Cutp[level].ReleaseTime;
}
void appSysParGetCutpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
	{
		if(level == 0x10)
			pPar->SetValue.l = SystemParemater.RomPar.UtHwSetValue;
		return;
	}
	pPar->SetValue.l = SystemParemater.RomPar.Cutp[level].SetValue;
	pPar->STime.l = SystemParemater.RomPar.Cutp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RomPar.Cutp[level].ReleaseValue;
	pPar->RTime.l = SystemParemater.RomPar.Cutp[level].ReleaseTime;
}
void appSysParSetCutpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
	{
		if(level == 0x10)
			SystemParemater.RomPar.UtHwSetValue = pPar->SetValue.l;
		return;
	}
	SystemParemater.RomPar.Cutp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Cutp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Cutp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Cutp[level].ReleaseTime = pPar->RTime.l;	
	appSysParResetIdleCount();
}

//-----------------------------------------------------
//	Docp
void appSysParGetDocpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	pPar->SetValue.l = SystemParemater.RomPar.Docp[level].SetValue;
	pPar->STime.l = SystemParemater.RomPar.Docp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RomPar.Docp[level].ReleaseValue;
	pPar->RTime.l = SystemParemater.RomPar.Docp[level].ReleaseTime;

}
void appSysParSetDocpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	SystemParemater.RomPar.Docp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Docp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Docp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Docp[level].ReleaseTime = pPar->RTime.l;	
	appSysParResetIdleCount();
}

//---------------------------------------------------
//Cocp
void appSysParGetCocpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	pPar->SetValue.l = SystemParemater.RomPar.Cocp[level].SetValue;
	pPar->STime.l = SystemParemater.RomPar.Cocp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RomPar.Cocp[level].ReleaseValue;
	pPar->RTime.l = SystemParemater.RomPar.Cocp[level].ReleaseTime;

}
void appSysParSetCocpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	SystemParemater.RomPar.Cocp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Cocp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Cocp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Cocp[level].ReleaseTime = pPar->RTime.l;	
	appSysParResetIdleCount();
}
//-----------------------------------------
//	PF
void appSysParGetOvpPfPar(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.OvpPf.SetValue;
	pPar->STime.l = SystemParemater.RomPar.OvpPf.SetTime; 
}
void appSysParSetOvpPfPar(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.OvpPf.SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.OvpPf.SetTime = pPar->STime.l;
	appSysParResetIdleCount();
}

void appSysParGetUvpPfPar(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.UvpPf.SetValue;
	pPar->STime.l = SystemParemater.RomPar.UvpPf.SetTime; 
}
void appSysParSetUvpPfPar(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.UvpPf.SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.UvpPf.SetTime = pPar->STime.l;
	appSysParResetIdleCount();
}

//------------------------------------------
void appSysParGetBalanceDuty(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.Balance.DutySet;
	pPar->STime.l = SystemParemater.RomPar.Balance.DutyRest;
	pPar->RelValue.l = SystemParemater.RomPar.Balance.TempSet;
	pPar->RTime.l = SystemParemater.RomPar.Balance.TempRelease;
}
void appSysParSetBalanceDuty(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.Balance.DutySet = pPar->SetValue.l;
	SystemParemater.RomPar.Balance.DutyRest = pPar->STime.l;
	SystemParemater.RomPar.Balance.TempSet = pPar->RelValue.l;
	SystemParemater.RomPar.Balance.TempRelease = pPar->RTime.l;
	appSysParResetIdleCount();
}
void appSysParGetBalanceChg(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.Balance.ChgSet;
	pPar->STime.l = SystemParemater.RomPar.Balance.ChgRelease;
	pPar->RelValue.l = SystemParemater.RomPar.Balance.ChgDeltaSet;
	pPar->RTime.l = SystemParemater.RomPar.Balance.ChgDeltaRelease;
}
void appSysParSetBalanceChg(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.Balance.ChgSet = pPar->SetValue.l;
	SystemParemater.RomPar.Balance.ChgRelease = pPar->STime.l;
	SystemParemater.RomPar.Balance.ChgDeltaSet = pPar->RelValue.l;
	SystemParemater.RomPar.Balance.ChgDeltaRelease = pPar->RTime.l;
	appSysParResetIdleCount();
}

void appSysParGetBalanceDhg(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.Balance.DhgSet;
	pPar->STime.l = SystemParemater.RomPar.Balance.DhgRelease;
	pPar->RelValue.l = SystemParemater.RomPar.Balance.DhgDeltaSet;
	pPar->RTime.l = SystemParemater.RomPar.Balance.DhgDeltaRelease;
}
void appSysParSetBalanceDhg(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.Balance.DhgSet = pPar->SetValue.l;
	SystemParemater.RomPar.Balance.DhgRelease = pPar->STime.l;
	SystemParemater.RomPar.Balance.DhgDeltaSet = pPar->RelValue.l;
	SystemParemater.RomPar.Balance.DhgDeltaRelease = pPar->RTime.l;
	appSysParResetIdleCount();
}
void appSysParGetBalanceRlx(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.Balance.RlxSet;
	pPar->STime.l = SystemParemater.RomPar.Balance.RlxRelease;
	pPar->RelValue.l = SystemParemater.RomPar.Balance.RlxDeltaSet;
	pPar->RTime.l = SystemParemater.RomPar.Balance.RlxDeltaRelease;
}
void appSysParSetBalanceRlx(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.Balance.RlxSet = pPar->SetValue.l;
	SystemParemater.RomPar.Balance.RlxRelease = pPar->STime.l;
	SystemParemater.RomPar.Balance.RlxDeltaSet = pPar->RelValue.l;
	SystemParemater.RomPar.Balance.RlxDeltaRelease = pPar->RTime.l;
	appSysParResetIdleCount();
}
//----------------------------------------------------------
//	Note Message
void appSysParGetNotwMessageString(uint8_t *pMsg)
{
	uint8_t	i;
	for(i=0; i<MAX_NOTE_MESSAGE_STRING_ITEM; i++)
	{
		pMsg[i] = SystemParemater.RomPar.NoteMessage[i];
		if(pMsg[i] == 0)
			break;
	}
}
void appSysParSetNotwMessageString(uint8_t *pMsg)
{
	uint8_t	i;
	for(i=0; i<MAX_NOTE_MESSAGE_STRING_ITEM; i++)
	{
		SystemParemater.RomPar.NoteMessage[i] = pMsg[i];
		if(pMsg[i] == 0)
			break;
	}
	appSysParResetIdleCount();
}
//----------------------------------------------------------------
//----------------------------------------------------------------
void DumpBuffer(uint8_t *pBuf,uint16_t len);


uint16_t appCalParSave(void)
{
	uint32_t	*par;
	uint32_t	Checksum = 0;
	tHalEeProm	mHalEeProm;
	
	memcpy(&SysCalPar.RomPar.HeadInfo, CAL_PAR_HEAD_INFO, 8);
	SysCalPar.RomPar.ParLeng = sizeof(tCalRomPar);
	SysCalPar.RomPar.DateCode = CAL_PAR_DATE_CODE;
	SysCalPar.RomPar.Checksum = 0;
	par =(uint32_t *)&SysCalPar.RomPar;
	SysCalPar.RomPar.Reserved = 0;
	while(1)
	{
		Checksum ^= (uint32_t)*par;
		par++;
		if(par >= &SysCalPar.RomPar.Reserved)
			break;
	}
	SysCalPar.RomPar.Checksum = Checksum;
	
	mHalEeProm.StartAddress = CAL_PAR_ADDR;
	mHalEeProm.Length = SysCalPar.RomPar.ParLeng;
	mHalEeProm.pDataBuffer = (uint8_t *)&SysCalPar.RomPar;
	
	if(HalEePromErase(&mHalEeProm) != 0)
			appSysParDebugMsg("Save Cal Par ..erase error");
	
	HalEePromWrite(&mHalEeProm);
	
	appSysParDebugMsg("Save Cal Par");
}
void appCalParSetRamValue(void)
{
	uint8_t	i;
	
	for(i=0; i<2; i++)
	{
		SysCalPar.RamPar.Current[i] = calCoef(
								SysCalPar.RomPar.Currentt[i].valL,
								SysCalPar.RomPar.Currentt[i].adcL,
								SysCalPar.RomPar.Currentt[i].valH,
								SysCalPar.RomPar.Currentt[i].adcH);
		SysCalPar.RamPar.VBat[i] = calCoef(
								SysCalPar.RomPar.VBat[i].valL,
								SysCalPar.RomPar.VBat[i].adcL,
								SysCalPar.RomPar.VBat[i].valH,
								SysCalPar.RomPar.VBat[i].adcH);

	}								

	
}
uint16_t appCalParLoad(void)
{
	uint8_t	buffer[2100];
	tHalEeProm	mHalEeProm;
	tSysCalPar	*pSysCalPar;
	uint8_t		flag = 0;
	uint32_t	*par;
	uint32_t	Checksum = 0;
	char	str[100];
	
	mHalEeProm.StartAddress = CAL_PAR_ADDR;
	mHalEeProm.Length = sizeof(tSysCalPar);
	mHalEeProm.pDataBuffer = buffer;
	
	appSysParDebugMsg("Load cal Par");
	HalEePromRead(&mHalEeProm);
	
	//DumpBuffer(buffer,64);
	
	pSysCalPar = (tSysCalPar *)buffer;
	if(memcmp(pSysCalPar->RomPar.HeadInfo, CAL_PAR_HEAD_INFO, 8) != 0)
	{
		appSysParDebugMsg("Cal Head Info Error");	
		flag = 1;
	}
	else if(pSysCalPar->RomPar.ParLeng != sizeof(tCalRomPar))
	{
		appSysParDebugMsg("Cal len Error");	
		flag = 1;
	}
	else if(pSysCalPar->RomPar.DateCode != CAL_PAR_DATE_CODE)
	{
		appSysParDebugMsg("Cal DateCode Error");	
		flag = 1;
	}
	if(!flag)
	{
		par =(uint32_t *)pSysCalPar;
		while(1)
		{
			Checksum ^= (uint32_t)*par;
			par++;
			if(par >= &pSysCalPar->RomPar.Reserved)
				break;
		}
		if(Checksum != 0)
		{	
			flag = 1;
			sprintf(str,"Cal Checksum Error =%.8lX %.8lX", Checksum,
					pSysCalPar->RomPar.Checksum	
						);
			appSysParDebugMsg(str);
		}
	}
	if(flag)
	{
		;//appSysParSave();
	}
	else
	{
		appSysParDebugMsg("Load Par ok");		
		memcpy(&SysCalPar.RomPar ,&pSysCalPar->RomPar, sizeof(tSysCalPar));
	}
}
//----------------------------------------------------------------

uint16_t appSysParSave(void)
{
	uint32_t	*par;
	uint32_t	Checksum = 0;
	tHalEeProm	mHalEeProm;
	
	memcpy(&SystemParemater.RomPar.HeadInfo, SYSPAR_HEAD_INFO, 8);
	SystemParemater.RomPar.ParLeng = sizeof(tSysRomPar);
	SystemParemater.RomPar.DateCode = SYSPAR_DATE_CODE;
	SystemParemater.RomPar.Checksum = 0;
	par =(uint32_t *)&SystemParemater.RomPar;
	SystemParemater.RomPar.Reserved = 0;
	while(1)
	{
		Checksum ^= (uint32_t)*par;
		par++;
		if(par >= &SystemParemater.RomPar.Reserved)
			break;
	}
	SystemParemater.RomPar.Checksum = Checksum;
	
	mHalEeProm.StartAddress = PAR_ADDR;
	mHalEeProm.Length = SystemParemater.RomPar.ParLeng;
	mHalEeProm.pDataBuffer = (uint8_t *)&SystemParemater.RomPar;
	
	if(HalEePromErase(&mHalEeProm) != 0)
			appSysParDebugMsg("Save Sys Par ..erase error");
	
	HalEePromWrite(&mHalEeProm);
	
	appSysParDebugMsg("Save Sys Par");
}

uint16_t appSysParLoad(void)
{
	uint8_t	buffer[2100];
	tHalEeProm	mHalEeProm;
	tSysPar		*pSysPar;
	uint8_t		flag = 0;
	uint32_t	*par;
	uint32_t	Checksum = 0;
	char	str[100];
	
	mHalEeProm.StartAddress = PAR_ADDR;
	mHalEeProm.Length = sizeof(tSysRomPar);
	mHalEeProm.pDataBuffer = buffer;
	
	appSysParDebugMsg("Load Par");
	HalEePromRead(&mHalEeProm);
	
	DumpBuffer(buffer,64);
	
	pSysPar = (tSysPar *)buffer;
	if(memcmp(pSysPar->RomPar.HeadInfo, SYSPAR_HEAD_INFO, 8) != 0)
	{
		appSysParDebugMsg("Head Info Error");	
		flag = 1;
	}
	else if(pSysPar->RomPar.ParLeng != sizeof(tSysRomPar))
	{
		appSysParDebugMsg("len Error");	
		flag = 1;
	}
	else if(pSysPar->RomPar.DateCode != SYSPAR_DATE_CODE)
	{
		appSysParDebugMsg("DateCode Error");	
		flag = 1;
	}
	if(!flag)
	{
		par =(uint32_t *)pSysPar;
		while(1)
		{
			Checksum ^= (uint32_t)*par;
			par++;
			if(par >= &pSysPar->RomPar.Reserved)
				break;
		}
		if(Checksum != 0)
		{	
			flag = 1;
			sprintf(str,"Checksum Error =%.8lX %.8lX", Checksum,
					pSysPar->RomPar.Checksum	
						);
			appSysParDebugMsg(str);
		}
	}
	if(flag)
	{
		;//appSysParSave();
	}
	else
	{
		appSysParDebugMsg("Load Par ok");		
		memcpy(&SystemParemater.RomPar ,&pSysPar->RomPar, sizeof(tSysRomPar));
	}
}
//----------------------------------------------------------------
uint16_t appSysParOpen(void)
{
	SystemParIdleCount = 0;
	
	appSysParSetDefaultRomValue();
	appSysParLoad();
	appCalParLoad();
	appCalParSetRamValue();
	
	appSysParUpdateCellNumber();
	appSysParUpdateNtcNumber();
	appSysParSetRamValue();
	return sizeof(tSysRomPar);
}


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


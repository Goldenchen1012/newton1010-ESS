/**
  ******************************************************************************
  * @file        ApiSysPar.c
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

#include "define.h"
#include "main.h"
#include "LibDebug.h"
#include "halafe.h"
#include "ApiSysPar.h"
#include "HalEeprom.h"
#include "LibSwTimer.h"
#include "LibNtc.h"
#include "LibCalibration.h"


void appSerialCanDavinciSendTextMessage(uint8_t *str);
#define	sysParDebugMsg(str)	appSerialCanDavinciSendTextMessage((uint8_t *)str)

void DumpBuffer(uint8_t *pBuf,uint16_t len);


/* Private typedef -----------------------------------------------------------*/
#define	BATINFO_OVP_PF_FLAG		0x0001
#define	BATINFO_UVP_PF_FLAG		0x0002
#define	MAX_BMU_NUM				64

typedef struct{
	uint8_t			HeadInfo[8];
	uint16_t		ParLeng;
	uint16_t		DateCode;
	uint32_t		Checksum;
	uint32_t		Qmax;
	uint16_t		QmaxUpdateTimes;
	uint16_t		CycleCount;
	uint16_t		PfFlag;
	uint32_t		Reserved;
}tBatteryInfo;


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
	uint32_t		CellFlag[MAX_BMU_NUM];
	uint32_t		NtcFlag[MAX_BMU_NUM];
	uint8_t			BmuNumber;
	uint16_t		TerminateVoltage;
	
	struct{
		uint16_t		SetValue;
		uint8_t			SetTime;
	}OtHwSetValue;
	struct{
		uint16_t		SetValue;
		uint8_t			SetTime;
	}UtHwSetValue;
	struct{
		uint16_t		SetValue;
		uint8_t			SetTime;
	}OvpHwSetValue;
	struct{
		uint16_t		SetValue;
		uint8_t			SetTime;
	}UvpHwSetValue;
	
	uint16_t		PreDischargeTime;
	uint16_t		RelayOnThreshold;
	
	struct{
		uint8_t		ReadInterval;
		uint8_t		SwDelay;	
	}InsulationResistance;
	
	struct{
		uint8_t		L1Time;
		uint8_t		L2Time;
	}AfeComm;
	
	struct{
		uint16_t	Current;
		uint16_t	Voltage;
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
	uint32_t	RelayActiveFlag;
	uint32_t	W5500_IpAddress;
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
}tSysPar;

typedef struct{
	uint8_t			HeadInfo[8];
	uint16_t		ParLeng;
	uint16_t		DateCode;
	uint32_t		Checksum;
	uint8_t			ScuID1;
	uint8_t			ScuID2;
	uint8_t			ScuID3;
	uint32_t		Reserved;
}tScuIdInfo;

/* Private define ------------------------------------------------------------*/

#define	SYSPAR_HEAD_INFO			"SysPar01"
#define	SYSPAR_DATE_CODE			0x1214
#define	PAR_ADDR					(0x08000000L + 510L * 1024L)

#define	CAL_PAR_HEAD_INFO			"CalPar01"
#define	CAL_PAR_DATE_CODE			0x1102
#define	CAL_PAR_ADDR				(0x08000000L + 508L * 1024L)

#define	BAT_INFO_PAR_HEAD_INFO		"BatPar01"
#define	BAT_INFO_PAR_DATE_CODE		0x1108
#define	BAT_INFO_PAR_ADDR			(0x08000000L + 506L * 1024L)

#define	SCUID_PAR_HEAD_INFO		"ScuID001"
#define	SCUID_PAR_DATE_CODE		0x2107
#define	SCUID_PAR_ADDR			(0x08000000L + 504L * 1024L)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
tBatteryInfo	mBatteryInfo;
static tSysPar	SystemParemater;
tSysCalPar	SysCalPar;
static tScuIdInfo	mScuIdInfo;

static	uint8_t BatInfoIdleCount = 0;
static	uint8_t SystemParIdleCount = 0;

/* Public variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void setBatteryInfoDefaultValue(void)
{
	mBatteryInfo.Qmax = 280001;
	mBatteryInfo.QmaxUpdateTimes = 0;
	mBatteryInfo.CycleCount = 0;
	mBatteryInfo.PfFlag =0;
}

static void saveBatteryInfoPar(void)
{
	uint32_t		*par;
	uint32_t		Checksum = 0;
	tHalEeProm		mHalEeProm;
	
	memcpy(&mBatteryInfo.HeadInfo, BAT_INFO_PAR_HEAD_INFO, 8);
	mBatteryInfo.ParLeng = sizeof(tBatteryInfo);
	mBatteryInfo.DateCode = BAT_INFO_PAR_DATE_CODE;
	mBatteryInfo.Checksum = 0;
	mBatteryInfo.Reserved = 0;
	par =(uint32_t *)&mBatteryInfo;
	while(1)
	{
		Checksum ^= (uint32_t)*par;
		par++;
		if(par >= &mBatteryInfo.Reserved)
			break;
	}
	mBatteryInfo.Checksum = Checksum;
	
	mHalEeProm.StartAddress = BAT_INFO_PAR_ADDR;
	mHalEeProm.Length = mBatteryInfo.ParLeng;
	mHalEeProm.pDataBuffer = (uint8_t *)&mBatteryInfo;
	
	if(HalEePromErase(&mHalEeProm) != 0)
		sysParDebugMsg("Save batinfo Par ..erase error");
	
	HalEePromWrite(&mHalEeProm);
	
	sysParDebugMsg("Save batinfo Par");
}


static void loadBatteryInfoPar(void)
{
	uint8_t			buffer[2100];
	tHalEeProm		mHalEeProm;
	tBatteryInfo	*pBatteryInfo;
	uint8_t			flag = 0;
	uint32_t		*par;
	uint32_t		Checksum = 0;
	char			str[100];
	
	mHalEeProm.StartAddress = BAT_INFO_PAR_ADDR;
	mHalEeProm.Length = sizeof(tBatteryInfo);
	mHalEeProm.pDataBuffer = buffer;
	
	sysParDebugMsg("Load bat Par");
	HalEePromRead(&mHalEeProm);
	
	//DumpBuffer(buffer,64);
	
	pBatteryInfo = (tBatteryInfo *)buffer;
	if(memcmp(pBatteryInfo->HeadInfo, BAT_INFO_PAR_HEAD_INFO, 8) != 0)
	{
		sysParDebugMsg("bat info Head Info Error");	
		flag = 1;
	}
	else if(pBatteryInfo->ParLeng != sizeof(tBatteryInfo))
	{
		sysParDebugMsg("batinfo len Error");	
		flag = 1;
	}
	else if(pBatteryInfo->DateCode != BAT_INFO_PAR_DATE_CODE)
	{
		sysParDebugMsg("bat info DateCode Error");	
		flag = 1;
	}
	if(!flag)
	{
		par =(uint32_t *)pBatteryInfo;
		while(1)
		{
			Checksum ^= (uint32_t)*par;
			par++;
			if(par >= &pBatteryInfo->Reserved)
				break;
		}
		if(Checksum != 0)
		{	
			flag = 1;
			sprintf(str,"BatInfo Checksum Error =%.8lX %.8lX", Checksum,
					pBatteryInfo->Checksum	
						);
			sysParDebugMsg(str);
		}
	}
	if(flag)
	{
	}
	else
	{
		sysParDebugMsg("BatInfo Load Par ok");		
		memcpy(&mBatteryInfo ,pBatteryInfo, sizeof(tBatteryInfo));
		
		sprintf(str,"Qmax=%d" ,mBatteryInfo.Qmax);
		sysParDebugMsg(str);
	}	
}
static void batteryInfoSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	//GPIOD->ODR |= GPIO_PIN_14;
	if(evt == LIB_SW_TIMER_EVT_SW_10MS_2)
	{
		if(BatInfoIdleCount)
		{
			BatInfoIdleCount--;
			if(!BatInfoIdleCount)
			{
				saveBatteryInfoPar();
				LibSwTimerClose(batteryInfoSwTimerHandler, 0);
			}
		}
	}
	//GPIOD->ODR &= ~GPIO_PIN_14;
}

static void resetBatteryInfoIdleCount(void)
{
	if(BatInfoIdleCount == 0)
	{
	  	LibSwTimerOpen(batteryInfoSwTimerHandler, 0);
	}
	BatInfoIdleCount  = 50;
}


static uint16_t loadSysPar(void)
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
	
	sysParDebugMsg("Load Par");
	HalEePromRead(&mHalEeProm);
	
	//DumpBuffer(buffer,64);
	
	pSysPar = (tSysPar *)buffer;
	if(memcmp(pSysPar->RomPar.HeadInfo, SYSPAR_HEAD_INFO, 8) != 0)
	{
		sysParDebugMsg("Head Info Error");	
		flag = 1;
	}
	else if(pSysPar->RomPar.ParLeng != sizeof(tSysRomPar))
	{
		sysParDebugMsg("len Error");	
		flag = 1;
	}
	else if(pSysPar->RomPar.DateCode != SYSPAR_DATE_CODE)
	{
		sysParDebugMsg("DateCode Error");	
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
			sysParDebugMsg(str);
		}
	}
	if(flag)
	{
	}
	else
	{
		sysParDebugMsg("Load Par ok");		
		memcpy(&SystemParemater.RomPar ,&pSysPar->RomPar, sizeof(tSysRomPar));
	}
}

static void saveSysPar(void)
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
			sysParDebugMsg("Save Sys Par ..erase error");
	
	HalEePromWrite(&mHalEeProm);
	
	sysParDebugMsg("Save Sys Par");
}



static void loadCalPar(void)
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
	
	sysParDebugMsg("Load cal Par");
	HalEePromRead(&mHalEeProm);
	
	//DumpBuffer(buffer,64);
	
	pSysCalPar = (tSysCalPar *)buffer;
	if(memcmp(pSysCalPar->RomPar.HeadInfo, CAL_PAR_HEAD_INFO, 8) != 0)
	{
		sysParDebugMsg("Cal Head Info Error");	
		flag = 1;
	}
	else if(pSysCalPar->RomPar.ParLeng != sizeof(tCalRomPar))
	{
		sysParDebugMsg("Cal len Error");	
		flag = 1;
	}
	else if(pSysCalPar->RomPar.DateCode != CAL_PAR_DATE_CODE)
	{
		sysParDebugMsg("Cal DateCode Error");	
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
			sysParDebugMsg(str);
		}
	}
	if(flag)
	{
		;
	}
	else
	{
		sysParDebugMsg("Load Par ok");		
		memcpy(&SysCalPar.RomPar ,&pSysCalPar->RomPar, sizeof(tSysCalPar));
	}
}

static void saveCalPar(void)
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
			sysParDebugMsg("Save Cal Par ..erase error");
	
	HalEePromWrite(&mHalEeProm);
	
	sysParDebugMsg("Save Cal Par");
}

//wwwwwwwwwwwwwwwwwwwwwwwwwwwww

static void loadScuIdPar(void)
{
	uint8_t		buffer[2100];
	tHalEeProm	mHalEeProm;
	tScuIdInfo	*pScuIdInfo;
	uint8_t		flag = 0;
	uint32_t	*par;
	uint32_t	Checksum = 0;
	uint8_t		scuid;
	char		str[100];
	
	mHalEeProm.StartAddress = SCUID_PAR_ADDR;
	mHalEeProm.Length = sizeof(tScuIdInfo);
	mHalEeProm.pDataBuffer = buffer;
	
	sysParDebugMsg("Load scuid Par");
	HalEePromRead(&mHalEeProm);
	
	//DumpBuffer(buffer,64);
	
	pScuIdInfo = (tScuIdInfo *)buffer;
	if(memcmp(pScuIdInfo->HeadInfo, SCUID_PAR_HEAD_INFO, 8) != 0)
	{
		sysParDebugMsg("scuid Head Info Error");	
		flag = 1;
	}
	else if(pScuIdInfo->ParLeng != sizeof(tScuIdInfo))
	{
		sysParDebugMsg("scuid len Error");	
		flag = 1;
	}
	else if(pScuIdInfo->DateCode != SCUID_PAR_DATE_CODE)
	{
		sysParDebugMsg("scuid DateCode Error");	
		flag = 1;
	}
	if(!flag)
	{
		par =(uint32_t *)pScuIdInfo;
		while(1)
		{
			Checksum ^= (uint32_t)*par;
			par++;
			if(par >= &pScuIdInfo->Reserved)
				break;
		}
		if(Checksum != 0)
		{	
			flag = 1;
			sprintf(str,"scuid Checksum Error =%.8lX %.8lX", Checksum,
					pScuIdInfo->Checksum	
						);
			sysParDebugMsg(str);
		}
	}
	if(flag)
	{
		mScuIdInfo.ScuID1 = 0xff;
	}
	else
	{
		sysParDebugMsg("Load scuid ok");		
		memcpy(&mScuIdInfo ,pScuIdInfo, sizeof(tSysCalPar));
		if(mScuIdInfo.ScuID1 == mScuIdInfo.ScuID2 ||
		   mScuIdInfo.ScuID1 == mScuIdInfo.ScuID3)
		{
			scuid = mScuIdInfo.ScuID1;
		}
		else if(mScuIdInfo.ScuID2 == mScuIdInfo.ScuID3)
			scuid = mScuIdInfo.ScuID2;
		else
			scuid = 0xff;
		mScuIdInfo.ScuID1 = scuid;		
		sprintf(str,"Scuid = %d",scuid);
		sysParDebugMsg(str);
	}
}
//static tScuIdInfo	ScuIdInfo;
/*
#define	SCUID_PAR_HEAD_INFO		"ScuID001"
#define	SCUID_PAR_DATE_CODE		0x2107
#define	SCUID_PAR_ADDR			(0x08000000L + 504L * 1024L)
*/
uint8_t apiSysParGetScuId(void)
{
	return mScuIdInfo.ScuID1;
}

void saveScuIdPar(uint8_t scuid)
{
	uint32_t	*par;
	uint32_t	Checksum = 0;
	tHalEeProm	mHalEeProm;
	
	memcpy(&mScuIdInfo.HeadInfo, SCUID_PAR_HEAD_INFO, 8);
	mScuIdInfo.ParLeng = sizeof(tScuIdInfo);
	mScuIdInfo.DateCode = SCUID_PAR_DATE_CODE;
	mScuIdInfo.Checksum = 0;
	mScuIdInfo.ScuID1 = scuid;
	mScuIdInfo.ScuID2 = scuid;
	mScuIdInfo.ScuID3 = scuid;
	par =(uint32_t *)&mScuIdInfo;
	mScuIdInfo.Reserved = 0;
	while(1)
	{
		Checksum ^= (uint32_t)*par;
		par++;
		if(par >= &mScuIdInfo.Reserved)
			break;
	}
	mScuIdInfo.Checksum = Checksum;
	
	mHalEeProm.StartAddress = SCUID_PAR_ADDR;
	mHalEeProm.Length = mScuIdInfo.ParLeng;
	mHalEeProm.pDataBuffer = (uint8_t *)&mScuIdInfo;
	
	if(HalEePromErase(&mHalEeProm) != 0)
		sysParDebugMsg("Save scuid Par ..erase error");
	
	HalEePromWrite(&mHalEeProm);
	
	sysParDebugMsg("Save scuid Par");
}
//wwwwwwwwwwwwwwwwwwwwwwwwwwwww

static void sysParSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
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
				saveSysPar();
				LibSwTimerClose(sysParSwTimerHandler, 0);
			}
		}
	}
}
static void resetSysParIdleCount(void)
{
	if(SystemParIdleCount == 0)
	{
	  	LibSwTimerOpen(sysParSwTimerHandler, 0);
	}
	SystemParIdleCount  = 50;
}
static void setCalParRamValue(void)
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


static void setDefaultOvpValue(void)
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
static void setDefaultUvpValue(void)
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
static void setDefaultCotpValue(void)
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
static void setDefaultCutpValue(void)
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
static void setDefaultDotpValue(void)
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
static void setDefaultDutpValue(void)
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

static void setDefaultCocpValue(void)
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

static void setDefaultDocpValue(void)
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
static void setDefaultBalanceValue(void)
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
static void setDefaultOcvTable(void)
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
static void setDefaultRaTable(void)
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

static void sysParSetDefaultRomValue(void)
{
	uint8_t	i;
	
	
	SystemParemater.RomPar.HwVersion=0x0102;
	SystemParemater.RomPar.ZeroCurrent = 200;
	SystemParemater.RomPar.MinChargeDischargeCurrent = 500;
	SystemParemater.RomPar.MinFlatVoltage = 3278;
	SystemParemater.RomPar.MaxFlatVoltage = 3344;
	SystemParemater.RomPar.TerminateVoltage = 3000;
	SystemParemater.RomPar.OtHwSetValue.SetValue = (40+60);
	SystemParemater.RomPar.OtHwSetValue.SetTime = 0;
	SystemParemater.RomPar.UtHwSetValue.SetValue = (40-10);
	SystemParemater.RomPar.UtHwSetValue.SetTime = 0;
	SystemParemater.RomPar.OvpHwSetValue.SetValue = 3800;
	SystemParemater.RomPar.OvpHwSetValue.SetTime = 0;
	SystemParemater.RomPar.UvpHwSetValue.SetValue = 2500;
	SystemParemater.RomPar.UvpHwSetValue.SetTime = 0;
	SystemParemater.RomPar.FullCharge.Current = 5250;
	SystemParemater.RomPar.FullCharge.Voltage = 3400;
	SystemParemater.RomPar.FullCharge.Time = 10;
	
	SystemParemater.RomPar.RelayOnThreshold = 100;
	SystemParemater.RomPar.PreDischargeTime = 20;

	SystemParemater.RomPar.InsulationResistance.ReadInterval = 10;
	SystemParemater.RomPar.InsulationResistance.SwDelay = 20;
	
	SystemParemater.RomPar.AfeComm.L1Time = 5;
	SystemParemater.RomPar.AfeComm.L2Time = 10;
	
	SystemParemater.RomPar.DesignedCapacity = 280000L;
	//SystemParemater.Qmax = 280000L;
	SystemParemater.RomPar.ZeroCurrent = 200;
	SystemParemater.RomPar.MinChargeDischargeCurrent = 300;
	SystemParemater.RomPar.BmuNumber = 2;
	SystemParemater.RomPar.RelayActiveFlag = 0;
	
	for(i=0; i<32; i++)
	{
		SystemParemater.RomPar.CellFlag[i] = 0xffff;
		SystemParemater.RomPar.NtcFlag[i] = 0xffff;		
	}

	setDefaultOvpValue();
	setDefaultUvpValue();
	setDefaultCotpValue();
    setDefaultCutpValue();
	setDefaultDotpValue();
	setDefaultDutpValue();
	setDefaultCocpValue();
	setDefaultDocpValue();
	setDefaultBalanceValue();
	setDefaultOcvTable();
	setDefaultRaTable();	
	
	setDefaultOvpPfValue();
	setDefaultUvpPfValue();
}
static void setSysParRamValue(void)
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

static void updateCellNumber(void)
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
static void updateNtcNumber(void)
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

/* Public function prototypes -----------------------------------------------*/
void apiSysParOvpPfClean(void)
{
	mBatteryInfo.PfFlag &= ~BATINFO_OVP_PF_FLAG;
	resetBatteryInfoIdleCount();
}
void apiSysParOvpPfSet(void)
{
	mBatteryInfo.PfFlag |= BATINFO_OVP_PF_FLAG;
	resetBatteryInfoIdleCount();
}
void apiSysParUvpPfClean(void)
{
	mBatteryInfo.PfFlag &= ~BATINFO_UVP_PF_FLAG;
	resetBatteryInfoIdleCount();
}
void apiSysParUvpPfSet(void)
{
	mBatteryInfo.PfFlag |= BATINFO_UVP_PF_FLAG;
	resetBatteryInfoIdleCount();
}

uint8_t apiSysParIsOvpPfSet(void)
{
	if(mBatteryInfo.PfFlag & BATINFO_OVP_PF_FLAG)
		return 1;
	return 0;
}

uint8_t apiSysParIsUvpPfSet(void)
{
	if(mBatteryInfo.PfFlag & BATINFO_UVP_PF_FLAG)
		return 1;
	return 0;
}


uint32_t apiSysParGetQmax(void)
{
	return mBatteryInfo.Qmax;
}

void apiSysParSetQmax(uint32_t Qmax)
{
	char	str[100];
	
	sprintf(str,"apiSysParSetQmax %d", Qmax);
	sysParDebugMsg(str);
	
	mBatteryInfo.Qmax = Qmax;
	resetBatteryInfoIdleCount();
}

uint16_t apiSysParGetQmaxUpdateTimes(void)
{
	return mBatteryInfo.QmaxUpdateTimes;
}

void apiSysParSetQmaxUpdateTimes(uint16_t times)
{
	sysParDebugMsg("apiSysParSetQmaxUpdateTimes");
	mBatteryInfo.QmaxUpdateTimes = times;
	resetBatteryInfoIdleCount();
}
uint16_t apiSysParGetCycleCount(void)
{
	return mBatteryInfo.CycleCount;
}

void apiSysParSetCycleCount(uint16_t count)
{
	sysParDebugMsg("apiSysParSetCycleCount");
	mBatteryInfo.CycleCount = count;
	resetBatteryInfoIdleCount();
}

uint16_t apiSysParGetPfFlag(void)
{
	return mBatteryInfo.PfFlag;
}

void apiSysParSetPfFlag(uint16_t flag)
{
	sysParDebugMsg("apiSysParSetPfFlag");

	mBatteryInfo.PfFlag = flag;
	resetBatteryInfoIdleCount();
}

int32_t appCurrDebug(uint8_t CurrentIndex, int32_t adc)
{
	return doCalibration(&SysCalPar.RamPar.Current[CurrentIndex], adc);
}
int32_t appVbatDebug(uint8_t VbatIndex, int32_t adc)
{
	return doCalibration(&SysCalPar.RamPar.VBat[VbatIndex], adc);
}

void apiCaliParSetCurrentValue(uint8_t CurrentIndex, uint8_t PointIndex,int32_t Value, int32_t Adc)
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
		setCalParRamValue();
		saveCalPar();
	}
}
void apiCaliParGetCurrentValue(uint8_t CurrentIndex, uint8_t PointIndex,int32_t *Value, int32_t *Adc)
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

void apiCaliParSetVbatValue(uint8_t VbatIndex, uint8_t PointIndex, int32_t Value, int32_t Adc)
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
		setCalParRamValue();
		saveCalPar();
	}
}
void apiCaliParGetVbatValue(uint8_t VbatIndex, uint8_t PointIndex,int32_t *Value, int32_t *Adc)
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

uint32_t apiCaliParGetChecksum(void)
{
	return SysCalPar.RomPar.Checksum;
}

//System parameter

//-----------------------------------------------------------


uint32_t apiSysParGetHwVersion(void)
{
	return SystemParemater.RomPar.HwVersion;
}
void apiSysParSetHwVersion(uint32_t version)
{
	SystemParemater.RomPar.HwVersion = version;
	resetSysParIdleCount();
}
#if 0
uint32_t apiSysParGetFwVersion(void)
{
	//SystemParemater.FwVersion = 0x010203;
	return 0;//SystemParemater.FwVersion;
}
#endif

uint8_t apiSysParGetBmuNumber(void)
{
	return SystemParemater.RomPar.BmuNumber;	
}
void apiSysParSetBmuNumber(uint8_t num)
{
	SystemParemater.RomPar.BmuNumber = num;
	resetSysParIdleCount();
}

uint32_t apiSysParGetCellFlag(uint8_t BmuIndex)
{
	return SystemParemater.RomPar.CellFlag[BmuIndex];	
}
void apiSysParSetCellFlag(uint8_t BmuIndex,uint32_t CellFlag)
{
	SystemParemater.RomPar.CellFlag[BmuIndex] = CellFlag;
	resetSysParIdleCount();
}

uint32_t apiSysParGetNtcFlag(uint8_t BmuIndex)
{
	return SystemParemater.RomPar.NtcFlag[BmuIndex];	
}

void apiSysParSetNtcFlag(uint8_t BmuIndex, uint32_t NtcFlag)
{
	SystemParemater.RomPar.NtcFlag[BmuIndex] = NtcFlag;
	resetSysParIdleCount();
}

uint16_t apiSysParGetZeroCurrentValue(void)
{
	return SystemParemater.RomPar.ZeroCurrent;	
}
void apiSysParSetZeroCurrentValue(uint16_t current)
{
	SystemParemater.RomPar.ZeroCurrent = current;
	resetSysParIdleCount();
}

uint16_t apiSysParGetMinChargeCurrentValue(void)
{
	return SystemParemater.RomPar.MinChargeDischargeCurrent;	
}

void apiSysParSetMinChargeCurrentValue(uint16_t current)
{
	SystemParemater.RomPar.MinChargeDischargeCurrent = current;	
	resetSysParIdleCount();
}

uint32_t apiSysParGetDesignedCapacity(void)
{
	return SystemParemater.RomPar.DesignedCapacity;
}

void apiSysParSetDesignedCapacity(uint32_t dc)
{
	SystemParemater.RomPar.DesignedCapacity = dc;
	resetSysParIdleCount();
}

uint16_t apiSysParGetMinFlatVoltage(void)
{
	return SystemParemater.RomPar.MinFlatVoltage;
}

uint16_t apiSysParGetMaxFlatVoltage(void)
{
	return SystemParemater.RomPar.MaxFlatVoltage;
}
void apiSysParGetFlatVoltage(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.MinFlatVoltage;
	pPar->STime.l = SystemParemater.RomPar.MaxFlatVoltage;
}

void apiSysParSetFlatVoltage(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.MinFlatVoltage = pPar->SetValue.l;
	SystemParemater.RomPar.MaxFlatVoltage = pPar->STime.l;
	resetSysParIdleCount();
}

void apiSysParGetFullChargeCondition(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.FullCharge.Current;
	pPar->STime.l = SystemParemater.RomPar.FullCharge.Voltage;
	pPar->RelValue.l = SystemParemater.RomPar.FullCharge.Time;
}
void apiSysParSetFullChargeCondition(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.FullCharge.Current = pPar->SetValue.l;
	SystemParemater.RomPar.FullCharge.Voltage = pPar->STime.l;
	SystemParemater.RomPar.FullCharge.Time = pPar->RelValue.l;
	resetSysParIdleCount();
}	
uint16_t apiSysParGetTerminateVoltage(void)
{
	return SystemParemater.RomPar.TerminateVoltage;
}
void apiSysParSetTerminateVoltage(uint16_t voltage)
{
	SystemParemater.RomPar.TerminateVoltage = voltage;
	resetSysParIdleCount();
}

void apiSysParGetAfeCommTime(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.AfeComm.L1Time;
	pPar->STime.l = SystemParemater.RomPar.AfeComm.L2Time;
}

void apiSysParSetAfeCommTime(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.AfeComm.L1Time = pPar->SetValue.l;
	SystemParemater.RomPar.AfeComm.L2Time = pPar->STime.l;
	resetSysParIdleCount();
}

void apiSysParGetInsulationResistance(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.InsulationResistance.ReadInterval;
	pPar->STime.l = SystemParemater.RomPar.InsulationResistance.SwDelay;
}

void apiSysParSetInsulationResistance(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.InsulationResistance.ReadInterval = pPar->SetValue.l;
	SystemParemater.RomPar.InsulationResistance.SwDelay = pPar->STime.l;
	resetSysParIdleCount();
}

uint16_t apiSysParGetPreDischargeTime(void)
{
	return SystemParemater.RomPar.PreDischargeTime;
}

void apiSysParSetPreDischargeTime(uint16_t time)
{
	SystemParemater.RomPar.PreDischargeTime = time;
	resetSysParIdleCount();
}
uint16_t apiSysParGetRelayOnDiffVoltage(void)
{
	return SystemParemater.RomPar.RelayOnThreshold;
}

void apiSysParSetRelayOnDiffVoltage(uint16_t voltage)
{
	SystemParemater.RomPar.RelayOnThreshold = voltage;
	resetSysParIdleCount();
}


uint16_t apiSysParGetCellNumber(void)
{
	return SystemParemater.RamPar.CellNumber;
}

uint16_t apiSysParGetNtcNumber(void)
{
	return SystemParemater.RamPar.NtcNumber;
}
//---------------------------------------
void apiSysParGetOcvTable(uint8_t index ,tOcvRaTable *pOcvTable)
{
	pOcvTable->Level =  SystemParemater.RomPar.OcvTable[index].Level;
	pOcvTable->Value =  SystemParemater.RomPar.OcvTable[index].Value;
}

void apiSysParGetRaTable(uint8_t index ,tOcvRaTable *pOcvTable)
{
	pOcvTable->Level =  SystemParemater.RomPar.RaTable[index].Level;
	pOcvTable->Value =  SystemParemater.RomPar.RaTable[index].Value;
}

void apiSysParSetOcvTable(uint8_t index ,tOcvRaTable *pOcvTable)
{
	SystemParemater.RomPar.OcvTable[index].Level = pOcvTable->Level;
	SystemParemater.RomPar.OcvTable[index].Value = pOcvTable->Value;
	resetSysParIdleCount();
}

void apiSysParSetRaTable(uint8_t index ,tOcvRaTable *pOcvTable)
{
	SystemParemater.RomPar.RaTable[index].Level = pOcvTable->Level;
	SystemParemater.RomPar.RaTable[index].Value = pOcvTable->Value;
	resetSysParIdleCount();
}

//---------------------------------------
//Protect parameter
//OVP
void apiSysParGetOvpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
	{
		if(level == 0x10)
		{
			pPar->SetValue.l = SystemParemater.RomPar.OvpHwSetValue.SetValue;
			pPar->STime.l = SystemParemater.RomPar.OvpHwSetValue.SetTime;
		}
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

void apiSysParSetOvpPar(uint8_t level, tScuProtectPar *pPar)
{	
	if(level >= 3)
	{
		if(level == 0x10)
		{
			SystemParemater.RomPar.OvpHwSetValue.SetValue = pPar->SetValue.l;
			SystemParemater.RomPar.OvpHwSetValue.SetTime = pPar->STime.l;
		}
		return;
	}
	SystemParemater.RomPar.Ovp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Ovp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Ovp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Ovp[level].ReleaseTime = pPar->RTime.l;
	resetSysParIdleCount();
}

// UVP

void apiSysParGetUvpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
	{
		if(level == 0x10)
		{
			pPar->SetValue.l = SystemParemater.RomPar.UvpHwSetValue.SetValue;		
			pPar->STime.l = SystemParemater.RomPar.UvpHwSetValue.SetTime;		
		}
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

void apiSysParSetUvpPar(uint8_t level, tScuProtectPar *pPar)
{	
	if(level >= 3)
	{
		if(level == 0x10)
		{
			SystemParemater.RomPar.UvpHwSetValue.SetValue = pPar->SetValue.l;
			SystemParemater.RomPar.UvpHwSetValue.SetTime = pPar->STime.l;		
		}
		return;
	}
	SystemParemater.RomPar.Uvp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Uvp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Uvp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Uvp[level].ReleaseTime = pPar->RTime.l;
	resetSysParIdleCount();
}

void apiSysParGet2ndOtProtectPar(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.OtHwSetValue.SetValue;
	pPar->STime.l = SystemParemater.RomPar.OtHwSetValue.SetTime;
}
void apiSysParSet2ndOtProtectPar(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.OtHwSetValue.SetValue = pPar->SetValue.l; 
	SystemParemater.RomPar.OtHwSetValue.SetTime = pPar->STime.l;
}
void apiSysParGet2ndUtProtectPar(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.UtHwSetValue.SetValue;
	pPar->STime.l = SystemParemater.RomPar.UtHwSetValue.SetTime;
}
void apiSysParSet2ndUtProtectPar(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.UtHwSetValue.SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.UtHwSetValue.SetTime = pPar->STime.l;
}

//---------------------------------------------------
//	Dotp
void apiSysParGetDotpProtectPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	pPar->SetValue.l = SystemParemater.RamPar.Dotp[level].SetAdcValue;
	pPar->STime.l = SystemParemater.RomPar.Dotp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RamPar.Dotp[level].ReleaseAdcValue;
	pPar->RTime.l = SystemParemater.RomPar.Dotp[level].ReleaseTime;
}


void apiSysParGetDotpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	pPar->SetValue.l = SystemParemater.RomPar.Dotp[level].SetValue;
	pPar->STime.l = SystemParemater.RomPar.Dotp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RomPar.Dotp[level].ReleaseValue;
	pPar->RTime.l = SystemParemater.RomPar.Dotp[level].ReleaseTime;
}
void apiSysParSetDotpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	SystemParemater.RomPar.Dotp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Dotp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Dotp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Dotp[level].ReleaseTime = pPar->RTime.l;	
	resetSysParIdleCount();
}
//----------- DUTP ---------------------------
void apiSysParGetDutpProtectPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	pPar->SetValue.l = SystemParemater.RamPar.Dutp[level].SetAdcValue;
	pPar->STime.l = SystemParemater.RomPar.Dutp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RamPar.Dutp[level].ReleaseAdcValue;
	pPar->RTime.l = SystemParemater.RomPar.Dutp[level].ReleaseTime;
}

void apiSysParGetDutpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	pPar->SetValue.l = SystemParemater.RomPar.Dutp[level].SetValue;
	pPar->STime.l = SystemParemater.RomPar.Dutp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RomPar.Dutp[level].ReleaseValue;
	pPar->RTime.l = SystemParemater.RomPar.Dutp[level].ReleaseTime;
}
void apiSysParSetDutpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	SystemParemater.RomPar.Dutp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Dutp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Dutp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Dutp[level].ReleaseTime = pPar->RTime.l;	
	resetSysParIdleCount();
}
//----------- DTP ---------------------------
void apiSysParGetDtpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	pPar->SetValue.l = SystemParemater.RomPar.Dtp[level].SetValue;
	pPar->STime.l = SystemParemater.RomPar.Dtp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RomPar.Dtp[level].ReleaseValue;
	pPar->RTime.l = SystemParemater.RomPar.Dtp[level].ReleaseTime;
}
void apiSysParSetDtpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	SystemParemater.RomPar.Dtp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Dtp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Dtp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Dtp[level].ReleaseTime = pPar->RTime.l;	
	resetSysParIdleCount();
}
//------------------------------------------------------
//	Cotp
void apiSysParGetCotpProtectPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	pPar->SetValue.l = SystemParemater.RamPar.Cotp[level].SetAdcValue;
	pPar->STime.l = SystemParemater.RomPar.Cotp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RamPar.Cotp[level].ReleaseAdcValue;
	pPar->RTime.l = SystemParemater.RomPar.Cotp[level].ReleaseTime;
}

void apiSysParGetCotpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
	{
		if(level == 0x10)
		{
			 pPar->SetValue.l = SystemParemater.RomPar.OtHwSetValue.SetValue;
			 pPar->STime.l = SystemParemater.RomPar.OtHwSetValue.SetTime;			
		}
		return;
	}
	pPar->SetValue.l = SystemParemater.RomPar.Cotp[level].SetValue;
	pPar->STime.l = SystemParemater.RomPar.Cotp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RomPar.Cotp[level].ReleaseValue;
	pPar->RTime.l = SystemParemater.RomPar.Cotp[level].ReleaseTime;
}
void apiSysParSetCotpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
	{
		if(level == 0x10)
		{
			SystemParemater.RomPar.OtHwSetValue.SetValue = pPar->SetValue.l;
			SystemParemater.RomPar.OtHwSetValue.SetTime = pPar->STime.l;
		}
		return;
	}
	SystemParemater.RomPar.Cotp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Cotp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Cotp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Cotp[level].ReleaseTime = pPar->RTime.l;
	resetSysParIdleCount();
}
	
//-----------------------------------------------	
//	Cutp
void apiSysParGetCutpProtectPar(uint8_t level, tScuProtectPar *pPar)
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
void apiSysParGetCutpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
	{
		if(level == 0x10)
		{
			pPar->SetValue.l = SystemParemater.RomPar.UtHwSetValue.SetValue;
			pPar->STime.l = SystemParemater.RomPar.UtHwSetValue.SetTime;
		}
		return;
	}
	pPar->SetValue.l = SystemParemater.RomPar.Cutp[level].SetValue;
	pPar->STime.l = SystemParemater.RomPar.Cutp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RomPar.Cutp[level].ReleaseValue;
	pPar->RTime.l = SystemParemater.RomPar.Cutp[level].ReleaseTime;
}
void apiSysParSetCutpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
	{
		if(level == 0x10)
		{
			SystemParemater.RomPar.UtHwSetValue.SetValue = pPar->SetValue.l;
			SystemParemater.RomPar.UtHwSetValue.SetTime = pPar->STime.l;			
		}
		return;
	}
	SystemParemater.RomPar.Cutp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Cutp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Cutp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Cutp[level].ReleaseTime = pPar->RTime.l;	
	resetSysParIdleCount();
}

//-----------------------------------------------------
//	Docp
void apiSysParGetDocpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	pPar->SetValue.l = SystemParemater.RomPar.Docp[level].SetValue;
	pPar->STime.l = SystemParemater.RomPar.Docp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RomPar.Docp[level].ReleaseValue;
	pPar->RTime.l = SystemParemater.RomPar.Docp[level].ReleaseTime;

}
void apiSysParSetDocpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	SystemParemater.RomPar.Docp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Docp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Docp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Docp[level].ReleaseTime = pPar->RTime.l;	
	resetSysParIdleCount();
}

//---------------------------------------------------
//Cocp
void apiSysParGetCocpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	pPar->SetValue.l = SystemParemater.RomPar.Cocp[level].SetValue;
	pPar->STime.l = SystemParemater.RomPar.Cocp[level].SetTime; 
	pPar->RelValue.l = SystemParemater.RomPar.Cocp[level].ReleaseValue;
	pPar->RTime.l = SystemParemater.RomPar.Cocp[level].ReleaseTime;

}
void apiSysParSetCocpPar(uint8_t level, tScuProtectPar *pPar)
{
	if(level >= 3)
		return;
	SystemParemater.RomPar.Cocp[level].SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.Cocp[level].SetTime = pPar->STime.l;
	SystemParemater.RomPar.Cocp[level].ReleaseValue = pPar->RelValue.l;
	SystemParemater.RomPar.Cocp[level].ReleaseTime = pPar->RTime.l;	
	resetSysParIdleCount();
}
//-----------------------------------------
//	PF
void apiSysParGetOvpPfPar(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.OvpPf.SetValue;
	pPar->STime.l = SystemParemater.RomPar.OvpPf.SetTime; 
}
void apiSysParSetOvpPfPar(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.OvpPf.SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.OvpPf.SetTime = pPar->STime.l;
	resetSysParIdleCount();
}

void apiSysParGetUvpPfPar(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.UvpPf.SetValue;
	pPar->STime.l = SystemParemater.RomPar.UvpPf.SetTime; 
}
void apiSysParSetUvpPfPar(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.UvpPf.SetValue = pPar->SetValue.l;
	SystemParemater.RomPar.UvpPf.SetTime = pPar->STime.l;
	resetSysParIdleCount();
}

//------------------------------------------
void apiSysParGetBalanceDuty(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.Balance.DutySet;
	pPar->STime.l = SystemParemater.RomPar.Balance.DutyRest;
	pPar->RelValue.l = SystemParemater.RomPar.Balance.TempSet;
	pPar->RTime.l = SystemParemater.RomPar.Balance.TempRelease;
}
void apiSysParSetBalanceDuty(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.Balance.DutySet = pPar->SetValue.l;
	SystemParemater.RomPar.Balance.DutyRest = pPar->STime.l;
	SystemParemater.RomPar.Balance.TempSet = pPar->RelValue.l;
	SystemParemater.RomPar.Balance.TempRelease = pPar->RTime.l;
	resetSysParIdleCount();
}
void apiSysParGetBalanceChg(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.Balance.ChgSet;
	pPar->STime.l = SystemParemater.RomPar.Balance.ChgRelease;
	pPar->RelValue.l = SystemParemater.RomPar.Balance.ChgDeltaSet;
	pPar->RTime.l = SystemParemater.RomPar.Balance.ChgDeltaRelease;
}
void apiSysParSetBalanceChg(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.Balance.ChgSet = pPar->SetValue.l;
	SystemParemater.RomPar.Balance.ChgRelease = pPar->STime.l;
	SystemParemater.RomPar.Balance.ChgDeltaSet = pPar->RelValue.l;
	SystemParemater.RomPar.Balance.ChgDeltaRelease = pPar->RTime.l;
	resetSysParIdleCount();
}

void apiSysParGetBalanceDhg(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.Balance.DhgSet;
	pPar->STime.l = SystemParemater.RomPar.Balance.DhgRelease;
	pPar->RelValue.l = SystemParemater.RomPar.Balance.DhgDeltaSet;
	pPar->RTime.l = SystemParemater.RomPar.Balance.DhgDeltaRelease;
}
void apiSysParSetBalanceDhg(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.Balance.DhgSet = pPar->SetValue.l;
	SystemParemater.RomPar.Balance.DhgRelease = pPar->STime.l;
	SystemParemater.RomPar.Balance.DhgDeltaSet = pPar->RelValue.l;
	SystemParemater.RomPar.Balance.DhgDeltaRelease = pPar->RTime.l;
	resetSysParIdleCount();
}
void apiSysParGetBalanceRlx(tScuProtectPar *pPar)
{
	pPar->SetValue.l = SystemParemater.RomPar.Balance.RlxSet;
	pPar->STime.l = SystemParemater.RomPar.Balance.RlxRelease;
	pPar->RelValue.l = SystemParemater.RomPar.Balance.RlxDeltaSet;
	pPar->RTime.l = SystemParemater.RomPar.Balance.RlxDeltaRelease;
}
void apiSysParSetBalanceRlx(tScuProtectPar *pPar)
{
	SystemParemater.RomPar.Balance.RlxSet = pPar->SetValue.l;
	SystemParemater.RomPar.Balance.RlxRelease = pPar->STime.l;
	SystemParemater.RomPar.Balance.RlxDeltaSet = pPar->RelValue.l;
	SystemParemater.RomPar.Balance.RlxDeltaRelease = pPar->RTime.l;
	resetSysParIdleCount();
}
//----------------------------------------------------------
//	Note Message
void apiSysParGetNotwMessageString(uint8_t *pMsg)
{
	uint8_t	i;
	for(i=0; i<MAX_NOTE_MESSAGE_STRING_ITEM; i++)
	{
		pMsg[i] = SystemParemater.RomPar.NoteMessage[i];
		if(pMsg[i] == 0)
			break;
	}
}
void apiSysParSetNotwMessageString(uint8_t *pMsg)
{
	uint8_t	i;
	for(i=0; i<MAX_NOTE_MESSAGE_STRING_ITEM; i++)
	{
		SystemParemater.RomPar.NoteMessage[i] = pMsg[i];
		if(pMsg[i] == 0)
			break;
	}
	resetSysParIdleCount();
}
uint32_t apiSysParGetChecksum(void)
{
	return SystemParemater.RomPar.Checksum;
}
//----------------------------------------------------------------

uint16_t apiSysParOpen(void)
{
	setBatteryInfoDefaultValue();
	loadBatteryInfoPar();
	
	SystemParIdleCount = 0;
	
	sysParSetDefaultRomValue();
	loadSysPar();
	loadCalPar();
	loadScuIdPar();
	setCalParRamValue();
	
	updateCellNumber();
	updateNtcNumber();
	setSysParRamValue();
	return sizeof(tSysRomPar);
	
}


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



/**
  ******************************************************************************
  * @file        ApiRamData.c
  * @author      Johnny
  * @version     v0.0.0
  * @date        2021/11/03
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
#include "define.h"
#include "main.h"
#include "LibDebug.h"
#include "halRtc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define	RTC_MAGIC_CODE_RAM_ADDR					RTC_BKP_DR0

#define	SYSTEM_SOC_RAM_ADDR						RTC_BKP_DR1

#define	LAST_CHG_DHG_TIME_RAM_ADDR				RTC_BKP_DR2
#define	LAST_CHG_DHG_TIME_CHECKSUM_RAM_ADDR		RTC_BKP_DR3

#define	LAST_RTC_TIME_RAM_ADDR					RTC_BKP_DR4
#define	LAST_RTC_TIME_CHECKSUM_RAM_ADDR			RTC_BKP_DR5

#define	TOTAL_DISCHARGE_COUNT_RAM_ADDR			RTC_BKP_DR6
#define	TOTAL_DISCHARGE_CHECKSUM_RAM_ADDR		RTC_BKP_DR7


/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static RTC_HandleTypeDef RtcHandle;

/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
void apiRamSaveRtcMagicCode(uint16_t MagicCode)
{
	HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_MAGIC_CODE_RAM_ADDR, MagicCode);
}
uint16_t apiRamLoadRtcMagicCode(void)
{
	return HAL_RTCEx_BKUPRead(&RtcHandle, RTC_MAGIC_CODE_RAM_ADDR);
}

#define	SOC_CHECKSUM	0xAA55
void apiRamSaveSoc(uint16_t soc)
{
	tLbyte	RamData;
	
	RamData.i[0] = soc;
	RamData.i[1] = RamData.i[0] ^ SOC_CHECKSUM;
	HAL_RTCEx_BKUPWrite(&RtcHandle, SYSTEM_SOC_RAM_ADDR, RamData.l);
}

uint16_t apiRamLoadSoc(void)
{	
	tLbyte	RamData;
	uint16_t	checksum;
	
	RamData.l = HAL_RTCEx_BKUPRead(&RtcHandle, SYSTEM_SOC_RAM_ADDR);
	checksum = RamData.i[1];
	checksum ^= RamData.i[0];
	if(checksum == SOC_CHECKSUM && RamData.i[0] <= 10000)
		return RamData.i[0];
	return 0xffff;
}
//---------------------------------
#define	TOTAL_DHG_COUNT_CHECKSUM	0x937A
void apiRamSaveTotalDisChargeCount(uint32_t count)
{
	uint32_t	checksum;
	checksum = count ^ TOTAL_DHG_COUNT_CHECKSUM;
	HAL_RTCEx_BKUPWrite(&RtcHandle, TOTAL_DISCHARGE_COUNT_RAM_ADDR, count);
	HAL_RTCEx_BKUPWrite(&RtcHandle, TOTAL_DISCHARGE_CHECKSUM_RAM_ADDR, checksum);
}
uint32_t apiRamLoadTotalDisChargeCount(void)
{
	uint32_t	count, checksum;
	
	count = HAL_RTCEx_BKUPRead(&RtcHandle, TOTAL_DISCHARGE_COUNT_RAM_ADDR);
	checksum = HAL_RTCEx_BKUPRead(&RtcHandle, TOTAL_DISCHARGE_CHECKSUM_RAM_ADDR);

	checksum ^= count; 
	if(checksum == TOTAL_DHG_COUNT_CHECKSUM)
		return count;
	return 0;
}
//---------------------------------
#if	0
#define	_RAM_ACC_POWER_CHECKSUM		0xA862
const WORD	Ram_AccAddrTable[][4]={
		{ACC1_CHG_POWER_LH,ACC1_CHG_POWER_LH,ACC1_CHG_POWER_HL,ACC1_CHG_POWER_HH},
		{ACC1_DHG_POWER_LH,ACC1_DHG_POWER_LH,ACC1_DHG_POWER_HL,ACC1_DHG_POWER_HH},
		{ACC2_CHG_POWER_LH,ACC2_CHG_POWER_LH,ACC2_CHG_POWER_HL,ACC2_CHG_POWER_HH},
		{ACC2_DHG_POWER_LH,ACC2_DHG_POWER_LH,ACC2_DHG_POWER_HL,ACC2_DHG_POWER_HH},
	};	
#endif
void ApiRamSaveAccPower(void)
{
#if	0	
	WORD	checksum,value;	
	BYTE	area,i,addrindex;

	checksum=_RAM_ACC_POWER_CHECKSUM;
	addrindex=0;
	for(area=0;area<2;area++)
	{
		for(i=1;i<4;i++)
		{
			value=BatteryCapInfo[area].AccChgPower.i[i];
			checksum^=value;
			BKP_WriteBackupRegister(Ram_AccAddrTable[addrindex][i],value);
		}
		addrindex++;

		for(i=1;i<4;i++)
		{
			value=BatteryCapInfo[area].AccDhgPower.i[i];
			checksum^=value;
			BKP_WriteBackupRegister(Ram_AccAddrTable[addrindex][i],value);
		}
		addrindex++;
	}	
	BKP_WriteBackupRegister(ACC2_DHG_POWER_CHK,checksum);
#endif	
}
void ApiRamLoadAccPower(void)
{
#if	0	
	char	str[100];
	WORD	checksum,value;	
	BYTE	area,i,addrindex;

	checksum=BKP_ReadBackupRegister(ACC2_DHG_POWER_CHK);
	addrindex=0;
	for(area=0;area<2;area++)
	{
		BatteryCapInfo[area].AccChgPower.i[0]=0;
		for(i=1;i<4;i++)
		{
			value=BKP_ReadBackupRegister(Ram_AccAddrTable[addrindex][i]);
			BatteryCapInfo[area].AccChgPower.i[i]=value;
			checksum^=value;
		}
		addrindex++;
		//-------------------------------------
		BatteryCapInfo[area].AccDhgPower.i[0]=0;
		for(i=1;i<4;i++)
		{
			value=BKP_ReadBackupRegister(Ram_AccAddrTable[addrindex][i]);
			BatteryCapInfo[area].AccDhgPower.i[i]=value;
			checksum^=value;
		}
		addrindex++;
	}
	if(checksum!=_RAM_ACC_POWER_CHECKSUM)
	{
		for(area=0;area<SystemParameter.BmuNumber;area++)
		{
			BatteryCapInfo[area].AccChgPower.ll=0;
			BatteryCapInfo[area].AccDhgPower.ll=0;
		}
		SendMessageToCan("Load Acc Ram Error!!");	
	}
#if	1	
	else
	{
		for(area=0;area<2;area++)
		{	
			sprintf(str,"Load AccChg=%d-%.8lX %.8lX",area,
				BatteryCapInfo[area].AccChgPower.l[1],
				BatteryCapInfo[area].AccChgPower.l[0]);
			SendMessageToCan((BYTE *)str);

			sprintf(str,"Load AccDhg=%d-%.8lX %.8lX",area,
				BatteryCapInfo[area].AccDhgPower.l[1],
				BatteryCapInfo[area].AccDhgPower.l[0]);
			SendMessageToCan((BYTE *)str);

		}
	}
#endif	
#endif
}

#define	LAST_RTC_TIME_CHECKSUM	0x551384AAL
void apiRamSaveRtcDateTime(void)
{
	uint32_t	sec,checksum;
	sec = HalRtcGetSmpUnixTime();
	checksum = sec ^ LAST_RTC_TIME_CHECKSUM;
	HAL_RTCEx_BKUPWrite(&RtcHandle, LAST_RTC_TIME_RAM_ADDR, sec);
	HAL_RTCEx_BKUPWrite(&RtcHandle, LAST_RTC_TIME_CHECKSUM_RAM_ADDR, checksum);
}
uint32_t apiRamLoadPowerOffDateTime(void)
{
	uint32_t	sec,checksum;
	
	sec = HAL_RTCEx_BKUPRead(&RtcHandle, LAST_RTC_TIME_RAM_ADDR);
	checksum = HAL_RTCEx_BKUPRead(&RtcHandle, LAST_RTC_TIME_CHECKSUM_RAM_ADDR);
	checksum ^= sec; 
	if(checksum == LAST_RTC_TIME_CHECKSUM)
		return sec;
	return 0xffffffffL;
}
#define	LAST_CHG_DHG_CHECKSUM	0xAA551384L
void apiRamSaveLastChgDhgTime(void)
{
	uint32_t	sec,checksum;
	sec = HalRtcGetSmpUnixTime();
	checksum = sec ^ LAST_CHG_DHG_CHECKSUM;
	HAL_RTCEx_BKUPWrite(&RtcHandle, LAST_CHG_DHG_TIME_RAM_ADDR, sec);
	HAL_RTCEx_BKUPWrite(&RtcHandle, LAST_CHG_DHG_TIME_CHECKSUM_RAM_ADDR, checksum);
}	

uint32_t apiRamLoadReleaseTime(void)
{
	uint32_t	sec,checksum,rtc;
	
	sec = HAL_RTCEx_BKUPRead(&RtcHandle, LAST_CHG_DHG_TIME_RAM_ADDR);
	checksum = HAL_RTCEx_BKUPRead(&RtcHandle, LAST_CHG_DHG_TIME_CHECKSUM_RAM_ADDR);

	checksum ^= sec;

	if(checksum == LAST_CHG_DHG_CHECKSUM)
	{
		rtc = HalRtcGetSmpUnixTime();
		if(rtc >= sec)
		{
			rtc -= sec;
			return rtc;
		}
	}
	return 0;
}

void apiRamOpen(void)
{
  	RtcHandle.Instance  = RTC;
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



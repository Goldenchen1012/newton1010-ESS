/**
  ******************************************************************************
  * @file        ApiEventFlashRom.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/12/01
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
#include "LibSwTimer.h"
#include "ApiEventLog.h"
#include "HalEeprom.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	eventFlashRomDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private define ------------------------------------------------------------*/
#define	EVENT_LOG_INI_CHECKSUM			0xA5
#define	FLASHROM_BASE_ADDR				0x08000000L
#define	FLASHROM_EVENT_LOG_START_ADDR	(FLASHROM_BASE_ADDR + (400 * 1024L))		//0x64000
#define	FLASHROM_EVENT_LOG_END_ADDR		(FLASHROM_BASE_ADDR + (500 * 1024L))		//0x7D000

enum{
	LOG_ADDR_INVALID = 0,
	LOG_ADDR_CHECKING,
	LOG_ADDR_VALID
};

/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint32_t	LastAddress;
	uint8_t		EventTempBuf[256];
	uint8_t		EventTempIndex;
	uint8_t		RecordAddressValid;
}tEventLog;
static tEventLog	mEventLog = {0};
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static 	tApiEventLogCallbackFunction eventCbFunction;
static 	uint8_t		*pEventLogDataBuf;
/* Public variables ---------------------------------------------------------*/
/* Private function prototypes ----------------------------------------------*/
uint8_t calEventLogCheckSum(uint8_t *pBuf)
{
	uint8_t		checksum,j;
	
	checksum = 0;
	for(j=0; j<8; j++)
		checksum ^= pBuf[j];
	return checksum;
}

static uint8_t isValidEventLog(uint8_t *pBuf)
{
	if(calEventLogCheckSum(pBuf) == EVENT_LOG_INI_CHECKSUM)
		return 1;
	return 0;
}
static void searchLastRecordAddress(void)
{
	tHalEeProm	mHalEeProm;
	uint8_t		buf[256];
	char		str[100];
	uint8_t		i;
	
//	sprintf(str,"Search Log Addr= %.8lX",  mEventLog.LastAddress);
//	eventFlashRomDebugMsg(str);
		
	mHalEeProm.StartAddress = mEventLog.LastAddress;
	mHalEeProm.Length = 8;
	mHalEeProm.pDataBuffer = buf;
	HalEePromRead(&mHalEeProm);
	if(isValidEventLog(buf) == 1)
	{
		mEventLog.RecordAddressValid = LOG_ADDR_CHECKING;
		mHalEeProm.StartAddress = mEventLog.LastAddress;
		mHalEeProm.Length = 256;
		mHalEeProm.pDataBuffer = buf;
		HalEePromRead(&mHalEeProm);
		i = 256 - 8;
		while(1)
		{
			if(isValidEventLog(&buf[i]) == 1)
			{
				mEventLog.LastAddress += (i+8);
				sprintf(str,"Last Addressr 1= %.8lX",  mEventLog.LastAddress);
				eventFlashRomDebugMsg(str);
				break;
			}
			if(i == 0)
				break;
			i -= 8;
		}
	}
	else
	{
		if(mEventLog.LastAddress <= FLASHROM_EVENT_LOG_START_ADDR)
		{
			mEventLog.LastAddress = FLASHROM_EVENT_LOG_START_ADDR;
			mEventLog.RecordAddressValid = LOG_ADDR_CHECKING;			
		}
		else
		{
			mEventLog.LastAddress -= 256;
		}
	}
}
static uint8_t isFinishWriteEventLogTempData(void)
{
	if(mEventLog.EventTempIndex == 0)
		return 1;
	return 0;
}

static void searchLastLogAddressSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{	
	char str[100];
	
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
		if(mEventLog.RecordAddressValid == LOG_ADDR_INVALID)
		 	searchLastRecordAddress();
	} 	
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_8)
	{
		//if(mEventLog.RecordAddressValid == LOG_ADDR_INVALID)
		// 	searchLastRecordAddress();
		//else 
		if(mEventLog.RecordAddressValid == LOG_ADDR_CHECKING)
		{
			if(isFinishWriteEventLogTempData())
			{
				LibSwTimerOpen(searchLastLogAddressSwTimerHandler, 0);
				mEventLog.RecordAddressValid = LOG_ADDR_VALID;
				sprintf(str,"Last Addressr 2= %.8lX",  mEventLog.LastAddress);
				eventFlashRomDebugMsg(str);
				eventFlashRomDebugMsg("Finish temp log data save....");
			}
		}		
	}
}
/* Public function prototypes -----------------------------------------------*/
uint32_t apiEventLogGetLogNumber(void)
{
	uint32_t 	num;
	num = mEventLog.LastAddress - FLASHROM_EVENT_LOG_START_ADDR;
	num /= 8;
	return num;
}
void apiEventLogReadLogData(uint32_t EvetIdex, uint8_t ReadNumber, tApiEventLogCallbackFunction CbFunction)
{
	uint32_t	EventNum;
	tHalEeProm	mHalEeProm;
	
	if(!CbFunction)
		return;
		
	if(ReadNumber > 16)
		ReadNumber = 16;
	EventNum = apiEventLogGetLogNumber();
	if(EvetIdex >= EventNum)
	{
		CbFunction(0, 0);
		return;
	}
	if((EvetIdex + ReadNumber) >= EventNum)
	{
		ReadNumber = EventNum - EvetIdex;	
	}
	if(ReadNumber == 0)
	{
		CbFunction(0, 0);
		return;
	}
	
	mHalEeProm.StartAddress = FLASHROM_EVENT_LOG_START_ADDR + EvetIdex * 8;
	mHalEeProm.Length = ReadNumber * 8;
	mHalEeProm.pDataBuffer = mEventLog.EventTempBuf;
	HalEePromRead(&mHalEeProm);
	
	CbFunction(ReadNumber, mEventLog.EventTempBuf);
	
	
//	eventCbFunction = CbFunction;
//	pEventLogDataBuf = pEventBuf;
}

void apiEventLogSaveLogData(uint8_t EventType, uint16_t Par)
{
	tErrCode	status;
	char	str[100];
	uint8_t		buf[10];
	tHalEeProm	mHalEeProm;
	uint8_t	checksum,i;
	tLbyte	Lbyte;
	
	Lbyte.l = HalRtcGetSmpUnixTime();

	buf[0] = EventType;
	buf[1] = Lbyte.b[0];	//RTC
	buf[2] = Lbyte.b[1];
	buf[3] = Lbyte.b[2];
	buf[4] = Lbyte.b[3];
	buf[5] = Par % 0x100;
	buf[6] = Par / 0x100;
	checksum = EVENT_LOG_INI_CHECKSUM;
	for(i=0; i<7; i++)
		checksum ^= buf[i];
	buf[7] = checksum;
	
	if(mEventLog.RecordAddressValid != LOG_ADDR_VALID)
	{
		eventFlashRomDebugMsg("Can't Save Event Log");
		return;	
	}
	if((mEventLog.LastAddress & 0x7ff) == 0)
	{
		mHalEeProm.StartAddress = mEventLog.LastAddress;
		mHalEeProm.Length = 8;
		mHalEeProm.pDataBuffer = buf;
		HalEePromErase(&mHalEeProm);
	}

	mHalEeProm.StartAddress = mEventLog.LastAddress;
	mHalEeProm.Length = 8;
	mHalEeProm.pDataBuffer = buf;
	status = HalEePromWrite(&mHalEeProm);
	mEventLog.LastAddress += 8;	
	sprintf(str,"Save Event Log:%.8lX %d ", mHalEeProm.StartAddress, status);
	eventFlashRomDebugMsg(str);
}
void apiEventLogOpen(void)
{
	mEventLog.RecordAddressValid = LOG_ADDR_INVALID;
	mEventLog.LastAddress = FLASHROM_EVENT_LOG_END_ADDR - 256;
	LibSwTimerOpen(searchLastLogAddressSwTimerHandler, 0);
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    




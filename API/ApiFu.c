/**
  ******************************************************************************
  * @file        ApiFu.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/26
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
#include "ApiFu.h"
#include "HalEeprom.h"
#include "AppSerialCanDavinci.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	fuDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private define ------------------------------------------------------------*/
#define		WAIT_FW_CHECK_TIME_SEC		20

static const BYTE	MagicCodeTable[][16]={
			"From APP To ISP",
			"FRoM IsP tO ApP",
			"fRom ApP ResET ",
			"fROM iSP ReSeT ",
			"UpdateFirmware1",
			"uPDatefIRmwaRE2",
			"               "};
		
typedef  void (*pFunction)(void);
	pFunction 	Jump_To_Application;
	DWORD		JumpAddress;
	DWORD		ApplicationAddress;
	


/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
const uint8_t	SmpFwHeadInfo1[]="SmpHvEssFirmware";
const uint8_t	SmpFwHeadInfo2[]="SMPhvessFirmWare";

static struct{
	uint8_t		FwHeadInfo1[16];
	tLbyte		Version;
	tLbyte		Length;
	tLbyte		BuildDate;
	tLbyte		BuildTime;
	tLbyte		CheckSum;
	tLbyte		Reserved[3];
	uint8_t		FwHeadInfo2[16];
	uint8_t		ProjectId[16];
	uint8_t		Rev[10];
}FwInformation __ALIGNED(4); 
/* Private variables ---------------------------------------------------------*/
static __IO uint8_t	MagicCodeBuf[24]  __attribute__((at(0x20000000)));
static __IO uint32_t	TestCount  __attribute__((at(0x2000001C)));

static uint8_t		updatFwSwTimerRunCount10ms = 0;
static	uint8_t		WaitFwCheckTime = 0;

/* Private function prototypes -----------------------------------------------*/
static void updatFwSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	uint8_t		FwStatus;
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
		;
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_8)
	{
		FwStatus = apiFuGetFwCheckStatus();
		if(FwStatus == API_FU_FW_CHECK_SUCCESS)
		{
			if(updatFwSwTimerRunCount10ms)
				updatFwSwTimerRunCount10ms--;
			if(updatFwSwTimerRunCount10ms == 0)
			{
				LibSwTimerClose(updatFwSwTimerHandler, 0);	
				apiFuSetMagicCode(MAGIC_CODE_UPDATE_FW1);
				apiFuJumpToBootloader();
			}
		}
		else if(FwStatus == API_FU_FW_CHECK_FAIL)
		{
			LibSwTimerClose(updatFwSwTimerHandler, 0);
			fuDebugMsg("Fw Check Fail SwTimer Close");
		}
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		if(WaitFwCheckTime)
		{
			WaitFwCheckTime--;
			if(!WaitFwCheckTime)
			{
				LibSwTimerClose(updatFwSwTimerHandler, 0);
				fuDebugMsg("updatFwSwTimerHandler Close");
			}
		}
	}
}

static void resetAndUpdateSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	static uint8_t	count =0;
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
		;
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_8)
	{
		count++;
		if(count >= 200)
		{
			apiFuSetMagicCode(MAGIC_CODE_UPDATE_FW1);
			apiFuJumpToBootloader();
		}
	}
}

static void resetAppSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	static uint8_t	count =0;
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
		;
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_8)
	{
		count++;
		if(count >= 100)
		{
			apiFuSetMagicCode(MAGIC_CODE_FROM_APP_RESET);
			apiFuJumpToBootloader();
		}
	}
}


/* Public function prototypes -----------------------------------------------*/
uint8_t apiFuGetMagicModeIndex(void)
{
	BYTE	c;
	BYTE	i;
	//--------------------------
	if(MagicCodeBuf[0]==0xAA && MagicCodeBuf[1]==0x55)
	{
		for(c=0; c<MAGIC_CODE_APP_NORMAL; c++)
		{
			for(i=0;i<15;i++)
			{
				if(MagicCodeBuf[2+i] != MagicCodeTable[c][i])
					break;
			}
			if(i>=15)
			{
				return c;
			}
		}
	}
	
	return MAGIC_CODE_APP_NORMAL;	
}

void apiFuSetMagicCode(uint8_t type)
{
	BYTE	i;
	if(type >= MAGIC_CODE_APP_NORMAL)
		type = MAGIC_CODE_APP_NORMAL;
	for(i=0;i<15;i++)
		MagicCodeBuf[2+i] = MagicCodeTable[type][i];
	MagicCodeBuf[0] = 0xAA;
	MagicCodeBuf[1] = 0x55;
}
void DumpBuffer(uint8_t *pBuf,uint16_t len);

void apiFuCheckMagicCode(void)
{
#if defined (USE_BOOTLOADER)
	tErrCode		result;
	tHalEeProm		mHalEeProm;
	uint8_t			index;
	char	str[100];
	
	index = apiFuGetMagicModeIndex();
	switch(index)
	{	
	case MAGIC_CODE_FROM_APP:
		fuDebugMsg("MAGIC_CODE_FROM_APP");
		break;
	case MAGIC_CODE_FROM_ISP:
		fuDebugMsg("MAGIC_CODE_FROM_ISP");
		break;
	case MAGIC_CODE_FROM_APP_RESET:
		fuDebugMsg("MAGIC_CODE_FROM_APP_RESET");
		break;
	case MAGIC_CODE_FROM_ISP_RESET:
		fuDebugMsg("MAGIC_CODE_FROM_ISP_RESET");
		break;
	case MAGIC_CODE_UPDATE_FW1:
		fuDebugMsg("MAGIC_CODE_UPDATE_FW1");
		break;
	case MAGIC_CODE_UPDATE_FW2:
		fuDebugMsg("MAGIC_CODE_UPDATE_FW2");
		break;
	case MAGIC_CODE_APP_NORMAL:
		fuDebugMsg("MAGIC_CODE_APP_NORMAL");
		break;
	}	
	apiFuSetMagicCode(MAGIC_CODE_APP_NORMAL);
	fuDebugMsg("Read Fw Info Error..0");
//	return;
	mHalEeProm.StartAddress = 0x08004000;
	mHalEeProm.Length = 80;
	mHalEeProm.pDataBuffer = (uint8_t *)&FwInformation;
	result = HalEePromRead(&mHalEeProm);
	DumpBuffer((uint8_t *)&FwInformation, 80);
		fuDebugMsg(FwInformation.FwHeadInfo1);
		fuDebugMsg(FwInformation.FwHeadInfo2);
		
	sprintf(str,"Ver=%X",FwInformation.Version.l);
	fuDebugMsg(str);

	sprintf(str,"Date=%X",FwInformation.BuildDate.l);
	fuDebugMsg(str);

	sprintf(str,"Time=%X",FwInformation.BuildTime.l);
	fuDebugMsg(str);

	if(result == RES_SUCCESS)
	{		
		if(memcmp(&FwInformation.FwHeadInfo1, &SmpFwHeadInfo1, 16) != 0 ||
		   memcmp(&FwInformation.FwHeadInfo2, &SmpFwHeadInfo2, 16) != 0)
		{
			memset(&FwInformation, 0, sizeof(FwInformation));
			fuDebugMsg("Read Fw Info Error");
		}
	}
	else
	{
		memset(&FwInformation, 0, sizeof(FwInformation));
		fuDebugMsg("Read Fw Info Error..2");
	} 
	
#endif	
}

uint32_t apiFuGetFwVersion(void)
{
	return FwInformation.Version.l;
}
uint32_t apiFuGetFwBuildDate(void)
{
	return FwInformation.BuildDate.l;
	
}
uint32_t apiFuGetFwBuildTime(void)
{
	return FwInformation.BuildTime.l;
}
uint32_t apiFuGetFwChecksum(void)
{
	return FwInformation.CheckSum.l;
}

//----------------------------------------------------------------
void apiFuJumpToBootloader(void)
{
//#if defined (USE_BOOTLOADER)
#if 1
	

	pFunction Jump_To_Application;
	DWORD	JumpAddress;
	DWORD	ApplicationAddress;
	
	__disable_irq();
	
	NVIC->ICER[0]=0xffffffff;
	NVIC->ICER[1]=0xffffffff;
	NVIC->ICER[2]=0xffffffff;
		
	ApplicationAddress = 0x8000000L;
	JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);
	Jump_To_Application = (pFunction) JumpAddress;
	/* Initialize user application's Stack Pointer */
#if 0	//use sw reset !!	
	//__set_MSP(*(__IO uint32_t*) ApplicationAddress);
	//SCB->VTOR = FLASH_BASE | 0;//VECT_TAB_OFFSET;
	//Jump_To_Application();
#endif	
	HAL_NVIC_SystemReset();	
	while(1);
#endif	
}

void apiFuUpdateFw(void)
{
	tErrCode status;
	
	status = LibSwTimerOpen(updatFwSwTimerHandler, 0);
	if(status == RES_SUCCESS)
	{
		updatFwSwTimerRunCount10ms = 100;
		WaitFwCheckTime = WAIT_FW_CHECK_TIME_SEC;
		fuDebugMsg("Sw Timer Open Success");
	}
	else
		fuDebugMsg("Sw Timer Open fail");
}

void apiFuResetAndUpdate(void)
{
	LibSwTimerOpen(resetAndUpdateSwTimerHandler, 0);
}
void apiFuResetApp(void)
{
	
	if( LibSwTimerOpen(resetAppSwTimerHandler, 0) == RES_SUCCESS)
	{
		char	str[100];
		TestCount++;
		sprintf(str,"Reset Count = %d",TestCount);
		fuDebugMsg(str);
	}
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    




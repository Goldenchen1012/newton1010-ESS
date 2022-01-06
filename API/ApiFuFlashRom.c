/**
  ******************************************************************************
  * @file        ApiFuFlashRom.c
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
#include <string.h>
#include "define.h"
#include "main.h"
#include "LibSwTimer.h"
#include "ApiFu.h"
#include "HalEeprom.h"


void appSerialCanDavinciSendTextMessage(char *str);
#define	fuDebugMsg(str)		appSerialCanDavinciSendTextMessage(str);

/* Private define ------------------------------------------------------------*/
#define	FLASHROM_BASE_ADDR				0x08000000L
#define	FLASHROM_FU_START_ADDR			(FLASHROM_BASE_ADDR + (256 * 1024L))

#define	FW_UPDATE_STATUS_INITIAL		0x01
#define	FW_UPDATE_STATUS_VERSION		0x02
#define	FW_UPDATE_STATUS_PACKAGENUM		0x04
#define	FW_UPDATE_STATUS_BASEADDR		0x08

/* Private typedef -----------------------------------------------------------*/
#define	FW_CODE_BUF_ITEM	2


typedef struct{
	tLbyte		UpdateAddress[FW_CODE_BUF_ITEM];
	DWORD		UpdateAddressTemp;
	DWORD		BaseAddress;
	WORD		FwPackageNum;
	WORD		FwPackageNumCnt;
	tLbyte		FwVersion;
	tLbyte		CodeBufStatus[FW_CODE_BUF_ITEM];
	BYTE		FwBufIndexSptr;
	BYTE		FwBufIndexEptr;
	BYTE		InfoStatus;
	uint8_t		CodeBuf[FW_CODE_BUF_ITEM][256] ;//__ALIGNED(4);
	uint8_t		FwCheckStatus;
	tApiFuCallbackFunction CbFunction;
	struct{
		uint16_t	PackageNumCnt;
		uint32_t	CheckSum;
	}FwCheck;
}tFwUpdateInfo;

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static tFwUpdateInfo	FwUpdateInfo;
/* Private function prototypes -----------------------------------------------*/
static uint8_t isValidInfoStatus(void)
{
	if((FwUpdateInfo.InfoStatus&0x0f) != 0x0f)
		return 0;
	return 1;
}

static void writeCodeDataToFlashRom(uint32_t addr, uint8_t *pDatBuf, uint16_t leng)
{
	tHalEeProm		mHalEeProm;
	char			str[100];
//	uint32_t		d;
	
	mHalEeProm.StartAddress = FLASHROM_FU_START_ADDR + addr;
	mHalEeProm.Length = leng;
	mHalEeProm.pDataBuffer = pDatBuf;
	
//	sprintf(str,"W Flash = %.8lX %d", mHalEeProm.StartAddress, leng);
//	fuDebugMsg(str);
	
	if((addr&0x7ff) == 0)
	{
		//fuDebugMsg("Erase");
		if(HalEePromErase(&mHalEeProm) != 0)
		{
			if(HalEePromErase(&mHalEeProm) != 0)
			{
				//fuDebugMsg("FU FlashRom Erase Error");
				return;
			}
		}
	}
	//fuDebugMsg("Write");
	//for(d=0; d<10000;d++);
	HalEePromWrite(&mHalEeProm);
}
static uint8_t isCorrectFwCodeHead1(uint8_t *pHeadinfo)
{
	if(memcmp(pHeadinfo, SmpFwHeadInfo1, 16) == 0)
		return 1;
	return 0;
	
}
static uint8_t isCorrectFwCodeHead2(uint8_t *pHeadinfo)
{
	if(memcmp(pHeadinfo, SmpFwHeadInfo2, 16) == 0)
		return 1;
	return 0;
}


static void checkFuCodeDataSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	uint8_t		i;
	uint32_t		buffer[65];
	tHalEeProm		mHalEeProm;
	uint8_t			MsgBuf[10];

	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
		;
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_7)
	{
		GPIOD->ODR |= GPIO_PIN_13;
		mHalEeProm.StartAddress = FwUpdateInfo.FwCheck.PackageNumCnt;
		mHalEeProm.StartAddress *= 256;
		mHalEeProm.StartAddress += FLASHROM_FU_START_ADDR;
		mHalEeProm.Length = 256;
		mHalEeProm.pDataBuffer = (uint8_t *)&buffer[0];
		HalEePromRead(&mHalEeProm);
		if(FwUpdateInfo.FwCheck.PackageNumCnt == 0)
		{
			if(isCorrectFwCodeHead1((uint8_t *)&buffer[0]) &&
			   isCorrectFwCodeHead2((uint8_t *)&buffer[12]))
			{
				
			}
			else
			{
				MsgBuf[0] = 1;
				FwUpdateInfo.CbFunction(API_FU_EVT_CHECK_RESULT, MsgBuf);
				FwUpdateInfo.FwCheckStatus = API_FU_FW_CHECK_FAIL;
				LibSwTimerClose(checkFuCodeDataSwTimerHandler, 0);
				fuDebugMsg("Fw Check error");
			}
		}
		for(i=0; i<64; i++)
		{
			FwUpdateInfo.FwCheck.CheckSum ^= buffer[i];
		}
		FwUpdateInfo.FwCheck.PackageNumCnt++;
		if(FwUpdateInfo.FwCheck.PackageNumCnt >= FwUpdateInfo.FwPackageNum)
		{
			if(FwUpdateInfo.FwCheck.CheckSum == 0)
			{
				MsgBuf[0] = 0;
				FwUpdateInfo.CbFunction(API_FU_EVT_CHECK_RESULT, MsgBuf);
				LibSwTimerClose(checkFuCodeDataSwTimerHandler, 0);
				FwUpdateInfo.FwCheckStatus = API_FU_FW_CHECK_SUCCESS;
				fuDebugMsg("Fw Check success");
			}
			else
				FwUpdateInfo.FwCheckStatus = API_FU_FW_CHECK_FAIL;
			LibSwTimerClose(checkFuCodeDataSwTimerHandler, 0);
			fuDebugMsg("FW Check Finish");
		}
		else
		{
			MsgBuf[0] = FwUpdateInfo.FwCheck.PackageNumCnt % 0x100;
			MsgBuf[1] = FwUpdateInfo.FwCheck.PackageNumCnt / 0x100;
			FwUpdateInfo.CbFunction(API_FU_EVT_FW_CHECKING, MsgBuf);	
			
			//MsgBuf[0] = 1;
			//FwUpdateInfo.CbFunction(API_FU_EVT_CHECK_RESULT, MsgBuf);
		}
		GPIOD->ODR &= ~GPIO_PIN_13;
	}	
}

static void checkRcvFuCodeData(void)
{
	uint8_t	MsgBuf[10];
	if(FwUpdateInfo.FwPackageNumCnt >= FwUpdateInfo.FwPackageNum)
	{
		fuDebugMsg("FW 接收完畢,自動檢查是否正確");
		FwUpdateInfo.FwCheck.PackageNumCnt = 0;
		FwUpdateInfo.FwCheck.CheckSum = 0x13848255;
				
		if(FwUpdateInfo.CbFunction)
		{
			MsgBuf[0] = FwUpdateInfo.FwPackageNum % 0x100;
			MsgBuf[1] = FwUpdateInfo.FwPackageNum / 0x100;
			FwUpdateInfo.CbFunction(API_FU_EVT_START_FW_CHECK, MsgBuf);				
		}
		FwUpdateInfo.FwCheckStatus = API_FU_FW_CHECKING;
		LibSwTimerOpen(checkFuCodeDataSwTimerHandler, 0);
	}
}		
/* Public function prototypes -----------------------------------------------*/
void apiFuStartUp(tApiFuCallbackFunction CbFunction)
{
	memset(&FwUpdateInfo, 0, sizeof(tFwUpdateInfo));
	//SystemParameter.Battery[0].SystemFlag2.l&=~SYSTEMFLAG2_FW_BUF_FULL;
	//SystemParameter.Battery[0].SystemFlag2.l&=~SYSTEMFLAG2_CAN2_FW_CHECK_ERROR;
	//SystemParameter.Battery[1].SystemFlag2.l&=~SYSTEMFLAG2_FW_BUF_FULL;
	//SystemParameter.Battery[1].SystemFlag2.l&=~SYSTEMFLAG2_CAN2_FW_CHECK_ERROR;
	//ValidFwCodeFlag=FALSE;
	//CheckAndUpdateFlag=FALSE;
	FwUpdateInfo.InfoStatus |= FW_UPDATE_STATUS_INITIAL;
//	FwUpdatePackageIdleTime = FWUPDATE_INFO_IDLE_TIME;
	FwUpdateInfo.UpdateAddress[0].l = 0xffffff;
	FwUpdateInfo.UpdateAddressTemp = 0xffffff;
	FwUpdateInfo.FwPackageNumCnt = 0;
	FwUpdateInfo.CbFunction = CbFunction;
//	fuDebugMsg("apiFuStartUp....0");
}
void apiFuSetTotalPackageNum(uint32_t num)
{
	FwUpdateInfo.FwPackageNum = num;
	if(FwUpdateInfo.FwPackageNum)
	{
		FwUpdateInfo.InfoStatus |= FW_UPDATE_STATUS_PACKAGENUM;
//		fuDebugMsg("apiFuSetTotalPackageNum....0");
	}
}

void apiFuRcvSetVersion(uint32_t Version)
{
	//-------------------------------
	//	check fw version
//	fuDebugMsg("Version");
	//if(Version > FwHeaderInfo.Version.l)	//版本號比ROM 裡面的新,可以更新FW
	if(1)
	{
		//fuDebugMsg("apiFuRcvSetVersion....0");
		FwUpdateInfo.InfoStatus |= FW_UPDATE_STATUS_VERSION;
//		FwUpdatePackageIdleTime = FWUPDATE_INFO_IDLE_TIME;
#ifdef SHOW_PDO_DEBUG_MSG				
		//sprintf(str,"Ver1:%.8lX %.8lX",value,FwHeaderInfo.Version.l);
		//SendUartMessage((BYTE *)str);
#endif				
	}
	else
	{
#ifdef SHOW_PDO_DEBUG_MSG				
		//sprintf(str,"Ver2:%.8lX %.8lX",value,FwHeaderInfo.Version.l);
		//SendUartMessage((BYTE *)str);
#endif				
		;
	}
	if((FwUpdateInfo.InfoStatus&0x0f) == 0x0f)
	{
		//FwHeadInfoTime=30;
		//		SystemParameter.Battery[0].SystemFlag2.l|=SYSTEMFLAG2_RCV_FW_UPDATE_HEAD_INFO;
		//		SystemParameter.Battery[1].SystemFlag2.l|=SYSTEMFLAG2_RCV_FW_UPDATE_HEAD_INFO;
	}
}

void apiFuRcvSetBaseAddr(uint32_t BaseAddr)
{
//	fuDebugMsg("Base Addr");
	//if(BaseAddr == EEPROM_FW_SECTOR_ADDR)
	{
		FwUpdateInfo.BaseAddress = BaseAddr;
		FwUpdateInfo.InfoStatus |= FW_UPDATE_STATUS_BASEADDR;
		//FwUpdatePackageIdleTime=FWUPDATE_INFO_IDLE_TIME;

#ifdef SHOW_PDO_DEBUG_MSG				
				//sprintf(str,"Rcv Base Address:%.8lX",FwUpdateInfo.BaseAddress);
				//SendUartMessage((BYTE *)str);
				//sprintf(str,"InfoStatus:%.2X",FwUpdateInfo.InfoStatus);
				//SendUartMessage((BYTE *)str);
#endif				
	}
}	

void apiFuSetUpgradeData(uint32_t address, uint8_t *pDatBuf, uint16_t leng)
{
	const uint8_t	BitsTab[]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
	uint16_t	u16;
	uint8_t		offset,B,b,i;
	char	str[100];
	uint8_t		buffer[10];
	
	if(isValidInfoStatus() == 0)
	{
		//fuDebugMsg("Rcv Data....0");
		return;
	}
		
	if(FwUpdateInfo.FwPackageNumCnt >= FwUpdateInfo.FwPackageNum)
	{
		//fuDebugMsg("Rcv Data....1");
		return;
	}
	if(FwUpdateInfo.UpdateAddressTemp == address)
	{
		//fuDebugMsg("Rcv Data....2");
		return;		//??????
	}
	//sprintf(str, "Rcv Data....:%.6lX %d" , address,leng);
	//fuDebugMsg(str);

	for(u16=0 ;u16<leng; u16 += 8)
	{				
#if	1
		if((FwUpdateInfo.UpdateAddress[FwUpdateInfo.FwBufIndexEptr].l&0xffffff00) != 
		   (address&0xffffff00L))
		{
			FwUpdateInfo.CodeBufStatus[FwUpdateInfo.FwBufIndexEptr].l = 0;
		}
		offset = (uint8_t)address;
		B = offset /64;
		b = ((offset /8) &0x07);
		
//		sprintf(str, "Rcv Data ....B d %.2X %d %d" ,offset ,B, b);
//		fuDebugMsg(str);
		
		FwUpdateInfo.CodeBufStatus[FwUpdateInfo.FwBufIndexEptr].b[B] |= BitsTab[b];
		for(i=0; i<8; i++)
			FwUpdateInfo.CodeBuf[FwUpdateInfo.FwBufIndexEptr][offset++] = pDatBuf[i];
		FwUpdateInfo.UpdateAddress[FwUpdateInfo.FwBufIndexEptr].l = address;
		FwUpdateInfo.UpdateAddressTemp = address;
		if(offset == 0 && 
			FwUpdateInfo.CodeBufStatus[FwUpdateInfo.FwBufIndexEptr].l == (DWORD)0xffffffffL)	//
		{
//			fuDebugMsg("............1");
			FwUpdateInfo.UpdateAddress[FwUpdateInfo.FwBufIndexEptr].b[0] = 0;	//????256B
		//	FwUpdateInfo.UpdateAddress[FwUpdateInfo.FwBufIndexEptr].l +=
		//				 FwUpdateInfo.BaseAddress;
			FwUpdateInfo.CodeBufStatus[FwUpdateInfo.FwBufIndexEptr].l = 0;
			FwUpdateInfo.FwPackageNumCnt++;
			//-------------------------------
			//	return update progress
//				fuDebugMsg("............2");
			//sprintf(str, "addr = %.8lX ",&FwUpdateInfo.CodeBuf[FwUpdateInfo.FwBufIndexEptr][0]);
			//fuDebugMsg(str);
			
			writeCodeDataToFlashRom(FwUpdateInfo.UpdateAddress[FwUpdateInfo.FwBufIndexEptr].l, 
					&FwUpdateInfo.CodeBuf[FwUpdateInfo.FwBufIndexEptr][0],
					256);
			if(FwUpdateInfo.CbFunction)
			{
				buffer[0] = FwUpdateInfo.FwPackageNumCnt % 0x100;
				buffer[1] = FwUpdateInfo.FwPackageNumCnt / 0x100;
				FwUpdateInfo.CbFunction(API_FU_EVT_PROGRESS, buffer);				
			}
			checkRcvFuCodeData();
//				else
//					fuDebugMsg("............3");
			//sprintf(str,"Full %d %d %.8lX",
			//	FwUpdateInfo.FwBufIndexEptr,FwUpdateInfo.FwPackageNumCnt,
			//	FwUpdateInfo.UpdateAddress[FwUpdateInfo.FwBufIndexEptr].l
			//	);
			//SendUartMessage((BYTE *)str);	
			
			FwUpdateInfo.FwBufIndexEptr++;
			if(FwUpdateInfo.FwBufIndexEptr >= FW_CODE_BUF_ITEM)
				FwUpdateInfo.FwBufIndexEptr = 0;			
			if(FwUpdateInfo.FwBufIndexSptr == FwUpdateInfo.FwBufIndexEptr)
			{
				FwUpdateInfo.FwBufIndexSptr++;
				if(FwUpdateInfo.FwBufIndexSptr >= FW_CODE_BUF_ITEM)
					FwUpdateInfo.FwBufIndexSptr = 0;
			}
			FwUpdateInfo.UpdateAddress[FwUpdateInfo.FwBufIndexEptr].l = 0xffffff;
		}							
#endif
		address += 8;
	}
	/*
	if(ResetToIspFlag || CheckAndUpdateFlag)	///??????Reset,??????SPI ROM
			return;
		if((FwUpdateInfo.InfoStatus&0x0f)!=0x0f)		//???????????
		{
			return;
		}
	*/
}
uint8_t	apiFuGetFwCheckStatus(void)
{
	return FwUpdateInfo.FwCheckStatus;
}


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    















/**
  ******************************************************************************
  * @file        AppSerialCanDavinciFirmwareUpgrade.c
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
#include "define.h"
#include "main.h"
#include "LibDebug.h"
#include "smp_debug.h"
#include "halafe.h"
#include "halTimer.h"
#include "HalRtc.h"
#include "smp_can.h"
#include "smp_can_fifo.h"
#include "LibSwTimer.h"
#include "AppSerialCanDavinci.h"
#include "AppBms.h"
#include "ApiFu.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	canFuDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/

static	uint16_t	rcv_pkg = 0;
void FuCallbackFunction(uint16_t evt, uint8_t *pMsgBuf)
{
	smp_can_package_t	CanPkg;

	switch(evt)
	{
	case API_FU_EVT_PROGRESS:
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_FU_TX, appProjectGetScuId(),
									SMP_FU_INFO_OBJ_INDEX,
									SMP_FU_INFO_PROGRESS_SUB_INDEX);
		CanPkg.dlc = 2;
		CanPkg.dat[0] = pMsgBuf[0];
		CanPkg.dat[1] = pMsgBuf[1];
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
		break;
	case API_FU_EVT_CHECK_RESULT:
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_FU_TX, appProjectGetScuId(),
									SMP_FU_INFO_OBJ_INDEX,
									SMP_FU_INFO_FW_CHECKING_SUB_INDEX);
		CanPkg.dlc = 2;
		CanPkg.dat[0] = 0;
		CanPkg.dat[1] = pMsgBuf[0];
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
		break;
	case API_FU_EVT_START_FW_CHECK:
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_FU_TX, appProjectGetScuId(),
									SMP_FU_INFO_OBJ_INDEX,
									SMP_FU_INFO_FW_CHECKING_SUB_INDEX);
		CanPkg.dlc = 3;
		CanPkg.dat[0] = 1;
		CanPkg.dat[1] = pMsgBuf[0];
		CanPkg.dat[2] = pMsgBuf[1];
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
		break;
	case API_FU_EVT_FW_CHECKING:
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_FU_TX, appProjectGetScuId(),
									SMP_FU_INFO_OBJ_INDEX,
									SMP_FU_INFO_FW_CHECKING_SUB_INDEX);
		CanPkg.dlc = 3;
		CanPkg.dat[0] = 2;
		CanPkg.dat[1] = pMsgBuf[0];
		CanPkg.dat[2] = pMsgBuf[1];
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
		break;
	}
//	canFuDebugMsg("FuCallbackFunction");
}

static void decodeFuInfoPackage(smp_can_package_t *pCanPkg)
{
	uint16_t	subindex;
	uint32_t	n1, n2;
	char str[100];
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);
	switch(subindex)
	{
	case SMP_FU_INFO_STARTUP_SUB_INDEX:
		n1 = GET_DWORD(&pCanPkg->dat[0]);
		apiFuStartUp(FuCallbackFunction);
		apiFuSetTotalPackageNum(n1);
//		canFuDebugMsg("Fu Rcv startup");
		break;
	case SMP_FU_INFO_BASE_ADDR_SUB_INDEX:
//		sprintf(str, "%.8lX %.8lX %.8lX",
//					(DWORD)pCanPkg,(DWORD) &pCanPkg->dat[0], (DWORD) &pCanPkg->dat[4]);
//		canFuDebugMsg(str);
		n1 = GET_DWORD(&pCanPkg->dat[0]);
		n2 = GET_DWORD(&pCanPkg->dat[4]);
		apiFuRcvSetVersion(n1);
		apiFuRcvSetBaseAddr(n2);
//		canFuDebugMsg("Fu Rcv Base Addr");
		rcv_pkg = 0;
		break;
	case SMP_FU_INFO_FW_CHECK_AND_UPDATE_SUB_INDEX:
		if(memcmp(&pCanPkg->dat[0], "ChKANdUd", 8) == 0)
		{
			canFuDebugMsg("Check & Update");	
			apiFuUpdateFw();
		}
		break;
	case SMP_FU_INFO_FW_RESET_AND_UPDATE_SUB_INDEX:
		if(memcmp(&pCanPkg->dat[0], "RStUPDaT", 8) == 0)
		{
			canFuDebugMsg("Reset And Update");
			apiFuResetAndUpdate();
		}
		break;
	case SMP_FU_INFO_FW_APP_RESET:
		if(memcmp(&pCanPkg->dat[0], "AppReSeT", 8) == 0)
		{
			canFuDebugMsg("App Reset");	
			apiFuResetApp();
		}
		break;
	case SMP_FU_INFO_GOTO_BOOTLOADER_SUB_INDEX:
		apiFuSetMagicCode(pCanPkg->dat[0]);
		apiFuJumpToBootloader();
		break;
	}
}


static uint8_t isValidScuId(uint8_t scuid)
{
	if(scuid == 0 || scuid == appProjectGetScuId())
		return 1;
	else
		return 0;
}

static void decodeFuDataPackage(smp_can_package_t *pCanPkg)
{
	uint8_t		objindex;
	uint16_t	subindex;
	uint32_t	address;
	char	str[100];
	
//	rcv_pkg++;
//	sprintf(str, "Fu Rcv code data :%d", rcv_pkg);
//	canFuDebugMsg(str);
	
	objindex = SMP_CAN_GET_OBJ_INDEX(pCanPkg->id);
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);
	
//	else if(objindex >=  &&
//			objindex <= SMP_FU_DATA_END_OBJ_INDEX)

	address = objindex - SMP_FU_DATA_START_OBJ_INDEX;
	address *= 8192L;
	address += ((uint32_t)subindex * 8L);

//		address  = 
	apiFuSetUpgradeData(address, pCanPkg->dat, pCanPkg->dlc);
//	void apiFuSetUpgradeData(uint32_t addr, uint8_t *pDatBuf, uint16_t leng);
}


			
			
void DavinciCanFunFuRx(smp_can_package_t *pCanPkg)
{
	uint8_t		objindex;
	uint8_t		sucid;
	uint32_t	address;

//	canFuDebugMsg("DavinciCanFunFuRx");
	
	sucid = SMP_CAN_GET_SCU_ID(pCanPkg->id);
	if(isValidScuId(sucid) == 0)
		return;
		
	objindex = SMP_CAN_GET_OBJ_INDEX(pCanPkg->id);
	
	if(objindex == SMP_FU_INFO_OBJ_INDEX)
	{
		decodeFuInfoPackage(pCanPkg);
	}
	else if(objindex >= SMP_FU_DATA_START_OBJ_INDEX &&
			objindex <= SMP_FU_DATA_END_OBJ_INDEX)
	{			
		decodeFuDataPackage(pCanPkg);
	}
}


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    

     
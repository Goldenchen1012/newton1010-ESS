/**
  ******************************************************************************
  * @file        ApiCanbusPkgFu.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/12/27
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
#define	canbusPkgFuDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/

static	uint16_t	rcv_pkg = 0;

void apiCanbusPkgFuDecodeInfoPackage(smp_can_package_t *pCanPkg, tApiFuCallbackFunction CbFunction)
{
	uint16_t	subindex;
	uint32_t	n1, n2;
	char str[100];
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);
	switch(subindex)
	{
	case SMP_FU_INFO_STARTUP_SUB_INDEX:
		n1 = GET_DWORD(&pCanPkg->dat[0]);
		apiFuStartUp(CbFunction);
		apiFuSetTotalPackageNum(n1);
//		canbusPkgFuDebugMsg("Fu Rcv startup");
		break;
	case SMP_FU_INFO_BASE_ADDR_SUB_INDEX:
//		sprintf(str, "%.8lX %.8lX %.8lX",
//					(DWORD)pCanPkg,(DWORD) &pCanPkg->dat[0], (DWORD) &pCanPkg->dat[4]);
//		canbusPkgFuDebugMsg(str);
		n1 = GET_DWORD(&pCanPkg->dat[0]);
		n2 = GET_DWORD(&pCanPkg->dat[4]);
		apiFuRcvSetVersion(n1);
		apiFuRcvSetBaseAddr(n2);
//		canbusPkgFuDebugMsg("Fu Rcv Base Addr");
		rcv_pkg = 0;
		break;
	case SMP_FU_INFO_FW_CHECK_AND_UPDATE_SUB_INDEX:
		if(memcmp(&pCanPkg->dat[0], "ChKANdUd", 8) == 0)
		{
			canbusPkgFuDebugMsg("Check & Update");	
			apiFuUpdateFw();
		}
		break;
	case SMP_FU_INFO_FW_RESET_AND_UPDATE_SUB_INDEX:
		if(memcmp(&pCanPkg->dat[0], "RStUPDaT", 8) == 0)
		{
			canbusPkgFuDebugMsg("Reset And Update");
			apiFuResetAndUpdate();
		}
		break;
	case SMP_FU_INFO_FW_APP_RESET:
		if(memcmp(&pCanPkg->dat[0], "AppReSeT", 8) == 0)
		{
			canbusPkgFuDebugMsg("App Reset");	
			apiFuResetApp();
		}
		break;
	case SMP_FU_INFO_GOTO_BOOTLOADER_SUB_INDEX:
		apiFuSetMagicCode(pCanPkg->dat[0]);
		apiFuJumpToBootloader();
		break;
	}
}

void apiCanbusPkgFuDecodeDataPackage(smp_can_package_t *pCanPkg)
{
	uint8_t		objindex;
	uint16_t	subindex;
	uint32_t	address;
	char	str[100];
	
//	rcv_pkg++;
//	sprintf(str, "Fu Rcv code data :%d", rcv_pkg);
//	canbusPkgFuDebugMsg(str);
	
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

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    

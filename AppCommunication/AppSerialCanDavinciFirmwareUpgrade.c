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
#include "ApiCanbusPkgFu.h"

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


static uint8_t isValidScuId(uint8_t scuid)
{
	if(scuid == 0 || scuid == appProjectGetScuId())
		return 1;
	else
		return 0;
}
			
			
void DavinciCanFunFuRx(smp_can_package_t *pCanPkg)
{
	uint8_t		objindex;
	uint8_t		scuid;
	uint32_t	address;

	scuid = SMP_CAN_GET_SCU_ID(pCanPkg->id);
	if(isValidScuId(scuid) == 0)
		return;
		
	objindex = SMP_CAN_GET_OBJ_INDEX(pCanPkg->id);
	
	if(objindex == SMP_FU_INFO_OBJ_INDEX)
	{
		apiCanbusPkgFuDecodeInfoPackage(pCanPkg, FuCallbackFunction);
	}
	else if(objindex >= SMP_FU_DATA_START_OBJ_INDEX &&
			objindex <= SMP_FU_DATA_END_OBJ_INDEX)
	{			
		apiCanbusPkgFuDecodeDataPackage(pCanPkg);
	}
}


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    

     
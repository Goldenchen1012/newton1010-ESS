/**
  ******************************************************************************
  * @file        AppSerialCanDavinciCommon.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/18
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
#include "smp_can.h"
#include "smp_can_fifo.h"
#include "LibSwTimer.h"
#include "AppSerialCanDavinci.h"
#include "AppBms.h"
#include "AppScuIdAssign.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	canCommonDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void DavinciCommonRequestScuId(smp_can_package_t *pCanPkg)
{
	uint8_t	scuid;
	smp_can_package_t	CanPkg;
	
	if(appBmsIsInAssignIdMode())
	{
		if(memcmp(&pCanPkg->dat, "GetScuID", 8) == 0)
		{
			scuid = appBmsGetScuId();
			CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_COMMON_TX, scuid,
									SMP_COMMON_GET_SCUID_OBJ_INDEX,
									scuid + 1);
			CanPkg.dlc = 8;
			memcpy(&CanPkg.dat[0], "RetScuID", 8);
			appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
		}
	}
}


 static void DavinciCommonSetScuId(smp_can_package_t *pCanPkg)
{
	uint8_t	scuid;
	smp_can_package_t	CanPkg;
	
	if(appBmsIsInScuIdRequestMode())
	{
		scuid = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);
		if(scuid>=2 && memcmp(&pCanPkg->dat, "RetScuID", 8) == 0)
		{
			CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_COMMON_RX, scuid,
									SMP_COMMON_STOP_ID_ASSIGN_OBJ_INDEX,
									scuid - 1);
			CanPkg.dlc = 8;
			memcpy(&CanPkg.dat[0], "StopIDAs", 8);
			appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
			
			appBmsAssignScuId(scuid);
		}
	}
}

static void DavinciCommonStopScuIdAssign(smp_can_package_t *pCanPkg)
{
	uint8_t	source_id;
	uint8_t	scuid;
	smp_can_package_t	CanPkg;
	
	//if(appBmsIsInAssignIdMode())
	{
		if(memcmp(&pCanPkg->dat, "StopIDAs", 8) == 0)
		{
			source_id = SMP_CAN_GET_SCU_ID(pCanPkg->id);
			scuid = appBmsGetScuId();
			if(source_id == (scuid + 1))
			{
				CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_COMMON_TX, source_id,
										SMP_COMMON_STOP_ID_ASSIGN_OBJ_INDEX,
										scuid);
				CanPkg.dlc = 8;
				memcpy(&CanPkg.dat[0], "StopedID", 8);
				appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
				appBmsStopOutputAssignIdSignal();
			}
		}
	}
}

static void DavinciCommonStopScuIdRequest(smp_can_package_t *pCanPkg)
{
	uint8_t	scuid;
	
	scuid = SMP_CAN_GET_SCU_ID(pCanPkg->id);

	if(scuid != appBmsGetScuId())
		return;
	 
	if(memcmp(&pCanPkg->dat, "StopedID", 8) == 0)
	{
		appBmsExitScuIdRequestMode();
		appBmsEnterScuIdAssignMode();
	}
}

static void DavinciCommonResetScuId(smp_can_package_t *pCanPkg)
{
	uint8_t	source_id;
	uint8_t	scuid;
	smp_can_package_t	CanPkg;
	
	if(memcmp(&pCanPkg->dat, "RstScuID", 8) == 0)
	{
		appBmsResetScuId();
		canCommonDebugMsg("RESET SCU ID");
	}
}

static void DavinciCommonFindFirstScu(smp_can_package_t *pCanPkg)
{
	uint8_t	source_id;
	uint8_t	scuid;
	smp_can_package_t	CanPkg;
	
	if(memcmp(&pCanPkg->dat, "FirstScu", 8) == 0)
	{
		appBmsFindFirstScu();
		canCommonDebugMsg("FIND SCU ID");
	}
}

//DavinciCommonResponseScuId
//----------------------------------------------------------
SMP_CAN_DECODE_CMD_START(mDavinciCommonCanDecodeTab)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_COMMON_RX, 0,
									SMP_COMMON_FIND_FIRST_SCU_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_FUN | CHECK_SMP_CAN_OBJ,
								DavinciCommonFindFirstScu)


	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_COMMON_RX, 0,
									SMP_COMMON_RESET_SCU_ID_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_FUN | CHECK_SMP_CAN_OBJ,
								DavinciCommonResetScuId)


	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_COMMON_RX, 0,
									SMP_COMMON_GET_SCUID_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_FUN | CHECK_SMP_CAN_OBJ,
								DavinciCommonRequestScuId)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_COMMON_TX, 0,
									SMP_COMMON_GET_SCUID_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_FUN | CHECK_SMP_CAN_OBJ,
								DavinciCommonSetScuId)
								
								
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_COMMON_RX, 0,
									SMP_COMMON_STOP_ID_ASSIGN_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_FUN | CHECK_SMP_CAN_OBJ,
								DavinciCommonStopScuIdAssign)
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_COMMON_TX, 0,
									SMP_COMMON_STOP_ID_ASSIGN_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_FUN | CHECK_SMP_CAN_OBJ,
								DavinciCommonStopScuIdRequest)

												
								
								

SMP_CAN_DECODE_CMD_END();


/* Public function prototypes -----------------------------------------------*/

void DavinciCanFunCommonRx(smp_can_package_t *pCanPkg)
{
	uint8_t	i,n;
	uint8_t cmdIndex;
	
	char	str[100];
	char	str1[100];
 	cmdIndex = 0;

	for(cmdIndex = 0; mDavinciCommonCanDecodeTab[cmdIndex].fun != 0; cmdIndex++)
	{
		if((mDavinciCommonCanDecodeTab[cmdIndex].canid & mDavinciCommonCanDecodeTab[cmdIndex].mask) == 
		   (mDavinciCommonCanDecodeTab[cmdIndex].mask & pCanPkg->id))
		{
			mDavinciCommonCanDecodeTab[cmdIndex].fun(pCanPkg);
		///	canCommonDebugMsg("RRR");
			break; 		
 		}
	}
}


/* Public function prototypes -----------------------------------------------*/

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    

     
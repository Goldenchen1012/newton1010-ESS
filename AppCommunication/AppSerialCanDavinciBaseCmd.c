/**
  ******************************************************************************
  * @file        AppSerialCanDavinciBaseCmd.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/19
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
#include "AppSerialCanDavinciCmd.h"
#include "AppSerialCanDavinciParameter.h"
#include "AppBms.h"

void appSerialUartSendMessage(uint8_t *str);
#define	canCmdDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void scuIdBrocast(smp_can_package_t *pCanPkg)
{
	uint8_t	scuid;
	scuid = SMP_CAN_GET_SCU_ID(pCanPkg->id);
	appBmsRcvScuIdBrocast(scuid);
}

static void scuSystemFlag(smp_can_package_t *pCanPkg)
{
	uint8_t	scuid;
	
	scuid = SMP_CAN_GET_SCU_ID(pCanPkg->id);
	appBmsSetScuSystemFlag(scuid, GET_DWORD(&pCanPkg->dat[0]), GET_DWORD(&pCanPkg->dat[4]));
}
static void scuCurrent(smp_can_package_t *pCanPkg)
{
	uint8_t	scuid;
	
	scuid = SMP_CAN_GET_SCU_ID(pCanPkg->id);
	appBmsSetScuCurrent(scuid, 
				(int32_t)GET_DWORD(&pCanPkg->dat[0]), 
				(int32_t)GET_DWORD(&pCanPkg->dat[4]));
}


static void scuVbat(smp_can_package_t *pCanPkg)
{
	uint8_t	scuid;
	
	scuid = SMP_CAN_GET_SCU_ID(pCanPkg->id);
	appBmsSetScuVbat(scuid, GET_DWORD(&pCanPkg->dat[0]), GET_DWORD(&pCanPkg->dat[4]));
}
static void scuMinMaxValue(smp_can_package_t *pCanPkg)
{
	uint8_t	scuid;
	tIbyte	MinCellVoltage;
	tIbyte	MaxCellVoltage;
	tIbyte	MinNtcTemp;
	tIbyte	MaxNtcTemp;	
	
	scuid = SMP_CAN_GET_SCU_ID(pCanPkg->id);
	
	MinCellVoltage.b[0] = pCanPkg->dat[0];
	MinCellVoltage.b[1] = pCanPkg->dat[1];

	MaxCellVoltage.b[0] = pCanPkg->dat[2];
	MaxCellVoltage.b[1] = pCanPkg->dat[3];

	MinNtcTemp.b[0] = pCanPkg->dat[4];
	MinNtcTemp.b[1] = pCanPkg->dat[5];

	MaxNtcTemp.b[0] = pCanPkg->dat[6];
	MaxNtcTemp.b[1] = pCanPkg->dat[7];	
	
	appBmsSetScuVbat(scuid, GET_DWORD(&pCanPkg->dat[0]), GET_DWORD(&pCanPkg->dat[4]));


}

SMP_CAN_DECODE_CMD_START(mDavinciCanBaseRxTxTab)
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, 0,
									SMP_BASE_SCU_ID_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								scuIdBrocast)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, 0,
									SMP_BASE_SYSTEM_FLAG_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								scuSystemFlag)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, 0,
									SMP_BASE_CURRENT_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								scuCurrent)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, 0,
									SMP_BASE_VB_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								scuVbat)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, 0,
									SMP_BASE_MIN_MAX_VALUE_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								scuMinMaxValue)



SMP_CAN_DECODE_CMD_END();


/* Public function prototypes -----------------------------------------------*/
void DavinciCanFunBaseRxTx(smp_can_package_t *pCanPkg)
{
	uint8_t cmdIndex;
	
	char	str[100];
	char	str1[100];
	
	//appSerialCanDavinciPutPkgToCanFifo(pCanPkg);
	//appSerialCanDavinciSendTextMessage("Decode Rx Cmd");

 	cmdIndex = 0;
	for(cmdIndex = 0; mDavinciCanBaseRxTxTab[cmdIndex].fun != 0; cmdIndex++)
	{
		if((mDavinciCanBaseRxTxTab[cmdIndex].canid & mDavinciCanBaseRxTxTab[cmdIndex].mask) == 
		   (mDavinciCanBaseRxTxTab[cmdIndex].mask & pCanPkg->id))// &&
		   //appSerialCanDavinciIsCorrectScuId(pCanPkg))
		{
			mDavinciCanBaseRxTxTab[cmdIndex].fun(pCanPkg);
		
			break; 		
 		}
	}
}



/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    

     
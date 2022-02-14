/**
  ******************************************************************************
  * @file        AppSerialCanDavinciDetail.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2022/02/10
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 Johnny</center></h2>
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
//#include "halafe.h"
//#include "halTimer.h"
//#include "smp_can.h"
//#include "smp_can_fifo.h"
#include "LibSwTimer.h"
#include "AppSerialCanDavinci.h"
//#include "ApiSysPar.h"
//#include "LibCalibration.h"
//#include "HalAfeADS7946.h"
//#include "HalBsp.h"
//#include "AppProject.h"
#include "AppBms.h"
		  

void appSerialCanDavinciSendTextMessage(char *str);
#define	appSerialCanDavinciDetailMsg(str)	appSerialCanDavinciSendTextMessage(str)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define		canDbgScuId()		appProjectGetScuId()

/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
int32_t appCurrDebug(uint8_t CurrentIndex, int32_t adc);
int32_t appVbatDebug(uint8_t VbatIndex, int32_t adc);

static void DavinciCanDetailCellVoltage(smp_can_package_t *pCanPkg)
{
	uint8_t		i;
	uint8_t		scuid;
	uint16_t	subindex, voltage;
	char	str[100];
	scuid = SMP_CAN_GET_SCU_ID(pCanPkg->id);
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);
	for(i=0; i<4; i++)
	{
		appBmsSetCellVoltage(scuid, subindex++, GET_WORD(&pCanPkg->dat[i*2]));
	}	
}

SMP_CAN_DECODE_CMD_START(mDavinciDetailCanDecodeTab)
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_DETAIL_TX, 0,
									SMP_DETAIL_CELL_VOLTAGE_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								DavinciCanDetailCellVoltage)

#if	0
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_DEBUG_RX, 0,
									SMP_DEBUG_MODE_CONTROL_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								DavinciCanDebugModeChange)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_DEBUG_RX, 0,
									SMP_DEBUG_SIMU_CURR_ADC_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								DavinciCanDebugCurrentAdc)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_DEBUG_RX, 0,
									SMP_DEBUG_SIMU_VBAT_ADC_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								DavinciCanDebugVbatAdc)


	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_DEBUG_RX, 0,
									SMP_DEBUG_CELLV_SIMU_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								DavinciCanDebugCellVSimu)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_DEBUG_RX, 0,
									SMP_DEBUG_NTCV_SIMU_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								DavinciCanDebugNtcVSimu)
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_DEBUG_RX, 0,
									SMP_DEBUG_NTCT_SIMU_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								DavinciCanDebugNtcTSimu)


	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_DEBUG_RX, 0,
									SMP_DEBUG_GPIO_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								DavinciCanDebugGpio)
								
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_DEBUG_RX, 0,
									SMP_DEBUG_SIMU_CURRENT_VALUE_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								DavinciCanDebugCurrentSimu)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_DEBUG_RX, 0,
									SMP_DEBUG_SIMU_VBAT_VALUE_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								DavinciCanDebugVBatSimu)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_DEBUG_RX, 0,
									SMP_DEBUG_RELAY_CONTROL_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								DavinciCanDebugRelayControl)
								
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_DEBUG_RX, 0,
									SMP_DEBUG_SCU_TEMP_SIMU_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								DavinciCanDebugScuTempSimu)
								
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_DEBUG_RX, 0,
									SMP_DEBUG_CLEAR_TEST_COUNT_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								DavinciCanDebugClearTestCount)
								
#endif


SMP_CAN_DECODE_CMD_END();


void DavinciCanFunDetailTx(smp_can_package_t *pCanPkg)
{
	uint8_t	i,n;
	uint8_t cmdIndex;
	
	char	str[100];
	char	str1[100];
 	cmdIndex = 0;

	for(cmdIndex = 0; mDavinciDetailCanDecodeTab[cmdIndex].fun != 0; cmdIndex++)
	{
		if((mDavinciDetailCanDecodeTab[cmdIndex].canid & mDavinciDetailCanDecodeTab[cmdIndex].mask) == 
		   (mDavinciDetailCanDecodeTab[cmdIndex].mask & pCanPkg->id))
		{
		//	sprintf(str,"Debug ID= %d",SMP_CAN_GET_SCU_ID(pCanPkg->id));
		//	appSerialCanDavinciDebugMsg(str);
			mDavinciDetailCanDecodeTab[cmdIndex].fun(pCanPkg);
			break; 		
 		}
	}
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    





/**
  ******************************************************************************
  * @file        AppSerialCanDavinciCmd.c
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
#include "ApiSysPar.h"
#include "ApiEventLog.h"
#include "AppGauge.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	canCmdDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define		canCmdScuId()		appProjectGetScuId()
/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


static void davinciCanAdc(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tLbyte	AdcValue;

	if(SMP_CAN_GET_SUB_INDEX(pCanPkg->id) == 0x00)
	{
		//canCmdDebugMsg("Read Current Adc1");
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canCmdScuId(),
									SMP_CMD_ADC_OBJ_INDEX,
									0x00);
		AdcValue.sl = halAfeGetCurrentAdcValue(0);
	}
	else if(SMP_CAN_GET_SUB_INDEX(pCanPkg->id) == 0x01)
	{
		//canCmdDebugMsg("Read Current Adc2");
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canCmdScuId(),
									SMP_CMD_ADC_OBJ_INDEX,
									0x01);
		AdcValue.sl = halAfeGetCurrentAdcValue(1);
	}
	else if(SMP_CAN_GET_SUB_INDEX(pCanPkg->id) == 0x10)
	{
		//canCmdDebugMsg("Read Vbat Adc1");
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canCmdScuId(),
									SMP_CMD_ADC_OBJ_INDEX,
									0x10);
		AdcValue.sl = halAfeGetVBatAdcValue(0);
	}
	else if(SMP_CAN_GET_SUB_INDEX(pCanPkg->id) == 0x11)
	{
		//canCmdDebugMsg("Read Vbat Adc2");
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canCmdScuId(),
									SMP_CMD_ADC_OBJ_INDEX,
									0x11);
		AdcValue.sl = halAfeGetVBatAdcValue(1);
	}
	else
		return;
	CanPkg.dlc = 4;
	CanPkg.dat[0] = AdcValue.b[0];
	CanPkg.dat[1] = AdcValue.b[1];
	CanPkg.dat[2] = AdcValue.b[2];
	CanPkg.dat[3] = AdcValue.b[3];
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}


static void davinciCanRtc(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tLbyte	Lbyte;
	tHalRtcDateTime	mRtcDateTime;

	if(SMP_CAN_GET_SUB_INDEX(pCanPkg->id) == SMP_RD_RTC_SUB_INDEX)
	{
		
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canCmdScuId(),
									SMP_CMD_RTC_OBJ_INDEX,
									SMP_RD_RTC_SUB_INDEX);
		Lbyte.l = HalRtcGetSmpUnixTime();


		CanPkg.dlc = 4;
		CanPkg.dat[0] = Lbyte.b[0];
		CanPkg.dat[1] = Lbyte.b[1];
		CanPkg.dat[2] = Lbyte.b[2];
		CanPkg.dat[3] = Lbyte.b[3];

		//canCmdDebugMsg("Read RTC");
	}
	else if(SMP_CAN_GET_SUB_INDEX(pCanPkg->id) == SMP_WR_RTC_SUB_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canCmdScuId(),
									SMP_CMD_RTC_OBJ_INDEX,
									SMP_WR_RTC_SUB_INDEX);
		Lbyte.l = HalRtcGetSmpUnixTime();


		CanPkg.dlc = 0;

		Lbyte.l = GET_DWORD(&pCanPkg->dat[0]);
		HalRtcSmpUnixTimeToDateTime(Lbyte.l, &mRtcDateTime);
		HalRtcSetupTime(mRtcDateTime.Hour, mRtcDateTime.Minute, mRtcDateTime.Second);
		HalRtcSetupDate(mRtcDateTime.Year, mRtcDateTime.Month, mRtcDateTime.Day);

		//canCmdDebugMsg("Write RTC");
	}
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}


static void davinciCanRelayOn(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	uint8_t	scuid;

	scuid = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);
	if(memcmp(&pCanPkg->dat[0], "RelayOn", 7) == 0) 
	{
		apiRelayControlSetMasterTurnOnFlag();
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, scuid,
									SMP_CMD_RELAY_ON_OBJ_INDEX,
									canCmdScuId());
		CanPkg.dlc = 0;									
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	}
}


static void davinciCanChecksumn(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	uint8_t	subindex;
	tLbyte	checksum;

	CanPkg.dlc = 4;
								
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);

	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canCmdScuId(),
									SMP_CMD_CHECKSUM_OBJ_INDEX,
									subindex);
									

	if(subindex == SMP_FW_CHECKSUM_SUB_INDEX)
	{
		checksum.l = 0x123456;	
	}
	else if(subindex == SMP_PAR_CHECKSUM_SUB_INDEX)
	{
		checksum.l = apiSysParGetChecksum();
	}
	else if(subindex == SMP_CAL_PAR_CHECKSUM_SUB_INDEX)
	{
		checksum.l = apiCaliParGetChecksum();
	}
	else
		CanPkg.dlc = 0;

	CanPkg.dat[0] = checksum.b[0];
	CanPkg.dat[1] = checksum.b[1];
	CanPkg.dat[2] = checksum.b[2];
	CanPkg.dat[3] = checksum.b[3];
	
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}

static void davinciCanPfFlag(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	uint8_t	subindex;
	tLbyte	checksum;

	CanPkg.dlc = 4;
								
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);

	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canCmdScuId(),
									SMP_CMD_CHECKSUM_OBJ_INDEX,
									subindex);
									
	if(subindex == SMP_PF_FLAG_CLEAN_ALL_SUB_INDEX)
	{
		apiSysParOvpPfClean();
		apiSysParUvpPfClean();
		apiProtectOvpPfClean();
		apiProtectUvpPfClean();
		canCmdDebugMsg("Clear All PF Flag");
	}
	else if(subindex == SMP_PF_FLAG_CLEAN_OVP_SUB_INDEX)
	{
		apiSysParOvpPfClean();
		apiProtectOvpPfClean();
		canCmdDebugMsg("Clear OVP PF Flag");	
	}
	else if(subindex == SMP_PF_FLAG_CLEAN_UVP_SUB_INDEX)
	{
		apiSysParUvpPfClean();
		apiProtectUvpPfClean();
		canCmdDebugMsg("Clear UVP PF Flag");
	}
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}

static void davinciCanCleanData(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	uint8_t	subindex;
	tLbyte	checksum;

	CanPkg.dlc = 4;
								
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);

	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canCmdScuId(),
									SMP_CMD_CLEAN_DATA_OBJ_INDEX,
									subindex);
	if(subindex == SMP_CLEAN_ALL_DATA_SUB_INDEX)
	{
		canCmdDebugMsg("Clear All data");
		
	}	
	else if(subindex == SMP_CLEAN_CYCLE_COUNT_SUB_INDEX)
	{
		canCmdDebugMsg("Clear cycle count");	
		appGaugeCleanCycleCount();
	}
	else if(subindex == SMP_CLEAN_FAULT_LOG1_SUB_INDEX)
	{
		canCmdDebugMsg("Clear fault log1");
	}
	else if(subindex == SMP_CLEAN_FAULT_LOG2_SUB_INDEX)
	{
		canCmdDebugMsg("Clear fault log2");
	}
}

static void faultLog1DataCallBack(uint8_t num, uint8_t *pBuf)
{
	char		str[100];
	uint8_t		n;
	uint8_t		index;
	smp_can_package_t	CanPkg;
	
	sprintf(str,"Read %d fault data ....", num);
	canCmdDebugMsg(str);
	
	index = 0;
	for(n=0; n<num; n++)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canCmdScuId(),
									SMP_CMD_FAULT_LOG_OBJ_INDEX,
									SMP_RET_FAULT_LOG1_DATA_SUB_INDEX+n);
		CanPkg.dlc = 8;
		memcpy(CanPkg.dat, &pBuf[index], 8);
		index += 8;
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	}
}

static void davinciCanFaultLog(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	uint8_t	subindex;
	tLbyte	Lbyte;
	tLbyte	ReadNum;
	CanPkg.dlc = 0;
								
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);

	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canCmdScuId(),
									SMP_CMD_FAULT_LOG_OBJ_INDEX,
									subindex);
									
	if(subindex == SMP_READ_FAULT_LOG1_NUM_SUB_INDEX)
	{
		CanPkg.dlc = 4;
		Lbyte.l = apiEventLogGetLogNumber();
		CanPkg.dat[0] = Lbyte.b[0];
		CanPkg.dat[1] = Lbyte.b[1];
		CanPkg.dat[2] = Lbyte.b[2];
		CanPkg.dat[3] = Lbyte.b[3];
		
		canCmdDebugMsg("Read Fault Log1 number");
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	}	
	else if(subindex == SMP_READ_FAULT_LOG2_NUM_SUB_INDEX)
	{
		canCmdDebugMsg("Read Fault Log2 number");
	}
	else if(subindex == SMP_READ_FAULT_LOG1_DATA_SUB_INDEX)
	{
		Lbyte.b[0] = pCanPkg->dat[0];
		Lbyte.b[1] = pCanPkg->dat[1];
		Lbyte.b[2] = pCanPkg->dat[2];
		Lbyte.b[3] = pCanPkg->dat[3];

		ReadNum.b[0] = pCanPkg->dat[4];
		ReadNum.b[1] = pCanPkg->dat[5];
		ReadNum.b[2] = pCanPkg->dat[6];
		ReadNum.b[3] = pCanPkg->dat[7];
		
		canCmdDebugMsg("Read Fault Log1 data");

		apiEventLogReadLogData(Lbyte.l, ReadNum.l, faultLog1DataCallBack);

	}
	else if(subindex == SMP_READ_FAULT_LOG2_DATA_SUB_INDEX)
	{
		canCmdDebugMsg("Read Fault Log2 data");
	}
}
static void davinciCanSoc(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	uint8_t	subindex;
	tIbyte	Soc0;
	tLbyte	ReadNum;
	CanPkg.dlc = 0;
								
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);

	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canCmdScuId(),
									SMP_CMD_SOC_OBJ_INDEX,
									subindex);			
	CanPkg.dlc = 0;						
	
	if(subindex == SMP_UPDATE_SOC_SUB_INDEX && 
	   memcmp(pCanPkg->dat,"UpdatSoc", 8)==0)
	{
		CanPkg.dlc = 0;
		canCmdDebugMsg("Update Soc");
		appGaugeUpdateSoc0();
	}	
	else if(subindex == SMP_SET_SOC_VALUE_SUB_INDEX)
	{
		canCmdDebugMsg("Set Soc");
		Soc0.i = GET_WORD(&pCanPkg->dat[0]);
		appGaugeSetSoc0(Soc0.i);
	}
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}


//=========================================================================

SMP_CAN_DECODE_CMD_START(mDavinciCanCmdRxTab)
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									SMP_CMD_PAR_WR_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								DavinciCanParameterRdWrite)
								
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									SMP_CMD_PAR_RD_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								DavinciCanParameterRdWrite)
								
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									SMP_CMD_RTC_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								davinciCanRtc)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									SMP_CMD_ADC_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								davinciCanAdc)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									SMP_CMD_RELAY_ON_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								davinciCanRelayOn)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									SMP_CMD_CHECKSUM_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								davinciCanChecksumn)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									SMP_CMD_PF_FLAG_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								davinciCanPfFlag)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									SMP_CMD_CLEAN_DATA_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								davinciCanCleanData)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									SMP_CMD_FAULT_LOG_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								davinciCanFaultLog)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									SMP_CMD_SOC_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								davinciCanSoc)



									
SMP_CAN_DECODE_CMD_END();


/* Public function prototypes -----------------------------------------------*/
void DavinciCanFunCmdRx(smp_can_package_t *pCanPkg)
{
	uint8_t	i,n;
	uint8_t cmdIndex;
	
	char	str[100];
	char	str1[100];
	
	//appSerialCanDavinciPutPkgToCanFifo(pCanPkg);
	//canCmdDebugMsg("Decode Rx Cmd");

 	cmdIndex = 0;
	for(cmdIndex = 0; mDavinciCanCmdRxTab[cmdIndex].fun != 0; cmdIndex++)
	{
		if((mDavinciCanCmdRxTab[cmdIndex].canid & mDavinciCanCmdRxTab[cmdIndex].mask) == 
		   (mDavinciCanCmdRxTab[cmdIndex].mask & pCanPkg->id) &&
		    appSerialCanDavinciIsCorrectScuId(pCanPkg))
		{
			mDavinciCanCmdRxTab[cmdIndex].fun(pCanPkg);
		
			break; 		
 		}
	}
}


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/

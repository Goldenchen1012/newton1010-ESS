/**
  ******************************************************************************
  * @file        AppSerialCanDavinciNotification.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/10/25
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
#include "LibNtc.h"
#include "AppSerialCanDavinci.h"
#include "AppSerialCanDavinciParameter.h"
#include "ApiProtectOvp.h"
#include "ApiProtectUvp.h"
#include "ApiProtectCotp.h"
#include "ApiProtectCutp.h"
#include "ApiProtectDotp.h"
#include "ApiProtectDutp.h"
#include "ApiProtectCocp.h"
#include "ApiProtectDocp.h"
#include "AppBalance.h"
#include "ApiSystemFlag.h"
#include "AppGauge.h"
#include "AppBms.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	notiDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private typedef -----------------------------------------------------------*/
typedef void (* tNotificationRunTable)(void);

/* Private define ------------------------------------------------------------*/
#define		notifyScuId()			appProjectGetScuId()
#define		notifyNtcNumber()		apiSysParGetNtcNumber()
#define		notifyCellNumber()		apiSysParGetCellNumber()

/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static	uint8_t CanDavinciNotificationFunIndex = 0;
static 	uint16_t	NotificationSubIndex = 0;

/* Private function prototypes -----------------------------------------------*/
static void notifyNextFunction(void)
{
	CanDavinciNotificationFunIndex++;
}

static void notifyFunNone(void)
{
	notifyNextFunction();
}

static void notifyBaseScuId(void)
{
	smp_can_package_t	CanPkg;
	tIbyte	subindex;
			
	
#define	CHIP_ID0	(DWORD)(*(DWORD*)(0x1FFF7590UL))
#define	CHIP_ID1	(DWORD)(*(DWORD*)(0x1FFF7594UL)) 
#define	CHIP_ID2	(DWORD)(*(DWORD*)(0x1FFF7598UL)) 

	subindex.i = appProjectGetTimerCount();
	subindex.i ^= subindex.b[1];
		
//	subindex = CHIP_ID0;
//	subindex ^= CHIP_ID1;
//	subindex ^= CHIP_ID2;
	subindex.i &= 0x3fe;
	if(appBmsIsMaster())
		subindex.i |= 0x01;
	
	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, notifyScuId(),
									SMP_BASE_SCU_ID_OBJ_INDEX,
									subindex.i);
	CanPkg.dlc = 0;
	
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}


static void notifyBaseSystemFlag(void)
{
	smp_can_package_t	CanPkg;
	tLbyte	Lbyte;
	
	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, notifyScuId(),
									SMP_BASE_SYSTEM_FLAG_OBJ_INDEX,
									0);
	CanPkg.dlc = 8;
	
	Lbyte.l = apiSystemFlagGetFlag1();
	CanPkg.dat[0] = Lbyte.b[0];
	CanPkg.dat[1] = Lbyte.b[1];
	CanPkg.dat[2] = Lbyte.b[2];
	CanPkg.dat[3] = Lbyte.b[3];

	Lbyte.l = apiSystemFlagGetFlag2();
	CanPkg.dat[4] = Lbyte.b[0];
	CanPkg.dat[5] = Lbyte.b[1];
	CanPkg.dat[6] = Lbyte.b[2];
	CanPkg.dat[7] = Lbyte.b[3];
	
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	
	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, notifyScuId(),
									SMP_BASE_SYSTEM_FLAG_OBJ_INDEX,
									1);
	CanPkg.dlc = 8;
	
	Lbyte.l = apiSystemFlagGetFlag3();
	CanPkg.dat[0] = Lbyte.b[0];
	CanPkg.dat[1] = Lbyte.b[1];
	CanPkg.dat[2] = Lbyte.b[2];
	CanPkg.dat[3] = Lbyte.b[3];

	Lbyte.l = apiSystemFlagGetFlag4();
	CanPkg.dat[4] = Lbyte.b[0];
	CanPkg.dat[5] = Lbyte.b[1];
	CanPkg.dat[6] = Lbyte.b[2];
	CanPkg.dat[7] = Lbyte.b[3];
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}

static void notifyBaseRmQmax(void)
{
	smp_can_package_t	CanPkg;
	tLbyte	Lbyte;
	
	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, notifyScuId(),
									SMP_BASE_RM_QMAX_OBJ_INDEX,
									0);
	CanPkg.dlc = 8;
								
	Lbyte.l = appGaugeGetRM();
	CanPkg.dat[0] = Lbyte.b[0];
	CanPkg.dat[1] = Lbyte.b[1];
	CanPkg.dat[2] = Lbyte.b[2];
	CanPkg.dat[3] = Lbyte.b[3];
	
	Lbyte.l = appGaugeGetQmax();
	CanPkg.dat[4] = Lbyte.b[0];
	CanPkg.dat[5] = Lbyte.b[1];
	CanPkg.dat[6] = Lbyte.b[2];
	CanPkg.dat[7] = Lbyte.b[3];

	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);

}

static void notifyBaseCurrent(void)
{
	smp_can_package_t	CanPkg;
	tLbyte	Curr1,Curr2;
	char	str[100];
	
	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, notifyScuId(),
									SMP_BASE_CURRENT_OBJ_INDEX,
									0);
	CanPkg.dlc = 8;
								
	Curr1.l = appGaugeGetCurrentValue(P_CURRENT);
	CanPkg.dat[0] = Curr1.b[0];
	CanPkg.dat[1] = Curr1.b[1];
	CanPkg.dat[2] = Curr1.b[2];
	CanPkg.dat[3] = Curr1.b[3];
	
	Curr2.l = appGaugeGetCurrentValue(N_CURRENT);
	CanPkg.dat[4] = Curr2.b[0];
	CanPkg.dat[5] = Curr2.b[1];
	CanPkg.dat[6] = Curr2.b[2];
	CanPkg.dat[7] = Curr2.b[3];

	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	
//	sprintf(str,"I1 = %d I2 = %d",Curr1.l, Curr2.l);
//	notiDebugMsg(str);
}

static void notifyBaseFCC(void)
{
	smp_can_package_t	CanPkg;
	tLbyte	Lbyte;
	
	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, notifyScuId(),
									SMP_BASE_FCC_OBJ_INDEX,
									0);
	CanPkg.dlc = 4;
								
	Lbyte.l = appGaugeGetFCC();
	CanPkg.dat[0] = Lbyte.b[0];
	CanPkg.dat[1] = Lbyte.b[1];
	CanPkg.dat[2] = Lbyte.b[2];
	CanPkg.dat[3] = Lbyte.b[3];
	
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}
static void notifyBaseVBatVoltage(void)
{
	smp_can_package_t	CanPkg;
	tLbyte	Lbyte;
	
	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, notifyScuId(),
									SMP_BASE_VB_OBJ_INDEX,
									0);
	CanPkg.dlc = 8;
								
	Lbyte.l = halAfeGetVBatVoltage(0);
	CanPkg.dat[0] = Lbyte.b[0];
	CanPkg.dat[1] = Lbyte.b[1];
	CanPkg.dat[2] = Lbyte.b[2];
	CanPkg.dat[3] = Lbyte.b[3];
	
	Lbyte.l = halAfeGetVBatVoltage(1);
	CanPkg.dat[4] = Lbyte.b[0];
	CanPkg.dat[5] = Lbyte.b[1];
	CanPkg.dat[6] = Lbyte.b[2];
	CanPkg.dat[7] = Lbyte.b[3];

	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}

static void notifyBaseMinMaxValue(void)
{
	uint8_t		bmu,posi;
	smp_can_package_t	CanPkg;
	tIbyte	Ibyte;
	
	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, notifyScuId(),
									SMP_BASE_MIN_MAX_VALUE_OBJ_INDEX,
									0);
	CanPkg.dlc = 8;
											
	Ibyte.i = halAfeGetMinCellVoltage(&bmu, &posi);
	CanPkg.dat[0] = Ibyte.b[0];
	CanPkg.dat[1] = Ibyte.b[1];
	CanPkg.dat[4] = bmu;
	CanPkg.dat[5] = posi;

	Ibyte.i = halAfeGetMaxCellVoltage(&bmu, &posi);
	CanPkg.dat[2] = Ibyte.b[0];
	CanPkg.dat[3] = Ibyte.b[1];
	CanPkg.dat[6] = bmu;
	CanPkg.dat[7] = posi;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	//----------------------------------------
	//	MinT & Max T
	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, notifyScuId(),
									SMP_BASE_MIN_MAX_VALUE_OBJ_INDEX,
									1);
	CanPkg.dlc = 8;
		
	Ibyte.i = HalAfeGetMinNtcTemp(&bmu, &posi);
	CanPkg.dat[0] = Ibyte.b[0];
	CanPkg.dat[1] = Ibyte.b[1];
	CanPkg.dat[4] = bmu;
	CanPkg.dat[5] = posi;

	Ibyte.i = HalAfeGetMaxNtcTemp(&bmu, &posi);
	CanPkg.dat[2] = Ibyte.b[0];
	CanPkg.dat[3] = Ibyte.b[1];
	CanPkg.dat[6] = bmu;
	CanPkg.dat[7] = posi;
	
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}

static void notifyBaseScuTemp(void)
{
	smp_can_package_t	CanPkg;
	tIbyte	Ibyte;
	uint8_t		u8;
	
	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, notifyScuId(),
									SMP_BASE_SCU_NTC_TEMP_OBJ_INDEX,
									0);
	CanPkg.dlc = 0;
	for(u8=0; u8<5; u8++)
	{				
		Ibyte.i = apiScuTempGetTemperature(u8);
		CanPkg.dat[CanPkg.dlc++] = Ibyte.b[0];
		CanPkg.dat[CanPkg.dlc++] = Ibyte.b[1];
		if(CanPkg.dlc >= 8)
		{
			appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
			CanPkg.dlc = 0;
			CanPkg.id += 4;
		}
	}
	if(CanPkg.dlc)
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}

static void notifyBaseQStartPackage(void)
{
	smp_can_package_t	CanPkg;
	tLbyte	Lbyte;
	tIbyte	Ibyte;
	uint8_t		u8;
	
	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, notifyScuId(),
									SMP_BASE_QSTART_RSOC_SOH_OBJ_INDEX,
									0);
	CanPkg.dlc = 8;
	
	Lbyte.l = appGaugeGetQStart();
	CanPkg.dat[0] = Lbyte.b[0];
	CanPkg.dat[1] = Lbyte.b[1];
	CanPkg.dat[2] = Lbyte.b[2];
	CanPkg.dat[3] = Lbyte.b[3];
	
	Ibyte.i = appGaugeGetRSoc();
	CanPkg.dat[4] = Ibyte.b[0];
	CanPkg.dat[5] = Ibyte.b[1];
	
	Ibyte.i=appGaugeGetSOH();
	CanPkg.dat[6] = Ibyte.b[0];
	CanPkg.dat[7] = Ibyte.b[1];

	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}

static void notifyBaseRamSocPackage(void)
{
	smp_can_package_t	CanPkg;
	tLbyte	Lbyte;
	tIbyte	Ibyte;
	uint8_t		u8;
	
	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, notifyScuId(),
									SMP_BASE_RAMSOC_ENDSOC_DSOC_SOC0_OBJ_INDEX,
									0);
	CanPkg.dlc = 8;
	
	Ibyte.i = appGaugeGetRamSoc();
	CanPkg.dat[0] = Ibyte.b[0];
	CanPkg.dat[1] = Ibyte.b[1];
	
	Ibyte.i = appGaugeGetEndOfSoc();
	CanPkg.dat[2] = Ibyte.b[0];
	CanPkg.dat[3] = Ibyte.b[1];
	

	Ibyte.i = appGaugeGetDisplaySoc();
	CanPkg.dat[4] = Ibyte.b[0];
	CanPkg.dat[5] = Ibyte.b[1];

	Ibyte.i = appGaugeGetSoc0();
	CanPkg.dat[6] = Ibyte.b[0];
	CanPkg.dat[7] = Ibyte.b[1];
	
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}

static void notifyBasePassCharge(void)
{
	smp_can_package_t	CanPkg;
	tLbyte	Lbyte;
	uint8_t		u8;
	
	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, notifyScuId(),
									SMP_BASE_QPASS_RPASS_OBJ_INDEX,
									0);
	CanPkg.dlc = 8;
	
	Lbyte.sl = appGaugeGetQPassCharge();
	CanPkg.dat[0] = Lbyte.b[0];
	CanPkg.dat[1] = Lbyte.b[1];
	CanPkg.dat[2] = Lbyte.b[2];
	CanPkg.dat[3] = Lbyte.b[3];
	
	Lbyte.sl = appGaugeGetRPassCharge();
	CanPkg.dat[4] = Lbyte.b[0];
	CanPkg.dat[5] = Lbyte.b[1];
	CanPkg.dat[6] = Lbyte.b[2];
	CanPkg.dat[7] = Lbyte.b[3];
	
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}


static void notifyBaseCycleCount(void)
{
	smp_can_package_t	CanPkg;
	tIbyte	Ibyte;
	
	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, notifyScuId(),
									SMP_BASE_CYCLE_COUNT_OBJ_INDEX,
									0);
	CanPkg.dlc = 2;
	
	Ibyte.i = appGaugeGetCyleCount();
	CanPkg.dat[0] = Ibyte.b[0];
	CanPkg.dat[1] = Ibyte.b[1];
	
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}
static void notifyBaseInfoPackage(void)
{
	notifyBaseScuId();
	notifyBaseSystemFlag();
	notifyBaseRmQmax();
	notifyBaseCurrent();
	notifyBaseFCC();
	notifyBaseVBatVoltage();
	notifyBaseMinMaxValue();
	notifyBaseScuTemp();
	notifyBaseQStartPackage();
	notifyBaseRamSocPackage();
	notifyBasePassCharge();
	notifyBaseCycleCount();
	
	notifyNextFunction();
}


static void appSerialCanDavinciNotificationCellVoltage(void)
{
	tIbyte		Ibyte;
	uint8_t		DatIndex;
	uint8_t		pkgnum = 0;
	smp_can_package_t	CanPkg;

	while(NotificationSubIndex < notifyCellNumber())
	{
		if((NotificationSubIndex & 0x03) == 0)
		{
			CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_DETAIL_TX, notifyScuId(),
									SMP_DETAIL_CELL_VOLTAGE_OBJ_INDEX,
									NotificationSubIndex);
			memset(&CanPkg.dat, 0, 8);
			DatIndex = 0;
		}
		Ibyte.i = halAfeGetCellVoltage(NotificationSubIndex++);

		CanPkg.dat[DatIndex++] = Ibyte.b[0];
		CanPkg.dat[DatIndex++] = Ibyte.b[1];
		if(DatIndex >= 8)
		{
			DatIndex = 0;
			CanPkg.dlc = 8;
			appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
			pkgnum++;
			if(pkgnum >= 30)
				break;
		}
	}
	if(DatIndex)
	{
		CanPkg.dlc = 8;
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	}
	if(NotificationSubIndex >= notifyCellNumber())
	{
		NotificationSubIndex = 0;
		//appSerialCanDavinciSendTextMessage("Notification Cell Voltage finish");
		notifyNextFunction();
	}
}

static void appSerialCanDavinciNotificationNtcVoltage(void)
{
	tIbyte		Ibyte;
	uint8_t		DatIndex;
	uint8_t		pkgnum = 0;
	smp_can_package_t	CanPkg;


	while(NotificationSubIndex < notifyNtcNumber())
	{
		if((NotificationSubIndex & 0x03) == 0)
		{
			CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_DETAIL_TX, notifyScuId(),
									SMP_DETAIL_CELL_NTC_VOLTAGE_OBJ_INDEX,
									NotificationSubIndex);
			memset(&CanPkg.dat, 0, 8);
			DatIndex = 0;
		}
		Ibyte.i = HalAfeGetNtcVoltage(NotificationSubIndex++);

		CanPkg.dat[DatIndex++] = Ibyte.b[0];
		CanPkg.dat[DatIndex++] = Ibyte.b[1];
		if(DatIndex >= 8)
		{
			DatIndex = 0;
			CanPkg.dlc = 8;
			appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
			pkgnum++;
			//if(pkgnum >= 30)
			//	break;
		}
	}
	if(DatIndex)
	{
		CanPkg.dlc = 8;
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	}
	if(NotificationSubIndex >= notifyNtcNumber())
	{
		NotificationSubIndex = 0;
		//appSerialCanDavinciSendTextMessage("Notification Ntc Voltage finish");
		notifyNextFunction();
	}
}

static void appSerialCanDavinciNotificationOvpFlag(void)
{
	tIbyte		Ibyte;
	uint8_t		DatIndex;
	uint8_t		pkgnum = 0;
	smp_can_package_t	CanPkg;

	while(NotificationSubIndex < notifyCellNumber())
	{
		if((NotificationSubIndex&0x07) == 0)
		{
			CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_DETAIL_TX, notifyScuId(),
									SMP_DETAIL_OVP_FLAG_OBJ_INDEX,
									NotificationSubIndex);
			memset(&CanPkg.dat, 0, 8);
			DatIndex = 0;
		}
		CanPkg.dat[DatIndex] = apiProtectOvpGetFlag(NotificationSubIndex);
		if(appBalanceIsBalanceSet(NotificationSubIndex))
			CanPkg.dat[DatIndex] |= 0x80;
					
		NotificationSubIndex++;
		DatIndex++;
		if(DatIndex >= 8)
		{
			DatIndex = 0;
			CanPkg.dlc = 8;
			appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
			pkgnum++;
			if(pkgnum >= 30)
				break;
		}
	}
	if(DatIndex)
	{
		CanPkg.dlc = 8;
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	}
	if(NotificationSubIndex >= notifyCellNumber())
	{
		NotificationSubIndex = 0;
		notifyNextFunction();
	}
}
static void appSerialCanDavinciNotificationUvpFlag(void)
{
	uint8_t		DatIndex;
	uint8_t		pkgnum = 0;
	smp_can_package_t	CanPkg;

	while(NotificationSubIndex < notifyCellNumber())
	{
		if((NotificationSubIndex&0x07) == 0)
		{
			CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_DETAIL_TX, notifyScuId(),
									SMP_DETAIL_UVP_FLAG_OBJ_INDEX,
									NotificationSubIndex);
			memset(&CanPkg.dat, 0, 8);
			DatIndex = 0;
		}
		CanPkg.dat[DatIndex++] = apiProtectUvpGetFlag(NotificationSubIndex++);
		if(DatIndex >= 8)
		{
			DatIndex = 0;
			CanPkg.dlc = 8;
			appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
			pkgnum++;
			if(pkgnum >= 30)
				break;
		}
	}
	if(DatIndex)
	{
		CanPkg.dlc = 8;
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	}
	if(NotificationSubIndex >= notifyCellNumber())
	{
		NotificationSubIndex = 0;
		notifyNextFunction();
	}
}
static void appSerialCanDavinciNotificationCotpFlag(void)
{
	uint8_t		DatIndex;
	uint8_t		pkgnum = 0;
	smp_can_package_t	CanPkg;

	while(NotificationSubIndex < notifyNtcNumber())
	{
		if((NotificationSubIndex & 0x07) == 0)
		{
			CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_DETAIL_TX, notifyScuId(),
									SMP_DETAIL_COTP_FLAG_OBJ_INDEX,
									NotificationSubIndex);
			memset(&CanPkg.dat, 0, 8);
			DatIndex = 0;
		}
		CanPkg.dat[DatIndex++] = apiProtectCotpGetFlag(NotificationSubIndex++);
		if(DatIndex >= 8)
		{
			DatIndex = 0;
			CanPkg.dlc = 8;
			appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
			pkgnum++;
			if(pkgnum >= 30)
				break;
		}
	}
	if(DatIndex)
	{
		CanPkg.dlc = 8;
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	}
	if(NotificationSubIndex >= notifyNtcNumber())
	{
		NotificationSubIndex = 0;
		notifyNextFunction();
	}
}
static void appSerialCanDavinciNotificationCutpFlag(void)
{
	uint8_t		DatIndex;
	uint8_t		pkgnum = 0;
	smp_can_package_t	CanPkg;

	while(NotificationSubIndex < notifyNtcNumber())
	{
		if((NotificationSubIndex & 0x07) == 0)
		{
			CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_DETAIL_TX, notifyScuId(),
									SMP_DETAIL_CUTP_FLAG_OBJ_INDEX,
									NotificationSubIndex);
			memset(&CanPkg.dat, 0, 8);
			DatIndex = 0;
		}
		CanPkg.dat[DatIndex++] = apiProtectCutpGetFlag(NotificationSubIndex++);
		if(DatIndex >= 8)
		{
			DatIndex = 0;
			CanPkg.dlc = 8;
			appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
			pkgnum++;
			if(pkgnum >= 30)
				break;
		}
	}
	if(DatIndex)
	{
		CanPkg.dlc = 8;
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	}
	if(NotificationSubIndex >= notifyNtcNumber())
	{
		NotificationSubIndex = 0;
		notifyNextFunction();
	}
}
static void appSerialCanDavinciNotificationDotpFlag(void)
{
	uint8_t		DatIndex;
	uint8_t		pkgnum = 0;
	smp_can_package_t	CanPkg;

	while(NotificationSubIndex < notifyNtcNumber())
	{
		if((NotificationSubIndex & 0x07) == 0)
		{
			CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_DETAIL_TX, notifyScuId(),
									SMP_DETAIL_DOTP_FLAG_OBJ_INDEX,
									NotificationSubIndex);
			memset(&CanPkg.dat, 0, 8);
			DatIndex = 0;
		}
		CanPkg.dat[DatIndex++] = apiProtectDotpGetFlag(NotificationSubIndex++);
		if(DatIndex >= 8)
		{
			DatIndex = 0;
			CanPkg.dlc = 8;
			appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
			pkgnum++;
			if(pkgnum >= 30)
				break;
		}
	}
	if(DatIndex)
	{
		CanPkg.dlc = 8;
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	}
	if(NotificationSubIndex >= notifyNtcNumber())
	{
		NotificationSubIndex = 0;
		notifyNextFunction();
	}
}
static void appSerialCanDavinciNotificationDutpFlag(void)
{
	uint8_t		DatIndex;
	uint8_t		pkgnum = 0;
	smp_can_package_t	CanPkg;

	while(NotificationSubIndex < notifyNtcNumber())
	{
		if((NotificationSubIndex & 0x07) == 0)
		{
			CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_DETAIL_TX, notifyScuId(),
									SMP_DETAIL_DUTP_FLAG_OBJ_INDEX,
									NotificationSubIndex);
			memset(&CanPkg.dat, 0, 8);
			DatIndex = 0;
		}
		CanPkg.dat[DatIndex++] = apiProtectDutpGetFlag(NotificationSubIndex++);
		if(DatIndex >= 8)
		{
			DatIndex = 0;
			CanPkg.dlc = 8;
			appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
			pkgnum++;
			if(pkgnum >= 30)
				break;
		}
	}
	if(DatIndex)
	{
		CanPkg.dlc = 8;
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	}
	if(NotificationSubIndex >= notifyNtcNumber())
	{
		NotificationSubIndex = 0;
		notifyNextFunction();
	}
}

static void appSerialCanDavinciNotificationEnd(void)
{
	//appSerialCanDavinciSendTextMessage("Notification End");

	CanDavinciNotificationFunIndex = 0;
	NotificationSubIndex = 0;
}


const tNotificationRunTable	NotificationFunctionTable[]={
	notifyFunNone,
	
	notifyBaseInfoPackage,
	appSerialCanDavinciNotificationCellVoltage,
	appSerialCanDavinciNotificationNtcVoltage,
	
	appSerialCanDavinciNotificationOvpFlag,
	appSerialCanDavinciNotificationUvpFlag,
	appSerialCanDavinciNotificationCotpFlag,
	appSerialCanDavinciNotificationCutpFlag,
	appSerialCanDavinciNotificationDotpFlag,
	appSerialCanDavinciNotificationDutpFlag,

	appSerialCanDavinciNotificationEnd
	
};

/* Public function prototypes -----------------------------------------------*/

void appSerialCanDavinciNotificationHandler(uint16_t evt)
{
	static	uint8_t	count = 0;
	
	if(appBmsIsScudIdReady()  == 0)
		return;
	
	if(CanDavinciNotificationFunIndex)
		NotificationFunctionTable[CanDavinciNotificationFunIndex]();
	
	count++;
	if(count >= 100)
	{
		count = 0;
	//	appSerialCanDavinciSendTextMessage("Notification Start");
		if(!CanDavinciNotificationFunIndex)
		{
			if(notifyScuId() == 0xff)
				return;			
			CanDavinciNotificationFunIndex = 1;
			NotificationSubIndex = 0;
		}
	}
}



/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    




/**
  ******************************************************************************
  * @file        AppSerialCanDavinciParameter.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/12/02
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
#include "ApiSysPar.h"
#include "ApiSysPar.h"


void appSerialUartSendMessage(uint8_t *str);
#define	appSerialCanDavinciParDebugMsg(str)	//appSerialCanDavinciSendTextMessage(str)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define	canParScuId()	appProjectGetScuId()

/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t	IdleCount = 0;

static uint8_t	StringMessageBuffer[MAX_NOTE_MESSAGE_STRING_ITEM + 2];


/* Private function prototypes -----------------------------------------------*/
static uint8_t isParWritable(void)
{
	return IdleCount;
}

static void magicCodeIdleSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
		return;
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		if(IdleCount == 0)
			LibSwTimerClose(magicCodeIdleSwTimerHandler, 0);
		else
		{
			IdleCount--;
			if(IdleCount == 0)
			{
				LibSwTimerClose(magicCodeIdleSwTimerHandler, 0);
				appSerialCanDavinciParDebugMsg("Close Magic Time");
			}
		}
	}
}

static void DavinciParameterMagicCode(smp_can_package_t *pCanPkg){
	smp_can_package_t	CanPkg;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_MAGIC_CODE);
		CanPkg.dlc = 1;
		CanPkg.dat[0] = IdleCount;
		
		appSerialCanDavinciParDebugMsg("Read Magic Code Idle Time");
	}
	else
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_MAGIC_CODE);
		CanPkg.dlc = 1;
		CanPkg.dat[0] = 0;
		if(memcmp(&pCanPkg->dat ,"EssWrPar", 8) == 0)
		{
			if(IdleCount == 0)
			{
				LibSwTimerOpen(magicCodeIdleSwTimerHandler, 0);
				appSerialCanDavinciParDebugMsg("Open Magic Time");
			}
			IdleCount = 10;
			CanPkg.dat[0] = 10;
		}
	}		
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}

static void DavinciCanParameterHwVersion(smp_can_package_t *pCanPkg){
	smp_can_package_t	CanPkg;
	tLbyte	version;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_HW_VERSION);
		CanPkg.dlc = 2;
		version.l = apiSysParGetHwVersion();
		CanPkg.dat[0] = version.b[0];
		CanPkg.dat[1] = version.b[1];
		
		//appSerialCanDavinciParDebugMsg("Read Hw Version");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_HW_VERSION);
		CanPkg.dlc = 0;

		version.l =  GET_WORD(&pCanPkg->dat[0]);
		apiSysParSetHwVersion(version.l);
		appSerialCanDavinciParDebugMsg("Write Hw Version");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}

static void DavinciCanParameterFwVersion(smp_can_package_t *pCanPkg){
	smp_can_package_t	CanPkg;
	tLbyte	version;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_FW_VERSION);
		CanPkg.dlc = 3;
		version.l = apiSysParGetFwVersion();
		CanPkg.dat[0] = version.b[0];
		CanPkg.dat[1] = version.b[1];
		CanPkg.dat[2] = version.b[2];
		
		appSerialCanDavinciParDebugMsg("Read Hw Version");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_FW_VERSION);
		CanPkg.dlc = 0;
		appSerialCanDavinciParDebugMsg("Can't write Hw Version");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);

}

static void DavinciCanParameterFwBuildDateTime(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tLbyte	version;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_FW_BUILD_DATE_TIME);
		CanPkg.dlc = 7;
		CanPkg.dat[3] = 0x20;
		CanPkg.dat[2] = 0x21;
		CanPkg.dat[1] = 0x10;
		CanPkg.dat[0] = 0x26;
		
		CanPkg.dat[6] = 0x17;
		CanPkg.dat[5] = 0x57;
		CanPkg.dat[4] = 0x26;
		
		appSerialCanDavinciParDebugMsg("Read Fw date time");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_FW_BUILD_DATE_TIME);
		CanPkg.dlc = 0;
		appSerialCanDavinciParDebugMsg("Can't write date time");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);

}

static void DavinciCanParameterZeroCurrent(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tIbyte	current;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_ZERO_CURRENT);
		CanPkg.dlc = 4;
		current.i = apiSysParGetZeroCurrentValue();
		CanPkg.dat[0] = current.b[0];
		CanPkg.dat[1] = current.b[1];
		current.i = apiSysParGetMinChargeCurrentValue();
		CanPkg.dat[2] = current.b[0];
		CanPkg.dat[3] = current.b[1];

		appSerialCanDavinciParDebugMsg("Read zero current");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_ZERO_CURRENT);
		CanPkg.dlc = 0;
		current.b[0] = pCanPkg->dat[0];
		current.b[1] = pCanPkg->dat[1];
		apiSysParSetZeroCurrentValue(current.i);
		
		current.b[0] = pCanPkg->dat[2];
		current.b[1] = pCanPkg->dat[3];
		apiSysParSetMinChargeCurrentValue(current.i);
		
		appSerialCanDavinciParDebugMsg("write zero current");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
}


static void DavinciCanParameterPreDischargeTime(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tIbyte	time;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_PRE_DHG_TIME);
		CanPkg.dlc = 2;
		time.i = apiSysParGetPreDischargeTime();
		CanPkg.dat[0] = time.b[0];
		CanPkg.dat[1] = time.b[1];
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_PRE_DHG_TIME);
									
		CanPkg.dlc = 0;
		time.b[0] = pCanPkg->dat[0];
		time.b[1] = pCanPkg->dat[1];
		apiSysParSetPreDischargeTime(time.i);
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
}

static void DavinciCanParameterRelayOnDiffVoltage(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tIbyte	voltage;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_MOS_ON_DIFF_VOLTAGE);
		CanPkg.dlc = 2;
		voltage.i = apiSysParGetRelayOnDiffVoltage();
		CanPkg.dat[0] = voltage.b[0];
		CanPkg.dat[1] = voltage.b[1];
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_MOS_ON_DIFF_VOLTAGE);
		CanPkg.dlc = 0;
		voltage.b[0] = pCanPkg->dat[0];
		voltage.b[1] = pCanPkg->dat[1];
		apiSysParSetRelayOnDiffVoltage(voltage.i);
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
}
                   			
static void DavinciCanParameterDesignedCapacity(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tLbyte	Lbyte;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_DESIGNED_CAPACITY);
		CanPkg.dlc = 4;
		Lbyte.l = apiSysParGetDesignedCapacity();
		
		CanPkg.dat[0] = Lbyte.b[0];
		CanPkg.dat[1] = Lbyte.b[1];
		CanPkg.dat[2] = Lbyte.b[2];
		CanPkg.dat[3] = Lbyte.b[3];

		appSerialCanDavinciParDebugMsg("Read designed capacity");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_DESIGNED_CAPACITY);
		CanPkg.dlc = 0;
		Lbyte.l = GET_DWORD(&pCanPkg->dat[0]);
		apiSysParSetDesignedCapacity(Lbyte.l);
		appSerialCanDavinciParDebugMsg("write designed capacity");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
}
            
static void DavinciCanParameterFullChargeCondition(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;
	
	

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_TAPER_CURRENT);
		CanPkg.dlc = 5;
		apiSysParGetFullChargeCondition(&ProtectPar);

		CanPkg.dat[0] = ProtectPar.SetValue.b[0];
		CanPkg.dat[1] = ProtectPar.SetValue.b[1];
		CanPkg.dat[2] = ProtectPar.STime.b[0];
		CanPkg.dat[3] = ProtectPar.STime.b[1];
		CanPkg.dat[4] = ProtectPar.RelValue.b[0];

		appSerialCanDavinciParDebugMsg("Read full charge condition");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_TAPER_CURRENT);																		
		CanPkg.dlc = 0;
		ProtectPar.SetValue.l = GET_WORD(&pCanPkg->dat[0]);
		ProtectPar.STime.l = GET_WORD(&pCanPkg->dat[2]);
		ProtectPar.RelValue.l = pCanPkg->dat[4];

		apiSysParSetFullChargeCondition(&ProtectPar);
		appSerialCanDavinciParDebugMsg("write full charge condition");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
}            

static void DavinciCanParameterFlatVoltage(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;
	
	

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_FLAT_VOLTAGE);
		CanPkg.dlc = 4;
		apiSysParGetFlatVoltage(&ProtectPar);

		CanPkg.dat[0] = ProtectPar.SetValue.b[0];
		CanPkg.dat[1] = ProtectPar.SetValue.b[1];
		CanPkg.dat[2] = ProtectPar.STime.b[0];
		CanPkg.dat[3] = ProtectPar.STime.b[1];
		appSerialCanDavinciParDebugMsg("Read flat voltage");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_FLAT_VOLTAGE);																		
		CanPkg.dlc = 0;
		ProtectPar.SetValue.l = GET_WORD(&pCanPkg->dat[0]);
		ProtectPar.STime.l = GET_WORD(&pCanPkg->dat[2]);

		apiSysParSetFlatVoltage(&ProtectPar);
		appSerialCanDavinciParDebugMsg("write flat voltage");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
}           
  
static void DavinciCanParameterTerminateVoltage(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tIbyte	voltage;
		
	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_TERMINATE_VOLTAGE);
		CanPkg.dlc = 2;
		voltage.i = apiSysParGetTerminateVoltage();

		CanPkg.dat[0] = voltage.b[0];
		CanPkg.dat[1] = voltage.b[1];
		appSerialCanDavinciParDebugMsg("Read terminate voltage");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_TERMINATE_VOLTAGE);																		
		CanPkg.dlc = 0;
		voltage.i = GET_WORD(&pCanPkg->dat[0]);
		apiSysParSetTerminateVoltage(voltage.i);
		appSerialCanDavinciParDebugMsg("write  terminate voltage");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
} 

static void DavinciCanParameterAfeCommunication(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;
		
	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_COMMUNICATION);
		CanPkg.dlc = 3;

		CanPkg.dat[0] = pCanPkg->dat[0];
		if(pCanPkg->dat[0] == 0x00)
		{
			apiSysParGetAfeCommTime(&ProtectPar);
			appSerialCanDavinciParDebugMsg("Read AFE comm.");			
			CanPkg.dat[1] = ProtectPar.SetValue.b[0];
			CanPkg.dat[2] = ProtectPar.STime.b[0];
		}
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_COMMUNICATION);																		
		CanPkg.dlc = 0;
		if(pCanPkg->dat[0] == 0x00)
		{
			ProtectPar.SetValue.l = pCanPkg->dat[1];
			ProtectPar.STime.l = pCanPkg->dat[2];
			apiSysParSetAfeCommTime(&ProtectPar);
			
			appSerialCanDavinciParDebugMsg("Write AFE comm.");			
		}
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
} 

static void DavinciCanParameterInsulationResistance(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;
		
	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_INSULATION_RESISTANCE);
		CanPkg.dlc = 2;
		
		apiSysParGetInsulationResistance(&ProtectPar);
		CanPkg.dat[0] = ProtectPar.SetValue.b[0];
		CanPkg.dat[1] = ProtectPar.STime.b[0];
		appSerialCanDavinciParDebugMsg("Read I.R.");			
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_INSULATION_RESISTANCE);																		
		CanPkg.dlc = 0;
		ProtectPar.SetValue.l = pCanPkg->dat[0];
		ProtectPar.STime.l = pCanPkg->dat[1];
		apiSysParSetInsulationResistance(&ProtectPar);
		appSerialCanDavinciParDebugMsg("Write I.R.");			
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
} 



static void DavinciCanParameterQmax(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tLbyte	Lbyte;
		
	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_QMAX);
		CanPkg.dlc = 4;
		Lbyte.l = appGaugeGetQmax();
		CanPkg.dat[0] = Lbyte.b[0];
		CanPkg.dat[1] = Lbyte.b[1];
		CanPkg.dat[2] = Lbyte.b[2];
		CanPkg.dat[3] = Lbyte.b[3];
		appSerialCanDavinciParDebugMsg("Read Qmax");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_QMAX);																		
		CanPkg.dlc = 0;
		Lbyte.l = GET_DWORD(&pCanPkg->dat[0]);
		appGaugeSetQmax(Lbyte.l);
		apiSysParSetQmax(Lbyte.l);

		appSerialCanDavinciParDebugMsg("write  Qmax");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
}                   			          

static void DavinciCanParameterRM(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tLbyte	Lbyte;
		
	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_RM);
		CanPkg.dlc = 4;
		Lbyte.l = appGaugeGetRM();
		CanPkg.dat[0] = Lbyte.b[0];
		CanPkg.dat[1] = Lbyte.b[1];
		CanPkg.dat[2] = Lbyte.b[2];
		CanPkg.dat[3] = Lbyte.b[3];
		appSerialCanDavinciParDebugMsg("Read Rm");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_RM);																		
		CanPkg.dlc = 0;
		Lbyte.l = GET_DWORD(&pCanPkg->dat[0]);
		appGaugeSetRM(Lbyte.l);

		appSerialCanDavinciParDebugMsg("write Rm");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
}   

                    			
static void DavinciCanParameterOcvTable(smp_can_package_t *pCanPkg)
{
	
	smp_can_package_t	CanPkg;
	tOcvRaTable			OcvTable;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_OCV_TABLE);
		CanPkg.dlc = 5;
		apiSysParGetOcvTable(pCanPkg->dat[1], &OcvTable);
		CanPkg.dat[0] = pCanPkg->dat[0];
		CanPkg.dat[1] = pCanPkg->dat[1];
		CanPkg.dat[2] = OcvTable.Level;
		CanPkg.dat[3] = OcvTable.Value % 0x100;
		CanPkg.dat[4] = OcvTable.Value / 0x100;		
		
		appSerialCanDavinciParDebugMsg("Read Ocv Table");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_OCV_TABLE);
		CanPkg.dlc = 0;


		OcvTable.Level = pCanPkg->dat[2];
		OcvTable.Value = GET_WORD(&pCanPkg->dat[3]);
		apiSysParSetOcvTable(pCanPkg->dat[1], &OcvTable);
		
		appSerialCanDavinciParDebugMsg("Write Ocv Table");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
}

static void DavinciCanParameterRaTable(smp_can_package_t *pCanPkg){
	
	smp_can_package_t	CanPkg;
	tOcvRaTable			OcvTable;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_RA_TABLE);
		CanPkg.dlc = 5;
		apiSysParGetRaTable(pCanPkg->dat[1], &OcvTable);
		CanPkg.dat[0] = pCanPkg->dat[0];
		CanPkg.dat[1] = pCanPkg->dat[1];
		CanPkg.dat[2] = OcvTable.Level;
		CanPkg.dat[3] = OcvTable.Value % 0x100;
		CanPkg.dat[4] = OcvTable.Value / 0x100;		
		
		appSerialCanDavinciParDebugMsg("Read Ra Table");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_RA_TABLE);
		CanPkg.dlc = 0;


		OcvTable.Level = pCanPkg->dat[2];
		OcvTable.Value = GET_WORD(&pCanPkg->dat[3]);
		apiSysParSetRaTable(pCanPkg->dat[1], &OcvTable);
		
		appSerialCanDavinciParDebugMsg("Write Ra Table");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
}
//---------------------------------
//Protect
static void DavinciCanParameterOvp(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_OVP_PROTECT);
				
		CanPkg.dlc = 7;
		CanPkg.dat[0] = pCanPkg->dat[0];		
		apiSysParGetOvpPar(pCanPkg->dat[0], &ProtectPar);
		CanPkg.dat[1] = ProtectPar.SetValue.b[0];
		CanPkg.dat[2] = ProtectPar.SetValue.b[1];
		CanPkg.dat[3] = ProtectPar.STime.b[0];
		CanPkg.dat[4] = ProtectPar.RelValue.b[0];
		CanPkg.dat[5] = ProtectPar.RelValue.b[1];
		CanPkg.dat[6] = ProtectPar.RTime.b[0];

		appSerialCanDavinciParDebugMsg("Read Ovp");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_OVP_PROTECT);
		CanPkg.dlc = 0;
		ProtectPar.SetValue.l = GET_WORD(&pCanPkg->dat[1]);
		ProtectPar.STime.l = pCanPkg->dat[3];
		ProtectPar.RelValue.l = GET_WORD(&pCanPkg->dat[4]);
		ProtectPar.RTime.l = pCanPkg->dat[6];

		apiSysParSetOvpPar(pCanPkg->dat[0], &ProtectPar);
		appSerialCanDavinciParDebugMsg("Write Ovp");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}
static void DavinciCanParameterUvp(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_UVP_PROTECT);
				
		CanPkg.dlc = 7;
		CanPkg.dat[0] = pCanPkg->dat[0];		
		apiSysParGetUvpPar(pCanPkg->dat[0], &ProtectPar);
		CanPkg.dat[1] = ProtectPar.SetValue.b[0];
		CanPkg.dat[2] = ProtectPar.SetValue.b[1];
		CanPkg.dat[3] = ProtectPar.STime.b[0];
		CanPkg.dat[4] = ProtectPar.RelValue.b[0];
		CanPkg.dat[5] = ProtectPar.RelValue.b[1];
		CanPkg.dat[6] = ProtectPar.RTime.b[0];

		appSerialCanDavinciParDebugMsg("Read Uvp");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_UVP_PROTECT);
		CanPkg.dlc = 0;
		ProtectPar.SetValue.l = GET_WORD(&pCanPkg->dat[1]);
		ProtectPar.STime.l = pCanPkg->dat[3];
		ProtectPar.RelValue.l = GET_WORD(&pCanPkg->dat[4]);
		ProtectPar.RTime.l = pCanPkg->dat[6];

		apiSysParSetUvpPar(pCanPkg->dat[0], &ProtectPar);
		appSerialCanDavinciParDebugMsg("Write Uvp");
	}
	else
		return;

	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}
static void DavinciCanParameterCotp(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_COTP_PROTECT);
				
		CanPkg.dlc = 5;
		CanPkg.dat[0] = pCanPkg->dat[0];		
		apiSysParGetCotpPar(pCanPkg->dat[0], &ProtectPar);
		CanPkg.dat[1] = ProtectPar.SetValue.b[0];
		CanPkg.dat[2] = ProtectPar.STime.b[0];
		CanPkg.dat[3] = ProtectPar.RelValue.b[0];
		CanPkg.dat[4] = ProtectPar.RTime.b[0];

		appSerialCanDavinciParDebugMsg("Read Cotp");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_COTP_PROTECT);
		CanPkg.dlc = 0;
		ProtectPar.SetValue.l = pCanPkg->dat[1];
		ProtectPar.STime.l = pCanPkg->dat[2];
		ProtectPar.RelValue.l = pCanPkg->dat[3];
		ProtectPar.RTime.l = pCanPkg->dat[4];

		apiSysParSetCotpPar(pCanPkg->dat[0], &ProtectPar);
		appSerialCanDavinciParDebugMsg("Write Cotp");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}
static void DavinciCanParameterCutp(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_CUTP_PROTECT);
				
		CanPkg.dlc = 5;
		CanPkg.dat[0] = pCanPkg->dat[0];		
		apiSysParGetCutpPar(pCanPkg->dat[0], &ProtectPar);
		CanPkg.dat[1] = ProtectPar.SetValue.b[0];
		CanPkg.dat[2] = ProtectPar.STime.b[0];
		CanPkg.dat[3] = ProtectPar.RelValue.b[0];
		CanPkg.dat[4] = ProtectPar.RTime.b[0];

		appSerialCanDavinciParDebugMsg("Read Cutp");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_CUTP_PROTECT);
		CanPkg.dlc = 0;
		ProtectPar.SetValue.l = pCanPkg->dat[1];
		ProtectPar.STime.l = pCanPkg->dat[2];
		ProtectPar.RelValue.l = pCanPkg->dat[3];
		ProtectPar.RTime.l = pCanPkg->dat[4];

		apiSysParSetCutpPar(pCanPkg->dat[0], &ProtectPar);
		appSerialCanDavinciParDebugMsg("Write Cutp");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}

static void DavinciCanParameterDotp(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_DOTP_PROTECT);
				
		CanPkg.dlc = 5;
		CanPkg.dat[0] = pCanPkg->dat[0];		
		apiSysParGetDotpPar(pCanPkg->dat[0], &ProtectPar);
		CanPkg.dat[1] = ProtectPar.SetValue.b[0];
		CanPkg.dat[2] = ProtectPar.STime.b[0];
		CanPkg.dat[3] = ProtectPar.RelValue.b[0];
		CanPkg.dat[4] = ProtectPar.RTime.b[0];

		appSerialCanDavinciParDebugMsg("Read Dotp");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_DOTP_PROTECT);
		CanPkg.dlc = 0;
		ProtectPar.SetValue.l = pCanPkg->dat[1];
		ProtectPar.STime.l = pCanPkg->dat[2];
		ProtectPar.RelValue.l = pCanPkg->dat[3];
		ProtectPar.RTime.l = pCanPkg->dat[4];

		apiSysParSetDotpPar(pCanPkg->dat[0], &ProtectPar);
		appSerialCanDavinciParDebugMsg("Write Dotp");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
}
static void DavinciCanParameterDutp(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_DUTP_PROTECT);
				
		CanPkg.dlc = 5;
		CanPkg.dat[0] = pCanPkg->dat[0];		
		apiSysParGetDutpPar(pCanPkg->dat[0], &ProtectPar);
		CanPkg.dat[1] = ProtectPar.SetValue.b[0];
		CanPkg.dat[2] = ProtectPar.STime.b[0];
		CanPkg.dat[3] = ProtectPar.RelValue.b[0];
		CanPkg.dat[4] = ProtectPar.RTime.b[0];

		appSerialCanDavinciParDebugMsg("Read Dutp");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_DUTP_PROTECT);
		CanPkg.dlc = 0;
		ProtectPar.SetValue.l = pCanPkg->dat[1];
		ProtectPar.STime.l = pCanPkg->dat[2];
		ProtectPar.RelValue.l = pCanPkg->dat[3];
		ProtectPar.RTime.l = pCanPkg->dat[4];

		apiSysParSetDutpPar(pCanPkg->dat[0], &ProtectPar);
		appSerialCanDavinciParDebugMsg("Write Dutp");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}
static void DavinciCanParameterDtp(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_DTP_PROTECT);
				
		CanPkg.dlc = 5;
		CanPkg.dat[0] = pCanPkg->dat[0];		
		
		apiSysParGetDtpPar(pCanPkg->dat[0], &ProtectPar);
		
		CanPkg.dat[1] = ProtectPar.SetValue.b[0];
		CanPkg.dat[2] = ProtectPar.STime.b[0];
		CanPkg.dat[3] = ProtectPar.RelValue.b[0];
		CanPkg.dat[4] = ProtectPar.RTime.b[0];

		appSerialCanDavinciParDebugMsg("Read Dtp");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_DTP_PROTECT);
		CanPkg.dlc = 0;
		ProtectPar.SetValue.l = pCanPkg->dat[1];
		ProtectPar.STime.l = pCanPkg->dat[2];
		ProtectPar.RelValue.l = pCanPkg->dat[3];
		ProtectPar.RTime.l = pCanPkg->dat[4];

		apiSysParSetDtpPar(pCanPkg->dat[0], &ProtectPar);
		appSerialCanDavinciParDebugMsg("Write Dtp");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	
}

static void DavinciCanParameterCocp(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_COCP_PROTECT);
				
		CanPkg.dlc = 7;
		CanPkg.dat[0] = pCanPkg->dat[0];		
		apiSysParGetCocpPar(pCanPkg->dat[0], &ProtectPar);
		CanPkg.dat[1] = ProtectPar.SetValue.b[0];
		CanPkg.dat[2] = ProtectPar.SetValue.b[1];
		CanPkg.dat[3] = ProtectPar.STime.b[0];
		CanPkg.dat[4] = ProtectPar.RelValue.b[0];
		CanPkg.dat[5] = ProtectPar.RelValue.b[1];
		CanPkg.dat[6] = ProtectPar.RTime.b[0];

		appSerialCanDavinciParDebugMsg("Read Cocp");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_COCP_PROTECT);
		CanPkg.dlc = 0;
		ProtectPar.SetValue.l = GET_WORD(&pCanPkg->dat[1]);
		ProtectPar.STime.l = pCanPkg->dat[3];
		ProtectPar.RelValue.l = GET_WORD(&pCanPkg->dat[4]);
		ProtectPar.RTime.l = pCanPkg->dat[6];

		apiSysParSetCocpPar(pCanPkg->dat[0], &ProtectPar);
		appSerialCanDavinciParDebugMsg("Write Cocp");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
}

static void DavinciCanParameterDocp(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_DOCP_PROTECT);
				
		CanPkg.dlc = 7;
		CanPkg.dat[0] = pCanPkg->dat[0];		
		apiSysParGetDocpPar(pCanPkg->dat[0], &ProtectPar);
		CanPkg.dat[1] = ProtectPar.SetValue.b[0];
		CanPkg.dat[2] = ProtectPar.SetValue.b[1];
		CanPkg.dat[3] = ProtectPar.STime.b[0];
		CanPkg.dat[4] = ProtectPar.RelValue.b[0];
		CanPkg.dat[5] = ProtectPar.RelValue.b[1];
		CanPkg.dat[6] = ProtectPar.RTime.b[0];

		appSerialCanDavinciParDebugMsg("Read Docp");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_DOCP_PROTECT);
		CanPkg.dlc = 0;
		ProtectPar.SetValue.l = GET_WORD(&pCanPkg->dat[1]);
		ProtectPar.STime.l = pCanPkg->dat[3];
		ProtectPar.RelValue.l = GET_WORD(&pCanPkg->dat[4]);
		ProtectPar.RTime.l = pCanPkg->dat[6];

		apiSysParSetDocpPar(pCanPkg->dat[0], &ProtectPar);
		appSerialCanDavinciParDebugMsg("Write Docp");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}

static void DavinciCanParameterPF(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_PF_PROTECT);				
		CanPkg.dlc = 4;
		CanPkg.dat[0] = pCanPkg->dat[0];		
		if(CanPkg.dat[0] == 0)
		{
			apiSysParGetOvpPfPar(&ProtectPar);
			appSerialCanDavinciParDebugMsg("Read OVF PF");
		}
		else if(CanPkg.dat[0] == 1)
		{
			apiSysParGetUvpPfPar(&ProtectPar);
			appSerialCanDavinciParDebugMsg("Read UVF PF");
		}
		CanPkg.dat[1] = ProtectPar.SetValue.b[0];
		CanPkg.dat[2] = ProtectPar.SetValue.b[1];
		CanPkg.dat[3] = ProtectPar.STime.b[0];
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_PF_PROTECT);
		CanPkg.dlc = 1;
		CanPkg.dat[0] = pCanPkg->dat[0];
		ProtectPar.SetValue.l = GET_WORD(&pCanPkg->dat[1]);
		ProtectPar.STime.l = pCanPkg->dat[3];

		if(pCanPkg->dat[0] == 0)
		{
			apiSysParSetOvpPfPar(&ProtectPar);
			appSerialCanDavinciParDebugMsg("Write OVF PF");
		}
		else if(pCanPkg->dat[0] == 1)
		{
			apiSysParSetUvpPfPar(&ProtectPar);
			appSerialCanDavinciParDebugMsg("Write UVF PF");
		}
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}
//---------------------------------

static void DavinciParameterBmuNumber(smp_can_package_t *pCanPkg){
	
	smp_can_package_t	CanPkg;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_BMU_NUMBER);
		CanPkg.dlc = 1;
		CanPkg.dat[0] = apiSysParGetBmuNumber();
		appSerialCanDavinciParDebugMsg("Read BMU Number");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_BMU_NUMBER);
		CanPkg.dlc = 0;

		apiSysParSetBmuNumber(pCanPkg->dat[0]);

		appSerialCanDavinciParDebugMsg("Write BMU Number");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
}
static void DavinciParameterCellNtcFlag(smp_can_package_t *pCanPkg){
	
	tLbyte	Lbyte;
	smp_can_package_t	CanPkg;
	
	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		appSerialCanDavinciParDebugMsg("Read Cell Ntc Flag");
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_CELL_NTC_FLAG);
		CanPkg.dlc = 8;
		CanPkg.dat[0] = pCanPkg->dat[0];
		Lbyte.l = apiSysParGetCellFlag(pCanPkg->dat[0]);
		CanPkg.dat[1] = Lbyte.b[0];
		CanPkg.dat[2] = Lbyte.b[1];
		CanPkg.dat[3] = Lbyte.b[2];
		CanPkg.dat[4] = Lbyte.b[3];
		Lbyte.l = apiSysParGetNtcFlag(pCanPkg->dat[0]);
		CanPkg.dat[5] = Lbyte.b[0];
		CanPkg.dat[6] = Lbyte.b[1];
		CanPkg.dat[7] = Lbyte.b[2];
	}
	else if(isParWritable())
	{
		appSerialCanDavinciParDebugMsg("Write Cell Ntc Flag");
		
		Lbyte.l = GET_DWORD(&pCanPkg->dat[1]);
		apiSysParSetCellFlag(pCanPkg->dat[0], Lbyte.l);
		
		Lbyte.l = GET_DWORD(&pCanPkg->dat[5]);
		Lbyte.b[3] = 0;
		apiSysParSetNtcFlag(pCanPkg->dat[0], Lbyte.l);
		
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_CELL_NTC_FLAG);
		CanPkg.dlc = 0;
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
}

static void DavinciCanParameterBalanceDuty(smp_can_package_t *pCanPkg)
{
	tLbyte	Lbyte;
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;
	
	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		appSerialCanDavinciParDebugMsg("Read Balance Duty");
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_BALANCE_DUTY);
		CanPkg.dlc = 4;
		apiSysParGetBalanceDuty(&ProtectPar);
		CanPkg.dat[0] = ProtectPar.SetValue.b[0];
		CanPkg.dat[1] = ProtectPar.STime.b[0];
		CanPkg.dat[2] = ProtectPar.RelValue.b[0];
		CanPkg.dat[3] = ProtectPar.RTime.b[0];
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_BALANCE_DUTY);

		CanPkg.dlc = 0;
		
		ProtectPar.SetValue.l = pCanPkg->dat[0];
		ProtectPar.STime.l = pCanPkg->dat[1];
		ProtectPar.RelValue.l = pCanPkg->dat[2];
		ProtectPar.RTime.l = pCanPkg->dat[3];
		apiSysParSetBalanceDuty(&ProtectPar);
		
		appSerialCanDavinciParDebugMsg("Write Balance Duty");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}

static void DavinciCanParameterBalanceChg(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;
	
	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		appSerialCanDavinciParDebugMsg("Read Balance Chg");
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_BALANCE_CHG);
		CanPkg.dlc = 8;
		apiSysParGetBalanceChg(&ProtectPar);
		CanPkg.dat[0] = ProtectPar.SetValue.b[0];
		CanPkg.dat[1] = ProtectPar.SetValue.b[1];
		CanPkg.dat[2] = ProtectPar.STime.b[0];
		CanPkg.dat[3] = ProtectPar.STime.b[1];
		CanPkg.dat[4] = ProtectPar.RelValue.b[0];
		CanPkg.dat[5] = ProtectPar.RelValue.b[1];
		CanPkg.dat[6] = ProtectPar.RTime.b[0];
		CanPkg.dat[7] = ProtectPar.RTime.b[1];
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_BALANCE_CHG);

		CanPkg.dlc = 0;
		
		ProtectPar.SetValue.l = GET_WORD(&pCanPkg->dat[0]);
		ProtectPar.STime.l = GET_WORD(&pCanPkg->dat[2]);
		ProtectPar.RelValue.l = GET_WORD(&pCanPkg->dat[4]);
		ProtectPar.RTime.l = GET_WORD(&pCanPkg->dat[6]);
		apiSysParSetBalanceChg(&ProtectPar);
		
		appSerialCanDavinciParDebugMsg("Write Balance Chg");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	
}
static void DavinciCanParameterBalanceDhg(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;
	
	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		appSerialCanDavinciParDebugMsg("Read Balance Dhg");
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_BALANCE_DHG);
		CanPkg.dlc = 8;
		apiSysParGetBalanceDhg(&ProtectPar);
		CanPkg.dat[0] = ProtectPar.SetValue.b[0];
		CanPkg.dat[1] = ProtectPar.SetValue.b[1];
		CanPkg.dat[2] = ProtectPar.STime.b[0];
		CanPkg.dat[3] = ProtectPar.STime.b[1];
		CanPkg.dat[4] = ProtectPar.RelValue.b[0];
		CanPkg.dat[5] = ProtectPar.RelValue.b[1];
		CanPkg.dat[6] = ProtectPar.RTime.b[0];
		CanPkg.dat[7] = ProtectPar.RTime.b[1];
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_BALANCE_DHG);

		CanPkg.dlc = 0;
		
		ProtectPar.SetValue.l = GET_WORD(&pCanPkg->dat[0]);
		ProtectPar.STime.l = GET_WORD(&pCanPkg->dat[2]);
		ProtectPar.RelValue.l = GET_WORD(&pCanPkg->dat[4]);
		ProtectPar.RTime.l = GET_WORD(&pCanPkg->dat[6]);
		apiSysParSetBalanceDhg(&ProtectPar);
		
		appSerialCanDavinciParDebugMsg("Write Balance Dhg");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
}
static void DavinciCanParameterBalanceRlx(smp_can_package_t *pCanPkg)
{
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;
	
	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		appSerialCanDavinciParDebugMsg("Read Balance Rlx");
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_BALANCE_RLX);
		CanPkg.dlc = 8;
		apiSysParGetBalanceRlx(&ProtectPar);
		CanPkg.dat[0] = ProtectPar.SetValue.b[0];
		CanPkg.dat[1] = ProtectPar.SetValue.b[1];
		CanPkg.dat[2] = ProtectPar.STime.b[0];
		CanPkg.dat[3] = ProtectPar.STime.b[1];
		CanPkg.dat[4] = ProtectPar.RelValue.b[0];
		CanPkg.dat[5] = ProtectPar.RelValue.b[1];
		CanPkg.dat[6] = ProtectPar.RTime.b[0];
		CanPkg.dat[7] = ProtectPar.RTime.b[1];
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_BALANCE_RLX);

		CanPkg.dlc = 0;
		
		ProtectPar.SetValue.l = GET_WORD(&pCanPkg->dat[0]);
		ProtectPar.STime.l = GET_WORD(&pCanPkg->dat[2]);
		ProtectPar.RelValue.l = GET_WORD(&pCanPkg->dat[4]);
		ProtectPar.RTime.l = GET_WORD(&pCanPkg->dat[6]);
		apiSysParSetBalanceRlx(&ProtectPar);
		
		appSerialCanDavinciParDebugMsg("Write Balance Rlx");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}


static void DavinciCanParameterNoteMessage(smp_can_package_t *pCanPkg)
{
	uint32_t		canid;	
	uint8_t			cansendbf[8];
	uint8_t			canDatLeng = 0;
	uint8_t			i,index;
	uint8_t			NoteLen;
	smp_can_package_t	CanPkg;
	tScuProtectPar		ProtectPar;

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		appSerialCanDavinciParDebugMsg("Read Note");
		
		apiSysParGetNotwMessageString(StringMessageBuffer);
		NoteLen = strlen((char *)StringMessageBuffer);		
		CanPkg.dat[0] = (NoteLen/6);
		if(NoteLen % 6)
			CanPkg.dat[0]++;
		CanPkg.dat[1] = 1;
		
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_NOTE_MESSAGE);
		index = 0;
		do{
			canDatLeng = NoteLen - index;	
			if(canDatLeng >6)
				canDatLeng = 6;
			for(i=0; i<canDatLeng; i++) 
				CanPkg.dat[2 + i] = StringMessageBuffer[index++];
			CanPkg.dlc = canDatLeng + 2;	
			appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
			CanPkg.dat[1] ++;
		}while(index < NoteLen);				
	}
	else if(isParWritable())
	{
		appSerialCanDavinciParDebugMsg("Write Note Message");
		if(pCanPkg->dat[1] == 0x00)
			return;
		if(pCanPkg->dat[1] ==0x01)
		{
			for(i=0; i<MAX_NOTE_MESSAGE_STRING_ITEM; i++)
				StringMessageBuffer[i] = 0;	
		}
		index = pCanPkg->dat[1] - 1;
		index *= 6;
		for(i=2; i<pCanPkg->dlc; i++)
		{
			if(index >= MAX_NOTE_MESSAGE_STRING_ITEM)
				break;
			StringMessageBuffer[index++] = pCanPkg->dat[i];
		} 		
		if(pCanPkg->dat[1] >= pCanPkg->dat[0])
		{
			apiSysParSetNotwMessageString(StringMessageBuffer);
			CanPkg.dat[0] = 0;
		}
		else
			CanPkg.dat[0] = 1;
		CanPkg.dlc = 1;
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_NOTE_MESSAGE);

		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	}
	else
		return;
}

static int32_t Get3BytesInt32(uint8_t *pDat)
{
	tLbyte	value;
	value.b[0] = pDat[0];
	value.b[1] = pDat[1];
	value.b[2] = pDat[2];
	if(value.b[2] &0x80)
		value.b[3] = 0xff;
	else
		value.b[3] = 0;
	return value.sl;
}

static void DavinciCanParameterCurrentCalibration(smp_can_package_t *pCanPkg)
{
	uint8_t			cansendbf[8];
	smp_can_package_t	CanPkg;
	tLbyte			Value;
	tLbyte			Adc;
	char			str[100];

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_CALIB_CURR);
		CanPkg.dlc = 8;
		apiCaliParGetCurrentValue(pCanPkg->dat[0], pCanPkg->dat[1],
									&Value.sl, &Adc.sl);
		CanPkg.dat[0] = pCanPkg->dat[0];	//current index
		CanPkg.dat[1] = pCanPkg->dat[1];	//point 0 or 1
		CanPkg.dat[2] = Value.b[0];	
		CanPkg.dat[3] = Value.b[1];
		CanPkg.dat[4] = Value.b[2];
		CanPkg.dat[5] = Adc.b[0];
		CanPkg.dat[6] = Adc.b[1];
		CanPkg.dat[7] = Adc.b[2];	

		appSerialCanDavinciParDebugMsg("Read Curr Cali");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_CALIB_CURR);
		CanPkg.dlc = 0;

		Value.sl = Get3BytesInt32(&pCanPkg->dat[2]);
		Adc.sl = Get3BytesInt32(&pCanPkg->dat[5]);
		apiCaliParSetCurrentValue(pCanPkg->dat[0], pCanPkg->dat[1],
									Value.sl, Adc.sl);
									
		sprintf(str,"Wr Curr %d %d",Value.sl, Adc.sl);
		appSerialCanDavinciParDebugMsg(str);//"Write Curr Cali");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}
static void DavinciCanParameterVBatCalibration(smp_can_package_t *pCanPkg)
{
//	int8_t				cansendbf[8];
	smp_can_package_t	CanPkg;
	tLbyte				Value;
	tLbyte				Adc;
	char				str[100];

	if(SMP_CAN_GET_OBJ_INDEX(pCanPkg->id) == SMP_CMD_PAR_RD_OBJ_INDEX)
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_RD_OBJ_INDEX,
									SMP_PAR_ID_CALIB_VB);
		CanPkg.dlc = 8;
		apiCaliParGetVbatValue(pCanPkg->dat[0], pCanPkg->dat[1],
									&Value.sl, &Adc.sl);
		CanPkg.dat[0] = pCanPkg->dat[0];	//vbat index
		CanPkg.dat[1] = pCanPkg->dat[1];	//point 0 or 1
		CanPkg.dat[2] = Value.b[0];	
		CanPkg.dat[3] = Value.b[1];
		CanPkg.dat[4] = Value.b[2];
		CanPkg.dat[5] = Adc.b[0];
		CanPkg.dat[6] = Adc.b[1];
		CanPkg.dat[7] = Adc.b[2];	

		appSerialCanDavinciParDebugMsg("Read Vbat Cali");
	}
	else if(isParWritable())
	{
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_TX, canParScuId(),
									SMP_CMD_PAR_WR_OBJ_INDEX,
									SMP_PAR_ID_CALIB_VB);
		CanPkg.dlc = 0;

		Value.sl = Get3BytesInt32(&pCanPkg->dat[2]);
		Adc.sl = Get3BytesInt32(&pCanPkg->dat[5]);
		apiCaliParSetVbatValue(pCanPkg->dat[0], pCanPkg->dat[1],
									Value.sl, Adc.sl);
									
		sprintf(str,"Wr Vbat %d %d",Value.sl, Adc.sl);
		appSerialCanDavinciParDebugMsg(str);//"Write vbat Cali");
	}
	else
		return;
	appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
}

//----------------------------------------------------------
SMP_CAN_DECODE_CMD_START(mDavinciParameterCanDecodeTab)
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_MAGIC_CODE),
								CHECK_SMP_CAN_SUB,
								DavinciParameterMagicCode)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_HW_VERSION),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterHwVersion)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_FW_VERSION),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterFwVersion)

	SMP_CAN_DECODE_CMD_CONTENT( MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0, 
									0,
             						SMP_PAR_ID_FW_BUILD_DATE_TIME), 
             					CHECK_SMP_CAN_SUB,
                    			DavinciCanParameterFwBuildDateTime)


	SMP_CAN_DECODE_CMD_CONTENT( MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0, 
									0,
             						SMP_PAR_ID_ZERO_CURRENT), 
             					CHECK_SMP_CAN_SUB,
                    			DavinciCanParameterZeroCurrent)
                    			
 	SMP_CAN_DECODE_CMD_CONTENT( MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0, 
									0,
             						SMP_PAR_ID_PRE_DHG_TIME), 
             					CHECK_SMP_CAN_SUB,
                    			DavinciCanParameterPreDischargeTime)
                   			
 	SMP_CAN_DECODE_CMD_CONTENT( MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0, 
									0,
             						SMP_PAR_ID_MOS_ON_DIFF_VOLTAGE), 
             					CHECK_SMP_CAN_SUB,
                    			DavinciCanParameterRelayOnDiffVoltage)
                    			

	SMP_CAN_DECODE_CMD_CONTENT( MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0, 
									0,
             						SMP_PAR_ID_DESIGNED_CAPACITY), 
             					CHECK_SMP_CAN_SUB,
                    			DavinciCanParameterDesignedCapacity)

  
	SMP_CAN_DECODE_CMD_CONTENT( MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0, 
									0,
             						SMP_PAR_ID_TAPER_CURRENT), 
             					CHECK_SMP_CAN_SUB,
                    			DavinciCanParameterFullChargeCondition)
                    			
	SMP_CAN_DECODE_CMD_CONTENT( MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0, 
									0,
             						SMP_PAR_ID_FLAT_VOLTAGE), 
             					CHECK_SMP_CAN_SUB,
                    			DavinciCanParameterFlatVoltage)

	SMP_CAN_DECODE_CMD_CONTENT( MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0, 
									0,
             						SMP_PAR_ID_TERMINATE_VOLTAGE), 
             					CHECK_SMP_CAN_SUB,
                    			DavinciCanParameterTerminateVoltage)

	SMP_CAN_DECODE_CMD_CONTENT( MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0, 
									0,
             						SMP_PAR_ID_COMMUNICATION), 
             					CHECK_SMP_CAN_SUB,
                    			DavinciCanParameterAfeCommunication)

	SMP_CAN_DECODE_CMD_CONTENT( MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0, 
									0,
             						SMP_PAR_ID_INSULATION_RESISTANCE), 
             					CHECK_SMP_CAN_SUB,
                    			DavinciCanParameterInsulationResistance)


	SMP_CAN_DECODE_CMD_CONTENT( MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0, 
									0,
             						SMP_PAR_ID_QMAX), 
             					CHECK_SMP_CAN_SUB,
                    			DavinciCanParameterQmax)
	SMP_CAN_DECODE_CMD_CONTENT( MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0, 
									0,
             						SMP_PAR_ID_RM), 
             					CHECK_SMP_CAN_SUB,
                    			DavinciCanParameterRM)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_NOTE_MESSAGE),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterNoteMessage)

/*
#define	    0x03
#define SMP_PAR_ID_MID                   0x04
#define	SMP_PAR_ID_MODEL_NAME            0x05
#define	SMP_PAR_ID_MODULE_NUMBER         0x06
#define	SMP_PAR_ID_PART_NUMBER           0x07
#define SMP_PAR_ID_CELL_TYPE             0x08
#define SMP_PAR_ID_BAUDRATE              0x09
#define 			0x0A
*/

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_BMU_NUMBER),
								CHECK_SMP_CAN_SUB,
								DavinciParameterBmuNumber)
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_CELL_NTC_FLAG),
								CHECK_SMP_CAN_SUB,
								DavinciParameterCellNtcFlag)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_OCV_TABLE),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterOcvTable)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_RA_TABLE),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterRaTable)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_OVP_PROTECT),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterOvp)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_UVP_PROTECT),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterUvp)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_COTP_PROTECT),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterCotp)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_CUTP_PROTECT),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterCutp)
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_DOTP_PROTECT),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterDotp)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_DUTP_PROTECT),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterDutp)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_DTP_PROTECT),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterDtp)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_COCP_PROTECT),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterCocp)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_DOCP_PROTECT),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterDocp)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_PF_PROTECT),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterPF)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_PF_PROTECT),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterPF)


	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_PF_PROTECT),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterBalanceDuty)


	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_BALANCE_DUTY),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterBalanceDuty)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_BALANCE_CHG),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterBalanceChg)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_BALANCE_DHG),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterBalanceDhg)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_BALANCE_RLX),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterBalanceRlx)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_CALIB_CURR),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterCurrentCalibration)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									SMP_PAR_ID_CALIB_VB),
								CHECK_SMP_CAN_SUB,
								DavinciCanParameterVBatCalibration)

SMP_CAN_DECODE_CMD_END();



void DavinciCanParameterRdWrite(smp_can_package_t *pCanPkg){
	
	uint8_t	i,n;
	uint8_t cmdIndex;
	
//	char	str[100];
//	char	str1[100];
 	cmdIndex = 0;
	//appSerialCanDavinciParDebugMsg("Decode RW par ID");

	for(cmdIndex = 0; mDavinciParameterCanDecodeTab[cmdIndex].fun != 0; cmdIndex++)
	{
		if((mDavinciParameterCanDecodeTab[cmdIndex].canid & mDavinciParameterCanDecodeTab[cmdIndex].mask) == 
		   (mDavinciParameterCanDecodeTab[cmdIndex].mask & pCanPkg->id) &&
		    appSerialCanDavinciIsCorrectScuId(pCanPkg))
		{
			mDavinciParameterCanDecodeTab[cmdIndex].fun(pCanPkg);
			break; 		
 		}
	}
}


/* Public function prototypes -----------------------------------------------*/

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


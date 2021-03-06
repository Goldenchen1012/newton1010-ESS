/**
  ******************************************************************************
  * @file        AppSerialCanDavinciDebug.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/10/28
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
#include "LibCalibration.h"
#include "HalAfeADS7946.h"
#include "HalBsp.h"
#include "AppProject.h"
		  

void appSerialCanDavinciSendTextMessage(char *str);
#define	appSerialCanDavinciDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)
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

static void DavinciCanDebugModeChange(smp_can_package_t *pCanPkg)
{
	uint8_t		i;
	uint16_t	subindex, voltage;
	char	str[100];
	
	//appSerialCanDavinciDebugMsg("Rcv Simu");
	//
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);
	if(subindex == SMP_SIMU_MODE_SUB_INDEX)
	{
		if(memcmp(pCanPkg->dat, "EnSimuM", 7) == 0)
		{
			appProjectEnableSimuMode();
			//appSerialCanDavinciDebugMsg("EnSimuM");
		}
		else if(memcmp(pCanPkg->dat, "ExSimuM", 7) == 0)
		{
			appProjectDisableSimuMode();
			//appSerialCanDavinciDebugMsg("ExSimuM");
		}
	}
	else if(subindex == SMP_ENG_MODE_SUB_INDEX)
	{
		if(memcmp(pCanPkg->dat, "EnterEng", 8) == 0)
		{
			appProjectEnableEngMode();
			appSerialCanDavinciDebugMsg("Enter Eng. Mode");
		}
		else if(memcmp(pCanPkg->dat, "ExitEngM", 8) == 0)
		{
			appProjectDisableEngMode();
			appSerialCanDavinciDebugMsg("Exit Eng. Mode");
		}
	}
}

static void DavinciCanDebugCurrentAdc(smp_can_package_t *pCanPkg)
{
	int32_t	adc;
	char	str[100];
	
	adc = (int32_t)(GET_WORD(&pCanPkg->dat[0]));

	halAfeSetCurrentAdcValue(SMP_CAN_GET_SUB_INDEX(pCanPkg->id), adc);

	sprintf(str,"adc:%d I = %d", adc, appCurrDebug(SMP_CAN_GET_SUB_INDEX(pCanPkg->id), adc));
	appSerialCanDavinciDebugMsg(str);
}
static void DavinciCanDebugVbatAdc(smp_can_package_t *pCanPkg)
{
	int32_t	adc;
	char	str[100];
	
	adc = (int32_t)(GET_WORD(&pCanPkg->dat[0]));

	halAfeSetVBatAdcValue(SMP_CAN_GET_SUB_INDEX(pCanPkg->id), adc);

	sprintf(str,"adc:%d V = %d", adc, appVbatDebug(SMP_CAN_GET_SUB_INDEX(pCanPkg->id), adc));
	appSerialCanDavinciDebugMsg(str);
}


static void DavinciCanDebugCellVSimu(smp_can_package_t *pCanPkg)
{
	uint8_t		i;
	uint16_t	subindex, voltage;
	char	str[100];
	
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);

	for(i=0; i<4 ; i++)
	{
		voltage = GET_WORD(&pCanPkg->dat[i*2]);
		halAfeSetCellVoltage(subindex++, voltage);
	}
}


static void DavinciCanDebugNtcVSimu(smp_can_package_t *pCanPkg)
{
	uint8_t		i;
	uint16_t	subindex, voltage;
	char	str[100];
	
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);

	for(i=0; i<4 ; i++)
	{
		voltage = GET_WORD(&pCanPkg->dat[i*2]);
		halAfeSetNtcAdcData(subindex++, voltage);
	}
	//appSerialCanDavinciDebugMsg("Decode NTC");

}
static void DavinciCanDebugNtcTSimu(smp_can_package_t *pCanPkg)
{
	uint8_t		i;
	uint16_t	subindex, voltage;
	int16_t		temp;
	char	str[100];
	
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);

	for(i=0; i<8 ; i++)
	{
		temp = (int16_t)pCanPkg->dat[i];		
		voltage = LibTemperatureToVoltage(temp);
		/*
		if(subindex == 0)
		{
			sprintf(str,"Set NTC T=%d V=%d",temp, voltage);
			appSerialCanDavinciDebugMsg(str);
		}
		*/
		GPIOD->ODR ^= 	GPIO_PIN_15;	
		halAfeSetNtcAdcData(subindex++, voltage);
	}
	//appSerialCanDavinciDebugMsg("Decode NTC");

}

static void DavinciCanDebugCurrentSimu(smp_can_package_t *pCanPkg)
{
	uint8_t		i;
	uint16_t	subindex, voltage;
	char	str[100];
	
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);

	if(subindex == 0)
	{
		HalAfeSetCurrentValue(0, (int32_t)GET_DWORD(&pCanPkg->dat[0]));
//		sprintf(str,"Decode Current:%d", (int32_t)GET_DWORD(&pCanPkg->dat[0]));
//		appSerialCanDavinciDebugMsg(str);
	}
	else if(subindex == 1)
	{
		HalAfeSetCurrentValue(1, (int32_t)GET_DWORD(&pCanPkg->dat[0]));
	}
}

static void DavinciCanDebugGpio(smp_can_package_t *pCanPkg)
{
	uint32_t	mask;
	uint32_t	dat;
	uint16_t	subindex;
	char		*ptr;
	uint8_t	i;
	smp_can_package_t	CanPkg;
	char	str[100];

	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);

	if(subindex & 0x200)	//get gpio name
	{
		mask = 1 << ((subindex&0x1f0)/ 0x10);
		ptr = halBspGetGpioControlMsg(subindex&0x0f, mask);
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_DEBUG_TX, canDbgScuId(),
							SMP_DEBUG_GPIO_OBJ_INDEX,
							subindex);
		CanPkg.dlc = 8;
		memcpy(CanPkg.dat, ptr, 8);
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);	
		sprintf(str,"Get Gpio Name %d %d", subindex&0x0f, (subindex&0x1f0)/0x10);
		appSerialCanDavinciDebugMsg(str);
	}
	else
	{
		mask = (uint32_t)GET_DWORD(&pCanPkg->dat[0]);
		dat = (uint32_t)GET_DWORD(&pCanPkg->dat[4]);		
		halBspGpioControl(subindex, mask, dat);	
		appSerialCanDavinciDebugMsg("Gpio Debu");
	}
}


static void DavinciCanDebugVBatSimu(smp_can_package_t *pCanPkg)
{
	uint8_t		i;
	uint16_t	subindex, voltage;
	char	str[100];
	
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);

	if(subindex == 0)
	{
		halAfeSetVBatVoltage(0, (uint32_t)GET_DWORD(&pCanPkg->dat[0]));
	}
	else if(subindex == 1)
	{
		halAfeSetVBatVoltage(1, (uint32_t)GET_DWORD(&pCanPkg->dat[0]));
	}
}
static void DavinciCanDebugRelayControl(smp_can_package_t *pCanPkg)
{
	uint8_t		i;
	uint16_t	subindex, voltage;
	char	str[100];
	
	if(appProjectIsInEngMode() == 0)
		return;
	
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);

	switch(subindex)
	{
	case SMP_PS_RELAY_SUB_INDEX:
		if(pCanPkg->dat[0])
		{
			HalBspRelayPsCtrl(1);
			appSerialCanDavinciDebugMsg("Ps Relay On");
		}
		else
		{
			HalBspRelayPsCtrl(0);
			appSerialCanDavinciDebugMsg("Ps Relay Off");
		}
		break;
	case SMP_P_MAIN_RELAY_SUB_INDEX:
		if(pCanPkg->dat[0])
		{
			halBspPostiveRelayOn();
			appSerialCanDavinciDebugMsg("P Main Relay On");
		}
		else
		{
			halBspPostiveRelayOff();
			appSerialCanDavinciDebugMsg("P Main Relay Off");
		}
		break;
	case SMP_N_MAIN_RELAY_SUB_INDEX:
		if(pCanPkg->dat[0])
		{
			halBspNegtiveRelayOn();
			appSerialCanDavinciDebugMsg("N Main Relay On");
		}
		else
		{
			halBspNegtiveRelayOff();
			appSerialCanDavinciDebugMsg("N Main Relay Off");
		}
		break;
	case SMP_PRE_RELAY_SUB_INDEX:
		if(pCanPkg->dat[0])
		{
			halBspPreDischargeRelayOn();
			appSerialCanDavinciDebugMsg("Pre Relay On");
		}
		else
		{
			halBspPreDischargeRelayOff();
			appSerialCanDavinciDebugMsg("Pre Relay Off");
		}
		break;
	case SMP_FAN_RELAY_SUB_INDEX:
		if(pCanPkg->dat[0])
		{
			halBspFanRelayOn();
			appSerialCanDavinciDebugMsg("FAN Relay On");
		}
		else
		{
			halBspFanRelayOff();
			appSerialCanDavinciDebugMsg("FAN Relay Off");
		}
		break;
	}
}

void halAfeClearTestCount(void);

static void DavinciCanDebugClearTestCount(smp_can_package_t *pCanPkg)
{
#define	CLEAR_AFE_TEST_COUNT	0	
	uint8_t		i;
	uint16_t	subindex, voltage;
	char	str[100];
	
	subindex = SMP_CAN_GET_SUB_INDEX(pCanPkg->id);
	if(subindex == CLEAR_AFE_TEST_COUNT)
	{
		halAfeClearTestCount();
	}

}


SMP_CAN_DECODE_CMD_START(mDavinciDebugCanDecodeTab)
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
									SMP_DEBUG_CLEAR_TEST_COUNT_OBJ_INDEX,
									0),
								CHECK_SMP_CAN_OBJ,
								DavinciCanDebugClearTestCount)
								



SMP_CAN_DECODE_CMD_END();


void DavinciCanFunDebugRx(smp_can_package_t *pCanPkg)
{
	uint8_t	i,n;
	uint8_t cmdIndex;
	
	char	str[100];
	char	str1[100];
 	cmdIndex = 0;
	//appSerialCanDavinciDebugMsg("Decode Debug Fun");

	for(cmdIndex = 0; mDavinciDebugCanDecodeTab[cmdIndex].fun != 0; cmdIndex++)
	{
		if((mDavinciDebugCanDecodeTab[cmdIndex].canid & mDavinciDebugCanDecodeTab[cmdIndex].mask) == 
		   (mDavinciDebugCanDecodeTab[cmdIndex].mask & pCanPkg->id)  &&
		   appSerialCanDavinciIsCorrectScuId(pCanPkg))
		{
		//	sprintf(str,"Debug ID= %d",SMP_CAN_GET_SCU_ID(pCanPkg->id));
		//	appSerialCanDavinciDebugMsg(str);
			mDavinciDebugCanDecodeTab[cmdIndex].fun(pCanPkg);
			break; 		
 		}
	}
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    





/**
  ******************************************************************************
  * @file        ApiSignalFeebback.c
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
#include "main.h"
#include "halafe.h"
#include "LibSwTimer.h"
#include "AppBalance.h"
#include "LibNtc.h"
#include "ApiSignalFeedback.h"
#include "ApiSysPar.h"
#include "HalBsp.h"
void appSerialCanDavinciSendTextMessage(char *msg);
#define	appSignalFeedbackDebugMsg(str)	//appSerialCanDavinciSendTextMessage(str)


/* Private define ------------------------------------------------------------*/
#define	LEVEL_CHANGE_COUNT	1
enum{
	SIGNAL_STATUS_UNKNOW = 0,
	SIGNAL_STATUS_CHANGING,
	SIGNAL_STATUS_HI,
	SIGNAL_STATUS_LOW
};
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef uint8_t (*tSignalCheckFun)(void);

/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
typedef struct{
	tSignalCheckFun	CheckFun;
	uint8_t		Status;
	uint8_t		Count;
	uint8_t		id;
	uint8_t		EventHi;
	uint8_t		EventLo;
}tSignalCheckTable;

static tSignalCheckTable	mSignalCheckTable[]={
	{HalBspGetDi1Status,	SIGNAL_STATUS_UNKNOW,	0,
		APP_SIGNAL_ID_DI1,
		APP_SIGNAL_FB_EVT_DI1_HI,	APP_SIGNAL_FB_EVT_DI1_LO},
	{HalBspGetDi2Status,	SIGNAL_STATUS_UNKNOW,	0,
		APP_SIGNAL_ID_DI2,
		APP_SIGNAL_FB_EVT_DI2_HI,	APP_SIGNAL_FB_EVT_DI2_LO},
	{HalBspGetEpoStatus,	SIGNAL_STATUS_UNKNOW,	0,
		APP_SIGNAL_ID_EPO,
		APP_SIGNAL_FB_EVT_EPO_HI,	APP_SIGNAL_FB_EVT_EPO_LO},	
	{HalBspGetSpStatus,		SIGNAL_STATUS_UNKNOW,	0,
		APP_SIGNAL_ID_SP,
		APP_SIGNAL_FB_EVT_SP_HI,	APP_SIGNAL_FB_EVT_SP_LO},
	{HalBspGetPs1Status,	SIGNAL_STATUS_UNKNOW,	0,
		APP_SIGNAL_ID_PS1,
		APP_SIGNAL_FB_EVT_PS1_HI,	APP_SIGNAL_FB_EVT_PS1_LO},
	{HalBspGetPs2Status,	SIGNAL_STATUS_UNKNOW,	0,
		APP_SIGNAL_ID_PS2,
		APP_SIGNAL_FB_EVT_PS2_HI,	APP_SIGNAL_FB_EVT_PS2_LO},
	{HalBspGetPs3Status,	SIGNAL_STATUS_UNKNOW,	0,
		APP_SIGNAL_ID_PS3,
		APP_SIGNAL_FB_EVT_PS3_HI,	APP_SIGNAL_FB_EVT_PS3_LO},
	{HalBspGetButtonStatus,	SIGNAL_STATUS_UNKNOW,	0,
		APP_SIGNAL_ID_BUTTON,
		APP_SIGNAL_FB_EVT_BUTTON_HI,	APP_SIGNAL_FB_EVT_BUTTON_LO},
#if 0		
	{HalBspGetK1Status,		SIGNAL_STATUS_UNKNOW,	0,
		APP_SIGNAL_ID_K1,
		APP_SIGNAL_FB_EVT_K1_HI,	APP_SIGNAL_FB_EVT_K1_LO},
	{HalBspGetK2Status,		SIGNAL_STATUS_UNKNOW,	0,
		APP_SIGNAL_ID_K2,
		APP_SIGNAL_FB_EVT_K2_HI,	APP_SIGNAL_FB_EVT_K2_LO},
	{HalBspGetK3Status,		SIGNAL_STATUS_UNKNOW,	0,
		APP_SIGNAL_ID_K3,
		APP_SIGNAL_FB_EVT_K3_HI,	APP_SIGNAL_FB_EVT_K3_LO},
	{HalBspGetK4Status,		SIGNAL_STATUS_UNKNOW,	0,
		APP_SIGNAL_ID_K4,
		APP_SIGNAL_FB_EVT_K4_HI,	APP_SIGNAL_FB_EVT_K4_LO},
#endif		
	{HalBspGetDocpLatchStatus,	SIGNAL_STATUS_UNKNOW,	0,
		APP_SIGNAL_ID_DOCP,
		APP_SIGNAL_FB_EVT_DOCP_HI,	APP_SIGNAL_FB_EVT_DOCP_LO},
	{HalBspGetCocpLatchStatus,	SIGNAL_STATUS_UNKNOW,	0,
		APP_SIGNAL_ID_COCP,
		APP_SIGNAL_FB_EVT_COCP_HI,	APP_SIGNAL_FB_EVT_COCP_LO},	
	{HalBspGetOdInStatus,	SIGNAL_STATUS_UNKNOW,	0,
		APP_SIGNAL_ID_OD_IN,
		APP_SIGNAL_FB_EVT_OD_IN_HI,	APP_SIGNAL_FB_EVT_OD_IN_LO},
	{halBspGetNfaultStatus,	SIGNAL_STATUS_UNKNOW,	0,
		APP_SIGNAL_NFAULT,
		APP_SIGNAL_FB_EVT_NFAULT_HI,	APP_SIGNAL_FB_EVT_NFAULT_LO},
		
		//BSP_NFAULT_READ
		
	{0,0,0,0}
};
	
static tApiSignalFeedbackEvtHandler SignalFeedbackEvtHandler;

/* Private function prototypes -----------------------------------------------*/

static void checkSignal(tSignalCheckTable *pChkTable)
{
	if(pChkTable->CheckFun())
	{
		if(pChkTable->Status != SIGNAL_STATUS_HI)
		{
			pChkTable->Count = 0;
			pChkTable->Status = SIGNAL_STATUS_HI;
		}
		pChkTable->Count++;
		if(pChkTable->Count >= LEVEL_CHANGE_COUNT)
		{
			if(pChkTable->Count == LEVEL_CHANGE_COUNT)
			{
				if(SignalFeedbackEvtHandler)
					SignalFeedbackEvtHandler(0, pChkTable->EventHi ,0);
			}
			pChkTable->Count = LEVEL_CHANGE_COUNT;
		}
	}
	else
	{
		if(pChkTable->Status != SIGNAL_STATUS_LOW)
		{
			pChkTable->Count = 0;			
			pChkTable->Status = SIGNAL_STATUS_LOW;
		}
		pChkTable->Count++;
		if(pChkTable->Count >= LEVEL_CHANGE_COUNT)
		{
			if(pChkTable->Count == LEVEL_CHANGE_COUNT)
			{
				if(SignalFeedbackEvtHandler)
					SignalFeedbackEvtHandler(0, pChkTable->EventLo,	0);
			}
			pChkTable->Count = LEVEL_CHANGE_COUNT;
		}
	}
}


static void signalFeedbackSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	uint8_t		i;
	//GPIOD->ODR |= GPIO_PIN_14;
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
		for(i=0; i<50; i++)
		{
			if(mSignalCheckTable[i].CheckFun == NULL)
				break;
			checkSignal(&mSignalCheckTable[i]);
		}
//		GPIOD->ODR ^= GPIO_PIN_14;
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		
	}
	//GPIOD->ODR &= ~GPIO_PIN_14;
}
/* Public function prototypes -----------------------------------------------*/
uint8_t apiSignalFeedbackGetStatus(uint8_t id)
{
	uint8_t	i;
	
	for(i=0; i<50; i++)
	{
		if(mSignalCheckTable[i].CheckFun == NULL)
			break;
		if(mSignalCheckTable[i].id == id)
		{
			if(mSignalCheckTable[i].Status == SIGNAL_STATUS_HI)
				return 1;
			else
				return 0;
		}
	}
}


void apiSignalFeedbackOpen(tApiSignalFeedbackEvtHandler evtHandler)
{  	
	SignalFeedbackEvtHandler = evtHandler;
	LibSwTimerOpen(signalFeedbackSwTimerHandler, 0);
}


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    




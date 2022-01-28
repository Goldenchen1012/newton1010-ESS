/**
  ******************************************************************************
  * @file        AppLed.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2022/1/26
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
#include <stdio.h>
#include "main.h"
#include "LibDebug.h"
#include "LibSwTimer.h"
#include "AppLed.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	ledDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)


/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t		delay;
	uint8_t 	step;
	uint8_t		cycle;
	uint16_t	substate;
	uint8_t		state;
	uint16_t	par;
}tLedState;
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static tLedState	mSocLedState = {0};
static tLedState	mStatusLedState = {0};

/* Private function prototypes -----------------------------------------------*/
static void statusLedHandler(void)
{
	switch(mStatusLedState.state)
	{
	case LED_STATE_ALARM:
	case LED_STATE_COMM:
	case LED_STATE_CRH:
	case LED_STATE_DSG:
		break;
	}
}
static void socLedHandler(void)
{
	switch(mSocLedState.state)
	{
	case LED_STATE_CAPACITY:
		break;
	case LED_STATE_WAIT_SYS_READY:
		if(mSocLedState.delay)
		{
			mSocLedState.delay--;
			break;
		}
		mSocLedState.delay = 40;
		mSocLedState.substate++;
		switch(mSocLedState.substate)
		{
		case 1:
			halLedSetPercent20LedState(1);
			break;
		case 2:
			halLedSetPercent40LedState(1);
			break;
		case 3:
			halLedSetPercent60LedState(1);
			break;
		case 4:
			halLedSetPercent80LedState(1);
			break;
		case 5:
			halLedSetPercent100LedState(1);
			break;
		case 6:
			halLedSetPercent100LedState(0);
			break;
		case 7:
			halLedSetPercent80LedState(0);
			break;
		case 8:
			halLedSetPercent60LedState(0);
			break;
		case 9:
			halLedSetPercent40LedState(0);
			break;
		case 10:
			halLedSetPercent20LedState(0);
			mSocLedState.substate = 0;
			break;
		}
		break;
	case LED_STATE_SCU_ID:
		
	}
}
static void ledSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	switch(evt)
	{
	case LIB_SW_TIMER_EVT_SW_1MS:
		break;
	case LIB_SW_TIMER_EVT_SW_10MS_7:
		socLedHandler();
		statusLedHandler();
		break;
	}
}
/* Public function prototypes -----------------------------------------------*/
void appLedSetState(uint16_t ShowTime, uint8_t state, uint16_t par)
{
	if(state == LED_STATE_CAPACITY ||
	   state == LED_STATE_WAIT_SYS_READY ||
	   state == LED_STATE_SCU_ID)
	{
		mSocLedState.state = state;
	}
	else
	{
		mStatusLedState.state = state;
	}
}

void appLedOpen(void)
{
	halLedOpen();
	memset(&mSocLedState, 0, sizeof(tLedState));
	memset(&mStatusLedState, 0, sizeof(tLedState));
	LibSwTimerOpen(ledSwTimerHandler, 0);
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    









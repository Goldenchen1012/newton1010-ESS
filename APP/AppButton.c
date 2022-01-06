/**
  ******************************************************************************
  * @file        AppButton.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/12/14
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
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "define.h"
#include "main.h"
#include "LibDebug.h"
#include "LibRegister.h"
#include "LibSwTimer.h"
#include "LibHwTimer.h"
#include "AppButton.h"
#include "ApiSignalFeedback.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	buttonDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)


/* Private define ------------------------------------------------------------*/
#define BUTTON_TIME_BASE 10 //ms

#define BUTTON_TIME_BASE_EVENT 	LIB_SW_TIMER_EVT_SW_10MS_2

#define	BSP_SW_READ()		apiSignalFeedbackGetStatus(APP_SIGNAL_ID_BUTTON)
#define	BSP_SW_PRESSED		0
#define	BSP_SW_RELEASED		1

/* Private typedef -----------------------------------------------------------*/
typedef enum{
  BUTTON_STATE_UNINIT = 0,
  BUTTON_STATE_RELEASE,
  BUTTON_STATE_PRESSING,
  BUTTON_STATE_LONG_PRESSED,
} tLibButtonState;

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint16_t pressCountKey0 = 0;
static tLibButtonState buttonStateKey0 = BUTTON_STATE_RELEASE;
static tLibRegister buttonRegister = {0};
static uint16_t powerOnOffsetKeyCount = 0;

/* Private function prototypes -----------------------------------------------*/
static void buttonSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	if(evt == BUTTON_TIME_BASE_EVENT)
	{
	    switch (buttonStateKey0)
	    {
        case BUTTON_STATE_RELEASE:
            if(BSP_SW_READ() == BSP_SW_PRESSED)
            {
                pressCountKey0 = 0;
                buttonStateKey0 = BUTTON_STATE_PRESSING;
                LibRegisterTypeHandlerExe(&buttonRegister, BUTTON_EVT_PRESS_KEY_0, 0);
            }
            break;
        case BUTTON_STATE_PRESSING:
            if(BSP_SW_READ() == BSP_SW_PRESSED)
            {
                if(powerOnOffsetKeyCount!=0)
                {
                    pressCountKey0 += powerOnOffsetKeyCount;
                    powerOnOffsetKeyCount = 0;
                }
                pressCountKey0 += BUTTON_TIME_BASE;
                if(pressCountKey0 % 1000 == 0 )
                {
                    LibRegisterTypeHandlerExe(&buttonRegister, BUTTON_EVT_PRESS_1S_KEY_0, 0);
                }
                if(pressCountKey0 >= LIB_BUTTON_LONG_PRESS_TIME)
                {
                    buttonStateKey0 = BUTTON_STATE_LONG_PRESSED;
                    LibRegisterTypeHandlerExe(&buttonRegister, BUTTON_EVT_LONG_PRESS_KEY_0, 0);
                }
            }
            else
            {
                buttonStateKey0 = BUTTON_STATE_RELEASE;
                LibRegisterTypeHandlerExe(&buttonRegister, BUTTON_EVT_RELEASE_KEY_0, 0);
                LibRegisterTypeHandlerExe(&buttonRegister, BUTTON_EVT_CLICK_KEY_0, 0);
            }
            break;
        case BUTTON_STATE_LONG_PRESSED:
            if(BSP_SW_READ() == BSP_SW_RELEASED)
            {
                buttonStateKey0 = BUTTON_STATE_RELEASE;
                LibRegisterTypeHandlerExe(&buttonRegister, BUTTON_EVT_RELEASE_KEY_0, 0);
            }
            else
            {
                pressCountKey0 += BUTTON_TIME_BASE;
                if(pressCountKey0 % 1000 == 0 )
                {
					pressCountKey0 = 0;
                    LibRegisterTypeHandlerExe(&buttonRegister, BUTTON_EVT_PRESS_1S_KEY_0, 0);
                }				
			}
            break;
        default:
            break;
        }
    }
}
/* Public function prototypes -----------------------------------------------*/
void appButtonOpen(tAppButtonEvtHandler EventHandler)
{
	LibRegisterAdd(&buttonRegister, EventHandler, 0);	
  	LibSwTimerOpen(buttonSwTimerHandler, 0);
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    









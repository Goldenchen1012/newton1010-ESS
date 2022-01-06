/**
  ******************************************************************************
  * @file        LibButton.c
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/14
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
#include "sdk_config.h"
#include "project_config.h"

#if defined(BOARD_E_BIKE_REV1)
#include "define.h"
#endif

#include "LibDebug.h"
#include "LibRegister.h"
#include "LibSwTimer.h"
#include "LibButton.h"
#include "boards.h"
#include "main.h"

#include <stdint.h>
#include <stdbool.h>

/* Private typedef -----------------------------------------------------------*/
typedef enum{
  BUTTON_STATE_UNINIT = 0,
  BUTTON_STATE_RELEASE,
  BUTTON_STATE_PRESSING,
  BUTTON_STATE_LONG_PRESSED,
} tLibButtonState;

/* Private define ------------------------------------------------------------*/
#define BUTTON_TIME_BASE 10 //ms

#define BUTTON_TIME_BASE_EVENT LIB_SW_TIMER_EVT_SW_10MS_2

#define BOOTLOADER_OFFSET_CNT 300

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint16_t pressCountKey0 = 0;
static tLibButtonState buttonStateKey0 = BUTTON_STATE_UNINIT;
static tLibRegister buttonRegister = {0};
static uint16_t powerOnOffsetKeyCount;
/* Private function prototypes -----------------------------------------------*/
static void LibButtonTimerHandler(__far void *dest, uint16_t evt, void *data){
    if(evt == BUTTON_TIME_BASE_EVENT){
	    switch (buttonStateKey0){
        case BUTTON_STATE_RELEASE:
            if(BSP_SW_READ() == BSP_SW_PRESSED){
                pressCountKey0 = 0;
                buttonStateKey0 = BUTTON_STATE_PRESSING;
                LibRegisterTypeHandlerExe(&buttonRegister, BUTTON_EVT_PRESS_KEY_0, 0);
            }
            break;
        case BUTTON_STATE_PRESSING:
            if(BSP_SW_READ() == BSP_SW_PRESSED){
                if(powerOnOffsetKeyCount!=0){
                    pressCountKey0 += powerOnOffsetKeyCount;
                    powerOnOffsetKeyCount = 0;
                }
                pressCountKey0 += BUTTON_TIME_BASE;
                if(pressCountKey0 % 1000 == 0 ){
                    LibRegisterTypeHandlerExe(&buttonRegister, BUTTON_EVT_PRESS_1S_KEY_0, 0);
                }

                if(pressCountKey0 >= LIB_BUTTON_LONG_PRESS_TIME){
                    buttonStateKey0 = BUTTON_STATE_LONG_PRESSED;
                    LibRegisterTypeHandlerExe(&buttonRegister, BUTTON_EVT_LONG_PRESS_KEY_0, 0);
                }
            }else{
                buttonStateKey0 = BUTTON_STATE_RELEASE;
                LibRegisterTypeHandlerExe(&buttonRegister, BUTTON_EVT_RELEASE_KEY_0, 0);
                LibRegisterTypeHandlerExe(&buttonRegister, BUTTON_EVT_CLICK_KEY_0, 0);
            }
            break;
        case BUTTON_STATE_LONG_PRESSED:
            if(BSP_SW_READ() == BSP_SW_RELEASED){
                buttonStateKey0 = BUTTON_STATE_RELEASE;
                LibRegisterTypeHandlerExe(&buttonRegister, BUTTON_EVT_RELEASE_KEY_0, 0);
            }else{
                pressCountKey0 += BUTTON_TIME_BASE;
                if(pressCountKey0 % 1000 == 0 ){
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

tErrCode LibButtonOpen(tLibButtonEvtHandler evtHandler){
    pressCountKey0 = 0;
    if(buttonStateKey0 == BUTTON_STATE_UNINIT){
      BSP_SW_OPEN();
    }
    buttonStateKey0 = BUTTON_STATE_RELEASE;
    
    LibSwTimerOpen(LibButtonTimerHandler, 0);
    return LibRegisterAdd(&buttonRegister, evtHandler, 0);
}

bool LibButtonIsPress(void){
    if(buttonStateKey0 == BUTTON_STATE_UNINIT){
      BSP_SW_OPEN();
      buttonStateKey0 = BUTTON_STATE_RELEASE;
    }

    if(BSP_SW_READ() == BSP_SW_PRESSED){
        return true;
    }else{
        return false;
    }
}

void LibSetPowerOnOffsetKeyCount(void){
    powerOnOffsetKeyCount = LibGetSwTimer() + BOOTLOADER_OFFSET_CNT;
	PowerOnWaitReadyTime = LibGetSwTimer() + BOOTLOADER_OFFSET_CNT;
}

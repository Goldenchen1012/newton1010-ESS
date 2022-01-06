/**
  ******************************************************************************
  * @file        LibLedControl.c
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
#include "project_config.h"

#if defined(BOARD_E_BIKE_REV1)
#include "define.h"	
#endif
#include "LibLedControl.h"
#include "LibSwTimer.h"
#include "boards.h"
#include <stdint.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define TIME_BASE 10 //ms

#if (TIME_BASE == 1)
#define TIME_BASE_EVENT LIB_SW_TIMER_EVT_SW_1MS
#elif (TIME_BASE == 10)
#define TIME_BASE_EVENT LIB_SW_TIMER_EVT_SW_10MS_1
#endif

/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
tLibLedControl mLed = {{0},0,LED_PRIORITY_L,0};
//= {.ledState = {0},
//                              .nowPriority = LED_PRIORITY_L,
//                              .subState = 0,
//                              .delayCount = 0};
tLibRegister libLedEvtHandlerRegister;
/* Private function prototypes -----------------------------------------------*/
void AppLedControlHandler(void);


void LibLedTimerHandler(__far void *dest, uint16_t evt, void *data){
    tLibLedPriority u8;

    if(evt == TIME_BASE_EVENT){
        if(mLed.delayCount > TIME_BASE){
            mLed.delayCount -= TIME_BASE;
        }else{
            mLed.delayCount = 0;
            AppLedControlHandler();
        }

        for(u8=mLed.nowPriority;u8<LED_PRIORITY_MAX;u8++){
            if(mLed.ledState[u8].actTime > 0){
                if(mLed.ledState[u8].actTime > TIME_BASE){
                    mLed.ledState[u8].actTime -= TIME_BASE;
                }else{
                    mLed.ledState[u8].actTime = 0;
                    LibLedControlSetState(u8, 0, LED_STATE_DISABLE);
                }
            }
        }
    }
}

tErrCode LibLedControlOpen(tLibLedEvtHandler evtHandler){
	return LibRegisterAdd(&libLedEvtHandlerRegister, evtHandler, 0);
}

// tErrCode LibLedControlClose(tLibLedEvtHandler evtHandler){
//     LibRegisterRm(&libLedEvtHandlerRegister, evtHandler);
// 	if(LibRegisterIsMemberNull(&libLedEvtHandlerRegister) == true){
// 		LibLedControlSetState(LED_PRIORITY_H, 0, LED_STATE_DISABLE);
// 		LibLedControlSetState(LED_PRIORITY_M, 0, LED_STATE_DISABLE);
// 		LibLedControlSetState(LED_PRIORITY_L, 0, LED_STATE_DISABLE);
// 	}
// }

tErrCode LibLedControlSetState(tLibLedPriority priority, uint32_t actTime, tLibLedState state){
    tLibLedPriority u8;
    tLibLedState nowState;

// 	if((LibRegisterIsMemberNull(&libLedEvtHandlerRegister) == true) 
// 		&& tLibLedState != LED_STATE_DISABLE){
// 	    return RES_ERROR_NOT_OPEN;		
// 	}
	
    if(priority >= LED_PRIORITY_MAX){
        return RES_ERROR_INVALID_PARAM;
    }

    if(state >= LED_STATE_MAX){
        return RES_ERROR_INVALID_PARAM;
    }

    if((actTime > 0) && (actTime < TIME_BASE)){
        return RES_ERROR_INVALID_PARAM;
    }

    nowState = mLed.ledState[mLed.nowPriority].state;
    mLed.ledState[priority].state = state;

    for(u8=LED_PRIORITY_H;u8<LED_PRIORITY_MAX;u8++){
        if(mLed.ledState[u8].state != LED_STATE_DISABLE){
            break;
        }
    }
    if(u8 >= LED_PRIORITY_MAX){
        u8 = LED_PRIORITY_L;
    }

    if(nowState != mLed.ledState[u8].state){
        mLed.subState = LED_SUB_STEP_INIT;
        mLed.delayCount = 0;
    }

    mLed.nowPriority = u8;
    mLed.ledState[priority].actTime = actTime;

    if((mLed.nowPriority == LED_PRIORITY_L) && (mLed.ledState[mLed.nowPriority].state == LED_STATE_DISABLE)){
        LibSwTimerClose(LibLedTimerHandler, 0);
    }else{
        LibSwTimerOpen(LibLedTimerHandler, 0);
    }
    if(mLed.subState == LED_SUB_STEP_INIT){
        AppLedControlHandler();
    }
	
	return RES_SUCCESS;
}
/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    

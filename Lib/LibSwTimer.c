/**
  ******************************************************************************
  * @file        LibSwTimer.c
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/16
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
#include "LibDebug.h"
#include "LibRegister.h"
#include "LibSwTimer.h"

#include <stdbool.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint16_t count1ms = 0;
static uint16_t count1s = 0;
static uint16_t delayCount = 0;


static tLibRegister EvtHandlerRegister={0}, EvtHandlerRegisterTask={0};
/* Private function prototypes -----------------------------------------------*/
uint16_t LibGetSwTimer(void)
{
	return count1ms;
}

void LibSwTimerClearCount(void)
{
	count1ms = 0;
	count1s = 0;
	delayCount = 0;
}


tErrCode LibSwTimerOpen(tLibSwTimerEvtHandler handler, __far void *dest){
static bool initFlag = false; 
  if(LibRegisterIsMemberNull(&EvtHandlerRegister) == true){
	  if(initFlag == true){
          count1ms = 0;
          count1s = 0;
	  }
  }
  
  if(initFlag == false){
	  initFlag = true;  
  }
  return LibRegisterAdd(&EvtHandlerRegister, handler, dest);
}

tErrCode LibSwTimerClose(tLibSwTimerEvtHandler handler, __far void *dest){
  return LibRegisterRm(&EvtHandlerRegister, handler, dest);
}

void LibSwTimerHwHandler(tLibSwTimerEvt evt, __far void *data){
  if(evt == LIB_SW_TIMER_EVT_HW_1MS){
    count1ms++;
    if(delayCount > 0){
      delayCount--;
    }
  }else if(evt == LIB_SW_TIMER_EVT_HW_5MS){
    count1ms+=5;
    if(delayCount > 5){
      delayCount--;
    }else{
      delayCount = 0;
    }
  }
}

void LibSwTimerHwDelay(uint16_t ms){
 //   uint32_t delayTimeout = 300000 
    delayCount = ms;
    while(delayCount >0);
}

void LibSwTimerHandle(void){
	uint8_t i;
	//LibRegisterTypeHandlerExe(&EvtHandlerRegisterTask, LIB_SW_TIMER_EVT_SW_TASK, 0);
  //for(i=0; i<10; i++)
  {
    if(count1ms > 0){

		//count1ms--;
		count1ms = 0;
		if(LibRegisterIsMemberNull(&EvtHandlerRegister) == true){
		return;
		}

		count1s++;
		LibRegisterTypeHandlerExe(&EvtHandlerRegister, LIB_SW_TIMER_EVT_SW_1MS, 0);
		LibRegisterTypeHandlerExe(&EvtHandlerRegister, count1s%10, 0);
	
		if(count1s%100 == 0){
		  LibRegisterTypeHandlerExe(&EvtHandlerRegister, LIB_SW_TIMER_EVT_SW_100MS, 0);
		}
		if(count1s%500 == 0){
		  LibRegisterTypeHandlerExe(&EvtHandlerRegister, LIB_SW_TIMER_EVT_SW_500MS, 0);
		} 

		if(count1s >= 1000){
		LibRegisterTypeHandlerExe(&EvtHandlerRegister, LIB_SW_TIMER_EVT_SW_1S, 0);
		count1s -= 1000;
		}
	}
  }
}

tErrCode LibSwTimerTaskOpen(tLibSwTimerEvtHandler handler, __far void *dest){
  return LibRegisterAdd(&EvtHandlerRegisterTask, handler, dest);
}

tErrCode LibSwTimerTaskClose(tLibSwTimerEvtHandler handler, __far void *dest){
  return LibRegisterRm(&EvtHandlerRegisterTask, handler, dest);
}

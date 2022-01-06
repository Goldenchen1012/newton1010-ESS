/**
  ******************************************************************************
  * @file        LibHwTimer.c
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
#include "LibHwTimer.h"

#include <stdbool.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static tLibRegister HwTimerEvtHandlerRegister;
/* Private function prototypes -----------------------------------------------*/

tErrCode LibHwTimerOpen(tLibHwTimerEvtHandler handler, __far void *dest){
static bool initFlag = false; 
  if(LibRegisterIsMemberNull(&HwTimerEvtHandlerRegister) == true){
	  if(initFlag == true){
//          count1ms = 0;
 //         count1s = 0;
	  }
  }
  
  if(initFlag == false){
	  initFlag = true;  
  }
  return LibRegisterAdd(&HwTimerEvtHandlerRegister, handler, dest);
}

tErrCode LibHwTimerClose(tLibHwTimerEvtHandler handler, __far void *dest){
  return LibRegisterRm(&HwTimerEvtHandlerRegister, handler, dest);
}

void LibHwTimerHandle(void){
	LibRegisterTypeHandlerExe(&HwTimerEvtHandlerRegister, LIB_HW_TIMER_EVT_1MS, 0);
}


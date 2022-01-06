/**
  ******************************************************************************
  * @file        LibSwTimer.h
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

#ifndef _LIB_SW_TIMER_H
#define _LIB_SW_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "LibDebug.h"
#include "LibRegister.h"

/* Public define ------------------------------------------------------------*/
#define tLibSwTimerEvtHandler tLibRegisterEvtHandler
/* Public typedef -----------------------------------------------------------*/
typedef enum {
  LIB_SW_TIMER_EVT_SW_10MS_0 = 0,
  LIB_SW_TIMER_EVT_SW_10MS_1,  
  LIB_SW_TIMER_EVT_SW_10MS_2,  
  LIB_SW_TIMER_EVT_SW_10MS_3,  
  LIB_SW_TIMER_EVT_SW_10MS_4,  
  LIB_SW_TIMER_EVT_SW_10MS_5,  
  LIB_SW_TIMER_EVT_SW_10MS_6,  
  LIB_SW_TIMER_EVT_SW_10MS_7,  
  LIB_SW_TIMER_EVT_SW_10MS_8,  
  LIB_SW_TIMER_EVT_SW_10MS_9,  
  LIB_SW_TIMER_EVT_SW_1MS,  
  LIB_SW_TIMER_EVT_SW_100MS,
  LIB_SW_TIMER_EVT_SW_500MS,
  LIB_SW_TIMER_EVT_SW_1S,
  
  LIB_SW_TIMER_EVT_SW_TASK,
  
  LIB_SW_TIMER_EVT_HW_1MS,
  LIB_SW_TIMER_EVT_HW_5MS,

} tLibSwTimerEvt;

/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
uint16_t LibGetSwTimer(void);
void LibSwTimerClearCount(void);
tErrCode LibSwTimerOpen(tLibSwTimerEvtHandler handler, __far void *dest);
tErrCode LibSwTimerClose(tLibSwTimerEvtHandler handler, __far void *dest);
void LibSwTimerHwHandler(tLibSwTimerEvt evt, __far void *data);
void LibSwTimerHandle(void);
void LibSwTimerHwDelay(uint16_t ms);

tErrCode LibSwTimerTaskOpen(tLibSwTimerEvtHandler handler, __far void *dest);
tErrCode LibSwTimerTaskClose(tLibSwTimerEvtHandler handler, __far void *dest);

#ifdef __cplusplus
}
#endif

#endif /* _LIB_SW_TIMER_H */

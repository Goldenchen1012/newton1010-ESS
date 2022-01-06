/**
  ******************************************************************************
  * @file        LibHwTimer.h
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/19
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _LIB_HW_TIMER_H
#define _LIB_HW_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "LibDebug.h"
#include "LibRegister.h"

/* Public define ------------------------------------------------------------*/
#define tLibHwTimerEvtHandler tLibRegisterEvtHandler
/* Public typedef -----------------------------------------------------------*/
typedef enum {
  LIB_HW_TIMER_EVT_1MS = 0,
} tLibHwTimerEvt;

/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
void LibHwTimerHandle(void);
tErrCode LibHwTimerOpen(tLibHwTimerEvtHandler handler, __far void *dest);
tErrCode LibHwTimerClose(tLibHwTimerEvtHandler handler, __far void *dest);
#ifdef __cplusplus
}
#endif

#endif /* _LIB_HW_TIMER_H */

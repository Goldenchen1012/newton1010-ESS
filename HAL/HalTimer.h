/**
  ******************************************************************************
  * @file        HalTmer.h
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/05
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _HAL_TIMER_H_
#define _HAL_TIMER_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "LibDebug.h"
#include "LibRegister.h"


#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
#define	HAL_TIMER_EVT_1MS		1
/* Public typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t		TimerNo;
	uint32_t	IntervalUs;
}tHalTimer;

#define tHalTimerEvtHandler tLibRegisterEvtHandler

/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
uint16_t halTimerGetCountValue(tHalTimer *pHalTimer);
tErrCode HalTimerOpen(tHalTimer *pHalTimer, tHalTimerEvtHandler evtHandler);
tErrCode HalSpiClose(tHalTimer *pHalTimer);


#endif /* _HAL_TIMER_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



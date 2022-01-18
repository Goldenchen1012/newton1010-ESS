/**
  ******************************************************************************
  * @file        ApiEventLog.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/25
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _API_EVENT_LOG_H_
#define _API_EVENT_LOG_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/* Public typedef -----------------------------------------------------------*/
typedef void(*tApiEventLogCallbackFunction)(uint8_t num, uint8_t *pMsgBuf);

/* Public define ------------------------------------------------------------*/

/* Public variables ---------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/

void apiEventLogClearLogData(void);

uint32_t apiEventLogGetLogNumber(void);

void apiEventLogReadLogData(uint32_t EvetIdex, uint8_t ReadNumber, tApiEventLogCallbackFunction CbFunction);
void apiEventLogSaveLogData(uint8_t EventType,uint16_t Par);
void apiEventLogOpen(void);


#ifdef __cplusplus
}
#endif


	

#endif /* _API_EVENT_LOG_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



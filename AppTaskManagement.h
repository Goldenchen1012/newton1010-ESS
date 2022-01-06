/**
  ******************************************************************************
  * @file        appTaskManagemente.h
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/12
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _APP_TASK_MANAGEMENT_H_
#define _APP_TASK_MANAGEMENT_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
#define	APP_TASK_EVT_T1MS	1
#define	APP_TASK_EVT_T10MS	10
#define	APP_TASK_EVT_T1SEC	100

void appTaskTimerHanlder(void *pin, uint16_t evt, void *pData);
void appTaskManagementHanlder(void);
void appTaskOpen(void);


/* Public macro -------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif


	

#endif /* _APP_TASK_MANAGEMENT_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



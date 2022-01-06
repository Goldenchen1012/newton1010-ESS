/**
  ******************************************************************************
  * @file        AppProjectHvEss.h
  * @author      Johnny
  * @version     v0.0
  * @date        2021/9/7
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _APP_PROJECT_HV_ESS_H_
#define _APP_PROJECT_HV_ESS_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/* Public define ------------------------------------------------------------*/
uint8_t appProjectGetSystemReadyFlag(void);
uint8_t appProjectGetRelayOnFlag(void);


void appProjectOpen(void);
/* Public macro -------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif


	

#endif /* _APP_PROJECT_HV_ESS_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



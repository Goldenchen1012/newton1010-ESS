/**
  ******************************************************************************
  * @file        AppProtectDtp.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2022/1/27
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */
#ifndef _API_PROTECT_DTP_H_
#define _API_PROTECT_DTP_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "AppProtect.h"
#include "LibRegister.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/

/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
void apiProtectDtpOpen(tAppProtectEvtHandler evtHandler);
uint8_t	apiProtectDtpGetFlag(void);
uint8_t apiProtectDtpHandler(void);

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif


#endif /* _API_PROTECT_DTP_H_ */


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



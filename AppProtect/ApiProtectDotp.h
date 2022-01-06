/**
  ******************************************************************************
  * @file        AppProtectDotp.h
  * @author      Johnny
  * @version     v0.0
  * @date        2021/9/8
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */
#ifndef _API_PROTECT_DOTP_H_
#define _API_PROTECT_DOTP_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "AppProtect.h"
#include "LibRegister.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/
//typedef void (* tLibRegisterEvtHandler)(__far void *dest, uint16_t evt, __far void *data);

/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
void apiProtectDotpOpen(tAppProtectEvtHandler evtHandler);
uint8_t	apiProtectDotpGetFlag(uint16_t NtcIndex);
uint8_t apiProtectDotpHandler(uint8_t ProtectLevel);

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif


#endif /* _API_PROTECT_DOTP_H_ */


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



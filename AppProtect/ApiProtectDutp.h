/**
  ******************************************************************************
  * @file        AppProtectDutp.h
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
#ifndef _API_PROTECT_DUTP_H_
#define _API_PROTECT_DUTP_H_
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
void apiProtectDutpOpen(tAppProtectEvtHandler evtHandler);
uint8_t	apiProtectDutpGetFlag(uint16_t NtcIndex);
uint8_t apiProtectDutpHandler(uint8_t ProtectLevel);

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif


#endif /* _API_PROTECT_DUTP_H_ */


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



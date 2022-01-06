/**
  ******************************************************************************
  * @file        ApiProtectCutp.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/23
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */
#ifndef _API_PROTECT_CUTP_H_
#define _API_PROTECT_CUTP_H_
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
void apiProtectCutpOpen(tAppProtectEvtHandler evtHandler);
uint8_t	apiProtectCutpGetFlag(uint16_t NtcIndex);
uint8_t apiProtectCutpHandler(uint8_t ProtectLevel);

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif


#endif /* _API_PROTECT_CUTP_H_ */


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



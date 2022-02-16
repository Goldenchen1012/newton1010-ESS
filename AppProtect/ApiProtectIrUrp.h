/**
  ******************************************************************************
  * @file        ApiProtectIrRup.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2022/02/15
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */
#ifndef _API_PROTECT_IR_URP_H_
#define _API_PROTECT_IR_URP_H_
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
uint8_t	apiProtectIrUrpGetRpFlag(void);
uint8_t	apiProtectIrUrpGetRnFlag(void);
void apiProtectIrUrpOpen(tAppProtectEvtHandler evtHandler);
uint8_t apiProtectIrUrpHandler(uint8_t ProtectLevel);

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif


#endif /* _API_PROTECT_DUTP_H_ */


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



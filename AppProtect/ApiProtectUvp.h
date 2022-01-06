/**
  ******************************************************************************
  * @file        AppProtectUvp.h
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
#ifndef _APP_PROTECT_UVP_H_
#define _APP_PROTECT_UVP_H_
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
void apiProtectUvpOpen(tAppProtectEvtHandler evtHandler);
uint8_t apiProtectUvpHandler(uint8_t ProtectLevel);
void apiProtectUvpPfHandler(void);
uint8_t	apiProtectUvpGetFlag(uint16_t CellIndex);
uint8_t	apiProtectUvpPfGetFlag(void);
void apiProtectUvpPfClean(void);

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif


#endif /* _APP_PROTECT_UVP_H_ */


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



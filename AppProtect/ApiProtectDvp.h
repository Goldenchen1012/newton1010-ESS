/**
  ******************************************************************************
  * @file        AppProtectDvp.h
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
#ifndef _APP_PROTECT_DVP_H_
#define _APP_PROTECT_DVP_H_
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
void apiProtectDvpOpen(tAppProtectEvtHandler evtHandler);
uint8_t apiProtectDvpHandler(void);
uint8_t	apiProtectDvpGetFlag(void);

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif


#endif /* _APP_PROTECT_UVP_H_ */


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



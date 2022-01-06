/**
  ******************************************************************************
  * @file        ApiProtectOvp.h
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
#ifndef _API_PROTECT_OVP_H_
#define _API_PROTECT_OVP_H_
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
void apiProtectOvpOpen(tAppProtectEvtHandler evtHandler);
void apiProtectOvpPfHandler(void);
uint8_t apiProtectOvpHandler(uint8_t ProtectLevel);
uint8_t	apiProtectOvpGetFlag(uint16_t CellIndex);
uint8_t	apiProtectOvpPfGetFlag(void);
void apiProtectOvpPfClean(void);

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif


#endif /* _API_PROTECT_OVP_H_ */


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



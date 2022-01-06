/**
  ******************************************************************************
  * @file        HalEEProm.h
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/05
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _HAL_EEPROM_H_
#define _HAL_EEPROM_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "LibDebug.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/
typedef struct{
	uint32_t	StartAddress;
	uint32_t	EndAddress;
	uint16_t	Length;
	uint8_t		*pDataBuffer;
}tHalEeProm;

/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
tErrCode HalEePromWrite(tHalEeProm *pEeProm);
tErrCode HalEePromErase(tHalEeProm *pEeProm);
tErrCode HalEePromRead(tHalEeProm *pEeProm);

#endif /* _HAL_EEPROM_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



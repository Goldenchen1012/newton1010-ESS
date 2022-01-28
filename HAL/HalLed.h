/**
  ******************************************************************************
  * @file        HalLed.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2022/1/26
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */
#ifndef _HAL_LED_H_
#define _HAL_LED_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/

/* Public macro -------------------------------------------------------------*/
void halLedSetPercent20LedState(uint8_t on);
void halLedSetPercent40LedState(uint8_t on);
void halLedSetPercent60LedState(uint8_t on);
void halLedSetPercent80LedState(uint8_t on);
void halLedSetPercent100LedState(uint8_t on);
void halLedSetChargeLedState(uint8_t on);
void halLedSeDisChargeLedState(uint8_t on);
void halLedSeAlarmLedState(uint8_t on);
void halLedSeCommLedState(uint8_t on);
void halLedAllLedOn(void);
void halLedAllLedOff(void);

void halLedOpen(void);



#ifdef __cplusplus
}
#endif


	

#endif /* _HAL_LED_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


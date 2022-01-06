/**
  ******************************************************************************
  * @file        LibNtc.h
  * @author      Johnny
  * @version     v1.0
  * @date        2021/10/25
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _LIB_NTC_H_
#define _LIB_NTC_H_

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

	
/* Public define ------------------------------------------------------------*/
	
/* Public typedef -----------------------------------------------------------*/

/* Public macro -------------------------------------------------------------*/

/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
uint16_t LibTemperatureToVoltage(int16_t temp);
uint16_t LibNtcRToTemperature(double NtcR);
uint16_t LibNtcVoltageToTemperature(uint16_t NtcVoltage);
uint16_t LibSetRealTemperatureToInternalValue(int16_t temp);


#ifdef __cplusplus
}
#endif

#endif /* _LIB_NTC_H_ */

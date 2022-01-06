/**
  ******************************************************************************
  * @file        HalRtc.h
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

#ifndef _HAL_RTC_H_
#define _HAL_RTC_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/
typedef struct{
	uint16_t	Year;
	uint8_t		Month;
	uint8_t		Day;
	uint8_t		Hour;
	uint8_t		Minute;
	uint8_t		Second;
}tHalRtcDateTime;
/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
tErrCode HalRtcOpen(void);
tErrCode HalRtcClose(void);
tErrCode HalRtcSetupDate(uint16_t year, uint8_t mon, uint8_t day);
tErrCode HalRtcSetupTime(uint8_t hour, uint8_t min, uint8_t sec);
void HalRtcGetDateTime(tHalRtcDateTime *pDateTime);
uint32_t HalRtcGetSmpUnixTime(void);
void HalRtcSmpUnixTimeToDateTime(uint32_t sec, tHalRtcDateTime *pRtcDateTime);
uint8_t	halRtcIsRtcValid(void);


#endif /* _HAL_RTC_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



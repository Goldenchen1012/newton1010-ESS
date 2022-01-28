/**
  ******************************************************************************
  * @file        AppLed.h
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

#ifndef _APP_LED_H_
#define _APP_LED_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/* Public typedef -----------------------------------------------------------*/

/* Public define ------------------------------------------------------------*/
enum{
	LED_STATE_CAPACITY = 1,
	LED_STATE_WAIT_SYS_READY,
	LED_STATE_SCU_ID,
	LED_STATE_SOC_OFF,
	LED_STATE_ALARM,
	LED_STATE_COMM,
	LED_STATE_CRH,
	LED_STATE_DSG	
};

/* Public macro -------------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
void appLedOpen(void);
void appLedSetState(uint16_t ShowTime, uint8_t state, uint16_t par);


#ifdef __cplusplus
}
#endif


	

#endif /* _APP_LED_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



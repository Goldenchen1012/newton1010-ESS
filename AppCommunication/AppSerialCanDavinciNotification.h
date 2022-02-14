/**
  ******************************************************************************
  * @file        AppSerialCanDavinciNotification.h
  * @author      Johnny
  * @version     v0.0.1
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

#ifndef _APP_SERIAL_CAN_DAVINCI_NOTIFICATION_H_
#define _APP_SERIAL_CAN_DAVINCI_NOTIFICATION_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "smp_can_fifo.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
uint8_t appSerialCanDavinciNotificationGetLastBroadcastScuId(void);
void appSerialCanDavinciNotificationSetLastBroadcastScuId(uint8_t scuid);

void appSerialCanDavinciNotificationHandler(uint16_t evt);



#ifdef __cplusplus
}
#endif


	

#endif /* _APP_SERIAL_CAN_DAVINCI_NOTIFICATION_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


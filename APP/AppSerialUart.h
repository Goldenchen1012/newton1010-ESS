/**
  ******************************************************************************
  * @file        AppSerialUart.h
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/12
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _APP_SERIAL_UART_H_
#define _APP_SERIAL_UART_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
void appSerialUartHvEssHanlder(uint8_t evt);
void appSerialUartHvEssOpen(void);


/* Public macro -------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif


	

#endif /* _APP_SERIAL_UART_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



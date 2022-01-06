/**
  ******************************************************************************
  * @file        HalUart.h
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

#ifndef _HAL_UART_H_
#define _HAL_UART_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
typedef struct{
	uint8_t		PortNo;
	uint32_t	Baudrate;
	uint8_t		StopBits;
	uint8_t		Parity;
}tHalUart;

/* Public typedef -----------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
tErrCode HalUartTx(tHalUart *pHalUart, uint8_t *pSendBuf, uint16_t leng);
tErrCode HalUartOpen(tHalUart *pHalUart);
tErrCode HalUartClose(tHalUart *pHalUart);
uint8_t IsUart2FifoEmpty(void);
uint8_t	GetUart2FifoData(void);

#endif /* _HAL_UART_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



/**
  ******************************************************************************
  * @file        HalSpi.h
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

#ifndef _HAL_SPI_H_
#define _HAL_SPI_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "LibDebug.h"


#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
typedef struct{
	uint8_t		PortNo;
}tHalSpi;

/* Public typedef -----------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
tErrCode HalSpiOpen(tHalSpi *pHalSpi);
tErrCode HalSpiClose(tHalSpi *pHalSpi);


#endif /* _HAL_SPI_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



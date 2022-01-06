/**
  ******************************************************************************
  * @file        HalCan.h
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

#ifndef _HAL_CAN_H_
#define _HAL_CAN_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "LibDebug.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
typedef struct{
	uint8_t		PortNo;
	uint32_t	Baudrate;
}tHalCan;

typedef struct{
	uint32_t	CanId;
	uint8_t		DLC;
	uint8_t		DataBuf[8];
	uint8_t		RtrFlag;
	uint8_t		StdFrameFlag;
}tHalCanTxMessage;


/* Public typedef -----------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
tErrCode HalCanOpen(tHalCan *pHalCan);


#endif /* _HAL_CAN_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



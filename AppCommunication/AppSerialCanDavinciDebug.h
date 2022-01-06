/**
  ******************************************************************************
  * @file        AppSerialCanDavinciDebug.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/10/20
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _APP_SERIAL_CAN_DAVINCI_DEBUG_H_
#define _APP_SERIAL_CAN_DAVINCI_DEBUG_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "smp_can_fifo.h"
#include "SmpCanBusProtocol.h"
#include "SmpParameterID.h"

#ifdef __cplusplus
extern "C" {
#endif
/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
void DavinciCanFunDebugRx(smp_can_package_t *pCanPkg);


#ifdef __cplusplus
}
#endif


	

#endif /* _APP_SERIAL_CAN_DAVINCI_DEBUG_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    





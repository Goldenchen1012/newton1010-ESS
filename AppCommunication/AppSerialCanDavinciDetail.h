/**
  ******************************************************************************
  * @file        AppSerialCanDavinciDetail.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2022/2/10
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _APP_SERIAL_CAN_DAVINCI_DETAIL_H_
#define _APP_SERIAL_CAN_DAVINCI_DETAIL_H_
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
void DavinciCanFunDetailTx(smp_can_package_t *pCanPkg);


#ifdef __cplusplus
}
#endif


	

#endif /* _APP_SERIAL_CAN_DAVINCI_DETAIL_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    





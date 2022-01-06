/**
  ******************************************************************************
  * @file        AppSerialCanDavinciCmd.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/19
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _APP_SERIAL_CAN_DAVINCI_CMD_H_
#define _APP_SERIAL_CAN_DAVINCI_CMD_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
void DavinciCanFunCmdRx(smp_can_package_t *pCanPkg);

#ifdef __cplusplus
}
#endif


	

#endif /* _APP_SERIAL_UART_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



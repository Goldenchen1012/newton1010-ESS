/**
  ******************************************************************************
  * @file        AppScuTemp.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/15
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _APP_SCU_TEMP_H_
#define _APP_SCU_TEMP_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/* Public typedef -----------------------------------------------------------*/

/* Public define ------------------------------------------------------------*/

/* Public macro -------------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
uint16_t apiScuTempGetTemperature(uint8_t index);

void apiScuTempOpen(void);

#ifdef __cplusplus
}
#endif


	

#endif /* _APP_SCU_TEMP_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



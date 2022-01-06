/********************************************************************************
  * @file        LibFaultLog.h
  * @author      Norman
  * @version     v1.0
  * @date        2019/11/18
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Norman</center></h2>
  *
  *
  ******************************************************************************
  */
#ifndef _LIB_FAULT_LOG_H
#define _LIB_FAULT_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

 
/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/

/* Public variables ---------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
void SaveFaultData(uint8_t type, uint16_t cell);
void NoClear_SaveFaultData(uint8_t type,uint16_t cell);
void libFaultLogOpen(void);

#ifdef __cplusplus
}
#endif

#endif 

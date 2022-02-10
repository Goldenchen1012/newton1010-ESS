/**
  ******************************************************************************
  * @file        AppScuIdAssign.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2022/02/09
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _APP_SCU_ID_ASSIGN_H_
#define _APP_SCU_ID_ASSIGN_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/* Public typedef -----------------------------------------------------------*/

/* Public define ------------------------------------------------------------*/

/* Public macro -------------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
void appBmsSendFindFirstScuCanPackage(void);

void appBmsStopOutputAssignIdSignal(void);

uint8_t appBmsIsInAssignIdMode(void);
uint8_t appBmsIsInScuIdRequestMode(void);
void appBmsAssignScuId(uint8_t id);
void appBmsExitScuIdRequestMode(void);
void appBmsEnterScuIdAssignMode(void);
void appBmsRcvScuIdBrocast(uint8_t scuid);

void appBmsFindFirstScu(void);
void appBmsResetScuId(void);
uint8_t appBmsGetScuId(void);

uint8_t appBmsIsScudIdReady(void);
uint8_t appBmsIsMaster(void);
void appScuIdAssignOpen(void);


#ifdef __cplusplus
}
#endif


	

#endif /* _APP_SCU_ID_ASSIGN_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



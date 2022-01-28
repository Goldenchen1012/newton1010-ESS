/**
  ******************************************************************************
  * @file        AppBms.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/10
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _APP_BMS_H_
#define _APP_BMS_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/* Public typedef -----------------------------------------------------------*/

/* Public define ------------------------------------------------------------*/

/* Public macro -------------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
void appBmsSetScuSystemFlag1(uint8_t scuid, uint32_t flag1, uint32_t flag2);
void appBmsSetScuSystemFlag2(uint8_t scuid, uint32_t flag1, uint32_t flag2);
void appBmsSetScuCurrent(uint8_t scuid, int32_t CurrentP, int32_t CurrentN);
void appBmsSetScuVbat(uint8_t scuid, uint32_t VbInt, uint32_t VbExt);
void appBmsSetMinCellVoltage(uint8_t scuid, uint16_t MinV, uint8_t bmu, uint8_t posi);
void appBmsSetMaxCellVoltage(uint8_t scuid, uint16_t MaxV, uint8_t bmu, uint8_t posi);
void appBmsSetMinNtcTemp(uint8_t scuid, uint16_t MinT, uint8_t bmu, uint8_t posi);
void appBmsSetMaxNtcTemp(uint8_t scuid, uint16_t MaxT, uint8_t bmu, uint8_t posi);


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
void appBmsOpen(void);

#ifdef __cplusplus
}
#endif


	

#endif /* _APP_BMS_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



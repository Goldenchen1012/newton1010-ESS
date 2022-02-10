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
#define	SYSTEM_MAX_SCU_NUM		32

/* Public macro -------------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
uint8_t appBmsSetScuSystemFlag1_2(uint8_t scuid, uint32_t flag1, uint32_t flag2);
uint8_t appBmsGetScuSystemFlag1_2(uint8_t scuid, uint32_t *flag1, uint32_t *flag2);

uint8_t appBmsSetScuSystemFlag3_4(uint8_t scuid, uint32_t flag3, uint32_t flag4);
uint8_t appBmsGetScuSystemFlag3_4(uint8_t scuid, uint32_t *flag3, uint32_t *flag4);

uint8_t appBmsSetScuCurrent(uint8_t scuid, int32_t CurrentP, int32_t CurrentN);
uint8_t appBmsGetScuCurrent(uint8_t scuid, int32_t *CurrentP, int32_t *CurrentN);

uint8_t appBmsSetScuVbat(uint8_t scuid, uint32_t VbInt, uint32_t VbExt);
uint8_t appBmsGetScuVbat(uint8_t scuid, uint32_t *VbInt, uint32_t *VbExt);

uint8_t appBmsSetMinCellVoltage(uint8_t scuid, uint16_t MinV, uint8_t bmu, uint8_t posi);
uint8_t appBmsGetMinCellVoltage(uint8_t scuid, uint16_t *MinV, uint8_t *bmu, uint8_t *posi);

uint8_t appBmsSetMaxCellVoltage(uint8_t scuid, uint16_t MaxV, uint8_t bmu, uint8_t posi);
uint8_t appBmsGetMaxCellVoltage(uint8_t scuid, uint16_t *MaxV, uint8_t *bmu, uint8_t *posi);

uint8_t appBmsSetMinNtcTemp(uint8_t scuid, uint16_t MinT, uint8_t bmu, uint8_t posi);
uint8_t appBmsGetMinNtcTemp(uint8_t scuid, uint16_t *MinT, uint8_t *bmu, uint8_t *posi);

uint8_t appBmsSetMaxNtcTemp(uint8_t scuid, uint16_t MaxT, uint8_t bmu, uint8_t posi);
uint8_t appBmsGetMaxNtcTemp(uint8_t scuid, uint16_t *MaxT, uint8_t *bmu, uint8_t *posi);

uint8_t appBmsSetCellVoltage(uint8_t scuid, uint16_t cells, uint16_t voltage);
uint8_t appBmsGetCellVoltage(uint8_t scuid, uint16_t cells, uint16_t *voltage);
uint8_t appBmsSetNtcVoltage(uint8_t scuid, uint16_t ntcs, uint16_t voltage);
uint8_t appBmsGetNtcVoltage(uint8_t scuid, uint16_t ntcs, uint16_t *voltage);

uint8_t appBmsIsScuCanTurnOnRelay(uint8_t scuid);
uint8_t appBmsIsScudIdReady(void);
uint8_t appBmsIsMaster(void);

uint8_t appBmsIsScuDataValid(uint8_t scuid);

uint8_t appBmsIsValidScuid(uint8_t scuid);
void putSelfDataToBmsBuffer(void);
uint8_t appBmsIsScuRelayOn(uint8_t scuid);


void appBmsOpen(void);

#ifdef __cplusplus
}
#endif


	

#endif /* _APP_BMS_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



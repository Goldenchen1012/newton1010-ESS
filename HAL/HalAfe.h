/**
  ******************************************************************************
  * @file        HalAfe.h
  * @author      Johnny
  * @version     v0.0
  * @date        2021/9/7
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef HAL_AFE_H_
#define HAL_AFE_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "LibRegister.h"

#ifdef __cplusplus
extern "C" {
#endif

enum{
	AFE_STATE_NORMAL = 0,
	AFE_STATE_INI,
};
	
enum{
	AFE_EVT_COMM_L1_SET = 1,
	AFE_EVT_COMM_L1_RELEASE,
	AFE_EVT_COMM_L2_SET,
	AFE_EVT_COMM_L2_RELEASE,
	AFE_EVT_END
};

enum{
	AFE_VBAT_INDEX = 0,
	AFE_VPACK_INDEX
};


#define	MAX_CELL_NUMBER		400
#define	MAX_NTC_NUMBER		400

/* Public define ------------------------------------------------------------*/
#define tAfeEvtHandler tLibRegisterEvtHandler
typedef void(*tAfeLineLossCallBack)(uint16_t channel, uint16_t *cellVoltage);

#define	tCellVoltage		uint16_t
#define	tNtcAdcData			uint16_t
#define	tBatteryVoltage		uint32_t
#define	tPackVoltage		uint32_t
#define	tCurrent			int32_t


int32_t	halAfeGetCurrentAdcValue(uint8_t CurrentIndex);
void halAfeSetCurrentAdcValue(uint8_t CurrentIndex,int32_t adcvalue);
int32_t	halAfeGetVBatAdcValue(uint8_t VbIndex);
void halAfeSetVBatAdcValue(uint8_t VbIndex,int32_t adcvalue);

tCellVoltage halAfeGetCellVoltage(uint16_t CellIndex);
tCurrent halAfeGetCurrentValue(uint8_t index);
tNtcAdcData HalAfeGetNtcAdc(uint16_t NtcIndex);
void halAfeSetCellVoltage(uint16_t cell, tCellVoltage voltage);
void halAfeSetNtcAdcData(uint16_t ntcs, tNtcAdcData adcdata);
void HalAfeSetCurrentValue(uint8_t index, int32_t current);
uint32_t halAfeGetVBatVoltage(uint8_t index);
void halAfeSetVBatVoltage(uint8_t index, uint32_t voltage);

tCellVoltage halAfeGetMaxCellVoltage(void);
tCellVoltage halAfeGetMinCellVoltage(void);
tNtcAdcData HalAfeGetMinNtcTempAdc(void);
tNtcAdcData HalAfeGetMaxNtcTempAdc(void);

void halAfeUpdateMinMaxCellVoltage(void);
void halAfeUpdateMinMaxNtcTempVoltage(void);

void halafeOpen(tAfeEvtHandler evtHandler, tAfeLineLossCallBack lineLossCb);
void halAfeSetPhysicalBalancePosition(uint8_t bmuindex, uint16_t position);
void halAfeSetBalanceOnFlag(uint8_t onflag);
uint8_t halAfeGetState(void);
uint8_t halAfeIsL1Protect(void);
uint8_t halAfeIsL2Protect(void);
void halAfeCalVbatFromCellVoltage(void);


#ifdef __cplusplus
}
#endif


	

#endif /* HAL_AFE_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    

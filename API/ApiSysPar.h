/**
  ******************************************************************************
  * @file        ApiSysPar.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/29
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _API_SYS_PAR_H_
#define _API_SYS_PAR_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "define.h"
#include "LibCalibration.h"


#ifdef __cplusplus
extern "C" {
#endif


#define	APP_SYS_PAR_MAX_OCV_TABLE_NUM	25
#define	APP_SYS_PAR_MAX_RA_TABLE_NUM	25


#define	CALI_P1_INDEX	0
#define	CALI_P2_INDEX	1

typedef struct{
    int32_t 	valL;
    int32_t 	valH;
    int32_t 	adcL;
    int32_t 	adcH;
}tCaliPar;


typedef struct{
	tLbyte			SetValue;
	tLbyte			STime;
	tLbyte			RelValue;
	tLbyte			RTime;
}tScuProtectPar;


typedef struct{
		uint8_t		Level;
		uint16_t	Value;
}tOcvRaTable;

typedef struct{
	uint8_t			HeadInfo[8];
	uint16_t		ParLeng;
	uint16_t		DateCode;
	uint32_t		Checksum;
	tCaliPar		Currentt[2];
	tCaliPar		VBat[2];
	uint32_t	Reserved;
}tCalRomPar;

typedef struct{
	tCalRomPar			RomPar;
	struct{	
		tCalibCoef		Current[2];
		tCalibCoef		VBat[2];
	}RamPar;
}tSysCalPar;

extern tSysCalPar	SysCalPar;

/* Public define ------------------------------------------------------------*/
#define	MAX_NOTE_MESSAGE_STRING_ITEM	100

/* Public macro -------------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
//	Battery Info
uint8_t apiSysParIsOvpPfSet(void);
uint8_t apiSysParIsUvpPfSet(void);
void apiSysParOvpPfClean(void);
void apiSysParOvpPfSet(void);
void apiSysParUvpPfClean(void);
void apiSysParUvpPfSet(void);

//	Cali
void apiCaliParSetCurrentValue(uint8_t CurrentNum, uint8_t PointIndex,int32_t Value, int32_t Adc);
void apiCaliParGetCurrentValue(uint8_t CurrentNum, uint8_t PointIndex,int32_t *Value, int32_t *Adc);
void apiCaliParSetVbatValue(uint8_t CurrentIndex, uint8_t PointIndex,int32_t Value, int32_t Adc);
void apiCaliParGetVbatValue(uint8_t CurrentNum, uint8_t PointIndex,int32_t *Value, int32_t *Adc);
uint32_t apiCaliParGetChecksum(void);





// system par
uint32_t apiSysParGetHwVersion(void);
void apiSysParSetHwVersion(uint32_t version);
uint32_t apiSysParGetFwVersion(void);
void appSysParSetFwVersion(uint32_t version);

uint8_t apiSysParGetBmuNumInModule(void);
void apiSysParSetBmuNumInModule(uint8_t BmuNumber);

uint8_t apiSysParGetBmuNumber(void);
void apiSysParSetBmuNumber(uint8_t BmuNumber);
uint32_t apiSysParGetCellFlag(uint8_t BmuIndex);
void apiSysParSetCellFlag(uint8_t BmuIndex,uint32_t CellFlag);
uint32_t apiSysParGetNtcFlag(uint8_t BmuIndex);
void apiSysParSetNtcFlag(uint8_t BmuIndex,uint32_t NtcFlag);

uint16_t apiSysParGetZeroCurrentValue(void);
void apiSysParSetZeroCurrentValue(uint16_t current);
uint16_t apiSysParGetMinChargeCurrentValue(void);
void apiSysParSetMinChargeCurrentValue(uint16_t current);

uint32_t apiSysParGetDesignedCapacity(void);
void apiSysParSetDesignedCapacity(uint32_t dc);

uint32_t apiSysParGetRelayActiveFlag(void);
void apiSysParSetRelayActiveFlag(uint32_t flag);

void apiSysParGetMaxCurrentValue(tScuProtectPar *pPar);
void apiSysParSetMaxCurrentValue(tScuProtectPar *pPar);

void apiSysParGetMaxPeakCurrentValue(tScuProtectPar *pPar);
void apiSysParSetMaxPeakCurrentValue(tScuProtectPar *pPar);

void apiSysParGetRateVoltage(tScuProtectPar *pPar);
void apiSysParSetRateVoltage(tScuProtectPar *pPar);

void apiSysParGetFullChargeCondition(tScuProtectPar *pPar);
void apiSysParSetFullChargeCondition(tScuProtectPar *pPar);


uint16_t apiSysParGetMinFlatVoltage(void);
uint16_t apiSysParGetMaxFlatVoltage(void);

void apiSysParGetFlatVoltage(tScuProtectPar *pPar);
void apiSysParSetFlatVoltage(tScuProtectPar *pPar);



uint16_t apiSysParGetCellNumber(void);
uint16_t apiSysParGetNtcNumber(void);

void apiSysParGetOcvTable(uint8_t index ,tOcvRaTable *pOcvTable);
void apiSysParGetRaTable(uint8_t index ,tOcvRaTable *pOcvTable);
void apiSysParSetOcvTable(uint8_t index ,tOcvRaTable *pOcvTable);
void apiSysParSetRaTable(uint8_t index ,tOcvRaTable *pOcvTable);

void apiSysParGetAfeCommTime(tScuProtectPar *pPar);
void apiSysParSetAfeCommTime(tScuProtectPar *pPar);
void apiSysParGetInsulationResistance(tScuProtectPar *pPar);
void apiSysParSetInsulationResistance(tScuProtectPar *pPar);

uint16_t apiSysParGetTerminateVoltage(void);
void apiSysParSetTerminateVoltage(uint16_t voltage);
uint16_t apiSysParGetPreDischargeTime(void);
void apiSysParSetPreDischargeTime(uint16_t time);
uint16_t apiSysParGetRelayOnDiffVoltage(void);
void apiSysParSetRelayOnDiffVoltage(uint16_t voltage);

uint8_t apiSysParGetScuId(void);
void saveScuIdPar(uint8_t scuid);

//protect
//uint16_t appSysParGetOvpSetValue(uint8_t level);
uint16_t appSysParGetOvpSetTime(uint8_t level);
uint16_t appSysParGetOvpReleaseValue(uint8_t level);
uint16_t appSysParGetOvpReleaseTime(uint8_t level);

void apiSysParGetOvpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetOvpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetUvpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetUvpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetDvpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetDvpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetModuleDvpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetModuleDvpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGet2ndOtProtectPar(tScuProtectPar *pPar);
void apiSysParSet2ndOtProtectPar(tScuProtectPar *pPar);
void apiSysParGet2ndUtProtectPar(tScuProtectPar *pPar);
void apiSysParSet2ndUtProtectPar(tScuProtectPar *pPar);

void apiSysParGetCotpProtectPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParGetCotpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetCotpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetCutpProtectPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParGetCutpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetCutpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetDotpProtectPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParGetDotpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetDotpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetDutpProtectPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParGetDutpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetDutpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetDtpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetDtpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetModuleDtpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetModuleDtpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetCocpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetCocpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetDocpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetDocpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetOvpPfPar(tScuProtectPar *pPar);
void apiSysParSetOvpPfPar(tScuProtectPar *pPar);

void apiSysParGetUvpPfPar(tScuProtectPar *pPar);
void apiSysParSetUvpPfPar(tScuProtectPar *pPar);

void apiSysParGetBalanceDuty(tScuProtectPar *pPar);
void apiSysParSetBalanceDuty(tScuProtectPar *pPar);
void apiSysParGetBalanceChg(tScuProtectPar *pPar);
void apiSysParSetBalanceChg(tScuProtectPar *pPar);
void apiSysParGetBalanceDhg(tScuProtectPar *pPar);
void apiSysParSetBalanceDhg(tScuProtectPar *pPar);
void apiSysParGetBalanceRlx(tScuProtectPar *pPar);
void apiSysParSetBalanceRlx(tScuProtectPar *pPar);

void apiSysParGetNotwMessageString(uint8_t *pMsg);
void apiSysParSetNotwMessageString(uint8_t *pMsg);

uint32_t apiSysParGetQmax(void);
void apiSysParSetQmax(uint32_t Qmax);
uint16_t apiSysParGetQmaxUpdateTimes(void);
void apiSysParSetQmaxUpdateTimes(uint16_t times);
uint16_t apiSysParGetCycleCount(void);
void apiSysParSetCycleCount(uint16_t count);
uint16_t apiSysParGetPfFlag(void);
void apiSysParSetPfFlag(uint16_t flag);
uint32_t apiSysParGetChecksum(void);

void apiSysParSetScuOtPar(uint8_t index, uint8_t level, tScuProtectPar *pPar);
void apiSysParGetScuOtPar(uint8_t index, uint8_t level, tScuProtectPar *pPar);

uint16_t apiSysParOpen(void);


#ifdef __cplusplus
}
#endif


	

#endif /* _API_SYS_PAR_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



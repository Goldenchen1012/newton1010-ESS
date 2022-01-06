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

#ifndef _APP_SYSTEM_PAR_H
#define _APP_SYSTEM_PAR_H
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
void appCaliParSetCurrentValue(uint8_t CurrentNum, uint8_t PointIndex,int32_t Value, int32_t Adc);
void appCaliParGetCurrentValue(uint8_t CurrentNum, uint8_t PointIndex,int32_t *Value, int32_t *Adc);
void appCaliParSetVbatValue(uint8_t CurrentIndex, uint8_t PointIndex,int32_t Value, int32_t Adc);
void appCaliParGetVbatValue(uint8_t CurrentNum, uint8_t PointIndex,int32_t *Value, int32_t *Adc);


#define	MAX_NOTE_MESSAGE_STRING_ITEM	100



// system par
uint32_t appSysParGetHwVersion(void);
void appSysParSetHwVersion(uint32_t version);
uint32_t appSysParGetFwVersion(void);
void appSysParSetFwVersion(uint32_t version);


uint8_t appSysParGetBmuNumber(void);
void appSysParSetBmuNumber(uint8_t BmuNumber);
uint32_t appSysParGetCellFlag(uint8_t BmuIndex);
void appSysParSetCellFlag(uint8_t BmuIndex,uint32_t CellFlag);
uint32_t appSysParGetNtcFlag(uint8_t BmuIndex);
void appSysParSetNtcFlag(uint8_t BmuIndex,uint32_t NtcFlag);

uint32_t appSysParGetQmax(void);

uint16_t appSysParGetZeroCurrentValue(void);
void appSysParSetZeroCurrentValue(uint16_t current);
uint16_t appSysParGetMinChargeCurrentValue(void);
void appSysParSetMinChargeCurrentValue(uint16_t current);

uint32_t appSysParGetDesignedCapacity(void);
void appSysParSetDesignedCapacity(uint32_t dc);

void appSysParGetFullChargeCondition(tScuProtectPar *pPar);
void appSysParSetFullChargeCondition(tScuProtectPar *pPar);


uint16_t appSysParGetMinFlatVoltage(void);
uint16_t appSysParGetMaxFlatVoltage(void);

void appSysParGetFlatVoltage(tScuProtectPar *pPar);
void appSysParSetFlatVoltage(tScuProtectPar *pPar);



uint16_t appSysParGetCellNumber(void);
uint16_t appSysParGetNtcNumber(void);

void appSysParGetOcvTable(uint8_t index ,tOcvRaTable *pOcvTable);
void appSysParGetRaTable(uint8_t index ,tOcvRaTable *pOcvTable);
void appSysParSetOcvTable(uint8_t index ,tOcvRaTable *pOcvTable);
void appSysParSetRaTable(uint8_t index ,tOcvRaTable *pOcvTable);


//protect
//uint16_t appSysParGetOvpSetValue(uint8_t level);
uint16_t appSysParGetOvpSetTime(uint8_t level);
uint16_t appSysParGetOvpReleaseValue(uint8_t level);
uint16_t appSysParGetOvpReleaseTime(uint8_t level);

void appSysParGetOvpPar(uint8_t level, tScuProtectPar *pPar);
void appSysParSetOvpPar(uint8_t level, tScuProtectPar *pPar);

void appSysParGetUvpPar(uint8_t level, tScuProtectPar *pPar);
void appSysParSetUvpPar(uint8_t level, tScuProtectPar *pPar);

void appSysParGet2ndOtProtectPar(tScuProtectPar *pPar);
void appSysParSet2ndOtProtectPar(tScuProtectPar *pPar);
void appSysParGet2ndUtProtectPar(tScuProtectPar *pPar);
void appSysParSet2ndUtProtectPar(tScuProtectPar *pPar);

void appSysParGetCotpProtectPar(uint8_t level, tScuProtectPar *pPar);
void appSysParGetCotpPar(uint8_t level, tScuProtectPar *pPar);
void appSysParSetCotpPar(uint8_t level, tScuProtectPar *pPar);

void appSysParGetCutpProtectPar(uint8_t level, tScuProtectPar *pPar);
void appSysParGetCutpPar(uint8_t level, tScuProtectPar *pPar);
void appSysParSetCutpPar(uint8_t level, tScuProtectPar *pPar);

void appSysParGetDotpProtectPar(uint8_t level, tScuProtectPar *pPar);
void appSysParGetDotpPar(uint8_t level, tScuProtectPar *pPar);
void appSysParSetDotpPar(uint8_t level, tScuProtectPar *pPar);

void appSysParGetDutpProtectPar(uint8_t level, tScuProtectPar *pPar);
void appSysParGetDutpPar(uint8_t level, tScuProtectPar *pPar);
void appSysParSetDutpPar(uint8_t level, tScuProtectPar *pPar);

void appSysParGetDtpPar(uint8_t level, tScuProtectPar *pPar);
void appSysParSetDtpPar(uint8_t level, tScuProtectPar *pPar);

void appSysParGetCocpPar(uint8_t level, tScuProtectPar *pPar);
void appSysParSetCocpPar(uint8_t level, tScuProtectPar *pPar);

void appSysParGetDocpPar(uint8_t level, tScuProtectPar *pPar);
void appSysParSetDocpPar(uint8_t level, tScuProtectPar *pPar);

void appSysParGetOvpPfPar(tScuProtectPar *pPar);
void appSysParSetOvpPfPar(tScuProtectPar *pPar);

void appSysParGetUvpPfPar(tScuProtectPar *pPar);
void appSysParSetUvpPfPar(tScuProtectPar *pPar);

void appSysParGetBalanceDuty(tScuProtectPar *pPar);
void appSysParSetBalanceDuty(tScuProtectPar *pPar);
void appSysParGetBalanceChg(tScuProtectPar *pPar);
void appSysParSetBalanceChg(tScuProtectPar *pPar);
void appSysParGetBalanceDhg(tScuProtectPar *pPar);
void appSysParSetBalanceDhg(tScuProtectPar *pPar);
void appSysParGetBalanceRlx(tScuProtectPar *pPar);
void appSysParSetBalanceRlx(tScuProtectPar *pPar);

void appSysParGetNotwMessageString(uint8_t *pMsg);
void appSysParSetNotwMessageString(uint8_t *pMsg);
/* Public macro -------------------------------------------------------------*/

uint16_t appSysParOpen(void);

#ifdef __cplusplus
}
#endif


	

#endif /* HAL_AFE_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



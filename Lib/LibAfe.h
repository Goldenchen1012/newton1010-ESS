/**
  ******************************************************************************
  * @file        LibAfe.h
  * @author      Norman
  * @version     v1.0
  * @date        2019/8/30
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Norman</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef LIB_AFE_H_
#define LIB_AFE_H_

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "LibRegister.h"
	
/* Public define ------------------------------------------------------------*/
#define tAfeEvtHandler tLibRegisterEvtHandler
#define t100uV uint16_t
#define tmV uint32_t
#define tuA int32_t
#define tOm uint16_t
#define tuOm int32_t
	
#if (AFE_ADC_DATA_LEN == AFE_ADC_DATA_LEN_16)
#define tAdcData uint16_t
#else
#define tAdcData uint32_t
#endif
#define tAdcDataS int32_t
	
/* Public typedef -----------------------------------------------------------*/

	
enum{
  LIB_AFE_EVT_DATE_UPDATED = 0,
  LIB_AFE_EVT_SHUTDOWN,
  LIB_AFE_EVT_SCD,
  LIB_AFE_EVT_RELEASED_SCD,
  LIB_AFE_EVT_ERROR,
  LIB_AFE_EVT_MAX
};

typedef enum{
  LIB_AFE_MOS_STATE_OFF = 0,
  LIB_AFE_MOS_STATE_DIS,
  LIB_AFE_MOS_STATE_CHG,
  LIB_AFE_MOS_STATE_ON
} tLibAfeMosState;

typedef enum{
  LIB_AFE_STATE_UNINIT = 0,
  LIB_AFE_STATE_WORKING,
  LIB_AFE_STATE_PROTECTION
} tLibAfeState;

typedef union
{
  struct
  {
    uint8_t ov:1;                   
    uint8_t uv:1; 
    uint8_t occ:1; 
    uint8_t ocd:1; 
    uint8_t scd:1; 
    uint8_t cvdduv:1; 
    uint8_t _reserved:2; 
  } bits;                              
  uint8_t b;                           
} tHalAfeStateProtection;

#if 0
typedef union{
	struct {
	    uint32_t c1:1;
	    uint32_t c2:1;
	    uint32_t c3:1;
	    uint32_t c4:1;
	    uint32_t c5:1;
	    uint32_t c6:1;
	    uint32_t c7:1;
	    uint32_t c8:1;
	    uint32_t c9:1;
	    uint32_t c10:1;
	    uint32_t c11:1;
	    uint32_t c12:1;
	    uint32_t c13:1;
	    uint32_t c14:1;
	    uint32_t c15:1;
	    uint32_t c16:1;
	    uint32_t c17:1;
	    uint32_t c18:1;
	    uint32_t c19:1;
	    uint32_t c20:1;
	    uint32_t c21:1;
	    uint32_t c22:1;
	    uint32_t c23:1;
	    uint32_t c24:1;
	    uint32_t c25:1;
	    uint32_t c26:1;
	    uint32_t c27:1;
	    uint32_t c28:1;
	    uint32_t c29:1;
	    uint32_t c30:1;
	    uint32_t c31:1;
    } bit;
	uint8_t b[4];
	uint16_t w[2];
	uint32_t l;
} tCellState;
#else
#define  tCellState uint32_t
#endif

typedef struct {  
    int32_t A1;    
    int32_t A2;  
    int32_t B;
} tCalibCoef;
/* Public macro -------------------------------------------------------------*/

/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
tErrCode libAfeInit(void);
void libAfeOpen(tAfeEvtHandler evtHandler);
void libAfeClose(tAfeEvtHandler evtHandler);
void libAfeShutdown(void);
void libAfeProtectionRecover(void);  


void libAfeAdcSetMeasuringInterval(uint16_t ms);  
tErrCode libAfeSetMos(tLibAfeMosState act);
tErrCode libAfeSetGpo(uint8_t id, bool value);
tuA libAfeAdcSetPackCurrentLsbToI(tAdcDataS lsb);
tAdcDataS libAfeAdcSetPackCurrentI2Lsb(tuA current);
tErrCode libAfeSetBalance(tCellState cellsSel);
tErrCode libAfeSetCalibrationParCell(uint8_t num, tCalibCoef coef);
tErrCode libAfeSetCalibrationParVb(tCalibCoef coef);
tErrCode libAfeSetCalibrationParVp(tCalibCoef coef);
tErrCode libAfeSetCalibrationParCurrent(tCalibCoef coef);


t100uV libAfeGetCellVoltage(uint8_t num);
tmV libAfeGetBatteryVoltage(void);
tmV libAfeGetPackVoltage(void);
tuA libAfeGetPackCurrent(void);
t100uV libAfeGetNtcVoltage(uint8_t num);
tOm libAfeGetNtcUpR(uint8_t num);  
t100uV libAfeGetNtcVdd(void);  
tAdcData libAfeGetAdcCellVoltage(uint8_t num);
tAdcData libAfeGetAdcBatteryVoltage(void);
tAdcData libAfeGetAdcPackVoltage(void);
tAdcDataS libAfeGetAdcPackCurrent(void);
tCalibCoef libAfeGetCalibrationParCell(uint8_t num);
tCalibCoef libAfeGetCalibrationParVb(void);
tCalibCoef libAfeGetCalibrationParVp(void);
tCalibCoef libAfeGetCalibrationParCurrent(void);

tLibAfeState libAfeGetState(void);
tLibAfeMosState libAfeGetMosState(void);
tHalAfeStateProtection libAfeGetProtectionState(void);

uint8_t libAfeGetCellNum(void);
uint8_t libAfeGetNtcNum(void);

bool libAfeIsBalancing(void);

#ifdef __cplusplus
}
#endif

#endif /* LIB_AFE_H_ */

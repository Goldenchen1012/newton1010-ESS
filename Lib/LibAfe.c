/**
  ******************************************************************************
  * @file        LibAfe.c
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/14
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "sdk_config.h"

#include "main.h"  //For balance
#include "TimeShare.h"  //For balance

#include "LibRegister.h"
#include "LibAfe.h"
#include "HalAfe.h"
#include "boards.h"
#include "LibSwTimer.h"
#include "LibCellBalance.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t interval;
  uint16_t count;
} tAfeReqAdc;

typedef struct{
  uint8_t state;
} tAfeReqBalance;

typedef struct{
  uint8_t state;
} tAfeReqProtect;

typedef struct{
  uint8_t state;
} tAfeReqGpio;


typedef struct{
  tAfeReqAdc adc;
  tAfeReqBalance balance;
  tAfeReqProtect protect;
  tAfeReqGpio gpio;
  uint8_t state;
} tAfeReq;

typedef struct{
  const tHalAfe *hal;
  tLibRegister  evtHandlerTab;
  tAfeReq request;
} tAfe;
/* Private define ------------------------------------------------------------*/
#define DO_NOT_WAIT 0xFF

/* Private macro -------------------------------------------------------------*/
static void libAfeHalHandler(__far void *dest, uint16_t evt, void *data);

#define LIB_AFE_DEF(afeId) \
  HAL_AFE_DEF(afeId##_HAL, libAfeHalHandler); \
  static tAfe afeId = { \
    &afeId##_HAL, {0}, 0 \
  };
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
LIB_AFE_DEF(mAfe);
//const static tAfe *afeTab[AFE_MAX] = {
//    &mAfe0,
//};
static uint8_t waitBalancePauseTime = DO_NOT_WAIT;

/* Private function prototypes -----------------------------------------------*/
static void libAfeHalHandler(__far void *dest, uint16_t evt, void *data){
  if(evt == HAL_AFE_EVT_DATE_UPDATED){
	waitBalancePauseTime = DO_NOT_WAIT;
	libCellBalanceStart();
    LibRegisterTypeHandlerExe(&mAfe.evtHandlerTab, LIB_AFE_EVT_DATE_UPDATED, 0);
  }else if(evt == HAL_AFE_EVT_SCD){
    LibRegisterTypeHandlerExe(&mAfe.evtHandlerTab, LIB_AFE_EVT_SCD, 0);  
  }else if(evt == HAL_AFE_EVT_RELEASED_SCD){
    LibRegisterTypeHandlerExe(&mAfe.evtHandlerTab, LIB_AFE_EVT_RELEASED_SCD, 0);  
  }
}

static void libAfeTimerHandler(__far void *dest, uint16_t evt, void *data){
	if(evt == LIB_SW_TIMER_EVT_SW_10MS_2){
		mAfe.request.adc.count += 10;
		if(mAfe.request.adc.count >= mAfe.request.adc.interval){
		    mAfe.request.adc.count -= mAfe.request.adc.interval;
			if(libCellBalanceIsBalancing() == true){
				if(waitBalancePauseTime == DO_NOT_WAIT){
				    waitBalancePauseTime = 0;
					libCellBalanceStop();
				}
			}else{
			    mAfe.hal->funs->startMeasure(mAfe.hal->data);
			}
		}	
    }else if(evt == LIB_SW_TIMER_EVT_SW_10MS_7){
		if(waitBalancePauseTime == DO_NOT_WAIT){
		    return;
        }
		if(waitBalancePauseTime < AFE_ADC_BALANCE_RELEASE_TIME){
			if(libCellBalanceIsBalancing() == true){
				waitBalancePauseTime = 0;
			}else{
				waitBalancePauseTime += 10;
				if(waitBalancePauseTime >= AFE_ADC_BALANCE_RELEASE_TIME){				
					mAfe.hal->funs->startMeasure(mAfe.hal->data);
				}
			}
		}
	}
}

static bool isCalibrationExist(const tCalibCoef *par){
	if((par->A1 == 0) && (par->A2 == 0)){
		return false;
	}else{
		return true;
	}
}

static int32_t doCalibration(const tCalibCoef *par, int32_t dataX){ //Y = A1*X + A2*X/10000 + B
	int32_t dataTemp;
	dataTemp = dataX * par->A1;
	dataTemp += (dataX * par->A2) / 10000;
	dataTemp += par->B;
	return dataTemp;
}

#if 0
#define ACCURATE_FACTOR 10  //AF
static int32_t reverseCalibration(const tCalibCoef *par, int32_t dataY){ //X = ((Y - B)*AF) / ((A1 + (A2/10000))*AF)
	int32_t dataTemp;
	dataTemp = (dataY - par->B) * ACCURATE_FACTOR;
	dataTemp /= (par->A1 * ACCURATE_FACTOR) + ((par->A2 * ACCURATE_FACTOR)/ 10000);
	return dataTemp;
}
#endif
/* Public function prototypes -----------------------------------------------*/

tErrCode libAfeInit(void){
  mAfe.hal->funs->open(mAfe.hal->data);
  return RES_SUCCESS;
}

void libAfeOpen(tAfeEvtHandler evtHandler){
  if(LibRegisterIsMemberNull(&mAfe.evtHandlerTab) == true){
    libAfeInit();
  }
  LibRegisterAdd(&mAfe.evtHandlerTab, evtHandler, 0);
}

void libAfeClose(tAfeEvtHandler evtHandler){
  LibRegisterRm(&mAfe.evtHandlerTab, evtHandler, 0);
  if(LibRegisterIsMemberNull(&mAfe.evtHandlerTab) == true){
    mAfe.hal->funs->close(mAfe.hal->data);
  }
}

void libAfeShutdown(void){
    LibRegisterTypeHandlerExe(&mAfe.evtHandlerTab, LIB_AFE_EVT_SHUTDOWN, 0);
	mAfe.hal->funs->shutdown(mAfe.hal->data);
}

void libAfeProtectionRecover(void){
	
}


void libAfeAdcSetMeasuringInterval(uint16_t ms){
  if(mAfe.request.adc.interval == ms){
    return;
  }
  mAfe.request.adc.interval = ms;
  mAfe.request.adc.count = 0;
  if(ms > 0){
    LibSwTimerOpen(libAfeTimerHandler, 0);
  }else{
    LibSwTimerClose(libAfeTimerHandler, 0);
  }
}

//---Get Function-------------------------------------------------------------------

t100uV libAfeGetCellVoltage(uint8_t num){
    if(num >= mAfe.hal->par->numMaxCell){
        return 0;
    }
  
  	if(isCalibrationExist(&mAfe.hal->data->calib->cell[num]) == true){
		return doCalibration(&mAfe.hal->data->calib->cell[num], (int32_t)mAfe.hal->data->cellVoltage[num]);		
	}else{
        return mAfe.hal->funs->cellLsbTo100uV(mAfe.hal->data->cellVoltage[num]);
	}
}

tAdcData libAfeGetAdcCellVoltage(uint8_t num){
  if(num >= mAfe.hal->par->numMaxCell){
    return 0;
  }
  return mAfe.hal->data->cellVoltage[num];
}

tmV libAfeGetBatteryVoltage(void){
//	int32_t lsbTemp;	
	if(isCalibrationExist(mAfe.hal->data->calib->vb) == true){
// 		lsbTemp = doCalibration(mAfe.hal->data->machining->vb.Calib, (int32_t)mAfe.hal->data->BatteryVoltage);
// 	    return mAfe.hal->funs->batteryLsbTomV(lsbTemp);
		return doCalibration(mAfe.hal->data->calib->vb, (int32_t)mAfe.hal->data->BatteryVoltage);
	}else{
        return mAfe.hal->funs->batteryLsbTomV(mAfe.hal->data->BatteryVoltage);
	}
}
tAdcData libAfeGetAdcBatteryVoltage(void){
  return mAfe.hal->data->BatteryVoltage;
}

tmV libAfeGetPackVoltage(void){
//	int32_t lsbTemp;
	
	if(isCalibrationExist(mAfe.hal->data->calib->vp) == true){
// 		lsbTemp = doCalibration(mAfe.hal->data->machining->vp.Calib, (int32_t)mAfe.hal->data->packVoltage);
// 		return mAfe.hal->funs->packLsbTomV(lsbTemp);
		return doCalibration(mAfe.hal->data->calib->vp, (int32_t)mAfe.hal->data->packVoltage);
	}else{
        return mAfe.hal->funs->packLsbTomV(mAfe.hal->data->packVoltage);
	}
}
tAdcData libAfeGetAdcPackVoltage(void){
  return mAfe.hal->data->packVoltage;
}

tuA libAfeGetPackCurrent(void){
//	int32_t lsbTemp;
	if(isCalibrationExist(mAfe.hal->data->calib->curr) == true){
// 		lsbTemp = doCalibration(mAfe.hal->data->machining->curr.Calib, (int32_t)mAfe.hal->data->packCurrent);
//         return mAfe.hal->funs->currentLsbTouA(lsbTemp, mAfe.hal->par->currentShunt);
		return doCalibration(mAfe.hal->data->calib->curr, (int32_t)mAfe.hal->data->packCurrent);
	}else{
#if (AFE_ADC_CURR_DIRECTION_REVERSED == 1)
        return (-1 * mAfe.hal->funs->currentLsbTouA(mAfe.hal->data->packCurrent, mAfe.hal->par->currentShunt));
#else
        return mAfe.hal->funs->currentLsbTouA(mAfe.hal->data->packCurrent, mAfe.hal->par->currentShunt);
#endif
	}
}

tAdcDataS libAfeGetAdcPackCurrent(void){
  return mAfe.hal->data->packCurrent;
}

t100uV libAfeGetNtcVoltage(uint8_t num){
    if(num >= mAfe.hal->par->numMaxNtc){
        return 0;
    }
    return mAfe.hal->funs->ntcLsbTo100uV(mAfe.hal->data->ntcR[num]);
}
tOm libAfeGetNtcUpR(uint8_t num){
    if(num >= mAfe.hal->par->numMaxNtc){
        return 0;
    }
	return mAfe.hal->funs->getNtcUpR(mAfe.hal->data, num);
}
t100uV libAfeGetNtcVdd(void){
	return mAfe.hal->funs->getNtcVdd(mAfe.hal->data);
}

uint8_t libAfeGetCellNum(void){
	return mAfe.hal->par->numMaxCell;
}
uint8_t libAfeGetNtcNum(void){
	return mAfe.hal->par->numMaxNtc;
}
//---Set Function-------------------------------------------------------------------
tLibAfeState libAfeGetState(void){
  if(mAfe.hal->data->stateSys == HAL_AFE_STATE_SHUTDOWN){
    return LIB_AFE_STATE_UNINIT;
  }else if(mAfe.hal->data->stateSys == HAL_AFE_STATE_PROTECTION){
    return LIB_AFE_STATE_PROTECTION;
  }else{
    return LIB_AFE_STATE_WORKING;
  }
}

tLibAfeMosState libAfeGetMosState(void){
  return (tLibAfeMosState)mAfe.hal->data->stateIo.bits.mos;
}

tCalibCoef libAfeGetCalibrationParCell(uint8_t num){
	return mAfe.hal->data->calib->cell[num];
}

tCalibCoef libAfeGetCalibrationParVb(void){
	return *mAfe.hal->data->calib->vb;
}
tCalibCoef libAfeGetCalibrationParVp(void){
	return *mAfe.hal->data->calib->vp;
}
tCalibCoef libAfeGetCalibrationParCurrent(void){
	return *mAfe.hal->data->calib->curr;
}

tErrCode libAfeSetMos(tLibAfeMosState state){
  return mAfe.hal->funs->setMos(mAfe.hal->data, (tHalAfeMosState)state);
}

tErrCode libAfeSetGpo(uint8_t id, bool value){
  if(id >= mAfe.hal->par->numMaxGpio){
    return RES_ERROR_INVALID_PARAM;
  }
  return mAfe.hal->funs->setGpo(mAfe.hal->data, id, value);
}

tuA libAfeAdcSetPackCurrentLsbToI(tAdcDataS lsb){
	return mAfe.hal->funs->currentLsbTouA(lsb, mAfe.hal->par->currentShunt);
}
tAdcDataS libAfeAdcSetPackCurrentI2Lsb(tuA current){
	return mAfe.hal->funs->currentI2Lsb(current, mAfe.hal->par->currentShunt);
}

tErrCode libAfeSetBalance(tCellState cellsSel){
	return mAfe.hal->funs->setBalance(mAfe.hal->data, cellsSel);
}

tErrCode libAfeSetCalibrationParCell(uint8_t num, tCalibCoef coef){
    if(num >= mAfe.hal->par->numMaxCell){
        return RES_ERROR_INVALID_PARAM;
    }
	mAfe.hal->data->calib->cell[num] = coef;
	return RES_SUCCESS;
}

tErrCode libAfeSetCalibrationParVb(tCalibCoef coef){
	*mAfe.hal->data->calib->vb = coef;
	return RES_SUCCESS;
}
tErrCode libAfeSetCalibrationParVp(tCalibCoef coef){
	*mAfe.hal->data->calib->vp = coef;
	return RES_SUCCESS;
}
tErrCode libAfeSetCalibrationParCurrent(tCalibCoef coef){
	*mAfe.hal->data->calib->curr = coef;
	return RES_SUCCESS;
}


bool libAfeIsBalancing(void){
    return mAfe.hal->data->updateFlag.bits.balance;	
}
/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    

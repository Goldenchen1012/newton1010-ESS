/**
  ******************************************************************************
  * @file        HalAfe.c
  * @author      Norman
  * @version     v1.0
  * @date        2019/9/4
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Norman</center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "boards.h"
#include "sdk_config.h"

//#include "TimeShare.h"  //For balance

#include "LibRegister.h"
#include "HalAfe.h"
#include "LibSwTimer.h"
#include "LibCellBalance.h"

#include <stdint.h>
/* Private typedef -----------------------------------------------------------*/
/*
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
//  tAfeReqBalance balance;
//  tAfeReqProtect protect;
//  tAfeReqGpio gpio;
//  uint8_t state;
} tAfeReq;

typedef struct{
  const tHalAfe *hal;
  tLibRegister  evtHandlerTab;
  tAfeReq request;
} tAfe;

*/
/* Private define ------------------------------------------------------------*/
#define DO_NOT_WAIT 0xFF

/* Private macro -------------------------------------------------------------*/
static void halAfeDrvHandler(__far void *dest, uint16_t evt, __far void *data);

/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
BSP_AFE_DEF(mAfe, halAfeDrvHandler);

static uint8_t waitBalancePauseTime = DO_NOT_WAIT;

/* Private function prototypes -----------------------------------------------*/
static void halAfeDrvHandler(__far void *dest, uint16_t evt, __far void *data){
  if(evt == HAL_AFE_EVT_DATE_UPDATED){
	waitBalancePauseTime = DO_NOT_WAIT;
	libCellBalanceStart();
    LibRegisterTypeHandlerExe(&mAfe.dat->evtHandlerTab, LIB_AFE_EVT_DATE_UPDATED, 0);
  }else if(evt == HAL_AFE_EVT_SCD){
    LibRegisterTypeHandlerExe(&mAfe.dat->evtHandlerTab, LIB_AFE_EVT_SCD, 0);  
  }else if(evt == HAL_AFE_EVT_RELEASED_SCD){
    LibRegisterTypeHandlerExe(&mAfe.dat->evtHandlerTab, LIB_AFE_EVT_RELEASED_SCD, 0);  
  }
}

static void halAfeTimerHandler(__far void *dest, uint16_t evt, __far void *data){
	if(evt == LIB_SW_TIMER_EVT_SW_10MS){
		mAfe.dat->reqAdc.count += 10;
		if(mAfe.dat->reqAdc.count >= mAfe.dat->reqAdc.interval){
		    mAfe.dat->reqAdc.count -= mAfe.dat->reqAdc.interval;
			if(libCellBalanceIsBalancing() == true){
				if(waitBalancePauseTime == DO_NOT_WAIT){
				    waitBalancePauseTime = 0;
					libCellBalanceStop();
				}
			}else{
			    mAfe.funs->startMeasure(mAfe.dat);
			}
		}	
    }else if(evt == LIB_SW_TIMER_EVT_SW_10MS){
		if(waitBalancePauseTime == DO_NOT_WAIT){
		    return;
        }
		if(waitBalancePauseTime < AFE_ADC_BALANCE_RELEASE_TIME){
			if(libCellBalanceIsBalancing() == true){
				waitBalancePauseTime = 0;
			}else{
				waitBalancePauseTime += 10;
				if(waitBalancePauseTime >= AFE_ADC_BALANCE_RELEASE_TIME){				
					mAfe.funs->startMeasure(mAfe.dat);
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

tErrCode halAfeInit(void){
  mAfe.funs->open(mAfe.dat);
  return RES_SUCCESS;
}

void halAfeOpen(tAfeEvtHandler evtHandler){
  if(LibRegisterIsMemberNull(&mAfe.dat->evtHandlerTab) == true){
    halAfeInit();
  }
  LibRegisterAdd(&mAfe.dat->evtHandlerTab, evtHandler, 0);
}

void halAfeClose(tAfeEvtHandler evtHandler){
  LibRegisterRm(&mAfe.dat->evtHandlerTab, evtHandler, 0);
  if(LibRegisterIsMemberNull(&mAfe.dat->evtHandlerTab) == true){
    mAfe.funs->close(mAfe.dat);
  }
}

void halAfeShutdown(void){
    LibRegisterTypeHandlerExe(&mAfe.dat->evtHandlerTab, LIB_AFE_EVT_SHUTDOWN, 0);
	mAfe.funs->shutdown(mAfe.dat);
}

void halAfeProtectionRecover(void){
	
}


void halAfeAdcSetMeasuringInterval(uint16_t ms){
  if(mAfe.dat->reqAdc.interval == ms){
    return;
  }
  mAfe.dat->reqAdc.interval = ms;
  mAfe.dat->reqAdc.count = 0;
  if(ms > 0){
    LibSwTimerOpen(LIB_SW_TIMER_PRIORITY_L, halAfeTimerHandler, 0);
  }else{
    LibSwTimerClose(LIB_SW_TIMER_PRIORITY_L, halAfeTimerHandler, 0);
  }
}

//---Get Function-------------------------------------------------------------------

t100uV halAfeGetCellVoltage(uint8_t num){
    if(num >= mAfe.par->numMaxCell){
        return 0;
    }
  
  	if(isCalibrationExist(&mAfe.dat->calib->cell[num]) == true){
		return doCalibration(&mAfe.dat->calib->cell[num], (int32_t)mAfe.dat->cellVoltage[num]);		
	}else{
        return mAfe.funs->cellLsbTo100uV(mAfe.dat->cellVoltage[num]);
	}
}

tAdcData halAfeGetAdcCellVoltage(uint8_t num){
  if(num >= mAfe.par->numMaxCell){
    return 0;
  }
  return mAfe.dat->cellVoltage[num];
}

tmV halAfeGetBatteryVoltage(void){
//	int32_t lsbTemp;	
	if(isCalibrationExist(mAfe.dat->calib->vb) == true){
// 		lsbTemp = doCalibration(mAfe.dat->machining->vb.Calib, (int32_t)mAfe.dat->BatteryVoltage);
// 	    return mAfe.funs->batteryLsbTomV(lsbTemp);
		return doCalibration(mAfe.dat->calib->vb, (int32_t)mAfe.dat->BatteryVoltage);
	}else{
        return mAfe.funs->batteryLsbTomV(mAfe.dat->BatteryVoltage);
	}
}
tAdcData halAfeGetAdcBatteryVoltage(void){
  return mAfe.dat->BatteryVoltage;
}

tmV halAfeGetPackVoltage(void){
//	int32_t lsbTemp;
	
	if(isCalibrationExist(mAfe.dat->calib->vp) == true){
// 		lsbTemp = doCalibration(mAfe.dat->machining->vp.Calib, (int32_t)mAfe.dat->packVoltage);
// 		return mAfe.funs->packLsbTomV(lsbTemp);
		return doCalibration(mAfe.dat->calib->vp, (int32_t)mAfe.dat->packVoltage);
	}else{
        return mAfe.funs->packLsbTomV(mAfe.dat->packVoltage);
	}
}
tAdcData halAfeGetAdcPackVoltage(void){
  return mAfe.dat->packVoltage;
}

tuA halAfeGetPackCurrent(void){
//	int32_t lsbTemp;
	if(isCalibrationExist(mAfe.dat->calib->curr) == true){
// 		lsbTemp = doCalibration(mAfe.dat->machining->curr.Calib, (int32_t)mAfe.dat->packCurrent);
//         return mAfe.funs->currentLsbTouA(lsbTemp, mAfe.par->currentShunt);
		return doCalibration(mAfe.dat->calib->curr, (int32_t)mAfe.dat->packCurrent);
	}else{
#if (AFE_ADC_CURR_DIRECTION_REVERSED == 1)     
        return (-1 * mAfe.funs->currentLsbTouA(mAfe.dat->packCurrent, mAfe.par->currentShunt));        
#else
        return mAfe.funs->currentLsbTouA(mAfe.dat->packCurrent, mAfe.par->currentShunt);        
#endif
	}
}

tAdcDataS halAfeGetAdcPackCurrent(void){
  return mAfe.dat->packCurrent;
}

t100uV halAfeGetNtcVoltage(uint8_t num){
    if(num >= mAfe.par->numMaxNtc){
        return 0;
    }
    return mAfe.funs->ntcLsbTo100uV(mAfe.dat->ntcR[num]);
}
tOm halAfeGetNtcUpR(uint8_t num){
    if(num >= mAfe.par->numMaxNtc){
        return 0;
    }
	return mAfe.funs->getNtcUpR(mAfe.dat, num);
}
t100uV halAfeGetNtcVdd(void){
	return mAfe.funs->getNtcVdd(mAfe.dat);
}

uint8_t halAfeGetCellNum(void){
	return mAfe.par->numMaxCell;
}
uint8_t halAfeGetNtcNum(void){
	return mAfe.par->numMaxNtc;
}
//---Set Function-------------------------------------------------------------------
thalAfeState halAfeGetState(void){
  if(mAfe.dat->stateSys == HAL_AFE_STATE_SHUTDOWN){
    return LIB_AFE_STATE_UNINIT;
  }else if(mAfe.dat->stateSys == HAL_AFE_STATE_PROTECTION){
    return LIB_AFE_STATE_PROTECTION;
  }else{
    return LIB_AFE_STATE_WORKING;
  }
}

tHalAfeMosState halAfeGetMosState(void){
  return mAfe.dat->stateIo.bits.mos;
}

tCalibCoef halAfeGetCalibrationParCell(uint8_t num){
	return mAfe.dat->calib->cell[num];
}

tCalibCoef halAfeGetCalibrationParVb(void){
	return *mAfe.dat->calib->vb;
}
tCalibCoef halAfeGetCalibrationParVp(void){
	return *mAfe.dat->calib->vp;
}
tCalibCoef halAfeGetCalibrationParCurrent(void){
	return *mAfe.dat->calib->curr;
}

tErrCode halAfeSetMos(tHalAfeMosState state){
  return mAfe.funs->setMos(mAfe.dat, state);
}


tErrCode halAfeSetGpo(uint8_t id, bool value){
  if(id >= mAfe.par->numMaxGpio){
    return RES_ERROR_INVALID_PARAM;
  }
  return mAfe.funs->setGpo(mAfe.dat, id, value);
}

tuA halAfeAdcSetPackCurrentLsbToI(tAdcDataS lsb){
	return mAfe.funs->currentLsbTouA(lsb, mAfe.par->currentShunt);
}
tAdcDataS halAfeAdcSetPackCurrentI2Lsb(tuA current){
	return mAfe.funs->currentI2Lsb(current, mAfe.par->currentShunt);
}

tErrCode halAfeSetBalance(tCellState cellsSel){
	return mAfe.funs->setBalance(mAfe.dat, cellsSel);
}

tErrCode halAfeSetCalibrationParCell(uint8_t num, tCalibCoef coef){
    if(num >= mAfe.par->numMaxCell){
        return RES_ERROR_INVALID_PARAM;
    }
	mAfe.dat->calib->cell[num] = coef;
	return RES_SUCCESS;
}

tErrCode halAfeSetCalibrationParVb(tCalibCoef coef){
	*mAfe.dat->calib->vb = coef;
	return RES_SUCCESS;
}
tErrCode halAfeSetCalibrationParVp(tCalibCoef coef){
	*mAfe.dat->calib->vp = coef;
	return RES_SUCCESS;
}
tErrCode halAfeSetCalibrationParCurrent(tCalibCoef coef){
	*mAfe.dat->calib->curr = coef;
	return RES_SUCCESS;
}


bool halAfeIsBalancing(void){
    return mAfe.dat->updateFlag.bits.balance;	
}

/************************ (C) COPYRIGHT Norman Tang *****END OF FILE****/    

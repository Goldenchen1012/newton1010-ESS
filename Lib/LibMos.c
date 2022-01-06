/**
  ******************************************************************************
  * @file        LibMos.c
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
#include <stdbool.h>
#include "dataType.h"
#include "sdk_config.h"
#include "boards.h"
#include "LibAfe.h"
#include "LibSwTimer.h"
#include "LibMos.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum{
  MOS_CTL_STATE_UNINIT = 0,
  MOS_CTL_STATE_OFF,
  MOS_CTL_STATE_ON,
  MOS_CTL_STATE_PRE_DSG,
  MOS_CTL_STATE_PRE_DSG_OFF,
  MOS_CTL_STATE_PRE_DSG_RETRY_ON,
  MOS_CTL_STATE_PRE_DSG_RETRY_OFF,
  MOS_CTL_STATE_PRE_DSG_RETRY_FAIL,
} tLibMosControlState;

typedef struct LibMos{
  tLibRegister  evtHandlerTab;
  tLibMosState state;
  tLibMosControlState ctlState;
  uint16_t preDsgTime;
  uint16_t preDsgTimeCount;
  uint8_t preDsgRetryCount;
} tLibMos;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define turnOnPreDsg() libAfeSetGpo(AFE_PRE_DSG_PIN_ID, 1)
#define turnOffPreDsg() libAfeSetGpo(AFE_PRE_DSG_PIN_ID, 0)

/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
tLibMos mMos = {.state = LIB_MOS_STATE_UNINIT, 
              .ctlState = MOS_CTL_STATE_UNINIT, 
              .preDsgTime = LIB_MOS_PRE_DSG_TIME_DEF,
              .preDsgTimeCount = 0,
              .preDsgRetryCount = 0};
/* Private function prototypes -----------------------------------------------*/
static void LibMosTimerHandler(uint16_t evt, void *data);

static tErrCode setMos(tLibMosState state){
    tErrCode errCode;
    if(state >= LIB_MOS_STATE_UNINIT){
      return RES_ERROR_INVALID_PARAM;
    }
    errCode = libAfeSetMos((tLibAfeMosState)state);
    if(state == LIB_MOS_STATE_OFF){
      mMos.ctlState = MOS_CTL_STATE_OFF;
      RegisterTypeHandlerExe(&mMos.evtHandlerTab, LIB_MOS_EVT_OFF, 0);
    }else{
      mMos.ctlState = MOS_CTL_STATE_ON;
      RegisterTypeHandlerExe(&mMos.evtHandlerTab, mMos.state, 0);
    }
    LibSwTimerClose(LibMosTimerHandler);
    return errCode;
}

static void LibMosTimerHandler(uint16_t evt, void *data){
  if(evt == LIB_SW_TIMER_EVT_SW_10MS_3){
    if((mMos.ctlState == MOS_CTL_STATE_PRE_DSG) 
        || (mMos.ctlState == MOS_CTL_STATE_PRE_DSG_RETRY_ON)){
      mMos.preDsgTimeCount += 10;
      if(mMos.preDsgTimeCount >= mMos.preDsgTime){
        mMos.preDsgTimeCount = 0;  
        if(libAfeAdcGetPackVoltage() >= LIB_MOS_PRE_DSG_VP_THRESHOLD){
          setMos(mMos.state);
          turnOffPreDsg();
        }else{
          mMos.ctlState = MOS_CTL_STATE_PRE_DSG_RETRY_OFF;  
          turnOffPreDsg();
          mMos.preDsgRetryCount++;
          if(mMos.preDsgRetryCount > LIB_MOS_PRE_DSG_RETRY_COUNT){
            mMos.state = LIB_MOS_STATE_RETEY_FAIL;
            mMos.ctlState = MOS_CTL_STATE_PRE_DSG_RETRY_FAIL;
            RegisterTypeHandlerExe(&mMos.evtHandlerTab, LIB_MOS_STATE_RETEY_FAIL, 0);
            LibSwTimerClose(LibMosTimerHandler);
          }
        }
      }
    }else if(mMos.ctlState == MOS_CTL_STATE_PRE_DSG_RETRY_OFF){
      mMos.preDsgTimeCount += 10;
      if(mMos.preDsgTimeCount >= LIB_MOS_PRE_DSG_RETRY_TIME){  
        mMos.preDsgTimeCount = 0;
        mMos.ctlState = MOS_CTL_STATE_PRE_DSG_RETRY_ON;  
        turnOnPreDsg();
      }
    }
  }
}

/* Public function prototypes -----------------------------------------------*/
tErrCode LibMosOpen(tLibMosEvtHandler evtHandler){
  return RegisterAdd(&mMos.evtHandlerTab, evtHandler);
}
tErrCode LibMosClose(tLibMosEvtHandler evtHandler){
  return RegisterRm(&mMos.evtHandlerTab, evtHandler);
}

void LibMosSetPreDsgMosTime(uint16_t timems){
  mMos.preDsgTime = timems;
}

tErrCode LibMosSetState(tLibMosState state){
  if(libAfeGetState() == LIB_AFE_STATE_PROTECTION){
    mMos.state = LIB_MOS_STATE_OFF;
    return RES_ERROR_FORBIDDEN; 
  } 
  if(state >= LIB_MOS_STATE_FAIL){
    return RES_ERROR_INVALID_PARAM; 
  }
  if(mMos.state == LIB_MOS_STATE_UNINIT){
    return RES_ERROR_UNINIT;
  }
  if(mMos.state == state){
    return RES_SUCCESS;
  }
 
  mMos.state = state;
  if((mMos.ctlState == MOS_CTL_STATE_UNINIT)
      || (mMos.ctlState == MOS_CTL_STATE_OFF)){
    if((state == LIB_MOS_STATE_ON) 
      || (state == LIB_MOS_STATE_DIS)){
      if(mMos.preDsgTime > 0){
        mMos.ctlState = MOS_CTL_STATE_PRE_DSG;
        mMos.preDsgTimeCount = 0;
        mMos.preDsgRetryCount = 0;
        LibSwTimerOpen(LibMosTimerHandler);
        turnOnPreDsg();
        return RES_SUCCESS;
      }
    }
  }
  return setMos(state);

}

tLibMosState LibMosGetState(void); 

tErrCode LibMosInit(tLibMosState initState){
  if(libAfeGetState() == LIB_AFE_STATE_UNINIT){
    libAfeInit();
  }

  if(mMos.state == LIB_MOS_STATE_UNINIT){
    mMos.state = (tLibMosState)libAfeGetMosState();
  }

  return LibMosSetState(initState);
}
/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    

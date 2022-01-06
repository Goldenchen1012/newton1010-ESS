/********************************************************************************
  * @file        LibMos.h
  * @author      Norman
  * @version     v1.0
  * @date        2019/9/6
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Norman</center></h2>
  *
  *
  ******************************************************************************
  */
#ifndef LIB_MOS_H_
#define LIB_MOS_H_

#include <stdint.h>
#include "datatype.h"
#include "LibRegister.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
#define tLibMosEvtHandler RegisterEvtHandler_t

/* Public typedef -----------------------------------------------------------*/

typedef enum{
  LIB_MOS_STATE_OFF = 0,
  LIB_MOS_STATE_DIS,
  LIB_MOS_STATE_CHG,
  LIB_MOS_STATE_ON,
  LIB_MOS_STATE_UNINIT,
  LIB_MOS_STATE_RETEY_FAIL,
  LIB_MOS_STATE_FAIL
} tLibMosState;

typedef enum{
  LIB_MOS_EVT_OFF = 0,
  LIB_MOS_EVT_DIS,
  LIB_MOS_EVT_CHG,
  LIB_MOS_EVT_ON,
  LIB_MOS_EVT_RETEY_FAIL,
  LIB_MOS_EVT_FAIL
} tLibMosEvt;

/* Public variables ---------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
tErrCode LibMosInit(tLibMosState initState); 
tErrCode LibMosOpen(tLibMosEvtHandler evtHandler); 
tErrCode LibMosClose(tLibMosEvtHandler evtHandler); 

void LibMosSetPreDsgMosTime(uint16_t timems);
tErrCode LibMosSetState(tLibMosState state); 
tLibMosState LibMosGetState(void); 

#ifdef __cplusplus
}
#endif

#endif 

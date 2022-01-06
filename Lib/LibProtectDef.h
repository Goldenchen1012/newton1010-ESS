/**
  ******************************************************************************
  * @file        LibProtectDef.h
  * @author      Norman
  * @version     v1.0
  * @date        2019/9/10
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Norman</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef LIB_PROTECT_DEF_H_
#define LIB_PROTECT_DEF_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "datatype.h"
#include "LibRegister.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
#define tLibProtectEvtHandler RegisterEvtHandler_t
/* Public typedef -----------------------------------------------------------*/
enum{
  LIB_PROTECT_DEF_EVT_OV = 0,
  LIB_PROTECT_DEF_EVT_ALARM_OV,
  LIB_PROTECT_DEF_EVT_UV,
  LIB_PROTECT_DEF_EVT_OC,
  LIB_PROTECT_DEF_EVT_OT,
  LIB_PROTECT_DEF_EVT_UT,
  LIB_PROTECT_DEF_EVT_SCD,
  LIB_PROTECT_DEF_EVT_ALARM_UV,
  LIB_PROTECT_DEF_EVT_ALARM_OC,
  LIB_PROTECT_DEF_EVT_ALARM_OT,
  LIB_PROTECT_DEF_EVT_ALARM_UT,

  LIB_PROTECT_EVT_MAX
};

/* Public macro -------------------------------------------------------------*/

/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/

void LibProtectOvOpen(tLibProtectEvtHandler handler);
void LibProtectOvClose(void);

#ifdef __cplusplus
}
#endif

#endif /* LIB_AFE_H_ */

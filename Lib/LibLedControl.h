/********************************************************************************
  * @file        LibLedControl.h
  * @author      Norman
  * @version     v1.0
  * @date        2019/9/20
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Norman</center></h2>
  *
  *
  ******************************************************************************
  */
#ifndef _LIB_LED_CONTROL_H
#define _LIB_LED_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "LibDebug.h"
#include "LibRegister.h"

#include <stdint.h>
 
/* Public define ------------------------------------------------------------*/
#define tLibLedEvtHandler tLibRegisterEvtHandler

/* Public typedef -----------------------------------------------------------*/
enum{
  LIB_LED_EVT_SHOW_END = 0,
  LIB_LED_EVT_MAX
};

typedef enum{
  LED_STATE_DISABLE = 0,
  LED_STATE_OFF,
  LED_STATE_ON,
  LED_STATE_ALARM,  
  LED_STATE_PROTECT,
  LED_STATE_PF,
  LED_STATE_CAPACITY_0_10,
  LED_STATE_CAPACITY_11_20,
  LED_STATE_CAPACITY_21_94,
  LED_STATE_CAPACITY_95_100,
  LED_STATE_CHARGING,
  LED_STATE_BUTTON_PRESS,
  LED_STATE_FAIL,
  LED_STATE_POWER_OFF,
  LED_STATE_POWER_ON,
  LED_STATE_OTP_UTP,
  LED_STATE_OCP,
  LED_STATE_OVP_UVP,
  LED_STATE_MAX,
} tLibLedState;

typedef enum{
  LED_PRIORITY_H = 0,
  LED_PRIORITY_M,
  LED_PRIORITY_L,
  LED_PRIORITY_MAX
} tLibLedPriority;

typedef struct {
  uint32_t actTime;
  tLibLedState state;
} tLibLedControlStatus;

typedef struct {
  tLibLedControlStatus ledState[LED_PRIORITY_MAX];
  uint16_t delayCount;
  tLibLedPriority nowPriority;
  uint8_t subState;
} tLibLedControl;

enum{
  LED_SUB_STEP_INIT = 0,
  LED_SUB_STEP_1,
  LED_SUB_STEP_2,
  LED_SUB_STEP_3,
  LED_SUB_STEP_4,
  LED_SUB_STEP_5,
  LED_SUB_STEP_6,
  LED_SUB_STEP_7,
  LED_SUB_STEP_MAX = 10,
} ;

/* Public variables ---------------------------------------------------------*/
extern tLibLedControl mLed;
extern tLibRegister libLedEvtHandlerRegister;
/* Public macro -------------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
tErrCode LibLedControlOpen(tLibLedEvtHandler evtHandler);
tErrCode LibLedControlSetState(tLibLedPriority priority, uint32_t actTime, tLibLedState state);
void appLedShowCapacity(void);

#ifdef __cplusplus
}
#endif

#endif 

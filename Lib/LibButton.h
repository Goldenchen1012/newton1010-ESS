/********************************************************************************
  * @file        LibButton.h
  * @author      Norman
  * @version     v1.0
  * @date        2019/9/23
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Norman</center></h2>
  *
  *
  ******************************************************************************
  */
#ifndef _LIB_BUTTON_H
#define _LIB_BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif
#include "LibDebug.h"
#include "LibRegister.h"

#include <stdint.h>
 
/* Public define ------------------------------------------------------------*/
#define tLibButtonEvtHandler tLibRegisterEvtHandler

/* Public typedef -----------------------------------------------------------*/

typedef enum{
  BUTTON_EVT_CLICK_KEY_0 = 0,
  BUTTON_EVT_CLICK_KEY_1,
  BUTTON_EVT_CLICK_KEY_2,
  BUTTON_EVT_LONG_PRESS_KEY_0 = 10,
  BUTTON_EVT_LONG_PRESS_KEY_1,
  BUTTON_EVT_LONG_PRESS_KEY_2,
  BUTTON_EVT_PRESS_1S_KEY_0 = 20,
  BUTTON_EVT_PRESS_1S_KEY_1,
  BUTTON_EVT_PRESS_1S_KEY_2,
  BUTTON_EVT_PRESS_KEY_0 = 30,
  BUTTON_EVT_PRESS_KEY_1,
  BUTTON_EVT_PRESS_KEY_2,
  BUTTON_EVT_RELEASE_KEY_0 = 40,
  BUTTON_EVT_RELEASE_KEY_1,
  BUTTON_EVT_RELEASE_KEY_2,

  BUTTON_EVT_MAX,
} tLibButtonEvt;

/* Public variables ---------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/

tErrCode LibButtonOpen(tLibButtonEvtHandler evtHandler);
bool LibButtonIsPress(void);
void LibSetPowerOnOffsetKeyCount(void);
#ifdef __cplusplus
}
#endif

#endif 

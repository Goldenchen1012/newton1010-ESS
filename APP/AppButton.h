/**
  ******************************************************************************
  * @file        AppButton.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/12/14
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _APP_BUTTON_H_
#define _APP_BUTTON_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "LibRegister.h"

#ifdef __cplusplus
extern "C" {
#endif

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

/* Public define ------------------------------------------------------------*/
#define tAppButtonEvtHandler tLibRegisterEvtHandler

/* Public macro -------------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
void appButtonOpen(tAppButtonEvtHandler EventHandler);


#ifdef __cplusplus
}
#endif

#endif /* _APP_BUTTON_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



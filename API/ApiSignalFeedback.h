/**
  ******************************************************************************
  * @file        AppSignalFeedback.h
  * @author      Johnny
  * @version     v0.0.0
  * @date        2021/10/28
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _API_SIGNAL_FEEDBACK_H_
#define _API_SIGNAL_FEEDBACK_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/* Public define ------------------------------------------------------------*/
#define tApiSignalFeedbackEvtHandler tLibRegisterEvtHandler

/* Public typedef -----------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/
enum{
	APP_SIGNAL_ID_DI1 = 1,
	APP_SIGNAL_ID_DI2,
	APP_SIGNAL_ID_EPO,
	APP_SIGNAL_ID_SP,
	APP_SIGNAL_ID_PS1,
	APP_SIGNAL_ID_PS2,
	APP_SIGNAL_ID_PS3,
	APP_SIGNAL_ID_BUTTON,
	APP_SIGNAL_ID_K1,
	APP_SIGNAL_ID_K2,
	APP_SIGNAL_ID_K3,
	APP_SIGNAL_ID_K4,
	APP_SIGNAL_ID_DOCP,
	APP_SIGNAL_ID_COCP,
	APP_SIGNAL_ID_OD_IN,
};	

enum{
	APP_SIGNAL_FB_EVT_DI1_HI = 1,
	APP_SIGNAL_FB_EVT_DI1_LO,
	APP_SIGNAL_FB_EVT_DI2_HI,
	APP_SIGNAL_FB_EVT_DI2_LO,
	APP_SIGNAL_FB_EVT_EPO_HI,
	APP_SIGNAL_FB_EVT_EPO_LO,
	APP_SIGNAL_FB_EVT_SP_HI,
	APP_SIGNAL_FB_EVT_SP_LO,
	APP_SIGNAL_FB_EVT_PS1_HI,
	APP_SIGNAL_FB_EVT_PS1_LO,
	APP_SIGNAL_FB_EVT_PS2_HI,
	APP_SIGNAL_FB_EVT_PS2_LO,
	APP_SIGNAL_FB_EVT_PS3_HI,
	APP_SIGNAL_FB_EVT_PS3_LO,
	APP_SIGNAL_FB_EVT_BUTTON_HI,
	APP_SIGNAL_FB_EVT_BUTTON_LO,
	APP_SIGNAL_FB_EVT_K1_HI,
	APP_SIGNAL_FB_EVT_K1_LO,
	APP_SIGNAL_FB_EVT_K2_HI,
	APP_SIGNAL_FB_EVT_K2_LO,
	APP_SIGNAL_FB_EVT_K3_HI,
	APP_SIGNAL_FB_EVT_K3_LO,
	APP_SIGNAL_FB_EVT_K4_HI,
	APP_SIGNAL_FB_EVT_K4_LO,
	APP_SIGNAL_FB_EVT_DOCP_HI,
	APP_SIGNAL_FB_EVT_DOCP_LO,
	APP_SIGNAL_FB_EVT_COCP_HI,
	APP_SIGNAL_FB_EVT_COCP_LO,
	APP_SIGNAL_FB_EVT_OD_IN_HI,
	APP_SIGNAL_FB_EVT_OD_IN_LO,
};

/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
void apiSignalFeedbackOpen(tApiSignalFeedbackEvtHandler evtHandler);
uint8_t apiSignalFeedbackGetStatus(uint8_t id);



#ifdef __cplusplus
}
#endif


	

#endif /* _API_SIGNAL_FEEDBACK_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    

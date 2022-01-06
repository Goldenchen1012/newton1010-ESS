/**
  ******************************************************************************
  * @file        AppGauge.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/23
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _APP_GAUGE_H_
#define _APP_GAUGE_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "LibRegister.h"
#include "halafe.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
#define tAppGaugeEvtHandler tLibRegisterEvtHandler

#define	APP_SCU_GAUGE_RELEASE_MODE		0
#define	APP_SCU_GAUGE_DISCHARGE_MODE	1
#define	APP_SCU_GAUGE_CHARGE_MODE		2
#define	APP_SCU_GAUGE_UNKNOW_MODE		3

enum{
	APP_GAUGE_UNKNOW = 0,
	APP_GAUGE_STATE_IDLE,
	APP_GAUGE_CHARGING,
	APP_GAUGE_FULL,
	APP_GAUGE_DISCHARGE,
	APP_GAUGE_EMPTY
};

enum{
	APP_GAUGE_EVENT_START_CHG_SOC = 0,
	APP_GAUGE_EVENT_START_DHG_SOC,
	APP_GAUGE_EVENT_IDLE_OVER_5HR,
	APP_GAUGE_EVENT_CANNOT_GET_5HR_SOC,
	APP_GAUGE_EVENT_UPDATE_QMAX_1ST,
	APP_GAUGE_EVENT_UPDATE_QMAX,
	APP_GAUGE_EVENT_GET_5HR_SOC,
	APP_GAUGE_EVENT_CAL_RA1 = 40,
	APP_GAUGE_EVT_END
};

void appGaugeCleanCycleCount(void);
void appGaugeSetSoc0(uint16_t soc);
void appGaugeUpdateSoc0(void);

void appGaugeOpen(tAppGaugeEvtHandler evtHandler);
uint8_t	appGaugeGetCurrentMode(void);
tCurrent appGaugeGetCurrentValue(void);

uint32_t appGaugeGetQmax(void);
void appGaugeSetQmax(uint32_t qmax);

uint32_t appGaugeGetQStart(void);
uint32_t appGaugeGetRM(void);
void appGaugeSetRM(uint32_t rm);

int32_t appGaugeGetQPassCharge(void);
int32_t appGaugeGetRPassCharge(void);
uint32_t appGaugeGetFCC(void);
uint16_t appGaugeGetSOH(void);
uint16_t appGaugeGetRSoc(void);
uint16_t appGaugeGetSoc0(void);

uint16_t appGaugeGetRSoc(void);
uint16_t appGaugeGetRamSoc(void);
uint16_t appGaugeGetEndOfSoc(void);
uint16_t appGaugeGetDisplaySoc(void);
uint16_t appGaugeGetCyleCount(void);


/* Public macro -------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif


	

#endif /* _APP_GAUGE_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



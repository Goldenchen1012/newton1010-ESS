/**
  ******************************************************************************
  * @file        AppProtect.h
  * @author      Johnny
  * @version     v0.0
  * @date        2021/9/8
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */
#ifndef _APP_PROTECT_H_
#define _APP_PROTECT_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "LibRegister.h"
#include "AppProtectEvent.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
#define tAppProtectEvtHandler tLibRegisterEvtHandler

#define	PROTECT_LEVEL	3

#define	PROTECT_FLAG_L1_MASK		(0x03 << 0)
#define	PROTECT_FLAG_L1_NORMAL		(0x00 << 0)
#define	PROTECT_FLAG_L1_SETTING		(0x01 << 0)
#define	PROTECT_FLAG_L1_SETTED		(0x02 << 0)
#define	PROTECT_FLAG_L1_REALSING	(0x03 << 0)

#define	PROTECT_FLAG_L2_MASK		(0x03 << 2)
#define	PROTECT_FLAG_L2_NORMAL		(0x00 << 2)
#define	PROTECT_FLAG_L2_SETTING		(0x01 << 2)
#define	PROTECT_FLAG_L2_SETTED		(0x02 << 2)
#define	PROTECT_FLAG_L2_REALSING	(0x03 << 2)

#define	PROTECT_FLAG_L3_MASK		(0x03 << 4)
#define	PROTECT_FLAG_L3_NORMAL		(0x00 << 4)
#define	PROTECT_FLAG_L3_SETTING		(0x01 << 4)
#define	PROTECT_FLAG_L3_SETTED		(0x02 << 4)
#define	PROTECT_FLAG_L3_REALSING	(0x03 << 4)

#define	PROTECT_FLAG_L4_MASK		(0x03 << 6)
#define	PROTECT_FLAG_L4_NORMAL		(0x00 << 6)
#define	PROTECT_FLAG_L4_SETTING		(0x01 << 6)
#define	PROTECT_FLAG_L4_SETTED		(0x02 << 6)
#define	PROTECT_FLAG_L4_REALSING	(0x03 << 6)

/* Public typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t		Mask;
	uint8_t		ClearMask;
	uint8_t		Setting;
	uint8_t		Setted;
	uint8_t		Releasing;
}tProtectFlagValue;

/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
void appProtectGetLevelMask(uint8_t Level, tProtectFlagValue *pProtectFlagValue);
uint8_t	appProtectIsUnderTemperter(tNtcVoltage NtcVoltage, tNtcVoltage CompareVoltage);
uint8_t	appProtectIsOverTemperter(tNtcVoltage NtcVoltage, tNtcVoltage CompareVoltage);
void appProtectOpen(tAppProtectEvtHandler evtHandler);
void appProtectHandler(uint16_t evt);

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif


#endif /* _APP_PROTECT_H_ */


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    






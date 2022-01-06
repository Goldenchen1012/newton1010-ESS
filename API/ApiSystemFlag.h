/**
  ******************************************************************************
  * @file        SystemFlag.h
  * @author      Johnny
  * @version     v0.0.0
  * @date        2021/11/02
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _SYSTEM_FLAG_H_
#define _SYSTEM_FLAG_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
#define	SYSTEM_FLAG1_OVP_L1		(1 << 0)
#define	SYSTEM_FLAG1_OVP_L2		(1 << 1)
#define	SYSTEM_FLAG1_OVP_L3		(1 << 2)
#define	SYSTEM_FLAG1_OVP_2nd	(1 << 3)
#define	SYSTEM_FLAG1_UVP_L1		(1 << 4)
#define	SYSTEM_FLAG1_UVP_L2		(1 << 5)
#define	SYSTEM_FLAG1_UVP_L3		(1 << 6)
#define	SYSTEM_FLAG1_UVP_2nd	(1 << 7)
#define	SYSTEM_FLAG1_COTP_L1	(1 << 8)
#define	SYSTEM_FLAG1_COTP_L2	(1 << 9)
#define	SYSTEM_FLAG1_COTP_L3	(1 << 10)
#define	SYSTEM_FLAG1_CUTP_L1	(1 << 11)
#define SYSTEM_FLAG1_CUTP_L2	(1 << 12)
#define SYSTEM_FLAG1_CUTP_L3	(1 << 13)
#define SYSTEM_FLAG1_DOTP_L1	(1 << 14)
#define SYSTEM_FLAG1_DOTP_L2	(1 << 15)
#define SYSTEM_FLAG1_DOTP_L3	(1 << 16)
#define SYSTEM_FLAG1_DUTP_L1	(1 << 17)
#define SYSTEM_FLAG1_DUTP_L2	(1 << 18)
#define SYSTEM_FLAG1_DUTP_L3	(1 << 19)
#define SYSTEM_FLAG1_UT_2nd		(1 << 20)
#define SYSTEM_FLAG1_OT_2nd		(1 << 21)
#define SYSTEM_FLAG1_COCP_L1	(1 << 22)
#define SYSTEM_FLAG1_COCP_L2	(1 << 23)
#define SYSTEM_FLAG1_COCP_L3	(1 << 24)
#define SYSTEM_FLAG1_COCP_LATCH	(1 << 25)
#define SYSTEM_FLAG1_DOCP_L1	(1 << 26)
#define SYSTEM_FLAG1_DOCP_L2	(1 << 27)
#define SYSTEM_FLAG1_DOCP_L3	(1 << 28)
#define SYSTEM_FLAG1_DOCP_LATCH	(1 << 29)
#define	SYSTEM_FLAG1_CANID_READY	(1U << 30)
#define SYSTEM_FLAG1_SYSTEM_READY	(1U << 31)

#define SYSTEM_FLAG2_AFE_L1			(1 << 0)
#define SYSTEM_FLAG2_AFE_L2			(1 << 1)
#define SYSTEM_FLAG2_EPO_ENABLE		(1 << 2)
#define SYSTEM_FLAG2_OVP_PF			(1 << 3)
#define SYSTEM_FLAG2_UVP_PF			(1 << 4)
#define SYSTEM_FLAG2_P_RELAY_FAIL	(1 << 5)
#define SYSTEM_FLAG2_M_RELAY_FAIL	(1 << 6)
#define SYSTEM_FLAG2_MASTER			(1 << 7)
#define SYSTEM_FLAG2_RELAY_ON		(1 << 8)
#define SYSTEM_FLAG2_RTC_VALID		(1 << 9)
#define SYSTEM_FLAG2_PS1			(1 << 10)
#define SYSTEM_FLAG2_PS2			(1 << 11)
#define SYSTEM_FLAG2_PS3			(1 << 12)
#define SYSTEM_FLAG2_K1				(1 << 13)
#define SYSTEM_FLAG2_K2				(1 << 14)
#define SYSTEM_FLAG2_K3				(1 << 15)
#define SYSTEM_FLAG2_K4				(1 << 16)
#define SYSTEM_FLAG2_SP_FB			(1 << 17)
#define SYSTEM_FLAG2_DI1			(1 << 18)
#define SYSTEM_FLAG2_DI2			(1 << 19)
#define SYSTEM_FLAG2_OD_IN			(1 << 20)
#define SYSTEM_FLAG2_NFAULT			(1 << 21)
#define	SYSTEM_FLAG2_ENG_MODE		(1 << 22)
#define	SYSTEM_FLAG2_AFE_INI_STATE	(1 << 23)

/* Public typedef -----------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
void appSystemFlagSetSystemReady(void);
void appSystemFlagClearReadOnFlag(void);

uint32_t apiSystemFlagGetFlag1(void);
uint32_t apiSystemFlagGetFlag2(void);

void apiSystemFlagOpen(void);

#endif /* _SYSTEM_FLAG_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


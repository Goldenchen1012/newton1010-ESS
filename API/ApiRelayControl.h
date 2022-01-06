/**
  ******************************************************************************
  * @file        AppRelayControl.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/10
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _APP_RELAY_CONTROL_H_
#define _APP_RELAY_CONTROL_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "define.h"

#ifdef __cplusplus
extern "C" {
#endif
/* Public typedef -----------------------------------------------------------*/

/* Public define ------------------------------------------------------------*/

/* Public macro -------------------------------------------------------------*/



#define FLAG1_ALARM_MASK  (SYSTEM_FLAG1_OVP_L1 \
                      | SYSTEM_FLAG1_OVP_L2 \
                      | SYSTEM_FLAG1_UVP_L1 \
                      | SYSTEM_FLAG1_UVP_L2 \
                      | SYSTEM_FLAG1_DOCP_L1 \
                      | 0)

#define FLAG1_PROTECT_MASK  (SYSTEM_FLAG1_OVP_L3 \
                        | SYSTEM_FLAG1_OVP_2nd \
                        | SYSTEM_FLAG1_UVP_L3 \
                        | SYSTEM_FLAG1_UVP_2nd \
                        | SYSTEM_FLAG1_COTP_L3 \
                        | SYSTEM_FLAG1_CUTP_L3 \
                    	| SYSTEM_FLAG1_DOTP_L3 \
                    	| SYSTEM_FLAG1_DUTP_L3 \
                    	| SYSTEM_FLAG1_UT_2nd \
                    	| SYSTEM_FLAG1_OT_2nd \
                    	| SYSTEM_FLAG1_COCP_L3 \
                    	| SYSTEM_FLAG1_COCP_LATCH \
                    	| SYSTEM_FLAG1_DOCP_L3 \
                    	| SYSTEM_FLAG1_DOCP_LATCH \
                        | 0)

#define FLAG2_ALARM_MASK  (0 \
						| 0 \
						| 0)	

#ifdef	DEBUG_MODE
#define FLAG2_PROTECT_MASK  (0 \
						| SYSTEM_FLAG2_OVP_PF \
						| SYSTEM_FLAG2_UVP_PF \
						| SYSTEM_FLAG2_P_RELAY_FAIL \
						| SYSTEM_FLAG2_M_RELAY_FAIL \
						| 0)	
#else						
#define FLAG2_PROTECT_MASK  (0 \
						| SYSTEM_FLAG2_AFE_L2 \
						| SYSTEM_FLAG2_EPO_ENABLE \
						| SYSTEM_FLAG2_OVP_PF \
						| SYSTEM_FLAG2_UVP_PF \
						| SYSTEM_FLAG2_P_RELAY_FAIL \
						| SYSTEM_FLAG2_M_RELAY_FAIL \
						| SYSTEM_FLAG2_SP_FB \
						| SYSTEM_FLAG2_AFE_INI_STATE \
						| 0)	
#endif

#if	0                        
//#define	
#define		(1 << 7)
#define	SYSTEM_FLAG1_COTP_L1	(1 << 8)
#define	SYSTEM_FLAG1_COTP_L2	(1 << 9)
#define		(1 << 10)
#define	SYSTEM_FLAG1_CUTP_L1	(1 << 11)
#define SYSTEM_FLAG1_CUTP_L2	(1 << 12)
#define 	(1 << 13)
#define SYSTEM_FLAG1_DOTP_L1	(1 << 14)
#define SYSTEM_FLAG1_DOTP_L2	(1 << 15)
#define 	(1 << 16)
#define SYSTEM_FLAG1_DUTP_L1	(1 << 17)
#define SYSTEM_FLAG1_DUTP_L2	(1 << 18)
#define 	(1 << 19)
#define 		(1 << 20)
#define 		(1 << 21)
#define SYSTEM_FLAG1_COCP_L1	(1 << 22)
#define SYSTEM_FLAG1_COCP_L2	(1 << 23)
#define 	(1 << 24)
#define 	(1 << 25)
#define 	(1 << 26)
#define SYSTEM_FLAG1_DOCP_L2	(1 << 27)
#define 	(1 << 28)
#define 	(1 << 29)
#define	SYSTEM_FLAG1_CANID			(1 << 30)
#define SYSTEM_FLAG1_SYSTEM_READY	(1 << 31)

#define SYSTEM_FLAG2_AFE_L1			(1 << 0)
#define SYSTEM_FLAG2_AFE_L2			(1 << 1)
#define 		(1 << 2)
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
#define 			(1 << 17)
#define SYSTEM_FLAG2_DI1			(1 << 18)
#define SYSTEM_FLAG2_DI2			(1 << 19)
#define SYSTEM_FLAG2_OD_IN			(1 << 20)
#define SYSTEM_FLAG2_NFAULT			(1 << 21)
#endif



/* Public function prototypes -----------------------------------------------*/
void apiRelayControlSetMasterTurnOnFlag(void);

void apiRelayControlOpen(void);
void apiRelayControlMainRelayOff(void);



#ifdef __cplusplus
}
#endif


	

#endif /* _APP_RELAY_CONTROL_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



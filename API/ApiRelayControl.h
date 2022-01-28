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
                      | SYSTEM_FLAG1_COTP_L1 \
                      | SYSTEM_FLAG1_COTP_L2 \
                      | SYSTEM_FLAG1_CUTP_L1 \
                      | SYSTEM_FLAG1_CUTP_L2 \
                      | SYSTEM_FLAG1_DOTP_L1 \
                      | SYSTEM_FLAG1_DOTP_L2 \
                      | SYSTEM_FLAG1_DUTP_L1 \
                      | SYSTEM_FLAG1_DUTP_L2 \
                      | SYSTEM_FLAG1_COCP_L1 \
                      | SYSTEM_FLAG1_COCP_L2 \
                      | SYSTEM_FLAG1_DOCP_L1 \
                      | SYSTEM_FLAG1_DOCP_L2 \
                      | 0)
/*
#define	SYSTEM_FLAG1_OVP_L3		(1 << 2)
#define	SYSTEM_FLAG1_OVP_2nd	(1 << 3)
#define	SYSTEM_FLAG1_UVP_L3		(1 << 6)
#define	SYSTEM_FLAG1_UVP_2nd	(1 << 7)
#define		(1 << 8)
#define		(1 << 9)
#define	SYSTEM_FLAG1_COTP_L3	(1 << 10)
#define		(1 << 11)
#define 	(1 << 12)
#define SYSTEM_FLAG1_CUTP_L3	(1 << 13)
#define 	(1 << 14)
#define 	(1 << 15)
#define SYSTEM_FLAG1_DOTP_L3	(1 << 16)
#define 	(1 << 17)
#define 	(1 << 18)
#define SYSTEM_FLAG1_DUTP_L3	(1 << 19)
#define SYSTEM_FLAG1_UT_2nd		(1 << 20)
#define SYSTEM_FLAG1_OT_2nd		(1 << 21)
#define 	(1 << 22)
#define 	(1 << 23)
#define SYSTEM_FLAG1_COCP_L3	(1 << 24)
#define SYSTEM_FLAG1_COCP_LATCH	(1 << 25)
#define 	(1 << 26)
#define 	(1 << 27)
#define SYSTEM_FLAG1_DOCP_L3	(1 << 28)
#define SYSTEM_FLAG1_DOCP_LATCH	(1 << 29)
#define	SYSTEM_FLAG1_CANID_READY	(1U << 30)
#define SYSTEM_FLAG1_SYSTEM_READY	(1U << 31)
*/                      

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

#define FLAG3_ALARM_MASK  (0 \
					  | SYSTEM_FLAG3_RLY1_OT_L1 \
                      | SYSTEM_FLAG3_RLY1_OT_L2 \
                      | SYSTEM_FLAG3_RLY2_OT_L1 \
                      | SYSTEM_FLAG3_RLY2_OT_L2 \
                      | SYSTEM_FLAG3_AMBI_OT_L1 \
                      | SYSTEM_FLAG3_AMBI_OT_L2 \
                      | SYSTEM_FLAG3_BUSBARP_OT_L1 \
                      | SYSTEM_FLAG3_BUSBARP_OT_L2 \
                      | SYSTEM_FLAG3_BUSBARN_OT_L1 \
                      | SYSTEM_FLAG3_BUSBARN_OT_L2 \
                      | 0)

#define FLAG3_PROTECT_MASK  (0 \
						| SYSTEM_FLAG3_RLY1_OT_L3 \
						| SYSTEM_FLAG3_RLY2_OT_L3 \
						| SYSTEM_FLAG3_AMBI_OT_L3 \
						| SYSTEM_FLAG3_BUSBARP_OT_L3 \
						| SYSTEM_FLAG3_BUSBARN_OT_L3 \
						| 0)
#define FLAG4_ALARM_MASK  (0 \
                      | 0)

#define FLAG4_PROTECT_MASK  (0 \
						| 0)

/* Public function prototypes -----------------------------------------------*/
void apiRelayControlSetMasterTurnOnFlag(void);

void apiRelayControlOpen(void);
void apiRelayControlMainRelayOff(void);



#ifdef __cplusplus
}
#endif


	

#endif /* _APP_RELAY_CONTROL_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



/**
  ******************************************************************************
  * @file        AppBalance.h
  * @author      Johnny
  * @version     v0.0
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

#ifndef _APP_BALANCE_H_
#define _APP_BALANCE_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/* Public define ------------------------------------------------------------*/
#define tAppBalanceEvtHandler tLibRegisterEvtHandler

/* Public typedef -----------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/
enum{
	APP_CHG_BALANCE_SET=1,
	APP_CHG_BALANCE_RELEASE,
	APP_DHG_BALANCE_SET,
	APP_DHG_BALANCE_RELEASE,
	APP_RLX_BALANCE_SET,
	APP_RLX_BALANCE_RELEASE,
};
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
uint8_t	appBalanceIsBalanceSet(uint16_t cell);
void appBalanceOpen(tAppBalanceEvtHandler );
uint8_t	appBalanceIsBalanceDutyOn(void);


#ifdef __cplusplus
}
#endif


	

#endif /* _APP_BALANCE_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    

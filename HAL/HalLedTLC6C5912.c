/**
  ******************************************************************************
  * @file        HalLedTLC6C5912.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2022/1/26
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"
#include "HalLed.h"
#include "smp_TLC6C5912.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	halLedDebugMsg(str)	appSerialCanDavinciSendTextMessage(str);

/* Private macro -------------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/

/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
void halLedSetPercent20LedState(uint8_t on)
{
	if(on)
		smp_TLC6C5912_Percent20_LED(LED_ON);
	else
		smp_TLC6C5912_Percent20_LED(LED_OFF);
}
void halLedSetPercent40LedState(uint8_t on)
{
	if(on)
		smp_TLC6C5912_Percent40_LED(LED_ON);
	else
		smp_TLC6C5912_Percent40_LED(LED_OFF);
}
void halLedSetPercent60LedState(uint8_t on)
{
	if(on)
		smp_TLC6C5912_Percent60_LED(LED_ON);
	else
		smp_TLC6C5912_Percent60_LED(LED_OFF);
}
void halLedSetPercent80LedState(uint8_t on)
{
	if(on)
		smp_TLC6C5912_Percent80_LED(LED_ON);
	else
		smp_TLC6C5912_Percent80_LED(LED_OFF);
}
void halLedSetPercent100LedState(uint8_t on)
{
	if(on)
		smp_TLC6C5912_Percent100_LED(LED_ON);
	else
		smp_TLC6C5912_Percent100_LED(LED_OFF);
}
void halLedSetChargeLedState(uint8_t on)
{
	if(on)	
		smp_TLC6C5912_Charge_LED(LED_ON);
	else
		smp_TLC6C5912_Charge_LED(LED_OFF);
}
void halLedSeDisChargeLedState(uint8_t on)
{
	if(on)	
		smp_TLC6C5912_Discharge_LED(LED_ON);
	else
		smp_TLC6C5912_Discharge_LED(LED_OFF);
}
void halLedSeAlarmLedState(uint8_t on)
{
	if(on)	
		smp_TLC6C5912_Alarm_LED(LED_ON);
	else
		smp_TLC6C5912_Alarm_LED(LED_OFF);
}
void halLedSeCommLedState(uint8_t on)
{
	if(on)	
		smp_TLC6C5912_Comm_LED(LED_ON);
	else
		smp_TLC6C5912_Comm_LED(LED_OFF);
}

void halLedAllLedOn(void)
{
	smp_TLC6C5912_All_LED_On();
}
void halLedAllLedOff(void)
{
	smp_TLC6C5912_All_LED_Off();
}
void halLedOpen(void)
{
	smp_TLC6C5912_Init();
	halLedAllLedOff();
}
/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


/**
  ******************************************************************************
  * @file        appTaskManagement.c
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/12
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"
#include "LibDebug.h"
#include "halafe.h"
#include "halTimer.h"
#include "halUart.h"
#include "appserialuart.h"
#include "apptaskmanagement.h"
#include "appprotect.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static tHalTimer	mHalTimer3={3, 1000};
static uint8_t	sw_count1ms = 0;
static uint16_t	t1s_count;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void appTaskT1msHanlder(void)
{
	//GPIOA->ODR ^= GPIO_PIN_0;
	appProtectHanlder(APP_TASK_EVT_T1MS);
}
void appTaskT10msHanlder(void)
{
//	GPIOA->ODR ^= (GPIO_PIN_1 | GPIO_PIN_3);//| GPIO_PIN_1 | GPIO_PIN_2);
	appSerialUartHvEssHanlder(APP_TASK_EVT_T10MS);	
	appProtectHanlder(APP_TASK_EVT_T10MS);
	AppScuGaugeHanlder(APP_TASK_EVT_T10MS);

}
void appTaskT1SecHanlder(void)
{
	AppScuGaugeHanlder(APP_TASK_EVT_T1SEC);
}
/* Public function prototypes -----------------------------------------------*/
void appTaskTimerHanlder(void *pin, uint16_t evt, void *pData)
{
	uint8_t	buf[10];
	uint8_t	i;
	sw_count1ms++;
}

void appTaskOpen(void){
	sw_count1ms = 0;
	t1s_count = 0;
	HalTimerOpen(&mHalTimer3, appTaskTimerHanlder);
}

void appTaskManagementHanlder(void)
{
	uint8_t	i;
	for(i=0; i<5; i++)
	{
		if(sw_count1ms > 0)
		{
			sw_count1ms--;
			appTaskT1msHanlder();
			t1s_count++;
			if( (t1s_count % 10) == 5)
				appTaskT10msHanlder();
			//else if( (t1s_count % 10) == 7)
			//	appTaskT10msHanlder();
			if(t1s_count >= 1000)
			{
				t1s_count -= 1000;
				appTaskT1SecHanlder();
			}
		}
	}
}


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    




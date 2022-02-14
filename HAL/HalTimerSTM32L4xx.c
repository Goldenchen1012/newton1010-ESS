/**
  ******************************************************************************
  * @file        HalTmerSTM32L4xx.c
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/7
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
#include "halTimer.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static TIM_HandleTypeDef    TimHandle3;
static tHalTimerEvtHandler	TimerEvtHandler3;

/* Private function prototypes -----------------------------------------------*/

/******************************************************************************/
/*                 STM32L4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&TimHandle3);
	if(TimerEvtHandler3){
	    TimerEvtHandler3(0,0,0);
	}
		
	#if 0
	GPIOA->ODR ^= (GPIO_PIN_1 | GPIO_PIN_3);//| GPIO_PIN_1 | GPIO_PIN_2);
	GPIOD->ODR ^= GPIO_PIN_13;
  #endif
	
}

/* Public function prototypes -----------------------------------------------*/
tErrCode HalTimerOpen(tHalTimer *pHalTimer, tHalTimerEvtHandler evtHandler)
{
	__IO uint32_t uwPrescalerValue = 0;

	if(pHalTimer->TimerNo == 3)
	{
      __HAL_RCC_TIM3_CLK_ENABLE();
		
		  uwPrescalerValue = (uint32_t)(SystemCoreClock / 1000000) - 1;

		  /* Set TIMx instance */
  		TimHandle3.Instance = TIM3;

		  /* Initialize TIMx peripheral as follows:
	       + Period = 10000 - 1
    	   + Prescaler = (SystemCoreClock/10000) - 1
    	   + ClockDivision = 0
	       + Counter direction = Up
  		*/
  		TimHandle3.Init.Period            = pHalTimer->IntervalUs - 1;
  		TimHandle3.Init.Prescaler         = uwPrescalerValue;
  		TimHandle3.Init.ClockDivision     = 0;
  		TimHandle3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  		TimHandle3.Init.RepetitionCounter = 0;

  		if (HAL_TIM_Base_Init(&TimHandle3) != HAL_OK)
  		{
    		/* Initialization Error */
    		return RES_ERROR_INIT;//Error_Handler();
  		}

  		/*##-2- Start the TIM Base generation in interrupt mode ####################*/
  		/* Start Channel1 */
  		if (HAL_TIM_Base_Start_IT(&TimHandle3) != HAL_OK)
  		{
    		/* Starting Error */
    		//Error_Handler();
    		return RES_ERROR_INIT;
  		}
  		TimerEvtHandler3 = evtHandler;
		
		  /* Enable and set Button EXTI Interrupt to the lowest priority */
		  //HAL_NVIC_SetPriority(TIM3_IRQn, 0x0F, 0x00);
		  HAL_NVIC_EnableIRQ(TIM3_IRQn);
	
	}
	else 
		return RES_ERROR_INVALID_PARAM;
	return RES_SUCCESS;
}

uint16_t halTimerGetCountValue(tHalTimer *pHalTimer)
{
	if(pHalTimer->TimerNo == 3)
	{
		return __HAL_TIM_GET_COUNTER(&TimHandle3);
	}
	return 0;
	
}

tErrCode HalTimerClose(tHalTimer *pHalTimer)
{
	if(pHalTimer->TimerNo == 3)
	{
		TimerEvtHandler3 = NULL;
	}
	else 
		return RES_ERROR_INVALID_PARAM;	
	return RES_SUCCESS;

}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    

/**
  ******************************************************************************
  * @file    stm32l4xx_Hero.c
  * @author  SMP Randy
  * @version V0.0.1
  * @date    10/January/2017
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_Davinci.h"
#include "smp_debug.h"
#include "main.h"
//#include "smp_cli.h"
//#include "smp_button.h"
//#include "smp_led.h"
/* Exported macro ------------------------------------------------------------*/
#define LOW_BAT_THRESHOLD		5
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef    smp_timer_handle;

//uint16_t smp_time_count = 0;

/**
  * @brief  Configures Extern GPIO.
  * @param  Extern GPIO: Extern GPIO to be configured. 
  *          This parameter can be one of the following values:
  *            @arg  (GPIOB,GPIO_PIN_5)
  * @retval None
  */
void BSP_EXT_Power_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	/* Enable the GPIO Clock */
	GPIOx_GPIO_CLK_ENABLE(GPIOx);
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Pin = GPIO_Pin;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct); 
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin,PinState);
}
#if	0
int8_t smp_timer_handler(uint32_t sample_rate)
{
	uint32_t prescaler_val = 0;
	
	prescaler_val = (uint32_t) ((SystemCoreClock / (sample_rate*1600)) - 1);
	
	/* Set TIMx instance */
	smp_timer_handle.Instance = SMP_TIM;
	smp_timer_handle.Init.Period = 1600 - 1;
	smp_timer_handle.Init.Prescaler = prescaler_val;
	smp_timer_handle.Init.ClockDivision = 0;
	smp_timer_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
		
	HAL_TIM_Base_DeInit(&smp_timer_handle);
	
	if(HAL_TIM_Base_Init(&smp_timer_handle) != HAL_OK)
		return SMP_ERROR_NOT_FOUND;

	/*##-2- Start the TIM Base generation in interrupt mode ####################*/
	/* Start Channel1 */
	if(HAL_TIM_Base_Start_IT(&smp_timer_handle) != HAL_OK)
		return SMP_ERROR_NOT_FOUND;
	
	return SMP_SUCCESS;
}

int smp_timer_deinit(void)
{
	HAL_TIM_Base_Stop_IT(&smp_timer_handle);
	HAL_TIM_Base_DeInit(&smp_timer_handle);
	return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == SMP_TIM){
		/* SMP timer handle here */
    smp_time_count++;
	}
}



uint16_t smp_time_count_get(void){
    return(smp_time_count);
}

void smp_time_count_set(uint16_t val){
    smp_time_count = val;
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Simplo all right reserved *****END OF FILE****/


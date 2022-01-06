/**
  ******************************************************************************
  * @file    stm32l4xx_Davinci.h
  * @author  Golden Chen
  * @version V0.0.2
  * @date    2021/12/30
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L4XX_DAVINCI_H
#define __STM32L4XX_DAVINCI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "Bsp.h"
/* Private macro -------------------------------------------------------------*/	 
#define GPIOx_GPIO_CLK_ENABLE(__PORT__)      	do { if((GPIO_TypeDef*)(__PORT__) == GPIOA)		{__HAL_RCC_GPIOA_CLK_ENABLE();} else \
													  if((GPIO_TypeDef*)(__PORT__) == GPIOB) 	{__HAL_RCC_GPIOB_CLK_ENABLE();} else \
													  if((GPIO_TypeDef*)(__PORT__) == GPIOC) 	{__HAL_RCC_GPIOC_CLK_ENABLE();} else \
													  if((GPIO_TypeDef*)(__PORT__) == GPIOD) 	{__HAL_RCC_GPIOD_CLK_ENABLE();} else \
													  if((GPIO_TypeDef*)(__PORT__) == GPIOE) 	{__HAL_RCC_GPIOE_CLK_ENABLE();} else \
													  if((GPIO_TypeDef*)(__PORT__) == GPIOF) 	{__HAL_RCC_GPIOF_CLK_ENABLE();} else \
													  if((GPIO_TypeDef*)(__PORT__) == GPIOG) 	{__HAL_RCC_GPIOG_CLK_ENABLE(); 		 \
																								 HAL_PWREx_EnableVddIO2();} 	else \
													  if((GPIO_TypeDef*)(__PORT__) == GPIOH) 	{__HAL_RCC_GPIOB_CLK_ENABLE();}  	 \
													} while(0)										  

/* Definition of SMP TIM instance */
#define SMP_TIM															   TIM2
#define SMP_TIM_CLK_ENABLE									   __HAL_RCC_TIM2_CLK_ENABLE
#define SMP_TIM_CLK_DISABLE									   __HAL_RCC_TIM2_CLK_DISABLE
/* Definition for SMP TIM NVIC */
#define SMP_TIM_IRQn                           TIM2_IRQn
#define SMP_TIM_IRQHandler                	   TIM2_IRQHandler
													
/* Definition of User TIM instance */
#define USER_TIM														   TIM3
#define USER_TIM_CLK_ENABLE()								   __HAL_RCC_TIM3_CLK_ENABLE()
#define USER_TIM_CLK_DISABLE()							   __HAL_RCC_TIM3_CLK_DISABLE()
/* Definition for User TIM NVIC */
#define USER_TIM_IRQn                      		 TIM3_IRQn
#define USER_TIM_IRQHandler                		 TIM3_IRQHandler

/* Definition of LED TIM instance */
#define LED_TIM															   TIM4
#define LED_TIM_CLK_ENABLE()								   __HAL_RCC_TIM4_CLK_ENABLE()
#define LED_TIM_CLK_DISABLE()								   __HAL_RCC_TIM4_CLK_DISABLE()
/* Definition for LED TIM NVIC */
#define LED_TIM_IRQn                      	   TIM4_IRQn
#define LED_TIM_IRQHandler                	   TIM4_IRQHandler													
/* Private variables ---------------------------------------------------------*/
//uint16_t smp_time_count;
/* Exported functions ------------------------------------------------------- */
void BSP_EXT_Power_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
int8_t smp_timer_handler(uint32_t sample_rate);
int8_t user_timer_handler(uint32_t sample_rate);
int8_t led_timer_handler(uint32_t sample_rate);
int smp_timer_deinit(void);
int user_timer_deinit(void);
int led_timer_deinit(void);

uint16_t smp_time_count_get(void);
void smp_time_count_set(uint16_t val);

#ifdef __cplusplus
}
#endif

#endif /* __STM32L4XX_DAVINCI_H */
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

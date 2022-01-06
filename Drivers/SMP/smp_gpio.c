/**
  ******************************************************************************
  * @file    smp_gpio.c
  * @author  Golden Chen
  * @version V0.0.1
  * @date    2021/09/13
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 SMP</center></h2>
  *
  * 
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "smp_gpio.h"
#include "smp_debug.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define SMP_GPIO_CLK_ENABLE(__PORT__)      	do { if((__PORT__) == SMP_GPIOA)	{__HAL_RCC_GPIOA_CLK_ENABLE();} else \
												 if((__PORT__) == SMP_GPIOB) 	{__HAL_RCC_GPIOB_CLK_ENABLE();} else \
												 if((__PORT__) == SMP_GPIOC) 	{__HAL_RCC_GPIOC_CLK_ENABLE();} else \
												 if((__PORT__) == SMP_GPIOD) 	{__HAL_RCC_GPIOD_CLK_ENABLE();} else \
												 if((__PORT__) == SMP_GPIOE) 	{__HAL_RCC_GPIOE_CLK_ENABLE();} else \
												 if((__PORT__) == SMP_GPIOF) 	{__HAL_RCC_GPIOF_CLK_ENABLE();} else \
												 if((__PORT__) == SMP_GPIOG) 	{__HAL_RCC_GPIOG_CLK_ENABLE(); 		 \
																					 HAL_PWREx_EnableVddIO2();}		 \
											   } while(0)
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
uint32_t SMP_GPIO_MODE(gpio_mode mode);
uint16_t SMP_GPIO_PIN(gpio_pin pin);
GPIO_TypeDef* SMP_GPIO_PORT(gpio_port port);
											   
int8_t smp_gpio_init(smp_gpio_t *p_gpio)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	/* Enable the GPIO Clock */
	SMP_GPIO_CLK_ENABLE(p_gpio->port);
	GPIO_InitStruct.Mode = SMP_GPIO_MODE(p_gpio->mode);
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Pin = SMP_GPIO_PIN(p_gpio->pin);
	HAL_GPIO_Init(SMP_GPIO_PORT(p_gpio->port), &GPIO_InitStruct);
	return SMP_SUCCESS;
}

int8_t smp_gpio_deinit(smp_gpio_t *p_gpio)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	/* Enable the GPIO Clock */
	SMP_GPIO_CLK_ENABLE(p_gpio->port);
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Pin = SMP_GPIO_PIN(p_gpio->pin);
	HAL_GPIO_Init(SMP_GPIO_PORT(p_gpio->port), &GPIO_InitStruct);
	HAL_GPIO_DeInit(SMP_GPIO_PORT(p_gpio->port), SMP_GPIO_PIN(p_gpio->pin));	
	return SMP_SUCCESS;
}

int8_t smp_gpio_set_state(smp_gpio_t *p_gpio, smp_gpio_state state)
{
	switch(state){
		case GPIO_ACTIVE_LOW:
			HAL_GPIO_WritePin(SMP_GPIO_PORT(p_gpio->port), SMP_GPIO_PIN(p_gpio->pin), GPIO_PIN_RESET);
			break;
		case GPIO_ACTIVE_HIGH:
			HAL_GPIO_WritePin(SMP_GPIO_PORT(p_gpio->port), SMP_GPIO_PIN(p_gpio->pin), GPIO_PIN_SET);
			break;
		case GPIO_ACTIVE_TOGGLE:
			HAL_GPIO_TogglePin(SMP_GPIO_PORT(p_gpio->port), SMP_GPIO_PIN(p_gpio->pin));
			break;		
		default: 
			return SMP_ERROR_INVALID_PARAM;
	}
	return SMP_SUCCESS;
}

int8_t smp_gpio_get_state(smp_gpio_t *p_gpio, smp_gpio_state *state)
{
	*state = (smp_gpio_state)HAL_GPIO_ReadPin(SMP_GPIO_PORT(p_gpio->port), SMP_GPIO_PIN(p_gpio->pin));
	return SMP_SUCCESS;
}

uint32_t SMP_GPIO_MODE(gpio_mode mode)
{
	switch(mode){
		case GPIO_MODE_INPUT:									return GPIO_MODE_INPUT;
		case SMP_GPIO_MODE_OUTPUT_PP:					return GPIO_MODE_OUTPUT_PP;
		default:
		case SMP_GPIO_MODE_OUTPUT_OD:					return GPIO_MODE_OUTPUT_OD;
		case SMP_GPIO_MODE_ANALOG:						return GPIO_MODE_ANALOG;
		case SMP_GPIO_MODE_IT_RISING:					return GPIO_MODE_IT_RISING;
		case SMP_GPIO_MODE_IT_FALLING:				return GPIO_MODE_IT_FALLING;
		case SMP_GPIO_MODE_IT_RISING_FALLING:	return GPIO_MODE_IT_RISING_FALLING;
	}
}

uint16_t SMP_GPIO_PIN(gpio_pin pin)
{
	switch(pin){
		default:
		case PIN0:	return GPIO_PIN_0;
		case PIN1:	return GPIO_PIN_1;
		case PIN2:	return GPIO_PIN_2;
		case PIN3:	return GPIO_PIN_3;
		case PIN4:	return GPIO_PIN_4;
		case PIN5:	return GPIO_PIN_5;
		case PIN6:	return GPIO_PIN_6;
		case PIN7:	return GPIO_PIN_7;
		case PIN8:	return GPIO_PIN_8;
		case PIN9:	return GPIO_PIN_9;
		case PIN10:	return GPIO_PIN_10;
		case PIN11:	return GPIO_PIN_11;
		case PIN12:	return GPIO_PIN_12;
		case PIN13:	return GPIO_PIN_13;
		case PIN14:	return GPIO_PIN_14;
		case PIN15:	return GPIO_PIN_15;			
	}
}

GPIO_TypeDef* SMP_GPIO_PORT(gpio_port port)
{
	switch(port){
		default:
		case SMP_GPIOA:	return GPIOA;
		case SMP_GPIOB:	return GPIOB;
		case SMP_GPIOC:	return GPIOC;
		case SMP_GPIOD:	return GPIOD;
		case SMP_GPIOE:	return GPIOE;
		case SMP_GPIOF:	return GPIOF;
		case SMP_GPIOG:	return GPIOG;
	}
}

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

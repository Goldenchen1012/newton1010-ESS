/**
  ******************************************************************************
  * @file    smp_gpio.h 
  * @author  Golden Chen 
  * @version V0.0.1
  * @date    2021/09/30
  * @brief   Header for smp_gpio.c module
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMP_GPIO_H
#define __SMP_GPIO_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32l4xx_hal.h"
/* Exported types ------------------------------------------------------------*/
typedef enum{
	SMP_GPIOA = 0,		/* GPIO port A */
	SMP_GPIOB,			/* GPIO port B */
	SMP_GPIOC,			/* GPIO port C */
	SMP_GPIOD,			/* GPIO port D */
	SMP_GPIOE,			/* GPIO port E */
	SMP_GPIOF,			/* GPIO port F */
	SMP_GPIOG			/* GPIO port G */
}gpio_port;

typedef enum{
	PIN0 = 0,	/* Pin 0 */
	PIN1,		/* Pin 1 */
	PIN2,		/* Pin 2 */
	PIN3,		/* Pin 3 */
	PIN4,		/* Pin 4 */
	PIN5,		/* Pin 5 */
	PIN6,		/* Pin 6 */
	PIN7,		/* Pin 7 */
	PIN8,		/* Pin 8 */
	PIN9,		/* Pin 9 */
	PIN10,		/* Pin 10 */
	PIN11,		/* Pin 11 */
	PIN12,		/* Pin 12 */
	PIN13,		/* Pin 13 */
	PIN14,		/* Pin 14 */
	PIN15		/* Pin 15 */
}gpio_pin;

typedef enum{
	SMP_GPIO_MODE_INPUT = 0,		/* Input float mode */
	SMP_GPIO_MODE_OUTPUT_PP,		/* Out push pull mode */
	SMP_GPIO_MODE_OUTPUT_OD,		/* Out open drain mode */
	SMP_GPIO_MODE_ANALOG,			/* Analog mode */
	SMP_GPIO_MODE_IT_RISING,		/* External interrupt mode with rising edge trigger */
	SMP_GPIO_MODE_IT_FALLING,		/* External interrupt mode with falling edge trigger */
	SMP_GPIO_MODE_IT_RISING_FALLING	/* External interrupt mode with rising/falling edge trigger */
}gpio_mode;

typedef enum{
	GPIO_ACTIVE_LOW = 0,	/* Indicates that a GPIO is active low.    */
	GPIO_ACTIVE_HIGH,		  /* Indicates that a GPIO is active high.   */
	GPIO_ACTIVE_TOGGLE    /* Indicates that a GPUO is active toggle. */
}smp_gpio_state;

typedef void(*gpio_handler_t)( void *p_context);

typedef struct{
	gpio_port			port;			/* GPIO port */
	gpio_pin			pin;			/* GPIO pin number */
	gpio_mode			mode;			/* GPIO mode */
	gpio_handler_t		gpio_handler;	/* Handler to be called when GPIO interrupt occurred */
}smp_gpio_t;
/* Exported constants --------------------------------------------------------*/ 
/* Exported defines --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int8_t smp_gpio_init(smp_gpio_t *p_gpio);
int8_t smp_gpio_deinit(smp_gpio_t *p_gpio);
int8_t smp_gpio_set_state(smp_gpio_t *p_gpio, smp_gpio_state state);
int8_t smp_gpio_get_state(smp_gpio_t *p_gpio, smp_gpio_state *state);
uint16_t SMP_GPIO_PIN(gpio_pin pin);

#endif /* __SMP_GPIO_H */

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

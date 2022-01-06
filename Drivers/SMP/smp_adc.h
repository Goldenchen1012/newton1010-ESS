/**
  ******************************************************************************
  * @file    smp_adc.h
  * @author  Steve Cheng/ Golden
  * @version V0.0.1
  * @date    2021/10/21
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _SMP_ADC_H_
#define _SMP_ADC_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "Bsp.h"

/* Define ------------------------------------------------------------------*/
#define ADC_MAX_IN_PIN_NUM                     16

#define ADC1_PIN_TOTAL_CNT                     5
#define	ADC2_PIN_TOTAL_CNT                     1
#define ADC3_PIN_TOTAL_CNT                     1

#ifndef ADC_CFGR_FIELDS_1
#define ADC_CFGR_FIELDS_1                      ((ADC_CFGR_RES    | ADC_CFGR_ALIGN   |          \
                                                 ADC_CFGR_CONT   | ADC_CFGR_OVRMOD  |          \
                                                 ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM |          \
                                                 ADC_CFGR_EXTEN  | ADC_CFGR_EXTSEL))            /*!< ADC_CFGR fields of parameters that can be updated when no regular conversion is on-going */
#endif
/*Hardware configuration*/

/* ----------- ADC Clock source ----------*/
/* Choose one(clock source= MSI/HSE/HSI)*/
#define ADC_PLL_CLOCK_SOURCE          1

#if (ADC_PLL_CLOCK_SOURCE == 0)
  #define PLL_SOURCE 	      RCC_PLLSOURCE_MSI
#elif (ADC_PLL_CLOCK_SOURCE == 1)
  #define ADC_PLL_SOURCE    RCC_PLLSOURCE_HSE
#else
  #define ADC_PLL_SOURCE    RCC_PLLSOURCE_HSI
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

typedef enum{
	adc1 = 0,
	adc2,
	adc3, 	
}adc_module_number;

/* Exported constants --------------------------------------------------------*/ 
extern volatile uint16_t SMPADCVALUE1[ADC_MAX_IN_PIN_NUM];
extern ADC_HandleTypeDef smp_adc_1;
extern DMA_HandleTypeDef smp_dma_adc1;
/* Exported defines --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void Enable_DMA_Control_Clock(void);
void smp_adc_gpio_init(ADC_HandleTypeDef* hadc);
void smp_adc_deinit(adc_module_number num);
int8_t smp_adc_adc_para_init(adc_module_number num);

int8_t hal_internal_adc_get(uint16_t *adc_data,adc_module_number num, bsp_adc_init_io_config adc_confing);
uint32_t GetADCChParaSTM32L496(adc_module_number num, GPIO_TypeDef *Port, uint16_t Pin);

#endif


/************************ (C) COPYRIGHT Simplo all right reserved *****END OF FILE****/



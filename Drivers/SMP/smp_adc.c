/**
  ******************************************************************************
  * @file    smp_adc.c
  * @author  Steve Cheng/ Golden
  * @version V0.0.1
  * @date    2021/10/21
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 SMP</center></h2>
  *
  * 
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "smp_adc.h"
#include "main.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
DMA_HandleTypeDef smp_dma_adc1;
ADC_HandleTypeDef smp_adc_1;
ADC_HandleTypeDef smp_adc_2;
ADC_HandleTypeDef smp_adc_3;
uint32_t HAL_RCC_ADC_CLK_ENABLED=0;

uint8_t bsp_mcu_adc_enable_cnt = 0;


bsp_adc_init_io_config bsp_mcu_adc_config_table[ADC_MAX_IN_PIN_NUM]={{BSP_MCU_ADC_GPIO_1 , BSP_MCU_ADC_PIN_1 },   
                                                                     {BSP_MCU_ADC_GPIO_2 , BSP_MCU_ADC_PIN_2 },
                                                                     {BSP_MCU_ADC_GPIO_3 , BSP_MCU_ADC_PIN_3 },
                                                                     {BSP_MCU_ADC_GPIO_4 , BSP_MCU_ADC_PIN_4 },
                                                                     {BSP_MCU_ADC_GPIO_5 , BSP_MCU_ADC_PIN_5 },
                                                                     {BSP_MCU_ADC_GPIO_6 , BSP_MCU_ADC_PIN_6 },
                                                                     {BSP_MCU_ADC_GPIO_7 , BSP_MCU_ADC_PIN_7 },
                                                                     {BSP_MCU_ADC_GPIO_8 , BSP_MCU_ADC_PIN_8 },
                                                                     {BSP_MCU_ADC_GPIO_9 , BSP_MCU_ADC_PIN_9 },
                                                                     {BSP_MCU_ADC_GPIO_10 , BSP_MCU_ADC_PIN_10 },
                                                                     {BSP_MCU_ADC_GPIO_11 , BSP_MCU_ADC_PIN_11 },
                                                                     {BSP_MCU_ADC_GPIO_12 , BSP_MCU_ADC_PIN_12 },																							
                                                                     {BSP_MCU_ADC_GPIO_13 , BSP_MCU_ADC_PIN_13 },
                                                                     {BSP_MCU_ADC_GPIO_14 , BSP_MCU_ADC_PIN_14 },
                                                                     {BSP_MCU_ADC_GPIO_15 , BSP_MCU_ADC_PIN_15 },
                                                                     {BSP_MCU_ADC_GPIO_16 , BSP_MCU_ADC_PIN_16 },																							
                                                                    };                                         

bsp_adc_init_io_config  *p_adc_cong[16]={0};																																		
																																		
uint32_t  adc_sconfig_rank[ADC_MAX_IN_PIN_NUM]={ ADC_REGULAR_RANK_1  ,
	                                               ADC_REGULAR_RANK_2  ,
                                                 ADC_REGULAR_RANK_3  ,
                                                 ADC_REGULAR_RANK_4  ,
                                                 ADC_REGULAR_RANK_5  ,
                                                 ADC_REGULAR_RANK_6  ,	
	                                               ADC_REGULAR_RANK_7  ,
                                                 ADC_REGULAR_RANK_8  ,
                                                 ADC_REGULAR_RANK_9  ,
                                                 ADC_REGULAR_RANK_10 ,	
		                                             ADC_REGULAR_RANK_11 ,
                                                 ADC_REGULAR_RANK_12 ,
                                                 ADC_REGULAR_RANK_13 ,
                                                 ADC_REGULAR_RANK_14 ,
                                                 ADC_REGULAR_RANK_15 ,
                                                 ADC_REGULAR_RANK_16 ,                         	
                                               };																														 
																														 
volatile uint16_t SMPADCVALUE1[ADC_MAX_IN_PIN_NUM],SMPADCVALUE2[ADC_MAX_IN_PIN_NUM],SMPADCVALUE3[ADC_MAX_IN_PIN_NUM];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc){
	GPIO_InitTypeDef GPIO_InitStruct[3] = {0};
	GPIO_InitTypeDef GPIO_InitStruct_temp[16]={0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	
	if(hadc->Instance==ADC1)
	{
      /* USER CODE BEGIN ADC1_MspInit 0 */

	    /* USER CODE END ADC1_MspInit 0 */
	    /** Initializes the peripherals clock
	    */
	    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	    PeriphClkInit.PLLSAI1.PLLSAI1Source = ADC_PLL_SOURCE;
	    PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	    PeriphClkInit.PLLSAI1.PLLSAI1N = 10;
	    PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
	    PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	    PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV8;
	    PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
	    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	    {
	        Error_Handler();
	    }

	    /* Peripheral clock enable */
	    __HAL_RCC_ADC_CLK_ENABLE();

	    __HAL_RCC_GPIOA_CLK_ENABLE();
			__HAL_RCC_GPIOB_CLK_ENABLE();
	    __HAL_RCC_GPIOC_CLK_ENABLE();
	    
			bsp_mcu_adc_enable_cnt = 0;
			for(int i=0 ;i < ADC_MAX_IN_PIN_NUM; i++){
			    if((bsp_mcu_adc_config_table[i].gpio_port!=BSP_MCU_ADC_DISABLE) && (bsp_mcu_adc_config_table[i].gpio_pin!=BSP_MCU_ADC_DISABLE)){
			        HalBspSetGpio(bsp_mcu_adc_config_table[i].gpio_port, bsp_mcu_adc_config_table[i].gpio_pin, GPIO_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL, 0);
				      bsp_mcu_adc_enable_cnt++;
			    } 
			}
		
	    /* ADC1 DMA Init */
	    /* ADC1 Init */
	    smp_dma_adc1.Instance = DMA1_Channel1;
	    smp_dma_adc1.Init.Request = DMA_REQUEST_0;
	    smp_dma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
	    smp_dma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
	    smp_dma_adc1.Init.MemInc = DMA_MINC_ENABLE;
	    smp_dma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	    smp_dma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	    smp_dma_adc1.Init.Mode = DMA_CIRCULAR;
 	    smp_dma_adc1.Init.Priority = DMA_PRIORITY_LOW;//DMA_PRIORITY_HIGH;
	    if (HAL_DMA_Init(&smp_dma_adc1) != HAL_OK)
	    {
	        Error_Handler();
	    }

	    __HAL_LINKDMA(hadc,DMA_Handle,smp_dma_adc1);

	    /* USER CODE BEGIN ADC1_MspInit 1 */

    	/* USER CODE END ADC1_MspInit 1 */
	   }	
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){
    #if 0
	  GPIOD->ODR ^= GPIO_PIN_15;
	  #endif
    __NOP();
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
    #if 0
	  GPIOD->ODR ^= GPIO_PIN_14;
	  #endif
	
	  __NOP();
}
void Enable_DMA_Control_Clock(void){
    __HAL_RCC_DMA1_CLK_ENABLE();
	
    #if 0
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    #endif 
}

void DMA1_Channel1_IRQHandler(void){
    #if 0
	  GPIOD->ODR ^= GPIO_PIN_13;
	  #endif
	
    HAL_DMA_IRQHandler(&smp_dma_adc1);
}

int8_t smp_adc_adc_para_init(adc_module_number num){
	Enable_DMA_Control_Clock();
	
	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};	
	
	static uint8_t adc_rank_cnt = 0;
	
	switch(num){
		case adc1:
			   bsp_mcu_adc_enable_cnt = 0;
			   for(int i=0 ;i < ADC_MAX_IN_PIN_NUM; i++){
			       if((bsp_mcu_adc_config_table[i].gpio_port!=BSP_MCU_ADC_DISABLE) && (bsp_mcu_adc_config_table[i].gpio_pin!=BSP_MCU_ADC_DISABLE)){
				          bsp_mcu_adc_enable_cnt++;
			       } 
			  }		
				smp_adc_1.Instance = ADC1;
				smp_adc_1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
				smp_adc_1.Init.Resolution = ADC_RESOLUTION_12B;
				smp_adc_1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
				smp_adc_1.Init.ScanConvMode = ADC_SCAN_ENABLE;
				smp_adc_1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
				smp_adc_1.Init.LowPowerAutoWait = ENABLE;
				smp_adc_1.Init.ContinuousConvMode = ENABLE;
				smp_adc_1.Init.NbrOfConversion = bsp_mcu_adc_enable_cnt;
				smp_adc_1.Init.DiscontinuousConvMode = DISABLE;
				smp_adc_1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
				smp_adc_1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
				smp_adc_1.Init.DMAContinuousRequests = ENABLE;
				smp_adc_1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
				smp_adc_1.Init.OversamplingMode = DISABLE;
		
				if (HAL_ADC_Init(&smp_adc_1) != HAL_OK){
					return SMP_ERROR_NULL;
				}
				multimode.Mode = ADC_MODE_INDEPENDENT;
				if (HAL_ADCEx_MultiModeConfigChannel(&smp_adc_1, &multimode) != HAL_OK)
				{
					return SMP_ERROR_NULL;
				}
				
				// Configure Regular Channel 1~16 (MAX Channel)
				//--------------------------------------------------------------
				adc_rank_cnt = 0;
				for(int i=0 ;i < ADC_MAX_IN_PIN_NUM; i++){
				    if((bsp_mcu_adc_config_table[i].gpio_port!=BSP_MCU_ADC_DISABLE) && (bsp_mcu_adc_config_table[i].gpio_pin!=BSP_MCU_ADC_DISABLE)){	
				        p_adc_cong[adc_rank_cnt] = &bsp_mcu_adc_config_table[i];
							
							  sConfig.Channel = GetADCChParaSTM32L496(adc1, bsp_mcu_adc_config_table[i].gpio_port, bsp_mcu_adc_config_table[i].gpio_pin);
				        sConfig.Rank = adc_sconfig_rank[adc_rank_cnt++];
				        sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
				
							  if(adc_rank_cnt==1){
				            sConfig.SingleDiff = ADC_SINGLE_ENDED;
				            sConfig.OffsetNumber = ADC_OFFSET_NONE;
				            sConfig.Offset = 0;
								}
								
				        if (HAL_ADC_ConfigChannel(&smp_adc_1, &sConfig) != HAL_OK)
				        {
					          return SMP_ERROR_NULL;
				        }
						}
			 }
								
       HAL_ADCEx_Calibration_Start(&smp_adc_1,ADC_SINGLE_ENDED);
       HAL_ADC_Start_DMA(&smp_adc_1,(uint32_t *)SMPADCVALUE1,adc_rank_cnt);
			 break;
		case adc2:
			 break;
		case adc3:
			 break;
		default:
			return SMP_ERROR_NOT_SUPPORTED;
			//break;
	}
	return SMP_SUCCESS;
}

/* 	
	According to ST32L496 datasheet pp.72~pp.78 pin additional function, 
	get the channel parameter when config the ADC module
*/

uint32_t GetADCChParaSTM32L496(adc_module_number num, GPIO_TypeDef *Port, uint16_t Pin){
	
	if((Port == GPIOC)&&(Pin == GPIO_PIN_0)){				    //PC0					//ADC1~ADC3 all available
		return ADC_CHANNEL_1;
	}else if((Port == GPIOC)&&(Pin == GPIO_PIN_1)){		  //PC1
		return ADC_CHANNEL_2;
	}else if((Port == GPIOC)&&(Pin == GPIO_PIN_2)){		  //PC2
		return ADC_CHANNEL_3;
	}else if((Port == GPIOC)&&(Pin == GPIO_PIN_3)){		  //PC3
		return ADC_CHANNEL_4;
	}
	if((num == adc1)||(num == adc2)){
		if((Port == GPIOA)&&(Pin == GPIO_PIN_0)){				  //PA0 			//ADC1, ADC2 are both available
			return ADC_CHANNEL_5;
		}else if((Port == GPIOA)&&(Pin == GPIO_PIN_1)){		//PA1 
			return ADC_CHANNEL_6;
		}else if((Port == GPIOA)&&(Pin == GPIO_PIN_2)){		//PA2 
			return ADC_CHANNEL_7;
		}else if((Port == GPIOA)&&(Pin == GPIO_PIN_3)){		//PA3 
			return ADC_CHANNEL_8;
		}else if((Port == GPIOA)&&(Pin == GPIO_PIN_4)){		//PA4 
			return ADC_CHANNEL_9;
		}else if((Port == GPIOA)&&(Pin == GPIO_PIN_5)){		//PA5 
			return ADC_CHANNEL_10;
		}else if((Port == GPIOA)&&(Pin == GPIO_PIN_6)){		//PA6 
			return ADC_CHANNEL_11;
		}else if((Port == GPIOA)&&(Pin == GPIO_PIN_7)){		//PA7 
			return ADC_CHANNEL_12;
		}else if((Port == GPIOC)&&(Pin == GPIO_PIN_4)){		//PC4 
			return ADC_CHANNEL_13;
		}else if((Port == GPIOC)&&(Pin == GPIO_PIN_5)){		//PC5 
			return ADC_CHANNEL_14;
		}else if((Port == GPIOB)&&(Pin == GPIO_PIN_0)){		//PB0 
			return ADC_CHANNEL_15;
		}else if((Port == GPIOB)&&(Pin == GPIO_PIN_1)){		//PB1 
			return ADC_CHANNEL_16;
		}
	}else if(num == adc3){
		if((Port == GPIOF)&&(Pin == PIN3)){				//PF3 			// Only ADC3 available
			return ADC_CHANNEL_6;
		}else if((Port == GPIOF)&&(Pin == PIN4)){		//PF4 
			return ADC_CHANNEL_7;
		}else if((Port == GPIOF)&&(Pin == PIN5)){		//PF5 
			return ADC_CHANNEL_8;
		}else if((Port == GPIOF)&&(Pin == PIN6)){		//PF6
			return ADC_CHANNEL_9;
		}else if((Port == GPIOF)&&(Pin == PIN7)){		//PF7
			return ADC_CHANNEL_10;
		}else if((Port == GPIOF)&&(Pin == PIN8)){		//PF8
			return ADC_CHANNEL_11;
		}else if((Port == GPIOF)&&(Pin == PIN9)){		//PF9
			return ADC_CHANNEL_12;
		}else if((Port == GPIOF)&&(Pin == PIN10)){		//PF10
			return ADC_CHANNEL_13;
		}	
	}
	return SMP_ERROR_NOT_SUPPORTED;
}

int8_t hal_internal_adc_channel_find(bsp_adc_init_io_config adc_confing){
uint8_t i=0;
    for(i = 0; i< bsp_mcu_adc_enable_cnt; i++){
		    if((adc_confing.gpio_port == p_adc_cong[i]->gpio_port) && (adc_confing.gpio_pin == p_adc_cong[i]->gpio_pin)){
				     return(i);
				}
		}	
		return(SMP_ERROR_NOT_SUPPORTED);
}

int8_t hal_internal_adc_get(uint16_t *adc_data,adc_module_number num, bsp_adc_init_io_config adc_confing){
int8_t	res=SMP_ERROR_NOT_SUPPORTED;
	
	if(num == adc1){
		res = hal_internal_adc_channel_find(adc_confing);
		
		if(res == SMP_ERROR_NOT_SUPPORTED) return(res);
		if(res >=0 && res <bsp_mcu_adc_enable_cnt){
		    *adc_data = SMPADCVALUE1[res];
		}
	}else if(num == adc2){
		if(res == SMP_ERROR_NOT_SUPPORTED) return(res);
		*adc_data = SMPADCVALUE1[res];
	}else if(num == adc3){
		if(res == SMP_ERROR_NOT_SUPPORTED) return(res);
		*adc_data = SMPADCVALUE1[res];
	}
	
	return 0;
}

/* 	
	Deinit ADC 
*/
void smp_adc_deinit(adc_module_number num){
	ADC_HandleTypeDef hadc;
	switch(num){
		case adc1:
			hadc=smp_adc_1;
			break;
		case adc2:
			hadc=smp_adc_2;
			break;
		case adc3:
			hadc=smp_adc_3;
			break;
	}
	HAL_ADC_DeInit(&hadc);
}

/************************ (C) COPYRIGHT Simplo all right reserved *****END OF FILE****/

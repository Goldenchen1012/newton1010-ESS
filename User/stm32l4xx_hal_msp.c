/**
  ******************************************************************************
  * @file    stm32l4xx_hal_msp.c
  * @author  Golden
  * @version V0.0.2
  * @date    2021/12/30
  * @brief   HAL MSP module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include "stm32l4xx_hal.h"
#include "Bsp.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief  Initializes the Global MSP.
  * @param  None
  * @retval None
  */
void HAL_MspInit(void)
{
}

/**
  * @brief  DeInitializes the Global MSP.
  * @param  None  
  * @retval None
  */
void HAL_MspDeInit(void)
{
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{  
	GPIO_InitTypeDef  GPIO_InitStruct;
//	RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
	if(hi2c->Instance == BSP_I2C0){
//		/*##-1- Configure the Discovery SMP I2C0 clock source. The clock is derived from the SYSCLK #*/
//		RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
//		RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
//		HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
		
		
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Enable GPIO TX/RX clock */
		BSP_I2C0_SCL_GPIO_CLK_ENABLE();
		BSP_I2C0_SDA_GPIO_CLK_ENABLE();

		/* Enable SMP I2C0 clock */
		BSP_I2C0_CLK_ENABLE(); 
		/* Force and release SMP I2C0 Peripheral Clock Reset */ 
		BSP_I2C0_FORCE_RESET();
		BSP_I2C0_RELEASE_RESET();		
		
		/*##-2- Configure peripheral GPIO ##########################################*/  
		/* SMP I2C0 TX GPIO pin configuration  */
		GPIO_InitStruct.Pin       = BSP_I2C0_SCL_PIN;		
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;	
		GPIO_InitStruct.Alternate = BSP_I2C0_SCL_AF;

		HAL_GPIO_Init(BSP_I2C0_SCL_GPIO_PORT, &GPIO_InitStruct);

		/* SMP I2C0 RX GPIO pin configuration  */
		GPIO_InitStruct.Pin			= BSP_I2C0_SDA_PIN;	
		GPIO_InitStruct.Alternate 	= BSP_I2C0_SDA_AF;

		HAL_GPIO_Init(BSP_I2C0_SDA_GPIO_PORT, &GPIO_InitStruct);
		
		/*##-3- Configure the NVIC for SMP I2C0 #########################################*/   
		/* NVIC for SMP I2C0 */
		HAL_NVIC_SetPriority(BSP_I2C0_ER_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(BSP_I2C0_ER_IRQn);
		HAL_NVIC_SetPriority(BSP_I2C0_EV_IRQn, 0, 2);
		HAL_NVIC_EnableIRQ(BSP_I2C0_EV_IRQn);
	}
}

/**
  * @brief I2C MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{

	if(hi2c->Instance == BSP_I2C0){	
		/*##-1- Reset peripherals ##################################################*/
		BSP_I2C0_FORCE_RESET();
		BSP_I2C0_RELEASE_RESET();
		/*##-2- Disable peripherals and GPIO Clocks ################################*/
		/* Configure SMP I2C0 Tx as alternate function  */
		HAL_GPIO_DeInit(BSP_I2C0_SCL_GPIO_PORT, BSP_I2C0_SCL_PIN);
		/* Configure SMP I2C0 Rx as alternate function  */
		HAL_GPIO_DeInit(BSP_I2C0_SDA_GPIO_PORT, BSP_I2C0_SDA_PIN);
		
		/*##-3- Disable the NVIC for SMP I2C0 ###########################################*/
		HAL_NVIC_DisableIRQ(BSP_I2C0_ER_IRQn);
		HAL_NVIC_DisableIRQ(BSP_I2C0_EV_IRQn);
	}
}

/**
  * @brief RTC MSP Initialization 
  *        This function configures the hardware resources used in this example
  * @param hrtc: RTC handle pointer
  * 
  * @note  Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to select 
  *        the RTC clock source; in this case the Backup domain will be reset in  
  *        order to modify the RTC Clock source, as consequence RTC registers (including 
  *        the backup registers) and RCC_BDCR register are set to their reset values.
  *             
  * @retval None
  */
#if	0
void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc)
{
	RCC_OscInitTypeDef        RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

	/*##-1- Enables the PWR Clock and Enables access to the backup domain ###################################*/
	/* To change the source clock of the RTC feature (LSE, LSI), You have to:
	 - Enable the power clock using __HAL_RCC_PWR_CLK_ENABLE()
	 - Enable write access using HAL_PWR_EnableBkUpAccess() function before to 
	   configure the RTC clock source (to be done once after reset).
	 - Reset the Back up Domain using __HAL_RCC_BACKUPRESET_FORCE() and 
	   __HAL_RCC_BACKUPRESET_RELEASE().
	 - Configure the needed RTc clock source */
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();

  
	/*##-2- Configue LSE as RTC clock soucre ###################################*/
	RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{ 

	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{ 

	}

	/*##-3- Enable RTC peripheral Clocks #######################################*/
	/* Enable RTC Clock */
	__HAL_RCC_RTC_ENABLE();

	/*##-4- Configure the NVIC for RTC Alarm ###################################*/
	HAL_NVIC_SetPriority(SMP_RTC_WKUP_IRQn, 0x0F, 0);
	HAL_NVIC_EnableIRQ(SMP_RTC_WKUP_IRQn);
}

/**
  * @brief RTC MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  * @param hrtc: RTC handle pointer
  * @retval None
  */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{
	/*##-1- Reset peripherals ##################################################*/
	__HAL_RCC_RTC_DISABLE();

	/*##-2- Disables the PWR Clock and Disables access to the backup domain ###################################*/
	HAL_PWR_DisableBkUpAccess();
	__HAL_RCC_PWR_CLK_DISABLE();

	HAL_NVIC_DisableIRQ(SMP_RTC_WKUP_IRQn);
}

#endif

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == SMP_TIM){
		SMP_TIM_CLK_ENABLE();
		/*##-7- Configure the NVIC for SMP TIM ########################################*/
		/* Set Interrupt Group Priority */ 
		HAL_NVIC_SetPriority(SMP_TIM_IRQn, 7, 0);
		/* Enable the SMP TIM global Interrupt */
		HAL_NVIC_EnableIRQ(SMP_TIM_IRQn);
	}else if(htim->Instance == USER_TIM){
		USER_TIM_CLK_ENABLE();
		/*##-7- Configure the NVIC for User TIM ########################################*/
		/* Set Interrupt Group Priority */ 
		HAL_NVIC_SetPriority(USER_TIM_IRQn, 7, 0);
		/* Enable the User TIM global Interrupt */
		HAL_NVIC_EnableIRQ(USER_TIM_IRQn);
	}else if(htim->Instance == LED_TIM){
		LED_TIM_CLK_ENABLE();
		/*##-7- Configure the NVIC for LED TIM ########################################*/
		/* Set Interrupt Group Priority */ 
		HAL_NVIC_SetPriority(LED_TIM_IRQn, 7, 0);
		/* Enable the LED TIM global Interrupt */
		HAL_NVIC_EnableIRQ(LED_TIM_IRQn);
	}
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim){
	if(htim->Instance == SMP_TIM){
		HAL_NVIC_DisableIRQ(SMP_TIM_IRQn);
		SMP_TIM_CLK_DISABLE();
	}else if(htim->Instance == USER_TIM){
		HAL_NVIC_DisableIRQ(USER_TIM_IRQn);
		USER_TIM_CLK_DISABLE();
	}else if(htim->Instance == LED_TIM){
		HAL_NVIC_DisableIRQ(LED_TIM_IRQn);
		LED_TIM_CLK_DISABLE();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

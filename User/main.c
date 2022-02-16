/**
  ******************************************************************************
  * @file        main.c
  * @author      Golden 
  * @version     v0.0.6
  * @date        2022/02/08
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021</center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "define.h"
#include "main.h"
#include "AppProject.h"
#include "LibSwTimer.h"
#include "HalBsp.h"
#include "ApiFu.h"
#include "smp_TLC6C5912.h"
#include "smp_drv_bq796xx.h"
#include "smp_adc.h"
#include "smp_ADS7946_Driver.h"
#include "ApiIRMonitoring.h"
#include "HalAfeADS7946.h"
#include "ApiModbusTCPIP.h"
#include "AppProjectTest.h"
#include "AppSerialUartDavinci.h"
#include "smp_MX25L_Driver.h"
#include "smp_max7219.h"
#include "smp_log_managment.h"
#include "SEGGER_RTT.h"
#include "RTT_Log.h"

#if 0
#define TEST_APP_PROJECT_FUNC
#include "Test_AppProjectHvEss.h"
#endif


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
																				
/* UNCOMMENT THE FOLLOWING LINE FOR FAST TRANSITION TOWARDS SMPS ACTIVATION */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;
static __IO uint32_t button_pressed = 0;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config_80MHz(void);
void SystemClock_Config_HSE_80MHz(void);
void SystemClock_Config_24MHz (void);

/* Private functions ---------------------------------------------------------*/

static void smp_DMA_Init(void)
{
	/* DMA controller clock enable */
	BSP_SPI1_DMAx_CLK_ENABLE();
	BSP_SPI2_DMAx_CLK_ENABLE();
	BSP_SPI3_DMAx_CLK_ENABLE();
	/* DMA interrupt init */
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BSP_SPI1_DMA_RX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(BSP_SPI1_DMA_RX_IRQn);
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BSP_SPI1_DMA_TX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(BSP_SPI1_DMA_TX_IRQn);
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BSP_SPI2_DMA_RX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(BSP_SPI2_DMA_RX_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BSP_SPI2_DMA_TX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(BSP_SPI2_DMA_TX_IRQn);
	/* DMA2_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BSP_SPI3_DMA_RX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(BSP_SPI3_DMA_RX_IRQn);
	/* DMA2_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BSP_SPI3_DMA_TX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(BSP_SPI3_DMA_TX_IRQn);

}


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  
	uint32_t		del;
  GPIO_InitTypeDef GPIO_InitStructure;

	uint32_t SMPS_status = 0;
	uint32_t resumed_from_standby = 0;
	
	/* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
  */
		
	NVIC->ICER[0]=0xFFFFFFFF;
	NVIC->ICER[1]=0xFFFFFFFF;
	NVIC->ICER[2]=0xFFFFFFFF;
	__enable_irq();
  HalBspInit();
	#if 0
  HAL_Init();
  #endif

  /* Configure the system clock to 80 MHz(HSI / HSE) */
	#if 0
  //SystemClock_Config_80MHz();
	#endif
  SystemClock_Config_HSE_80MHz();
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();	
	__HAL_RCC_RTCAPB_CLK_ENABLE();

  #if 0
  HAL_RCC_MCOConfig( RCC_MCO1 ,RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_16);          //Output SYSCLK to GPIOA PIN8
  #endif
	
	
	GPIO_InitStructure.Pin = GPIO_PIN_13 | GPIO_PIN_14 |GPIO_PIN_15;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure); 

	
	GPIO_InitStructure.Pin = GPIO_PIN_6;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
  //Select goto Test_main code or main code
  //-----------------------------------------------------------------------
	#ifdef TEST_APP_PROJECT_FUNC
  Test_main();	
	#endif
	appProjectOpen();
	LibSwTimerClearCount();
			
  apiFuCheckMagicCode();
		 	
	while(1)
	{		
		LibSwTimerHandle();		
	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 24000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 24000000
  * @param  None
  * @retval None
  */

void SystemClock_Config_24MHz(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Select MSI as system clock source */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI; 
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Update MSI to 24Mhz (RCC_MSIRANGE_9) */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config_80MHz(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

void SystemClock_Config_HSE_80MHz(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}




/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* Suspend tick */
 // HAL_SuspendTick();
  
  /* Turn LED2 */
  //BSP_LED_On(LED2);
  //while (1)
 // {
  ///}
	return;
}

void SysTick_Handler(void)
{
	HAL_IncTick();
}
/**
  * @brief SYSTICK callback
  * @param None
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{

  HAL_IncTick();
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


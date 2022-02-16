/**
  ******************************************************************************
  * @file    smp_spi_DMA.c
  * @author  John Chen/Golden
  * @version V0.0.3
  * @date    2021/12/30
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
#include <string.h>
#include "Bsp.h"
#include "smp_spi_DMA.h"
#include "smp_debug.h"
#include "stm32l4xx_hal.h" 
#include "smp_fifo.h"
#include "smp_gpio.h"
#include "main.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// BSP_SPI1~3
SPI_HandleTypeDef smp_spi1_handle;
SPI_HandleTypeDef smp_spi2_handle;
SPI_HandleTypeDef smp_spi3_handle;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static smp_spi_event_t spi1_evt_cb = 0;
static smp_spi_event_t spi2_evt_cb = 0;
static smp_spi_event_t spi3_evt_cb = 0;
static DMA_HandleTypeDef hdma_tx;
static DMA_HandleTypeDef hdma_rx;

smp_spi_cs_t CS[3];
typedef enum{
	SPI_Both = 0,												
	SPI_Transfer_only,														
	SPI_Recived_only,
	SPI_Done	
}spi_flag;

uint8_t uSPIFlag[3] = {SPI_Done,SPI_Done,SPI_Done};

int8_t smp_spi_get_status(smp_spi_t *spi)
{
	SPI_HandleTypeDef temp_spi_handle;
	if( spi->num == SPI_module1){
		temp_spi_handle = smp_spi1_handle;
	}else if( spi->num == SPI_module2){
		temp_spi_handle = smp_spi2_handle;
	}else if( spi->num == SPI_module3){
		temp_spi_handle = smp_spi3_handle;
	}else{
		return SMP_SPI_EVENT_TRANSFERR_ERROR;	
	}
	if(temp_spi_handle.State ==	HAL_SPI_STATE_READY){
		return SMP_SPI_EVENT_DONE;
	}
	return SMP_SPI_EVENT_TRANSFER_BUSY;
}

int8_t smp_spi_master_init(smp_spi_t *p_spi, smp_spi_event_t smp_spi_event_handler, const bool lsb)
{
	uSPIFlag[p_spi->num] = SPI_Done;
	switch(p_spi->num){
		case SPI_module1:
			__SPI1_CLK_ENABLE();
			smp_spi1_handle.Instance               = BSP_SPI1;
			smp_spi1_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; 
			smp_spi1_handle.Init.Direction         = SPI_DIRECTION_2LINES;
			switch(p_spi->mode){
				case SPI_mode0:
					smp_spi1_handle.Init.CLKPolarity  	= SPI_POLARITY_LOW;
					smp_spi1_handle.Init.CLKPhase       = SPI_PHASE_1EDGE;  
					break;
				case SPI_mode1:
					smp_spi1_handle.Init.CLKPolarity  	= SPI_POLARITY_LOW; 
					smp_spi1_handle.Init.CLKPhase       = SPI_PHASE_2EDGE;  
					break;
				case SPI_mode2:
					smp_spi1_handle.Init.CLKPolarity  	= SPI_POLARITY_HIGH; 
					smp_spi1_handle.Init.CLKPhase       = SPI_PHASE_1EDGE; 
					break;
				case SPI_mode3:
					smp_spi1_handle.Init.CLKPolarity  	= SPI_POLARITY_HIGH; 
					smp_spi1_handle.Init.CLKPhase       = SPI_PHASE_2EDGE; 					
					break;				
				default:
					return SMP_ERROR_INVALID_PARAM;
			}
			smp_spi1_handle.Init.DataSize          = SPI_DATASIZE_8BIT;
			if(lsb == true)
				smp_spi1_handle.Init.FirstBit      = SPI_FIRSTBIT_LSB;
			else	
				smp_spi1_handle.Init.FirstBit      = SPI_FIRSTBIT_MSB;
			smp_spi1_handle.Init.TIMode            = SPI_TIMODE_DISABLE;
			smp_spi1_handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
			smp_spi1_handle.Init.CRCPolynomial     = 7;
			smp_spi1_handle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
			smp_spi1_handle.Init.NSS               = SPI_NSS_SOFT;
			smp_spi1_handle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;

			smp_spi1_handle.Init.Mode = SPI_MODE_MASTER;

			if(HAL_SPI_Init(&smp_spi1_handle) != HAL_OK)
			{
			/* Initialization Error */
				return SMP_ERROR_NOT_FOUND;
			}
			spi1_evt_cb = smp_spi_event_handler;
			break;
		case SPI_module2:
			__SPI2_CLK_ENABLE();
			smp_spi2_handle.Instance               = BSP_SPI2;
			smp_spi2_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; 
			smp_spi2_handle.Init.Direction         = SPI_DIRECTION_2LINES;
			switch(p_spi->mode){
				case SPI_mode0:
					smp_spi2_handle.Init.CLKPolarity  	= SPI_POLARITY_LOW;
					smp_spi2_handle.Init.CLKPhase       = SPI_PHASE_1EDGE;  
					break;
				case SPI_mode1:
					smp_spi2_handle.Init.CLKPolarity  	= SPI_POLARITY_LOW; 
					smp_spi2_handle.Init.CLKPhase       = SPI_PHASE_2EDGE;  
					break;
				case SPI_mode2:
					smp_spi2_handle.Init.CLKPolarity  	= SPI_POLARITY_HIGH; 
					smp_spi2_handle.Init.CLKPhase       = SPI_PHASE_1EDGE; 
					break;
				case SPI_mode3:
					smp_spi2_handle.Init.CLKPolarity  	= SPI_POLARITY_HIGH; 
					smp_spi2_handle.Init.CLKPhase       = SPI_PHASE_2EDGE; 					
					break;				
				default:
					return SMP_ERROR_INVALID_PARAM;
			}
			smp_spi2_handle.Init.DataSize          = SPI_DATASIZE_8BIT;
			if(lsb == true)
				smp_spi2_handle.Init.FirstBit      = SPI_FIRSTBIT_LSB;
			else	
				smp_spi2_handle.Init.FirstBit      = SPI_FIRSTBIT_MSB;
			smp_spi2_handle.Init.TIMode            = SPI_TIMODE_DISABLE;
			smp_spi2_handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
			smp_spi2_handle.Init.CRCPolynomial     = 7;
			smp_spi2_handle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
			smp_spi2_handle.Init.NSS               = SPI_NSS_SOFT;
			smp_spi2_handle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;

			smp_spi2_handle.Init.Mode = SPI_MODE_MASTER;

			if(HAL_SPI_Init(&smp_spi2_handle) != HAL_OK)
			{
			/* Initialization Error */
				return SMP_ERROR_NOT_FOUND;
			}
			spi2_evt_cb = smp_spi_event_handler;
			break;
		case SPI_module3:
			__SPI3_CLK_ENABLE();
			smp_spi3_handle.Instance               = BSP_SPI3;
			smp_spi3_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; 
			smp_spi3_handle.Init.Direction         = SPI_DIRECTION_2LINES;
			switch(p_spi->mode){
				case SPI_mode0:
					smp_spi3_handle.Init.CLKPolarity  	= SPI_POLARITY_LOW;
					smp_spi3_handle.Init.CLKPhase       = SPI_PHASE_1EDGE;  
					break;
				case SPI_mode1:
					smp_spi3_handle.Init.CLKPolarity  	= SPI_POLARITY_LOW; 
					smp_spi3_handle.Init.CLKPhase       = SPI_PHASE_2EDGE;  
					break;
				case SPI_mode2:
					smp_spi3_handle.Init.CLKPolarity  	= SPI_POLARITY_HIGH; 
					smp_spi3_handle.Init.CLKPhase       = SPI_PHASE_1EDGE; 
					break;
				case SPI_mode3:
					smp_spi3_handle.Init.CLKPolarity  	= SPI_POLARITY_HIGH; 
					smp_spi3_handle.Init.CLKPhase       = SPI_PHASE_2EDGE; 					
					break;				
				default:
					return SMP_ERROR_INVALID_PARAM;
			}
			smp_spi3_handle.Init.DataSize          = SPI_DATASIZE_8BIT;
			if(lsb == true)
				smp_spi3_handle.Init.FirstBit      = SPI_FIRSTBIT_LSB;
			else	
				smp_spi3_handle.Init.FirstBit      = SPI_FIRSTBIT_MSB;
			smp_spi3_handle.Init.TIMode            = SPI_TIMODE_DISABLE;
			smp_spi3_handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
			smp_spi3_handle.Init.CRCPolynomial     = 7;
			smp_spi3_handle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
			smp_spi3_handle.Init.NSS               = SPI_NSS_SOFT;
			smp_spi3_handle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;

			smp_spi3_handle.Init.Mode = SPI_MODE_MASTER;

			if(HAL_SPI_Init(&smp_spi3_handle) != HAL_OK)
			{
			/* Initialization Error */
				return SMP_ERROR_NOT_FOUND;
			}
			spi3_evt_cb = smp_spi_event_handler;
			break;
		default:
			/* SPI number unknown */
			return SMP_ERROR_INVALID_PARAM;
	}
	return SMP_SUCCESS;	
}

int8_t smp_spi_master_deinit(smp_spi_t *p_spi)
{
	switch(p_spi->num){
		case SPI_module1:
			if(smp_spi1_handle.Instance != 0){
				if(HAL_SPI_DeInit(&smp_spi1_handle) != HAL_OK)
				{
					/* Deinitialization Error */
					return SMP_ERROR_NOT_FOUND;
				}
				smp_spi1_handle.Instance = 0;
			}
			break;
		case SPI_module2:
			if(smp_spi2_handle.Instance != 0){
				if(HAL_SPI_DeInit(&smp_spi2_handle) != HAL_OK)
				{
					/* Deinitialization Error */
					return SMP_ERROR_NOT_FOUND;
				}
				smp_spi2_handle.Instance = 0;
			}
			break;
		case SPI_module3:
			if(smp_spi3_handle.Instance != 0){
				if(HAL_SPI_DeInit(&smp_spi3_handle) != HAL_OK)
				{
					/* Deinitialization Error */
					return SMP_ERROR_NOT_FOUND;
				}
				smp_spi3_handle.Instance = 0;
			}
			break;
		default:
			/* SPI number unknown */
			return SMP_ERROR_NOT_SUPPORTED;
	}
	return SMP_SUCCESS;	
}

int8_t smp_spi_master_cs_init(smp_spi_cs_t *p_cs)
{		
	p_cs->cs_handler.mode = SMP_GPIO_MODE_OUTPUT_OD;
	
	if(smp_gpio_init(&p_cs->cs_handler) != HAL_OK){
		return SMP_ERROR_NOT_FOUND;
	}
	smp_gpio_set_state(&p_cs->cs_handler, GPIO_ACTIVE_HIGH);
	return SMP_SUCCESS;
}

int8_t smp_spi_master_cs_deinit(smp_spi_cs_t *p_cs)
{	
	if(smp_gpio_deinit(&p_cs->cs_handler) != HAL_OK){
		return SMP_ERROR_NOT_FOUND;
	}
	return SMP_SUCCESS;
}

int8_t smp_spi_master_cs_set(smp_spi_cs_t *p_cs, uint8_t status)
{
	int8_t temp_status;
	if(status == GPIO_ACTIVE_LOW){
		temp_status = smp_gpio_set_state(&p_cs->cs_handler, GPIO_ACTIVE_LOW);
		if(p_cs->spi_num == SPI_module1){
			spi1_evt_cb(SMP_SPI_EVENT_TRANSFER_BUSY);
		}else if(p_cs->spi_num == SPI_module2){
			spi2_evt_cb(SMP_SPI_EVENT_TRANSFER_BUSY);
		}else if(p_cs->spi_num == SPI_module3){
			spi3_evt_cb(SMP_SPI_EVENT_TRANSFER_BUSY);
		}
		return temp_status;		
	}else if(status == GPIO_ACTIVE_HIGH){
		temp_status = smp_gpio_set_state(&p_cs->cs_handler, GPIO_ACTIVE_HIGH);
		uSPIFlag[p_cs->spi_num] = SPI_Done;
		if(p_cs->spi_num == SPI_module1){
			spi1_evt_cb(SMP_SPI_EVENT_DONE);
		}else if(p_cs->spi_num == SPI_module2){
			spi2_evt_cb(SMP_SPI_EVENT_DONE);
		}else if(p_cs->spi_num == SPI_module3){
			spi3_evt_cb(SMP_SPI_EVENT_DONE);
		}
		return temp_status;
	}
	return SMP_ERROR_NOT_FOUND;
}
uint8_t * g_rx_data[3];
uint16_t g_rx_size[3];
int8_t smp_spi_master_send_recv(smp_spi_t *spi, uint8_t *tx_data,uint16_t tx_size, uint8_t *rx_data, uint16_t rx_size,smp_spi_cs_t *p_cs)
{
	HAL_StatusTypeDef status;
	SPI_HandleTypeDef temp_spi_handle;
	
	if(uSPIFlag[spi->num] != SPI_Done){
		return SMP_ERROR_BUSY;
	}
	smp_spi_master_cs_set(p_cs,GPIO_ACTIVE_LOW);
	CS[spi->num] = *p_cs;
	
	if( spi->num == SPI_module1){
		temp_spi_handle = smp_spi1_handle;
	}else if( spi->num == SPI_module2){
		temp_spi_handle = smp_spi2_handle;
	}else if( spi->num == SPI_module3){
		temp_spi_handle = smp_spi3_handle;
	}else{
		return SMP_ERROR_NOT_SUPPORTED;	
	}
	if(tx_size == 0){
		uSPIFlag[spi->num] = SPI_Recived_only;
	}else if(rx_size == 0){
		uSPIFlag[spi->num] = SPI_Transfer_only;
	}else{
		uSPIFlag[spi->num] = SPI_Both;
	}
	if(rx_size != 0){
		g_rx_data[spi->num] = (uint8_t *)rx_data;
		g_rx_size[spi->num] = rx_size;
		if(uSPIFlag[spi->num]==SPI_Recived_only){
			status = HAL_SPI_Receive_DMA(&temp_spi_handle, (uint8_t *)rx_data, rx_size);
		}	
	}
	if(tx_size != 0){	
		status = HAL_SPI_Transmit_DMA(&temp_spi_handle, (uint8_t *)tx_data, tx_size);
	}

	if(status!=HAL_OK){
		return SMP_ERROR_NOT_FOUND;
	}
	return SMP_SUCCESS;	
}

uint8_t smp_spi_master_is_spi_ready(smp_spi_t *spi)
{
	if(uSPIFlag[spi->num] != SPI_Done)
		return 0;
	return 1;
}

int8_t smp_spi_master_send_recv_blocking(smp_spi_t *spi, uint8_t *tx_data,uint16_t tx_size, uint8_t *rx_data, uint16_t rx_size,smp_spi_cs_t *p_cs)
{
	HAL_StatusTypeDef status;
	SPI_HandleTypeDef temp_spi_handle;
	//uint8_t temp_spi_cnt = 0;
	smp_spi_master_cs_set(p_cs,GPIO_ACTIVE_LOW);
	#if	0
	while (uSPIFlag[spi->num] != SPI_Done) {
		HAL_Delay(10);
		temp_spi_cnt++;
		if(temp_spi_cnt >= 10){
			return SMP_ERROR_BUSY;
		}
	}
	#endif
	CS[spi->num] = *p_cs;
	
	if( spi->num == SPI_module1){
		temp_spi_handle = smp_spi1_handle;
	}else if( spi->num == SPI_module2){
		temp_spi_handle = smp_spi2_handle;
	}else if( spi->num == SPI_module3){
		temp_spi_handle = smp_spi3_handle;
	}else{
		return SMP_ERROR_NOT_SUPPORTED;	
	}
	
	if(tx_size != 0){	
		status = HAL_SPI_Transmit(&temp_spi_handle, (uint8_t *)tx_data, tx_size,100);
	}
	if(rx_size != 0){
		status = HAL_SPI_Receive(&temp_spi_handle, (uint8_t *)rx_data, rx_size,100);
	}
	smp_spi_master_cs_set(p_cs,GPIO_ACTIVE_HIGH);
	if(status!=HAL_OK){
		return SMP_ERROR_NOT_FOUND;
	}else{
		return SMP_SUCCESS;	
	}
}
/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == BSP_SPI1){
		if(spi1_evt_cb!=NULL)
			spi1_evt_cb(SMP_SPI_EVENT_DONE);
	}
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *h_spi)
{
  if(h_spi->Instance == BSP_SPI1){
		spi1_evt_cb(SMP_SPI_EVENT_TRANSFERR_ERROR);
		
	}else if(h_spi->Instance == BSP_SPI2){
		spi2_evt_cb(SMP_SPI_EVENT_TRANSFERR_ERROR);
		
	}else if(h_spi->Instance == BSP_SPI3){
		spi3_evt_cb(SMP_SPI_EVENT_TRANSFERR_ERROR);
	}
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */
	if(hdma_spi1_rx.State == HAL_SPI_STATE_READY){	
		smp_spi_master_cs_set(&CS[SPI_module1],GPIO_ACTIVE_HIGH);			
	}

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */
	if(hdma_spi1_tx.State == HAL_SPI_STATE_READY){
		if(uSPIFlag[SPI_module1] == SPI_Transfer_only){
			smp_spi_master_cs_set(&CS[SPI_module1],GPIO_ACTIVE_HIGH);
		}else if(uSPIFlag[SPI_module1] == SPI_Both){
		
			HAL_SPI_Receive_DMA(&smp_spi1_handle, (uint8_t *)g_rx_data[SPI_module1], g_rx_size[SPI_module1]);
		}
		
	}
	 
  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */
	if(hdma_spi2_rx.State == HAL_SPI_STATE_READY){	
			smp_spi_master_cs_set(&CS[SPI_module2],GPIO_ACTIVE_HIGH);		
		}
  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */
	if(hdma_spi2_tx.State == HAL_SPI_STATE_READY){
		if(uSPIFlag[SPI_module2] == SPI_Transfer_only){
			smp_spi_master_cs_set(&CS[SPI_module2],GPIO_ACTIVE_HIGH);
		}else if(uSPIFlag[SPI_module2] == SPI_Both){
			HAL_SPI_Receive_DMA(&smp_spi2_handle, (uint8_t *)g_rx_data[SPI_module2], g_rx_size[SPI_module2]);
		}
		
	}
  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel1 global interrupt.
  */
void DMA2_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel1_IRQn 0 */

  /* USER CODE END DMA2_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_rx);
  /* USER CODE BEGIN DMA2_Channel1_IRQn 1 */
	if(hdma_spi3_rx.State == HAL_SPI_STATE_READY){	
			smp_spi_master_cs_set(&CS[SPI_module3],GPIO_ACTIVE_HIGH);		
		}
  /* USER CODE END DMA2_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel2 global interrupt.
  */
void DMA2_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel2_IRQn 0 */

  /* USER CODE END DMA2_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
  /* USER CODE BEGIN DMA2_Channel2_IRQn 1 */
	if(hdma_spi3_tx.State == HAL_SPI_STATE_READY){
		if(uSPIFlag[SPI_module3] == SPI_Transfer_only){
			smp_spi_master_cs_set(&CS[SPI_module3],GPIO_ACTIVE_HIGH);
		}else if(uSPIFlag[SPI_module3] == SPI_Both){
			HAL_SPI_Receive_DMA(&smp_spi3_handle, (uint8_t *)g_rx_data[SPI_module3], g_rx_size[SPI_module3]);
		}
			
		}
  /* USER CODE END DMA2_Channel2_IRQn 1 */
}
/**
  * @}
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI1)
  {
    /* USER CODE BEGIN SPI1_MspInit 0 */
	  BSP_SPI1_DMAx_CLK_ENABLE();
	  /* DMA1_Channel2_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(BSP_SPI1_DMA_RX_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(BSP_SPI1_DMA_RX_IRQn);
	  /* DMA1_Channel3_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(BSP_SPI1_DMA_TX_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(BSP_SPI1_DMA_TX_IRQn);
    /* USER CODE END SPI1_MspInit 0 */
    /* Peripheral clock enable */
    BSP_SPI1_CLK_ENABLE();

    BSP_SPI1_SCK_GPIO_CLK_ENABLE();
	  BSP_SPI1_MISO_GPIO_CLK_ENABLE();
	  BSP_SPI1_MOSI_GPIO_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = BSP_SPI1_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BSP_SPI1_SCK_AF;
    HAL_GPIO_Init(BSP_SPI1_SCK_GPIO_PORT, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = BSP_SPI1_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BSP_SPI1_MISO_AF;
    HAL_GPIO_Init(BSP_SPI1_MISO_GPIO_PORT, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = BSP_SPI1_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BSP_SPI1_MOSI_AF;
    HAL_GPIO_Init(BSP_SPI1_MOSI_GPIO_PORT, &GPIO_InitStruct);

    /* SPI1 DMA Init */
    /* SPI1_TX Init */
    hdma_spi1_tx.Instance = BSP_SPI1_TX_DMA_CHANNEL;
    hdma_spi1_tx.Init.Request = BSP_SPI1_TX_DMA_REQUEST;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmatx,hdma_spi1_tx);

    /* SPI1_RX Init */
    hdma_spi1_rx.Instance = BSP_SPI1_RX_DMA_CHANNEL;
    hdma_spi1_rx.Init.Request = BSP_SPI1_RX_DMA_REQUEST;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmarx,hdma_spi1_rx);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  else if(hspi->Instance==SPI2)
  {
    /* USER CODE BEGIN SPI2_MspInit 0 */
	  BSP_SPI2_DMAx_CLK_ENABLE();
	  /* DMA1_Channel4_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(BSP_SPI2_DMA_RX_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(BSP_SPI2_DMA_RX_IRQn);
	  /* DMA1_Channel5_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(BSP_SPI2_DMA_TX_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(BSP_SPI2_DMA_TX_IRQn);
    /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    BSP_SPI2_CLK_ENABLE();
	  BSP_SPI2_SCK_GPIO_CLK_ENABLE();
	  BSP_SPI2_MISO_GPIO_CLK_ENABLE();
	  BSP_SPI2_MOSI_GPIO_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PC1     ------> SPI2_MOSI
    PC2     ------> SPI2_MISO
    PB10     ------> SPI2_SCK
    */
    GPIO_InitStruct.Pin = BSP_SPI2_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BSP_SPI2_SCK_AF;
    HAL_GPIO_Init(BSP_SPI2_SCK_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BSP_SPI2_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BSP_SPI2_MISO_AF;
    HAL_GPIO_Init(BSP_SPI2_MISO_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BSP_SPI2_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BSP_SPI2_MOSI_AF;
    HAL_GPIO_Init(BSP_SPI2_MOSI_GPIO_PORT, &GPIO_InitStruct);

    /* SPI2 DMA Init */
    /* SPI2_TX Init */
    hdma_spi2_tx.Instance = BSP_SPI2_TX_DMA_CHANNEL;
    hdma_spi2_tx.Init.Request = BSP_SPI2_TX_DMA_REQUEST;
    hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi2_tx.Init.Mode = DMA_NORMAL;
    hdma_spi2_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmatx,hdma_spi2_tx);

    /* SPI2_RX Init */
    hdma_spi2_rx.Instance = BSP_SPI2_RX_DMA_CHANNEL;
    hdma_spi2_rx.Init.Request = BSP_SPI2_RX_DMA_REQUEST;
    hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi2_rx.Init.Mode = DMA_NORMAL;
    hdma_spi2_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmarx,hdma_spi2_rx);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
  else if(hspi->Instance==SPI3)
  {
	  /* USER CODE BEGIN SPI3_MspInit 0 */
	  BSP_SPI3_DMAx_CLK_ENABLE();
	  /* DMA2_Channel1_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(BSP_SPI3_DMA_RX_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(BSP_SPI3_DMA_RX_IRQn);
	  /* DMA2_Channel2_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(BSP_SPI3_DMA_TX_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(BSP_SPI3_DMA_TX_IRQn);
	  /* USER CODE END SPI3_MspInit 0 */
    /* Peripheral clock enable */
    BSP_SPI3_CLK_ENABLE();

	  BSP_SPI3_SCK_GPIO_CLK_ENABLE();
	  BSP_SPI3_MISO_GPIO_CLK_ENABLE();
	  BSP_SPI3_MOSI_GPIO_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = BSP_SPI3_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BSP_SPI3_SCK_AF;
    HAL_GPIO_Init(BSP_SPI3_SCK_GPIO_PORT, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = BSP_SPI3_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BSP_SPI3_MISO_AF;
    HAL_GPIO_Init(BSP_SPI3_MISO_GPIO_PORT, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = BSP_SPI3_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BSP_SPI3_MOSI_AF;
    HAL_GPIO_Init(BSP_SPI3_MOSI_GPIO_PORT, &GPIO_InitStruct);

    /* SPI3 DMA Init */
    /* SPI3_TX Init */
    hdma_spi3_tx.Instance = BSP_SPI3_TX_DMA_CHANNEL;
    hdma_spi3_tx.Init.Request = BSP_SPI3_TX_DMA_REQUEST;
    hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_tx.Init.Mode = DMA_NORMAL;
    hdma_spi3_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmatx,hdma_spi3_tx);

    /* SPI3_RX Init */
    hdma_spi3_rx.Instance = BSP_SPI3_RX_DMA_CHANNEL;
    hdma_spi3_rx.Init.Request = BSP_SPI3_RX_DMA_REQUEST;
    hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_rx.Init.Mode = DMA_NORMAL;
    hdma_spi3_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmarx,hdma_spi3_rx);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }

}

/************************ (C) COPYRIGHT Simplo all right reserved *****END OF FILE****/


/**
  ******************************************************************************
  * @file    smp_uart.c
  * @author  Golden Chen
  * @version V0.0.3
  * @date    2022/01/07
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 </center></h2>
  *
  * 
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Bsp.h"
#include "smp_uart.h"
#include "smp_debug.h"
#include "smp_fifo.h"
#include "smp_gpio.h"
#include "SEGGER_RTT.h"
#include "RTT_Log.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// smp uart 0
UART_HandleTypeDef 				smp_uart0_handle = {0};
smp_uart_event_t 					uart0_evt_cb = 0;
smp_fifo_t 								uart0_rx_fifo = {0};
smp_fifo_t 								uart0_tx_fifo = {0};
uint8_t 									uart0_tx_buffer[UART0_TX_BUFFER_SIZE] = {0};
uint8_t 									uart0_rx_buffer[UART0_RX_BUFFER_SIZE] = {0};

// smp uart 1
UART_HandleTypeDef 				smp_uart1_handle = {0};
smp_uart_event_t 					uart1_evt_cb = 0;
smp_fifo_t 								uart1_rx_fifo = {0};
smp_fifo_t 								uart1_tx_fifo = {0};
uint8_t 									uart1_tx_buffer[UART1_TX_BUFFER_SIZE] = {0};
uint8_t 									uart1_rx_buffer[UART1_RX_BUFFER_SIZE] = {0};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

extern smp_gpio_t 						gpio_test_p;
extern smp_gpio_state					gpio_test_p_state;	

int8_t uart_fifo_init(smp_fifo_t * p_fifo, uint8_t * p_buf, uint16_t buf_size)
{
    // Check buffer for null pointer.
    if (p_buf == NULL){
        return SMP_ERROR_NULL;
    }

    p_fifo->buffer_addr   = (char *)p_buf;
    p_fifo->buffer_size 	= buf_size - 1;
    p_fifo->in      			= 0;
    p_fifo->out     			= 0;

    return SMP_SUCCESS;
}

int8_t smp_uart_init(smp_uart_t *p_uart, smp_uart_event_t smp_uart_event_handler)
{
	if(p_uart->num == UART0){
		smp_uart0_handle.Instance        = BSP_UART0;
		smp_uart0_handle.Init.BaudRate   = p_uart->baud_rate;

		smp_uart0_handle.Init.WordLength = UART_WORDLENGTH_8B;
		smp_uart0_handle.Init.StopBits   = UART_STOPBITS_1;
		if(p_uart->use_parity == PARITY_NONE){
			smp_uart0_handle.Init.Parity     = UART_PARITY_NONE;
		}else if(p_uart->use_parity == PARITY_EVEN){
			smp_uart0_handle.Init.Parity     = UART_PARITY_EVEN;
		}else if(p_uart->use_parity == PARITY_ODD){
			smp_uart0_handle.Init.Parity     = UART_PARITY_ODD;
		}else{
			return SMP_ERROR_INVALID_PARAM;
		}
		if(p_uart->flow_ctrl == UART_FLOW_CTRL_DISABLE){
			smp_uart0_handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
		}else if(p_uart->flow_ctrl == UART_FLOW_CTRL_ENABLE){
			smp_uart0_handle.Init.HwFlowCtl  = UART_HWCONTROL_RTS_CTS;
		}else{
			return SMP_ERROR_INVALID_PARAM;
		}
		
    //Golden Add 2022.01.07		
		smp_uart0_handle.Init.OverSampling = UART_OVERSAMPLING_16;	
    smp_uart0_handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    smp_uart0_handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
			
		smp_uart0_handle.Init.Mode       = UART_MODE_TX_RX;
		if(HAL_UART_DeInit(&smp_uart0_handle) != HAL_OK){
			return SMP_ERROR_NOT_FOUND;
		}  
		if(HAL_UART_Init(&smp_uart0_handle) != HAL_OK){
			return SMP_ERROR_NOT_FOUND;
		}
		uart0_evt_cb = smp_uart_event_handler;
		// Configure buffer RX buffer.
		uart_fifo_init(&uart0_rx_fifo, p_uart->buffers.rx_buf, p_uart->buffers.rx_buf_size);
		// Configure buffer TX buffer.
		uart_fifo_init(&uart0_tx_fifo, p_uart->buffers.tx_buf, p_uart->buffers.tx_buf_size);

		if(HAL_UART_Receive_DMA(&smp_uart0_handle, uart0_rx_buffer, UART0_RX_BUFFER_SIZE) != HAL_OK){
				return SMP_ERROR_NOT_FOUND;
		}
	}else	if(p_uart->num == UART1){
		smp_uart1_handle.Instance        = BSP_UART1;
		smp_uart1_handle.Init.BaudRate   = p_uart->baud_rate;

		smp_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;
		smp_uart1_handle.Init.StopBits   = UART_STOPBITS_1;
		
		if(p_uart->use_parity == PARITY_NONE){
			smp_uart1_handle.Init.Parity     = UART_PARITY_NONE;
		}else if(p_uart->use_parity == PARITY_EVEN){
			smp_uart1_handle.Init.Parity     = UART_PARITY_EVEN;
		}else if(p_uart->use_parity == PARITY_ODD){
			smp_uart1_handle.Init.Parity     = UART_PARITY_ODD;
		}else{
			return SMP_ERROR_INVALID_PARAM;
		}
		if(p_uart->flow_ctrl == UART_FLOW_CTRL_DISABLE){
			smp_uart1_handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
		}else if(p_uart->flow_ctrl == UART_FLOW_CTRL_ENABLE){
			smp_uart1_handle.Init.HwFlowCtl  = UART_HWCONTROL_RTS_CTS;
		}else{
			return SMP_ERROR_INVALID_PARAM;
		}
			
		smp_uart1_handle.Init.Mode       = UART_MODE_TX_RX;
		if(HAL_UART_DeInit(&smp_uart1_handle) != HAL_OK){
			return SMP_ERROR_NOT_FOUND;
		}  
		if(HAL_UART_Init(&smp_uart1_handle) != HAL_OK){
			return SMP_ERROR_NOT_FOUND;
		}
		uart1_evt_cb = smp_uart_event_handler;
		
		// Configure buffer RX buffer.
		uart_fifo_init(&uart1_rx_fifo, p_uart->buffers.rx_buf, p_uart->buffers.rx_buf_size);
		// Configure buffer TX buffer.
		uart_fifo_init(&uart1_tx_fifo, p_uart->buffers.tx_buf, p_uart->buffers.tx_buf_size);
		
		HAL_UART_Receive_IT(&smp_uart1_handle, uart1_rx_buffer, UART1_RX_BUFFER_SIZE);

		#if 0
		if(HAL_UART_Receive_DMA(&smp_uart1_handle, uart1_rx_buffer, UART3_RX_BUFFER_SIZE) != HAL_OK){
				return SMP_ERROR_NOT_FOUND;
		}
		#endif
	 }else{
		return SMP_ERROR_NOT_SUPPORTED;
	 }		
	 return SMP_SUCCESS;
}

int8_t smp_uart_deinit(smp_uart_t *p_uart)
{
	if(p_uart->num == UART0){
		if(smp_uart0_handle.Instance != 0){
			if(HAL_UART_DeInit(&smp_uart0_handle) != HAL_OK)
				return SMP_ERROR_NOT_FOUND;
			smp_uart0_handle.Instance = 0;
		}
	}else if(p_uart->num == UART1){
		if(smp_uart1_handle.Instance != 0){
			if(HAL_UART_DeInit(&smp_uart1_handle) != HAL_OK)
				return SMP_ERROR_NOT_FOUND;
			smp_uart1_handle.Instance = 0;
		}
	}		
	return SMP_SUCCESS;
}

int8_t smp_uart_put(smp_uart_t *p_uart, uint8_t byte)
{
	uint16_t size = 0, i =0;
	if(p_uart->num == UART0){
        if (smp_fifo_push(&uart0_tx_fifo, byte) == SMP_SUCCESS){
        // The new byte has been added to FIFO. It will be picked up from there
        // (in 'uart_event_handler') when all preceding bytes are transmitted.
        // But if UART is not transmitting anything at the moment, we must start
        // a new transmission here.
            if((smp_uart0_handle.gState == HAL_UART_STATE_READY)||(smp_uart0_handle.gState == HAL_UART_STATE_BUSY_RX)){
				        if(smp_fifo_get_size(&uart0_tx_fifo,&size)==SMP_SUCCESS){
                    if(size>UART0_TX_BUFFER_SIZE)	size = UART0_TX_BUFFER_SIZE;
						        // This operation should be almost always successful, since we've
						        // just added a byte to FIFO, but if some bigger delay occurred
						        // (some heavy interrupt handler routine has been executed) since
						        // that time, FIFO might be empty already.
									  if(uart0_evt_cb)
				                uart0_evt_cb(UART_TX_READY_TO_SEND);
									
						        for(i=0;i<size;i++){	
							          smp_fifo_pop(&uart0_tx_fifo, (char *)&uart0_tx_buffer[i]);
						        }
						        HAL_UART_Transmit_DMA(&smp_uart0_handle, &uart0_tx_buffer[0], size);
			         }
           }
	     }
	}else if(p_uart->num == UART1){
        if (smp_fifo_push(&uart1_tx_fifo, byte) == SMP_SUCCESS){
        // The new byte has been added to FIFO. It will be picked up from there
        // (in 'uart_event_handler') when all preceding bytes are transmitted.
        // But if UART is not transmitting anything at the moment, we must start
        // a new transmission here.
			      if((smp_uart1_handle.gState == HAL_UART_STATE_READY)||(smp_uart1_handle.gState == HAL_UART_STATE_BUSY_RX)){
				        if(smp_fifo_get_size(&uart1_tx_fifo,&size)==SMP_SUCCESS){
					          if(size>UART1_TX_BUFFER_SIZE) size = UART1_TX_BUFFER_SIZE;
						        // This operation should be almost always successful, since we've
						        // just added a byte to FIFO, but if some bigger delay occurred
						        // (some heavy interrupt handler routine has been executed) since
						        // that time, FIFO might be empty already.
										if(uart1_evt_cb)
				                uart1_evt_cb(UART_TX_READY_TO_SEND);
										
						        for(i=0;i<size;i++){	
							          smp_fifo_pop(&uart1_tx_fifo, (char *)&uart1_tx_buffer[i]);
						        }
						        //HAL_UART_Transmit_DMA(&smp_uart1_handle, &uart1_tx_buffer[0], size);
								HAL_UART_Transmit_IT(&smp_uart1_handle, &uart1_tx_buffer[0], size);
			           }
            }	
	       }
	}else{
		return SMP_ERROR_FULL;
	}
	
	return SMP_SUCCESS;
}

int8_t smp_uart_get(smp_uart_t *p_uart, uint8_t *p_byte)
{
	int8_t bdata;
	if(p_uart->num == UART0){
	    bdata = smp_fifo_pop(&uart0_rx_fifo, (char *)p_byte);
	}else if(p_uart->num == UART1){
	    bdata = smp_fifo_pop(&uart1_rx_fifo, (char *)p_byte);
	}
	
	return(bdata);
}
uint16_t datacnt = 0;

int8_t smp_uart_get_string(smp_uart_t *p_uart, uint8_t *p_byte)
{
	int8_t bdata;
	if(p_uart->num == UART0){
			while(1){
				bdata = smp_fifo_pop(&uart0_rx_fifo, (char *)&p_byte[datacnt]);
				datacnt++;
				if(datacnt >= 256){
					datacnt = 0;
					return SMP_ERROR_INVALID_LENGTH;
				}		
				if(p_byte[datacnt - 1] == '\n')
				{
				  datacnt = 0;
					return SMP_SUCCESS;
				}else{			
					return SMP_ERROR_INVALID_DATA;
				}
			}
	    
	}else if(p_uart->num == UART1){
			while(1){
				bdata = smp_fifo_pop(&uart1_rx_fifo, (char *)&p_byte[datacnt]);
				datacnt++;
				if(datacnt >= 256){
					datacnt = 0;
					return SMP_ERROR_INVALID_LENGTH;
				}		
				if(p_byte[datacnt - 1] == '\n')
				{
				  datacnt = 0;
					return SMP_SUCCESS;
				}else{			
					return SMP_ERROR_INVALID_DATA;
				}
			}
	}
	
	return SMP_ERROR_INVALID_DATA;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uint16_t size = 0, i =0;
	if(huart->Instance == BSP_UART0){
		if(smp_fifo_get_size(&uart0_tx_fifo,&size)==SMP_SUCCESS){
			if(size>UART0_TX_BUFFER_SIZE)
				size = UART0_TX_BUFFER_SIZE;
			// This operation should be almost always successful, since we've
			// just added a byte to FIFO, but if some bigger delay occurred
			// (some heavy interrupt handler routine has been executed) since
			// that time, FIFO might be empty already.
			for(i=0;i<size;i++){
				smp_fifo_pop(&uart0_tx_fifo, (char *)&uart0_tx_buffer[i]);
			}
			HAL_UART_Transmit_DMA(&smp_uart0_handle, &uart0_tx_buffer[0], size);
		}else{
			if(uart0_evt_cb)
				uart0_evt_cb(UART_TX_EMPTY);
		}
	}else if(huart->Instance == BSP_UART1){
		if(smp_fifo_get_size(&uart1_tx_fifo,&size)==SMP_SUCCESS){
			if(size>UART1_TX_BUFFER_SIZE)
				size = UART1_TX_BUFFER_SIZE;
			// This operation should be almost always successful, since we've
			// just added a byte to FIFO, but if some bigger delay occurred
			// (some heavy interrupt handler routine has been executed) since
			// that time, FIFO might be empty already.
			for(i=0;i<size;i++){
				smp_fifo_pop(&uart1_tx_fifo, (char *)&uart1_tx_buffer[i]);
			}
			HAL_UART_Transmit_IT(&smp_uart1_handle, &uart1_tx_buffer[0], size);
		}else{
			if(uart1_evt_cb)
				uart1_evt_cb(UART_TX_EMPTY);
		}
	}
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t flag=0;
	
	if(huart->Instance == BSP_UART0){
		
		if(smp_fifo_push(&uart0_rx_fifo, (char)uart0_rx_buffer[0]) == SMP_SUCCESS){
			flag = 1;
		}else{
			flag = 0;
		}		
			
		if(HAL_UART_Receive_DMA(huart, uart0_rx_buffer, UART0_RX_BUFFER_SIZE) != HAL_OK){
			return;
		}
		
		if(flag == 1){
			if(uart0_evt_cb)
				uart0_evt_cb(UART_DATA_READY);
		}else{
			if(uart0_evt_cb)
				uart0_evt_cb(UART_BUFFER_FULL);
		}			
		
		/* Test debug, but may demage the uart function.
    gpio_test_p_state = GPIO_ACTIVE_TOGGLE;
    smp_gpio_set_state(&gpio_test_p, gpio_test_p_state);		
		*/
		
	}else if(huart->Instance == BSP_UART1){
		if(smp_fifo_push(&uart1_rx_fifo, (char)uart1_rx_buffer[0]) == SMP_SUCCESS){
			flag = 1;
		}else{
			flag = 0;
		}		
			
		if(HAL_UART_Receive_IT(huart, uart1_rx_buffer, UART1_RX_BUFFER_SIZE) != HAL_OK){
			return;
		}
		
		if(flag == 1){
        if(uart1_evt_cb)
		        uart1_evt_cb(UART_DATA_READY);
		}else{
			  if(uart1_evt_cb)
				    uart1_evt_cb(UART_BUFFER_FULL);
		}			
		
		/* Test debug, but may demage the uart function.
    gpio_test_p_state = GPIO_ACTIVE_TOGGLE;
    smp_gpio_set_state(&gpio_test_p, gpio_test_p_state);		
		*/	
	}		

}



/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - DMA configuration for transmission request by peripheral 
  *           - NVIC configuration for DMA interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	if(huart->Instance==BSP_UART0){
		static DMA_HandleTypeDef uart0_dma_tx;
		static DMA_HandleTypeDef uart0_dma_rx;
  
		GPIO_InitTypeDef  GPIO_InitStruct;

		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		
		// Initializes the peripherals clock
		if(huart->Instance==USART2){
		    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
        PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
           SEGGER_RTT_printf(0,"BSP_UART0 Init PCLK1 Error!");  
        }
	  }
		
		/* Enable SMP UART0 clock */
		BSP_UART0_CLK_ENABLE();

		/* Enable SMP UART0 GPIO TX/RX clock */
		BSP_UART0_TX_GPIO_CLK_ENABLE();
		BSP_UART0_RX_GPIO_CLK_ENABLE();		
		
		/* Enable SMP UART0 DMA clock */
		BSP_UART0_DMA_CLK_ENABLE();

		/*##-2- Configure peripheral GPIO ##########################################*/  
		/* BSP_UART0 TX GPIO pin configuration  */
		GPIO_InitStruct.Pin       = BSP_UART0_TX_PIN | BSP_UART0_RX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = BSP_UART0_TX_AF;

		HAL_GPIO_Init(BSP_UART0_TX_GPIO_PORT, &GPIO_InitStruct);

		/* BSP_UART0 RX GPIO pin configuration  */
		//GPIO_InitStruct.Pin = BSP_UART0_RX_PIN;
		//GPIO_InitStruct.Alternate = BSP_UART0_RX_AF;
		//HAL_GPIO_Init(BSP_UART0_RX_GPIO_PORT, &GPIO_InitStruct);

		/*##-3- Configure BSP_UART0 DMA ##################################################*/
		/* Configure BSP_UART0 DMA handler for Transmission process */
		uart0_dma_tx.Instance                 = BSP_UART0_TX_DMA_CHANNEL;
		uart0_dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
		uart0_dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
		uart0_dma_tx.Init.MemInc              = DMA_MINC_ENABLE;
		uart0_dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		uart0_dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		uart0_dma_tx.Init.Mode                = DMA_NORMAL;
		uart0_dma_tx.Init.Priority            = DMA_PRIORITY_LOW;
		uart0_dma_tx.Init.Request             = BSP_UART0_TX_DMA_REQUEST;

		if(HAL_DMA_Init(&uart0_dma_tx)!= HAL_OK)
    {
        SEGGER_RTT_printf(0,"BSP_UART0 Init DMA TX Error!");
    }
		
		/* Associate the initialized DMA handle to the BSP_UART0 handle */
		__HAL_LINKDMA(huart, hdmatx, uart0_dma_tx);

		/* Configure the DMA handler for reception process */
		uart0_dma_rx.Instance                 = BSP_UART0_RX_DMA_CHANNEL;
		uart0_dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
		uart0_dma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
		uart0_dma_rx.Init.MemInc              = DMA_MINC_ENABLE;
		uart0_dma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		uart0_dma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		uart0_dma_rx.Init.Mode                = DMA_NORMAL;
		uart0_dma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
		uart0_dma_rx.Init.Request             = BSP_UART0_RX_DMA_REQUEST;

		if(HAL_DMA_Init(&uart0_dma_rx)!= HAL_OK)
    {
        SEGGER_RTT_printf(0,"BSP_UART0 Init DMA RX Error!");
    }

		/* Associate the initialized DMA handle to BSP_UART0 handle */
		__HAL_LINKDMA(huart, hdmarx, uart0_dma_rx);

		/*##-4- Configure the NVIC for DMA #########################################*/
		/* NVIC configuration for DMA transfer complete interrupt (BSP_UART0) */
		HAL_NVIC_SetPriority(BSP_UART0_DMA_TX_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(BSP_UART0_DMA_TX_IRQn);

		/* NVIC configuration for DMA transfer complete interrupt (BSP_UART0) */
		HAL_NVIC_SetPriority(BSP_UART0_DMA_RX_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(BSP_UART0_DMA_RX_IRQn);

		/* NVIC for BSP_UART0, to catch the TX complete */
		HAL_NVIC_SetPriority(BSP_UART0_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(BSP_UART0_IRQn);
		
	}else if(huart->Instance==BSP_UART1){
		static DMA_HandleTypeDef uart1_dma_tx;
		static DMA_HandleTypeDef uart1_dma_rx;
  
		GPIO_InitTypeDef  GPIO_InitStruct;

		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Enable SMP UART GPIO TX/RX clock */
		BSP_UART1_TX_GPIO_CLK_ENABLE();
		BSP_UART1_RX_GPIO_CLK_ENABLE();

		/* Enable SMP UART clock */
		BSP_UART1_CLK_ENABLE();

		/* Enable SMP UART DMA clock */
		BSP_UART1_DMA_CLK_ENABLE();

		/*##-2- Configure peripheral GPIO ##########################################*/  
		/* SMP UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin       = BSP_UART1_TX_PIN | BSP_UART1_RX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = BSP_UART1_TX_AF | BSP_UART1_RX_AF;

		HAL_GPIO_Init(BSP_UART1_TX_GPIO_PORT, &GPIO_InitStruct);

		/*##-3- Configure SMP UART0 DMA ##################################################*/
		/* Configure SMP UART DMA handler for Transmission process */
		#if	0
		uart1_dma_tx.Instance                 = SMP_UART1_TX_DMA_CHANNEL;
		uart1_dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
		uart1_dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
		uart1_dma_tx.Init.MemInc              = DMA_MINC_ENABLE;
		uart1_dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		uart1_dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		uart1_dma_tx.Init.Mode                = DMA_NORMAL;
		uart1_dma_tx.Init.Priority            = DMA_PRIORITY_LOW;
		uart1_dma_tx.Init.Request             = SMP_UART1_TX_DMA_REQUEST;

		HAL_DMA_Init(&uart1_dma_tx);
	
		/* Associate the initialized DMA handle to the SMP UART handle */
		__HAL_LINKDMA(huart, hdmatx, uart1_dma_tx);
		#endif
		
		/* Configure the DMA handler for reception process */
		#if	0
		uart1_dma_rx.Instance                 = SMP_UART1_RX_DMA_CHANNEL;
		uart1_dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
		uart1_dma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
		uart1_dma_rx.Init.MemInc              = DMA_MINC_ENABLE;
		uart1_dma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		uart1_dma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		uart1_dma_rx.Init.Mode                = DMA_NORMAL;
		uart1_dma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
		uart1_dma_rx.Init.Request             = SMP_UART1_RX_DMA_REQUEST;

		HAL_DMA_Init(&uart1_dma_rx);

		/* Associate the initialized DMA handle to SMP UART handle */
		__HAL_LINKDMA(huart, hdmarx, uart1_dma_rx);
		
		/*##-4- Configure the NVIC for DMA #########################################*/
		/* NVIC configuration for DMA transfer complete interrupt (SMP UART) */
		HAL_NVIC_SetPriority(SMP_UART1_DMA_TX_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(SMP_UART1_DMA_TX_IRQn);

		/* NVIC configuration for DMA transfer complete interrupt (SMP UART) */
		HAL_NVIC_SetPriority(SMP_UART1_DMA_RX_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(SMP_UART1_DMA_RX_IRQn);
		#endif
		/* NVIC for BSP_UART1, to catch the TX complete */
		HAL_NVIC_SetPriority(BSP_UART1_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(BSP_UART1_IRQn);
	}else if(huart->Instance==USART2){
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
	if(huart->Instance == BSP_UART0){

	    /*##-1- Reset peripherals ##################################################*/
		  BSP_UART0_FORCE_RESET();
		  BSP_UART0_RELEASE_RESET();

		  /*##-2- Disable peripherals and GPIO Clocks #################################*/
		  /* Configure BSP_UART0 Tx as alternate function  */
		  HAL_GPIO_DeInit(BSP_UART0_TX_GPIO_PORT, BSP_UART0_TX_PIN);
		  /* Configure BSP_UART0 Rx as alternate function  */
		  HAL_GPIO_DeInit(BSP_UART0_RX_GPIO_PORT, BSP_UART0_RX_PIN);

		  /*##-3- Disable BSP_UART0 DMA #####################################################*/
		  /* De-Initialize BSP_UART0 DMA channel associated to reception process */
		  if(huart->hdmarx != 0){
			    HAL_DMA_DeInit(huart->hdmarx);
		  }
		  /* De-Initialize BSP_UART0 DMA channel associated to transmission process */
		  if(huart->hdmatx != 0){
			    HAL_DMA_DeInit(huart->hdmatx);
		  }  

		  /*##-4- Disable the NVIC for BSP_UART0 DMA ###########################################*/
		  HAL_NVIC_DisableIRQ(BSP_UART0_DMA_TX_IRQn);
		  HAL_NVIC_DisableIRQ(BSP_UART0_DMA_RX_IRQn);
	}else if(huart->Instance == BSP_UART1){
		  /*##-1- Reset peripherals ##################################################*/
		  BSP_UART1_FORCE_RESET();
		  BSP_UART1_RELEASE_RESET();

		  /*##-2- Disable peripherals and GPIO Clocks #################################*/
		  /* Configure BSP_UART1 Tx as alternate function  */
		  HAL_GPIO_DeInit(BSP_UART1_TX_GPIO_PORT, BSP_UART1_TX_PIN);
		  /* Configure BSP_UART1 Rx as alternate function  */
		  HAL_GPIO_DeInit(BSP_UART1_RX_GPIO_PORT, BSP_UART1_RX_PIN);

		  /*##-3- Disable BSP_UART DMA #####################################################*/
		  /* De-Initialize BSP_UART DMA channel associated to reception process */
		  if(huart->hdmarx != 0){
			    HAL_DMA_DeInit(huart->hdmarx);
		  }
		  /* De-Initialize BSP_UART DMA channel associated to transmission process */
		  if(huart->hdmatx != 0){
			    HAL_DMA_DeInit(huart->hdmatx);
		  }  
		  /*##-4- Disable the NVIC for BSP_UART1 DMA ###########################################*/
      #if 0
			HAL_NVIC_DisableIRQ(BSP_UART1_DMA_TX_IRQn);
      HAL_NVIC_DisableIRQ(BSP_UART1_DMA_RX_IRQn);
      #endif
	}
}

/**
  * @brief  This function handles DMA interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "smp_uart.h" and related to DMA  
  *         used for USART data transmission     
  */
void BSP_UART0_DMA_RX_IRQHandler(void)
{
	HAL_DMA_IRQHandler(smp_uart0_handle.hdmarx);
}

/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "smp_uart.h" and related to DMA  
  *         used for USART data reception    
  */
void BSP_UART0_DMA_TX_IRQHandler(void)
{
    HAL_DMA_IRQHandler(smp_uart0_handle.hdmatx);
}


void BSP_UART1_DMA_RX_IRQHandler(void)
{
    HAL_DMA_IRQHandler(smp_uart1_handle.hdmarx);
}

/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "smp_uart.h" and related to DMA  
  *         used for USART data reception    
  */
void BSP_UART1_DMA_TX_IRQHandler(void)
{
    HAL_DMA_IRQHandler(smp_uart1_handle.hdmatx);
}

/**
  * @brief  This function handles UART interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "smp_uart.h" and related to DMA  
  *         used for USART data transmission     
  */
void BSP_UART0_IRQHandler(void)
{
    HAL_UART_IRQHandler(&smp_uart0_handle);
}

void BSP_UART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&smp_uart1_handle);
}

/************************ (C) COPYRIGHT *****END OF FILE****/

/**
  ******************************************************************************
  * @file    smp_can.c
  * @author  Johnny/ Golden
  * @version V0.0.1
  * @date    2022/01/06
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021</center></h2>
  *
  * 
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Bsp.h"
#include "smp_can_fifo.h"
#include "smp_can.h"
#include "smp_debug.h"
#include "stm32l4xx_hal.h" 
#include "smp_gpio.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// BSP_CAN0
CAN_HandleTypeDef              smp_can0_handle = {0};
smp_can_event_t                can0_evt_cb = 0;
smp_can_fifo_t                 can0_rx_fifo = {0};
smp_can_fifo_t                 can0_tx_fifo = {0};

#if 0
uint8_t                        can0_tx_buffer[BSP_CAN0_TX_BUFFER_SIZE] = {0};
uint8_t                        can0_rx_buffer[BSP_CAN0_RX_BUFFER_SIZE] = {0};
#endif 

// BSP_CAN1
CAN_HandleTypeDef              smp_can1_handle = {0};
smp_can_event_t                can1_evt_cb = 0;
smp_can_fifo_t                 can1_rx_fifo = {0};
smp_can_fifo_t                 can1_tx_fifo = {0};

#if 0
uint8_t                        can1_tx_buffer[BSP_CAN1_TX_BUFFER_SIZE] = {0};
uint8_t                        can1_rx_buffer[BSP_CAN1_RX_BUFFER_SIZE] = {0};
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

//extern smp_gpio_t 						gpio_test_p;
//extern smp_gpio_state					gpio_test_p_state;	

int8_t can_fifo_init(smp_can_fifo_t * p_fifo, smp_can_package_t * p_buf, uint16_t buf_size)
{
    // Check buffer for null pointer.
    if (p_buf == NULL){
        return SMP_ERROR_NULL;
    }

    p_fifo->buffer_addr   = (smp_can_package_t *)p_buf;
    p_fifo->buffer_size 	= buf_size - 1;
    p_fifo->in      			= 0;
    p_fifo->out     			= 0;

    return SMP_SUCCESS;
}

int8_t smp_can_init(smp_can_t *p_can, smp_can_event_t smp_can_event_handler)
{
	CAN_FilterTypeDef  	sFilterConfig;

	if(p_can->num == __CAN0){
		//__CAN1_CLK_ENABLE();
		
 		smp_can0_handle.Instance = BSP_CAN0;

		smp_can0_handle.Init.TimeTriggeredMode = DISABLE;
		smp_can0_handle.Init.AutoBusOff = DISABLE;
		smp_can0_handle.Init.AutoWakeUp = DISABLE;
		smp_can0_handle.Init.AutoRetransmission = DISABLE;
  		smp_can0_handle.Init.ReceiveFifoLocked = DISABLE;
  		smp_can0_handle.Init.TransmitFifoPriority = ENABLE;
  		smp_can0_handle.Init.Mode = CAN_MODE_NORMAL;
  		smp_can0_handle.Init.SyncJumpWidth = CAN_SJW_1TQ;
  		smp_can0_handle.Init.TimeSeg1 = CAN_BS1_4TQ;
  		smp_can0_handle.Init.TimeSeg2 = CAN_BS2_5TQ;
  		smp_can0_handle.Init.Prescaler = 16;

  		if (HAL_CAN_Init(&smp_can0_handle) != HAL_OK)
  		{
    		/* Initialization Error */
    		return SMP_ERROR_NOT_FOUND;//SMP_ERROR_NOT_FOUND;//Error_Handler();
  		}
  
		/*##-2- Configure the CAN Filter ###########################################*/
  		sFilterConfig.FilterBank = 0;
  		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  		sFilterConfig.FilterIdHigh = 0x0000;
  		sFilterConfig.FilterIdLow = 0x0000;
  		sFilterConfig.FilterMaskIdHigh = 0x0000;
  		sFilterConfig.FilterMaskIdLow = 0x0000;
  		sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  		sFilterConfig.FilterActivation = ENABLE;
  		sFilterConfig.SlaveStartFilterBank = 0;
  		if (HAL_CAN_ConfigFilter(&smp_can0_handle, &sFilterConfig) != HAL_OK)
  		{
    		/* Filter configuration Error */
    		return SMP_ERROR_NOT_FOUND;//SMP_ERROR_NOT_FOUND;//Error_Handler();
  		}
  		/*##-3- Start the CAN peripheral ###########################################*/
  		if (HAL_CAN_Start(&smp_can0_handle) != HAL_OK)
  		{
    		/* Start Error */
    		//Error_Handler();
    		return SMP_ERROR_NOT_FOUND;
  		}	
		/*##-4- Activate CAN RX notification #######################################*/
		if (HAL_CAN_ActivateNotification(&smp_can0_handle,
				(CAN_IT_RX_FIFO0_MSG_PENDING |
				  CAN_IT_TX_MAILBOX_EMPTY
				)) != HAL_OK)
		{
			/* Notification Error */
			//Error_Handler();
			return SMP_ERROR_NOT_FOUND;
		}  
		can0_evt_cb = smp_can_event_handler;
		// Configure buffer RX buffer.
		can_fifo_init(&can0_rx_fifo, p_can->buffers.rx_buf, p_can->buffers.rx_buf_size);
		// Configure buffer TX buffer.
		can_fifo_init(&can0_tx_fifo, p_can->buffers.tx_buf, p_can->buffers.tx_buf_size);

	}else	if(p_can->num == __CAN1){
		  smp_can1_handle.Instance = BSP_CAN1;

		  smp_can1_handle.Init.TimeTriggeredMode = DISABLE;
		  smp_can1_handle.Init.AutoBusOff = DISABLE;
		  smp_can1_handle.Init.AutoWakeUp = DISABLE;
		  smp_can1_handle.Init.AutoRetransmission = DISABLE;
  		smp_can1_handle.Init.ReceiveFifoLocked = DISABLE;
  		smp_can1_handle.Init.TransmitFifoPriority = ENABLE;
  		smp_can1_handle.Init.Mode = CAN_MODE_NORMAL;
  		smp_can1_handle.Init.SyncJumpWidth = CAN_SJW_1TQ;
  		smp_can1_handle.Init.TimeSeg1 = CAN_BS1_4TQ;
  		smp_can1_handle.Init.TimeSeg2 = CAN_BS2_5TQ;
  		smp_can1_handle.Init.Prescaler = 16;

  		if (HAL_CAN_Init(&smp_can1_handle) != HAL_OK)
  		{
    		/* Initialization Error */
    		return SMP_ERROR_NOT_FOUND;//Error_Handler();
  		}
  
		/*##-2- Configure the CAN Filter ###########################################*/
  		sFilterConfig.FilterBank = 14;
  		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  		sFilterConfig.FilterIdHigh = 0x0000;
  		sFilterConfig.FilterIdLow = 0x0000;
  		sFilterConfig.FilterMaskIdHigh = 0x0000;
  		sFilterConfig.FilterMaskIdLow = 0x0000;
  		sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  		sFilterConfig.FilterActivation = ENABLE;
  		sFilterConfig.SlaveStartFilterBank = 14;

  		if (HAL_CAN_ConfigFilter(&smp_can1_handle, &sFilterConfig) != HAL_OK)
  		{
    		/* Filter configuration Error */
    		return SMP_ERROR_NOT_FOUND;//Error_Handler();
  		}

  		/*##-3- Start the CAN peripheral ###########################################*/
  		if (HAL_CAN_Start(&smp_can1_handle) != HAL_OK)
  		{
    		/* Start Error */
    		//Error_Handler();
    		return SMP_ERROR_NOT_FOUND;
  		}	
		/*##-4- Activate CAN RX notification #######################################*/
		if (HAL_CAN_ActivateNotification(&smp_can1_handle,
				(CAN_IT_RX_FIFO0_MSG_PENDING |
				  CAN_IT_TX_MAILBOX_EMPTY
				)) != HAL_OK)
		{
			/* Notification Error */
			//Error_Handler();
			return SMP_ERROR_NOT_FOUND;
		}  
		can1_evt_cb = smp_can_event_handler;
		// Configure buffer RX buffer.
		can_fifo_init(&can1_rx_fifo, p_can->buffers.rx_buf, p_can->buffers.rx_buf_size);
		// Configure buffer TX buffer.
		can_fifo_init(&can1_tx_fifo, p_can->buffers.tx_buf, p_can->buffers.tx_buf_size);

	 }else{
		return SMP_ERROR_NOT_SUPPORTED;
	 }		
	 return SMP_SUCCESS;
}

int8_t smp_can_deinit(smp_can_t *p_can)
{
	if(p_can->num == __CAN0){
		HAL_CAN_DeInit(&smp_can0_handle);		
	}else if(p_can->num == __CAN1){
		#if	0
		if(smp_uart1_handle.Instance != 0){
			if(HAL_UART_DeInit(&smp_uart1_handle) != HAL_OK)
				return SMP_ERROR_NOT_FOUND;
			smp_uart1_handle.Instance = 0;
		}
		#endif
	}		
	return SMP_SUCCESS;
}

int8_t smp_can_put(smp_can_t *p_can, smp_can_package_t *pCanDat)
{
	int8_t	status = SMP_SUCCESS;
	uint16_t size = 0, i =0;
	uint32_t              TxMailbox;
	CAN_TxHeaderTypeDef   TxHeader;
	smp_can_package_t		CanPkg;
	uint8_t	buf[10];

	
	if(p_can->num == __CAN0){
		status = smp_can_fifo_push(&can0_tx_fifo, pCanDat);// == SMP_SUCCESS)
//			 if (smp_can_fifo_push(&can1_tx_fifo, pCanDat) == SMP_SUCCESS)
		if(1)
        {
        	//if()	//check empty mailbox
        	if(HAL_CAN_GetTxMailboxesFreeLevel(&smp_can0_handle) != 0)
        	{
//				GPIOD->ODR |= GPIO_PIN_13;
       			if(smp_can_fifo_pop(&can0_tx_fifo, &CanPkg) == SMP_SUCCESS)
       			{
       				if(CanPkg.id & CAN_STD_MASK)
       				{
						TxHeader.StdId = (CanPkg.id & CAN_ID_MASK);
						TxHeader.IDE = CAN_ID_STD;
					}
					else
					{
						TxHeader.ExtId = (CanPkg.id & CAN_ID_MASK);
						TxHeader.IDE = CAN_ID_EXT;
					}
					if(CanPkg.id & CAN_RTR_MASK)
						TxHeader.RTR = CAN_RTR_REMOTE;
					else
						TxHeader.RTR = CAN_RTR_DATA;
  					TxHeader.DLC = CanPkg.dlc;
					TxHeader.TransmitGlobalTime = DISABLE;
					HAL_CAN_AddTxMessage(&smp_can0_handle, &TxHeader, CanPkg.dat, &TxMailbox);
      			}
//				GPIOD->ODR &= ~GPIO_PIN_13;

  			}	
	   	}		
	}else if(p_can->num == __CAN1){		
        status = smp_can_fifo_push(&can1_tx_fifo, pCanDat);// == SMP_SUCCESS)
//			 if (smp_can_fifo_push(&can1_tx_fifo, pCanDat) == SMP_SUCCESS)
		if(1)
        {
        	//if()	//check empty mailbox
        	if(HAL_CAN_GetTxMailboxesFreeLevel(&smp_can1_handle) != 0)
        	{
//				GPIOD->ODR |= GPIO_PIN_13;
       			if(smp_can_fifo_pop(&can1_tx_fifo, &CanPkg) == SMP_SUCCESS)
       			{
       				if(CanPkg.id & CAN_STD_MASK)
       				{
						TxHeader.StdId = (CanPkg.id & CAN_ID_MASK);
						TxHeader.IDE = CAN_ID_STD;
					}
					else
					{
						TxHeader.ExtId = (CanPkg.id & CAN_ID_MASK);
						TxHeader.IDE = CAN_ID_EXT;
					}
					if(CanPkg.id & CAN_RTR_MASK)
						TxHeader.RTR = CAN_RTR_REMOTE;
					else
						TxHeader.RTR = CAN_RTR_DATA;
  					TxHeader.DLC = CanPkg.dlc;
					TxHeader.TransmitGlobalTime = DISABLE;
					HAL_CAN_AddTxMessage(&smp_can1_handle, &TxHeader, CanPkg.dat, &TxMailbox);
      			}
//				GPIOD->ODR &= ~GPIO_PIN_13;

  			}	
	   	}
	}else{
		return SMP_ERROR_FULL;
	}
	
	return status;
}


int8_t smp_can_get(smp_can_t *p_can, smp_can_package_t *pCanDat)
{
	int8_t bdata;
	if(p_can->num == __CAN0){
	    bdata = smp_can_fifo_pop(&can0_rx_fifo, pCanDat);
	}else if(p_can->num == __CAN1){
	    bdata = smp_can_fifo_pop(&can1_rx_fifo, pCanDat);
	}
	
	return(bdata);
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//void HAL_CAN_TxCpltCallback(UART_HandleTypeDef *huart)
//void HAL_CAN_TxMailbox1AbortCallback
void PutCanDataToMailBox(CAN_HandleTypeDef *hcan)
{	
	uint16_t size = 0, i =0;
	uint32_t              TxMailbox;
	CAN_TxHeaderTypeDef   TxHeader;
	smp_can_package_t		CanPkg;
	uint8_t	buf[10];


	if(hcan->Instance == BSP_CAN0){
		if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 0)
        {
//        	GPIOD->ODR ^= GPIO_PIN_14;
       		if(smp_can_fifo_pop(&can0_tx_fifo, &CanPkg) == SMP_SUCCESS)
       		{
       			if(CanPkg.id & CAN_STD_MASK)
       			{
					TxHeader.StdId = (CanPkg.id & CAN_ID_MASK);
					TxHeader.IDE = CAN_ID_STD;
				}
				else
				{
					TxHeader.ExtId = (CanPkg.id & CAN_ID_MASK);
					TxHeader.IDE = CAN_ID_EXT;
				}
				if(CanPkg.id & CAN_RTR_MASK)
					TxHeader.RTR = CAN_RTR_REMOTE;
				else
					TxHeader.RTR = CAN_RTR_DATA;
  					TxHeader.DLC = CanPkg.dlc;
				TxHeader.TransmitGlobalTime = DISABLE;
				HAL_CAN_AddTxMessage(&smp_can0_handle, &TxHeader, CanPkg.dat, &TxMailbox);
      		}
  		}			
	}else if(hcan->Instance == BSP_CAN1){
		
		//if()	//check empty mailbox
      	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 0)
        {
//        	GPIOD->ODR ^= GPIO_PIN_14;
       		if(smp_can_fifo_pop(&can1_tx_fifo, &CanPkg) == SMP_SUCCESS)
       		{
       			if(CanPkg.id & CAN_STD_MASK)
       			{
					TxHeader.StdId = (CanPkg.id & CAN_ID_MASK);
					TxHeader.IDE = CAN_ID_STD;
				}
				else
				{
					TxHeader.ExtId = (CanPkg.id & CAN_ID_MASK);
					TxHeader.IDE = CAN_ID_EXT;
				}
				if(CanPkg.id & CAN_RTR_MASK)
					TxHeader.RTR = CAN_RTR_REMOTE;
				else
					TxHeader.RTR = CAN_RTR_DATA;
  					TxHeader.DLC = CanPkg.dlc;
				TxHeader.TransmitGlobalTime = DISABLE;
				HAL_CAN_AddTxMessage(&smp_can1_handle, &TxHeader, CanPkg.dat, &TxMailbox);
      		}
  		}	
	}
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	PutCanDataToMailBox(hcan);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	PutCanDataToMailBox(hcan);
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	PutCanDataToMailBox(hcan);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t flag=0;
	CAN_RxHeaderTypeDef   RxHeader;
	uint8_t               RxData[8];
	smp_can_package_t	CanPkg;

	
	if(hcan->Instance == BSP_CAN0){
		/* Get RX message */
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		{
			/* Reception Error */
			//Error_Handler();
			return;
		}
		if(RxHeader.IDE == CAN_ID_STD)
		{
			CanPkg.id = RxHeader.StdId;
			CanPkg.id |= CAN_STD_MASK;
		}
		else
		{
			CanPkg.id = RxHeader.ExtId;
		}
		if(RxHeader.RTR == CAN_RTR_REMOTE)
		{
			CanPkg.id |= CAN_RTR_MASK;
		}
		CanPkg.dlc = RxHeader.DLC;
		if(CanPkg.dlc > 8)
			CanPkg.dlc = 8;
		memcpy(CanPkg.dat, RxData, CanPkg.dlc);
		
		if(smp_can_fifo_push(&can0_rx_fifo, &CanPkg) == SMP_SUCCESS)
		{
			if(can0_evt_cb)
				can0_evt_cb(CAN_DATA_READY);
		}else{
			if(can0_evt_cb)
				can0_evt_cb(CAN_BUFFER_FULL);
		}			
		
	}else if(hcan->Instance == BSP_CAN1){
//			GPIOD->ODR ^= GPIO_PIN_15;
		/* Get RX message */
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		{
			/* Reception Error */
			//Error_Handler();
			return;
		}
		if(RxHeader.IDE == CAN_ID_STD)
		{
			CanPkg.id = RxHeader.StdId;
			CanPkg.id |= CAN_STD_MASK;
		}
		else
		{
			CanPkg.id = RxHeader.ExtId;
		}
		if(RxHeader.RTR == CAN_RTR_REMOTE)
		{
			CanPkg.id |= CAN_RTR_MASK;
		}
		CanPkg.dlc = RxHeader.DLC;
		if(CanPkg.dlc > 8)
			CanPkg.dlc = 8;
		memcpy(CanPkg.dat, RxData, CanPkg.dlc);
		
		if(smp_can_fifo_push(&can1_rx_fifo, &CanPkg) == SMP_SUCCESS)
		{
			if(can1_evt_cb)
				can1_evt_cb(CAN_DATA_READY);
		}else{
			if(can1_evt_cb)
				can1_evt_cb(CAN_BUFFER_FULL);
		}		
  	}
}




void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{	
	GPIO_InitTypeDef   GPIO_InitStruct;
	
	if(hcan->Instance == BSP_CAN0)
	{
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* BSP_CAN1 Periph clock enable */
		BSP_CAN0_CLK_ENABLE();
		/* Enable GPIO clock ****************************************/
		BSP_CAN0_RX_GPIO_CLK_ENABLE();
		BSP_CAN0_TX_GPIO_CLK_ENABLE();
		
		/*##-2- Configure peripheral GPIO ##########################################*/
		/* BSP_CAN0 TX GPIO pin configuration */
		GPIO_InitStruct.Pin = BSP_CAN0_TX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Alternate =  BSP_CAN0_TX_AF;
		HAL_GPIO_Init(BSP_CAN0_TX_GPIO_PORT, &GPIO_InitStruct);

		/* BSP_CAN0 RX GPIO pin configuration */
		GPIO_InitStruct.Pin = BSP_CAN0_RX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Alternate =  BSP_CAN0_RX_AF;

		HAL_GPIO_Init(BSP_CAN0_RX_GPIO_PORT, &GPIO_InitStruct);

		/*##-3- Configure the NVIC #################################################*/
		/* NVIC configuration for BSP_CAN1 Reception complete interrupt */
		HAL_NVIC_SetPriority(BSP_CAN0_RX_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(BSP_CAN0_RX_IRQn); 	

		HAL_NVIC_SetPriority(BSP_CAN0_TX_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(BSP_CAN0_TX_IRQn); 	
				
	}
	else if(hcan->Instance == BSP_CAN1)
	{
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* BSP_CAN1 Periph clock enable */
		BSP_CAN1_CLK_ENABLE();
		/* Enable GPIO clock ****************************************/
		BSP_CAN1_RX_GPIO_CLK_ENABLE();
		BSP_CAN1_TX_GPIO_CLK_ENABLE();
		
		/*##-2- Configure peripheral GPIO ##########################################*/
		/* BSP_CAN1 TX GPIO pin configuration */
		GPIO_InitStruct.Pin = BSP_CAN1_TX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Alternate =  BSP_CAN1_TX_AF;
		HAL_GPIO_Init(BSP_CAN1_TX_GPIO_PORT, &GPIO_InitStruct);

		/* BSP_CAN1 RX GPIO pin configuration */
		GPIO_InitStruct.Pin = BSP_CAN1_RX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Alternate =  BSP_CAN1_RX_AF;

		HAL_GPIO_Init(BSP_CAN1_RX_GPIO_PORT, &GPIO_InitStruct);

		/*##-3- Configure the NVIC #################################################*/
		/* NVIC configuration for BSP_CAN1 Reception complete interrupt */
		HAL_NVIC_SetPriority(BSP_CAN1_RX_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(BSP_CAN1_RX_IRQn); 	

		HAL_NVIC_SetPriority(BSP_CAN1_TX_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(BSP_CAN1_TX_IRQn); 	

    #if 0
		GPIOD->ODR ^= GPIO_PIN_15;
		CAN2_TX_IRQn                = 86,     /*!< CAN2 TX interrupt                                                 */
		CAN2_RX0_IRQn               = 87,     /*!< CAN2 RX0 interrupt                                                */
		CAN2_RX1_IRQn               = 88,     /*!< CAN2 RX1 interrupt                                                */
    #endif

	}	
	
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan)
{
	GPIO_InitTypeDef   GPIO_InitStruct;
	
	if(hcan->Instance == BSP_CAN0)
	{
#if 1 
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* BSP_CAN1 Periph clock enable */
		__HAL_RCC_CAN1_CLK_DISABLE();
		/* Enable GPIO clock ****************************************/
		BSP_CAN0_RX_GPIO_CLK_ENABLE();
		BSP_CAN0_TX_GPIO_CLK_ENABLE();
		
		/*##-2- Configure peripheral GPIO ##########################################*/
		/* BSP_CAN0 TX GPIO pin configuration */
		GPIO_InitStruct.Pin = BSP_CAN0_TX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Alternate =  0;
		HAL_GPIO_Init(BSP_CAN0_TX_GPIO_PORT, &GPIO_InitStruct);

		/* BSP_CAN0 RX GPIO pin configuration */
		GPIO_InitStruct.Pin = BSP_CAN0_RX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Alternate =  0;

		HAL_GPIO_Init(BSP_CAN0_RX_GPIO_PORT, &GPIO_InitStruct);

		/*##-3- Configure the NVIC #################################################*/
		/* NVIC configuration for BSP_CAN1 Reception complete interrupt */
		HAL_NVIC_SetPriority(BSP_CAN0_RX_IRQn, 1, 0);
		HAL_NVIC_DisableIRQ(BSP_CAN0_RX_IRQn); 	

		HAL_NVIC_SetPriority(BSP_CAN0_TX_IRQn, 1, 0);
		HAL_NVIC_DisableIRQ(BSP_CAN0_TX_IRQn); 	
#endif			
	}
	else if(hcan->Instance == BSP_CAN1)
	{
#if 1		
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* BSP_CAN1 Periph clock enable */
		__HAL_RCC_CAN2_CLK_DISABLE();
		/* Enable GPIO clock ****************************************/
		BSP_CAN1_RX_GPIO_CLK_ENABLE();
		BSP_CAN1_TX_GPIO_CLK_ENABLE();
		
		/*##-2- Configure peripheral GPIO ##########################################*/
		/* BSP_CAN1 TX GPIO pin configuration */
		GPIO_InitStruct.Pin = BSP_CAN1_TX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Alternate =  BSP_CAN1_TX_AF;
		HAL_GPIO_Init(BSP_CAN1_TX_GPIO_PORT, &GPIO_InitStruct);

		/* BSP_CAN1 RX GPIO pin configuration */
		GPIO_InitStruct.Pin = BSP_CAN1_RX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Alternate =  BSP_CAN1_RX_AF;

		HAL_GPIO_Init(BSP_CAN1_RX_GPIO_PORT, &GPIO_InitStruct);

		/*##-3- Configure the NVIC #################################################*/
		/* NVIC configuration for BSP_CAN1 Reception complete interrupt */
		HAL_NVIC_SetPriority(BSP_CAN1_RX_IRQn, 1, 0);
		HAL_NVIC_DisableIRQ(BSP_CAN1_RX_IRQn); 	

		HAL_NVIC_SetPriority(BSP_CAN1_TX_IRQn, 1, 0);
		HAL_NVIC_DisableIRQ(BSP_CAN1_TX_IRQn); 	
#endif
	}	
}

void BSP_CAN0_TX_IRQHandler(void)
{
	#if 0
  GPIOD->ODR ^= GPIO_PIN_14;
  #endif
	
	HAL_CAN_IRQHandler(&smp_can0_handle);
}

void BSP_CAN0_RX_IRQHandler(void)
{
	#if 0
  GPIOD->ODR ^= GPIO_PIN_15;
  #endif
	
	HAL_CAN_IRQHandler(&smp_can0_handle);
	
}

void BSP_CAN1_TX_IRQHandler(void)
{
	#if 0
  GPIOD->ODR ^= GPIO_PIN_14;
  #endif
	
	HAL_CAN_IRQHandler(&smp_can1_handle);
}

void BSP_CAN1_RX_IRQHandler(void)
{
	#if 0
  GPIOD->ODR ^= GPIO_PIN_15;
  #endif
	
	HAL_CAN_IRQHandler(&smp_can1_handle);
	
}

/************************ (C) COPYRIGHT Simplo all right reserved *****END OF FILE****/


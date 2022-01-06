/**
  ******************************************************************************
  * @file        HalUartSTM32L4xx.c
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/12
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"
#include "halcan.h"
#include "LibDebug.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static 	CAN_HandleTypeDef 	CanHandle1;
static 	CAN_HandleTypeDef 	CanHandle2;

/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
tErrCode HalCanTx(tHalCan *pHalCan, tHalCanTxMessage *pCanTxBuf)
{
	uint32_t              TxMailbox;
	CAN_TxHeaderTypeDef   TxHeader;
	
	if(pHalCan->PortNo == 2)
	{
		if(pCanTxBuf->StdFrameFlag)
		{
			TxHeader.StdId = pCanTxBuf->CanId;;
			TxHeader.IDE = CAN_ID_STD;
		}
		else		
		{
			TxHeader.ExtId = pCanTxBuf->CanId;
			TxHeader.IDE = CAN_ID_EXT;
		}
		if(pCanTxBuf->RtrFlag)
			TxHeader.RTR = CAN_RTR_REMOTE;
		else
			TxHeader.RTR = CAN_RTR_DATA;
  		TxHeader.DLC = pCanTxBuf->DLC;
  		TxHeader.TransmitGlobalTime = DISABLE;
  
//   		TxHeader.TransmitGlobalTime = DISABLE;
   
		HAL_CAN_AddTxMessage(&CanHandle2, &TxHeader, pCanTxBuf->DataBuf, &TxMailbox);
	}
	else
		return RES_ERROR_INVALID_PARAM;
	return RES_SUCCESS;
}

void CAN1_RX0_IRQHandler(void)
{
  	HAL_CAN_IRQHandler(&CanHandle1);
}


#if	0
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
#if	0	
  GPIO_InitTypeDef   GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* CAN1 Periph clock enable */
  CANx_CLK_ENABLE();
  /* Enable GPIO clock ****************************************/
  CANx_GPIO_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* CAN1 TX GPIO pin configuration */
  GPIO_InitStruct.Pin = CANx_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate =  CANx_TX_AF;

  HAL_GPIO_Init(CANx_TX_GPIO_PORT, &GPIO_InitStruct);

  /* CAN1 RX GPIO pin configuration */
  GPIO_InitStruct.Pin = CANx_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate =  CANx_RX_AF;

  HAL_GPIO_Init(CANx_RX_GPIO_PORT, &GPIO_InitStruct);

  /*##-3- Configure the NVIC #################################################*/
  /* NVIC configuration for CAN1 Reception complete interrupt */
  HAL_NVIC_SetPriority(CANx_RX_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(CANx_RX_IRQn);
#endif  
}

#endif


tErrCode HalCanOpen(tHalCan *pHalCan)
{
	CAN_FilterTypeDef  	sFilterConfig;
	GPIO_InitTypeDef 	GPIO_InitStructure;

	if(pHalCan->PortNo == 1)
	{
		__CAN1_CLK_ENABLE();
		
		CanHandle1.Instance = CAN1;

		CanHandle1.Init.TimeTriggeredMode = DISABLE;
		CanHandle1.Init.AutoBusOff = DISABLE;
		CanHandle1.Init.AutoWakeUp = DISABLE;
		CanHandle1.Init.AutoRetransmission = DISABLE;
  		CanHandle1.Init.ReceiveFifoLocked = DISABLE;
  		CanHandle1.Init.TransmitFifoPriority = ENABLE;
  		CanHandle1.Init.Mode = CAN_MODE_NORMAL;
  		CanHandle1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  		CanHandle1.Init.TimeSeg1 = CAN_BS1_4TQ;
  		CanHandle1.Init.TimeSeg2 = CAN_BS2_5TQ;
  		CanHandle1.Init.Prescaler = 8;

  		if (HAL_CAN_Init(&CanHandle1) != HAL_OK)
  		{
    		/* Initialization Error */
    		Error_Handler();
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
  		sFilterConfig.SlaveStartFilterBank = 14;

  		if (HAL_CAN_ConfigFilter(&CanHandle1, &sFilterConfig) != HAL_OK)
  		{
    		/* Filter configuration Error */
    		Error_Handler();
  		}

  		/*##-3- Start the CAN peripheral ###########################################*/
  		if (HAL_CAN_Start(&CanHandle1) != HAL_OK)
  		{
    		/* Start Error */
    		//Error_Handler();
  		}	
  		
  	//	define CANx_RX_IRQn                   CAN1_RX0_IRQn
//#define CANx_RX_IRQHandler             CAN1_RX0_IRQHandler


	}
	else if(pHalCan->PortNo == 2)
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();
		__CAN2_CLK_ENABLE();
		
		GPIO_InitStructure.Pin = GPIO_PIN_5;		//rx
		GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStructure.Pull = GPIO_PULLUP;
		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStructure.Alternate = GPIO_AF3_CAN2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		 /* CAN2 TX GPIO pin configuration  */
		GPIO_InitStructure.Pin = GPIO_PIN_6;
		GPIO_InitStructure.Alternate = GPIO_AF8_CAN2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		HAL_CAN_DeInit(&CanHandle2);
	//CAN_StructInit(&CAN_InitStructure);
		
		CanHandle2.Instance = CAN2;

		CanHandle2.Init.TimeTriggeredMode = DISABLE;
		CanHandle2.Init.AutoBusOff = DISABLE;
		CanHandle2.Init.AutoWakeUp = DISABLE;
		CanHandle2.Init.AutoRetransmission = DISABLE;
  		CanHandle2.Init.ReceiveFifoLocked = DISABLE;
  		CanHandle2.Init.TransmitFifoPriority = DISABLE;
  		CanHandle2.Init.Mode = CAN_MODE_NORMAL;
  		CanHandle2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  		CanHandle2.Init.TimeSeg1 = CAN_BS1_4TQ;
  		CanHandle2.Init.TimeSeg2 = CAN_BS2_5TQ;
  		CanHandle2.Init.Prescaler = 16;

  		if (HAL_CAN_Init(&CanHandle2) != HAL_OK)
  		{
    		/* Initialization Error */
    		return -1;//RES_ERROR_INIT;//Error_Handler();
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
  		sFilterConfig.SlaveStartFilterBank = 14;

  		if (HAL_CAN_ConfigFilter(&CanHandle2, &sFilterConfig) != HAL_OK)
  		{
    		/* Filter configuration Error */
    		return -2;//RES_ERROR_INIT;//Error_Handler();
  		}

  		/*##-3- Start the CAN peripheral ###########################################*/
  		if (HAL_CAN_Start(&CanHandle2) != HAL_OK)
  		{
    		/* Start Error */
    		//Error_Handler();
			return -3;//RES_ERROR_INIT;
  		}	
  		/* Activate CAN RX notification */
  		if (HAL_CAN_ActivateNotification(&CanHandle2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)  //========== CRITICAL ==========//
		//if(0)
  		{
    		/* Notification Error */
	  		//Error_Handler();
	  		//bCAN1initialized = false;
	  		return -4;
  		}
	}
	else
		return RES_ERROR_INVALID_PARAM;
	return RES_SUCCESS;
}

tErrCode HalCanClose(tHalCan *pHalCan)
{
	if(pHalCan->PortNo == 1)
	{
		
	}
	else if(pHalCan->PortNo == 2)
	{
		
	}
	else
		return RES_ERROR_INVALID_PARAM;
	return RES_SUCCESS;

}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    




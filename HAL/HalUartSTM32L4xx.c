/**
  ******************************************************************************
  * @file        HalUartSTM32L4.c
  * @author      Johnny
  * @version     v0.0
  * @date        2021/9/7
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
#include "LibDebug.h"
#include "haluart.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define	UART2_FIFO_NUM	400
#define	UART2_TX_FIFO_NUM	500
#define	UART2_TX_BUF_NUM	50

#define	UART3_FIFO_NUM	400
#define	UART3_TX_FIFO_NUM	500
#define	UART3_TX_BUF_NUM	50

/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
static	uint8_t	Uart2BufTemp[2];
static uint8_t	Uart2Buffer[UART2_FIFO_NUM + 2];
static uint16_t	Uart2BufSptr;
static uint16_t	Uart2BufEptr;

static __IO uint8_t	Uart2TxFifo[UART2_TX_FIFO_NUM + 2];
static __IO uint16_t	Uart2TxFifoSptr;
static __IO	uint16_t	Uart2TxFifoEptr;


static	uint8_t	Uart3BufTemp[2];
static uint8_t	Uart3Buffer[UART3_FIFO_NUM + 2];
static uint16_t	Uart3BufSptr;
static uint16_t	Uart3BufEptr;

static __IO uint8_t		Uart3TxFifo[UART3_TX_FIFO_NUM + 2];
static __IO uint16_t	Uart3TxFifoSptr;
static __IO	uint16_t	Uart3TxFifoEptr;

static 	UART_HandleTypeDef UartHandle2;
static 	UART_HandleTypeDef UartHandle3;

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
uint8_t IsUart2FifoEmpty(void)
{
	if(Uart2BufEptr == Uart2BufSptr)
		return 1;
	else
		return 0;
}
uint8_t	GetUart2FifoData(void)
{
	uint8_t	dat;
	
	dat = Uart2Buffer[Uart2BufSptr++];

	if(Uart2BufSptr >= UART2_FIFO_NUM)
		Uart2BufSptr = 0;
	return dat;
}

uint8_t IsUart3FifoEmpty(void)
{
	if(Uart3BufEptr == Uart3BufSptr)
		return 1;
	else
		return 0;
}
uint8_t	GetUart3FifoData(void)
{
	uint8_t	dat;
	
	dat = Uart3Buffer[Uart3BufSptr++];

	if(Uart3BufSptr >= UART3_FIFO_NUM)
		Uart3BufSptr = 0;
	return dat;
}

void AddUart2Fifo(uint8_t dat)
{
	Uart2Buffer[Uart2BufEptr++] = dat;
	if(Uart2BufEptr >= UART2_FIFO_NUM)
		Uart2BufEptr = 0;
}

void AddUart3Fifo(uint8_t dat)
{
	Uart3Buffer[Uart3BufEptr++] = dat;
	if(Uart3BufEptr >= UART3_FIFO_NUM)
		Uart3BufEptr = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	AddUart3Fifo(Uart3BufTemp[0]);
	HAL_UART_Receive_IT(huart, (uint8_t *)Uart3BufTemp, 1);
		
}

static void TransmitUart2Fifo(void)
{
	uint16_t	i;
	static uint8_t	txbuf[UART2_TX_BUF_NUM];
	
//	if(Uart2TxFifoSptr == Uart2TxFifoEptr)
//		return;
	
	for(i=0; i<UART2_TX_BUF_NUM; i++)
	{
		if(Uart2TxFifoSptr == Uart2TxFifoEptr)
			break;
		txbuf[i] = Uart2TxFifo[Uart2TxFifoSptr++];
		if(Uart2TxFifoSptr >= UART2_TX_FIFO_NUM)
		 	Uart2TxFifoSptr = 0;
	}
	if(i)
	{
		HAL_UART_Transmit_IT(&UartHandle2, txbuf, i);		
	}	
}


static void TransmitUart3Fifo(void)
{
	uint16_t	i;
	static uint8_t	txbuf[UART3_TX_BUF_NUM];
	
	
	for(i=0; i<UART3_TX_BUF_NUM; i++)
	{
		if(Uart3TxFifoSptr == Uart3TxFifoEptr)
			break;
		txbuf[i] = Uart3TxFifo[Uart3TxFifoSptr++];
		if(Uart3TxFifoSptr >= UART3_TX_FIFO_NUM)
		 	Uart3TxFifoSptr = 0;
	}
	if(i)
	{
		HAL_UART_Transmit_IT(&UartHandle3, txbuf, i);		
	}	
}
//transmit complete
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	TransmitUart3Fifo();
}

static void AddTxDataToFifo2(uint8_t *pSendBuf, uint16_t leng)
{
	uint16_t	i;
	__disable_irq();
	for(i=0; i<leng; i++)
	{
		Uart2TxFifo[Uart2TxFifoEptr++] = pSendBuf[i];
		if(Uart2TxFifoEptr >= UART2_TX_FIFO_NUM)
			Uart2TxFifoEptr = 0	;
	}
	__enable_irq();
}

static void AddTxDataToFifo3(uint8_t *pSendBuf, uint16_t leng)
{
	uint16_t	i;
	__disable_irq();
	for(i=0; i<leng; i++)
	{
		Uart3TxFifo[Uart3TxFifoEptr++] = pSendBuf[i];
		if(Uart3TxFifoEptr >= UART2_TX_FIFO_NUM)
			Uart3TxFifoEptr = 0	;
	}
	__enable_irq();
}
/* Public function prototypes -----------------------------------------------*/




/******************************************************************************/
/*                 STM32L4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l4xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles UART interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA  
  *         used for USART data transmission     
  */
void USART2_IRQHandler(void)
{
//	GPIOA->ODR |= GPIO_PIN_0;	

	HAL_UART_IRQHandler(&UartHandle2);
//	GPIOA->ODR &= ~GPIO_PIN_0;	
}

void USART3_IRQHandler(void)
{
//	GPIOA->ODR |= GPIO_PIN_0;	

	HAL_UART_IRQHandler(&UartHandle3);
//	GPIOA->ODR &= ~GPIO_PIN_0;	
}

tErrCode HalUartTx(tHalUart *pHalUart, uint8_t *pSendBuf, uint16_t leng)
{
	static	uint8_t	debug=0;
	//uint16_t	i;
	if(pHalUart->PortNo == 2)
	{
//		 if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK)
		//if(debug >= 5)
		//	return 0;
		debug++;
 		AddTxDataToFifo2(pSendBuf, leng);
 		
		if (UartHandle2.gState == HAL_UART_STATE_READY)
		{
			TransmitUart2Fifo();
		}
	}
	else if(pHalUart->PortNo == 3)
	{
		debug++;
 		AddTxDataToFifo3(pSendBuf, leng);
 		
		if (UartHandle3.gState == HAL_UART_STATE_READY)
		{
			TransmitUart3Fifo();
		}
	}
	return RES_SUCCESS;
}



tErrCode HalUartOpen(tHalUart *pHalUart)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	
	if(pHalUart->PortNo == 2)
	{
		__HAL_RCC_GPIOD_CLK_ENABLE();
		__HAL_RCC_USART2_CLK_ENABLE();			
		
		Uart2TxFifoSptr = 0;
		Uart2TxFifoEptr = 0;

		
		GPIO_InitStructure.Pin = GPIO_PIN_5;		//tx
		GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStructure.Pull = GPIO_PULLUP;
		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStructure); 
		
		 /* UART RX GPIO pin configuration  */
		GPIO_InitStructure.Pin = GPIO_PIN_6;
		GPIO_InitStructure.Alternate = GPIO_AF7_USART2;

		HAL_GPIO_Init(GPIOD, &GPIO_InitStructure); 
		
		UartHandle2.Instance        = USART2;

  		UartHandle2.Init.BaudRate   = pHalUart->Baudrate;
  		UartHandle2.Init.WordLength = UART_WORDLENGTH_8B;
  		UartHandle2.Init.StopBits   = UART_STOPBITS_1;
  		UartHandle2.Init.Parity     = UART_PARITY_NONE;
  		UartHandle2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  		UartHandle2.Init.Mode       = UART_MODE_TX_RX;
  		if(HAL_UART_Init(&UartHandle2) != HAL_OK)
  		{
  			return RES_ERROR_INIT;
  		}	
		
		HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);		
		HAL_NVIC_EnableIRQ(USART2_IRQn);		
			
		if(HAL_UART_Receive_IT(&UartHandle2, (uint8_t *)Uart2BufTemp, 1) != HAL_OK)
		{
			;//Error_Handler();
		}
	}
	else if(pHalUart->PortNo == 3)
	{
#if	1		
		__HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_USART3_CLK_ENABLE();			
		
		Uart3TxFifoSptr = 0;
		Uart3TxFifoEptr = 0;
		
		GPIO_InitStructure.Pin = GPIO_PIN_10;		//tx
		GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStructure.Pull = GPIO_PULLUP;
		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStructure.Alternate = GPIO_AF7_USART3;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure); 
		
		 /* UART RX GPIO pin configuration  */
		GPIO_InitStructure.Pin = GPIO_PIN_11;
		GPIO_InitStructure.Alternate = GPIO_AF7_USART3;

		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure); 
		
		UartHandle3.Instance        = USART3;

  		UartHandle3.Init.BaudRate   = pHalUart->Baudrate;
  		UartHandle3.Init.WordLength = UART_WORDLENGTH_8B;
  		UartHandle3.Init.StopBits   = UART_STOPBITS_1;
  		UartHandle3.Init.Parity     = UART_PARITY_NONE;
  		UartHandle3.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  		UartHandle3.Init.Mode       = UART_MODE_TX_RX;
  		if(HAL_UART_Init(&UartHandle3) != HAL_OK)
  		{
  			return RES_ERROR_INIT;
  		}	
		
		HAL_NVIC_SetPriority(USART3_IRQn, 0, 1);		
		HAL_NVIC_EnableIRQ(USART3_IRQn);
			
		if(HAL_UART_Receive_IT(&UartHandle3, (uint8_t *)Uart3BufTemp, 1) != HAL_OK)
		{
			;//Error_Handler();
		}
#endif		
	}
	else 
		return RES_ERROR_INVALID_PARAM;
	return RES_SUCCESS;
}


tErrCode HalUartClose(tHalUart *pHalUart)
{
	if(pHalUart->PortNo == 3)
	{
		
	}
	else 
		return RES_ERROR_INVALID_PARAM;	
	return RES_SUCCESS;

}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    




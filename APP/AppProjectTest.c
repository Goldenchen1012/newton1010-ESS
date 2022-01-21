/**
  ******************************************************************************
  * @file        AppSerialUartDavinci.c
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
#include "define.h"
#include "main.h"
#include "LibDebug.h"
#include "smp_debug.h"
#include "halafe.h"
#include "halTimer.h"
#include "smp_uart.h"
#include "smp_fifo.h"
#include "ApiProtectOvp.h"
#include "ApiProtectUvp.h"
#include "ApiProtectCotp.h"
#include "ApiProtectCutp.h"
#include "ApiProtectDotp.h"
#include "ApiProtectDutp.h"
#include "AppGauge.h"
#include "AppSerialUartDavinci.h"

#include "EBikeUartProtocol.h"
#include "LibSwTimer.h"
#include "LibNtc.h"
#include "smp_ADS7946_Driver.h"
#include "smp_w5500_DMA.h"
#include "smp_MX25L_Driver.h"

#if 0
#define	BSP_UART2_TEST
#endif

#if 0
#define	TCPIP_SOCKET_TEST
#endif

#if 1
#define	FU_OTHER_SCU_TEST
#endif

#ifdef BSP_UART2_TEST
#define	BSP_UART2_TX_BUF_SIZE	100
#define	BSP_UART2_RX_BUF_SIZE	100

static uint8_t uart_tx_buffer[BSP_UART2_TX_BUF_SIZE] = {0};
static uint8_t uart_rx_buffer[BSP_UART2_RX_BUF_SIZE] = {0};

#define BSP_UART2_PAR        {                                                                                \
                                .num                        = UART2,                                         \
                                .baud_rate                  = 19200,                                        \
                                .flow_ctrl                  = UART_FLOW_CTRL_DISABLE,                        \
                                .use_parity                 = PARITY_NONE,                                   \
                                .buffers.rx_buf             = uart_tx_buffer,                        \
                                .buffers.rx_buf_size        = BSP_UART2_TX_BUF_SIZE,                              \
                                .buffers.tx_buf             = uart_rx_buffer,                        \
                                .buffers.tx_buf_size        = BSP_UART2_RX_BUF_SIZE                               \
                          }  
smp_uart_t bsp_uart2_par = BSP_UART2_PAR;
#endif

#ifdef TCPIP_SOCKET_TEST

#define	TCPIP_TEST_BUF_SIZE					100
static uint8_t TcpipTestRxBuf[TCPIP_TEST_BUF_SIZE];
static uint8_t TcpipTestTxBuf[TCPIP_TEST_BUF_SIZE];


#define TCPIP_TEST_SOCKET1 {														\
					.Num						= NULL,						\
					.Protocol 					= Sn_MR_TCP,				\
					.PortNum  					= 1,						\
					.DeviceID					= 1,						\
					.Memory.rx_buf_Ptr          = TcpipTestRxBuf,					\
					.Memory.rx_buf_size         = TCPIP_TEST_BUF_SIZE,       	\
					.Memory.tx_buf_Ptr          = TcpipTestTxBuf,					\
					.Memory.tx_buf_size         = TCPIP_TEST_BUF_SIZE 		\
					}		
W5500_Socket_parm tcpipTestSocket1 = TCPIP_TEST_SOCKET1;

#define TCPIP_TEST_SOCKET2 {														\
					.Num						= NULL,						\
					.Protocol 					= Sn_MR_TCP,				\
					.PortNum  					= 2,						\
					.DeviceID					= 1,						\
					.Memory.rx_buf_Ptr          = TcpipTestRxBuf,					\
					.Memory.rx_buf_size         = TCPIP_TEST_BUF_SIZE,       	\
					.Memory.tx_buf_Ptr          = TcpipTestTxBuf,					\
					.Memory.tx_buf_size         = TCPIP_TEST_BUF_SIZE 		\
					}		
W5500_Socket_parm tcpipTestSocket2 = TCPIP_TEST_SOCKET2;

#define TCPIP_TEST_SOCKET3 {														\
					.Num						= NULL,						\
					.Protocol 					= Sn_MR_TCP,				\
					.PortNum  					= 3,						\
					.DeviceID					= 1,						\
					.Memory.rx_buf_Ptr          = TcpipTestRxBuf,					\
					.Memory.rx_buf_size         = TCPIP_TEST_BUF_SIZE,       	\
					.Memory.tx_buf_Ptr          = TcpipTestTxBuf,					\
					.Memory.tx_buf_size         = TCPIP_TEST_BUF_SIZE 		\
					}		
W5500_Socket_parm tcpipTestSocket3 = TCPIP_TEST_SOCKET3;

#define TCPIP_TEST_SOCKET4 {														\
					.Num						= NULL,						\
					.Protocol 					= Sn_MR_TCP,				\
					.PortNum  					= 4,						\
					.DeviceID					= 1,						\
					.Memory.rx_buf_Ptr          = TcpipTestRxBuf,					\
					.Memory.rx_buf_size         = TCPIP_TEST_BUF_SIZE,       	\
					.Memory.tx_buf_Ptr          = TcpipTestTxBuf,					\
					.Memory.tx_buf_size         = TCPIP_TEST_BUF_SIZE 		\
					}		
W5500_Socket_parm tcpipTestSocket4 = TCPIP_TEST_SOCKET4;

#define TCPIP_TEST_SOCKET5 {														\
					.Num						= NULL,						\
					.Protocol 					= Sn_MR_TCP,				\
					.PortNum  					= 5,						\
					.DeviceID					= 1,						\
					.Memory.rx_buf_Ptr          = TcpipTestRxBuf,					\
					.Memory.rx_buf_size         = TCPIP_TEST_BUF_SIZE,       	\
					.Memory.tx_buf_Ptr          = TcpipTestTxBuf,					\
					.Memory.tx_buf_size         = TCPIP_TEST_BUF_SIZE 		\
					}		
W5500_Socket_parm tcpipTestSocket5 = TCPIP_TEST_SOCKET5;

#endif


static uint8_t	SystemReadyFlag = 0;
tHalTimer	mHalTimer4={3, 1000};

void appSerialCanDavinciSendTextMessage(char *str);
#define	teatDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)


#ifdef TCPIP_SOCKET_TEST

static uint16_t tcpip_cb1(W5500_cb_type p_evt, uint16_t DataLen){
	static	uint32_t	offset = 0;
	static uint8_t cnt =0;
	char	str1[10];
	char	str[200];
	uint16_t	i;
	switch(p_evt){
		case W5500_DATA_RECV:
			offset += DataLen;
			sprintf(str, "Rcv1 W5500(%d %d):", DataLen, offset);
			teatDebugMsg(str);
			sprintf(TcpipTestTxBuf, "ret:%3d", cnt++);
			TcpipTestTxBuf[7] = 0;
			return 8;
			break;
		case W5500_Socket_REG_Success:
			teatDebugMsg("TCP/IP Success");
			break;
		case W5500_COMMUNICATE_ERR:
			teatDebugMsg("TCP/IP Error");
			break;
	}
	return 0;
}	

#endif

static void DebugGPIOInit(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	__HAL_RCC_GPIOE_CLK_ENABLE();
	GPIO_InitStructure.Pin = GPIO_PIN_10 | GPIO_PIN_15;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure); 

}

static void ADDS7946_Test_API1_callBack(uint8_t *pDat, uint8_t size){
	
}

static void ADDS7946_Test_API2_callBack(uint8_t *pDat, uint8_t size){
	

}
static void ADDS7946_Test_API1_SwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr){
	if(evt == LIB_SW_TIMER_EVT_SW_10MS_0){	
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);		
		smp_ADS7946_get_data(channel_1,CS_0,ADDS7946_Test_API1_callBack);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
	}

}
static void ADS7946_Test_API1_2_SwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr){
	static uint8_t Cnt_1S = 0;
	if(evt == LIB_SW_TIMER_EVT_SW_10MS_1){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
		smp_ADS7946_get_data(channel_1,CS_1,ADDS7946_Test_API2_callBack);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
	}
	if(evt == LIB_SW_TIMER_EVT_SW_1S){
		Cnt_1S++;
		if(Cnt_1S== 2){
			Cnt_1S = 0;
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
			smp_ADS7946_get_data(channel_1,CS_1,ADDS7946_Test_API2_callBack);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
		}
	}

}

static void ADDS7946_Test_API1_Open(void){
	smp_ADS7946_init();
	LibSwTimerOpen(ADDS7946_Test_API1_SwTimerHandler, 0);
}

static void ADDS7946_Test_API2_Open(void){
	smp_ADS7946_init();
	LibSwTimerOpen(ADS7946_Test_API1_2_SwTimerHandler, 0);
}

static void appProjectTestSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	static	uint8_t		SystemReadyCount = 10;
	char	str[100];

    if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
			
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		if(SystemReadyCount)
		{
			SystemReadyCount--;
			if(!SystemReadyCount)
				SystemReadyFlag = 1;
		}
	}
}

static void appProjectTestHwTimerHandler(void *pin, uint16_t evt, void *pData)
{
	LibSwTimerHwHandler(LIB_SW_TIMER_EVT_HW_1MS, 0);
	LibHwTimerHandle();
}
#ifdef BSP_UART2_TEST	

void bsp_uart2_cb(uart_evt_type p_evt)
{
  switch(p_evt){
    case UART_DATA_READY:
      /* received data handle */
    break;
    case UART_TX_EMPTY:
      /* Data transmission complete handle */
      halBspRs485RxEnable();
    break;
    case UART_COMMUNICATION_ERR:
      /* occurred during reception */
    break;
    case UART_BUFFER_FULL:
      /* occurred UART buffer full */
    break;  
    case UART_TX_READY_TO_SEND:
    	halBspRs485TxEnable();
    	break;  
    default:
    break;
  }
}
static uint8_t	res_count = 0;
//static uint8_t	
static void uart2_Response(void)
{
	uint8_t	i,ch;
	uint8_t	res[]={0x01,0x03,0x02,0x03,0x04,0xb9,0x77};

	halBspWatchDogEnable();

	for(i=0; i<7; i++)
			smp_uart_put(&bsp_uart2_par, res[i]);	
//void halBspWatchDogReset(void);
}

static void uart2_rcv(void)
{
	uint8_t	i,ch;
	char	str1[10];
	char	str[200]={0};
	uint8_t	buf[20] = {0};
	
	
	for(i=0; i<20; i++)
	{
		 if(smp_uart_get(&bsp_uart2_par, &ch) != SMP_SUCCESS)
		 	break;
		 buf[i] = ch;
		 if(str[0] == 0)
	 		sprintf(str,"RS485:");
		 sprintf(str1,"%.2X ",ch);
		 strcat(str, str1);
	}	
	if(str[0])
		teatDebugMsg(str);
	if(buf[0]==0x01 && buf[1] == 0x03)
	{ 	
		res_count = 5;	
	}
}
#endif

#ifdef FU_OTHER_SCU_TEST

#endif

static void testSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	uint8_t	buffer[10];
	uint8_t	i;
		
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
		
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_1)
	{
#ifdef BSP_UART2_TEST		
		GPIOC->ODR ^= GPIO_PIN_6;
		
		//for(i=0; i<10; i++)
			//buffer[i] = 0x90+i;
		//for(i=0; i<10; i++)
		//	smp_uart_put(&bsp_uart2_par, buffer[i]);
		uart2_rcv();
		if(res_count)
		{
			res_count--;
			if(!res_count)
				uart2_Response();		
		}
		halBspWatchDogReset();
#endif		
	}
}


//			smp_mx25l_flash_fast_read_data_bytes_addr(addr, &FwUpdateInfo.CodeBuf[0][0], 256, spiromEventHandler);




void appTestProjectOpen(void)
{
	
#if	0	
	DebugGPIOInit();
	HalTimerOpen(&mHalTimer4, appProjectTestHwTimerHandler);
	ADDS7946_Test_API1_Open();
	ADDS7946_Test_API2_Open();
#endif	
	
#ifdef BSP_UART2_TEST	

	if(smp_uart_init(&bsp_uart2_par, bsp_uart2_cb)==SMP_SUCCESS){
      	teatDebugMsg("smp uart initial success!");
	 }else{
     	teatDebugMsg("smp uart initial fail!");
  	}  


	LibSwTimerOpen(testSwTimerHandler, 0);
#endif

#ifdef TCPIP_SOCKET_TEST
#if	1
	if(W5500_Socket_Register(&tcpipTestSocket1 , tcpip_cb1) == SMP_SUCCESS)
		teatDebugMsg("Socket 1 open success");
	else
		teatDebugMsg("Socket 1 open fail"); 
#if	0	
	if(W5500_Socket_Register(&tcpipTestSocket2 , tcpip_cb1) == SMP_SUCCESS) 
		teatDebugMsg("Sock 2 open success");
	else
		teatDebugMsg("Sock 2 open fail"); 
	if(W5500_Socket_Register(&tcpipTestSocket3 , tcpip_cb1) == SMP_SUCCESS) 
		teatDebugMsg("Sock 3 open success");
	else
		teatDebugMsg("Sock 3 open fail"); 
	if(W5500_Socket_Register(&tcpipTestSocket4 , tcpip_cb1) == SMP_SUCCESS) 
		teatDebugMsg("Sock 4 open success");
	else
		teatDebugMsg("Sock 4 open fail"); 
	if(W5500_Socket_Register(&tcpipTestSocket5 , tcpip_cb1) == SMP_SUCCESS) 
		teatDebugMsg("Sock 5 open success");
	else
		teatDebugMsg("Sock 5 open fail"); 
#endif		
#endif		
#endif
}
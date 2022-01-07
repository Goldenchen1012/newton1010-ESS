/**
******************************************************************************
* @file    smp_W5500_DMA.h
* @author  Steve Cheng
* @version V0.0.2
* @date    2022/01/07
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
  
#ifndef _SMP_W5500_DMA_HAL_
#define _SMP_W5500_DMA_HAL_  
#include "Bsp.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "stm32l4xx_hal.h"
#include "smp_gpio.h"
#include "smp_spi_DMA.h"
#include "LibSwTimer.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>


#define W5500_MAX_SOCKET_NUM 8

#define W5500_SPI_USE_DMA

#define HW_RESET_DELAY_TIME_MS 50

//#define USE_DHCP


#define PCB_VER_1

/* STM32 Pin define */

#ifdef PCB_VER_1
#define W5500_RST_PORT 	GPIOE
#define W5500_RST_PIN	GPIO_PIN_1

#define W5500_INT_PORT 	GPIOB
#define W5500_INT_PIN	GPIO_PIN_7
#else
#define W5500_RST_PORT 	GPIOB
#define W5500_RST_PIN	GPIO_PIN_6

#define W5500_INT_PORT 	GPIOD
#define W5500_INT_PIN	GPIO_PIN_8
#endif
/* Default network information  */
#define MAC_ADDR_BYTE_LEN 6
#define GATE_WAY_ADDR_BYTE_LEN 4 
#define SUB_MASK_ADDR_BYTE_LEN 4
#define IP_ADDR_BYTE_LEN 4
#define NETINFO_RW_STEP 4
#define DEFAULT_MAC_ADDR {0x00, 0xf1, 0xbe, 0xc4, 0xa1, 0x05}
#define DEFAULT_IP_ADDR {192, 168, 1, 16}
#define DEFAULT_SUB_MASK {255, 255, 255, 0}
#define DEFAULT_GW_ADDR {192, 168, 1, 1}
#define DEFAULT_DNS_ADDR {8, 8, 8, 8}
#define DATA_BUF_MAX_SIZE 2000

#define SOCKET_NUM_BASE 0x1F
#define TEST_REG_ADDR  0x00
typedef struct{
	uint32_t AddrSel;
	uint16_t Len;
	uint8_t *pData;
}R_W_NetInfo;

typedef enum{
	W5500_DATA_RECV = 0,
	W5500_Socket_REG_Success,
	W5500_COMMUNICATE_ERR
}W5500_cb_type;

typedef struct{
	uint8_t					*rx_buf_Ptr;
	uint16_t				rx_buf_size;			/* Size of the RX buffer */
	uint8_t					*tx_buf_Ptr;
	uint16_t				tx_buf_size;			/* Size of the TX buffer */
}w5500_socket_buffer_t;

typedef struct{
	uint8_t					Num;
	uint8_t					Protocol;	//MACRAW mode should be only used in Socket 0. Protocol include Sn_MR_MACRAW Sn_MR_IPRAW Sn_MR_UDP Sn_MR_TCP Sn_MR_CLOSE
	uint16_t 				PortNum;
	uint8_t					DeviceID;
	w5500_socket_buffer_t 	Memory;
	uint16_t				DestPort;
}W5500_Socket_parm;

typedef uint16_t (*smp_w5500_event_t)(W5500_cb_type p_W5500event, uint16_t DataLen);

typedef struct{
	uint8_t SocketStatus;
	smp_w5500_event_t cbFunPtr;	
	W5500_Socket_parm Parm;

}W5500_RegisterList;	
/********** W5500 Init Step **********/
typedef enum{
	Init_HW_Reset_1,
	Init_HW_Reset_2,
	Init_SW_Reset,
	Init_PHY_Conf,
	Init_PHY_Read,
	Init_PHY_Rst,
	Init_PHY_Verify_Read,
	Init_PHY_Verify,
	Init_SET_NetInfo,
	Init_Read_NetInfo,
	Init_Verify_NetInfo,
	Init_SET_SUBMask,
	Init_SET_LocalIP,
	Init_INT_MASK_Config,
	Init_SocketNum_INT_MASK_Config,
	Init_SocketEvent_INT_MASK_Config,
	Init_End
}W5500_init_step;
/*************************************/
/********* W5500 Server Step *********/
typedef enum{
	/******* Socket reset and open *******/
	Server_Read_Socket_Status,
	Server_Check_Socket_Status,
	Socket_Open_Read_IP,
	Socket_Open_Check_IP,
	Set_Socket_Close,
	Read_Cmd_Register_Status_0,
	Check_Close_Cmd_Recv,
	RST_All_INT_Flag,
	Read_Socket_Status_0,
	Verify_Socket_Close_0,
	Set_Socket_Mode,
	Set_Socket_Port_Highbyte,
	Set_Socket_Port_Lowbyte,
	Set_Socket_Open,
	Read_Cmd_Register_Status_1,
	Check_Open_Cmd_Recv,
	Read_Socket_Status_1,
	Verify_Socket_Close_1,
	/******* Socket listen *******/	
	Socket_Listen_Read_Mode,
	Socket_Listen_Verify_Mode,
	Set_Socket_Listen,
	Read_Cmd_Register_Status_2,
	Check_Listen_Cmd_Recv,
	Read_Socket_Status_2,
	Verify_Socket_Listen,
	/****** Socket establish *****/	
	Read_SIR_Register,
	Check_SocketNum_Int,
	Read_SIR_End,
	Read_Socket_INT_event,
	Verify_Connect_Event,
	Clear_CON_INT_Flag,
	Read_Remote_IP,
	Read_Remote_Port_HighByte,
	Save_High_Byte_Read_Remote_Port_LowByte,
	Save_Port_Low_Byte,
	Read_Recv_Cnt_HighByte,
	Read_Recv_Cnt_LowByte,
	Check_Recv_Data,
	Read_Buf_Max_Len,
	Check_Oversize_Event,
	Read_Socket_Buf_Addr_HighByte,
	Read_Socket_Buf_Addr_LowByte,
	Read_Socket_Buf_data,
	ParserData,
	Update_Buf_Offset_Highbyte,
	Update_Buf_Offset_Lowbyte,
	Set_Scoket_Recv,
	Read_Cmd_Register_Status_3,
	Check_Recv_Cmd_Recv,
	Clear_Recv_INT_Flag,
	Read_Tx_Buf_Max_Len,
	Check_Response_Oversize,
	Read_W5500_Tx_FSR_High,
	Read_W5500_Tx_FSR_Low,
	Read_W5500_Tx_Ptr_Highbyte,
	Read_W5500_Tx_Ptr_Lowbyte,
	Get_W5500_Tx_Ptr_Addr_and_Trans,
	Update_Tx_Ptr_Highbyte,
	Update_Tx_Ptr_Lowbyte,
	Set_Socket_Send,
	Read_Cmd_Register_Status_4,
	Check_Send_Cmd_Recv,
	Read_Socket_INT_event_1,
	Clear_SendOK_INT_Flag,
	Verify_Send_End,
	/****** Socket disconnect *****/		
	Set_Socket_Disconnect,
	Read_Cmd_Register_Status_5,
	Check_Discon_Cmd_Recv,
	Read_Socket_INT_event_2,
	Verify_Disconnct_End,
	Clear_DISCON_INT_Flag,
	Server_End,
	//----------------
	Read_Socket_INT_Event_New_1,
	Check_INT_Event,
}W5500_server_step;


/*************************************/
enum{
	W5500_SPI_Idle = 0,
	W5500_SPI_Busy,
	W5500_SPI_Done
};

enum{
	Socket_Disable = 0,
	Socket_Enable,
	Socket_Close,
	Socket_Open
};
enum{
	Event_Handle_Done,
	Event_Handle_Ing
};

enum{
	W5500_Status_Reset,
	W5500_Status_Init_End,
	W5500_Status_Server_Open
};

int8_t W5500_Init_Step(uint8_t Step);
int8_t W5500_Server_Step(uint8_t Step, uint8_t SocketNum);
int8_t W5500_Server_Step_New(uint8_t Step, uint8_t SocketNum);
int8_t W5500_Socket_Register(W5500_Socket_parm *parm, smp_w5500_event_t w5500_event_Handler);
int8_t W5500_Read_Socket_INTReg(void);
int8_t W5500_Socket_Reopen_Step(uint8_t Step, uint8_t SocketNum);


void Hal_W5500_Open(void);

extern smp_gpio_t		PB12;
extern smp_spi_cs_t 	W5500_CS;
extern smp_spi_t 		SPI_W5500;

extern volatile bool W5500_Read_SnIR_End;
extern volatile uint16_t W5500_INT_Cnt;

#endif

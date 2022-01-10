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


int8_t W5500_Init_Step(uint8_t Step);
int8_t W5500_Server_Step(uint8_t Step, uint8_t SocketNum);
int8_t W5500_Server_Step_New(uint8_t Step, uint8_t SocketNum);
int8_t W5500_Socket_Register(W5500_Socket_parm *parm, smp_w5500_event_t w5500_event_Handler);
int8_t W5500_Read_Socket_INTReg(void);
int8_t W5500_Socket_Reopen_Step(uint8_t Step, uint8_t SocketNum);


void Hal_W5500_Open(void);

extern volatile bool W5500_Read_SnIR_End;
extern volatile uint16_t W5500_INT_Cnt;

#endif

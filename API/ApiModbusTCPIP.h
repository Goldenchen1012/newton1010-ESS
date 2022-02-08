/**
******************************************************************************
* @file    ApiModbusTCPIP.h
* @author  Steve Cheng
* @version V0.0.1
* @date    2021/12/17
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
#ifndef _SMP_MODBUS_TCPIP_
#define _SMP_MODBUS_TCPIP_

#include "smp_w5500_DMA.h"
#include "halafe.h"
#include "AppGauge.h"
#include "LibNtc.h"
#include "ApiScuTemp.h"
#include "AppBms.h"


void Modbus_TCPIP_Socket_Open(void);
uint16_t Modbus_Parser(W5500_Socket_parm Socket, uint16_t DataLen);
uint8_t ModbusTCPIP_Func_ReadHoldReg( uint16_t xu16StartRegAddr, uint16_t xu16ReadRegCnt, W5500_Socket_parm Socket);
uint8_t ModbusTCPIP_Func_WriteMultiReg(uint16_t xu16StartRegAddr, uint16_t xu16WriteRegCnt, W5500_Socket_parm Socket);
extern uint8_t ModbusTCPIP_buffer[DATA_BUF_MAX_SIZE];
#endif

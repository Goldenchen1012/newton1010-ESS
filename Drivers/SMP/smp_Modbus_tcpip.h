/**
******************************************************************************
* @file    smp_modbus_TCPIP.h
* @author  Steve Cheng
* @version V0.0.1
* @date    2021/12/10
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

#define MODBUS_TCPIP_SOCKET_PORT			502
/* MODBUS TCP/IP Standard protocol */
#define MODBUS_TCPIP_SOCKET_PORT			502
#define MODBUS_TCPIP_READ_WRITE_MAXWORDCNT  127  //  Single communication of Max Word Count are 127-3.5 (header+ ID 7 bytes ) words  




/* TCP/IP Data frame protocal offset */
#define MODBUS_TCPIP_HEADER_LEN         6
#define MODBUS_TCPIP_PROTOCOLID_OFFSET 	2
#define MODBUS_TCPIP_PROTOCOLID		   	0x0000

#define MODBUS_TCPIP_MASSAGELEN_OFFSET  4
#define MODBUS_TCPIP_UNITID_OFFSET  	6
#define MODBUS_TCPIP_PDU_OFFSET			7
#define MODBUS_TCPIP_REG_CNT_OFFSET		10
/*TCP/IP Function code ---- Standard */
#define READ_COIL_STATUS        0x01
#define READ_INPUT_STATUS       0x02
#define READ_HOLDING_REG        0x03
#define READ_INPUT_REG          0x04
#define FORCE_SINGLE_COIL       0x05
#define PRESET_SINGLE_REG       0x06
#define FORCE_MULTIPLE_COIL     0x0F  
#define PRESEET_MULTIPLE_REG    0x10
/*TCP/IP Function code ---- Private */


/* TCP/IP Function code Response Protocol */
#define MODBUS_TCPIP_NORMAL_RES_COPY_LEN	8
//Read Holding Register
#define READ_HOLDING_REG_RES_HEADER_LEN		3		//(ID 1, FUNC code 1 byteCnt 1)
#define READ_HOLDING_REG_RES_BYTECNT_OFFSET 8		
#define READ_HOLDING_REG_RES_DATA_OFFSET	9

//Write Multiple register
#define WRITE_MULTIPLE_REGISTER_DATA_OFFSET		14
#define WRITE_MULTIPLE_REGISTER_BYTECNT_OFFSET	12
#define WRITE_MULTIPLE_REGISTER_WORDCNT_OFFSET	10
#define WRITE_MULTIPLE_REGISTER_RES_LEN		6
/*TCP/IP expection code*/
#define MODBUS_TCPIP_EXCEPTION_FUN_CODE 	0x80
#define MODBUS_TCPIP_EXCEPTION_RES_LEN 		3
#define MODBUS_TCPIP_EXCEPTION_DATA_OFFSET  8

enum{
	HIGH_Byte = 0,
	LOW_Byte
};

enum{
    Normal = 0,
    Illegal_Function,
    Illegal_DataAddr,
    Illegal_DataValue,
    Slave_DeviceFail,
    Acknowledge,
    Slave_Device_Busy,
    Negative_Acknowledge,
    Memory_Parity_Error,
    Gateway_Path_Unavail = 10,
    Gateway_Target_ResponseFail
};

void Modbus_TCPIP_Socket_Open(void);
uint16_t Modbus_TCPIP_Parser(W5500_Socket_parm Socket, uint16_t DataLen);
uint8_t ModbusTCPIP_Func_ReadHoldReg( uint16_t xu16StartRegAddr, uint16_t xu16ReadRegCnt, W5500_Socket_parm Socket);
uint8_t ModbusTCPIP_Func_WriteMultiReg(uint16_t xu16StartRegAddr, uint16_t xu16WriteRegCnt, W5500_Socket_parm Socket);
extern uint8_t ModbusTCPIP_buffer[DATA_BUF_MAX_SIZE];
#endif
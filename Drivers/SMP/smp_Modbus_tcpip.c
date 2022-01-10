/**
******************************************************************************
* @file    smp_modbus_TCPIP.c
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
#include "smp_Modbus_tcpip.h"
uint8_t ModbusTCPIP_buffer[DATA_BUF_MAX_SIZE];
uint8_t ResponseLength;
#define SOCKET_EMS {														\
					.Num						= NULL,						\
					.Protocol 					= Sn_MR_TCP,				\
					.PortNum  					= MODBUS_TCPIP_SOCKET_PORT,	\
					.DeviceID					= 1,						\
					.Memory.rx_buf_Ptr          = ModbusTCPIP_buffer,		\
					.Memory.rx_buf_size         = DATA_BUF_MAX_SIZE,       	\
					.Memory.tx_buf_Ptr          = ModbusTCPIP_buffer,		\
					.Memory.tx_buf_size         = DATA_BUF_MAX_SIZE 		\
					}

#if 1
W5500_Socket_parm Socket_ModbusTCPIP = SOCKET_EMS;	
uint8_t ModbusTest1_buffer[DATA_BUF_MAX_SIZE];
#define SOCKET_TEST1 {														\
					.Num						= NULL,						\
					.Protocol 					= Sn_MR_TCP,				\
					.PortNum  					= 5000,						\
					.DeviceID					= 1,						\
					.Memory.rx_buf_Ptr          = ModbusTest1_buffer,		\
					.Memory.rx_buf_size         = DATA_BUF_MAX_SIZE,       	\
					.Memory.tx_buf_Ptr          = ModbusTest1_buffer,		\
					.Memory.tx_buf_size         = DATA_BUF_MAX_SIZE 		\
					}		
W5500_Socket_parm Socket_Test1 = SOCKET_TEST1;	
uint8_t ModbusTest2_buffer[DATA_BUF_MAX_SIZE];
#define SOCKET_TEST2 {														\
					.Num						= NULL,						\
					.Protocol 					= Sn_MR_TCP,				\
					.PortNum  					= 5200,						\
					.DeviceID					= 1,						\
					.Memory.rx_buf_Ptr          = ModbusTest2_buffer,		\
					.Memory.rx_buf_size         = DATA_BUF_MAX_SIZE,       	\
					.Memory.tx_buf_Ptr          = ModbusTest2_buffer,		\
					.Memory.tx_buf_size         = DATA_BUF_MAX_SIZE 		\
					}		
W5500_Socket_parm Socket_Test2 = SOCKET_TEST2;	
#endif
/******************************************************************************
* @version 	V0.0.1
* @date    	2021/11/24
* @brief   	ModbusTCPIP call back function 
* @param	w5500 call back event and parser data length
* @ret		retuen response dataframe length
******************************************************************************/
uint16_t ModbusTCPIP_cb(W5500_cb_type p_evt, uint16_t DataLen){
	switch(p_evt){
		case W5500_DATA_RECV:
			return Modbus_TCPIP_Parser(Socket_ModbusTCPIP,DataLen);
			break;
		case W5500_Socket_REG_Success:
			break;
		case W5500_COMMUNICATE_ERR:
			break;
	}
	return 0;
}					
uint16_t ModbusTest1(W5500_cb_type p_evt, uint16_t DataLen){
	switch(p_evt){
		case W5500_DATA_RECV:
			return Modbus_TCPIP_Parser(Socket_Test1,DataLen);
			break;
		case W5500_Socket_REG_Success:
			break;
		case W5500_COMMUNICATE_ERR:
			break;
	}
	return 0;
}					
uint16_t ModbusTest2(W5500_cb_type p_evt, uint16_t DataLen){
	switch(p_evt){
		case W5500_DATA_RECV:
			return Modbus_TCPIP_Parser(Socket_Test2,DataLen);
			break;
		case W5500_Socket_REG_Success:
			break;
		case W5500_COMMUNICATE_ERR:
			break;
	}
	return 0;
}	
/********** Byte Word Convert **********/
uint16_t ByteToWord(uint8_t highbyte,uint8_t lowbyte){
	uint16_t ret;
	ret = ((((uint16_t)highbyte)<<8)&0xFF00) + ((uint16_t)lowbyte&0x00FF);
	return ret;
}

uint8_t WordToByte(uint16_t xu16Data, uint8_t xu8Type){
	uint8_t ret;
	switch(xu8Type){
		case HIGH_Byte:
			ret = (uint8_t)((xu16Data&0xFF00)>>8);	
			break;
		case LOW_Byte:
			ret = (uint8_t)(xu16Data&0x00FF);	
			break;
	}
	return ret;
}


void Modbus_TCPIP_Socket_Open(void){
	/* Register socket and caall back function */
//	W5500_Socket_Register(&Socket_ModbusTCPIP, ModbusTCPIP_cb);   
//	W5500_Socket_Register(&Socket_Test1 , ModbusTest1); 
//	W5500_Socket_Register(&Socket_Test2 , ModbusTest2);   	
}


uint16_t Modbus_TCPIP_Parser(W5500_Socket_parm Socket, uint16_t DataLen){
	uint8_t *TempDataPtr,lu8ExceptionCode = Normal;
	uint16_t lu16FunctionCode,lu16StartRegAddr, lu16RegCnt;
	
	/* Offset to Protocal byte offset */
	TempDataPtr = Socket.Memory.rx_buf_Ptr + MODBUS_TCPIP_PROTOCOLID_OFFSET;
	/* Confirm receive data frame protocal is TCP/IP */
	if(ByteToWord(*TempDataPtr,*(TempDataPtr+1))!= MODBUS_TCPIP_PROTOCOLID){
		//SEGGER_RTT_printf(0,"Protocol ID not TCPIP, recv %x \r\n",ByteToWord(*TempDataPtr,*(TempDataPtr+1)));
		lu8ExceptionCode = Gateway_Target_ResponseFail;
	}else{
		/* Check message length  */
		TempDataPtr = Socket.Memory.rx_buf_Ptr + MODBUS_TCPIP_MASSAGELEN_OFFSET;
		if(ByteToWord(*TempDataPtr,*(TempDataPtr+1))!= (DataLen-MODBUS_TCPIP_HEADER_LEN)){
			lu8ExceptionCode = Illegal_DataValue;
		}
		/* Confirm Unit ID byte */
		TempDataPtr = Socket.Memory.rx_buf_Ptr + MODBUS_TCPIP_UNITID_OFFSET;
		if(*TempDataPtr != Socket.DeviceID ){
			lu8ExceptionCode = Gateway_Target_ResponseFail;
		}	
		if(lu8ExceptionCode == Normal){
			/* According to function code, response data or overwrite valaue at register */
			TempDataPtr = Socket.Memory.rx_buf_Ptr + MODBUS_TCPIP_PDU_OFFSET;
			lu16FunctionCode = *TempDataPtr;
			lu16StartRegAddr = ByteToWord(*(TempDataPtr+1),*(TempDataPtr+2));
			lu16RegCnt = ByteToWord(*(TempDataPtr+3),*(TempDataPtr+4));
			//SEGGER_RTT_printf(0,"Function code : %d  Start addr: %d  RegCnt : %d\r\n",lu16FunctionCode, lu16StartRegAddr, lu16RegCnt);
			//SEGGER_RTT_printf(0,"Socket Num : %d\r\n",Socket.Num);
			switch(lu16FunctionCode){
				case READ_HOLDING_REG:
					lu8ExceptionCode = ModbusTCPIP_Func_ReadHoldReg(lu16StartRegAddr, lu16RegCnt, Socket);
					break;
				case PRESEET_MULTIPLE_REG:
					lu8ExceptionCode =  ModbusTCPIP_Func_WriteMultiReg(lu16StartRegAddr, lu16RegCnt, Socket);
					break;
				default:
					lu8ExceptionCode = Illegal_Function;
					break;
			}
		}
		/********** Fill response data frame (Trans ID, Protocol ID, and Unit ID are as same as recv data) **********/
		
		/* Handdle Exception response data */
		if(lu8ExceptionCode != Normal){
			/* Set response massage length */
			TempDataPtr = Socket.Memory.tx_buf_Ptr + MODBUS_TCPIP_MASSAGELEN_OFFSET;
			*TempDataPtr = WordToByte(MODBUS_TCPIP_EXCEPTION_RES_LEN, HIGH_Byte);
			*(TempDataPtr+1) = WordToByte(MODBUS_TCPIP_EXCEPTION_RES_LEN, LOW_Byte);
			/* Set response function code */
			TempDataPtr = Socket.Memory.tx_buf_Ptr + MODBUS_TCPIP_PDU_OFFSET;
			*TempDataPtr |= MODBUS_TCPIP_EXCEPTION_FUN_CODE;
			/* Set data */
			TempDataPtr = Socket.Memory.tx_buf_Ptr + MODBUS_TCPIP_EXCEPTION_DATA_OFFSET;
			*TempDataPtr = lu8ExceptionCode;

			ResponseLength = MODBUS_TCPIP_EXCEPTION_RES_LEN+MODBUS_TCPIP_HEADER_LEN;
			return ResponseLength;
		/* Handle normal response data */
		}else{
			/* Set massage length of response */
			TempDataPtr = Socket.Memory.tx_buf_Ptr + MODBUS_TCPIP_MASSAGELEN_OFFSET;
			*TempDataPtr = WordToByte(ResponseLength, HIGH_Byte);
			*(TempDataPtr+1) = WordToByte(ResponseLength, LOW_Byte);
			/* Update total length of response frame */
			ResponseLength+= MODBUS_TCPIP_HEADER_LEN;
		}
	}
	
	return ResponseLength;
}



/******************************************************************************
* @version 	V0.0.1
* @date    	2021/11/23
* @brief   	For MODBUS TCP/IP Function code : Read_Holding_register
*			Command protocol : (PDU) ID, Function code, First Register address,  Total Read register count ("word count")
*			Response protocol: (PDU) ID, Function code, response data "byte count", data
*			Communication example : Command : 11, 03, 006B, 0003 
*									Response: 11, 03, 06, 0102, 0304, 0506
* @param	Receive Data start pointer, Register address, Read word amount
* @ret		expection code
******************************************************************************/
uint8_t ModbusTCPIP_Func_ReadHoldReg( uint16_t xu16StartRegAddr, uint16_t xu16ReadRegCnt, W5500_Socket_parm Socket){
	uint8_t *TempDataPtr;
	uint16_t lu16ByteCnt,lu16EndRegAddr,i;
	/* lu16EndRegAddr = BMS info size + SCU info size */
	lu16EndRegAddr = 0x40; //For test
	/* Check ask addr and length wheather over max Len */
	if(xu16ReadRegCnt > MODBUS_TCPIP_READ_WRITE_MAXWORDCNT){
		return Illegal_DataValue;
	}else if((xu16StartRegAddr+xu16ReadRegCnt)>lu16EndRegAddr){
		return Illegal_DataAddr;
	}else{
	/* Fill response data frame */	
		lu16ByteCnt = xu16ReadRegCnt*2;
		TempDataPtr = Socket.Memory.tx_buf_Ptr + READ_HOLDING_REG_RES_BYTECNT_OFFSET;
		*TempDataPtr = (uint8_t)lu16ByteCnt;
		/* Copy data from mapping memory */
		TempDataPtr = Socket.Memory.tx_buf_Ptr + READ_HOLDING_REG_RES_DATA_OFFSET;   
		for(i = 0; i<lu16ByteCnt; i++){
			*(TempDataPtr+i) = 0;  // Should be change to mapping memory array e.g. : Data[xu16StartRegAddr+i]
		}	
	}
	ResponseLength = lu16ByteCnt+READ_HOLDING_REG_RES_HEADER_LEN;
	return Normal;
}
  
/******************************************************************************
* @version 	V0.0.1
* @date    	2021/11/23
* @brief   	For MODBUS TCP/IP Function code : Preset multiple register
*			Command protocol : (PDU) ID, Function code, First Register address, write register count ("word count"), write byte count, Data
*			Response protocol: (PDU) ID, Function code, First Register address, word count
*			Communication example : Command : 11, 10, 0001, 0002, 04, 000A, 0102
*									Response: 11, 10, 0001, 0002
* @param	Receive Data start pointer, Register address, Read word amount
* @ret		expection code
******************************************************************************/
uint8_t ModbusTCPIP_Func_WriteMultiReg(uint16_t xu16StartRegAddr, uint16_t xu16WriteRegCnt, W5500_Socket_parm Socket){
	uint8_t *TempDataPtr, lu8ByteCnt;

	/* Get write byte Count from client command */
	TempDataPtr = Socket.Memory.rx_buf_Ptr + WRITE_MULTIPLE_REGISTER_BYTECNT_OFFSET;
	lu8ByteCnt = *TempDataPtr;
	
	//SEGGER_RTT_printf(0,"Write Reg Cnt: %d  Byte Cnt %d\r\n",xu16WriteRegCnt,lu8ByteCnt);
	if((xu16WriteRegCnt > MODBUS_TCPIP_READ_WRITE_MAXWORDCNT)||(xu16WriteRegCnt == 0)||((xu16WriteRegCnt*2) != (uint16_t)lu8ByteCnt)){
		return Illegal_DataValue;
	}else{
		switch( xu16StartRegAddr){
		case TEST_REG_ADDR:
			// Data pointer start at TempDataPtr = Socket_ModbusTCPIP.buffers.tx_buf + WRITE_MULTIPLE_REGISTER_DATA_OFFSET;
			break;
		default:
			return Illegal_DataAddr;
			break;
		}
	}
	ResponseLength = WRITE_MULTIPLE_REGISTER_RES_LEN;
	return Normal;
	
}




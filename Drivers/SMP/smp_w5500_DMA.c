/**
******************************************************************************
* @file    smp_W5500_DMA.c
* @author  Steve Cheng
* @version V0.0.4
* @date    2022/01/17
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

#include "smp_w5500_DMA.h"
#include <stdbool.h>
#include <string.h>
#define WORD_BYTE_LEN	2

#define _W5500_SPI_VDM_OP_          0x00
#define _W5500_SPI_FDM_OP_LEN1_     0x01
#define _W5500_SPI_FDM_OP_LEN2_     0x02
#define _W5500_SPI_FDM_OP_LEN4_     0x03
#define SOCK_ANY_PORT_NUM  0xC000	

#define DEFAULT_MAC_ADDR {0x00, 0xf1, 0xbe, 0xc4, 0xa1, 0x05}
#define DEFAULT_IP_ADDR {192, 168, 1, 16}
#define DEFAULT_SUB_MASK {255, 255, 255, 0}
#define DEFAULT_GW_ADDR {192, 168, 1, 1}
#define DEFAULT_DNS_ADDR {8, 8, 8, 8}
static uint8_t DefMac[MAC_ADDR_BYTE_LEN] = DEFAULT_MAC_ADDR;
static uint8_t DefIp[IP_ADDR_BYTE_LEN] = DEFAULT_IP_ADDR;
static uint8_t DefSn[SUB_MASK_ADDR_BYTE_LEN] = DEFAULT_SUB_MASK;
static uint8_t DefGw[GATE_WAY_ADDR_BYTE_LEN] = DEFAULT_GW_ADDR;
static uint8_t DefDns[IP_ADDR_BYTE_LEN] = DEFAULT_DNS_ADDR;

wiz_NetInfo TempNetInfo,DefNetInfo,RemoteNetInfo;

typedef enum{
	Send_Command,
	Read_Command_Reg,
	Verify_Command_Recv
}Socket_Command_Step;
static Socket_Command_Step Send_Command_Step = Send_Command;

typedef enum{
	Send_Ing,
	Send_Done
}SendCommand_Status;

typedef enum{
	Config_Parm,
	Read_Reg,
	Verify_Parm
}Config_and_Verify_Step;
static Config_and_Verify_Step Ini_Reg_Step = Config_Parm;

typedef enum{
	Config_Ing,
	Config_Done,
	Config_Fail
}ConfigStatus;

typedef enum{
	Sock_Increase_Ing,
	Sock_Num_Reset
}Sock_Index_Cycle_Status;

typedef struct{
	uint32_t AddrSel;
	uint8_t Len;
}R_W_NetInfo;

enum{
	W_R_MAC_Addr,
	W_R_Gate,
	W_R_SUB_MASK,
	W_R_IP_Addr
};


const R_W_NetInfo ConfNetInfoAddrTable[4] = {
	{SHAR , MAC_ADDR_BYTE_LEN},
	{GAR, GATE_WAY_ADDR_BYTE_LEN},
	{SUBR, SUB_MASK_ADDR_BYTE_LEN},
	{SIPR, IP_ADDR_BYTE_LEN}
};
static uint8_t NetInfoConfIndex = 0;

typedef enum{
	SPI_Idle = 0,
	SPI_Busy,
	SPI_Done
}W5500_SPI_Status;
W5500_SPI_Status gSPI_Status;

typedef enum{
	Socket_Disable = 0,
	Socket_Enable,
	Socket_Close,
	Socket_Open
}W5500_Socket_Status;

typedef struct{
	W5500_Socket_Status SocketStatus;
	smp_w5500_event_t cbFunPtr;	
	W5500_Socket_parm Parm;
	wiz_NetInfo Remote;
}W5500_RegisterList;

W5500_RegisterList smpW5500Socket[W5500_MAX_SOCKET_NUM];

smp_gpio_t		W5500_RST_PIN;
smp_gpio_t		W5500_INT_PIN;
smp_gpio_state 	W5500_INT_PIN_State;
smp_gpio_t		W5500_SPI_CS_PIN;
smp_spi_cs_t 	W5500_CS;
smp_spi_t 		SPI_W5500;


uint8_t W5500_tx_Buf[BSP_SPI2_TX_BUFFER_SIZE];
uint8_t Readbyte_value;
uint8_t ReadWord_value[2];
uint8_t SocketEnableCount;

uint16_t SocketHandleByteLen[W5500_MAX_SOCKET_NUM];
uint16_t RX_Buf_Addr;
uint16_t TX_Buf_Addr;
uint32_t SPI_Command_Frame;
static uint8_t ms_Count = 0;
static uint8_t gSocketNum = 0;
wiz_PhyConf PHYconf, PHYconf_Verify;
wiz_NetInfo TempNetInfo;

static bool Flag_Check_INT = false;

static char str[100];
void appSerialCanDavinciSendTextMessage(char *msg);

#define	W5500Debug(str)	appSerialCanDavinciSendTextMessage(str)
/**
  * @brief This function handles W5500 INT EXTI Pin interrupt.
  */
void EXTI9_5_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(SMP_GPIO_PIN(BSP_W5500_INT_PIN));
}

void smp_w5500_spiDMA_write_byte(uint32_t AddrSel, uint8_t Value){
	gSPI_Status = SPI_Busy;
	AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);
	W5500_tx_Buf[0] = (AddrSel & 0x00FF0000) >> 16;
	W5500_tx_Buf[1] = (AddrSel & 0x0000FF00) >> 8;
	W5500_tx_Buf[2] = (AddrSel & 0x000000FF) >> 0;
	W5500_tx_Buf[3] = Value;
	smp_spi_master_send_recv(&SPI_W5500, W5500_tx_Buf, 4, 0, 0, &W5500_CS);

}

void smp_w5500_spiDMA_read_byte(uint32_t AddrSel){	
	gSPI_Status = SPI_Busy;
	AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);
	W5500_tx_Buf[0] = (AddrSel & 0x00FF0000) >> 16;
	W5500_tx_Buf[1] = (AddrSel & 0x0000FF00) >> 8;
	W5500_tx_Buf[2] = (AddrSel & 0x000000FF) >> 0;
	smp_spi_master_send_recv(&SPI_W5500, W5500_tx_Buf, 3, &Readbyte_value, 1, &W5500_CS);
	
}

void smp_w5500_spiDMA_write_word(uint32_t AddrSel, uint16_t Value){
	gSPI_Status = SPI_Busy;
	AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);
	W5500_tx_Buf[0] = (AddrSel & 0x00FF0000) >> 16;
	W5500_tx_Buf[1] = (AddrSel & 0x0000FF00) >> 8;
	W5500_tx_Buf[2] = (AddrSel & 0x000000FF) >> 0;
	W5500_tx_Buf[3] = (uint8_t)(Value>>8);
	W5500_tx_Buf[4] = (uint8_t)Value;
	smp_spi_master_send_recv(&SPI_W5500, W5500_tx_Buf, 5, 0, 0, &W5500_CS);
}

void smp_w5500_spiDMA_read_word(uint32_t AddrSel){
	gSPI_Status = SPI_Busy;
	AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);
	W5500_tx_Buf[0] = (AddrSel & 0x00FF0000) >> 16;
	W5500_tx_Buf[1] = (AddrSel & 0x0000FF00) >> 8;
	W5500_tx_Buf[2] = (AddrSel & 0x000000FF) >> 0;
	smp_spi_master_send_recv(&SPI_W5500, W5500_tx_Buf, 3, &ReadWord_value[0], 2, &W5500_CS);
}

void smp_w5500_spiDMA_WriteMulti(uint32_t AddrSel, uint8_t* pBuf,uint16_t Len){
	uint8_t i;
	gSPI_Status = SPI_Busy;
	 AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);
	W5500_tx_Buf[0] = (AddrSel & 0x00FF0000) >> 16;
	W5500_tx_Buf[1] = (AddrSel & 0x0000FF00) >> 8;
	W5500_tx_Buf[2] = (AddrSel & 0x000000FF) >> 0;
	for(i=0;i<Len;i++){
		W5500_tx_Buf[i+3] = pBuf[i];
	}
	smp_spi_master_send_recv(&SPI_W5500, W5500_tx_Buf, Len+3, 0, 0, &W5500_CS);
	
}

void smp_w5500_spiDMA_ReadMulti(uint32_t AddrSel, uint8_t* Ptr, uint16_t Len){
	gSPI_Status = SPI_Busy;
	AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);
	W5500_tx_Buf[0] = (AddrSel & 0x00FF0000) >> 16;
	W5500_tx_Buf[1] = (AddrSel & 0x0000FF00) >> 8;
	W5500_tx_Buf[2] = (AddrSel & 0x000000FF) >> 0;
	smp_spi_master_send_recv(&SPI_W5500, W5500_tx_Buf, 3, Ptr, Len, &W5500_CS);
	
}

void spi_master_w5500_event_handler(smp_spi_evt_type p_evt)
{
	switch(p_evt){
		case SMP_SPI_EVENT_DONE:
			gSPI_Status = SPI_Idle;
			break;
		case SMP_SPI_EVENT_TRANSFER_BUSY:
			gSPI_Status = SPI_Busy;	
			break;
		case SMP_SPI_EVENT_TRANSFERR_ERROR:
			break;
		default:
			break;
	}
}


uint8_t w5500_phyconf_transfer_to_uint8(wiz_PhyConf phyconf){
	uint8_t ret = 0;
	if(phyconf.by == PHY_CONFBY_SW)
		ret |= PHYCFGR_OPMD;
	else
		ret &= ~PHYCFGR_OPMD;
	if(phyconf.mode == PHY_MODE_AUTONEGO)
		ret |= PHYCFGR_OPMDC_ALLA;
	else{
		if(phyconf.duplex == PHY_DUPLEX_FULL){
			if(phyconf.speed == PHY_SPEED_100)
				ret |= PHYCFGR_OPMDC_100F;
			else
				ret |= PHYCFGR_OPMDC_10F;
		}else{
			if(phyconf.speed == PHY_SPEED_100)
				ret |= PHYCFGR_OPMDC_100H;
			else
				ret |= PHYCFGR_OPMDC_10H;
		}
	}
	return ret;
} 
wiz_PhyConf w5500_uint8_transfer_to_phyconf(uint8_t value){
	wiz_PhyConf ret;
	ret.by = (value & PHYCFGR_OPMD) ? PHY_CONFBY_SW : PHY_CONFBY_HW;
	switch(value & PHYCFGR_OPMDC_ALLA)
	{
		case PHYCFGR_OPMDC_ALLA:
		case PHYCFGR_OPMDC_100FA: 
			ret.mode = PHY_MODE_AUTONEGO;
			break;
		default:
			ret.mode = PHY_MODE_MANUAL;
			break;
	}
	switch(value & PHYCFGR_OPMDC_ALLA)
	{
		case PHYCFGR_OPMDC_100FA:
		case PHYCFGR_OPMDC_100F:
		case PHYCFGR_OPMDC_100H:
			ret.speed = PHY_SPEED_100;
			break;
		default:
			ret.speed = PHY_SPEED_10;
		break;
		}
		switch(value & PHYCFGR_OPMDC_ALLA)
		{
			case PHYCFGR_OPMDC_100FA:
			case PHYCFGR_OPMDC_100F:
			case PHYCFGR_OPMDC_10F:
				ret.duplex = PHY_DUPLEX_FULL;
				break;
			default:
				ret.duplex = PHY_DUPLEX_HALF;
				break;
	}
	return ret;

}
void smp_w5500_spi_config(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	
	W5500_RST_PIN.pin = BSP_W5500_RST_PIN;
	W5500_RST_PIN.port = BSP_W5500_RST_PORT;
	W5500_RST_PIN.mode = SMP_GPIO_MODE_OUTPUT_PP;
	smp_gpio_init(&W5500_RST_PIN); 
	
	/*EXTI Pin init*/
	W5500_INT_PIN.pin = BSP_W5500_INT_PIN;
	W5500_INT_PIN.port = BSP_W5500_INT_PORT;
	W5500_INT_PIN.mode = SMP_GPIO_MODE_INPUT;
	smp_gpio_init(&W5500_INT_PIN);
	
	W5500_SPI_CS_PIN.pin = BSP_W5500_SPI_SCS_PIN;
	W5500_SPI_CS_PIN.port = BSP_W5500_SPI_SCS_PORT;	
	
	W5500_CS.spi_num = SPI_module2;
	W5500_CS.cs_handler = W5500_SPI_CS_PIN;	
	smp_spi_master_cs_init(&W5500_CS);
	
	SPI_W5500.num = SPI_module2;
	SPI_W5500.mode = SPI_mode0;
	smp_spi_master_init(&SPI_W5500, spi_master_w5500_event_handler, false);

}

int8_t W5500_Socket_Register(W5500_Socket_parm *parm, smp_w5500_event_t w5500_event_Handler){
	static uint8_t i;
	for(i=0; i<W5500_MAX_SOCKET_NUM;i++){		
		if(smpW5500Socket[i].SocketStatus == Socket_Disable){
			/* Socket i is availible, write parameter to socket parameter */
			smpW5500Socket[i].SocketStatus = Socket_Close;
			smpW5500Socket[i].cbFunPtr = w5500_event_Handler;
			smpW5500Socket[i].Parm.Num = i;						//Socket number hardware(w5500) limit 0~7 
			smpW5500Socket[i].Parm.Memory.rx_buf_Ptr = parm->Memory.rx_buf_Ptr;
			smpW5500Socket[i].Parm.Memory.tx_buf_Ptr = parm->Memory.tx_buf_Ptr;
			smpW5500Socket[i].Parm.Memory.rx_buf_size = parm->Memory.rx_buf_size;
			smpW5500Socket[i].Parm.Memory.tx_buf_size = parm->Memory.tx_buf_size;
			smpW5500Socket[i].Parm.Protocol = parm->Protocol;
			smpW5500Socket[i].Parm.PortNum = parm->PortNum;
			smpW5500Socket[i].Parm.DeviceID = parm->DeviceID;
			break;
		}
	}
	/*Socket Full */
	if(i==W5500_MAX_SOCKET_NUM){
		return SMP_ERROR_FULL;
	}else{
		parm->Num = i;
	}
	SocketEnableCount++;
	return SMP_SUCCESS;
}

static uint8_t Increase_Socket_Num(void){
	gSocketNum++;
	if(gSocketNum == SocketEnableCount){
		gSocketNum = 0;
		return Sock_Num_Reset;
	}
	return Sock_Increase_Ing;
}

static uint8_t W5500_Send_Socket_Command(uint8_t SockNum, uint8_t Command){
	switch(Send_Command_Step){
		case Send_Command:
			smp_w5500_spiDMA_write_byte(Sn_CR(SockNum), Command);
			Send_Command_Step = Read_Command_Reg;
			break;
		case Read_Command_Reg:
			smp_w5500_spiDMA_read_byte(Sn_CR(SockNum));
			Send_Command_Step = Verify_Command_Recv;
			break;
		case Verify_Command_Recv:
			if(Readbyte_value == 0){
				Send_Command_Step = Send_Command;
				return Send_Done;
			}else{
				Send_Command_Step = Read_Command_Reg;
			}
			break;
	}
	return Send_Ing;
}

static void (*W5500Function)(void);

static void W5500_Error(void){
	if(smpW5500Socket[gSocketNum].cbFunPtr){
		smpW5500Socket[gSocketNum].cbFunPtr(W5500_COMMUNICATE_ERR,0);
	}
}

static uint8_t Config_and_Verify_Byte_Regsiter(uint32_t Addr, uint8_t Value){
	
	switch(Ini_Reg_Step){
		case Config_Parm:
			smp_w5500_spiDMA_write_byte(Addr,Value);
			Ini_Reg_Step = Read_Reg;
			break;
		case Read_Reg:
			smp_w5500_spiDMA_read_byte(Addr);
			Ini_Reg_Step = Verify_Parm;
			break;
		case Verify_Parm:
			Ini_Reg_Step = Config_Parm;
			if(Readbyte_value != Value)
				return Config_Fail;
			else
				return Config_Done;
			
			break;
		default:
			W5500Function = W5500_Error;
			break;
	}
	return Config_Ing;

}

static uint8_t Config_and_Verify_Word_Regsiter(uint32_t Addr, uint16_t Value){
	
	switch(Ini_Reg_Step){
		case Config_Parm:
			smp_w5500_spiDMA_write_word(Addr,Value);
			Ini_Reg_Step = Read_Reg;
			break;
		case Read_Reg:
			smp_w5500_spiDMA_read_word(Addr);
			Ini_Reg_Step = Verify_Parm;
			break;
		case Verify_Parm:
			Ini_Reg_Step = Config_Parm;
			if((ReadWord_value[0] != (uint8_t)(Value>>8))||(ReadWord_value[1]!=(uint8_t)Value))
				return Config_Fail;
			else
				return Config_Done;
			break;
		default:
			W5500Function = W5500_Error;
			break;
	}
	return Config_Ing;

}

typedef void (*W5500_Socket_Server_Table)(void);
/* ----------Sock Reopen Function Pointer Index---------- */
enum{
	Read_Socket_Status_Index,
	Socket_Status_Handle_Index,
	Read_Local_IP_Index,
	Check_Local_IP_AlreadySet_Index,
	Send_Command_Socket_Close_Index,
	Reset_Socket_INT_Register_Index,
	Verify_Socket_Status_Index,
	Check_Socket_Is_Close_Index,
	Config_Socket_Protocol_Index,
	Config_Socket_PortNum_Index,
	Send_Socket_Open_Command_Index,
	Read_Socket_Protocol_Index,
	Verify_Socket_Protocol_Index,
	Send_Socket_Listen_Command_Index,
	
};

static uint8_t Sock_Reopen_Index = 0;
static void Sock_Reopen_Next_Function(void){
	Sock_Reopen_Index++;
}


static void Read_Socket_Status(void){
	smp_w5500_spiDMA_read_byte(Sn_SR(gSocketNum));
	Sock_Reopen_Next_Function();
}

static void Socket_Status_Handle(void){
	switch(Readbyte_value){
		case SOCK_CLOSED:
			Sock_Reopen_Index = Read_Local_IP_Index;
			break;
		case SOCK_INIT:
			Sock_Reopen_Index = Read_Socket_Protocol_Index;
			break;
		case SOCK_LISTEN:
			smpW5500Socket[gSocketNum].SocketStatus = Socket_Open;
			Sock_Reopen_Index = 0;
			Flag_Check_INT = false;
			break;
	}
}
/* ---------- Socket Close >>> Socket Init ---------- */
static void Read_Local_IP(void){
	smp_w5500_spiDMA_ReadMulti(SIPR,TempNetInfo.ip,IP_ADDR_BYTE_LEN);	
	Sock_Reopen_Next_Function();
}

static void Check_Local_IP_AlreadySet(void){
	static uint8_t i;
	for(i=0;i<IP_ADDR_BYTE_LEN;i++){
		if(TempNetInfo.ip[i]!=0){
			break;
		}
	}
	if(i == IP_ADDR_BYTE_LEN){
		W5500Function = W5500_Error;
	}else{
		Sock_Reopen_Next_Function();
	}
}

static void Send_Command_Socket_Close(void){
	static uint8_t Result; 
	Result = W5500_Send_Socket_Command(gSocketNum,Sn_CR_CLOSE);
	if(Result == Send_Done){
		Sock_Reopen_Next_Function();
	}
}

static void Reset_Socket_INT_Register(void){
	smp_w5500_spiDMA_write_byte(Sn_IR(gSocketNum), 0xFF&SOCKET_NUM_BASE);
	Sock_Reopen_Next_Function();
	Flag_Check_INT = true;
}

static void Check_Socket_Is_Close(void){
	if(Readbyte_value == SOCK_CLOSED)
		Sock_Reopen_Next_Function();
	else
		W5500Function = W5500_Error;
}

static void Config_Socket_Protocol(void){
	static uint8_t Result; 
	Result = Config_and_Verify_Byte_Regsiter(Sn_MR(gSocketNum), smpW5500Socket[gSocketNum].Parm.Protocol);
	if( Result == Config_Done){
		Sock_Reopen_Next_Function();
	}else if (Result == Config_Fail){
		W5500Function = W5500_Error;
	}		
}

static void Config_Socket_PortNum(void){
	static uint8_t Result; 
	Result = Config_and_Verify_Word_Regsiter(Sn_PORT(gSocketNum),smpW5500Socket[gSocketNum].Parm.PortNum);
	if( Result == Config_Done){
		Sock_Reopen_Next_Function();
	}else if (Result == Config_Fail){
		W5500Function = W5500_Error;
	}
}



static void Send_Socket_Open_Command(void){
	static uint8_t Result; 
	Result = W5500_Send_Socket_Command(gSocketNum,Sn_CR_OPEN);
	if(Result == Send_Done){
		Sock_Reopen_Index = Read_Socket_Status_Index;
	}
}

/* ---------- Socket Init >>> Socket Listen ---------- */
static void Read_Socket_Protocol(void){
	smp_w5500_spiDMA_read_byte(Sn_MR(gSocketNum));
	Sock_Reopen_Next_Function();
}

static void Verify_Socket_Protocol(void){
	if((Readbyte_value&0x0F) != smpW5500Socket[gSocketNum].Parm.Protocol)
		W5500Function = W5500_Error;
	else
		Sock_Reopen_Next_Function();
}

static void Send_Socket_Listen_Command(void){
	static uint8_t Result; 
	Result = W5500_Send_Socket_Command(gSocketNum,Sn_CR_LISTEN);
	if(Result == Send_Done){
		Sock_Reopen_Index = Read_Socket_Status_Index;
	}
}




const W5500_Socket_Server_Table Sock_Reopen_Function_Table[] = {
	Read_Socket_Status,
	Socket_Status_Handle,
/* ---------- Socket Close >>> Socket Init ---------- */	
	Read_Local_IP,
	Check_Local_IP_AlreadySet,
	Send_Command_Socket_Close,
	Reset_Socket_INT_Register,
	Read_Socket_Status,
	Check_Socket_Is_Close,
	Config_Socket_Protocol,
	Config_Socket_PortNum,
	Send_Socket_Open_Command,
/* ---------- Socket Init >>> Socket Listen ---------- */	
	Read_Socket_Protocol,
	Verify_Socket_Protocol,
	Send_Socket_Listen_Command,
	
	Sock_Reopen_Next_Function
};

/* ----------Sock Evt Function Pointer Index---------- */
enum{
	Read_INT_Summary_Index,
	Verify_Which_Socket_INT_Index,
	Read_INT_Evt_Index,
	Socket_INTEvt_Handle_Index,
/* ---------- Connect INT ---------- */		
	Read_Remote_IP_Index,
	Read_Remote_Mac_Index,
	Read_Remote_Port_Index,
	Save_Remote_Port_Index,
	Socket_Con_CBFunction_Index,
	Reset_Con_INT_Bit_Index,
/* ---------- Recv INT ----------*/	
	Read_Socket_RX_Buf_Data_Size_Highbyte_Index,
	Read_Socket_RX_Buf_Data_Size_Lowbyte_Index,
	RX_Data_Oversize_Handle_Index,
	Read_Socket_RX_Buf_Addr_HighByte_Index,
	Read_Socket_RX_Buf_Addr_LowByte_Index,
	Read_Socket_Buf_Data_Index,
	ParserData_CB_and_Update_RX_Buf_Addr_Index,
	Write_RX_Buf_Addr_HighByte_Index,
	Write_RX_Buf_Addr_LowByte_Index,
	Send_Socket_Recv_Command_Index,
	Read_TX_Buf_Size_Index,
	TX_Data_Oversize_Handle_Index,
	Read_Socket_TX_Buf_Addr_HighByte_Index,
	Read_Socket_TX_Buf_Addr_LowByte_Index,
	Write_Socket_TXBuf_Data_Index,
	Write_TX_Buf_Addr_HighByte_Index,
	Write_TX_Buf_Addr_LowByte_Index,
	Send_Socket_SEND_Command_Index,
	Read_Sock_INTEvt_Index,
	Verify_Data_Send_Done_Index,
	Clr_INT_RECV_and_SENDOK_Bits_Index,
/* ---------- Disonnect INT ---------- */
	Send_Socket_Close_EvtHandle_Index,
	Reset_SocketStatus_Index,
	Socket_Reopen_Index,
	Clr_INT_Discon_Bit_Index
	
};

static uint8_t Sock_EvtHandle_Index = 0;
static void Sock_EvtHandle_Next_Function(void){
	Sock_EvtHandle_Index++;
}

static void Read_INT_Summary(void){
	smp_w5500_spiDMA_read_byte(SIR);
	Sock_EvtHandle_Next_Function();
}

static void Verify_Which_Socket_INT(void){
	static uint8_t i;
	if(Readbyte_value == 0){
		Flag_Check_INT = false;
		Sock_EvtHandle_Index = Read_INT_Summary_Index;
	}else{
		for(i=0;i<W5500_MAX_SOCKET_NUM;i++){
			if(((Readbyte_value>>i)&0x01) == 1){
				gSocketNum = i;
				break;
			}
		}
		Sock_EvtHandle_Next_Function();
	}
}

static void Read_INT_Evt(void){
	smp_w5500_spiDMA_read_byte(Sn_IR(gSocketNum));
	Sock_EvtHandle_Next_Function();
}

static void Socket_INTEvt_Handle(void){
	switch(Readbyte_value){
		case Sn_IR_CON:
			Sock_EvtHandle_Index = Read_Remote_IP_Index;
			break;
		case Sn_IR_RECV:
			Sock_EvtHandle_Index = 	Read_Socket_RX_Buf_Data_Size_Highbyte_Index;
			break;
		case Sn_IR_TIMEOUT:
		case Sn_IR_DISCON:
			Sock_EvtHandle_Index = Send_Socket_Close_EvtHandle_Index;
			break;
		default:
			break;
	}

}
/* ---------- Connect INT ---------- */	
static void Read_Remote_IP(void){
	smp_w5500_spiDMA_ReadMulti(Sn_DIPR(gSocketNum), smpW5500Socket[gSocketNum].Remote.ip, IP_ADDR_BYTE_LEN);
	Sock_EvtHandle_Next_Function();
}

static void Read_Remote_Mac(void){
	smp_w5500_spiDMA_ReadMulti(Sn_DHAR(gSocketNum), smpW5500Socket[gSocketNum].Remote.mac, MAC_ADDR_BYTE_LEN);
	Sock_EvtHandle_Next_Function();
}

static void Read_Remote_Port(void){
	smp_w5500_spiDMA_read_word(Sn_DPORT(gSocketNum));
	Sock_EvtHandle_Next_Function();
}

static void Save_Remote_Port(void){
	smpW5500Socket[gSocketNum].Parm.DestPort = (((uint16_t)ReadWord_value[0])<<8) + (uint16_t)ReadWord_value[1];
	Sock_EvtHandle_Next_Function();
}

static void Socket_Con_CBFunction(void){
	if(smpW5500Socket[gSocketNum].cbFunPtr){
		smpW5500Socket[gSocketNum].cbFunPtr(W5500_Socket_Connect,0);
	}
	Sock_EvtHandle_Next_Function();
}

static void Reset_Con_INT_Bit(void){
	smp_w5500_spiDMA_write_byte(Sn_IR(gSocketNum), (Sn_IR_CON&SOCKET_NUM_BASE)); 
	Sock_EvtHandle_Index = Read_INT_Summary_Index;
	Flag_Check_INT = true;
}
/* ---------- Receive INT ---------- */
static void Read_Socket_RX_Buf_Data_Size_Highbyte(void){
	smp_w5500_spiDMA_read_byte(Sn_RX_RSR(gSocketNum));
	Sock_EvtHandle_Next_Function();
}

static void Read_Socket_RX_Buf_Data_Size_Lowbyte(void){
	SocketHandleByteLen[gSocketNum] = ((uint16_t)Readbyte_value) << 8;
	smp_w5500_spiDMA_read_byte(WIZCHIP_OFFSET_INC(Sn_RX_RSR(gSocketNum),1));
	Sock_EvtHandle_Next_Function();
}

static void RX_Data_Oversize_Handle(void){
	SocketHandleByteLen[gSocketNum] += (uint16_t)Readbyte_value;
	if(SocketHandleByteLen[gSocketNum] > smpW5500Socket[gSocketNum].Parm.Memory.rx_buf_size){
		SocketHandleByteLen[gSocketNum] = smpW5500Socket[gSocketNum].Parm.Memory.rx_buf_size;
	}
	Sock_EvtHandle_Next_Function();
}

static void Read_Socket_RX_Buf_Addr_HighByte(void){
	smp_w5500_spiDMA_read_byte(Sn_RX_RD(gSocketNum));
	Sock_EvtHandle_Next_Function();
}

static void Read_Socket_RX_Buf_Addr_LowByte(void){
	RX_Buf_Addr = (((uint16_t)Readbyte_value)<<8);
	smp_w5500_spiDMA_read_byte(WIZCHIP_OFFSET_INC(Sn_RX_RD(gSocketNum),1));
	Sock_EvtHandle_Next_Function();
}

static void Read_Socket_Buf_Data(void){
	RX_Buf_Addr += (uint16_t)Readbyte_value;
	SPI_Command_Frame = ((uint32_t)RX_Buf_Addr<<8)+ (WIZCHIP_RXBUF_BLOCK(gSocketNum)<< 3); 
	smp_w5500_spiDMA_ReadMulti(SPI_Command_Frame,smpW5500Socket[gSocketNum].Parm.Memory.rx_buf_Ptr,SocketHandleByteLen[gSocketNum]);
	Sock_EvtHandle_Next_Function();
}

static void ParserData_CB_and_Update_RX_Buf_Addr(void){
	RX_Buf_Addr += SocketHandleByteLen[gSocketNum];
	if(smpW5500Socket[gSocketNum].cbFunPtr){
		SocketHandleByteLen[gSocketNum] = smpW5500Socket[gSocketNum].cbFunPtr(W5500_DATA_RECV,SocketHandleByteLen[gSocketNum]);
	}
	smp_w5500_spiDMA_write_byte(Sn_IR(gSocketNum), (Sn_IR_RECV&SOCKET_NUM_BASE)); 
	Flag_Check_INT = true;
	Sock_EvtHandle_Next_Function();
}

static void Write_RX_Buf_Addr_HighByte(void){
	smp_w5500_spiDMA_write_byte(Sn_RX_RD(gSocketNum),(uint8_t)(RX_Buf_Addr>>8));
	Sock_EvtHandle_Next_Function();
}

static void Write_RX_Buf_Addr_LowByte(void){
	smp_w5500_spiDMA_write_byte(WIZCHIP_OFFSET_INC(Sn_RX_RD(gSocketNum),1),(uint8_t)RX_Buf_Addr);
	Sock_EvtHandle_Next_Function();
}

static void Send_Socket_Recv_Command(void){
	static uint8_t Result; 
	Result = W5500_Send_Socket_Command(gSocketNum,Sn_CR_RECV);
	if(Result == Send_Done){
		Sock_EvtHandle_Next_Function();
	}
}

static void Read_TX_Buf_Size(void){
	smp_w5500_spiDMA_read_byte(Sn_TXBUF_SIZE(gSocketNum));
	Sock_EvtHandle_Next_Function();
}

static void TX_Data_Oversize_Handle(void){
	static uint16_t Size;
	Size = ((uint16_t)Readbyte_value)<<10;	
	if(SocketHandleByteLen[gSocketNum]>Size){
		SocketHandleByteLen[gSocketNum] = Size;
	}
	Sock_EvtHandle_Next_Function();
}

static void Read_Socket_TX_Buf_Addr_HighByte(void){
	smp_w5500_spiDMA_read_byte(Sn_TX_WR(gSocketNum));
	Sock_EvtHandle_Next_Function();
}

static void Read_Socket_TX_Buf_Addr_LowByte(void){
	TX_Buf_Addr = (((uint16_t)Readbyte_value)<<8);
	smp_w5500_spiDMA_read_byte(WIZCHIP_OFFSET_INC(Sn_TX_WR(gSocketNum),1));
	Sock_EvtHandle_Next_Function();
}

static void Write_Socket_TXBuf_Data(void){
	
	TX_Buf_Addr +=(uint16_t)Readbyte_value;
	SPI_Command_Frame = (((uint32_t)TX_Buf_Addr) << 8) + (WIZCHIP_TXBUF_BLOCK(gSocketNum)<< 3);
	smp_w5500_spiDMA_WriteMulti(SPI_Command_Frame,smpW5500Socket[gSocketNum].Parm.Memory.tx_buf_Ptr, SocketHandleByteLen[gSocketNum]);
	TX_Buf_Addr += SocketHandleByteLen[gSocketNum]; 
	Sock_EvtHandle_Next_Function();
}

static void Write_TX_Buf_Addr_HighByte(void){
 
	smp_w5500_spiDMA_write_byte(Sn_TX_WR(gSocketNum),(uint8_t)(TX_Buf_Addr>>8));
	Sock_EvtHandle_Next_Function();
	
}

static void Write_TX_Buf_Addr_LowByte(void){
 
	smp_w5500_spiDMA_write_byte(WIZCHIP_OFFSET_INC(Sn_TX_WR(gSocketNum),1),(uint8_t)TX_Buf_Addr);
	Sock_EvtHandle_Next_Function();
}

static void Send_Socket_SEND_Command(void){
	static uint8_t Result; 
	Result = W5500_Send_Socket_Command(gSocketNum,Sn_CR_SEND);
	if(Result == Send_Done){
		Sock_EvtHandle_Next_Function();
	}
}

static void Read_Sock_INTEvt(void){
	smp_w5500_spiDMA_read_byte(Sn_IR(gSocketNum));
	Sock_EvtHandle_Next_Function();
}

static void Verify_Data_Send_Done(void){
	if((Readbyte_value&=SOCKET_NUM_BASE)&Sn_IR_SENDOK )
		Sock_EvtHandle_Next_Function();
	else 
		Sock_EvtHandle_Index = Read_Sock_INTEvt_Index;
}

static void Clr_INT_RECV_and_SENDOK_Bits(void){
	smp_w5500_spiDMA_write_byte(Sn_IR(gSocketNum), (Sn_IR_SENDOK&SOCKET_NUM_BASE)); 
	//smp_w5500_spiDMA_write_byte(Sn_IR(gSocketNum), ((Sn_IR_RECV|Sn_IR_SENDOK)&SOCKET_NUM_BASE)); 
	Sock_EvtHandle_Index = Read_INT_Summary_Index;
	Flag_Check_INT = true;
}
/* ---------- Disonnect INT ---------- */
static void Send_Socket_Close_EvtHandle(void){
	static uint8_t Result; 
	Result = W5500_Send_Socket_Command(gSocketNum,Sn_CR_CLOSE);
	if(Result == Send_Done){
		Sock_EvtHandle_Next_Function();
	}
}

static void Reset_SocketStatus(void){
	smpW5500Socket[gSocketNum].SocketStatus = Socket_Close;
	Sock_EvtHandle_Next_Function();
}

static void Socket_Reopen(void){
	Sock_Reopen_Function_Table[Sock_Reopen_Index]();
	if(smpW5500Socket[gSocketNum].SocketStatus == Socket_Open){
		Flag_Check_INT = true;
		Sock_EvtHandle_Index = Read_INT_Summary_Index;
	}
}

static void Clr_INT_Discon_Bit(void){
	smp_w5500_spiDMA_write_byte(Sn_IR(gSocketNum), (Sn_CR_DISCON&SOCKET_NUM_BASE)); 
	Sock_EvtHandle_Index = Read_INT_Summary_Index;
}

const W5500_Socket_Server_Table Sock_Evt_Handle_Function_Table[]={
	Read_INT_Summary,
	Verify_Which_Socket_INT,
	Read_INT_Evt,
	Socket_INTEvt_Handle,
/* ---------- Connect INT ---------- */	
	Read_Remote_IP,
	Read_Remote_Mac,
	Read_Remote_Port,
	Save_Remote_Port,
	Socket_Con_CBFunction,
	Reset_Con_INT_Bit,
/* ---------- Recv INT ----------*/
	Read_Socket_RX_Buf_Data_Size_Highbyte,
	Read_Socket_RX_Buf_Data_Size_Lowbyte,
	RX_Data_Oversize_Handle,
	Read_Socket_RX_Buf_Addr_HighByte,
	Read_Socket_RX_Buf_Addr_LowByte,
	Read_Socket_Buf_Data,
	ParserData_CB_and_Update_RX_Buf_Addr,
	Write_RX_Buf_Addr_HighByte,
	Write_RX_Buf_Addr_LowByte,
	Send_Socket_Recv_Command,
	Read_TX_Buf_Size,
	TX_Data_Oversize_Handle,
	Read_Socket_TX_Buf_Addr_HighByte,
	Read_Socket_TX_Buf_Addr_LowByte,
	Write_Socket_TXBuf_Data,
	Write_TX_Buf_Addr_HighByte,
	Write_TX_Buf_Addr_LowByte,
	Send_Socket_SEND_Command,
	Read_Sock_INTEvt,
	Verify_Data_Send_Done,
	Clr_INT_RECV_and_SENDOK_Bits,
/* ---------- Disonnect INT ---------- */	
	Send_Socket_Close_EvtHandle,
	Reset_SocketStatus,
	Socket_Reopen,
	
	Sock_EvtHandle_Next_Function
};

static void W5500_Server(void){
	static uint8_t Result;
	if(gSPI_Status == SPI_Idle){
		smp_gpio_get_state(&W5500_INT_PIN,&W5500_INT_PIN_State);
		if((W5500_INT_PIN_State == GPIO_ACTIVE_LOW)||Flag_Check_INT){
			Sock_Evt_Handle_Function_Table[Sock_EvtHandle_Index]();
		}
	}
}
/*---------- W55500 Initial Function ----------*/
typedef void (*W5500_INIT_Table)(void);
static uint8_t W5500_INIT_Index = 0;
static void W5500_Point_Next_Function(void){
	W5500_INIT_Index++;
}

static void RST_PIN_Toggle(void){
	if(ms_Count == 0){	
		smp_gpio_set_state(&W5500_RST_PIN, GPIO_ACTIVE_LOW);
		ms_Count++;
	}else if(ms_Count == HW_RESET_DELAY_TIME_MS){
		ms_Count = 0;
		smp_gpio_set_state(&W5500_RST_PIN, GPIO_ACTIVE_HIGH);
		W5500_Point_Next_Function();
	}	
}
	
static void Write_Rst_Register(void){
	smp_w5500_spiDMA_write_byte(MR, MR_RST);
	W5500_Point_Next_Function();
}	
	
static void Conf_Default_PHY_Register(void){
	PHYconf.by = PHY_CONFBY_SW;
	PHYconf.mode = PHY_MODE_MANUAL;
	PHYconf.speed = PHY_SPEED_100;
	PHYconf.duplex = PHY_DUPLEX_FULL;
	smp_w5500_spiDMA_write_byte(PHYCFGR, w5500_phyconf_transfer_to_uint8(PHYconf));
	W5500_Point_Next_Function();
}

static void Read_PHY_Conf_Register(void){
	smp_w5500_spiDMA_read_byte(PHYCFGR);
	W5500_Point_Next_Function();
}

static void Conf_PHY_Register_With_Rst_Bit(void){
	Readbyte_value |= ~PHYCFGR_RST;
	smp_w5500_spiDMA_write_byte(PHYCFGR, Readbyte_value);
	W5500_Point_Next_Function();
}

static void Verify_PHY_Parameter(void){
	PHYconf_Verify = w5500_uint8_transfer_to_phyconf(Readbyte_value);
	if((PHYconf_Verify.by != PHYconf.by)||(PHYconf_Verify.mode != PHYconf.mode)||
		(PHYconf_Verify.speed != PHYconf.speed)||(PHYconf_Verify.duplex != PHYconf.duplex)){
		W5500Function = W5500_Error;
	}
	W5500_Point_Next_Function();
}

static void Init_Default_NetInfo(void){
	switch(NetInfoConfIndex){
		case W_R_MAC_Addr:
			smp_w5500_spiDMA_WriteMulti(ConfNetInfoAddrTable[NetInfoConfIndex].AddrSel,DefMac,ConfNetInfoAddrTable[NetInfoConfIndex].Len);
			break;
		case W_R_Gate:
			smp_w5500_spiDMA_WriteMulti(ConfNetInfoAddrTable[NetInfoConfIndex].AddrSel,DefGw,ConfNetInfoAddrTable[NetInfoConfIndex].Len);
			break;
		case W_R_SUB_MASK:
			smp_w5500_spiDMA_WriteMulti(ConfNetInfoAddrTable[NetInfoConfIndex].AddrSel,DefSn,ConfNetInfoAddrTable[NetInfoConfIndex].Len);
			break;
		case W_R_IP_Addr:
			smp_w5500_spiDMA_WriteMulti(ConfNetInfoAddrTable[NetInfoConfIndex].AddrSel,DefIp,ConfNetInfoAddrTable[NetInfoConfIndex].Len);
			break;
		default:
			W5500Function = W5500_Error;
			break;
	}
	NetInfoConfIndex++;
	if(NetInfoConfIndex > W_R_IP_Addr){
		NetInfoConfIndex = 0;
		W5500_Point_Next_Function();
	}
}

static void Read_NetInfo(void){
	switch(NetInfoConfIndex){
		case W_R_MAC_Addr:
			smp_w5500_spiDMA_ReadMulti(ConfNetInfoAddrTable[NetInfoConfIndex].AddrSel,TempNetInfo.mac,ConfNetInfoAddrTable[NetInfoConfIndex].Len);
			break;
		case W_R_Gate:
			smp_w5500_spiDMA_ReadMulti(ConfNetInfoAddrTable[NetInfoConfIndex].AddrSel,TempNetInfo.gw,ConfNetInfoAddrTable[NetInfoConfIndex].Len);
			break;
		case W_R_SUB_MASK:
			smp_w5500_spiDMA_ReadMulti(ConfNetInfoAddrTable[NetInfoConfIndex].AddrSel,TempNetInfo.sn,ConfNetInfoAddrTable[NetInfoConfIndex].Len);
			break;
		case W_R_IP_Addr:
			smp_w5500_spiDMA_ReadMulti(ConfNetInfoAddrTable[NetInfoConfIndex].AddrSel,TempNetInfo.ip,ConfNetInfoAddrTable[NetInfoConfIndex].Len);
			break;
		default:
			W5500Function = W5500_Error;
			break;
	}
	NetInfoConfIndex++;
	if(NetInfoConfIndex > W_R_IP_Addr){
		NetInfoConfIndex = 0;
		W5500_Point_Next_Function();
	}
}

static void Verify_NetInfo_Parm(void){
	if( (memcmp(TempNetInfo.mac, DefMac, MAC_ADDR_BYTE_LEN)!=0) ||
		(memcmp(TempNetInfo.ip, DefIp, IP_ADDR_BYTE_LEN)!=0) ||
		(memcmp(TempNetInfo.sn, DefSn, SUB_MASK_ADDR_BYTE_LEN)!=0) ||
		(memcmp(TempNetInfo.gw, DefGw, GATE_WAY_ADDR_BYTE_LEN)!=0)){
		W5500Function = W5500_Error;
	}
	W5500_Point_Next_Function();
}

static void Cofig_Int_Mask_Reg(void){
	static uint8_t Result; 
	Result = Config_and_Verify_Byte_Regsiter(_IMR_,0x00);
	if( Result == Config_Done){
		W5500_Point_Next_Function();
	}else if (Result == Config_Fail){
		W5500Function = W5500_Error;
	}
}

static void Config_Socket_Num_Int_Mask(void){
	static uint8_t Result; 
	Result = Config_and_Verify_Byte_Regsiter(SIMR,0xFF);
	if( Result == Config_Done){
		W5500_Point_Next_Function();
	}else if (Result == Config_Fail){
		W5500Function = W5500_Error;
	}
}

static void Config_Socket_INT_Wait_Time(void){
	static uint8_t Result; 
	Result = Config_and_Verify_Word_Regsiter(INTLEVEL,1000);
	if( Result == Config_Done){	
		W5500_Point_Next_Function();		
	}else if (Result == Config_Fail){
		W5500Function = W5500_Error;
	}
}

static void Config_Socket_Int_Mask(void){
	static uint8_t Result; 
	Result = Config_and_Verify_Byte_Regsiter(Sn_IMR(gSocketNum), (Sn_IR_TIMEOUT|Sn_IR_RECV|Sn_IR_DISCON|Sn_IR_CON));
	if( Result == Config_Done){
		if(Increase_Socket_Num() == Sock_Num_Reset){
			W5500_Point_Next_Function();		
		}
	}else if (Result == Config_Fail){
		W5500Function = W5500_Error;
	}
}


static void Ini_End(void){
	if(smpW5500Socket[gSocketNum].SocketStatus == Socket_Close){
		Sock_Reopen_Function_Table[Sock_Reopen_Index]();
		if(smpW5500Socket[gSocketNum].SocketStatus == Socket_Open){
			if(Increase_Socket_Num() == Sock_Num_Reset){
				W5500_INIT_Index = 0;
				W5500Function = W5500_Server;
			}
		}
	}
}

const W5500_INIT_Table W5500init_Function_Table[]={
	RST_PIN_Toggle,
	Write_Rst_Register,
	Conf_Default_PHY_Register,
	Read_PHY_Conf_Register,
	Conf_PHY_Register_With_Rst_Bit,
	Read_PHY_Conf_Register,
	Verify_PHY_Parameter,
	Init_Default_NetInfo,
	Read_NetInfo,
	Verify_NetInfo_Parm,
	Cofig_Int_Mask_Reg,
	Config_Socket_Num_Int_Mask,
	Config_Socket_INT_Wait_Time,
	Config_Socket_Int_Mask,
	Ini_End,
	
	W5500_Point_Next_Function
};
static void W5500_Init(void){
	if(gSPI_Status == SPI_Idle){
		W5500init_Function_Table[W5500_INIT_Index]();
	}
}

static void W5500SwTimerHandler_New(__far void *dest, uint16_t evt, void *vDataPtr){
	if(evt == LIB_SW_TIMER_EVT_SW_1MS){
		W5500Function();
	}
}

void Hal_W5500_Open(void){
	smp_w5500_spi_config();
	LibSwTimerOpen(W5500SwTimerHandler_New, 0);
	W5500Function = W5500_Init;
}
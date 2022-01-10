/**
******************************************************************************
* @file    smp_W5500_DMA.c
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

#include "smp_w5500_DMA.h"
#include <stdbool.h>
#include <string.h>

#define DMA_Test 


#define _W5500_SPI_VDM_OP_          0x00
#define _W5500_SPI_FDM_OP_LEN1_     0x01
#define _W5500_SPI_FDM_OP_LEN2_     0x02
#define _W5500_SPI_FDM_OP_LEN4_     0x03
#define SOCK_ANY_PORT_NUM  0xC000

typedef struct{
	uint32_t AddrSel;
	uint16_t Len;
	uint8_t *pData;
}R_W_NetInfo;

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


static uint16_t sock_any_port = SOCK_ANY_PORT_NUM;
static uint16_t sock_io_mode = 0;
static uint16_t sock_is_sending = 0;
static uint16_t sock_remained_size[_WIZCHIP_SOCK_NUM_];
uint8_t  sock_pack_info[_WIZCHIP_SOCK_NUM_] = {0,};

volatile uint16_t SocketHandleByteLen[W5500_MAX_SOCKET_NUM] = {0,0,0,0,0,0,0,0};
uint16_t gu16RecvBytesCnt;

static uint8_t socketStatus[W5500_MAX_SOCKET_NUM];
static uint8_t SocketEnableCount = 0;
uint8_t Socket_Event_Handling_Status[W5500_MAX_SOCKET_NUM];


static uint8_t gu8W5500_SPI_Status = W5500_SPI_Idle;
static uint8_t gu8_W5500_Init_Steps = Init_End;
static uint8_t gu8Socket_Sever_Step[W5500_MAX_SOCKET_NUM] = {0};
static uint8_t gu8Socket_Reopen_Step[W5500_MAX_SOCKET_NUM] = {0};
static uint8_t gu8SockNum = 0;


static uint8_t DefMac[MAC_ADDR_BYTE_LEN] = DEFAULT_MAC_ADDR;
static uint8_t DefIp[IP_ADDR_BYTE_LEN] = DEFAULT_IP_ADDR;
static uint8_t DefSn[SUB_MASK_ADDR_BYTE_LEN] = DEFAULT_SUB_MASK;
static uint8_t DefGw[GATE_WAY_ADDR_BYTE_LEN] = DEFAULT_GW_ADDR;
static uint8_t DefDns[IP_ADDR_BYTE_LEN] = DEFAULT_DNS_ADDR;


uint8_t W5500_tx_Buf[BSP_SPI2_TX_BUFFER_SIZE];
uint8_t Readbyte_value;
uint8_t gu8Temp;
uint16_t gu16Temp;
uint32_t gu32Temp;
volatile bool W5500_Read_SnIR_End;
volatile uint16_t W5500_INT_Cnt = 0;

smp_gpio_t		W5500_RST_PIN;
smp_gpio_t		W5500_INT_PIN;
smp_gpio_t		W5500_SPI_CS_PIN;
smp_spi_cs_t 	W5500_CS;
smp_spi_t 		SPI_W5500;


wiz_NetInfo TmpNetInfo,RemoteNetInfo;

W5500_RegisterList smpW5500Socket[W5500_MAX_SOCKET_NUM];



/**
  * @brief This function handles W5500 INT EXTI Pin interrupt.
  */
void EXTI9_5_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(SMP_GPIO_PIN(BSP_W5500_INT_PIN));
}

void smp_w5500_spiDMA_write_byte(uint32_t AddrSel, uint8_t Value){
	gu8W5500_SPI_Status = W5500_SPI_Busy;
	AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);
	W5500_tx_Buf[0] = (AddrSel & 0x00FF0000) >> 16;
	W5500_tx_Buf[1] = (AddrSel & 0x0000FF00) >> 8;
	W5500_tx_Buf[2] = (AddrSel & 0x000000FF) >> 0;
	W5500_tx_Buf[3] = Value;
	smp_spi_master_send_recv(&SPI_W5500, W5500_tx_Buf, 4, 0, 0, &W5500_CS);

}

void smp_w5500_spiDMA_read_byte(uint32_t AddrSel){	
	gu8W5500_SPI_Status = W5500_SPI_Busy;
	AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);
	W5500_tx_Buf[0] = (AddrSel & 0x00FF0000) >> 16;
	W5500_tx_Buf[1] = (AddrSel & 0x0000FF00) >> 8;
	W5500_tx_Buf[2] = (AddrSel & 0x000000FF) >> 0;
	smp_spi_master_send_recv(&SPI_W5500, W5500_tx_Buf, 3, &Readbyte_value, 1, &W5500_CS);
	
}
void smp_w5500_spiDMA_read_byte_block(uint32_t AddrSel){
	gu8W5500_SPI_Status = W5500_SPI_Busy;
	AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);
	W5500_tx_Buf[0] = (AddrSel & 0x00FF0000) >> 16;
	W5500_tx_Buf[1] = (AddrSel & 0x0000FF00) >> 8;
	W5500_tx_Buf[2] = (AddrSel & 0x000000FF) >> 0;
	smp_spi_master_send_recv_blocking(&SPI_W5500, W5500_tx_Buf, 3, &Readbyte_value, 1, &W5500_CS);

}

void smp_w5500_spiDMA_WriteMulti(uint32_t AddrSel, uint8_t* pBuf,uint16_t Len){
	uint8_t i;
	gu8W5500_SPI_Status = W5500_SPI_Busy;
	 AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);
	W5500_tx_Buf[0] = (AddrSel & 0x00FF0000) >> 16;
	W5500_tx_Buf[1] = (AddrSel & 0x0000FF00) >> 8;
	W5500_tx_Buf[2] = (AddrSel & 0x000000FF) >> 0;
	for(i=0;i<Len;i++){
		W5500_tx_Buf[i+3] = pBuf[i];
	}
	smp_spi_master_send_recv(&SPI_W5500, W5500_tx_Buf, Len+3, 0, 0, &W5500_CS);
	
}

void smp_w5500_spiDMA_ReadMulti(uint32_t AddrSel, uint16_t Len, uint8_t* Ptr){
	gu8W5500_SPI_Status = W5500_SPI_Busy;
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
			gu8W5500_SPI_Status = W5500_SPI_Idle;
			break;
		case SMP_SPI_EVENT_TRANSFER_BUSY:
			gu8W5500_SPI_Status = W5500_SPI_Busy;	
			break;
		case SMP_SPI_EVENT_TRANSFERR_ERROR:
			break;
		default:
			break;
	}
}


static void W5500SwTimerHandler_New(__far void *dest, uint16_t evt, void *vDataPtr){
	static int8_t SocketNum = 0;
	
	if(evt == LIB_SW_TIMER_EVT_SW_1MS){
		if(gu8_W5500_Init_Steps != Init_End){
			W5500_Init_Step(gu8_W5500_Init_Steps);
		}else{
			if(socketStatus[gu8SockNum] == SOCK_CLOSED){
				Socket_Event_Handling_Status[gu8SockNum] = Event_Handle_Ing;
				W5500_Socket_Reopen_Step(gu8Socket_Reopen_Step[gu8SockNum],gu8SockNum);
				if(Socket_Event_Handling_Status[gu8SockNum] == Event_Handle_Done){
					gu8SockNum++;
				}
				if(gu8SockNum >= SocketEnableCount){
					gu8SockNum = 0;	
				}
			}else if(W5500_INT_Cnt){
				if(!W5500_Read_SnIR_End){
					SocketNum = W5500_Read_Socket_INTReg();
				}else{	
					W5500_Server_Step_New(gu8Socket_Sever_Step[SocketNum], SocketNum);
					if(Socket_Event_Handling_Status[SocketNum] == Event_Handle_Done){
						if(W5500_INT_Cnt){
							W5500_INT_Cnt--;
						}
					}	
				}
			}
		}
		
	}

}


void smp_w5500_getMAC(void){
	if(gu8W5500_SPI_Status == W5500_SPI_Idle){
		smp_w5500_spiDMA_ReadMulti(SHAR, MAC_ADDR_BYTE_LEN, TmpNetInfo.mac);	
	}
}

void smp_w5500_getGW(void){
	if(gu8W5500_SPI_Status == W5500_SPI_Idle){
		smp_w5500_spiDMA_ReadMulti(GAR, GATE_WAY_ADDR_BYTE_LEN,  TmpNetInfo.gw);	
	}
}
void smp_w5500_getSUBMASK(void){
	if(gu8W5500_SPI_Status == W5500_SPI_Idle){
		smp_w5500_spiDMA_ReadMulti(SUBR, SUB_MASK_ADDR_BYTE_LEN,  TmpNetInfo.sn);	
	}
}
void smp_w5500_getLocalIP(void){
	if(gu8W5500_SPI_Status == W5500_SPI_Idle){
		smp_w5500_spiDMA_ReadMulti(SIPR, IP_ADDR_BYTE_LEN,  TmpNetInfo.ip);	
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

	/*Configure GPIO pins : PB6 RST */
	/*GPIO_InitStruct.Pin = BSP_W5500_RST_PIN ;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BSP_W5500_RST_PORT, &GPIO_InitStruct);*/
	
	W5500_RST_PIN.pin = BSP_W5500_RST_PIN;
	W5500_RST_PIN.port = BSP_W5500_RST_PORT;
	W5500_RST_PIN.mode = SMP_GPIO_MODE_OUTPUT_PP;
	smp_gpio_init(&W5500_RST_PIN); 
		
	
	/*Configure GPIO pin : PD8 INT Pin */  
	/*GPIO_InitStruct.Pin = BSP_W5500_INT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BSP_W5500_INT_PORT , &GPIO_InitStruct);*/
	
	/*EXTI Pin init*/
	W5500_INT_PIN.pin = BSP_W5500_INT_PIN;
	W5500_INT_PIN.port = BSP_W5500_INT_PORT;
	W5500_INT_PIN.mode = SMP_GPIO_MODE_IT_FALLING;
	smp_gpio_init(&W5500_INT_PIN);
	
	/*EXTI interrupt vector init*/
	HAL_NVIC_SetPriority(BSP_W5500_INT_PIN_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(BSP_W5500_INT_PIN_IRQn);
	
	W5500_SPI_CS_PIN.pin = BSP_W5500_SPI_SCS_PIN;
	W5500_SPI_CS_PIN.port = BSP_W5500_SPI_SCS_PORT;	
	
	W5500_CS.spi_num = SPI_module2;
	W5500_CS.cs_handler = W5500_SPI_CS_PIN;	
	smp_spi_master_cs_init(&W5500_CS);
	
	SPI_W5500.num = SPI_module2;
	SPI_W5500.mode = SPI_mode0;
	smp_spi_master_init(&SPI_W5500, spi_master_w5500_event_handler, false);

}

void Hal_W5500_Open(void){
	smp_w5500_spi_config();
	LibSwTimerOpen(W5500SwTimerHandler_New, 0);
	gu8_W5500_Init_Steps = Init_HW_Reset_1;
}

int8_t W5500_Socket_Register(W5500_Socket_parm *parm, smp_w5500_event_t w5500_event_Handler){
	uint8_t i;
	for(i=0; i<W5500_MAX_SOCKET_NUM;i++){		
		if(smpW5500Socket[i].SocketStatus == Socket_Disable){
			/* Socket i is availible, write parameter to socket parameter */
			smpW5500Socket[i].SocketStatus = Socket_Enable;
			smpW5500Socket[i].cbFunPtr = w5500_event_Handler;
			smpW5500Socket[i].Parm.Num = i;						//Socket number hardware(w5500) limit 0~7 
			smpW5500Socket[i].Parm.Memory.rx_buf_Ptr = parm->Memory.rx_buf_Ptr;
			smpW5500Socket[i].Parm.Memory.tx_buf_Ptr = parm->Memory.tx_buf_Ptr;
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
	socketStatus[i] = SOCK_CLOSED;
	return SMP_SUCCESS;
}

int8_t W5500_Init_Step(uint8_t Step){

	wiz_PhyConf PHYconf, PHYconf_Verify;
	static R_W_NetInfo InitSetNetInfo[4] = {
		{SHAR , MAC_ADDR_BYTE_LEN, DefMac},
		{GAR, GATE_WAY_ADDR_BYTE_LEN, DefGw},
		{SUBR, SUB_MASK_ADDR_BYTE_LEN, DefSn},
		{SIPR, IP_ADDR_BYTE_LEN, DefIp}
	};
	static R_W_NetInfo InitReadNetInfo[4] ={
		{SHAR , MAC_ADDR_BYTE_LEN, TmpNetInfo.mac},
		{GAR, GATE_WAY_ADDR_BYTE_LEN, TmpNetInfo.gw},
		{SUBR, SUB_MASK_ADDR_BYTE_LEN, TmpNetInfo.sn},
		{SIPR, IP_ADDR_BYTE_LEN, TmpNetInfo.ip}
	}; 
	static uint8_t SocketNum = 0;
	static uint8_t ms_Count = 0;
	static uint8_t Step_RW_NetInfo = 0;
	static int8_t smpResult = SMP_SUCCESS;
	
	if(gu8W5500_SPI_Status == W5500_SPI_Idle){	
		switch(Step){
			case Init_HW_Reset_1:
				//HAL_GPIO_WritePin(BSP_W5500_RST_PORT, BSP_W5500_RST_PIN, GPIO_PIN_RESET);
				smp_gpio_set_state(&W5500_RST_PIN, GPIO_ACTIVE_LOW);
				ms_Count = 0;
				gu8_W5500_Init_Steps = Init_HW_Reset_2;
			case Init_HW_Reset_2:
				ms_Count++;
				if(ms_Count == HW_RESET_DELAY_TIME_MS){
					ms_Count = 0;
					//HAL_GPIO_WritePin(BSP_W5500_RST_PORT, BSP_W5500_RST_PIN, GPIO_PIN_SET);
					smp_gpio_set_state(&W5500_RST_PIN, GPIO_ACTIVE_HIGH);
					gu8_W5500_Init_Steps = Init_SW_Reset;
				}
				break;
			case Init_SW_Reset:
				smp_w5500_spiDMA_write_byte(MR, MR_RST);
				gu8_W5500_Init_Steps = Init_PHY_Conf;
				break;
			case Init_PHY_Conf:
				PHYconf.by = PHY_CONFBY_SW;
				PHYconf.mode = PHY_MODE_MANUAL;
				PHYconf.speed = PHY_SPEED_100;
				PHYconf.duplex = PHY_DUPLEX_FULL;
				smp_w5500_spiDMA_write_byte(PHYCFGR, w5500_phyconf_transfer_to_uint8(PHYconf));
				gu8_W5500_Init_Steps = Init_PHY_Read;			
				break;
			case Init_PHY_Read:				
				smp_w5500_spiDMA_read_byte(PHYCFGR);
				gu8_W5500_Init_Steps = Init_PHY_Rst;				
				break;
			case Init_PHY_Rst:				
				gu8Temp = Readbyte_value;
				gu8Temp |= ~PHYCFGR_RST;
				smp_w5500_spiDMA_write_byte(PHYCFGR, gu8Temp);
				gu8_W5500_Init_Steps = Init_PHY_Verify_Read;				
				break;
			case Init_PHY_Verify_Read:				
				smp_w5500_spiDMA_read_byte(PHYCFGR);
				gu8_W5500_Init_Steps = Init_PHY_Verify;				
				break;
			case Init_PHY_Verify:
				PHYconf_Verify = w5500_uint8_transfer_to_phyconf(Readbyte_value);
				if((PHYconf_Verify.by != PHYconf.by)||(PHYconf_Verify.mode != PHYconf.mode)||(PHYconf_Verify.speed != PHYconf.speed)||(PHYconf_Verify.duplex != PHYconf.duplex)){
					smpResult = SMP_ERROR_NOT_FOUND;
				}
				gu8_W5500_Init_Steps = Init_SET_NetInfo;				
				break;
			case Init_SET_NetInfo:				
				smp_w5500_spiDMA_WriteMulti(InitSetNetInfo[Step_RW_NetInfo].AddrSel,InitSetNetInfo[Step_RW_NetInfo].pData,InitSetNetInfo[Step_RW_NetInfo].Len);
				Step_RW_NetInfo++;
				if(Step_RW_NetInfo == NETINFO_RW_STEP){
					Step_RW_NetInfo = 0;
					gu8_W5500_Init_Steps = Init_Read_NetInfo;		
				}
				break;
			case Init_Read_NetInfo:				
				smp_w5500_spiDMA_ReadMulti(InitReadNetInfo[Step_RW_NetInfo].AddrSel,InitReadNetInfo[Step_RW_NetInfo].Len, InitReadNetInfo[Step_RW_NetInfo].pData);
				Step_RW_NetInfo++;
				if(Step_RW_NetInfo == NETINFO_RW_STEP){
					Step_RW_NetInfo = 0;
					gu8_W5500_Init_Steps = Init_Verify_NetInfo;		
				}
				break;
			case Init_Verify_NetInfo:
#ifdef USE_DHCP
				TmpNetInfo.dhcp = NETINFO_DHCP;
#else
				TmpNetInfo.dhcp = NETINFO_STATIC;
#endif
				memcpy(TmpNetInfo.dns ,DefDns,  IP_ADDR_BYTE_LEN);
				if( (memcmp(TmpNetInfo.mac, DefMac, MAC_ADDR_BYTE_LEN)!=0) ||
					(memcmp(TmpNetInfo.ip, DefIp, IP_ADDR_BYTE_LEN)!=0) ||
					(memcmp(TmpNetInfo.sn, DefSn, SUB_MASK_ADDR_BYTE_LEN)!=0) ||
					(memcmp(TmpNetInfo.gw, DefGw, GATE_WAY_ADDR_BYTE_LEN)!=0)){
					smpResult = SMP_ERROR_NOT_FOUND;
				}
				gu8_W5500_Init_Steps = Init_INT_MASK_Config;
				break;
			case Init_INT_MASK_Config:
				smp_w5500_spiDMA_write_byte(_IMR_,0xFF);
				gu8_W5500_Init_Steps = Init_SocketNum_INT_MASK_Config;
				break;
			case Init_SocketNum_INT_MASK_Config:
				smp_w5500_spiDMA_write_byte(SIMR,0xFF);
				gu8_W5500_Init_Steps = Init_SocketEvent_INT_MASK_Config;
				break;
			case Init_SocketEvent_INT_MASK_Config:
				smp_w5500_spiDMA_write_byte(Sn_IMR(SocketNum), (Sn_IR_TIMEOUT|Sn_IR_RECV|Sn_IR_DISCON|Sn_IR_CON));
				SocketNum++;
				if(SocketNum == W5500_MAX_SOCKET_NUM){
					gu8_W5500_Init_Steps = Init_End;
				}
			break;
			default: 
				break;
		}
	}
	return smpResult;
}



int8_t W5500_Read_Socket_INTReg(void){
	static uint8_t socketNum = 0; 
		smp_w5500_spiDMA_read_byte_block(SIR);
		
		if(Readbyte_value){
			socketNum = Readbyte_value-1;
			Socket_Event_Handling_Status[socketNum] = Event_Handle_Ing;
			gu8Socket_Sever_Step[socketNum] = Read_Socket_INT_Event_New_1;
		}else{
			Socket_Event_Handling_Status[socketNum] = Event_Handle_Done;
		}
		W5500_Read_SnIR_End = true;
		return socketNum;
}

int8_t W5500_Socket_Reopen_Step(uint8_t Step, uint8_t SocketNum){
	static uint8_t ms_Count = 0,i,lu8Temp;
	static W5500_RegisterList *TempSocket;
		
	TempSocket = &smpW5500Socket[SocketNum];	
	if(gu8W5500_SPI_Status == W5500_SPI_Idle){	
		switch(Step){
			case Server_Read_Socket_Status:
				smp_w5500_spiDMA_read_byte(Sn_SR(SocketNum));
				gu8Socket_Reopen_Step[SocketNum] = Server_Check_Socket_Status;
				break;
			case Server_Check_Socket_Status:
				ms_Count++;
				if(ms_Count == 2){
					ms_Count = 0;
					switch(Readbyte_value){
						case SOCK_CLOSED:
							Socket_Event_Handling_Status[SocketNum] = Event_Handle_Ing;
							gu8Socket_Reopen_Step[SocketNum] = Socket_Open_Read_IP;
							break;
						case SOCK_INIT:
							gu8Socket_Reopen_Step[SocketNum] = Socket_Listen_Read_Mode;
							break;
						case SOCK_LISTEN:
							gu8Socket_Reopen_Step[SocketNum] = Server_Read_Socket_Status;
							socketStatus[SocketNum] = Socket_Open;
							Socket_Event_Handling_Status[SocketNum] = Event_Handle_Done;
							break;
					}
				}
				break;
			/******* Socket reset and open *******/
			case Socket_Open_Read_IP:
				smp_w5500_getLocalIP();
				gu8Socket_Reopen_Step[SocketNum] = Socket_Open_Check_IP;
				break;
			case Socket_Open_Check_IP:
				for(i=0;i<IP_ADDR_BYTE_LEN;i++){
					if(TmpNetInfo.ip[i]!=0){
						break;
					}
				}
				if(i == IP_ADDR_BYTE_LEN){
					return SMP_ERROR_NOT_FOUND;
				}else{
					gu8Socket_Reopen_Step[SocketNum] = Set_Socket_Close;
				}
				break;	
			case Set_Socket_Close:
				smp_w5500_spiDMA_write_byte(Sn_CR(SocketNum), Sn_CR_CLOSE);
				gu8Socket_Reopen_Step[SocketNum] = Read_Cmd_Register_Status_0;
				break;
			case Read_Cmd_Register_Status_0:
				smp_w5500_spiDMA_read_byte(Sn_CR(SocketNum));
				gu8Socket_Reopen_Step[SocketNum] = Check_Close_Cmd_Recv;
				break;
			case Check_Close_Cmd_Recv:
				if(Readbyte_value!=0){
					gu8Socket_Reopen_Step[SocketNum] = Read_Cmd_Register_Status_0;
				}else{
					gu8Socket_Reopen_Step[SocketNum] = RST_All_INT_Flag;
				}
				break;
			case RST_All_INT_Flag:
				smp_w5500_spiDMA_write_byte(Sn_IR(SocketNum), 0xFF&SOCKET_NUM_BASE);
				sock_io_mode &= ~(1<<SocketNum);
				sock_is_sending &= ~(1<<SocketNum);
				sock_remained_size[SocketNum] = 0;
				sock_pack_info[SocketNum] = 0;
				gu8Socket_Reopen_Step[SocketNum] = Read_Socket_Status_0;
				break;
			case Read_Socket_Status_0:
				smp_w5500_spiDMA_read_byte(Sn_SR(SocketNum));
				gu8Socket_Reopen_Step[SocketNum] = Verify_Socket_Close_0;
				break;
			case Verify_Socket_Close_0:
				if(Readbyte_value!= SOCK_CLOSED){
					gu8Socket_Reopen_Step[SocketNum] = Read_Socket_Status_0;
				}else{
					gu8Socket_Reopen_Step[SocketNum] = Set_Socket_Mode;
				}
				break;
			case Set_Socket_Mode:
				smp_w5500_spiDMA_write_byte(Sn_MR(SocketNum), TempSocket->Parm.Protocol);
				gu8Socket_Reopen_Step[SocketNum] = Set_Socket_Port_Highbyte;
				break;
			case Set_Socket_Port_Highbyte:
				if(!TempSocket->Parm.PortNum)
				{
				    TempSocket->Parm.PortNum = sock_any_port++;
				    if(sock_any_port == 0xFFF0) sock_any_port = SOCK_ANY_PORT_NUM;
				}else{
					gu8Temp = (uint8_t)(TempSocket->Parm.PortNum >> 8);
					smp_w5500_spiDMA_write_byte(Sn_PORT(SocketNum), gu8Temp);
					gu8Socket_Reopen_Step[SocketNum] = Set_Socket_Port_Lowbyte;
				}
				break;
			case Set_Socket_Port_Lowbyte:
					gu8Temp = (uint8_t)TempSocket->Parm.PortNum;
					smp_w5500_spiDMA_write_byte(WIZCHIP_OFFSET_INC(Sn_PORT(SocketNum),1), gu8Temp);
					gu8Socket_Reopen_Step[SocketNum] = Set_Socket_Open;
				break;
			case Set_Socket_Open:
					smp_w5500_spiDMA_write_byte(Sn_CR(SocketNum), Sn_CR_OPEN);
					gu8Socket_Reopen_Step[SocketNum] = Read_Cmd_Register_Status_1;
				break;
			case Read_Cmd_Register_Status_1:
					smp_w5500_spiDMA_read_byte(Sn_CR(SocketNum));
					gu8Socket_Reopen_Step[SocketNum] = Check_Open_Cmd_Recv;
				break;
			case Check_Open_Cmd_Recv:
				if(Readbyte_value!=0){
					gu8Socket_Reopen_Step[SocketNum] = Read_Cmd_Register_Status_1;
				}else{
					sock_io_mode &= ~(1 <<SocketNum);
					sock_io_mode |= ((0x00 & SF_IO_NONBLOCK) << SocketNum);   
					sock_is_sending &= ~(1<<SocketNum);
					sock_remained_size[SocketNum] = 0;
					sock_pack_info[SocketNum] = PACK_COMPLETED;
					gu8Socket_Reopen_Step[SocketNum] = Read_Socket_Status_1;
				}	
				break;
			case Read_Socket_Status_1:
				smp_w5500_spiDMA_read_byte(Sn_SR(SocketNum));
				gu8Socket_Reopen_Step[SocketNum] = Verify_Socket_Close_1;
				break;
			case Verify_Socket_Close_1:
				if(Readbyte_value == SOCK_CLOSED){
					gu8Socket_Reopen_Step[SocketNum] = Read_Socket_Status_1;
				}else{
					gu8Socket_Reopen_Step[SocketNum] = Server_Read_Socket_Status;
				}
				break;
			/************ End ************/
			/******* Socket listen *******/	
			case Socket_Listen_Read_Mode:
				smp_w5500_spiDMA_read_byte(Sn_MR(SocketNum));
				gu8Socket_Reopen_Step[SocketNum] = Socket_Listen_Verify_Mode;
				break;
			case Socket_Listen_Verify_Mode:
				if((Readbyte_value&0x0F)!= TempSocket->Parm.Protocol){
					return SMP_ERROR_NOT_FOUND;
				}else{
					gu8Socket_Reopen_Step[SocketNum] = Set_Socket_Listen;
				}
			case Set_Socket_Listen:
				smp_w5500_spiDMA_write_byte(Sn_CR(SocketNum), Sn_CR_LISTEN);
				gu8Socket_Reopen_Step[SocketNum] = Read_Cmd_Register_Status_2;
				break;
			case Read_Cmd_Register_Status_2:
				smp_w5500_spiDMA_read_byte(Sn_CR(SocketNum));
				gu8Socket_Reopen_Step[SocketNum] = Check_Listen_Cmd_Recv;
				break;
			case Check_Listen_Cmd_Recv:
				if(Readbyte_value != 0){
					gu8Socket_Reopen_Step[SocketNum] = Read_Cmd_Register_Status_2;
				}else{
					gu8Socket_Reopen_Step[SocketNum] = Read_Socket_Status_2;
				}
				break;
			case Read_Socket_Status_2:
				smp_w5500_spiDMA_read_byte(Sn_SR(SocketNum));
				gu8Socket_Reopen_Step[SocketNum] = Verify_Socket_Listen;
				break;
			case  Verify_Socket_Listen:
				if(Readbyte_value != SOCK_LISTEN){
					return SMP_ERROR_NOT_FOUND;
				}else{
					gu8Socket_Reopen_Step[SocketNum] = Server_Read_Socket_Status;
				}
				break;
			/************ End ************/
		
			
		}
	}
	return SMP_SUCCESS;
}


int8_t W5500_Server_Step_New(uint8_t Step, uint8_t SocketNum){
	static int8_t smpResult = SMP_SUCCESS;
	static W5500_RegisterList *TempSocket;
	static uint8_t ms_Count = 0, recv_no_data_cnt = 0;
	TempSocket = &smpW5500Socket[SocketNum];
	if(gu8W5500_SPI_Status == W5500_SPI_Idle){	
		switch(Step){
			case Read_Socket_INT_Event_New_1:
				smp_w5500_spiDMA_read_byte(Sn_IR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Check_INT_Event;
				break;
			case Check_INT_Event:
				switch(Readbyte_value){
					case Sn_IR_CON:
						gu8Socket_Sever_Step[SocketNum] = Clear_CON_INT_Flag;
						break;
					case Sn_IR_RECV:
						gu8Socket_Sever_Step[SocketNum] = Read_Recv_Cnt_HighByte;
						break;
					case Sn_IR_DISCON:
						gu8Socket_Sever_Step[SocketNum] = Set_Socket_Disconnect;
						break;
					case Sn_IR_TIMEOUT:
						//while(1);
						break;
					case Sn_IR_SENDOK:
						//while(1);
						break;
					default:
						Socket_Event_Handling_Status[SocketNum] = Event_Handle_Done;
						gu8Socket_Sever_Step[SocketNum] = Read_Socket_INT_Event_New_1;
						break;
				}
				break;
			/****** Socket establish : Sn_IR_CON*****/			
			case Clear_CON_INT_Flag:
				smp_w5500_spiDMA_write_byte(Sn_IR(SocketNum), (Sn_IR_CON&SOCKET_NUM_BASE)); 
				gu8Socket_Sever_Step[SocketNum] = Read_Remote_IP;
				break;
			case Read_Remote_IP:
				smp_w5500_spiDMA_ReadMulti(Sn_DIPR(SocketNum),IP_ADDR_BYTE_LEN, RemoteNetInfo.ip);
				gu8Socket_Sever_Step[SocketNum] =  Read_Remote_Port_HighByte;
				break;
			case Read_Remote_Port_HighByte:
				smp_w5500_spiDMA_read_byte(Sn_DPORT(SocketNum));
				gu8Socket_Sever_Step[SocketNum]  = Save_High_Byte_Read_Remote_Port_LowByte;
			case Save_High_Byte_Read_Remote_Port_LowByte:
				ms_Count++;
				if(ms_Count == 2){
					ms_Count = 0;
					smpW5500Socket[SocketNum].Parm.DestPort = ((uint16_t)Readbyte_value)<<8;
					smp_w5500_spiDMA_read_byte(WIZCHIP_OFFSET_INC(Sn_DPORT(SocketNum),1));
					gu8Socket_Sever_Step[SocketNum] = Save_Port_Low_Byte;
				}
				break;
			case Save_Port_Low_Byte:
				ms_Count++;
				if(ms_Count == 2){
					ms_Count = 0;
					smpW5500Socket[SocketNum].Parm.DestPort += Readbyte_value;
					gu8Socket_Sever_Step[SocketNum] = Read_Socket_INT_Event_New_1;
					Socket_Event_Handling_Status[SocketNum] = Event_Handle_Done;
				}
				break;
			/******  End *****/	
			/****** Socket establish : Sn_IR_Recv *****/					
			case Read_Recv_Cnt_HighByte:
				smp_w5500_spiDMA_read_byte(Sn_RX_RSR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Read_Recv_Cnt_LowByte;
				break;
			case Read_Recv_Cnt_LowByte:
				ms_Count++;
				if(ms_Count == 2){
					ms_Count = 0;
					gu16RecvBytesCnt = Readbyte_value;
					gu16RecvBytesCnt = gu16RecvBytesCnt<<8;
					//gu16Temp = (uint16_t)Readbyte_value;
					//gu16Temp = (gu16Temp<<8);
					smp_w5500_spiDMA_read_byte(WIZCHIP_OFFSET_INC(Sn_RX_RSR(SocketNum),1));
					gu8Socket_Sever_Step[SocketNum] = Check_Recv_Data;		
				}
				break;
			case Check_Recv_Data:
				ms_Count++;
				if(ms_Count == 2){
					gu16RecvBytesCnt = gu16RecvBytesCnt+Readbyte_value;
					SocketHandleByteLen[SocketNum] = gu16RecvBytesCnt;
					ms_Count = 0;
					if(SocketHandleByteLen[SocketNum]> 0){
						gu8Socket_Sever_Step[SocketNum] = Read_Socket_Buf_Addr_HighByte;
					}else{
						gu8Socket_Sever_Step[SocketNum] =  Read_Recv_Cnt_HighByte;
						recv_no_data_cnt++;
						if(recv_no_data_cnt == 2){
							recv_no_data_cnt = 0;
							gu8Socket_Sever_Step[SocketNum] = Read_Socket_INT_Event_New_1;
							Socket_Event_Handling_Status[SocketNum] = Event_Handle_Done;
						}
					}
				}
				break;
			case Read_Socket_Buf_Addr_HighByte:
				smp_w5500_spiDMA_read_byte(Sn_RX_RD(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Read_Socket_Buf_Addr_LowByte;
				break;
			case Read_Socket_Buf_Addr_LowByte:
				ms_Count++;
				if(ms_Count == 2){
					ms_Count = 0;
					gu16Temp = Readbyte_value;
					gu16Temp = gu16Temp <<8;
					smp_w5500_spiDMA_read_byte(WIZCHIP_OFFSET_INC(Sn_RX_RD(SocketNum),1));
					gu8Socket_Sever_Step[SocketNum] = Read_Socket_Buf_data;
				}
				break;
			case Read_Socket_Buf_data:
				ms_Count++;
				if(ms_Count == 2){
					ms_Count = 0;
					gu16Temp += Readbyte_value;
					gu32Temp = ((uint32_t)gu16Temp<<8)+ (WIZCHIP_RXBUF_BLOCK(SocketNum)<< 3); 
					smp_w5500_spiDMA_ReadMulti(gu32Temp,SocketHandleByteLen[SocketNum], TempSocket->Parm.Memory.rx_buf_Ptr);
					gu8Socket_Sever_Step[SocketNum] = ParserData;
				}
				
				break;
			case ParserData:
				gu16Temp += SocketHandleByteLen[SocketNum];
				if(TempSocket->cbFunPtr){
					SocketHandleByteLen[SocketNum] = TempSocket->cbFunPtr(W5500_DATA_RECV,SocketHandleByteLen[SocketNum]);
				}
				gu8Socket_Sever_Step[SocketNum] = Update_Buf_Offset_Highbyte;
				break;
			case Update_Buf_Offset_Highbyte:
				ms_Count++;
				if(ms_Count == 2){
					ms_Count = 0;
					smp_w5500_spiDMA_write_byte(Sn_RX_RD(SocketNum),(uint8_t)(gu16Temp>>8));
					gu8Socket_Sever_Step[SocketNum] = Update_Buf_Offset_Lowbyte;				
				}
				break;
			case Update_Buf_Offset_Lowbyte:
				ms_Count++;
				if(ms_Count == 2){
					ms_Count = 0;
					smp_w5500_spiDMA_write_byte(WIZCHIP_OFFSET_INC(Sn_RX_RD(SocketNum),1),(uint8_t)gu16Temp);
					gu8Socket_Sever_Step[SocketNum] = Set_Scoket_Recv;
				}
				break;
			case Set_Scoket_Recv:
				smp_w5500_spiDMA_write_byte(Sn_CR(SocketNum), Sn_CR_RECV);
				gu8Socket_Sever_Step[SocketNum] = Read_Cmd_Register_Status_3;
				break;
			case Read_Cmd_Register_Status_3:
				smp_w5500_spiDMA_read_byte(Sn_CR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Check_Recv_Cmd_Recv;
				break;
			case Check_Recv_Cmd_Recv:
				if(Readbyte_value!=0){
					gu8Socket_Sever_Step[SocketNum] = Read_Cmd_Register_Status_3;
				}else{
					gu8Socket_Sever_Step[SocketNum] = Read_Tx_Buf_Max_Len;
				}
				break;
			case Read_Tx_Buf_Max_Len:
				smp_w5500_spiDMA_read_byte(Sn_TXBUF_SIZE(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Check_Response_Oversize;
				break;
			case Check_Response_Oversize:
				gu16Temp = ((uint16_t)Readbyte_value)<<10;
				if(SocketHandleByteLen[SocketNum] > gu16Temp){
					SocketHandleByteLen[SocketNum] = gu16Temp;
				}
				gu8Socket_Sever_Step[SocketNum] = Read_W5500_Tx_FSR_High;
				break;
			case Read_W5500_Tx_FSR_High:
				smp_w5500_spiDMA_read_byte(Sn_TX_FSR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Read_W5500_Tx_FSR_Low;
				break;
			case Read_W5500_Tx_FSR_Low:
				smp_w5500_spiDMA_read_byte(WIZCHIP_OFFSET_INC(Sn_TX_FSR(SocketNum),1));
				gu8Socket_Sever_Step[SocketNum] = Read_W5500_Tx_Ptr_Highbyte;
				break;
			case Read_W5500_Tx_Ptr_Highbyte:
				smp_w5500_spiDMA_read_byte(Sn_TX_WR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Read_W5500_Tx_Ptr_Lowbyte;
				break;
			case Read_W5500_Tx_Ptr_Lowbyte:
				gu16Temp = Readbyte_value;
				gu16Temp = gu16Temp<<8;
				smp_w5500_spiDMA_read_byte(WIZCHIP_OFFSET_INC(Sn_TX_WR(SocketNum),1));
				gu8Socket_Sever_Step[SocketNum] = Get_W5500_Tx_Ptr_Addr_and_Trans;
				break;
			case Get_W5500_Tx_Ptr_Addr_and_Trans:
				gu16Temp +=(uint16_t)Readbyte_value;
				gu32Temp = (((uint32_t)gu16Temp) << 8) + (WIZCHIP_TXBUF_BLOCK(SocketNum)<< 3);
				smp_w5500_spiDMA_WriteMulti(gu32Temp, TempSocket->Parm.Memory.tx_buf_Ptr, SocketHandleByteLen[SocketNum]);
				gu8Socket_Sever_Step[SocketNum] = Update_Tx_Ptr_Highbyte;
				break;
			case Update_Tx_Ptr_Highbyte:
				gu16Temp += SocketHandleByteLen[SocketNum]; 
				SocketHandleByteLen[SocketNum] = 0;
				smp_w5500_spiDMA_write_byte(Sn_TX_WR(SocketNum),(uint8_t)(gu16Temp>>8));
				gu8Socket_Sever_Step[SocketNum] = Update_Tx_Ptr_Lowbyte;
				break;
			case Update_Tx_Ptr_Lowbyte:
				smp_w5500_spiDMA_write_byte(WIZCHIP_OFFSET_INC(Sn_TX_WR(SocketNum),1),(uint8_t)gu16Temp);
				gu8Socket_Sever_Step[SocketNum] = Set_Socket_Send;
				break;
			case Set_Socket_Send:
				smp_w5500_spiDMA_write_byte(Sn_CR(SocketNum), Sn_CR_SEND);
				gu8Socket_Sever_Step[SocketNum] = Read_Cmd_Register_Status_4;
				break;			
			case Read_Cmd_Register_Status_4:
				smp_w5500_spiDMA_read_byte(Sn_CR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Check_Send_Cmd_Recv;
				break;
			case Check_Send_Cmd_Recv:
				if(Readbyte_value!=0){
					gu8Socket_Sever_Step[SocketNum] = Read_Cmd_Register_Status_4;
				}else{
					gu8Socket_Sever_Step[SocketNum] = Read_Socket_INT_event_1;
				}
				break;
			case Read_Socket_INT_event_1:
				smp_w5500_spiDMA_read_byte_block(Sn_IR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Verify_Send_End;
				break;
			case Verify_Send_End:
				if((Readbyte_value&=SOCKET_NUM_BASE)&Sn_IR_RECV ){
					gu8Socket_Sever_Step[SocketNum] = Clear_Recv_INT_Flag;
				}else{
					gu8Socket_Sever_Step[SocketNum] = Read_Socket_INT_Event_New_1;
				}
			case Clear_Recv_INT_Flag:
				smp_w5500_spiDMA_write_byte(Sn_IR(SocketNum), ((Sn_IR_RECV|Sn_IR_SENDOK)&SOCKET_NUM_BASE)); 
				gu8Socket_Sever_Step[SocketNum] = Read_Socket_INT_event_1;
				Socket_Event_Handling_Status[SocketNum] = Event_Handle_Done;
				break;
			/************ End ************/
			/******* Socket disconnect *******/	
			case Set_Socket_Disconnect:
				smp_w5500_spiDMA_write_byte(Sn_CR(SocketNum), Sn_CR_DISCON);
				gu8Socket_Sever_Step[SocketNum] = Read_Cmd_Register_Status_5;
				break;				
			case Read_Cmd_Register_Status_5:
				smp_w5500_spiDMA_read_byte(Sn_CR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Check_Discon_Cmd_Recv;
				break;
			case Check_Discon_Cmd_Recv:
				if(Readbyte_value!=0){
					gu8Socket_Sever_Step[SocketNum] = Read_Cmd_Register_Status_5;
				}else{
					gu8Socket_Sever_Step[SocketNum] = Clear_DISCON_INT_Flag;
				}
				break;	
			case Clear_DISCON_INT_Flag:
				if(ms_Count==0){
					smp_w5500_spiDMA_write_byte(Sn_IR(SocketNum), (Sn_IR_DISCON &SOCKET_NUM_BASE)); 
					ms_Count++;
				}else{
					W5500_Socket_Reopen_Step(gu8Socket_Reopen_Step[SocketNum],SocketNum);
					if(Socket_Event_Handling_Status[SocketNum] == Event_Handle_Done){
						ms_Count = 0;
						gu8Socket_Sever_Step[SocketNum] = Read_Socket_INT_Event_New_1;
					}
				}
				break;
		}
	}

	return smpResult;

}


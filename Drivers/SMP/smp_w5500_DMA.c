/**
******************************************************************************
* @file    smp_W5500_DMA.c
* @author  Steve Cheng/ Golden
* @version V0.0.1
* @date    2021/12/30
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
#include "Bsp.h"
#include "smp_w5500_DMA.h"
#include <stdbool.h>
#include <string.h>

#define _W5500_SPI_VDM_OP_                     0x00
#define _W5500_SPI_FDM_OP_LEN1_                0x01
#define _W5500_SPI_FDM_OP_LEN2_                0x02
#define _W5500_SPI_FDM_OP_LEN4_                0x03

#define SOCK_ANY_PORT_NUM  0xC000
static uint16_t sock_any_port = SOCK_ANY_PORT_NUM;
static uint16_t sock_io_mode = 0;
static uint16_t sock_is_sending = 0;
static uint16_t sock_remained_size[_WIZCHIP_SOCK_NUM_] = {0,0,};
uint8_t  sock_pack_info[_WIZCHIP_SOCK_NUM_] = {0,};

volatile uint16_t SocketHandleByteLen[W5500_MAX_SOCKET_NUM] = {0,0,0,0,0,0,0,0};
volatile uint16_t gu16Temp;
static uint8_t socketStatus[W5500_MAX_SOCKET_NUM];
static uint8_t SocketEnableCount = 0;
uint8_t Socket_Event_Handling_Status[W5500_MAX_SOCKET_NUM];

static uint8_t gu8W5500_SPI_Status = W5500_SPI_Idle;
static uint8_t gu8_W5500_Init_Steps = Init_End;
static uint8_t gu8Socket_Sever_Step[W5500_MAX_SOCKET_NUM] = {0};

static uint8_t DefMac[MAC_ADDR_BYTE_LEN] = DEFAULT_MAC_ADDR;
static uint8_t DefIp[IP_ADDR_BYTE_LEN] = DEFAULT_IP_ADDR;
static uint8_t DefSn[SUB_MASK_ADDR_BYTE_LEN] = DEFAULT_SUB_MASK;
static uint8_t DefGw[GATE_WAY_ADDR_BYTE_LEN] = DEFAULT_GW_ADDR;
static uint8_t DefDns[IP_ADDR_BYTE_LEN] = DEFAULT_DNS_ADDR;

uint8_t W5500_tx_Buf[BSP_SPI2_TX_BUFFER_SIZE];
uint8_t Readbyte_value;
bool W5500INT_Low;

smp_gpio_t		PB12;
smp_spi_cs_t 	W5500_CS;
smp_spi_t 		SPI_W5500;

wiz_NetInfo TmpNetInfo,RemoteNetInfo;

W5500_RegisterList smpW5500Socket[W5500_MAX_SOCKET_NUM];

/**
  * @brief This function handles W5500 INT EXTI Pin interrupt.
  */
void EXTI9_5_IRQHandler(void)
{
#ifdef PCB_VER_1
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
#else
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
#endif

}

void smp_w5500_spiDMA_write_byte(uint32_t AddrSel, uint8_t Value){
	uint8_t spi_data[4];
	gu8W5500_SPI_Status = W5500_SPI_Busy;
	AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);
	spi_data[0] = (AddrSel & 0x00FF0000) >> 16;
	spi_data[1] = (AddrSel & 0x0000FF00) >> 8;
	spi_data[2] = (AddrSel & 0x000000FF) >> 0;
	spi_data[3] = Value;
	smp_spi_master_send_recv(&SPI_W5500, spi_data, 4, 0, 0, &W5500_CS);

}

void smp_w5500_spiDMA_read_byte(uint32_t AddrSel){
	uint8_t spi_data[3];
	
	gu8W5500_SPI_Status = W5500_SPI_Busy;
	AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);
	spi_data[0] = (AddrSel & 0x00FF0000) >> 16;
	spi_data[1] = (AddrSel & 0x0000FF00) >> 8;
	spi_data[2] = (AddrSel & 0x000000FF) >> 0;
	smp_spi_master_send_recv(&SPI_W5500, spi_data, 3, &Readbyte_value, 1, &W5500_CS);
	
}

void smp_w5500_spiDMA_WriteMulti(uint32_t AddrSel, uint8_t* pBuf,uint16_t Len){
	uint8_t spi_data[3],i;
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
	uint8_t spi_data[3];
	
	gu8W5500_SPI_Status = W5500_SPI_Busy;
	AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);
	spi_data[0] = (AddrSel & 0x00FF0000) >> 16;
	spi_data[1] = (AddrSel & 0x0000FF00) >> 8;
	spi_data[2] = (AddrSel & 0x000000FF) >> 0;
	smp_spi_master_send_recv(&SPI_W5500, spi_data, 3, Ptr, Len, &W5500_CS);
	
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

static void W5500SwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr){
	static uint8_t SocketNum = 0;
	static uint8_t EventDoneCheck = 0;
	if(evt == LIB_SW_TIMER_EVT_SW_1MS){
		if(gu8_W5500_Init_Steps != Init_End){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
			W5500_Init_Step(gu8_W5500_Init_Steps);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
		}else{
			if(W5500INT_Low){
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
				if(smpW5500Socket[SocketNum].SocketStatus == Socket_Enable){
					Socket_Event_Handling_Status[SocketNum] = Event_Handle_Ing;
					W5500_Server_Step(gu8Socket_Sever_Step[SocketNum],SocketNum);
				}
				if(Socket_Event_Handling_Status[SocketNum] == Event_Handle_Done){
					SocketNum++;
				}
				if(SocketNum == SocketEnableCount){
					SocketNum = 0;
					W5500INT_Low = false;
				}
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
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

	
	
#ifdef PCB_VER_1
	/*Configure GPIO pins : PE1 RST for PCB 1*/
	__HAL_RCC_GPIOE_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	/*Configure GPIO pin : PB7 INT Pin for PCB 1 */  
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#else
	/*Configure GPIO pins : PB6 RST */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/*Configure GPIO pin : PD8 INT Pin */  
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
#endif	
	
	/*EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	
	PB12.port = SMP_GPIOB;
	PB12.pin = PIN12;	
	
	W5500_CS.spi_num = SPI_module2;
	W5500_CS.cs_handler = PB12;	
	smp_spi_master_cs_init(&W5500_CS);
	
	SPI_W5500.num = SPI_module2;
	SPI_W5500.mode = SPI_mode0;
	smp_spi_master_init(&SPI_W5500, spi_master_w5500_event_handler, false);

}

void Hal_W5500_Open(void){
	smp_w5500_spi_config();
	LibSwTimerOpen(W5500SwTimerHandler, 0);
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
	W5500INT_Low = true;
	SocketEnableCount++;	
	
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
	uint8_t Tmp;
	static uint8_t SocketNum = 0;
	static uint8_t ms_Count = 0;
	static uint8_t Step_RW_NetInfo = 0;
	static int8_t smpResult = SMP_SUCCESS;
	bool CommEndFlag;
	
	if(gu8W5500_SPI_Status == W5500_SPI_Idle){	
		switch(Step){
			case Init_HW_Reset_1:
				HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_RESET);
				ms_Count = 0;
				gu8_W5500_Init_Steps = Init_HW_Reset_2;
			case Init_HW_Reset_2:
				ms_Count++;
				if(ms_Count == HW_RESET_DELAY_TIME_MS){
					ms_Count = 0;
					HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_SET);
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
				Tmp = Readbyte_value;
				Tmp |= ~PHYCFGR_RST;
				smp_w5500_spiDMA_write_byte(PHYCFGR, Tmp);
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

int8_t W5500_Server_Step(uint8_t Step, uint8_t SocketNum){
	static int8_t smpResult = SMP_SUCCESS;
	static W5500_RegisterList *TempSocket;
	static uint8_t ms_Count = 0;
	uint8_t i,lu8TTemp;
	uint16_t lu16Temp;
	uint32_t lu32Temp;
	TempSocket = &smpW5500Socket[SocketNum];
	if(gu8W5500_SPI_Status == W5500_SPI_Idle){	
		switch(Step){
			case Server_Read_Socket_Status:
				Socket_Event_Handling_Status[SocketNum] = Event_Handle_Ing;
				smp_w5500_spiDMA_read_byte(Sn_SR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Server_Check_Socket_Status;
				break;
			case Server_Check_Socket_Status:
				socketStatus[SocketNum] = Readbyte_value;
				switch(Readbyte_value){
					case SOCK_CLOSED:
						gu8Socket_Sever_Step[SocketNum] = Socket_Open_Read_IP;
						break;
					case SOCK_INIT:
						gu8Socket_Sever_Step[SocketNum] = Socket_Listen_Read_Mode;
						break;
					case SOCK_LISTEN:
						gu8Socket_Sever_Step[SocketNum] = Server_Read_Socket_Status;
						Socket_Event_Handling_Status[SocketNum] = Event_Handle_Done;
						//W5500INT_Low = false;
						break;
					case SOCK_SYNRECV:
						gu8Socket_Sever_Step[SocketNum] = Read_Socket_INT_event;
						break;
					case SOCK_ESTABLISHED:
						gu8Socket_Sever_Step[SocketNum] = Read_Socket_INT_event;
						break;
					case SOCK_CLOSE_WAIT:
						gu8Socket_Sever_Step[SocketNum] = Set_Socket_Disconnect;
						break;
					case SOCK_LAST_ACK:
						gu8Socket_Sever_Step[SocketNum] = Set_Socket_Disconnect;
						break;
					default:
						break;
				}	
				break;
			/******* Socket reset and open *******/
			case Socket_Open_Read_IP:
				smp_w5500_getLocalIP();
				gu8Socket_Sever_Step[SocketNum] = Socket_Open_Check_IP;
				break;
			case Socket_Open_Check_IP:
				for(i=0;i<IP_ADDR_BYTE_LEN;i++){
					if(TmpNetInfo.ip[i]!=0){
						break;
					}
				}
				if(i == IP_ADDR_BYTE_LEN){
					//while(1);
					smpResult = SMP_ERROR_NOT_FOUND;
				}else{
					gu8Socket_Sever_Step[SocketNum] = Set_Socket_Close;
				}
				break;	
			case Set_Socket_Close:
				smp_w5500_spiDMA_write_byte(Sn_CR(SocketNum), Sn_CR_CLOSE);
				gu8Socket_Sever_Step[SocketNum] = Read_Cmd_Register_Status_0;
				break;
			case Read_Cmd_Register_Status_0:
				smp_w5500_spiDMA_read_byte(Sn_CR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Check_Close_Cmd_Recv;
				break;
			case Check_Close_Cmd_Recv:
				if(Readbyte_value!=0){
					gu8Socket_Sever_Step[SocketNum] = Read_Cmd_Register_Status_0;
				}else{
					gu8Socket_Sever_Step[SocketNum] = RST_All_INT_Flag;
				}
				break;
			case RST_All_INT_Flag:
				smp_w5500_spiDMA_write_byte(Sn_IR(SocketNum), 0xFF&SOCKET_NUM_BASE);
				sock_io_mode &= ~(1<<SocketNum);
				sock_is_sending &= ~(1<<SocketNum);
				sock_remained_size[SocketNum] = 0;
				sock_pack_info[SocketNum] = 0;
				gu8Socket_Sever_Step[SocketNum] = Read_Socket_Status_0;
				break;
			case Read_Socket_Status_0:
				smp_w5500_spiDMA_read_byte(Sn_SR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Verify_Socket_Close_0;
				break;
			case Verify_Socket_Close_0:
				if(Readbyte_value!= SOCK_CLOSED){
					gu8Socket_Sever_Step[SocketNum] = Read_Socket_Status_0;
				}else{
					gu8Socket_Sever_Step[SocketNum] = Set_Socket_Mode;
				}
				break;
			case Set_Socket_Mode:
				smp_w5500_spiDMA_write_byte(Sn_MR(SocketNum), TempSocket->Parm.Protocol);
				gu8Socket_Sever_Step[SocketNum] = Set_Socket_Port_Highbyte;
				break;
			case Set_Socket_Port_Highbyte:
				if(!TempSocket->Parm.PortNum)
				{
				    TempSocket->Parm.PortNum = sock_any_port++;
				    if(sock_any_port == 0xFFF0) sock_any_port = SOCK_ANY_PORT_NUM;
				}else{
					lu8TTemp = (uint8_t)(TempSocket->Parm.PortNum >> 8);
					smp_w5500_spiDMA_write_byte(Sn_PORT(SocketNum), lu8TTemp);
					gu8Socket_Sever_Step[SocketNum] = Set_Socket_Port_Lowbyte;
				}
				break;
			case Set_Socket_Port_Lowbyte:
					lu8TTemp = (uint8_t)TempSocket->Parm.PortNum;
					smp_w5500_spiDMA_write_byte(WIZCHIP_OFFSET_INC(Sn_PORT(SocketNum),1), lu8TTemp);
					gu8Socket_Sever_Step[SocketNum] = Set_Socket_Open;
				break;
			case Set_Socket_Open:
					smp_w5500_spiDMA_write_byte(Sn_CR(SocketNum), Sn_CR_OPEN);
					gu8Socket_Sever_Step[SocketNum] = Read_Cmd_Register_Status_1;
				break;
			case Read_Cmd_Register_Status_1:
					smp_w5500_spiDMA_read_byte(Sn_CR(SocketNum));
					gu8Socket_Sever_Step[SocketNum] = Check_Open_Cmd_Recv;
				break;
			case Check_Open_Cmd_Recv:
				if(Readbyte_value!=0){
					gu8Socket_Sever_Step[SocketNum] = Read_Cmd_Register_Status_1;
				}else{
					sock_io_mode &= ~(1 <<SocketNum);
					sock_io_mode |= ((0x00 & SF_IO_NONBLOCK) << SocketNum);   
					sock_is_sending &= ~(1<<SocketNum);
					sock_remained_size[SocketNum] = 0;
					sock_pack_info[SocketNum] = PACK_COMPLETED;
					gu8Socket_Sever_Step[SocketNum] = Read_Socket_Status_1;
				}	
				break;
			case Read_Socket_Status_1:
				smp_w5500_spiDMA_read_byte(Sn_SR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Verify_Socket_Close_1;
				break;
			case Verify_Socket_Close_1:
				if(Readbyte_value == SOCK_CLOSED){
					gu8Socket_Sever_Step[SocketNum] = Read_Socket_Status_1;
				}else{
					gu8Socket_Sever_Step[SocketNum] = Server_Read_Socket_Status;
				}
				break;
			/************ End ************/
			/******* Socket listen *******/	
			case Socket_Listen_Read_Mode:
				smp_w5500_spiDMA_read_byte(Sn_MR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Socket_Listen_Verify_Mode;
				break;
			case Socket_Listen_Verify_Mode:
				if((Readbyte_value&0x0F)!= TempSocket->Parm.Protocol){
					smpResult = SMP_ERROR_NOT_FOUND;
				}else{
					gu8Socket_Sever_Step[SocketNum] = Set_Socket_Listen;
				}
			case Set_Socket_Listen:
				smp_w5500_spiDMA_write_byte(Sn_CR(SocketNum), Sn_CR_LISTEN);
				gu8Socket_Sever_Step[SocketNum] = Read_Cmd_Register_Status_2;
				break;
			case Read_Cmd_Register_Status_2:
				smp_w5500_spiDMA_read_byte(Sn_CR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Check_Listen_Cmd_Recv;
				break;
			case Check_Listen_Cmd_Recv:
				if(Readbyte_value != 0){
					gu8Socket_Sever_Step[SocketNum] = Read_Cmd_Register_Status_2;
				}else{
					gu8Socket_Sever_Step[SocketNum] = Read_Socket_Status_2;
				}
				break;
			case Read_Socket_Status_2:
				smp_w5500_spiDMA_read_byte(Sn_SR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Verify_Socket_Listen;
				break;
			case  Verify_Socket_Listen:
				if(Readbyte_value != SOCK_LISTEN){
					smpResult = SMP_ERROR_NOT_FOUND;
				}else{
					gu8Socket_Sever_Step[SocketNum] = Server_Read_Socket_Status;
				}
				break;
			/************ End ************/
			/****** Socket establish *****/		
			case Read_Socket_INT_event:
				smp_w5500_spiDMA_read_byte(Sn_IR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Verify_Connect_Event;
				break;
			case Verify_Connect_Event:
				lu8TTemp = Readbyte_value;
				if((lu8TTemp&=SOCKET_NUM_BASE)&Sn_IR_CON){
					gu8Socket_Sever_Step[SocketNum] = Clear_CON_INT_Flag;
				}else{
					gu8Socket_Sever_Step[SocketNum] =  Read_Recv_Cnt_HighByte;
				}
				break;
			case Clear_CON_INT_Flag:
				smp_w5500_spiDMA_write_byte(Sn_IR(SocketNum), (Sn_IR_CON&SOCKET_NUM_BASE)); 
				gu8Socket_Sever_Step[SocketNum] = Read_Socket_INT_event;
				break;
			case Read_Remote_IP:
				smp_w5500_spiDMA_ReadMulti(Sn_DIPR(SocketNum),IP_ADDR_BYTE_LEN, RemoteNetInfo.ip);
				gu8Socket_Sever_Step[SocketNum] =  Read_Remote_Port_HighByte;
				break;
			case Read_Remote_Port_HighByte:
				smp_w5500_spiDMA_read_byte(Sn_DPORT(SocketNum));
				gu8Socket_Sever_Step[SocketNum]  = Save_High_Byte_Read_Remote_Port_LowByte;
			case Save_High_Byte_Read_Remote_Port_LowByte:
				smpW5500Socket[SocketNum].Parm.DestPort = ((uint16_t)Readbyte_value)<<8;
				smp_w5500_spiDMA_read_byte(WIZCHIP_OFFSET_INC(Sn_DPORT(SocketNum),1));
				gu8Socket_Sever_Step[SocketNum] = Save_Port_Low_Byte;
			case Save_Port_Low_Byte:
				smpW5500Socket[SocketNum].Parm.DestPort +=(( uint16_t)Readbyte_value);
				gu8Socket_Sever_Step[SocketNum] = Read_Socket_INT_event;
				break;
			case Read_Recv_Cnt_HighByte:
				smp_w5500_spiDMA_read_byte(Sn_RX_RSR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Read_Recv_Cnt_LowByte;
				break;
			case Read_Recv_Cnt_LowByte:
				gu16Temp = (uint16_t)Readbyte_value;
				SocketHandleByteLen[SocketNum] = gu16Temp<<8;
				smp_w5500_spiDMA_read_byte(WIZCHIP_OFFSET_INC(Sn_RX_RSR(SocketNum),1));
				gu8Socket_Sever_Step[SocketNum] = Check_Recv_Data;			
			case Check_Recv_Data:
				ms_Count++;
				if(ms_Count == 2){
					SocketHandleByteLen[SocketNum] = SocketHandleByteLen[SocketNum] + ((uint16_t)Readbyte_value);
					ms_Count = 0;
					if(SocketHandleByteLen[SocketNum]> 0){
						gu8Socket_Sever_Step[SocketNum] = Read_Buf_Max_Len;
					}else{
						gu8Socket_Sever_Step[SocketNum] = Server_Read_Socket_Status;
						Socket_Event_Handling_Status[SocketNum] = Event_Handle_Done;
						//W5500INT_Low = false;
					}
				}
				break;
			case Read_Buf_Max_Len:
					smp_w5500_spiDMA_read_byte(Sn_RXBUF_SIZE(SocketNum));
					gu8Socket_Sever_Step[SocketNum] = Check_Oversize_Event;
				break;
			case Check_Oversize_Event:
				ms_Count++;
				if(ms_Count == 1){
					ms_Count = 0;
					lu16Temp = ((uint16_t)Readbyte_value)<< 10;
					if(SocketHandleByteLen[SocketNum] > lu16Temp){
						SocketHandleByteLen[SocketNum] = lu16Temp;
					}
				}
				gu8Socket_Sever_Step[SocketNum] = Read_Socket_Buf_Addr_HighByte;
				break;
			case Read_Socket_Buf_Addr_HighByte:
				smp_w5500_spiDMA_read_byte(Sn_RX_RD(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Read_Socket_Buf_Addr_LowByte;
				break;
			case Read_Socket_Buf_Addr_LowByte:
				gu16Temp = Readbyte_value;
				gu16Temp = gu16Temp<<8;
				smp_w5500_spiDMA_read_byte(WIZCHIP_OFFSET_INC(Sn_RX_RD(SocketNum),1));
				gu8Socket_Sever_Step[SocketNum] = Read_Socket_Buf_data;
				break;
			case Read_Socket_Buf_data:
				gu16Temp += Readbyte_value;
				lu32Temp = ((uint32_t)gu16Temp<<8)+ (WIZCHIP_RXBUF_BLOCK(SocketNum)<< 3); 
				smp_w5500_spiDMA_ReadMulti(lu32Temp,SocketHandleByteLen[SocketNum], TempSocket->Parm.Memory.rx_buf_Ptr);
				gu8Socket_Sever_Step[SocketNum] = ParserData_and_Update_Buf_Offset_Highbyte;
				break;
			case ParserData_and_Update_Buf_Offset_Highbyte:
				gu16Temp += SocketHandleByteLen[SocketNum];
				if(TempSocket->cbFunPtr){
					SocketHandleByteLen[SocketNum] = TempSocket->cbFunPtr(W5500_DATA_RECV,SocketHandleByteLen[SocketNum]);
				}
				smp_w5500_spiDMA_write_byte(Sn_RX_RD(SocketNum),(uint8_t)(gu16Temp>>8));
				gu8Socket_Sever_Step[SocketNum] = Update_Buf_Offset_Lowbyte;
				break;
			case Update_Buf_Offset_Lowbyte:
				smp_w5500_spiDMA_write_byte(WIZCHIP_OFFSET_INC(Sn_RX_RD(SocketNum),1),(uint8_t)gu16Temp);
				gu8Socket_Sever_Step[SocketNum] = Set_Scoket_Recv;
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
				lu16Temp = ((uint16_t)Readbyte_value)<<10;
				if(TempSocket->Parm.Memory.tx_buf_size > lu16Temp){
					TempSocket->Parm.Memory.tx_buf_size = lu16Temp;
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
				lu32Temp = (((uint32_t)gu16Temp) << 8) + (WIZCHIP_TXBUF_BLOCK(SocketNum)<< 3);
				smp_w5500_spiDMA_WriteMulti(lu32Temp, TempSocket->Parm.Memory.tx_buf_Ptr, SocketHandleByteLen[SocketNum]);
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
				smp_w5500_spiDMA_read_byte(Sn_IR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Verify_Send_End;
				break;
			case Verify_Send_End:
				ms_Count++;
				if(ms_Count == 2){
					ms_Count = 0;
					if((Readbyte_value&=SOCKET_NUM_BASE)&Sn_IR_RECV ){
						gu8Socket_Sever_Step[SocketNum] = Clear_SendOK_INT_Flag;
					}else{
						gu8Socket_Sever_Step[SocketNum] =  Read_Socket_INT_event_1;
					}
				}
				break;
			case Clear_SendOK_INT_Flag:
				smp_w5500_spiDMA_write_byte(Sn_IR(SocketNum), (Sn_IR_RECV &SOCKET_NUM_BASE)); 
				gu8Socket_Sever_Step[SocketNum] = Read_Socket_INT_event;
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
					gu8Socket_Sever_Step[SocketNum] = Read_Socket_INT_event_2;
				}
				break;	
			case Read_Socket_INT_event_2:
				smp_w5500_spiDMA_read_byte(Sn_IR(SocketNum));
				gu8Socket_Sever_Step[SocketNum] = Verify_Disconnct_End;
				break;
			case Verify_Disconnct_End:
				ms_Count++;
				if(ms_Count == 2){
					ms_Count = 0;
					if((Readbyte_value&=SOCKET_NUM_BASE)&Sn_IR_DISCON ){
						gu8Socket_Sever_Step[SocketNum] = Clear_DISCON_INT_Flag;
					}else{
						gu8Socket_Sever_Step[SocketNum] =  Read_Socket_INT_event_2;
					}
				}
				break;
			case Clear_DISCON_INT_Flag:
				smp_w5500_spiDMA_write_byte(Sn_IR(SocketNum), (Sn_IR_DISCON &SOCKET_NUM_BASE)); 
				gu8Socket_Sever_Step[SocketNum] = Server_Read_Socket_Status;
				break;
			/************ End ************/
			default:
				break;
		}
	}

	return smpResult;

}

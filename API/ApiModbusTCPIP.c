/**
******************************************************************************
* @file    ApiModbusTCPIP.c
* @author  Steve Cheng
* @version V0.0.3
* @date    2022/02/10
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
#include "ApiModbusTCPIP.h"


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



//========== Project protocol ==========
#define BMS_SLAVE_ID_OFFSET	0x80
#define MAX_SCU_CNT			0x20

#define MAX_SLAVE_ID		(BMS_SLAVE_ID_OFFSET+MAX_SCU_CNT)

#define SCU_INF_LAST_ADDR  0x0027
#define SCU_CELL_V_START_ADDR 0x0101
#define SCU_CELL_V_LAST_ADDR 0x0240
#define SCU_CELL_T_START_ADDR 0x0401
#define SCU_CELL_T_LAST_ADDR 0x0540
#define SCU_BMU_V_START_ADDR 0x0601
#define SCU_BMU_V_LAST_ADDR 0x061E
#define SCU_BMU_T_START_ADDR 0x0801
#define SCU_BMU_T_LAST_ADDR 0x081E

#define BMS_RACK_V_START_ADDR 0x0101
#define BMS_RACK_V_LAST_ADDR (BMS_RACK_V_START_ADDR+MAX_SCU_CNT)
#define BMS_RACK_T_START_ADDR 0x0201
#define BMS_RACK_T_LAST_ADDR (BMS_RACK_T_START_ADDR+MAX_SCU_CNT)

#define MAX_CELL_CNT 320
#define MAX_BMU_CNT 30	
#define SINGLE_SCU_CELL_SERIES_CNT 16

#define RELAY_CONTROL_ADDR 0x0003

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
    Gateway_Target_ResponseFail,
	
};

typedef enum{
	scu_data,
	bms_data
}modbus_tcp_data_source;

//========== Debug message function ==========
static char str[100];
void appSerialCanDavinciSendTextMessage(char *msg);
#define	modbus_tcp_debug(str)	//appSerialCanDavinciSendTextMessage(str)

//========== Get data through external function define ==========
static uint8_t data_index;
#define get_data(data_index)	0
#define get_scu_cellV_data(slave_id, cell_index) halAfeGetCellVoltage(cell_index)
#define get_scu_ntcV_data(slave_id, cell_index) HalAfeGetNtcVoltage(cell_index)
#define get_scu_bmu_totalV(slave_id,bmu_index)	0


//========== W5500 Socket Register parameter ==========
uint8_t ModbusTCPIP_buffer[DATA_BUF_MAX_SIZE];
uint8_t ResponseLength;
#define SOCKET_EMS {														\
					.Num						= NULL,						\
					.Protocol 					= Sn_MR_TCP,				\
					.PortNum  					= MODBUS_TCPIP_SOCKET_PORT,	\
					.Memory.rx_buf_Ptr          = ModbusTCPIP_buffer,		\
					.Memory.rx_buf_size         = DATA_BUF_MAX_SIZE,       	\
					.Memory.tx_buf_Ptr          = ModbusTCPIP_buffer,		\
					.Memory.tx_buf_size         = DATA_BUF_MAX_SIZE 		\
					}
W5500_Socket_parm Socket_ModbusTCPIP = SOCKET_EMS;	

static uint16_t reg_addr=0;				
static uint16_t reg_cnt = 0;
static uint8_t slave_id = 0;
static uint8_t* Ptr_response;
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
			return  Modbus_Parser(Socket_ModbusTCPIP,DataLen);
			break;
		case W5500_Socket_REG_Success:
			break;
		case W5500_COMMUNICATE_ERR:
			break;
		case W5500_Socket_Connect:
			break;
	}
	return 0;
}					



void Modbus_TCPIP_Socket_Open(void){
	/* Register socket and caall back function */
	W5500_Socket_Register(&Socket_ModbusTCPIP, ModbusTCPIP_cb);   
}



static uint8_t (*modbus_data_handle)(uint8_t* ptr);			
typedef uint8_t (*modbus_tcp_get_data_table)(void);
//----------Get SCU DATA ----------
static uint16_t scu_get_data_index = 0;

static uint8_t unsupport_reg_addr(void){
	return Illegal_DataAddr;
}

static uint8_t space_function(void){
	return Normal;
}

static void increase_get_data_index(uint16_t wordlen){
	scu_get_data_index += wordlen;
	
	sprintf(str,"SCU_GET_INDEX : %x",scu_get_data_index);
	modbus_tcp_debug(str);
	
	*(Socket_ModbusTCPIP.Memory.tx_buf_Ptr + READ_HOLDING_REG_RES_BYTECNT_OFFSET) += (uint8_t)(wordlen*2); 
	
	Ptr_response+=(wordlen*2);
	ResponseLength += wordlen*2;
	reg_cnt--;
}

static uint8_t scu_get_protect_flag_1(void){
	/*Get Data*/
	modbus_tcp_debug("Protect flag_1");
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_protect_flag_2(void){
	/*Get Data*/
	modbus_tcp_debug("Protect flag_2");
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_protect_flag_3(void){
	/*Get Data*/
	modbus_tcp_debug("Protect flag_3");
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_status(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_module_parm(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_max_cell_voltage(void){
	/*Get Data*/
	*Ptr_response = get_data(data_index);
	*(Ptr_response+1) = get_data(data_index);
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_min_cell_voltage(void){
	/*Get Data*/
	*Ptr_response = get_data(data_index);
	*(Ptr_response+1) = get_data(data_index);
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_max_cell_deltav(void){
	/*Get Data*/
	*Ptr_response = get_data(data_index);
	*(Ptr_response+1) = get_data(data_index);
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_max_bmu_voltage(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_min_bmu_voltage(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_max_bmu_deltav(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_max_cell_v_index(void){
	/*Get Data*/
	*Ptr_response = get_data(data_index);
	*(Ptr_response+1) = get_data(data_index);
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_min_cell_v_index(void){
	/*Get Data*/
	*Ptr_response = get_data(data_index);
	*(Ptr_response+1) = get_data(data_index);
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_extreme_bmu_v_index(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_max_cell_temp(void){
	uint16_t temp;
	
	temp = get_data(data_index);
	temp = LibNtcVoltageToTemperature(temp);
	/*Get Data*/
	*Ptr_response = (uint8_t)(temp>>8);
	*(Ptr_response+1) = (uint8_t)temp;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_min_cell_temp(void){
	uint16_t temp;
	
	temp = get_data(data_index);
	temp = LibNtcVoltageToTemperature(temp);
	/*Get Data*/
	*Ptr_response = (uint8_t)(temp>>8);
	*(Ptr_response+1) = (uint8_t)temp;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_max_cell_deltat(void){
	uint16_t temp;
	
	temp = get_data(data_index);
	temp = LibNtcVoltageToTemperature(temp);
	/*Get Data*/
	*Ptr_response = (uint8_t)(temp>>8);
	*(Ptr_response+1) = (uint8_t)temp;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_max_bmu_temp(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_min_bmu_temp(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_max_bmu_deltat(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_max_cell_t_index(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_min_cell_t_index(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_extreme_bmu_t_index(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_soc(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}
static uint8_t scu_get_soh(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_get_data_index(1);
	return Normal;
}

static uint8_t scu_get_current(void){
	sprintf(str, "Get Current %x ",get_data(data_index));
	modbus_tcp_debug(str);
	/*Get Data*/
	*Ptr_response = get_data(data_index);
	*(Ptr_response+1) = get_data(data_index);
	*(Ptr_response+2) = get_data(data_index);
	*(Ptr_response+3) = get_data(data_index);
	increase_get_data_index(2);
	return Normal;
}
static uint8_t scu_get_power(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	*(Ptr_response+2) = 0;
	*(Ptr_response+3) = 0;
	increase_get_data_index(2);
	return Normal;
}
static uint8_t scu_get_sug_chgpower(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	*(Ptr_response+2) = 0;
	*(Ptr_response+3) = 0;
	increase_get_data_index(2);
	return Normal;
}
static uint8_t scu_get_sug_dhgpower(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	*(Ptr_response+2) = 0;
	*(Ptr_response+3) = 0;
	increase_get_data_index(2);
	return Normal;
}
static uint8_t scu_get_total_chgpower(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	*(Ptr_response+2) = 0;
	*(Ptr_response+3) = 0;
	increase_get_data_index(2);
	return Normal;
}
static uint8_t scu_get_total_dhgpower(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	*(Ptr_response+2) = 0;
	*(Ptr_response+3) = 0;	
	increase_get_data_index(2);
	return Normal;
}
static uint8_t scu_get_fcc(void){
	modbus_tcp_debug("FCC");
	/*Get Data*/
	*Ptr_response = get_data(data_index);
	*(Ptr_response+1) = get_data(data_index);
	*(Ptr_response+2) = get_data(data_index);
	*(Ptr_response+3) = get_data(data_index);
	increase_get_data_index(2);
	return Normal;
}
static uint8_t scu_get_rc(void){
	modbus_tcp_debug("RC");
	/*Get Data*/
	*Ptr_response =  get_data(data_index);
	*(Ptr_response+1) = get_data(data_index);
	*(Ptr_response+2) = get_data(data_index);
	*(Ptr_response+3) = get_data(data_index);
	increase_get_data_index(2);
	return Normal;
}

const modbus_tcp_get_data_table scu_get_data_function[]={
	scu_get_protect_flag_1,				// 0x00
	scu_get_protect_flag_2,				// 0x01
	scu_get_protect_flag_3,				// 0x02
	scu_get_status,						// 0x03
	scu_get_module_parm,				// 0x04
	scu_get_max_cell_voltage,			// 0x05		
	scu_get_min_cell_voltage,			// 0x06
	scu_get_max_cell_deltav,			// 0x07
	scu_get_max_bmu_voltage,			// 0x08
	scu_get_min_bmu_voltage,			// 0x09
	scu_get_max_bmu_deltav,				// 0x0A
	scu_get_max_cell_v_index,			// 0x0B
	scu_get_min_cell_v_index,			// 0x0C
	scu_get_extreme_bmu_v_index,		// 0x0D
	scu_get_max_cell_temp,				// 0x0E
	scu_get_min_cell_temp,				// 0x0F
	scu_get_max_cell_deltat,			// 0x10
	scu_get_max_bmu_temp,				// 0x11
	scu_get_min_bmu_temp,				// 0x12
	scu_get_max_bmu_deltat,				// 0x13
	scu_get_max_cell_t_index,			// 0x14
	scu_get_min_cell_t_index,			// 0x15
	scu_get_extreme_bmu_t_index,		// 0X16
	scu_get_soc,						// 0X17
	scu_get_soh,						// 0X18
	scu_get_current,					// 0X19
	space_function,						
	scu_get_power,						// 0x1B
	space_function,						
	scu_get_sug_chgpower,				// 0X1D
	space_function,						
	scu_get_sug_dhgpower,				// 0X1F
	space_function,						
	scu_get_total_chgpower,				// 0X21
	space_function,						
	scu_get_total_dhgpower,				// 0X23
	space_function,						
	scu_get_fcc,						// 0X25
	space_function,						
	scu_get_rc,							// 0X27
	space_function,
	unsupport_reg_addr,
};
static uint8_t get_scu_data(uint8_t* response_first_ptr){
	static uint16_t i,cell_index,bmu_index,cell_voltage,cell_ntc_vol,temp;
	
	Ptr_response = response_first_ptr;
	ResponseLength+=READ_HOLDING_REG_RES_HEADER_LEN;
	
	if((scu_get_data_index+reg_cnt)<=0x21){
		for(i=reg_cnt;i>0;i--){
			if(scu_get_data_function[scu_get_data_index]()== Illegal_DataAddr)
				return Illegal_DataAddr;
		}
	}else if((scu_get_data_index>=SCU_CELL_V_START_ADDR)&&(scu_get_data_index+reg_cnt)<=(SCU_CELL_V_LAST_ADDR+1)){
		cell_index = scu_get_data_index-SCU_CELL_V_START_ADDR;
		*(Socket_ModbusTCPIP.Memory.tx_buf_Ptr + READ_HOLDING_REG_RES_BYTECNT_OFFSET) = (uint8_t)(reg_cnt*2);
		
		for(i=reg_cnt; i>0; i--){
			cell_voltage = halAfeGetCellVoltage(cell_index);
			sprintf(str, "Cell %x V %d",cell_index,cell_voltage);
			modbus_tcp_debug(str);
			*Ptr_response = (uint8_t)(cell_voltage>>8);
			*(Ptr_response+1) = (uint8_t)cell_voltage;
			cell_index++;
			Ptr_response+=2;
			ResponseLength+=2;
		}
		
	}else if((scu_get_data_index>=SCU_CELL_T_START_ADDR)&&(scu_get_data_index+reg_cnt)<=(SCU_CELL_T_LAST_ADDR+1)){
		cell_index = scu_get_data_index-SCU_CELL_T_START_ADDR;
		*(Socket_ModbusTCPIP.Memory.tx_buf_Ptr + READ_HOLDING_REG_RES_BYTECNT_OFFSET) = (uint8_t)(reg_cnt*2);
		
		for(i=reg_cnt;i>0;i--){
			cell_ntc_vol = HalAfeGetNtcVoltage(cell_index);
			temp = LibNtcVoltageToTemperature(cell_ntc_vol);
			
			if(temp/1000>5)
				temp+=10;
			temp/=10;
			sprintf(str, "Cell %x T %d",cell_index,temp);
			modbus_tcp_debug(str);
			
			*Ptr_response = (uint8_t)(temp>>8);
			*(Ptr_response+1) = (uint8_t)temp;
			
			cell_index++;
			Ptr_response+=2;
			ResponseLength+=2;
		}
	}else if((scu_get_data_index>=SCU_BMU_V_START_ADDR)&&(scu_get_data_index+reg_cnt)<=(SCU_BMU_V_LAST_ADDR+1)){
		bmu_index = scu_get_data_index-SCU_BMU_V_START_ADDR;
		*(Socket_ModbusTCPIP.Memory.tx_buf_Ptr + READ_HOLDING_REG_RES_BYTECNT_OFFSET) = (uint8_t)(reg_cnt*2);
		
		for(i=reg_cnt;i>0;i--){
			sprintf(str, "BMU_INDEX: %x V:0 ",bmu_index);
			modbus_tcp_debug(str);
			
			*Ptr_response = 0;
			*(Ptr_response+1) = 0;
			bmu_index++;
			Ptr_response+=2;
			ResponseLength+=2;
		}	
	}else if((scu_get_data_index>=SCU_BMU_T_START_ADDR)&&(scu_get_data_index+reg_cnt)<=(SCU_BMU_T_LAST_ADDR+1)){
		bmu_index = scu_get_data_index-SCU_BMU_T_START_ADDR;
		*(Socket_ModbusTCPIP.Memory.tx_buf_Ptr + READ_HOLDING_REG_RES_BYTECNT_OFFSET) = (uint8_t)(reg_cnt*2);
		
		for(i=reg_cnt;i>0;i--){
			sprintf(str, "BMU_INDEX: %x V:0 ",bmu_index);
			modbus_tcp_debug(str);
			
			*Ptr_response = 0;
			*(Ptr_response+1) = 0;
			bmu_index++;
			Ptr_response+=2;
			ResponseLength+=2;
		}	
	}else{
		return Illegal_DataAddr;
	}
	
	return Normal;
}

//----------Get BMS DATA ----------
static uint16_t bms_get_data_index = 0;
static void increase_bms_get_data_index(uint16_t wordlen){
	bms_get_data_index += wordlen;
	
	sprintf(str,"BMS_GET_INDEX : %x",bms_get_data_index);
	modbus_tcp_debug(str);
	
	*(Socket_ModbusTCPIP.Memory.tx_buf_Ptr + READ_HOLDING_REG_RES_BYTECNT_OFFSET) += (uint8_t)(wordlen*2); 
	Ptr_response+=(wordlen*2);
	ResponseLength += wordlen*2;
	reg_cnt--;
}

static uint8_t bms_get_protect_flag_1(void){
	/*Get Data*/
	modbus_tcp_debug("BMS_Protect flag_1");
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_bms_get_data_index(1);
	return Normal;
}

static uint8_t bms_get_protect_flag_2(void){
	/*Get Data*/
	modbus_tcp_debug("BMS_Protect flag_2");
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_bms_get_data_index(1);
	return Normal;
}

static uint8_t bms_get_protect_flag_3(void){
	/*Get Data*/
	modbus_tcp_debug("BMS_Protect flag_3");
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_bms_get_data_index(1);
	return Normal;
}

static uint8_t bms_get_warning_position(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	*(Ptr_response+2) = 0;
	*(Ptr_response+3) = 0;
	increase_bms_get_data_index(2);
	return Normal;
}

static uint8_t bms_get_protect_position(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	*(Ptr_response+2) = 0;
	*(Ptr_response+3) = 0;
	increase_bms_get_data_index(2);
	return Normal;
}

static uint8_t bms_get_total_current(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	*(Ptr_response+2) = 0;
	*(Ptr_response+3) = 0;
	increase_bms_get_data_index(2);
	return Normal;
}

static uint8_t bms_get_total_power(void){
	/*Get Data*/
	*Ptr_response = 0; 
	*(Ptr_response+1) = 0;
	*(Ptr_response+2) = 0;
	*(Ptr_response+3) = 0;
	increase_bms_get_data_index(2);
	return Normal;
}

static uint8_t bms_get_recommand_chgpower(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	*(Ptr_response+2) = 0;
	*(Ptr_response+3) = 0;
	increase_bms_get_data_index(2);
	return Normal;
}

static uint8_t bms_get_recommand_dhgpower(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	*(Ptr_response+2) = 0;
	*(Ptr_response+3) = 0;
	increase_bms_get_data_index(2);
	return Normal;
}

static uint8_t bms_get_acc_chg_power(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	*(Ptr_response+2) = 0;
	*(Ptr_response+3) = 0;
	increase_bms_get_data_index(2);
	return Normal;
}

static uint8_t bms_get_acc_dhg_power(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	*(Ptr_response+2) = 0;
	*(Ptr_response+3) = 0;
	increase_bms_get_data_index(2);
	return Normal;
}

static uint8_t bms_get_total_SOC(void){
	
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_bms_get_data_index(1);
	return Normal;
}

static uint8_t bms_get_total_SOH(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_bms_get_data_index(1);
	return Normal;
}

static uint8_t bms_get_maxV_call_position(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_bms_get_data_index(1);
	return Normal;
}

static uint8_t bms_get_maxV_bmu_position(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_bms_get_data_index(1);
	return Normal;
}

static uint8_t bms_get_maxV_rack_position(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_bms_get_data_index(1);
	return Normal;
}

static uint8_t bms_get_minV_call_position(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_bms_get_data_index(1);
	return Normal;
}

static uint8_t bms_get_minV_bmu_position(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_bms_get_data_index(1);
	return Normal;
}

static uint8_t bms_get_minV_rack_position(void){
	/*Get Data*/
	*Ptr_response = 0;
	*(Ptr_response+1) = 0;
	increase_bms_get_data_index(1);
	return Normal;
}

const modbus_tcp_get_data_table bms_get_data_function[]={
	bms_get_protect_flag_1,		//0x00
	bms_get_protect_flag_2,		//0x01
	bms_get_protect_flag_3,		//0x02
	bms_get_warning_position,	//0x03
	space_function,		
	bms_get_protect_position,	//0x05
	space_function,		
	bms_get_total_current,		//0x07
	space_function,		
	bms_get_total_power,		//0x09
	space_function,	
	bms_get_recommand_chgpower,	//0x0B
	space_function,	
	bms_get_recommand_dhgpower,	//0x0D
	space_function,	
	bms_get_acc_chg_power,		//0x0F
	space_function,	
	bms_get_acc_dhg_power,		//0x11
	space_function,	
	bms_get_total_SOC,			//0x13
	bms_get_total_SOH,			//0x14
	bms_get_maxV_call_position,	//0x15
	bms_get_maxV_bmu_position,	//0x16
	bms_get_maxV_rack_position,	//0x17
	bms_get_minV_call_position,	//0x18
	bms_get_minV_bmu_position,	//0x19
	bms_get_minV_rack_position,	//0x1A
	
	unsupport_reg_addr,	
};


static uint8_t get_bms_data(uint8_t* response_first_ptr){
	static uint16_t i,scu_index;
	
	Ptr_response = response_first_ptr;
	ResponseLength+=READ_HOLDING_REG_RES_HEADER_LEN;
	
	sprintf(str,"StartAddr %4x REG_CNT %2x",bms_get_data_index,reg_cnt);
	modbus_tcp_debug(str);
	
	if((bms_get_data_index+reg_cnt)<=0x1A){
		for(i=reg_cnt;i>0;i--){
			if(bms_get_data_function[bms_get_data_index]() == Illegal_DataAddr)
				
				return Illegal_DataAddr;
		}
	}else if((bms_get_data_index>=BMS_RACK_V_START_ADDR)&&(bms_get_data_index+reg_cnt)<=(BMS_RACK_V_LAST_ADDR)){
		scu_index = bms_get_data_index-BMS_RACK_V_START_ADDR;
		*(Socket_ModbusTCPIP.Memory.tx_buf_Ptr + READ_HOLDING_REG_RES_BYTECNT_OFFSET) = (uint8_t)(reg_cnt*2);
		
		for(i=reg_cnt;i>0;i--){
			
			*Ptr_response = 0;
			*(Ptr_response+1) = 0;
			scu_index++;
			Ptr_response+=2;
			ResponseLength+=2;
		}	
	}else if((bms_get_data_index>=BMS_RACK_T_START_ADDR)&&(bms_get_data_index+reg_cnt)<=(BMS_RACK_T_LAST_ADDR)){
		scu_index = bms_get_data_index-BMS_RACK_T_START_ADDR;
		*(Socket_ModbusTCPIP.Memory.tx_buf_Ptr + READ_HOLDING_REG_RES_BYTECNT_OFFSET) = (uint8_t)(reg_cnt*2);
		
		for(i=reg_cnt;i>0;i--){
			
			*Ptr_response = 0;
			*(Ptr_response+1) = 0;
			scu_index++;
			Ptr_response+=2;
			ResponseLength+=2;
		}	
	}else{
		return Illegal_DataAddr;
	}
	return Normal;
}

static uint8_t write_reg_function(uint8_t* response_first_ptr){
	if(scu_get_data_index != 0x0003){
		return Illegal_DataAddr;
	}else{
		modbus_tcp_debug("Recv Relay Control Command");
		ResponseLength = 4;
	}
	return Normal;
}

//==========  MODBUS Parser Supprot RTU ==========//

static void (*modbus_parser)(W5500_Socket_parm Socket, uint16_t DataLen);

static void modbus_tcpip_parser(W5500_Socket_parm Socket, uint16_t DataLen){
	static uint8_t *TempDataPtr;
	static uint8_t i,lu8ExceptionCode = Normal;
	
	ResponseLength = 0;
	TempDataPtr = Socket.Memory.rx_buf_Ptr;
	
	//========== Get Slave ID ==========
	slave_id = *(TempDataPtr+6);
	
	//========== Get REG count ==========
	reg_cnt = ByteToWord(*(TempDataPtr+10),*(TempDataPtr+11));
	
	//========== Verify protocol ID and Message length ==========
	if((ByteToWord(*(TempDataPtr+2),*(TempDataPtr+3))!= MODBUS_TCPIP_PROTOCOLID)||
		(ByteToWord(*(TempDataPtr+4),*(TempDataPtr+5))!=(DataLen-MODBUS_TCPIP_HEADER_LEN))){
		modbus_tcp_debug("Protocol err or length err");
		lu8ExceptionCode = Gateway_Target_ResponseFail;
			
	
	}else if(reg_cnt > MODBUS_TCPIP_READ_WRITE_MAXWORDCNT){
		lu8ExceptionCode = Memory_Parity_Error;	
	// ========== Check slave address ==========
	//			1.						2.										3.
	}else if( (slave_id == 0)||(slave_id > MAX_SLAVE_ID )||((slave_id>MAX_SCU_CNT)&&(slave_id<=BMS_SLAVE_ID_OFFSET))){
		modbus_tcp_debug("Ask illeagle slve address");	
		lu8ExceptionCode = Memory_Parity_Error;
	}else{
		//========== Check function code ==========	
		if(*(TempDataPtr+7) ==  PRESEET_MULTIPLE_REG){
			modbus_data_handle = write_reg_function;
		}else if (*(TempDataPtr+7) == READ_HOLDING_REG){
			if(slave_id<= MAX_SCU_CNT){
				//========== Get Start REG Addr ==========
				scu_get_data_index = ByteToWord(*(TempDataPtr+8),*(TempDataPtr+9));
				modbus_data_handle = get_scu_data;
			}else if(slave_id >  BMS_SLAVE_ID_OFFSET){
				//========== Get Start REG Addr ==========
				bms_get_data_index = ByteToWord(*(TempDataPtr+8),*(TempDataPtr+9));
				modbus_data_handle = get_bms_data;
			}
		}else{
			lu8ExceptionCode = Illegal_Function;
		}
	}
	
	//========== Get Data ==========
	*(Socket_ModbusTCPIP.Memory.tx_buf_Ptr + READ_HOLDING_REG_RES_BYTECNT_OFFSET) = 0;
	lu8ExceptionCode = modbus_data_handle(Socket.Memory.tx_buf_Ptr+9);
	
	//========== Fill Response frame ==========
	if(lu8ExceptionCode != Normal){
		
		sprintf(str,"Exception code: %x",lu8ExceptionCode);
		modbus_tcp_debug(str);	
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
		
	/* Handle normal response data */
	}else{
		
		/* Set massage length of response */
		TempDataPtr = Socket.Memory.tx_buf_Ptr + MODBUS_TCPIP_MASSAGELEN_OFFSET;
		*TempDataPtr = WordToByte(ResponseLength, HIGH_Byte);
		*(TempDataPtr+1) = WordToByte(ResponseLength, LOW_Byte);
		
		/* Update total length of response frame */
		ResponseLength+= MODBUS_TCPIP_HEADER_LEN;
		
		//========== Print response frame ========== 
		TempDataPtr = Socket.Memory.tx_buf_Ptr;
		for(i=0;i<ResponseLength;i++){
			sprintf(str," %x",*(TempDataPtr+i));
			modbus_tcp_debug(str);	
		}
	}
}

static void modbus_rtu_parser(W5500_Socket_parm Socket, uint16_t DataLen){
	ResponseLength = 0;
}

uint16_t Modbus_Parser(W5500_Socket_parm Socket, uint16_t DataLen){
	uint8_t *TempDataPtr;
	static uint8_t lu8ExceptionCode = Normal,i;
	ResponseLength = 0;
	
	
	/* Debug print receive data */
	TempDataPtr = Socket.Memory.rx_buf_Ptr;
	for(i=0;i<DataLen;i++){
		sprintf(str," %x", Socket.Memory.rx_buf_Ptr[i]);
		modbus_tcp_debug(str);	
	}
	switch(Socket.Protocol){
		/* Use MODBUS TCP Protocol */
		case Sn_MR_TCP:
			modbus_parser = modbus_tcpip_parser;
			break;
		/* Use MODBUS RTU Protocol */
		default:
			modbus_parser = modbus_rtu_parser;
			break;		
	}
	modbus_parser(Socket,DataLen);
	return ResponseLength;
}


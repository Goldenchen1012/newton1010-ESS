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

/* Private typedef -----------------------------------------------------------*/
#define	UART_TX_BUF_SIZE	2000
#define	UART_RX_BUF_SIZE	500
typedef struct{
	uint8_t		Status;
	uint16_t	Index;
	uint8_t		Checksum;
	uint8_t		Buffer[300];
}tHvEssUartDecode;

uint8_t  Davinci_uart_tx_buffer[UART_TX_BUF_SIZE] = {0};
uint8_t  Davinci_uart_rx_buffer[UART_RX_BUF_SIZE] = {0};
uint16_t Davinci_uart_rx_cnt =0;

#define DAVINCI_UART        {                                                                                \
                                .num                        = UART1,                                         \
                                .baud_rate                  = 115200,                                        \
                                .flow_ctrl                  = UART_FLOW_CTRL_DISABLE,                        \
                                .use_parity                 = PARITY_NONE,                                   \
                                .buffers.rx_buf             = Davinci_uart_rx_buffer,                        \
                                .buffers.rx_buf_size        = UART_TX_BUF_SIZE,                              \
                                .buffers.tx_buf             = Davinci_uart_tx_buffer,                        \
                                .buffers.tx_buf_size        = UART_RX_BUF_SIZE                               \
                          }  


smp_uart_t mDavinci_uart = DAVINCI_UART;

	
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint16_t	SubIndex = 0;

void (*ReportScuUartMessage)(void) = {0};
tHvEssUartDecode	HvEssUartDecodeBuf={0};

/* Private function prototypes -----------------------------------------------*/
//--Send: 5A 05 55 70 70 50 69 

void DavinciUart_cb(uart_evt_type p_evt){
  switch(p_evt){
    case UART_DATA_READY:
      /* received data handle */
     Davinci_uart_rx_cnt++;
    break;
    case UART_TX_EMPTY:
      /* Data transmission complete handle */
    break;
    case UART_COMMUNICATION_ERR:
      /* occurred during reception */
    break;
    case UART_BUFFER_FULL:
      /* occurred UART buffer full */
    break;    
    default:
    break;
  }
}

static void calUartChecksumAndPutToFifo(uint8_t *buf)
{
	uint8_t	i,n,checksum=0;
	
	buf[0] = 0x5A;	
	for(n=1; n<buf[1]; n++)
		checksum ^= buf[n];
	buf[n++] = checksum;
	buf[n++] = 0x69;
	
	for(i=0; i<n; i++)
		smp_uart_put(&mDavinci_uart, buf[i]);
}

static void appSerialUartSendBaseInfo(void)
{
	tIbyte	Ibyte;
	tLbyte	Lbyte;
	BYTE	i;//,ch;
	BYTE	len;//,checksum;
	uint8_t	buf[100];
	uint8_t	index;
	
//	if(!SystemReadyFlag)
//		return;
	//-----------------------------
	//	soc
	buf[1] = 7;
	buf[2] = EBIKE_UART_PROTOCOL;
	buf[3] = SUB_CMD_RETURN_INFORMATION;
	buf[4] = B0_INFO_SOC;
	Ibyte.i = appGaugeGetRSoc();
	buf[5] = Ibyte.b[0];
	buf[6] = Ibyte.b[1];
	calUartChecksumAndPutToFifo(buf);
	//--------------------------------------
	//	base information

	len = 5 + 29;
	buf[1] = len;
	buf[2] = EBIKE_UART_PROTOCOL;
	buf[3] = SUB_CMD_RETURN_INFORMATION;
	buf[4] = B0_INFO_OTHER_MESSAGE;
	index = 5;
	//-------------------------
	//B1-B2:Dod0	2
	Ibyte.i = (10000 - appGaugeGetSoc0());
	buf[index++] = Ibyte.b[0];
	buf[index++] = Ibyte.b[1];
	//--------------------------
	//B3: Soh		1
	buf[index++] = appGaugeGetSOH()/100;
	//-------------------------
	//B4-B7:RM		4
	Lbyte.l = appGaugeGetRM();
	for(i=0;i<4;i++)
	{
		buf[index++] = Lbyte.b[i];
	}
	//---------------------------
	//B8-B11:Rp	4
	Lbyte.sl = appGaugeGetRPassCharge();
	for(i=0;i<4;i++)
	{
		buf[index++] = Lbyte.b[i];
	}
	//---------------------------
	//B12-B15:Qp	4
	Lbyte.sl = appGaugeGetQPassCharge();
	for(i=0;i<4;i++)
	{
		buf[index++] = Lbyte.b[i];
	}
	//---------------------------
	//B16-B19:QStart	4
	Lbyte.l = appGaugeGetQStart();
	for(i=0;i<4;i++)
	{
		buf[index++] = Lbyte.b[i];
	}
	//---------------------------
	//B20-B23:FCC	4
	Lbyte.l = appGaugeGetFCC();

	for(i=0;i<4;i++)
	{
		buf[index++] = Lbyte.b[i];
	}
	//---------------------------
	//B24-B27:Qmax	4
	Lbyte.l = appGaugeGetQmax();
	for(i=0;i<4;i++)
	{
		buf[index++] = Lbyte.b[i];
	}
	//---------------------------
	//B28-B29:Cycle count	2
	Ibyte.i = 25;//10000-BatteryCapInfo.Soc0;
	buf[index++] = Ibyte.b[0];
	buf[index++] = Ibyte.b[1];
	
	calUartChecksumAndPutToFifo(buf);
	
	ReportScuUartMessage = 0;
}


static void appSerialUartSendCotpFlag(void)
{
	BYTE	len;
	BYTE	i;
	uint8_t	buf[100];
	uint8_t	index;
	
//	if(!SystemReadyFlag)
//		return;
	
	len = 5 + 4 * 5;
	buf[1] = len;
	buf[2] = EBIKE_UART_PROTOCOL;
	buf[3] = SUB_CMD_RETURN_INFORMATION;
	buf[4] = B0_INFO_COTP_CUTP_DOTP;	//'n'
	index = 5;	
	for(i=0; i<5; i++)
	{
		buf[index++] = apiProtectCotpGetFlag(i);
		buf[index++] = apiProtectCutpGetFlag(i);
		buf[index++] = apiProtectDotpGetFlag(i);
		buf[index++] = apiProtectDutpGetFlag(i);
	}
	calUartChecksumAndPutToFifo(buf);
	
	ReportScuUartMessage = appSerialUartSendBaseInfo;
}

static void appSerialUartSendCurrent(void)
{
	BYTE	i;
	tLbyte	Lbyte;
	uint8_t	buf[100];
	uint8_t	index;
	
	buf[1] = 9;
	buf[2] = EBIKE_UART_PROTOCOL;
	buf[3] = SUB_CMD_RETURN_INFORMATION;
	buf[4] = B0_INFO_CURRENT;
	index = 5;
	Lbyte.sl = appGaugeGetCurrentValue(P_CURRENT);
	for(i=0; i<4; i++)	
	{
		buf[index++] = Lbyte.b[i];
	}
	calUartChecksumAndPutToFifo(buf);
	//------------------------------------------
	buf[1] = 9;
	buf[2] = EBIKE_UART_PROTOCOL;
	buf[3] = SUB_CMD_RETURN_INFORMATION;
	buf[4] = B0_INFO_REAL_CURRENT;
	index = 5;
	Lbyte.sl = halAfeGetCurrentValue(0);
	for(i=0; i<4; i++)	
	{
		buf[index++] = Lbyte.b[i];
	}
	calUartChecksumAndPutToFifo(buf);
	
	ReportScuUartMessage = appSerialUartSendCotpFlag;
}

static void appSerialUartSendOvpUvpFlag(void)
{
	BYTE	cell;
	BYTE	len;
	uint8_t	buf[100];
	uint8_t	index;
//	if(!SystemReadyFlag)
//		return;
	
	len = 5 + 16 * 2;
//	bufKgEbikeUartMsgOut(0x5A);
//	CalCheckSum = 0;
	buf[1] = len ;
	buf[2] = EBIKE_UART_PROTOCOL;
	buf[3] = SUB_CMD_RETURN_INFORMATION;
	buf[4] = B0_INFO_OVP_UVP;
	index = 5;
	for(cell=0; cell<16; cell++)
	{
		buf[index++] = apiProtectOvpGetFlag(cell);
		buf[index++] = apiProtectUvpGetFlag(cell);
	}
	calUartChecksumAndPutToFifo(buf);
	
	ReportScuUartMessage = appSerialUartSendCurrent;
}


static void appSerialUartSendNtcTemp(void)
{
	uint8_t	buf[100];
	tIbyte	Ibyte;
	
	Ibyte.i = HalAfeGetNtcAdc(SubIndex);
	buf[1] = 8;
	buf[2] = 'U';
	buf[3] = 'I';
	buf[4] = 'N';
	buf[5] = SubIndex + 1;
	buf[6] = Ibyte.b[0];
	buf[7] = Ibyte.b[1];
	calUartChecksumAndPutToFifo(buf);
	SubIndex++;
	if(SubIndex >= 16)
	{
		ReportScuUartMessage = appSerialUartSendOvpUvpFlag;	
	}	
}

static void appSerialUartSendCellVoltage(void)
{
	uint8_t	buf[100];
	tIbyte	Ibyte;
	
	Ibyte.i = halAfeGetCellVoltage(SubIndex);
	buf[1] = 8;
	buf[2] = 'U';
	buf[3] = 'I';
	buf[4] = 'C';
	buf[5] = SubIndex + 1;
	buf[6] = Ibyte.b[0];
	buf[7] = Ibyte.b[1];
	calUartChecksumAndPutToFifo(buf);
	SubIndex++;
	if(SubIndex >= 16)
	{
		SubIndex = 0;
		ReportScuUartMessage = appSerialUartSendNtcTemp;
	}
}


void appSerialUartSendMessage(uint8_t *str)
{
	BYTE	buffer[256];
	BYTE	i,len,checksum,index;
	
	len = strlen((char *)str);
	buffer[0] = 0x5A;
	buffer[1] = len+4;
	buffer[2] = 'T';
	checksum = 'T';
	checksum ^= buffer[1];
	index = 3;
	for(i=0;i<len;i++)
	{
		buffer[index++]=str[i];
		checksum^=str[i];
	}
	buffer[index++]=0;
	buffer[index++]=checksum;
	buffer[index++]=0x69;
	
	for(i=0; i<index; i++)
		smp_uart_put(&mDavinci_uart, buffer[i]);
}

static void appSerialUartPackageParser(void)
{
	uint8_t		cell,ntc;
	uint8_t 	index;
	tIbyte		CellVoltage;
	static tLbyte		cur;
	static int32_t		i32;
	
	if(HvEssUartDecodeBuf.Buffer[1] != 'U')
		return;
	switch(HvEssUartDecodeBuf.Buffer[2])
	{
	case 'S'://cimu
		switch(HvEssUartDecodeBuf.Buffer[3])
		{
		case 'C':	//cell voltage
			cell = 0;
			for(index = 4; index<(HvEssUartDecodeBuf.Buffer[0]-1);)
			{
				CellVoltage.b[0] = HvEssUartDecodeBuf.Buffer[index++];
				CellVoltage.b[1] = HvEssUartDecodeBuf.Buffer[index++];
				halAfeSetCellVoltage(cell++, CellVoltage.i);				
			}
			break;
		case 'N':	//ntc
			ntc = 0;
			for(index = 4; index<(HvEssUartDecodeBuf.Buffer[0]-1);)
			{
				CellVoltage.b[0] = HvEssUartDecodeBuf.Buffer[index++];
				CellVoltage.b[1] = HvEssUartDecodeBuf.Buffer[index++];
				
				CellVoltage.i /= 100;
				CellVoltage.i = LibTemperatureToVoltage(CellVoltage.i);
				
				halAfeSetNtcAdcData(ntc++, CellVoltage.i);				
			}
			break;
		case 'I':	//current
			index = 4;
			cur.b[0] = HvEssUartDecodeBuf.Buffer[index++];
			cur.b[1] = HvEssUartDecodeBuf.Buffer[index++];
			cur.b[2] = HvEssUartDecodeBuf.Buffer[index++];
			cur.b[3] = HvEssUartDecodeBuf.Buffer[index++];
			
			i32 = (LONG)(cur.l&0xfffffffL);
			if(cur.b[3] & 0x80L)		//charge 
			{
				i32 *= -1;
			}
			HalAfeSetCurrentValue(0, i32);

			break;
		}
		break;		
	}
}

static void appSerialUartPackageJudgee(void)
{
	uint8_t	i,ch;
	
	for(i=0; i<200; i++)
	{
		 if(smp_uart_get(&mDavinci_uart, &ch) != SMP_SUCCESS)
		 	break;
		if(HvEssUartDecodeBuf.Status == 0)
		{
			if(ch == 0x5A)
			{
				HvEssUartDecodeBuf.Status = 1;
				HvEssUartDecodeBuf.Index = 0;
				HvEssUartDecodeBuf.Checksum = 0;
			}
		}
		else if(HvEssUartDecodeBuf.Status == 1)
		{
			HvEssUartDecodeBuf.Checksum ^= ch;
			HvEssUartDecodeBuf.Buffer[HvEssUartDecodeBuf.Index++] = ch;
			if(HvEssUartDecodeBuf.Index >= HvEssUartDecodeBuf.Buffer[0])
			{
				HvEssUartDecodeBuf.Status = 2;
			}
		}
		else if(HvEssUartDecodeBuf.Status == 2)
		{
			if(ch==0x69 && HvEssUartDecodeBuf.Checksum==0)
			{
				appSerialUartPackageParser();
				//appSerialUartSendMessage("Uart Brocast");		
			}			
			HvEssUartDecodeBuf.Status = 0;
		}
		else
			HvEssUartDecodeBuf.Status = 0;
	}
}
/* Public function prototypes -----------------------------------------------*/

  
void appSerialUartDavinciTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	static	uint8_t	count = 0;
	
	//GPIOD->ODR |= GPIO_PIN_14;
//	if(ReportScuUartMessage)
//		ReportScuUartMessage();
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
		
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_3)
	{
		appSerialUartPackageJudgee();
		
		count++;
		if(count >= 100)	
		{
			count = 0;
			SubIndex = 0;
//			ReportScuUartMessage = appSerialUartSendCellVoltage;
//			GPIOC->ODR ^= GPIO_PIN_6;
			{
				static	uint8_t ccc=0;
				char	str[100];			
				sprintf(str,"Uart Brocast:%d",ccc++);
//				appSerialUartSendMessage(str);
			}
		}
	}
	//GPIOD->ODR &= ~GPIO_PIN_14;
}

void appSerialUartDavinciOpen(void)
{
	if(smp_uart_init(&mDavinci_uart, DavinciUart_cb)==SMP_SUCCESS){
      //SMP_PRINT("smp uart initial success!\r\n");
	 }else{
      //SMP_PRINT("smp uart initial fail!\r\n");
  	}  

  	LibSwTimerOpen(appSerialUartDavinciTimerHandler, 0);
}



/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



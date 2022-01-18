/**
  ******************************************************************************
  * @file        AppTest.c
  * @author      Johnny
  * @version     v0.0.0
  * @date        2021/12/22
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
#include "main.h"
#include "halafe.h"
#include "LibSwTimer.h"
#include "AppBalance.h"
#include "LibNtc.h"
#include "AppGauge.h"
#include "ApiSysPar.h"
#include "smp_w5500_DMA.h"
#include "AppSerialCanDavinci.h"
#include "ApiFu.h"
#include "ApiCanbusPkgFu.h"

void appSerialCanDavinciSendTextMessage(char *msg);

#define	appTcpipSmpDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)


/* Private define ------------------------------------------------------------*/
#define	appTcpipScuId()		appProjectGetScuId()

#define	SMP_TCPIP_PKG_IDLE_COUNT	50

#define SMP_TCPIP_PKG_START_CODE    0x5A
#define SMP_TCPIP_PKG_END_CODE      0x69
#define	SMP_TCPIP_PKG_TYPE_FU		'C'

#define	SMP_TCPIP_DATA_BUF_MAX_SIZE					2100
static uint8_t RxBuf[SMP_TCPIP_DATA_BUF_MAX_SIZE];
static uint8_t TxBuf[SMP_TCPIP_DATA_BUF_MAX_SIZE];

#define SMP_TCPIO_SOCKET {														\
					.Num						= NULL,						\
					.Protocol 					= Sn_MR_TCP,				\
					.PortNum  					= 1384,						\
					.DeviceID					= 1,						\
					.Memory.rx_buf_Ptr          = RxBuf,					\
					.Memory.rx_buf_size         = SMP_TCPIP_DATA_BUF_MAX_SIZE,       	\
					.Memory.tx_buf_Ptr          = TxBuf,					\
					.Memory.tx_buf_size         = SMP_TCPIP_DATA_BUF_MAX_SIZE 		\
					}		
W5500_Socket_parm SmpTcpipSocket = SMP_TCPIO_SOCKET;

#define SMP_TCPIO_SOCKET1 {														\
					.Num						= NULL,						\
					.Protocol 					= Sn_MR_TCP,				\
					.PortNum  					= 1234,						\
					.DeviceID					= 1,						\
					.Memory.rx_buf_Ptr          = RxBuf,					\
					.Memory.rx_buf_size         = SMP_TCPIP_DATA_BUF_MAX_SIZE,       	\
					.Memory.tx_buf_Ptr          = TxBuf,					\
					.Memory.tx_buf_size         = SMP_TCPIP_DATA_BUF_MAX_SIZE 		\
					}		
W5500_Socket_parm SmpTcpipSocket1 = SMP_TCPIO_SOCKET1;


/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t		IdleCount;
	uint8_t		Status;
	uint8_t		Index;
	uint8_t		Checksum;
	uint8_t		Buffer[260];
}tTcpipFuDecode;

static tTcpipFuDecode	TcpipFuDecode;

/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void tcpipFuCallBack(uint16_t evt, uint8_t *pMsgBuf)
{
	smp_can_package_t	CanPkg;

	switch(evt)
	{
	case API_FU_EVT_PROGRESS:
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_FU_TX, appProjectGetScuId(),
									SMP_FU_INFO_OBJ_INDEX,
									SMP_FU_INFO_PROGRESS_SUB_INDEX);
		CanPkg.dlc = 2;
		CanPkg.dat[0] = pMsgBuf[0];
		CanPkg.dat[1] = pMsgBuf[1];
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
		break;
	case API_FU_EVT_CHECK_RESULT:
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_FU_TX, appProjectGetScuId(),
									SMP_FU_INFO_OBJ_INDEX,
									SMP_FU_INFO_FW_CHECKING_SUB_INDEX);
		CanPkg.dlc = 2;
		CanPkg.dat[0] = 0;
		CanPkg.dat[1] = pMsgBuf[0];
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
		break;
	case API_FU_EVT_START_FW_CHECK:
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_FU_TX, appProjectGetScuId(),
									SMP_FU_INFO_OBJ_INDEX,
									SMP_FU_INFO_FW_CHECKING_SUB_INDEX);
		CanPkg.dlc = 3;
		CanPkg.dat[0] = 1;
		CanPkg.dat[1] = pMsgBuf[0];
		CanPkg.dat[2] = pMsgBuf[1];
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
		break;
	case API_FU_EVT_FW_CHECKING:
		CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_FU_TX, appProjectGetScuId(),
									SMP_FU_INFO_OBJ_INDEX,
									SMP_FU_INFO_FW_CHECKING_SUB_INDEX);
		CanPkg.dlc = 3;
		CanPkg.dat[0] = 2;
		CanPkg.dat[1] = pMsgBuf[0];
		CanPkg.dat[2] = pMsgBuf[1];
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
		break;
	}
//	canFuDebugMsg("FuCallbackFunction");
}


static void sendFuDataToAnotherScu(smp_can_package_t *pCanPkg, uint8_t times)
{
	uint16_t	i;
	for(i=0; i<times; i++)
		appSerialCanDavinciPutPkgToCanFifo(pCanPkg);
}

static void paserCanbusFuPkg(smp_can_package_t *pCanPkg,uint8_t times)
{
	uint8_t		objindex;
	uint8_t		scuid;
	uint32_t	address;

//	canFuDebugMsg("DavinciCanFunFuRx");
		
	scuid = SMP_CAN_GET_SCU_ID(pCanPkg->id);
	if(scuid != appTcpipScuId())
		sendFuDataToAnotherScu(pCanPkg, times);
		
	if(scuid == 0 || scuid == appTcpipScuId())
	{		
		objindex = SMP_CAN_GET_OBJ_INDEX(pCanPkg->id);
	
		if(objindex == SMP_FU_INFO_OBJ_INDEX)
		{
			apiCanbusPkgFuDecodeInfoPackage(pCanPkg, tcpipFuCallBack);
		}
		else if(objindex >= SMP_FU_DATA_START_OBJ_INDEX &&
				objindex <= SMP_FU_DATA_END_OBJ_INDEX)
		{			
			apiCanbusPkgFuDecodeDataPackage(pCanPkg);
		}
	}
}

static uint16_t	paserTcpipFuPackage(uint8_t *pBuf)
{
//	Rcv W5500:5A 11 43 02 78 56 34 12 08 80 81 82 83 84 85 86 87 50 69 
	char	str[100];
	char	str1[10];
	
	smp_can_package_t	CanPkg;
	uint8_t		times;
	uint8_t		i,len,n;
	uint8_t		index;
	len = pBuf[0] - 1; 
	for(i = 2; i<len; )
	{
		index = i + 6;
		if(index  >= len)
		{
			appTcpipSmpDebugMsg("len1");
			break;
		}
		index += pBuf[i + 5];
		if(index > len)
		{
			appTcpipSmpDebugMsg("len2");
			break;
		}
		
		times = pBuf[i++];
		CanPkg.id = GET_DWORD(&pBuf[i]);	
		i += 4;
		CanPkg.dlc =  pBuf[i++];
		sprintf(str, "CAN %d %.8lX %d %d",
					times,
					CanPkg.id,
					CanPkg.dlc,
					i - 6);
		for(n=0; n<CanPkg.dlc; n++)
		{
			CanPkg.dat[n] = pBuf[i++];
#if	0			
			sprintf(str1," %.2X ",CanPkg.dat[n]);
			strcat(str, str1);			
#endif			
		}
		appTcpipSmpDebugMsg(str);
		
		paserCanbusFuPkg(&CanPkg, times);
	}
	return 0;
}

static uint16_t	judegeTcpipPayload(uint8_t *pBuf, uint16_t DatLen)
{
	uint16_t	i;
	uint8_t		ch;

	for(i=0; i<DatLen; i++)
	{
		ch = pBuf[i];
		TcpipFuDecode.IdleCount = SMP_TCPIP_PKG_IDLE_COUNT;
		if(TcpipFuDecode.Status == 0)
		{
			if(ch == SMP_TCPIP_PKG_START_CODE)			
			{
				TcpipFuDecode.Status  = 1;
				TcpipFuDecode.Index = 0;
				TcpipFuDecode.Checksum = 0;
			}
		}
		else if(TcpipFuDecode.Status == 1)
		{
			TcpipFuDecode.Checksum  ^= ch;
			TcpipFuDecode.Buffer[TcpipFuDecode.Index++] = ch;
			if(TcpipFuDecode.Buffer[0] < 3)
			{
				TcpipFuDecode.Status = 0;
			}
			if(TcpipFuDecode.Index >= TcpipFuDecode.Buffer[0])
			{
				TcpipFuDecode.Status  = 2;
			}
		}
		else if(TcpipFuDecode.Status == 2)
		{
			if(TcpipFuDecode.Checksum  == 0 && ch == SMP_TCPIP_PKG_END_CODE)
			{
				if(TcpipFuDecode.Buffer[1] == SMP_TCPIP_PKG_TYPE_FU)
				{
					paserTcpipFuPackage(TcpipFuDecode.Buffer);
				}
			}
			TcpipFuDecode.Status = 0;
		}
		else
		{
			TcpipFuDecode.Status = 0;
		}
	}		
}

static uint16_t SmpTcpip_CB(W5500_cb_type p_evt, uint16_t DataLen){
	static	uint32_t	offset = 0;
	static uint8_t cnt =0;
	char	str1[10];
	char	str[200];
	uint16_t	i;
	switch(p_evt){
		case W5500_DATA_RECV:
//			GPIOD->ODR |= GPIO_PIN_15;	
			offset += DataLen;
			sprintf(str, "Rcv W5500(%d %d):", DataLen, offset);
			#if	0
			for(i=0; i<DataLen; i++)
			{
				if(i > 20)
					break;
				sprintf(str1,"%.2X ", RxBuf[i]);
				strcat(str, str1);			
			}
			#endif
			appTcpipSmpDebugMsg(str);
			judegeTcpipPayload(RxBuf,DataLen);
			
			//return Modbus_TCPIP_Parser(Socket_Test1,DataLen);
			//strcpy(TxBuf , "Response");
			sprintf(TxBuf, "RET:%3d", cnt++);
			TxBuf[7] = 0;
//			GPIOD->ODR &= ~GPIO_PIN_15;
			return 8;
			break;
		case W5500_Socket_REG_Success:
			appTcpipSmpDebugMsg("TCP/IP Success");
			break;
		case W5500_COMMUNICATE_ERR:
			appTcpipSmpDebugMsg("TCP/IP Error");
			break;
	}
	return 0;
}		


static uint16_t SmpTcpip_CB1(W5500_cb_type p_evt, uint16_t DataLen){
	static	uint32_t	offset = 0;
	static uint8_t cnt =0;
	char	str1[10];
	char	str[200];
	uint16_t	i;
	switch(p_evt){
		case W5500_DATA_RECV:
//			GPIOD->ODR |= GPIO_PIN_15;	
			offset += DataLen;
			sprintf(str, "Rcv1 W5500(%d %d):", DataLen, offset);
			#if	0
			for(i=0; i<DataLen; i++)
			{
				if(i > 20)
					break;
				sprintf(str1,"%.2X ", RxBuf[i]);
				strcat(str, str1);			
			}
			#endif
			appTcpipSmpDebugMsg(str);
			//judegeTcpipPayload(RxBuf,DataLen);
			
			//return Modbus_TCPIP_Parser(Socket_Test1,DataLen);
			//strcpy(TxBuf , "Response");
			sprintf(TxBuf, "ret:%3d", cnt++);
			TxBuf[7] = 0;
//			GPIOD->ODR &= ~GPIO_PIN_15;
			return 8;
			break;
		case W5500_Socket_REG_Success:
			appTcpipSmpDebugMsg("TCP/IP Success");
			break;
		case W5500_COMMUNICATE_ERR:
			appTcpipSmpDebugMsg("TCP/IP Error");
			break;
	}
	return 0;
}		

static void smpTcpipSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
		return;
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_6)
	{
		if(TcpipFuDecode.IdleCount)
		{
			TcpipFuDecode.IdleCount--;
			if(TcpipFuDecode.IdleCount == 0)
			{
				TcpipFuDecode.Status = 0;
				appTcpipSmpDebugMsg("Reset Tcpip Status");
			}
		}
	}
}

/* Public function prototypes -----------------------------------------------*/

void appTcpipSmpOpen(void)
{
	memset(&TcpipFuDecode, 0, sizeof(tTcpipFuDecode));
	
	LibSwTimerOpen(smpTcpipSwTimerHandler, 0);
	W5500_Socket_Register(&SmpTcpipSocket , SmpTcpip_CB); 

//	W5500_Socket_Register(&SmpTcpipSocket1 , SmpTcpip_CB1); 

	appTcpipSmpDebugMsg("appTcpipSmpOpen");
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



/**
  ******************************************************************************
  * @file        AppSerialCanDavinci.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/10/20
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
#include "smp_can.h"
#include "smp_can_fifo.h"
#include "LibSwTimer.h"
#include "AppSerialCanDavinci.h"

#include "AppSerialCanDavinciDebug.h"
#include "AppSerialCanDavinciCommon.h"
#include "AppSerialCanDavinciCmd.h"
#include "AppSerialCanDavinciBaseCmd.h"
#include "AppSerialCanDavinciFirmwareUpgrade.h"
#include "HalRtc.h"
#include "AppBms.h"

#define	appSerialCanDavinciDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private typedef -----------------------------------------------------------*/
#define	CAN_TX_BUF_SIZE	400
#define	CAN_RX_BUF_SIZE	400

smp_can_package_t	Davinci_can_tx_buffer[CAN_TX_BUF_SIZE] = {0};
smp_can_package_t  Davinci_can_rx_buffer[CAN_RX_BUF_SIZE] = {0};


#define DAVINCI_CAN0		{                                                                      \
                                .num                        = __CAN0,                            \
                                .baud_rate                  = 500000,                          \
                                .buffers.rx_buf             = Davinci_can_rx_buffer,              \
                                .buffers.rx_buf_size        = CAN_TX_BUF_SIZE,                              \
                                .buffers.tx_buf             = Davinci_can_tx_buffer,              \
                                .buffers.tx_buf_size        = CAN_RX_BUF_SIZE                               \
                          }  


#define DAVINCI_CAN		{                                                                      \
                                .num                        = __CAN1,                            \
                                .baud_rate                  = 500000,                          \
                                .buffers.rx_buf             = Davinci_can_rx_buffer,              \
                                .buffers.rx_buf_size        = CAN_TX_BUF_SIZE,                              \
                                .buffers.tx_buf             = Davinci_can_tx_buffer,              \
                                .buffers.tx_buf_size        = CAN_RX_BUF_SIZE                               \
                          }  


smp_can_t mDavinci_can0 = DAVINCI_CAN0;
smp_can_t mDavinci_can = DAVINCI_CAN;

	
/* Private define ------------------------------------------------------------*/



/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint32_t	can1_buf_c = 0;
/* Private function prototypes -----------------------------------------------*/

SMP_CAN_DECODE_CMD_START(mDavinciCanFunDecodeTab)
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_RX, 0,
									0,
									0),
								CHECK_SMP_CAN_FUN,
								DavinciCanFunBaseRxTx)
								
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_BASE_TX, 0,
									0,
									0),
								CHECK_SMP_CAN_FUN,
								DavinciCanFunBaseRxTx)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_CMD_RX, 0,
									0,
									0),
								CHECK_SMP_CAN_FUN,
								DavinciCanFunCmdRx)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_DEBUG_RX, 0,
									0,
									0),
								CHECK_SMP_CAN_FUN,
								DavinciCanFunDebugRx)

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_COMMON_RX, 0,
									0,
									0),
								CHECK_SMP_CAN_FUN,
								DavinciCanFunCommonRx)								
	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_COMMON_TX, 0,
									0,
									0),
								CHECK_SMP_CAN_FUN,
								DavinciCanFunCommonRx)								

	SMP_CAN_DECODE_CMD_CONTENT(	MAKE_SMP_CAN_ID(SMP_CAN_FUN_FU_RX, 0,
									0,
									0),
								CHECK_SMP_CAN_FUN,
								DavinciCanFunFuRx)								

SMP_CAN_DECODE_CMD_END();



//--Send: 5A 05 55 70 70 50 69 

void DavinciCan_cb(can_evt_type p_evt){
  switch(p_evt){
    case CAN_DATA_READY:
      /* received data handle */
		can1_buf_c++;
    break;
    case CAN_TX_EMPTY:
      /* Data transmission complete handle */
    break;
    case CAN_COMMUNICATION_ERR:
      /* occurred during reception */
    break;
    case CAN_BUFFER_FULL:
      /* occurred UART buffer full */
    break;    
    default:
    break;
  }
}
static void dump_danpackage(smp_can_package_t *pCanPkg)
{
	char	str[100];
	char	str1[10];
	int		i;
	sprintf(str,"RCV: %.8lX %d",pCanPkg->id , pCanPkg->dlc);
	
	for(i=0; i<pCanPkg->dlc; i++)
	{
		sprintf(str1," %.2X",pCanPkg->dat[i]);
		strcat(str, str1);
	}
//	appSerialCanDavinciDebugMsg(str);
}


static void canDavinciPaserCanPackage(void)
{
	uint8_t	i,n;
	smp_can_package_t	CanPkg;
	uint8_t cmdIndex;
	
	char	str[100];
	char	str1[100];
	for(i=0; i<6; i++)
	{
		if(smp_can_get(&mDavinci_can, &CanPkg) != SMP_SUCCESS)
			break;
		dump_danpackage(&CanPkg);	
	 	cmdIndex = 0;
 		for(cmdIndex = 0; mDavinciCanFunDecodeTab[cmdIndex].fun != 0; cmdIndex++)
 		{
 			//sprintf(str,"%.8X %.8X %.8X",mDavinciCanFunDecodeTab[cmdIndex].canid,
 			//		mDavinciCanFunDecodeTab[cmdIndex].mask,
 			//		CanPkg.id
 			//	);
 			//appSerialCanDavinciDebugMsg(str);
 			
 			if((mDavinciCanFunDecodeTab[cmdIndex].canid & mDavinciCanFunDecodeTab[cmdIndex].mask) == 
 		  	 	(mDavinciCanFunDecodeTab[cmdIndex].mask & CanPkg.id))
 		  	{
 				mDavinciCanFunDecodeTab[cmdIndex].fun(&CanPkg);
 				break;
 			}
 		}
	}	
		
}

/* Public function prototypes -----------------------------------------------*/
uint8_t appSerialCanDavinciIsCorrectScuId(smp_can_package_t *pCanPkg)
{
	if(SMP_CAN_GET_SCU_ID(pCanPkg->id) == appProjectGetScuId())
		return 1;
	else
		return 0;
}

void appSerialCanDavinciPutPkgToCanFifo(smp_can_package_t *pCanPkg)
{
//	SMP_CAN_GET_SCU_ID(id)		((id>>18)&0x7f)
//	appSerialCanDavinciPutPkgToCanFifo
	
	smp_can_put(&mDavinci_can, pCanPkg);
}


void appSerialCanDavinciSendTextMessage(char *msg)
{
	uint8_t	buffer[256];
	uint8_t	i,j,len,checksum,index;
	smp_can_package_t	CanPkg;
	uint8_t		scuid;
	
	if(appBmsIsScudIdReady() == 0)
	{
		scuid = 1;
	//	return;
	}
	else
	{
		scuid = appProjectGetScuId();
	}
	
	len = strlen((char *)msg);
	buffer[0] = 0x5A;
	buffer[1] = len + 5;
	buffer[2] = 0;
	buffer[3] = 'T';
	checksum = 'T';
	checksum ^= buffer[1];
	index = 4;
	for(i=0; i<len; i++)
	{
		buffer[index++] = msg[i];
		checksum ^= msg[i];
	}
	buffer[index++] = 0;
	buffer[index++] = checksum;
	buffer[index++] = 0x69;


	CanPkg.id = MAKE_SMP_CAN_ID(SMP_CAN_FUN_DEBUG_TX, scuid, //appProjectGetScuId(),
									SMP_DEBUG_PKG_TX_OBJ_INDEX,
									0);
	for(i=0; i<index; i+=8)
	{
		len = index - i;
		if(len > 8)
			len = 8;
		memcpy(CanPkg.dat, &buffer[i], len);
		CanPkg.dlc = len;
		appSerialCanDavinciPutPkgToCanFifo(&CanPkg);
	}
}



void appSerialCanDavinciTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
		canDavinciPaserCanPackage();
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_3)
	{
		appSerialCanDavinciNotificationHandler(evt);
	}
}

void appSerialCanDavinciOpen(void)
{
	char	str[100];
	#if	1
	int	res;
	
	res = smp_can_init(&mDavinci_can0, DavinciCan_cb);//
	sprintf(str,"Can0 ini %d", res);
	appSerialCanDavinciDebugMsg(str);

	
	#if 0
	if(smp_can_init(&mDavinci_can, DavinciCan_cb)==SMP_SUCCESS){
	    appSerialCanDavinciDebugMsg("Can 1 ini success");
	 }else{
	    appSerialCanDavinciDebugMsg("Can 1 ini fail");
  }  
  #endif
	
	#endif
  	LibSwTimerOpen(appSerialCanDavinciTimerHandler, 0);
}

void appSerialCanDavinciClose(void)
{
	smp_can_deinit(&mDavinci_can0);
	smp_can_deinit(&mDavinci_can);
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



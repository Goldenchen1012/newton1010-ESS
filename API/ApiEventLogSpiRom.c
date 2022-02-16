/**
  ******************************************************************************
  * @file        ApiEventLoSpiRom.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2022/1/19
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "define.h"
#include "main.h"
#include "LibSwTimer.h"
#include "ApiEventLog.h"
#include "HalEeprom.h"
#include "smp_log_managment.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	eventSpiRomDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)

/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t	eventLogBuffer[260];
static tApiEventLogCallbackFunction EventCbFunction = 0;
static uint8_t	eventLogReadNum = 0;
/* Public variables ---------------------------------------------------------*/
/* Private function prototypes ----------------------------------------------*/
static void logEventHandler(smp_log_evt_type p_evt)
{
//#define	SHOW_LOG_DEBUG_MSG	
	//eventSpiRomDebugMsg("SPI Event logEventHandler");

	switch(p_evt)
	{
	case SMP_LOG_EVENT_SECTOR_HEADER_LOAD_DONE:
		//eventSpiRomDebugMsg("SMP_LOG_EVENT_SECTOR_HEADER_LOAD_DONE");
#ifdef SHOW_LOG_DEBUG_MSG		
				LOG_CYAN("load head\r\n");
				LOG_CYAN("header %x\r\n",header_package.header[0]);
				LOG_CYAN("header %x\r\n",header_package.header[1]);
				LOG_CYAN("header %x\r\n",header_package.header[2]);
				LOG_CYAN("header %x\r\n",header_package.header[3]);
				LOG_CYAN("reflash head page%x\r\n",header_package.reflash_memory_head_page);
				LOG_CYAN("reflash current page%x\r\n",header_package.reflash_memory_current_page);
				LOG_CYAN("reflash cnt%d\r\n",header_package.reflash_total_log_cnt);
				LOG_CYAN("fix curent page%x\r\n",header_package.fix_memory_current_page);
				LOG_CYAN("fix cnt%d\r\n",header_package.fix_total_log_cnt);
#endif		
		break;
	case SMP_LOG_EVENT_SECTOR_HEADER_SAVE_DONE:
		//eventSpiRomDebugMsg("SMP_LOG_EVENT_SECTOR_HEADER_SAVE_DONE");
#ifdef SHOW_LOG_DEBUG_MSG			
				LOG_CYAN("save head\r\n");
				LOG_CYAN("header %x\r\n",header_package.header[0]);
				LOG_CYAN("header %x\r\n",header_package.header[1]);
				LOG_CYAN("header %x\r\n",header_package.header[2]);
				LOG_CYAN("header %x\r\n",header_package.header[3]);
				LOG_CYAN("reflash head page%x\r\n",header_package.reflash_memory_head_page);
				LOG_CYAN("reflash current page%x\r\n",header_package.reflash_memory_current_page);
				LOG_CYAN("reflash cnt%d\r\n",header_package.reflash_total_log_cnt);
				LOG_CYAN("fix curent page%x\r\n",header_package.fix_memory_current_page);
				LOG_CYAN("fix cnt%d\r\n",header_package.fix_total_log_cnt);
#endif		
		break;
	case SMP_LOG_EVENT_PAGE_LOAD_DONE:
		//eventSpiRomDebugMsg("SMP_LOG_EVENT_PAGE_LOAD_DONE");
		if(EventCbFunction)
		{
			EventCbFunction(eventLogReadNum, eventLogBuffer);
		}
#ifdef SHOW_LOG_DEBUG_MSG			
				LOG_CYAN("page load\r\n");
				for(int i = 0; i < 256;i++){				
					LOG_CYAN("%d,%x\r\n",i,page_data_buffer[i]);
				}
#endif		
		break;
	case SMP_LOG_EVENT_PAGE_SAVE_DONE:
		//eventSpiRomDebugMsg("SMP_LOG_EVENT_PAGE_SAVE_DONE");
#ifdef SHOW_LOG_DEBUG_MSG			
				LOG_CYAN("page save\r\n");
#endif		
		break;
	case SMP_LOG_EVENT_MEMORY_FULL:
		//eventSpiRomDebugMsg("SMP_LOG_EVENT_MEMORY_FULL");
#ifdef SHOW_LOG_DEBUG_MSG			
				LOG_CYAN("fix memory full\r\n");
#endif		
		break;
	case SMP_LOG_EVENT_ERROR:
		//eventSpiRomDebugMsg("");
#ifdef SHOW_LOG_DEBUG_MSG			
				LOG_CYAN("Error\r\n");
#endif		
		break;
	default:
		eventSpiRomDebugMsg("Default log Event");
		break;
	}
}


/* Public function prototypes -----------------------------------------------*/

/*
				} 
				if(!strcmp((char *)rx_data, "clean reflash\r\n")){ 
					LOG_YELLOW("EVENT_LOG Clean Reflash Memory\r\n");					
					app_flash_log_managment_clean_reflash_memory();
				} 
				if(!strcmp((char *)rx_data, "clean fix\r\n")){   
					LOG_YELLOW("EVENT_LOG Clean Fix Memory\r\n");
					app_flash_log_managment_clean_fix_memory();
*/

void apiEventLogClearLogData(void)
{
	app_flash_log_managment_clean_all_memory();
}

uint32_t apiEventLogGetLogNumber(void)
{
	char	str[100];
	smp_sector_header_package	header_package;
	
	app_flash_sector_header_get(&header_package);	
	
	sprintf(str,"Head %.2X %.2X %.2X %.2X", 
					header_package.header[0],
					header_package.header[1],
					header_package.header[2],
					header_package.header[3]);
	eventSpiRomDebugMsg(str);
	
	sprintf(str, "Log Num1 = %d", header_package.reflash_total_log_cnt);
	eventSpiRomDebugMsg(str);

	sprintf(str, "Log Num2 = %d", header_package.fix_total_log_cnt);
	eventSpiRomDebugMsg(str);
	
	
	return header_package.reflash_total_log_cnt;
	//return 0;
}
void apiEventLogReadLogData(uint32_t EvetIdex, uint8_t ReadNumber, tApiEventLogCallbackFunction CbFunction)
{
	uint32_t	EventNum;
	tHalEeProm	mHalEeProm;
	
	if(!CbFunction)
		return;
	EventCbFunction = CbFunction;
	if(ReadNumber > 32)
		ReadNumber = 32;
	eventLogReadNum = ReadNumber;
	app_flash_page_data_load(eventLogBuffer, EvetIdex, ReadNumber, SMP_REFLASH_MEMORY);		
		//eventLogBuffer
#if	0		
void app_flash_page_data_load(uint8_t * RX_buffer , uint16_t log_start_position, uint16_t log_length,smp_flash_type flash_type){

//					app_flash_page_data_load(page_data_buffer,start_package,length_data,SMP_REFLASH_MEMORY);


	EventNum = apiEventLogGetLogNumber();
	if(EvetIdex >= EventNum)
	{
		CbFunction(0, 0);
		return;
	}
	if((EvetIdex + ReadNumber) >= EventNum)
	{
		ReadNumber = EventNum - EvetIdex;	
	}
	if(ReadNumber == 0)
	{
		CbFunction(0, 0);
		return;
	}
	
	mHalEeProm.StartAddress = FLASHROM_EVENT_LOG_START_ADDR + EvetIdex * 8;
	mHalEeProm.Length = ReadNumber * 8;
	mHalEeProm.pDataBuffer = mEventLog.EventTempBuf;
	HalEePromRead(&mHalEeProm);
	
	CbFunction(ReadNumber, mEventLog.EventTempBuf);
#endif	
}
void apiEventLogSaveLogData(uint8_t EventType,uint16_t Par)
{
	smp_log_package log_package;
	tLbyte			RtcSecond;
	uint8_t			i;
	uint8_t			*pBuf;

	RtcSecond.l = HalRtcGetSmpUnixTime();

	log_package.ID = EventType;
	log_package.SMP_RTC[0] = RtcSecond.b[0];
	log_package.SMP_RTC[1] = RtcSecond.b[1];
	log_package.SMP_RTC[2] = RtcSecond.b[2];
	log_package.SMP_RTC[3] = RtcSecond.b[3];
	log_package.data[0] = Par % 0x100;
	log_package.data[1] = Par / 0x100;
	log_package.sum = EVENT_LOG_INI_CHECKSUM;
	
	pBuf =(uint8_t *)&log_package;
	
	for(i=0; i<7; i++)
		log_package.sum ^= pBuf[i];

	app_flash_page_data_push(log_package, SMP_REFLASH_MEMORY);
}
void apiEventLogOpen(void)
{
	int		i;
	smp_sector_header_package	header_package;
	char	str[100];

	app_flash_log_managment_init(logEventHandler);
	
	eventSpiRomDebugMsg("SPI Event log open");
//	for(i=0; i<33; i++)
//		apiEventLogSaveLogData(0x11,0x1234);
	app_flash_sector_header_get(&header_package);	

	sprintf(str,"Head %.2X %.2X %.2X %.2X", 
					header_package.header[0],
					header_package.header[1],
					header_package.header[2],
					header_package.header[3]);
	eventSpiRomDebugMsg(str);
	
	sprintf(str, "Log Num1 = %d", header_package.reflash_total_log_cnt);
	eventSpiRomDebugMsg(str);

	sprintf(str, "Log Num2 = %d", header_package.fix_total_log_cnt);
	eventSpiRomDebugMsg(str);
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    




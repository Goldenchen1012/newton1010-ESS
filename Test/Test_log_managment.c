/**
  ******************************************************************************
  * @file    Test_log_managment.c
  * @author  Golden Chen
  * @version V0.0.1
  * @date    2022/02/14
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 SMP</center></h2>
  *
  * 
  *
  ******************************************************************************
  */

#include "smp_log_managment.h"
#include "Test_log_managment.h"
#include <string.h>	
#include "main.h"
#include "LibSwTimer.h"
#include "smp_uart.h"
#include "SEGGER_RTT.h"
#include "RTT_Log.h"

uint32_t test_event_log_cnt = 0;

extern uint16_t Davinci_uart_rx_cnt;
extern smp_uart_t mDavinci_uart;
static smp_sector_header_package	header_package;
uint8_t page_data_buffer[256];
uint16_t kk = 0;


uint8_t temp[18];
static uint8_t temp1[4];
static uint8_t temp2[4];
uint8_t temp3[14];
uint16_t start_package;
uint16_t length_data;
uint16_t long_run_error_cnt = 0;
uint32_t long_run_cnt = 0;
//---------------------------------------------------------------------------------------
int a2i(char* txt)
{
    int sum, digit, i;
    sum = 0;
    for (i = 0; i < 4; i++) {
        digit = txt[i] - 0x30;
        sum = (sum * 10) + digit;
    }
    return sum;
}


void test_uart_rx_process(void)
{
	static uint8_t rx_data[256];
	static int8_t fifo_res;

	if(Davinci_uart_rx_cnt>0){
		fifo_res = smp_uart_get_string(&mDavinci_uart, rx_data);
		--Davinci_uart_rx_cnt;
		if(fifo_res == SMP_SUCCESS)
		{	
				LOG_YELLOW("EVENT_LOG UART %s\r\n", rx_data);
				if(!strcmp((char *)rx_data, "long run get\r\n")){ 
					LOG_CYAN("Error %d\r\n",long_run_error_cnt);
					LOG_CYAN("Error %d\r\n",long_run_cnt);
				}
				if(!strcmp((char *)rx_data, "clean all\r\n")){ 
					LOG_YELLOW("EVENT_LOG Clean ALL Memory\r\n");					
					app_flash_log_managment_clean_all_memory();
				} 
				if(!strcmp((char *)rx_data, "clean reflash\r\n")){ 
					LOG_YELLOW("EVENT_LOG Clean Reflash Memory\r\n");					
					app_flash_log_managment_clean_reflash_memory();
				} 
				if(!strcmp((char *)rx_data, "clean fix\r\n")){   
					LOG_YELLOW("EVENT_LOG Clean Fix Memory\r\n");
					app_flash_log_managment_clean_fix_memory();
				} 
//				if(!strcmp((char *)rx_data, "head clean\r\n")){ 
//					LOG_YELLOW("EVENT_LOG Clean head\r\n");
//					app_flash_log_managment_clean_head();
//				}
//				if(!strcmp((char *)rx_data, "head save\r\n")){ 
//					LOG_YELLOW("EVENT_LOG Save Header\r\n");
//					header_package.header[0] = 'S';
//					header_package.header[1] = 'M';
//					header_package.header[2] = 'P';
//					header_package.header[3] = 'S';//S : sector
//					header_package.reflash_memory_head_page = REFLASH_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
//					header_package.reflash_memory_current_page = REFLASH_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
//					header_package.reflash_total_log_cnt = kk;
//					header_package.fix_memory_current_page = FIX_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
//					header_package.fix_total_log_cnt = kk;
//					app_flash_sector_header_save(&header_package);
//					kk +=1;
//				} 
				if(!strcmp((char *)rx_data, "head load\r\n")){  
					LOG_YELLOW("EVENT_LOG Load Header\r\n");
					app_flash_sector_header_load(&header_package);		
				} 
				if(!strcmp((char *)rx_data, "head get\r\n")){  
					LOG_YELLOW("EVENT_LOG Load Header\r\n");
					app_flash_sector_header_get(&header_package);	
					LOG_CYAN("header %x\r\n",header_package.header[0]);
					LOG_CYAN("header %x\r\n",header_package.header[1]);
					LOG_CYAN("header %x\r\n",header_package.header[2]);
					LOG_CYAN("header %x\r\n",header_package.header[3]);
					LOG_CYAN("reflash head page%x\r\n",header_package.reflash_memory_head_page);
					LOG_CYAN("reflash current page%x\r\n",header_package.reflash_memory_current_page);
					LOG_CYAN("reflash cnt%d\r\n",header_package.reflash_total_log_cnt);
					LOG_CYAN("fix curent page%x\r\n",header_package.fix_memory_current_page);
					LOG_CYAN("fix cnt%d\r\n",header_package.fix_total_log_cnt);
				} 
				if(!strcmp((char *)rx_data, "data save reflash\r\n")){  
					LOG_YELLOW("EVENT_LOG Force Save reflash\r\n");
					app_flash_page_data_save(SMP_REFLASH_MEMORY);
				} 


				if(!strcmp((char *)rx_data, "data save fix\r\n")){  
					LOG_YELLOW("EVENT_LOG Force Save Fix\r\n");
					app_flash_page_data_save(SMP_FIX_MEMORY);
				} 
				if(!strcmp((char *)rx_data, "check head\r\n")){  
					LOG_YELLOW("EVENT_LOG Check Head\r\n");
					app_flash_check_head();
				} 
				

				memcpy(temp,&rx_data[0],17);
				if(!strcmp((char *)temp, "data load reflash")){ 	//data load reflash xxxx xxxx
					memcpy(temp1,&rx_data[18],4);
					memcpy(temp2,&rx_data[23],4);
					start_package =  a2i((char *)temp1);
					length_data =  a2i((char *)temp2);
					LOG_YELLOW("EVENT_LOG Load Reflash %d %d\r\n",start_package,length_data);
					memset(&page_data_buffer[0], 0, 256);
					app_flash_page_data_load(page_data_buffer,start_package,length_data,SMP_REFLASH_MEMORY);
				} 
				if(!strcmp((char *)temp, "data push reflash")){  
					smp_log_package log_package;
					memcpy(temp1,&rx_data[18],4);
					length_data =  a2i((char *)temp1);
					LOG_YELLOW("EVENT_LOG Push Reflash %d\r\n",length_data);
					for(int i = 0; i < length_data;i++){
						log_package.ID = 0xD6;
						log_package.SMP_RTC[0] = 0x75;
						log_package.SMP_RTC[1] = 0x04;
						log_package.SMP_RTC[2] = 0;
						log_package.SMP_RTC[3] = 0;
						log_package.data[0] = 0x00;
						log_package.data[1] = 0x00;
						log_package.sum = 0x02;
						app_flash_page_data_push(log_package,SMP_REFLASH_MEMORY);
					}
				} 
				
				memcpy(temp3,&rx_data[0],13);
				if(!strcmp((char *)temp3, "data load fix")){ 	//data load fix xxxx xxxx
					memcpy(temp1,&rx_data[14],4);
					memcpy(temp2,&rx_data[19],4);
					start_package =  a2i((char *)temp1);
					length_data =  a2i((char *)temp2);
					LOG_YELLOW("EVENT_LOG Load Fix %d %d\r\n",start_package,length_data);
					memset(&page_data_buffer[0], 0, 256);
					app_flash_page_data_load(page_data_buffer,start_package,length_data,SMP_FIX_MEMORY);
				} 
				if(!strcmp((char *)temp3, "data push fix")){  
					smp_log_package log_package;
					memcpy(temp1,&rx_data[14],4);
					length_data =  a2i((char *)temp1);
					LOG_YELLOW("EVENT_LOG Push Fix %d\r\n",length_data);
					for(int i = 0; i < length_data;i++){
						log_package.ID = 0xD6;
						log_package.SMP_RTC[0] = 0x75;
						log_package.SMP_RTC[1] = 0x04;
						log_package.SMP_RTC[2] = 0;
						log_package.SMP_RTC[3] = 0;
						log_package.data[0] = 0x00;
						log_package.data[1] = 0x00;
						log_package.sum = 0x02;
						app_flash_page_data_push(log_package,SMP_FIX_MEMORY);
					}
				} 

				memset(&rx_data[0], 0, sizeof(rx_data));
		}
	}
}

static void smp_flash_log_SwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		smp_log_package log_package;
		log_package.ID = 0xD6;
		log_package.SMP_RTC[0] = 0x75;
		log_package.SMP_RTC[1] = 0x04;
		log_package.SMP_RTC[2] = 0;
		log_package.SMP_RTC[3] = 0;
		log_package.data[0] = 0x00;
		log_package.data[1] = 0x00;
		log_package.sum = 0x02;
		app_flash_page_data_push(log_package,SMP_REFLASH_MEMORY);
		
		app_flash_sector_header_get(&header_package);	
//		LOG_CYAN("header %x\r\n",header_package.header[0]);
//		LOG_CYAN("header %x\r\n",header_package.header[1]);
//		LOG_CYAN("header %x\r\n",header_package.header[2]);
//		LOG_CYAN("header %x\r\n",header_package.header[3]);
//		LOG_CYAN("reflash head page%x\r\n",header_package.reflash_memory_head_page);
//		LOG_CYAN("reflash current page%x\r\n",header_package.reflash_memory_current_page);
//		LOG_CYAN("reflash cnt%d\r\n",header_package.reflash_total_log_cnt);
//		LOG_CYAN("fix curent page%x\r\n",header_package.fix_memory_current_page);
//		LOG_CYAN("fix cnt%d\r\n",header_package.fix_total_log_cnt);
		
		memset(&page_data_buffer[0], 0, 256);
		app_flash_page_data_load(page_data_buffer,header_package.reflash_total_log_cnt - 1,1,SMP_REFLASH_MEMORY);
	
	}
}
void app_flash_log_event_handler(smp_log_evt_type p_evt)
{
	switch(p_evt){
		smp_log_package log_package;
		case SMP_LOG_EVENT_PAGE_LOAD_DONE:
//				LOG_CYAN("page load\r\n");
//				for(int i = 0; i < 256;i++){				
//					LOG_CYAN("%d,%x\r\n",i,page_data_buffer[i]);
//				}
				
				log_package.ID = 0xD6;
				log_package.SMP_RTC[0] = 0x75;
				log_package.SMP_RTC[1] = 0x04;
				log_package.SMP_RTC[2] = 0;
				log_package.SMP_RTC[3] = 0;
				log_package.data[0] = 0x00;
				log_package.data[1] = 0x00;
				log_package.sum = 0x02;
				long_run_cnt ++;
				if(memcmp(&log_package,&page_data_buffer[0],8))
				{
					long_run_error_cnt ++;
					LOG_CYAN("Error\r\n");
				}
		break;
		case SMP_LOG_EVENT_PAGE_SAVE_DONE:
				LOG_CYAN("page save\r\n");
		break;
		case SMP_LOG_EVENT_MEMORY_FULL:
				LOG_CYAN("fix memory full\r\n");
		break;
		case SMP_LOG_EVENT_ERROR:
				LOG_CYAN("Error\r\n");
		break;
		default:
		break;
	}
}

void Test_Event_Log_FUNC(void){
  smp_mx25l_status mx251_status;

	smp_mx25l_flash_init();
	app_flash_log_managment_init(app_flash_log_event_handler);
	LibSwTimerOpen(smp_flash_log_SwTimerHandler, 0);
	HAL_Delay(1000);	
	
	while(test_event_log_cnt< TEST_EVENT_LOG_CYCLE_CNT)
	{	
		LibSwTimerHandle();
		test_uart_rx_process();
		test_event_log_cnt++;
		HAL_Delay(100);
	}
}

//--------FILE END------------------------------------------------------------------------------------------

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
#include <string.h>	
#include "main.h"
#include "smp_uart.h"
#include "SEGGER_RTT.h"
#include "RTT_Log.h"

extern uint16_t Davinci_uart_rx_cnt;
extern smp_uart_t mDavinci_uart;
smp_sector_header_package	header_package;
uint8_t page_data_buffer[256];

//John test event log appcalition, but this test code no use smp_uart, so that Golden modify to "test_uart_rx_process" for test event log.
//---------------------------------------------------------------------------------------
#if 0
uint16_t kk = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {
		if(usart_buf.aRxBuff == '\n'){
			usart_buf.Rx_end_flag = 1;
			usart_buf.RxBuff[usart_buf.RxSize] = usart_buf.aRxBuff;
			printf("UART %s",usart_buf.RxBuff);
			if(!strcmp((char *)usart_buf.RxBuff, "clean all\r\n")){       
				app_flash_log_managment_clean_all_memory();
			} 
			if(!strcmp((char *)usart_buf.RxBuff, "clean reflash\r\n")){       
				app_flash_log_managment_clean_reflash_memory();
			} 
			if(!strcmp((char *)usart_buf.RxBuff, "clean fix\r\n")){       
				app_flash_log_managment_clean_fix_memory();
			} 
			if(!strcmp((char *)usart_buf.RxBuff, "head clean\r\n")){ 
				app_flash_log_managment_clean_head();
			}
			if(!strcmp((char *)usart_buf.RxBuff, "head save\r\n")){  
				header_package.header[0] = 'S';
				header_package.header[1] = 'M';
				header_package.header[2] = 'P';
				header_package.header[3] = 'S';//S : sector
				header_package.reflash_memory_head_page = REFLASH_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
				header_package.reflash_memory_current_page = REFLASH_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
				header_package.reflash_total_log_cnt = kk;
				header_package.fix_memory_current_page = FIX_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
				header_package.fix_total_log_cnt = kk;
				app_flash_sector_header_save(&header_package);
				kk +=1;
			} 
			if(!strcmp((char *)usart_buf.RxBuff, "head load\r\n")){  
				app_flash_sector_header_load(&header_package);		
			} 
			if(!strcmp((char *)usart_buf.RxBuff, "data save reflash\r\n")){  
				app_flash_page_data_save(SMP_REFLASH_MEMORY);
			} 
			if(!strcmp((char *)usart_buf.RxBuff, "data save fix\r\n")){  
				app_flash_page_data_save(SMP_FIX_MEMORY);
			} 
			uint8_t temp[18];
			uint8_t temp2[4];
			uint8_t temp4[4];
			uint16_t start_package;
			uint16_t length_data;
			memcpy(temp,&usart_buf.RxBuff[0],18);
			if(!strcmp((char *)temp, "data load reflash ")){ 	//data load reflash xxxx xxxx
				memcpy(temp2,&usart_buf.RxBuff[18],4);
				memcpy(temp4,&usart_buf.RxBuff[23],4);
				start_package =  a2i((char *)temp2);
				length_data =  a2i((char *)temp4);
				memset(&page_data_buffer[0], 0, 256);
				app_flash_page_data_load(page_data_buffer,start_package,length_data,SMP_REFLASH_MEMORY);
			} 
			if(!strcmp((char *)temp, "data push reflash ")){  
				smp_log_package log_package;
				memcpy(temp2,&usart_buf.RxBuff[18],4);
				length_data =  a2i((char *)temp2);
				for(int i = 0; i < length_data;i++){
					log_package.ID = 0xaa;
					log_package.SMP_RTC[0] = i;
					log_package.SMP_RTC[1] = 0;
					log_package.SMP_RTC[2] = 0;
					log_package.SMP_RTC[3] = 0;
					log_package.data[0] = 0x12;
					log_package.data[1] = 0x34;
					log_package.sum = 0xa5;
					app_flash_page_data_push(log_package,SMP_REFLASH_MEMORY);
				}
			} 
			uint8_t temp3[14];
			memcpy(temp3,&usart_buf.RxBuff[0],14);
			if(!strcmp((char *)temp3, "data load fix ")){ 	//data load fix xxxx xxxx
				memcpy(temp2,&usart_buf.RxBuff[14],4);
				memcpy(temp4,&usart_buf.RxBuff[19],4);
				start_package =  a2i((char *)temp2);
				length_data =  a2i((char *)temp4);
				memset(&page_data_buffer[0], 0, 256);
				app_flash_page_data_load(page_data_buffer,start_package,length_data,SMP_FIX_MEMORY);
			} 
			if(!strcmp((char *)temp3, "data push fix ")){  
				smp_log_package log_package;
				memcpy(temp2,&usart_buf.RxBuff[14],4);
				length_data =  a2i((char *)temp2);
				for(int i = 0; i < length_data;i++){
					log_package.ID = 0xaa;
					log_package.SMP_RTC[0] = i;
					log_package.SMP_RTC[1] = 0;
					log_package.SMP_RTC[2] = 0;
					log_package.SMP_RTC[3] = 0;
					log_package.data[0] = 0x12;
					log_package.data[1] = 0x34;
					log_package.sum = 0xa5;
					app_flash_page_data_push(log_package,SMP_FIX_MEMORY);
				}
			} 
			memset(usart_buf.RxBuff,0,256);
			usart_buf.RxSize = 0;
			usart_buf.Rx_end_flag = 0;
			
		}else{
			usart_buf.RxBuff[usart_buf.RxSize] = usart_buf.aRxBuff;
			usart_buf.RxSize++;
		}
		HAL_UART_Receive_IT(huart, &usart_buf.aRxBuff, 1);
	}
}
#endif 

//---------------------------------------------------------------------------------------
// MCU UART3 Input char 'A'~'D' to test event log function api.
void test_uart_rx_process(void)
{
static uint8_t rx_data = 0;
static int8_t fifo_res;
	
	if(Davinci_uart_rx_cnt>0){
		fifo_res = smp_uart_get(&mDavinci_uart, &rx_data);
		--Davinci_uart_rx_cnt;
		if(fifo_res == SMP_SUCCESS)
		{	
      switch(rx_data){
			case 'A':	
				LOG_YELLOW("EVENT_LOG Clean ALL Memory\r\n");
				app_flash_log_managment_clean_all_memory();
			  break; 
			case 'B':
				LOG_YELLOW("EVENT_LOG Clean REFLASH Memory\r\n");
				app_flash_log_managment_clean_reflash_memory();
			  break;
			case 'C':		
				LOG_YELLOW("EVENT_LOG Clean FIX Memory\r\n");
				app_flash_log_managment_clean_fix_memory();
			  break;
			case 'D':
				LOG_YELLOW("EVENT_LOG Clean HEAD\r\n");
				app_flash_log_managment_clean_head();
			  break;
		  }
		}
	}
}

void app_flash_log_event_handler(smp_log_evt_type p_evt)
{
	switch(p_evt){
		case SMP_LOG_EVENT_SECTOR_HEADER_LOAD_DONE:
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
		break;
		case SMP_LOG_EVENT_SECTOR_HEADER_SAVE_DONE:
				LOG_CYAN("save head\r\n");
				LOG_CYAN("header %x\r\n",header_package.header[0]);
				LOG_CYAN("header %x\r\n",header_package.header[1]);
				LOG_CYAN("header %x\r\n",header_package.header[2]);
				LOG_CYAN("header %x\r\n",header_package.header[3]);
				LOG_CYAN("reflash head page%x\r\n",header_package.reflash_memory_head_page);
				LOG_CYAN("reflash cnt%x\r\n",header_package.reflash_memory_current_page);
				LOG_CYAN("reflash cnt%d\r\n",header_package.reflash_total_log_cnt);
				LOG_CYAN("fix curent page%x\r\n",header_package.fix_memory_current_page);
				LOG_CYAN("fix cnt%d\r\n",header_package.fix_total_log_cnt);
		break;
		case SMP_LOG_EVENT_PAGE_LOAD_DONE:
				LOG_CYAN("page load\r\n");
				for(int i = 0; i < 256;i++){				
					LOG_CYAN("%d,%x\r\n",i,page_data_buffer[i]);
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
	
	uint16_t test_event_log_cnt = 0;
	
	app_flash_log_managment_init(app_flash_log_event_handler);
	HAL_Delay(1000);
	while(test_event_log_cnt<50)
	{
		
	  smp_mx25l_flash_read_status(&mx251_status);
		if((mx251_status.status1&STATUS_WRITE_IN_PROGRESS)==0){		
			 MX25L_SPI_send_command();
		}
    
		test_uart_rx_process();
		
		test_event_log_cnt++;
		HAL_Delay(100);
		LOG_WHITE("Event log#%d transfer\r\n", test_event_log_cnt);
	}
}

//--------FILE END------------------------------------------------------------------------------------------

/**
  ******************************************************************************
  * @file    Test_MX25L_Driver.c
  * @author  Golden Chen
  * @version V0.0.1
  * @date    2022/02/11
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

#include "smp_MX25L_Driver.h"
#include "Test_MX25L_Driver.h"
#include "Bsp.h"
#include "smp_debug.h"
#include "smp_gpio.h"
#include "smp_fifo_flash.h"
#include <string.h>	
#include "SEGGER_RTT.h"
#include "RTT_Log.h"
void flash_event_handler(smp_flash_evt_type p_evt);
uint8_t write_buffer[256]={0};
uint8_t read_buffer[3];
void Test_Event_MX25L_Driver(void){
  smp_mx25l_status mx251_status;

	write_buffer[0] = 0x12;
	write_buffer[1] = 0x34;
	write_buffer[2] = 0x56;
	smp_mx25l_flash_init();
	HAL_Delay(100);
	smp_mx25l_flash_sector_erase_sectornum(0,flash_event_handler);
	smp_mx25l_flash_fast_read_data_bytes_page(0, read_buffer , 3,flash_event_handler);
	smp_mx25l_flash_page_program(0, write_buffer , 3,flash_event_handler);
	smp_mx25l_flash_fast_read_data_bytes_page(0, read_buffer , 3,flash_event_handler);

	while(1)
	{	
		smp_mx25l_flash_read_status(&mx251_status);
		if((mx251_status.status1&STATUS_WRITE_IN_PROGRESS)==0){		
			 smp_mx25l_flash_reset();
			 MX25L_SPI_send_command();
		}
		HAL_Delay(1000);
	}
}

void flash_event_handler(smp_flash_evt_type p_evt)
{

	switch(p_evt){
		case SMP_FLASH_EVENT_READ_DONE:
			SEGGER_RTT_printf(0," %x,%x,%x\r\n" ,read_buffer[0],read_buffer[1],read_buffer[2]);
		break;
		case SMP_FLASH_EVENT_WRITE_DONE:
			SEGGER_RTT_printf(0," %x,%x,%x\r\n" ,read_buffer[0],read_buffer[1],read_buffer[2]);
		break;
		default:
		break;
	
	}

}

//--------FILE END------------------------------------------------------------------------------------------

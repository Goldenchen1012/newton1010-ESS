/**
  ******************************************************************************
  * @file    smp_log_managment.c
  * @author  John Chen
  * @version V0.0.1
  * @date    2022/01/03
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
#include "smp_log_managment.h"
#include <string.h>	
#include "RTT_Log.h"
#ifdef SMP_APP_FM_NOR_FLASH_ENABLE
smp_sector_header_package  g_sector_header_package;

smp_page_header_package g_reflash_page_header_package;
uint8_t log_reflash_package_buffer[LOG_PACKAGE_BUFFER_SIZE];

smp_page_header_package g_fix_page_header_package;
uint8_t log_fix_package_buffer[LOG_PACKAGE_BUFFER_SIZE];

typedef struct{
		uint8_t * RX_buffer;
		uint32_t log_start_addr;
		uint16_t log_length_byte;
		uint16_t read_package_byte;
	}smp_data_load_type;

void app_clean_event_handler(smp_flash_evt_type p_evt);
void app_sector_header_event_handler(smp_flash_evt_type p_evt);
void app_flash_check_reflash_head_event_handler(smp_flash_evt_type p_evt);
void app_flash_check_fix_head_event_handler(smp_flash_evt_type p_evt);
void app_log_event_handler(smp_flash_evt_type p_evt);
static smp_log_event_t log_evt_cb = 0;
smp_sector_header_package* g_sector_header_p;
uint8_t g_sector_header_cnt = 0;
smp_sector_header_package  g_sector_header_package_init;
smp_data_load_type data_load_type;
	
smp_sector_header_package g_sector_header_package_check;
uint8_t check_head_buffer[256];
	
int8_t app_flash_log_managment_init(smp_log_event_t smp_log_event_handle){
	app_flash_sector_header_load(&g_sector_header_package_init);

	log_evt_cb = smp_log_event_handle;

	g_reflash_page_header_package.package_num = 0;
	g_reflash_page_header_package.page_usage_size = 0;
	memset(&log_reflash_package_buffer[0], 0xff, LOG_PACKAGE_BUFFER_SIZE);
	
	g_fix_page_header_package.package_num = 0;
	g_fix_page_header_package.page_usage_size = 0;
	memset(&log_fix_package_buffer[0], 0xff, LOG_PACKAGE_BUFFER_SIZE);
	
	return SMP_SUCCESS;
}

void app_flash_log_managment_clean_all_memory(void){
	app_flash_log_managment_clean_reflash_memory();
	app_flash_log_managment_clean_fix_memory();
}

void app_flash_log_managment_clean_head(void){
	g_sector_header_package.header[0] = 'S';
	g_sector_header_package.header[1] = 'M';
	g_sector_header_package.header[2] = 'P';
	g_sector_header_package.header[3] = 'S';//S : sector
	g_sector_header_package.reflash_memory_head_page = REFLASH_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
	g_sector_header_package.reflash_memory_current_page = REFLASH_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;	
	g_sector_header_package.reflash_total_log_cnt = 0x0;
	g_sector_header_package.fix_memory_current_page = FIX_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
	g_sector_header_package.fix_total_log_cnt = 0x0;
	g_sector_header_package.sum = g_sector_header_package.header[0] + 
																g_sector_header_package.header[1] + 
																g_sector_header_package.header[2] + 
																g_sector_header_package.header[3] + 
																g_sector_header_package.reflash_memory_head_page + 
																g_sector_header_package.reflash_memory_current_page + 
																g_sector_header_package.reflash_total_log_cnt +
																g_sector_header_package.fix_memory_current_page +     
																g_sector_header_package.fix_total_log_cnt;
	g_sector_header_cnt = 0;
	smp_mx25l_flash_sector_erase_sectornum(HEADER_SECTOR_MEMORY_START_SECTOR,app_clean_event_handler);
}

void app_flash_log_managment_clean_reflash_memory(void){
	int i ;
	for(i = REFLASH_MEMORY_START_SECTOR ; i<=REFLASH_MEMORY_END_SECTOR ;i++){
		smp_mx25l_flash_sector_erase_sectornum(i,app_clean_event_handler);
	}
	g_sector_header_package.reflash_memory_head_page = REFLASH_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
	g_sector_header_package.reflash_memory_current_page = REFLASH_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
	g_sector_header_package.reflash_total_log_cnt = 0x0;
	
	///page head
	memset(&log_reflash_package_buffer[0], 0xff, LOG_PACKAGE_BUFFER_SIZE);
	g_reflash_page_header_package.page_usage_size = 0;
	g_reflash_page_header_package.package_num = 0;		
	
	app_flash_log_managment_clean_head();
}

void app_flash_log_managment_clean_fix_memory(void){
	int i ;
	for(i = FIX_MEMORY_START_SECTOR ; i<=FIX_MEMORY_END_SECTOR ;i++){
		smp_mx25l_flash_sector_erase_sectornum(i,app_clean_event_handler);
	}
	g_sector_header_package.fix_memory_current_page = FIX_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
	g_sector_header_package.fix_total_log_cnt = 0x0;
	
	///page head
	memset(&log_fix_package_buffer[0], 0xff, LOG_PACKAGE_BUFFER_SIZE);
	g_fix_page_header_package.page_usage_size = 0;
	g_fix_page_header_package.package_num = 0;		
	
	app_flash_log_managment_clean_head();
}

void app_flash_sector_header_save(smp_sector_header_package * sector_header){
	if((g_sector_header_cnt ==0)||(g_sector_header_cnt >= PAGE_NUM_IN_SECTOR)){
		smp_mx25l_flash_sector_erase_sectornum(HEADER_SECTOR_MEMORY_START_SECTOR,app_sector_header_event_handler);
		g_sector_header_cnt = 0;
	}
	sector_header->sum = sector_header->header[0] + 
											 sector_header->header[1] + 
											 sector_header->header[2] + 
											 sector_header->header[3] + 
											 sector_header->reflash_memory_head_page + 
											 sector_header->reflash_memory_current_page + 
											 sector_header->reflash_total_log_cnt +
											 sector_header->fix_memory_current_page +     
											 sector_header->fix_total_log_cnt;
	smp_mx25l_flash_page_program(HEADER_SECTOR_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR + g_sector_header_cnt,(uint8_t*)sector_header,sizeof(smp_sector_header_package),app_sector_header_event_handler);
	g_sector_header_cnt++;
}

void app_flash_sector_header_load(smp_sector_header_package * sector_header){
	g_sector_header_p = sector_header;
	g_sector_header_cnt = 0;
	smp_mx25l_flash_fast_read_data_bytes_page(HEADER_SECTOR_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR,(uint8_t*)sector_header,sizeof(smp_sector_header_package),app_sector_header_event_handler);
}

void app_flash_sector_header_get(smp_sector_header_package * sector_header){
	memcpy(sector_header,&g_sector_header_package,sizeof(smp_sector_header_package));
	sector_header->reflash_total_log_cnt += g_reflash_page_header_package.package_num;
	sector_header->fix_total_log_cnt += g_fix_page_header_package.package_num;
	if(sector_header->reflash_total_log_cnt > LOG_NUM_IN_PAGE*PAGE_NUM_IN_SECTOR*REFLASH_MEMORY_SECTOR_SIZE){
		g_sector_header_package.reflash_total_log_cnt -= LOG_NUM_IN_PAGE * PAGE_NUM_IN_SECTOR;
	}
	if(sector_header->fix_total_log_cnt > LOG_NUM_IN_PAGE*PAGE_NUM_IN_SECTOR*FIX_MEMORY_SECTOR_SIZE){
		g_sector_header_package.fix_total_log_cnt -= LOG_NUM_IN_PAGE * PAGE_NUM_IN_SECTOR;
	}
}

void app_flash_check_head(void){
	memset(check_head_buffer, 0, 256);
	g_sector_header_package_check.header[0] = 'S';
	g_sector_header_package_check.header[1] = 'M';
	g_sector_header_package_check.header[2] = 'P';
	g_sector_header_package_check.header[3] = 'S';//S : sector
	g_sector_header_package_check.reflash_memory_head_page = g_sector_header_package.reflash_memory_head_page;
	g_sector_header_package_check.reflash_memory_current_page = g_sector_header_package.reflash_memory_head_page;	
	g_sector_header_package_check.reflash_total_log_cnt = 0x0;
	g_sector_header_package_check.fix_memory_current_page = FIX_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
	g_sector_header_package_check.fix_total_log_cnt = 0x0;

	smp_mx25l_flash_fast_read_data_bytes_page(g_sector_header_package_check.reflash_memory_current_page, check_head_buffer, PAGE_SIZE, app_flash_check_reflash_head_event_handler);
}

void app_flash_page_data_push(smp_log_package log_package,smp_flash_type flash_type){
	switch(flash_type){
		case SMP_REFLASH_MEMORY:
			LOG_YELLOW("data push %d,%x\r\n",g_reflash_page_header_package.page_usage_size,g_sector_header_package.reflash_memory_current_page);
			if((g_reflash_page_header_package.page_usage_size + LOG_PACKAGE_SIZE) > LOG_PACKAGE_BUFFER_SIZE){
				///page save
				app_flash_page_data_save(SMP_REFLASH_MEMORY);
				///copy to buffer
				memcpy(&log_reflash_package_buffer[g_reflash_page_header_package.page_usage_size], &log_package, LOG_PACKAGE_SIZE);
				g_reflash_page_header_package.page_usage_size += LOG_PACKAGE_SIZE;
				g_reflash_page_header_package.package_num += 1;		
			}else if ((g_reflash_page_header_package.page_usage_size + LOG_PACKAGE_SIZE) == LOG_PACKAGE_BUFFER_SIZE){
				///copy to buffer
				memcpy(&log_reflash_package_buffer[g_reflash_page_header_package.page_usage_size], &log_package, LOG_PACKAGE_SIZE);
				g_reflash_page_header_package.page_usage_size += LOG_PACKAGE_SIZE;
				g_reflash_page_header_package.package_num += 1;
				///page save
				app_flash_page_data_save(SMP_REFLASH_MEMORY);
			}else{
				memcpy(&log_reflash_package_buffer[g_reflash_page_header_package.page_usage_size], &log_package, LOG_PACKAGE_SIZE);
				g_reflash_page_header_package.page_usage_size += LOG_PACKAGE_SIZE;
				g_reflash_page_header_package.package_num += 1;	
			}
		break;
		case SMP_FIX_MEMORY:
			LOG_YELLOW("data push %d,%x\r\n",g_fix_page_header_package.page_usage_size,g_sector_header_package.fix_memory_current_page);
			if(g_sector_header_package.fix_memory_current_page % PAGE_NUM_IN_SECTOR == 0){
				if(g_sector_header_package.fix_memory_current_page >= ((FIX_MEMORY_END_SECTOR + 1) * PAGE_NUM_IN_SECTOR)){
					log_evt_cb(SMP_LOG_EVENT_MEMORY_FULL);
					return;
				}			
			}
			if((g_fix_page_header_package.page_usage_size + LOG_PACKAGE_SIZE) > LOG_PACKAGE_BUFFER_SIZE){
				///page save
				app_flash_page_data_save(SMP_FIX_MEMORY);
				///copy to buffer
				memcpy(&log_fix_package_buffer[g_fix_page_header_package.page_usage_size], &log_package, LOG_PACKAGE_SIZE);
				g_fix_page_header_package.page_usage_size += LOG_PACKAGE_SIZE;
				g_fix_page_header_package.package_num += 1;		
			}else if ((g_fix_page_header_package.page_usage_size + LOG_PACKAGE_SIZE) == LOG_PACKAGE_BUFFER_SIZE){
				///copy to buffer
				memcpy(&log_fix_package_buffer[g_fix_page_header_package.page_usage_size], &log_package, LOG_PACKAGE_SIZE);
				g_fix_page_header_package.page_usage_size += LOG_PACKAGE_SIZE;
				g_fix_page_header_package.package_num += 1;
				///page save
				app_flash_page_data_save(SMP_FIX_MEMORY);
			}else{
				memcpy(&log_fix_package_buffer[g_fix_page_header_package.page_usage_size], &log_package, LOG_PACKAGE_SIZE);
				g_fix_page_header_package.page_usage_size += LOG_PACKAGE_SIZE;
				g_fix_page_header_package.package_num += 1;				
			}
		break;

		default:
		break;
	}
}

void app_flash_page_data_save(smp_flash_type flash_type){
	switch(flash_type){
		case SMP_REFLASH_MEMORY:
			if(g_sector_header_package.reflash_memory_current_page % PAGE_NUM_IN_SECTOR == 0){
				if(g_sector_header_package.reflash_memory_current_page >= ((REFLASH_MEMORY_END_SECTOR + 1) * PAGE_NUM_IN_SECTOR)){
					g_sector_header_package.reflash_memory_current_page = REFLASH_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
					g_sector_header_package.reflash_memory_head_page = (REFLASH_MEMORY_START_SECTOR + 1) * PAGE_NUM_IN_SECTOR;
					smp_mx25l_flash_sector_erase_sectornum(g_sector_header_package.reflash_memory_current_page / PAGE_NUM_IN_SECTOR,app_log_event_handler);
					LOG_YELLOW("sector erase %x\r\n",g_sector_header_package.reflash_memory_current_page / PAGE_NUM_IN_SECTOR);
					g_sector_header_package.reflash_total_log_cnt -= LOG_NUM_IN_PAGE * PAGE_NUM_IN_SECTOR;
				}
			}
			if((g_sector_header_package.reflash_memory_current_page == g_sector_header_package.reflash_memory_head_page)&&(g_sector_header_package.reflash_memory_head_page !=REFLASH_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR)){
				g_sector_header_package.reflash_memory_head_page += PAGE_NUM_IN_SECTOR;
				smp_mx25l_flash_sector_erase_sectornum(g_sector_header_package.reflash_memory_current_page / PAGE_NUM_IN_SECTOR,app_log_event_handler);
				LOG_YELLOW("sector erase %x\r\n",g_sector_header_package.reflash_memory_current_page / PAGE_NUM_IN_SECTOR);
				g_sector_header_package.reflash_total_log_cnt -= LOG_NUM_IN_PAGE * PAGE_NUM_IN_SECTOR;
				if(g_sector_header_package.reflash_memory_head_page >= ((REFLASH_MEMORY_END_SECTOR + 1) * PAGE_NUM_IN_SECTOR)){
					g_sector_header_package.reflash_memory_head_page = REFLASH_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
				}
			}
			if(g_reflash_page_header_package.package_num < LOG_NUM_IN_PAGE){
				g_sector_header_package.reflash_total_log_cnt += g_reflash_page_header_package.package_num;
				app_flash_sector_header_save(&g_sector_header_package);
				smp_mx25l_flash_page_program(g_sector_header_package.reflash_memory_current_page,(uint8_t*)log_reflash_package_buffer,LOG_PACKAGE_BUFFER_SIZE,app_log_event_handler);
				LOG_YELLOW("page program %x\r\n",g_sector_header_package.reflash_memory_current_page);
				memset(&log_reflash_package_buffer[0], 0xff, LOG_PACKAGE_BUFFER_SIZE);
				g_reflash_page_header_package.package_num = 0;	
			}else{
				g_sector_header_package.reflash_memory_current_page += 1;
				g_sector_header_package.reflash_total_log_cnt += g_reflash_page_header_package.package_num;
				app_flash_sector_header_save(&g_sector_header_package);
				
				smp_mx25l_flash_page_program(g_sector_header_package.reflash_memory_current_page - 1,(uint8_t*)log_reflash_package_buffer,LOG_PACKAGE_BUFFER_SIZE,app_log_event_handler);
				LOG_YELLOW("page program %x\r\n",g_sector_header_package.reflash_memory_current_page);
					
				memset(&log_reflash_package_buffer[0], 0xff, LOG_PACKAGE_BUFFER_SIZE);
				g_reflash_page_header_package.page_usage_size = 0;
				g_reflash_page_header_package.package_num = 0;	
			}
		break;
		case SMP_FIX_MEMORY:
			if(g_sector_header_package.fix_memory_current_page % PAGE_NUM_IN_SECTOR == 0){
				if(g_sector_header_package.fix_memory_current_page >= ((FIX_MEMORY_END_SECTOR + 1) * PAGE_NUM_IN_SECTOR)){
					log_evt_cb(SMP_LOG_EVENT_MEMORY_FULL);
					return;
				}			
			}
			if(g_fix_page_header_package.package_num < LOG_NUM_IN_PAGE){
				g_sector_header_package.fix_total_log_cnt += g_fix_page_header_package.package_num;
				app_flash_sector_header_save(&g_sector_header_package);		
				smp_mx25l_flash_page_program(g_sector_header_package.fix_memory_current_page - 1,(uint8_t*)log_fix_package_buffer,LOG_PACKAGE_BUFFER_SIZE,app_log_event_handler);
				LOG_YELLOW("page program %d\r\n",g_sector_header_package.fix_memory_current_page);						
				memset(&log_fix_package_buffer[0], 0xff, LOG_PACKAGE_BUFFER_SIZE);
				g_fix_page_header_package.package_num = 0;			
			}else{
				g_sector_header_package.fix_memory_current_page += 1;
				g_sector_header_package.fix_total_log_cnt += g_fix_page_header_package.package_num;
				app_flash_sector_header_save(&g_sector_header_package);
				
				smp_mx25l_flash_page_program(g_sector_header_package.fix_memory_current_page - 1,(uint8_t*)log_fix_package_buffer,LOG_PACKAGE_BUFFER_SIZE,app_log_event_handler);
				LOG_YELLOW("page program %d\r\n",g_sector_header_package.fix_memory_current_page);
							
				memset(&log_fix_package_buffer[0], 0xff, LOG_PACKAGE_BUFFER_SIZE);
				g_fix_page_header_package.page_usage_size = 0;
				g_fix_page_header_package.package_num = 0;			
			}				
		break;

		default:
		break;
	}
}


void app_flash_page_data_load(uint8_t * RX_buffer , uint16_t log_start_position, uint16_t log_length,smp_flash_type flash_type){
	uint32_t temp_addr;
	uint8_t addr[3];
	if(log_length > 32){
		log_evt_cb(SMP_LOG_EVENT_ERROR);
		return;
	}
	data_load_type.RX_buffer = RX_buffer;
	data_load_type.log_length_byte = log_length* LOG_PACKAGE_SIZE;
	data_load_type.read_package_byte = 0;
	switch(flash_type){
		case SMP_REFLASH_MEMORY:
			if(log_start_position + log_length > (g_sector_header_package.reflash_total_log_cnt + g_reflash_page_header_package.package_num)){
				log_evt_cb(SMP_LOG_EVENT_ERROR);
				return;
			}
			if(g_reflash_page_header_package.package_num > 0){
				app_flash_page_data_save(SMP_REFLASH_MEMORY);
			}
			temp_addr =  (g_sector_header_package.reflash_memory_head_page << MX25L_MX25L6433F_PAGE_SHIFT) + log_start_position * LOG_PACKAGE_SIZE;
			if(temp_addr >= ((REFLASH_MEMORY_END_SECTOR  + 1) * PAGE_NUM_IN_SECTOR) << MX25L_MX25L6433F_PAGE_SHIFT){
					temp_addr -= (REFLASH_MEMORY_SECTOR_SIZE * PAGE_NUM_IN_SECTOR) << MX25L_MX25L6433F_PAGE_SHIFT;
			}
			data_load_type.log_start_addr = temp_addr;
			addr[0] = (uint8_t)(temp_addr >> 16) ;
			addr[1] = (uint8_t)(temp_addr >> 8) ;
			addr[2] = (uint8_t)(temp_addr);
			LOG_YELLOW("addr load %x %x\r\n",temp_addr,data_load_type.log_length_byte);
			smp_mx25l_flash_fast_read_data_bytes_addr(addr,(uint8_t*)RX_buffer,data_load_type.log_length_byte,app_log_event_handler);
		break;
		case SMP_FIX_MEMORY:
			if(log_start_position + log_length > (g_sector_header_package.fix_total_log_cnt + g_fix_page_header_package.package_num)){
				log_evt_cb(SMP_LOG_EVENT_ERROR);
				return;
			}
			if(g_fix_page_header_package.package_num > 0){
				app_flash_page_data_save(SMP_FIX_MEMORY);
			}
			temp_addr =  ((FIX_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR) << MX25L_MX25L6433F_PAGE_SHIFT) + log_start_position * LOG_PACKAGE_SIZE;
			if(temp_addr >= ((FIX_MEMORY_END_SECTOR  + 1) * PAGE_NUM_IN_SECTOR) << MX25L_MX25L6433F_PAGE_SHIFT){
					log_evt_cb(SMP_LOG_EVENT_ERROR);
					return;
			}
			data_load_type.log_start_addr = temp_addr;
			addr[0] = (uint8_t)(temp_addr >> 16) ;
			addr[1] = (uint8_t)(temp_addr >> 8) ;
			addr[2] = (uint8_t)(temp_addr);
			LOG_YELLOW("addr load %x %x\r\n",temp_addr,data_load_type.log_length_byte);
			smp_mx25l_flash_fast_read_data_bytes_addr(addr,(uint8_t*)RX_buffer,data_load_type.log_length_byte,app_log_event_handler);

		break;

		default:
		break;
	}

}

void app_sector_header_event_handler(smp_flash_evt_type p_evt)
{
	uint16_t temp_check_sum;
	switch(p_evt){
		case SMP_FLASH_EVENT_READ_DONE:
			temp_check_sum = g_sector_header_p->header[0] + 
															 g_sector_header_p->header[1] + 
															 g_sector_header_p->header[2] + 
															 g_sector_header_p->header[3] + 
															 g_sector_header_p->reflash_memory_head_page + 
															 g_sector_header_p->reflash_memory_current_page + 
															 g_sector_header_p->reflash_total_log_cnt +
															 g_sector_header_p->fix_memory_current_page +     
															 g_sector_header_p->fix_total_log_cnt;
				if((g_sector_header_p->header[0] == 'S')&&(g_sector_header_p->header[1] == 'M')&&(g_sector_header_p->header[2] == 'P')&&(g_sector_header_p->sum == temp_check_sum)){
				g_sector_header_cnt += 1;
				memcpy(&g_sector_header_package,g_sector_header_p,sizeof(smp_sector_header_package));
				//printf("init aa%d\r\n",g_sector_header_cnt);
				if(g_sector_header_cnt < PAGE_NUM_IN_SECTOR){
					smp_mx25l_flash_fast_read_data_bytes_page(HEADER_SECTOR_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR + g_sector_header_cnt,(uint8_t*)g_sector_header_p,sizeof(smp_sector_header_package),app_sector_header_event_handler);
					//printf("init bb%d\r\n",g_sector_header_cnt);
				}else{
					app_flash_check_head();
					//log_evt_cb(SMP_LOG_EVENT_SECTOR_HEADER_LOAD_DONE);///done
				}		
			}else{
				if((g_sector_header_package.header[0] != 'S')||(g_sector_header_package.header[1] != 'M')||(g_sector_header_package.header[2] != 'P')){
					g_sector_header_package.header[0] = 'S';
					g_sector_header_package.header[1] = 'M';
					g_sector_header_package.header[2] = 'P';
					g_sector_header_package.header[3] = 'S';//S : sector
					g_sector_header_package.reflash_memory_head_page = REFLASH_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
					g_sector_header_package.reflash_memory_current_page = REFLASH_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;	
					g_sector_header_package.reflash_total_log_cnt = 0x0;
					g_sector_header_package.fix_memory_current_page = FIX_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
					g_sector_header_package.fix_total_log_cnt = 0x0;
					g_sector_header_package.sum = g_sector_header_package.header[0] + 
												 g_sector_header_package.header[1] + 
												 g_sector_header_package.header[2] + 
												 g_sector_header_package.header[3] + 
												 g_sector_header_package.reflash_memory_head_page + 
												 g_sector_header_package.reflash_memory_current_page + 
												 g_sector_header_package.reflash_total_log_cnt +
												 g_sector_header_package.fix_memory_current_page +     
												 g_sector_header_package.fix_total_log_cnt;
					//printf("init %d\r\n",g_sector_header_package.reflash_memory_current_page);
					
				}
				g_reflash_page_header_package.package_num = 0;
				g_reflash_page_header_package.page_usage_size = (g_sector_header_package.reflash_total_log_cnt % (LOG_PACKAGE_BUFFER_SIZE/LOG_PACKAGE_SIZE)) * LOG_PACKAGE_SIZE;
				g_fix_page_header_package.package_num = 0;
				g_fix_page_header_package.page_usage_size = (g_sector_header_package.fix_total_log_cnt % (LOG_PACKAGE_BUFFER_SIZE/LOG_PACKAGE_SIZE)) * LOG_PACKAGE_SIZE;

				if(g_reflash_page_header_package.page_usage_size != 0){
					g_sector_header_package.reflash_memory_current_page -= 1;
				}
				if(g_fix_page_header_package.page_usage_size != 0){
					g_sector_header_package.fix_memory_current_page -= 1;
				}
				memcpy(g_sector_header_p,&g_sector_header_package,sizeof(smp_sector_header_package));
				app_flash_check_head();
				//log_evt_cb(SMP_LOG_EVENT_SECTOR_HEADER_LOAD_DONE);///done
			}
		
			break;
		case SMP_FLASH_EVENT_WRITE_DONE:
			
			break;
		default:
			break;
	
	}

}

void app_flash_check_reflash_head_event_handler(smp_flash_evt_type p_evt)
{
	int i;
	switch(p_evt){
		case SMP_FLASH_EVENT_READ_DONE:
			for(i = LOG_PACKAGE_SIZE - 1; i < PAGE_SIZE;i = i + LOG_PACKAGE_SIZE){
				if(check_head_buffer[i] ==  0xff){
					smp_mx25l_flash_fast_read_data_bytes_page(g_sector_header_package_check.fix_memory_current_page, check_head_buffer, PAGE_SIZE, app_flash_check_fix_head_event_handler);
					return;///done
				}else if(check_head_buffer[i] ==  0xa5){
					g_sector_header_package_check.reflash_total_log_cnt++;
				}
			}
			g_sector_header_package_check.reflash_memory_current_page++;
			if(g_sector_header_package_check.reflash_memory_current_page >= ((REFLASH_MEMORY_END_SECTOR + 1) * PAGE_NUM_IN_SECTOR)){
					g_sector_header_package_check.reflash_memory_current_page = REFLASH_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
			}
			if(g_sector_header_package_check.reflash_memory_current_page == g_sector_header_package_check.reflash_memory_head_page)
			{
				smp_mx25l_flash_fast_read_data_bytes_page(g_sector_header_package_check.fix_memory_current_page, check_head_buffer, PAGE_SIZE, app_flash_check_fix_head_event_handler);
				return;///done
			}
			memset(check_head_buffer, 0, 256);
			smp_mx25l_flash_fast_read_data_bytes_page(g_sector_header_package_check.reflash_memory_current_page, check_head_buffer, PAGE_SIZE, app_flash_check_reflash_head_event_handler);
			break;
		case SMP_FLASH_EVENT_WRITE_DONE:
			
			break;
		default:
			break;
	}
}

void app_flash_check_fix_head_event_handler(smp_flash_evt_type p_evt)
{
	int i;
	switch(p_evt){
		case SMP_FLASH_EVENT_READ_DONE:
			for(i = LOG_PACKAGE_SIZE - 1; i < PAGE_SIZE;i = i + LOG_PACKAGE_SIZE){
				if(check_head_buffer[i] ==  0xff){
					g_sector_header_package_check.sum = g_sector_header_package_check.header[0] + 
																							g_sector_header_package_check.header[1] + 
																							g_sector_header_package_check.header[2] + 
																							g_sector_header_package_check.header[3] + 
																							g_sector_header_package_check.reflash_memory_head_page + 
																							g_sector_header_package_check.reflash_memory_current_page + 
																							g_sector_header_package_check.reflash_total_log_cnt +
																							g_sector_header_package_check.fix_memory_current_page +     
																							g_sector_header_package_check.fix_total_log_cnt;
					memcpy(&g_sector_header_package,&g_sector_header_package_check,sizeof(smp_sector_header_package));
					LOG_CYAN("reflash head page%x\r\n",g_sector_header_package_check.reflash_memory_head_page);
					LOG_CYAN("reflash current page%x\r\n",g_sector_header_package_check.reflash_memory_current_page);
					LOG_CYAN("reflash cnt%d\r\n",g_sector_header_package_check.reflash_total_log_cnt);
					LOG_CYAN("fix curent page%x\r\n",g_sector_header_package_check.fix_memory_current_page);
					LOG_CYAN("fix cnt%d\r\n",g_sector_header_package_check.fix_total_log_cnt);
					log_evt_cb(SMP_LOG_EVENT_SECTOR_HEADER_LOAD_DONE);
					return;///done
				}else if(check_head_buffer[i] ==  0xa5){
					g_sector_header_package_check.fix_total_log_cnt++;
				}
			}
			g_sector_header_package_check.fix_memory_current_page++;
			if(g_sector_header_package_check.fix_memory_current_page >= ((FIX_MEMORY_END_SECTOR + 1) * PAGE_NUM_IN_SECTOR)){
					LOG_CYAN("reflash head page%x\r\n",g_sector_header_package_check.reflash_memory_head_page);
					LOG_CYAN("reflash current page%x\r\n",g_sector_header_package_check.reflash_memory_current_page);
					LOG_CYAN("reflash cnt%d\r\n",g_sector_header_package_check.reflash_total_log_cnt);
					LOG_CYAN("fix curent page%x\r\n",g_sector_header_package_check.fix_memory_current_page);
					LOG_CYAN("fix cnt%d\r\n",g_sector_header_package_check.fix_total_log_cnt);
					log_evt_cb(SMP_LOG_EVENT_SECTOR_HEADER_LOAD_DONE);
					return;///done
			}

			memset(check_head_buffer, 0, 256);
			smp_mx25l_flash_fast_read_data_bytes_page(g_sector_header_package_check.fix_memory_current_page, check_head_buffer, PAGE_SIZE, app_flash_check_fix_head_event_handler);
			break;
		case SMP_FLASH_EVENT_WRITE_DONE:
			
			break;
		default:
			break;
	}
}

void app_log_event_handler(smp_flash_evt_type p_evt)
{
	
	switch(p_evt){
		case SMP_FLASH_EVENT_READ_DONE:
			log_evt_cb(SMP_LOG_EVENT_PAGE_LOAD_DONE);
			break;
		case SMP_FLASH_EVENT_WRITE_DONE:
			log_evt_cb(SMP_LOG_EVENT_PAGE_SAVE_DONE);
			break;
		case SMP_FLASH_EVENT_ERASE_DONE:
		
			break;
		default:
			break;
	
	}

}

void app_clean_event_handler(smp_flash_evt_type p_evt)
{
	switch(p_evt){
		case SMP_FLASH_EVENT_READ_DONE:

			break;
		case SMP_FLASH_EVENT_WRITE_DONE:

			break;
		case SMP_FLASH_EVENT_ERASE_DONE:
		
			break;
		default:
			break;
	
	}

}
#endif



///******************************END OF FILE*****************************/

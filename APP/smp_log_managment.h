/**
  ******************************************************************************
  * @file    smp_log_managment.h
  * @author  John Chen
  * @version V0.0.1
  * @date    2022/01/03
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */
#ifndef __SMP_LOG_MANAGMENT_H
#define __SMP_LOG_MANAGMENT_H

#include <stdint.h>
#include "smp_MX25L_Driver.h"
#include "stm32l4xx_Davinci.h"

#ifdef SMP_APP_FM_NOR_FLASH_ENABLE

	#define RESERVED_MEMORY_START_SECTOR 0
	#define RESERVED_MEMORY_SECTOR_SIZE 128  //512k >> 12 MX25L_MX25L6433F_SECTOR_SHIFT
	#define RESERVED_MEMORY_END_SECTOR (RESERVED_MEMORY_START_SECTOR + RESERVED_MEMORY_SECTOR_SIZE - 1)

	#define HEADER_SECTOR_MEMORY_START_SECTOR (RESERVED_MEMORY_END_SECTOR + 1)
	#define HEADER_SECTOR_MEMORY_SIZE 1
	#define HEADER_SECTOR_MEMORY_END_SECTOR (HEADER_SECTOR_MEMORY_START_SECTOR + HEADER_SECTOR_MEMORY_SIZE - 1)

	#define REFLASH_MEMORY_START_SECTOR (HEADER_SECTOR_MEMORY_END_SECTOR + 1)
	#define REFLASH_MEMORY_SECTOR_SIZE 4
	#define REFLASH_MEMORY_END_SECTOR (REFLASH_MEMORY_START_SECTOR + REFLASH_MEMORY_SECTOR_SIZE - 1)

	#define FIX_MEMORY_START_SECTOR (REFLASH_MEMORY_END_SECTOR + 1)
	#define FIX_MEMORY_SECTOR_SIZE 4
	#define FIX_MEMORY_END_SECTOR (FIX_MEMORY_START_SECTOR + FIX_MEMORY_SECTOR_SIZE - 1)

	#define MEMORY_TOTAL_SECTOR_SIZE MX25L_MX25L6433F_NSECTORS

	#if REFLASH_MEMORY_END_SECTOR >= 2048 // MX25L_MX25L6433F_NSECTORS = 2048 
		#error invild memory size
	#endif

	#if FIX_MEMORY_END_SECTOR >= 2048 // MX25L_MX25L6433F_NSECTORS = 2048 
		#error invild memory size
	#endif
  #define PAGE_NUM_IN_SECTOR 16
	#define LOG_PACKAGE_BUFFER_SIZE 256
	#define PAGE_SIZE 256 ///byte
	#define LOG_PACKAGE_SIZE 8 ///byte
	#define LOG_NUM_IN_PAGE PAGE_SIZE/LOG_PACKAGE_SIZE
	typedef enum{
		SMP_REFLASH_MEMORY = 0,
		SMP_FIX_MEMORY,
	}smp_flash_type;
	
	typedef struct{
		uint8_t header[4];
		uint16_t reflash_memory_head_page;
		uint16_t reflash_memory_current_page;
		uint16_t reflash_total_log_cnt;
		uint16_t fix_memory_current_page;
		uint16_t fix_total_log_cnt;
		uint16_t sum;
	}smp_sector_header_package;
	
	typedef struct{
		uint8_t package_num;
		uint8_t page_usage_size;
	}smp_page_header_package;
	
	typedef struct{
		uint8_t ID;
		uint8_t SMP_RTC[4];
		uint8_t data[2];
		uint8_t sum;
	}smp_log_package;
	
	typedef enum{
		SMP_LOG_EVENT_SECTOR_HEADER_SAVE_DONE = 0,
		SMP_LOG_EVENT_SECTOR_HEADER_LOAD_DONE,
		SMP_LOG_EVENT_PAGE_SAVE_DONE,
		SMP_LOG_EVENT_PAGE_LOAD_DONE,
		SMP_LOG_EVENT_MEMORY_FULL,
		SMP_LOG_EVENT_BUSY,
		SMP_LOG_EVENT_ERROR
	}smp_log_evt_type;
	typedef void (*smp_log_event_t)(smp_log_evt_type p_evt);
	
	
	int8_t app_flash_log_managment_init(smp_log_event_t smp_log_event_handle);
	void app_flash_log_managment_clean_all_memory(void);
	void app_flash_log_managment_clean_head(void);
	void app_flash_log_managment_clean_reflash_memory(void);
	void app_flash_log_managment_clean_fix_memory(void);
	void app_flash_sector_header_save(smp_sector_header_package * sector_header);
	void app_flash_sector_header_load(smp_sector_header_package * sector_header);
	void app_flash_check_head(void);
	void app_flash_page_data_push(smp_log_package log_package,smp_flash_type flash_type);
	void app_flash_page_data_save(smp_flash_type flash_type);
	void app_flash_page_data_load(uint8_t * RX_buffer , uint16_t log_start_position, uint16_t log_length,smp_flash_type flash_type);

#endif


#endif /* __SMP_LOG_MANAGMENT_H */


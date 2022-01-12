/**
  ******************************************************************************
  * @file    smp_MX25L_Driver.h 
  * @author  John Chen
  * @version V0.0.1
  * @date    2021/12/13
  * @brief
  ******************************************************************************
  */
#ifndef __SMP_FLASH_DRIVER_H
#define __SMP_FLASH_DRIVER_H

#include <stdint.h>
#include "smp_spi_DMA.h"
#include "smp_fifo_flash.h"

/* MX25L6433F capacity is 64Mbit  (8192Kbit x 8) =   8Mb (1024kb x 8) */
#define MX25L_MX25L6433F_SECTOR_SHIFT  12    /* Sector size 1 << 12 = 4Kb */
#define MX25L_MX25L6433F_NSECTORS      2048
#define MX25L_MX25L6433F_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */

typedef struct{
	smp_flash_package		*rx_buf;				/* Pointer to the RX buffer */
	uint32_t				rx_buf_size;			/* Size of the RX buffer */
	//smp_flash_package		*tx_buf;				/* Pointer to the TX buffer */
	//uint32_t				tx_buf_size;			/* Size of the TX buffer */
}flash_buffer_t;

typedef struct{
	flash_buffer_t		buffers;			
}smp_flash_t;

typedef struct{
	uint8_t Manufacture_ID;
	uint16_t Device_ID;
}smp_mx25l_ID;

#define STATUS_WRITE_IN_PROGRESS       1<<0
#define STATUS_WRITE_ENABLE_LATCH      1<<1
#define STATUS_LEVEL_OF_PROTECTED      0b1111<<2
#define STATUS_QUAD_ENABLE             1<<6
#define STATUS_REGISTER_WRITE_DISABLE  1<<7
typedef struct{
	uint8_t status1;
	uint8_t status2;
}smp_mx25l_status;

#define CONFIG_ODS                     1<<0
#define CONFIG_TOP_BOTTOM_SELECTED     1<<3
#define CONFIG_DUMMY_CYCLE             1<<6
typedef struct{
	uint8_t config1;
	uint8_t config2;
}smp_mx25l_config;

int8_t smp_mx25l_flash_init(void);
int8_t smp_mx25l_flash_write_enable(void);
int8_t smp_mx25l_flash_write_disable(void);
int8_t smp_mx25l_flash_read_ID(smp_mx25l_ID *mx251_ID);
int8_t smp_mx25l_flash_read_status(smp_mx25l_status *mx251_status);
int8_t smp_mx25l_flash_read_configuration(smp_mx25l_config *mx251_config);
int8_t smp_mx25l_flash_write_status(smp_mx25l_status *mx251_status,smp_mx25l_config *mx251_config);
int8_t smp_mx25l_flash_read_data_bytes(uint8_t *flash_addr,uint8_t *buffer,uint16_t read_byte_num,smp_flash_event_t smp_flash_event_handle);
int8_t smp_mx25l_flash_fast_read_data_bytes_addr(uint8_t *flash_addr ,uint8_t *buffer, uint16_t read_byte_num , smp_flash_event_t smp_flash_event_handle);
int8_t smp_mx25l_flash_fast_read_data_bytes_page(uint16_t page, uint8_t *buffer,uint16_t read_byte_num , smp_flash_event_t smp_flash_event_handle);
int8_t smp_mx25l_flash_sector_erase(uint8_t *flash_addr,smp_flash_event_t smp_flash_event_handle);
int8_t smp_mx25l_flash_sector_erase_sectornum(uint16_t sector_num , smp_flash_event_t smp_flash_event_handle);
int8_t smp_mx25l_flash_block_erase(uint8_t *flash_addr,smp_flash_event_t smp_flash_event_handle);
int8_t smp_mx25l_flash_chip_erase(smp_flash_event_t smp_flash_event_handle);
int8_t smp_mx25l_flash_page_program(uint16_t page,uint8_t *buffer,uint16_t write_byte_num,smp_flash_event_t smp_flash_event_handle);
int8_t smp_mx25l_flash_deep_power_down(void);
int8_t smp_mx25l_flash_release_deep_power_down(void);
void MX25L_SPI_send_command(void);
#endif /* __SPI_FLASH_H */


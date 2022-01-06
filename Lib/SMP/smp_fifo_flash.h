/**
  ******************************************************************************
  * @file    smp_fifo_flash.h 
  * @author  John Chen
  * @version V0.0.1
  * @date    2021/11/24
  * @brief   Header for smp_fifo_flash.c 
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMP_FIFO_FLASH_H
#define __SMP_FIFO_FLASH_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
typedef enum{
	SMP_FLASH_EVENT_READ_DONE = 0,
	SMP_FLASH_EVENT_WRITE_DONE,
	SMP_FLASH_EVENT_ERASE_DONE,
	SMP_FLASH_EVENT_BUSY,
	SMP_FLASH_EVENT_ERROR
}smp_flash_evt_type;
typedef void (*smp_flash_event_t)(smp_flash_evt_type p_evt);

typedef struct{
	uint8_t	command;
	uint8_t	addr[3];
	uint8_t	Dummy;
	uint16_t  R_W_bytes;
	uint8_t page_buffer[256];
	uint8_t * read_buffer;
	smp_flash_event_t flash_callback;
}smp_flash_package;

typedef struct{
	smp_flash_package		*buffer_addr;	/* FIFO queue buffer start address */
	uint16_t	buffer_size;	/* FIFO queue buffer size */
	uint32_t	in;				/* FIFO queue buffer push in address */
	uint32_t	out;			/* FIFO queue buffer pop out address */
}smp_fifo_flash_t;

/* Exported constants --------------------------------------------------------*/ 
/* Exported defines --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int8_t smp_fifo_flash_open(smp_fifo_flash_t *p_fifo);
int8_t smp_fifo_flash_push(smp_fifo_flash_t *p_fifo, smp_flash_package *pFlashPkg);
int8_t smp_fifo_flash_pop(smp_fifo_flash_t *p_fifo, smp_flash_package *pFlashPkg);
int8_t smp_fifo_flash_read(smp_fifo_flash_t *p_fifo, smp_flash_package *pFlashPkg);
int8_t smp_fifo_flash_back(smp_fifo_flash_t *p_fifo);
int8_t smp_fifo_flash_check(smp_fifo_flash_t *p_fifo, char *val, uint32_t index);
int8_t smp_fifo_flash_get_size(smp_fifo_flash_t *p_fifo, uint16_t *size);
int8_t smp_fifo_flash_clean(smp_fifo_flash_t *p_fifo);

#endif /* __SMP_FIFO_FLASH_H */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Simplo all right reserved *****END OF FILE****/

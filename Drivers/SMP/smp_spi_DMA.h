/**
  ******************************************************************************
  * @file    smp_spi_DMA.h 
  * @author  John Chen
  * @version V0.0.1
  * @date    2021/11/06
  * @brief   Header for smp_spi.c module
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMP_SPI_H
#define __SMP_SPI_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "smp_gpio.h"
/* Exported types ------------------------------------------------------------*/
extern SPI_HandleTypeDef smp_spi1_handle;
extern SPI_HandleTypeDef smp_spi2_handle;
extern SPI_HandleTypeDef smp_spi3_handle;

typedef enum{
	SPI_module1 = 0,											/* SPI module 0 */
	SPI_module2,													/* SPI module 1 */
	SPI_module3														/* SPI module 2 */
}spi_module_number;

typedef enum{
	SPI_mode0 = 0,												/* SPI mode 0 */
	SPI_mode1,														/* SPI mode 1 */
	SPI_mode2,														/* SPI mode 2 */
	SPI_mode3															/* SPI mode 3 */
}spi_mode_number;

typedef struct{
	spi_module_number		num;					/* SPI number */
	spi_mode_number			mode;					// SPI Mode Number
}smp_spi_t;

typedef struct{
	spi_module_number		spi_num;					/* SPI number */
	smp_gpio_t					cs_handler;
}smp_spi_cs_t;



typedef enum{
	SMP_SPI_EVENT_DONE = 0,		/* Transfer done. */
	SMP_SPI_EVENT_TRANSFER_BUSY,
	SMP_SPI_EVENT_TRANSFERR_ERROR
}smp_spi_evt_type;
typedef void (*smp_spi_event_t)(smp_spi_evt_type p_evt);
/* Exported constants --------------------------------------------------------*/ 
extern uint8_t uSPIFlag[3];
/* Exported defines --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

int8_t smp_spi_get_status(smp_spi_t *spi);
int8_t smp_spi_master_cs_init(smp_spi_cs_t *p_cs);
int8_t smp_spi_master_cs_deinit(smp_spi_cs_t *p_cs);
int8_t smp_spi_master_cs_set(smp_spi_cs_t *p_cs, uint8_t status);
int8_t smp_spi_master_init(smp_spi_t *p_spi, smp_spi_event_t smp_spi_event_handler, const bool lsb);
int8_t smp_spi_master_deinit(smp_spi_t *p_spi);
int8_t smp_spi_master_send_recv(smp_spi_t *spi, uint8_t *tx_data,uint16_t tx_size, uint8_t *rx_data, uint16_t rx_size,smp_spi_cs_t *p_cs);
int8_t smp_spi_master_send_recv_blocking(smp_spi_t *spi, uint8_t *tx_data,uint16_t tx_size, uint8_t *rx_data, uint16_t rx_size,smp_spi_cs_t *p_cs);
#endif /* __SMP_SPI_H */

/************************ (C) COPYRIGHT Simplo all right reserved *****END OF FILE****/

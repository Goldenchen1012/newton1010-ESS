/**
  ******************************************************************************
  * @file    smp_can_fifo.h 
  * @author  Johnny Wang
  * @version V0.0.1
  * @date    2021/10/20
  * @brief   Header for smp_can_fifo.c module
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMP_CAN_FIFO_H
#define __SMP_CAN_FIFO_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Exported types ------------------------------------------------------------*/
typedef struct{
	uint32_t	id;
	uint8_t		dlc;
	uint8_t		dat[8];
}smp_can_package_t;

typedef struct{
	smp_can_package_t	*buffer_addr;	             /* FIFO queue buffer start address */
	uint16_t	buffer_size;	                     /* FIFO queue buffer size */
	uint32_t	in;				                         /* FIFO queue buffer push in address */
	uint32_t	out;			                        /* FIFO queue buffer pop out address */
}smp_can_fifo_t;

/* Exported constants --------------------------------------------------------*/ 
/* Exported defines --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int8_t smp_can_fifo_open(smp_can_fifo_t *p_fifo);
int8_t smp_can_fifo_push(smp_can_fifo_t *p_fifo, smp_can_package_t *val);
int8_t smp_can_fifo_pop(smp_can_fifo_t *p_fifo, smp_can_package_t *val);
int8_t smp_can_fifo_back(smp_can_fifo_t *p_fifo);
int8_t smp_can_fifo_check(smp_can_fifo_t *p_fifo, char *val, uint32_t index);
int8_t smp_can_fifo_get_size(smp_can_fifo_t *p_fifo, uint16_t *size);
int8_t smp_can_fifo_clean(smp_can_fifo_t *p_fifo);

#endif /* __SMP_CAN_FIFO_H */

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

/**
  ******************************************************************************
  * @file    smp_fifo.h 
  * @author  Golden
  * @version V0.0.3
  * @date    2022/01/13
  * @brief   Header for smp_fifo.c module
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMP_FIFO_H
#define __SMP_FIFO_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
typedef struct{
	char		*buffer_addr;	/* FIFO queue buffer start address */
	uint16_t	buffer_size;	/* FIFO queue buffer size */
	uint32_t	in;				/* FIFO queue buffer push in address */
	uint32_t	out;			/* FIFO queue buffer pop out address */
}smp_fifo_t;

/* Exported constants --------------------------------------------------------*/ 
/* Exported defines --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int8_t smp_fifo_open(smp_fifo_t *p_fifo);
int8_t smp_fifo_push(smp_fifo_t *p_fifo, char val);
int8_t smp_fifo_pop(smp_fifo_t *p_fifo, char *val);
int8_t smp_fifo_back(smp_fifo_t *p_fifo);
int8_t smp_fifo_check(smp_fifo_t *p_fifo, char *val, uint32_t index);
int8_t smp_fifo_get_size(smp_fifo_t *p_fifo, uint16_t *size);
int8_t smp_fifo_clean(smp_fifo_t *p_fifo);

#ifdef __cplusplus
}
#endif

#endif /* __SMP_FIFO_H */
/************************ (C) COPYRIGHT Simplo all right reserved *****END OF FILE****/

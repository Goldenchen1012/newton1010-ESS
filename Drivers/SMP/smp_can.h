/**
  ******************************************************************************
  * @file    smp_can.h 
  * @author  Johnny Wang / Golden
  * @version V0.0.1
  * @date    2022/01/03
  * @brief   Header for smp_can.c module
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMP_CAN_H
#define __SMP_CAN_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "smp_can_fifo.h"
/* Exported types ------------------------------------------------------------*/
#define	CAN_STD_MASK		0x80000000L
#define	CAN_RTR_MASK		0x40000000L
#define	CAN_ID_MASK			0x1FFFFFFFL

typedef enum{
	CAN_DATA_READY = 0,					                 /* CAN data has been received */
	CAN_BUFFER_FULL,       			                 /* An error in the FIFO buffer full. The FIFO error code is stored in can_evt_type. */
	CAN_COMMUNICATION_ERR,				               /* CAN has occurred during reception */
	CAN_TX_EMPTY                                 /* CAN has complete transmission of all variable data */
}can_evt_type;

typedef void(*smp_can_event_t)(can_evt_type p_evt);

#if	1
typedef enum{
	__CAN0 = 0,                                  /* BSP_CAN module 0 */
	__CAN1                                       /* BSP_CAN module 1 */
}can_module_number;
#endif

typedef struct{
	smp_can_package_t		*rx_buf;                 /* Pointer to the RX buffer */
	uint32_t				    rx_buf_size;			       /* Size of the RX buffer */
	smp_can_package_t		*tx_buf;                 /* Pointer to the TX buffer */
	uint32_t				    tx_buf_size;             /* Size of the TX buffer */
}can_buffer_t;

typedef struct{
	can_module_number	  num;                     /* CAN number */
	uint32_t			      baud_rate;               /* CAN baud rate configuration */
	can_buffer_t		    buffers;                 /* CAN buffer for transmitting/receiving data */
}smp_can_t;

/* Exported constants --------------------------------------------------------*/ 
/* Exported defines --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int8_t smp_can_init(smp_can_t *p_can, smp_can_event_t smp_can_event_handler);
int8_t smp_can_deinit(smp_can_t *p_can);
int8_t smp_can_put(smp_can_t *p_can, smp_can_package_t *pDat);
int8_t smp_can_get(smp_can_t *p_can,smp_can_package_t *pDat);
#endif /* __SMP_CAN_H */

/************************ (C) COPYRIGHT Simplo all right reserved *****END OF FILE****/

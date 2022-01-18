/**
  ******************************************************************************
  * @file    smp_uart.h 
  * @author  Golden Chen
  * @version V0.0.1
  * @date    2021/09/14
  * @brief   Header for smp_uart.c module
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMP_UART_H
#define __SMP_UART_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
/* Exported types ------------------------------------------------------------*/
typedef enum{
	UART_DATA_READY = 0,					/* UART data has been received */
	UART_BUFFER_FULL,       			/* An error in the FIFO buffer full. The FIFO error code is stored in uart_evt_type. */
	UART_COMMUNICATION_ERR,				/* UART has occurred during reception */
	UART_TX_EMPTY,							  /* UART has complete transmission of all variable data */ 
	UART_TX_READY_TO_SEND         /* UART data TX  for RS485 directiion use.*/
}uart_evt_type;

typedef void(*smp_uart_event_t)(uart_evt_type p_evt);

typedef enum{
	UART0 = 0,										/* UART module 0 */
	UART1,													/* UART module 1 */
	UART2
}uart_module_number;

typedef enum{
	UART_FLOW_CTRL_DISABLE = 0,		/* UART HW flow control is disabled. */
	UART_FLOW_CTRL_ENABLE					/* Standard UART HW flow control is enabled */
}uart_flow_ctrl;

typedef enum{
	PARITY_NONE = 0,							/* UART no event parity */
	PARITY_EVEN,									/* UART even event parity */
	PARITY_ODD										/* UART odd event parity */
}uart_parity;

typedef struct{
	uint8_t			*rx_buf;					/* Pointer to the RX buffer */
	uint32_t		rx_buf_size;			/* Size of the RX buffer */
	uint8_t			*tx_buf;					/* Pointer to the TX buffer */
	uint32_t		tx_buf_size;			/* Size of the TX buffer */
}uart_buffer_t;

typedef struct{
	uart_module_number	num;			/* UART number */
	uint32_t			baud_rate;			/* UART baud rate configuration */
	uart_flow_ctrl		flow_ctrl;	/* UART flow control mode for the peripheral */
	uart_parity			use_parity;		/* UART event parity */
	uart_buffer_t		buffers;			/* UART buffer for transmitting/receiving data */
}smp_uart_t;

/* Exported constants --------------------------------------------------------*/ 

#define UART0_TX_BUFFER_SIZE                    256
#define UART0_RX_BUFFER_SIZE                    1

#define UART1_TX_BUFFER_SIZE                    256
#define UART1_RX_BUFFER_SIZE                    1

#define UART2_TX_BUFFER_SIZE                    256
#define UART2_RX_BUFFER_SIZE                    1

/* Exported defines --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int8_t smp_uart_init(smp_uart_t *p_uart, smp_uart_event_t smp_uart_event_handler);
int8_t smp_uart_deinit(smp_uart_t *p_uart);
int8_t smp_uart_put(smp_uart_t *p_uart, uint8_t byte);
int8_t smp_uart_get(smp_uart_t *p_uart, uint8_t *p_byte);
int8_t smp_uart_get_string(smp_uart_t *p_uart, uint8_t *p_byte);
#endif /* __SMP_UART_H */

/************************ (C) COPYRIGHT Simplo all right reserved *****END OF FILE****/

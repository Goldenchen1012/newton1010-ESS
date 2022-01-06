/**
  ******************************************************************************
  * @file    smp_ADS7946_Driver.h 
  * @author  John Chen
  * @version V0.0.1
  * @date    2021/11/11
  * @brief   Header for ADS7946 driver
  ******************************************************************************
  */
	
#include "main.h"
#include <string.h>	
#include "smp_spi_DMA.h"	

#ifndef _SMP_ADS7946_DRIVER_
#define _SMP_ADS7946_DRIVER_
typedef enum{
	channel_0 = 0,												
	channel_1						
}smp_ADS7946_channel_num;
	
typedef enum{
	CS_0 = 0,												
	CS_1						
}smp_ADS7946_CS_num;

typedef struct{
	smp_ADS7946_channel_num channel_sel;
	smp_spi_cs_t ADS7946_CS0;
}smp_ADS7946_control;
	
typedef void(*ads7946_CB_Fun_t)(uint8_t *pDatBuf,uint8_t bufsize);

//int8_t smp_ADS7946_init(void);
int8_t smp_ADS7946_init(void);
int8_t smp_ADS7946_deinit(void);
int8_t smp_ADS7946_get_data(smp_ADS7946_channel_num channel_sel,smp_ADS7946_CS_num CS,ads7946_CB_Fun_t evt_callback);

#endif

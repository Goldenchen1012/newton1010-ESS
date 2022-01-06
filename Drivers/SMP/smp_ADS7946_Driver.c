/**
  ******************************************************************************
  * @file    smp_ADS7946_Driver.c
  * @author  John Chen/ Golden
  * @version V0.0.2
  * @date    2021/12/30
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

#include "Bsp.h"
#include "smp_debug.h"
#include "smp_gpio.h"
#include "smp_ADS7946_Driver.h"
#include <stdbool.h>

smp_gpio_t ADS7956_PDEN_handler;
smp_gpio_t ADS7956_CH_SEL_handler;

smp_gpio_t ADS7946_PB0;
smp_spi_cs_t ADS7946_CS0;

smp_gpio_t ADS7946_PB1;
smp_spi_cs_t ADS7946_CS1;
smp_spi_t ADS7946_SPI_0;

static bool Flag_ADS7946_HW_Init_Done = false;
uint8_t ADS7946_tx_data[BSP_SPI1_TX_BUFFER_SIZE];
uint8_t ADS7946_rx_data[BSP_SPI1_RX_BUFFER_SIZE];

void ADS7946_SPI_0_event_handler(smp_spi_evt_type p_evt);

#if 0
int8_t smp_ADS7946_power_on(void);
int8_t smp_ADS7946_power_down(void);
#endif

int8_t smp_ADS7946_channel_select(smp_ADS7946_channel_num channel_sel);

ads7946_CB_Fun_t	ads7946_event_cb = 0;


int8_t smp_ADS7946_init(void)
{
  //initial ADS7946 PDEN
	#if 0
	ADS7956_PDEN_handler.port = SMP_GPIOB;
	ADS7956_PDEN_handler.pin = PIN3;
	ADS7956_PDEN_handler.mode = SMP_GPIO_MODE_OUTPUT_OD;	
	if(smp_gpio_init(&ADS7956_PDEN_handler) != HAL_OK){
    return SMP_ERROR_NOT_FOUND;
	}
	#endif 
	
	if(Flag_ADS7946_HW_Init_Done){
		return SMP_SUCCESS;
	}
	
	//initial ADS7946 CH_SEL
	ADS7956_CH_SEL_handler.port = BSP_ADS7946_CH_SEL_GPIO_PORT;
	ADS7956_CH_SEL_handler.pin = BSP_ADS7946_CH_SEL_PIN;
	ADS7956_CH_SEL_handler.mode = SMP_GPIO_MODE_OUTPUT_OD;	
	if(smp_gpio_init(&ADS7956_CH_SEL_handler) != HAL_OK){
		return SMP_ERROR_NOT_FOUND;
	}
	
	// Initial ADS7946 SPI
	// CS0 Initial
	ADS7946_PB0.port = BSP_ADS7946_CS_MAIN_GPIO_PORT;
	ADS7946_PB0.pin = BSP_ADS7946_CS_MAIN_PIN;	
	ADS7946_CS0.spi_num = SPI_module1;
	ADS7946_CS0.cs_handler = ADS7946_PB0;	
	smp_spi_master_cs_init(&ADS7946_CS0);
	
	// CS1 Initial
	ADS7946_PB1.port = BSP_ADS7946_CS_AUX_GPIO_PORT;
	ADS7946_PB1.pin = BSP_ADS7946_CS_AUX_PIN;	
	ADS7946_CS1.spi_num = SPI_module1;
	ADS7946_CS1.cs_handler = ADS7946_PB1;	
	smp_spi_master_cs_init(&ADS7946_CS1);
	ADS7946_SPI_0.num = SPI_module1;
	ADS7946_SPI_0.mode = SPI_mode0;
	
	#if 0
  smp_ADS7946_power_on();
  #endif
	
	if(smp_spi_master_init(&ADS7946_SPI_0, ADS7946_SPI_0_event_handler, false) != SMP_SUCCESS){
		return SMP_ERROR_NOT_FOUND;
	}
	if(!Flag_ADS7946_HW_Init_Done){
		Flag_ADS7946_HW_Init_Done = true;
	}
	

	return SMP_SUCCESS;	
}

int8_t smp_ADS7946_deinit(void)
{
  #if 0
	smp_ADS7946_power_down();
	#endif
	smp_gpio_deinit(&ADS7956_PDEN_handler);
	smp_gpio_deinit(&ADS7956_CH_SEL_handler);
	
	smp_spi_master_cs_deinit(&ADS7946_CS0);
	smp_spi_master_cs_deinit(&ADS7946_CS1);
	if(smp_spi_master_deinit(&ADS7946_SPI_0) != SMP_SUCCESS){
		return SMP_ERROR_NOT_FOUND;
	}
	
	return SMP_SUCCESS;	
}

#if 0
int8_t smp_ADS7946_power_on(void)
{
	 return smp_gpio_set_state(&ADS7956_PDEN_handler, GPIO_ACTIVE_LOW);;
}

int8_t smp_ADS7946_power_down(void)
{
	return smp_gpio_set_state(&ADS7956_PDEN_handler, GPIO_ACTIVE_HIGH);;
}
#endif

int8_t smp_ADS7946_channel_select(smp_ADS7946_channel_num channel_sel)
{
	if(channel_sel == channel_0){
      return smp_gpio_set_state(&ADS7956_CH_SEL_handler, GPIO_ACTIVE_LOW);
	}else if(channel_sel == channel_1){
	    return smp_gpio_set_state(&ADS7956_CH_SEL_handler, GPIO_ACTIVE_HIGH);
	}else{
	    return SMP_ERROR_INVALID_PARAM;
	}
}

int8_t smp_ADS7946_get_data(smp_ADS7946_channel_num channel_sel,smp_ADS7946_CS_num CS, ads7946_CB_Fun_t evt_callback)
{
	if(ads7946_event_cb){
		return SMP_ERROR_BUSY;
	}else{
		ads7946_event_cb = evt_callback;
	}
	
	smp_ADS7946_channel_select(channel_sel);
	if(CS==CS_0){
		return smp_spi_master_send_recv(&ADS7946_SPI_0,  0 , 0 , ADS7946_rx_data, 4 , &ADS7946_CS0);
	}else if(CS==CS_1){
		return smp_spi_master_send_recv(&ADS7946_SPI_0,  0 , 0 , ADS7946_rx_data, 4 , &ADS7946_CS1);
	}else{
		return SMP_ERROR_INVALID_PARAM;
	}	
}

void ADS7946_SPI_0_event_handler(smp_spi_evt_type p_evt)
{
	static uint8_t	rxdata[4];
	#if 0
	uint8_t test[3];
	#endif
	
	switch(p_evt){
		case SMP_SPI_EVENT_DONE:	
			rxdata[0] = ADS7946_rx_data[0];
			rxdata[1] = ADS7946_rx_data[1];
			rxdata[2] = ADS7946_rx_data[2];
			rxdata[3] = ADS7946_rx_data[3];
		  #if 0
			memset(ADS7946_rx_data, 0, sizeof(ADS7946_rx_data));
			#endif
		  if(ads7946_event_cb){
				ads7946_event_cb(rxdata, 4);
				ads7946_event_cb = 0;
			}				
		break;
		case SMP_SPI_EVENT_TRANSFER_BUSY:
			
			break;
		default:
		break;
	}
}




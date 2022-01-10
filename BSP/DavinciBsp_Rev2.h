/**
  ******************************************************************************
  * @file        DavinciBsp_Rev2.h
  * @author      Johnny/ Golden Chen
  * @version     v0.0.2
  * @date        2022/01/06
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */
#ifndef _DAVINCI_BSP_REV2_H_
#define _DAVINCI_BSP_REV2_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_nucleo_144.h"
#include "smp_debug.h"
#include "smp_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
GPIO_TypeDef  *gpio_port;
uint16_t      gpio_pin;
}bsp_adc_init_io_config;

/* Public define ------------------------------------------------------------*/
#define BSP_BQ7600	
#define BSP_AD7946 
#define BSP_MX25L
#define BSP_W5500
#define BSP_IRM

#ifndef SMP_APP_FM_NOR_FLASH_ENABLE
	#define SMP_APP_FM_NOR_FLASH_ENABLE 1
#endif

void HalBspSetGpio(GPIO_TypeDef  *GPIOx, uint16_t Pin, uint32_t mode,uint32_t pull, uint32_t speed);

// Board Input function pin define
//----------------------------------------------------------------------------------
#define	BSP_DI1_GPIO                           GPIOE
#define	BSP_DI1_PIN						                 GPIO_PIN_2
#define BSP_DI1_OPEN()                         {HalBspSetGpio(	BSP_DI1_GPIO, BSP_DI1_PIN, \
										                           GPIO_MODE_INPUT, GPIO_PULLUP, \
									 	                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_DI1_READ()                         (BSP_DI1_GPIO->IDR & BSP_DI1_PIN)

#define	BSP_DI2_GPIO                           GPIOE
#define	BSP_DI2_PIN                            GPIO_PIN_3
#define BSP_DI2_OPEN()                         {HalBspSetGpio(	BSP_DI2_GPIO, BSP_DI2_PIN, \
										                           GPIO_MODE_INPUT, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_DI2_READ()                         (BSP_DI2_GPIO->IDR & BSP_DI2_PIN)
				
#define	BSP_EPO_GPIO				                   GPIOC
#define	BSP_EPO_PIN					                   GPIO_PIN_0
#define BSP_EPO_OPEN()                         {HalBspSetGpio(	BSP_EPO_GPIO, BSP_EPO_PIN, \
										                           GPIO_MODE_INPUT, GPIO_PULLDOWN, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_EPO_READ()                         (BSP_EPO_GPIO->IDR & BSP_EPO_PIN)
		
#define	BSP_SP_FB_GPIO                         GPIOC
#define	BSP_SP_FB_PIN                          GPIO_PIN_1
#define BSP_SP_FB_OPEN()                       {HalBspSetGpio(BSP_SP_FB_GPIO, BSP_SP_FB_PIN, \
										                           GPIO_MODE_INPUT, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_SP_FB_READ()                       (BSP_SP_FB_GPIO->IDR & BSP_SP_FB_PIN)

#define	BSP_PS1_OK_GPIO					               GPIOE
#define	BSP_PS1_OK_PIN                         GPIO_PIN_7
#define BSP_PS1_OK_OPEN()                      {HalBspSetGpio(BSP_PS1_OK_GPIO, BSP_PS1_OK_PIN, \
										                           GPIO_MODE_INPUT, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_PS1_OK_READ()                      (BSP_PS1_OK_GPIO->IDR & BSP_PS1_OK_PIN)

#define	BSP_PS2_OK_GPIO                        GPIOE
#define	BSP_PS2_OK_PIN                         GPIO_PIN_8
#define BSP_PS2_OK_OPEN()                      {HalBspSetGpio(BSP_PS2_OK_GPIO, BSP_PS2_OK_PIN, \
										                           GPIO_MODE_INPUT, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_PS2_OK_READ()                      (BSP_PS2_OK_GPIO->IDR & BSP_PS2_OK_PIN)

#define	BSP_PS3_OK_GPIO                        GPIOE
#define	BSP_PS3_OK_PIN                         GPIO_PIN_9
#define BSP_PS3_OK_OPEN()                      {HalBspSetGpio(BSP_PS3_OK_GPIO, BSP_PS3_OK_PIN, \
										                           GPIO_MODE_INPUT, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_PS3_OK_READ()                      (BSP_PS3_OK_GPIO->IDR & BSP_PS3_OK_PIN)

#define	BSP_BUTTON_GPIO                        GPIOE
#define	BSP_BUTTON_PIN                         GPIO_PIN_14
#define BSP_BUTTON_OPEN()                      {HalBspSetGpio(BSP_BUTTON_GPIO, BSP_BUTTON_PIN, \
										                           GPIO_MODE_INPUT, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_BUTTON_READ()                      (BSP_BUTTON_GPIO->IDR & BSP_BUTTON_PIN)

#define	BSP_K1_FB_GPIO                         GPIOD
#define	BSP_K1_FB_PIN                          GPIO_PIN_13
#define BSP_K1_FB_OPEN()                       {HalBspSetGpio(BSP_K1_FB_GPIO, BSP_K1_FB_PIN, \
										                           GPIO_MODE_INPUT, GPIO_PULLUP, \
                                               GPIO_SPEED_FREQ_HIGH);}
#define BSP_K1_FB_READ()                       (BSP_K1_FB_GPIO->IDR & BSP_K1_FB_PIN)

#define BSP_K2_FB_GPIO                         GPIOD
#define	BSP_K2_FB_PIN                          GPIO_PIN_14
#define BSP_K2_FB_OPEN()                       {HalBspSetGpio(BSP_K2_FB_GPIO, BSP_K2_FB_PIN, \
                                               GPIO_MODE_INPUT, GPIO_PULLUP, \
                                               GPIO_SPEED_FREQ_HIGH);}
#define BSP_K2_FB_READ()                       (BSP_K2_FB_GPIO->IDR & BSP_K2_FB_PIN)

#define	BSP_K3_FB_GPIO                         GPIOD
#define	BSP_K3_FB_PIN                          GPIO_PIN_15
#define BSP_K3_FB_OPEN()                       {HalBspSetGpio(BSP_K3_FB_GPIO, BSP_K3_FB_PIN, \
										                           GPIO_MODE_INPUT, GPIO_PULLUP, \
                                               GPIO_SPEED_FREQ_HIGH);}
#define BSP_K3_FB_READ()                       (BSP_K3_FB_GPIO->IDR & BSP_K3_FB_PIN)

#define	BSP_K4_FB_GPIO                         GPIOC
#define	BSP_K4_FB_PIN                          GPIO_PIN_6
#define BSP_K4_FB_OPEN()                       {HalBspSetGpio(BSP_K4_FB_GPIO, BSP_K4_FB_PIN, \
	                                             GPIO_MODE_INPUT, GPIO_PULLUP, \
                                               GPIO_SPEED_FREQ_HIGH);}
#define BSP_K4_FB_READ()                       (BSP_K4_FB_GPIO->IDR & BSP_K4_FB_PIN)

#define	BSP_DOCP_LATCH_GPIO                    GPIOC
#define	BSP_DOCP_LATCH_PIN                     GPIO_PIN_7
#define BSP_DOCP_LATCH_OPEN()                  {HalBspSetGpio(BSP_DOCP_LATCH_GPIO, BSP_DOCP_LATCH_PIN, \
                                               GPIO_MODE_INPUT, GPIO_PULLUP, \
                                               GPIO_SPEED_FREQ_HIGH);}
#define BSP_DOCP_LATCH_READ()                  (BSP_DOCP_LATCH_GPIO->IDR & BSP_DOCP_LATCH_PIN)

#define	BSP_COCP_LATCH_GPIO                    GPIOC
#define	BSP_COCP_LATCH_PIN                     GPIO_PIN_8
#define BSP_COCP_LATCH_OPEN()                  {HalBspSetGpio(BSP_COCP_LATCH_GPIO, BSP_COCP_LATCH_PIN, \
										                           GPIO_MODE_INPUT, GPIO_PULLUP, \
                                               GPIO_SPEED_FREQ_HIGH);}
#define BSP_COCP_LATCH_READ()                  (BSP_COCP_LATCH_GPIO->IDR & BSP_COCP_LATCH_PIN)

#define	BSP_OD_IN_GPIO                         GPIOB
#define	BSP_OD_IN_PIN                          GPIO_PIN_3
#define BSP_OD_IN_OPEN()                       {HalBspSetGpio(BSP_OD_IN_GPIO,BSP_OD_IN_PIN, \
										                           GPIO_MODE_INPUT, GPIO_NOPULL, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_OD_IN_READ()                       (BSP_OD_IN_GPIO->IDR & BSP_OD_IN_PIN)
																							 
#define	BSP_NFAULT_GPIO                         GPIOE
#define	BSP_NFAULT_PIN                          GPIO_PIN_4
#define BSP_NFAULT_OPEN()                       {HalBspSetGpio(BSP_NFAULT_GPIO,BSP_NFAULT_PIN, \
										                            GPIO_MODE_INPUT, GPIO_NOPULL, \
										                            GPIO_SPEED_FREQ_HIGH);}
#define BSP_NFAULT_READ()                       (BSP_NFAULT_GPIO->IDR & BSP_NFAULT_PIN)	 
//----------------------------------------------------------------------------------

// Board Output function pin define
//----------------------------------------------------------------------------------
#define	BSP_DO1_GPIO                           GPIOE
#define	BSP_DO1_PIN                            GPIO_PIN_5
#define BSP_DO1_OPEN()                         {HalBspSetGpio(	BSP_DO1_GPIO, BSP_DO1_PIN, \
                                               GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
                                               GPIO_SPEED_FREQ_HIGH);}
#define BSP_DO1_HI()                           (BSP_DO1_GPIO->BSRR = BSP_DO1_PIN)
#define BSP_DO1_LO()                           (BSP_DO1_GPIO->BRR = BSP_DO1_PIN)
										
#define	BSP_DO2_GPIO                           GPIOE
#define	BSP_DO2_PIN                            GPIO_PIN_6
#define BSP_DO2_OPEN()                         {HalBspSetGpio(	BSP_DO2_GPIO, BSP_DO2_PIN, \
										                           GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_DO2_HI()                           (BSP_DO2_GPIO->BSRR = BSP_DO2_PIN)
#define BSP_DO2_LO()                           (BSP_DO2_GPIO->BRR = BSP_DO2_PIN)
	
#if 0																							 
#define	BSP_ADC_CH_SEL_GPIO                    GPIOC
#define	BSP_ADC_CH_SEL_PIN                     GPIO_PIN_5
#define BSP_ADC_CH_SEL_OPEN()                  {HalBspSetGpio(	BSP_ADC_CH_SEL_GPIO, BSP_ADC_CH_SEL_PIN, \
										                           GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
                                               GPIO_SPEED_FREQ_HIGH);}
#define BSP_ADC_CH_SEL_HI()                    (BSP_ADC_CH_SEL_GPIO->BSRR = BSP_ADC_CH_SEL_PIN)
#define BSP_ADC_CH_SEL_LO()                    (BSP_ADC_CH_SEL_GPIO->BRR = BSP_ADC_CH_SEL_PIN)

#define	BSP_ADC_CS_MAIN_GPIO                   GPIOB
#define	BSP_ADC_CS_MAIN_PIN                    GPIO_PIN_0
#define BSP_ADC_CS_MAIN_OPEN()                 {HalBspSetGpio(	BSP_ADC_CS_MAIN_GPIO, BSP_ADC_CS_MAIN_PIN, \
										                           GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
                                               GPIO_SPEED_FREQ_HIGH);}
#define BSP_ADC_CS_MAIN_HI()                   (BSP_ADC_CS_MAIN_GPIO->BSRR = BSP_ADC_CS_MAIN_PIN)
#define BSP_ADC_CS_MAIN_LO()                   (BSP_ADC_CS_MAIN_GPIO->BRR = BSP_ADC_CS_MAIN_PIN)

#define	BSP_ADC_CS_AUX_GPIO                    GPIOB
#define	BSP_ADC_CS_AUX_PIN                     GPIO_PIN_0
#define BSP_ADC_CS_AUX_OPEN()                  {HalBspSetGpio(	BSP_ADC_CS_AUX_GPIO, BSP_ADC_CS_AUX_PIN, \
										                           GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_ADC_CS_AUX_HI()                    (BSP_ADC_CS_AUX_GPIO->BSRR = BSP_ADC_CS_AUX_PIN)
#define BSP_ADC_CS_AUX_LO()                    (BSP_ADC_CS_AUX_GPIO->BRR = BSP_ADC_CS_AUX_PIN)

#endif

#define	BSP_K1_GPIO                            GPIOD
#define	BSP_K1_PIN                             GPIO_PIN_9
#define BSP_K1_OPEN()                          {HalBspSetGpio(	BSP_K1_GPIO, BSP_K1_PIN, \
										                           GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_K1_HI()                            (BSP_K1_GPIO->BSRR = BSP_K1_PIN)
#define BSP_K1_LO()                            (BSP_K1_GPIO->BRR = BSP_K1_PIN)

#define	BSP_K2_GPIO						                 GPIOD
#define	BSP_K2_PIN						                 GPIO_PIN_10
#define BSP_K2_OPEN()                          {HalBspSetGpio(	BSP_K2_GPIO, BSP_K2_PIN, \
										                           GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_K2_HI()                            (BSP_K2_GPIO->BSRR = BSP_K2_PIN)
#define BSP_K2_LO()                            (BSP_K2_GPIO->BRR = BSP_K2_PIN)

#define	BSP_K3_GPIO                            GPIOD
#define	BSP_K3_PIN                             GPIO_PIN_11
#define BSP_K3_OPEN()                          {HalBspSetGpio(	BSP_K3_GPIO, BSP_K3_PIN, \
										                           GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_K3_HI()                            (BSP_K3_GPIO->BSRR = BSP_K3_PIN)
#define BSP_K3_LO()                            (BSP_K3_GPIO->BRR = BSP_K3_PIN)

#define	BSP_K4_GPIO						                 GPIOD
#define	BSP_K4_PIN						                 GPIO_PIN_12
#define BSP_K4_OPEN()                          {HalBspSetGpio(	BSP_K4_GPIO, BSP_K4_PIN, \
										                           GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_K4_HI()                            (BSP_K4_GPIO->BSRR = BSP_K4_PIN)
#define BSP_K4_LO()                            (BSP_K4_GPIO->BRR = BSP_K4_PIN)

#define	BSP_RELAY_PS_GPIO				               GPIOB
#define	BSP_RELAY_PS_PIN                       GPIO_PIN_2
#define BSP_RELAY_PS_OPEN()                    {HalBspSetGpio(	BSP_RELAY_PS_GPIO, BSP_RELAY_PS_PIN, \
                                               GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
                                               GPIO_SPEED_FREQ_HIGH);}
#define BSP_RELAY_PS_HI()                      (BSP_RELAY_PS_GPIO->BSRR = BSP_RELAY_PS_PIN)
#define BSP_RELAY_PS_LO()                      (BSP_RELAY_PS_GPIO->BRR = BSP_RELAY_PS_PIN)

#define	BSP_TOWER_LIGHT_RED_GPIO               GPIOD
#define	BSP_TOWER_LIGHT_RED_PIN                GPIO_PIN_0
#define BSP_TOWER_LIGHT_RED_OPEN()             {HalBspSetGpio(	BSP_TOWER_LIGHT_RED_GPIO, BSP_TOWER_LIGHT_RED_PIN, \
										                           GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_TOWER_LIGHT_RED_HI()               (BSP_TOWER_LIGHT_RED_GPIO->BSRR = BSP_TOWER_LIGHT_RED_PIN)
#define BSP_TOWER_LIGHT_RED_LO()               (BSP_TOWER_LIGHT_RED_GPIO->BRR = BSP_TOWER_LIGHT_RED_PIN)

#define	BSP_TOWER_LIGHT_ORANGE_GPIO            GPIOD
#define	BSP_TOWER_LIGHT_ORANGE_PIN             GPIO_PIN_1
#define BSP_TOWER_LIGHT_ORANGE_OPEN()          {HalBspSetGpio(	BSP_TOWER_LIGHT_ORANGE_GPIO,BSP_TOWER_LIGHT_ORANGE_PIN, \
										                           GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_TOWER_LIGHT_ORANGE_HI()            (BSP_TOWER_LIGHT_ORANGE_GPIO->BSRR = BSP_TOWER_LIGHT_ORANGE_PIN)
#define BSP_TOWER_LIGHT_ORANGE_LO()            (BSP_TOWER_LIGHT_ORANGE_GPIO->BRR = BSP_TOWER_LIGHT_ORANGE_PIN)

#define	BSP_TOWER_LIGHT_GREEN_GPIO		         GPIOD
#define	BSP_TOWER_LIGHT_GREEN_PIN              GPIO_PIN_2
#define BSP_TOWER_LIGHT_GREEN_OPEN()           {HalBspSetGpio(	BSP_TOWER_LIGHT_GREEN_GPIO,BSP_TOWER_LIGHT_GREEN_PIN, \
										                           GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_TOWER_LIGHT_GREEN_HI()             (BSP_TOWER_LIGHT_GREEN_GPIO->BSRR = BSP_TOWER_LIGHT_GREEN_PIN)
#define BSP_TOWER_LIGHT_GREEN_LO()             (BSP_TOWER_LIGHT_GREEN_GPIO->BRR = BSP_TOWER_LIGHT_GREEN_PIN)

#define	BSP_OCP_RELEASE_GPIO		               GPIOD
#define	BSP_OCP_RELEASE_PIN                    GPIO_PIN_3
#define BSP_OCP_RELEASE_OPEN()                 {HalBspSetGpio(	BSP_OCP_RELEASE_GPIO, BSP_OCP_RELEASE_PIN, \
                                               GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_OCP_RELEASE_HI()                   (BSP_OCP_RELEASE_GPIO->BSRR = BSP_OCP_RELEASE_PIN)
#define BSP_OCP_RELEASE_LO()                   (BSP_OCP_RELEASE_GPIO->BRR = BSP_OCP_RELEASE_PIN)

#define	BSP_OD_OUT_GPIO                        GPIOB
#define	BSP_OD_OUT_PIN                         GPIO_PIN_4
#define BSP_OD_OUT_OPEN()                      {HalBspSetGpio(	BSP_OD_OUT_GPIO,BSP_OD_OUT_PIN, \
										                           GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_OD_OUT_HI()                        (BSP_OD_OUT_GPIO->BSRR = BSP_OD_OUT_PIN)
#define BSP_OD_OUT_LO()                        (BSP_OD_OUT_GPIO->BRR = BSP_OD_OUT_PIN)

#define	BSP_RS485_DERE_GPIO                    GPIOB
#define	BSP_RS485_DERE_PIN                     GPIO_PIN_7
#define BSP_RS485_DERE_OPEN()                  {HalBspSetGpio(	BSP_RS485_DERE_GPIO,BSP_RS485_DERE_PIN, \
										                           GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
										                           GPIO_SPEED_FREQ_HIGH);}
#define BSP_RS485_DERE_HI()                    (BSP_RS485_DERE_GPIO->BSRR = BSP_RS485_DERE_PIN)
#define BSP_RS485_DERE_LO()                    (BSP_RS485_DERE_GPIO->BRR = BSP_RS485_DERE_PIN)
																							 
							
#define	BSP_TERMLR_GPIO                       	GPIOB
#define	BSP_TERMLR_PIN                           GPIO_PIN_5
#define BSP_TERMLR_OPEN()                      {HalBspSetGpio(	BSP_TERMLR_GPIO, BSP_TERMLR_PIN, \
                                               GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, \
                                               GPIO_SPEED_FREQ_HIGH);}
#define BSP_TERMLR_HI()                        (BSP_TERMLR_GPIO->BSRR = BSP_TERMLR_PIN)
#define BSP_TERMLR_LO()                        (BSP_TERMLR_GPIO->BRR = BSP_TERMLR_PIN)
																							 
/********************************* BSP_BQ79600 I/O Pin define(please use "smp_gpio.h") *************/
#ifdef BSP_BQ7600																							 
#define BSP_BQ796XX_NCS_PIN                    PIN4
#define BSP_BQ796XX_NCS_PORT                   SMP_GPIOD

#define BSP_BQ796XX_RX_PIN                     PIN5
#define BSP_BQ796XX_RX_PORT                    SMP_GPIOD																							 
#endif									
																							 
/********************************* BSP_ADS7946 I/O Pin define(please use "smp_gpio.h") *************/	
#ifdef BSP_AD7946 																							 
#define BSP_ADS7946_CH_SEL_PIN                 PIN5
#define BSP_ADS7946_CH_SEL_GPIO_PORT           SMP_GPIOC
#define BSP_ADS7946_CS_MAIN_PIN                PIN0
#define BSP_ADS7946_CS_MAIN_GPIO_PORT          SMP_GPIOB
#define BSP_ADS7946_CS_AUX_PIN                 PIN1
#define BSP_ADS7946_CS_AUX_GPIO_PORT           SMP_GPIOB																							 
#endif 									
																							 
/*********************************BSP_MX25L  (SPI Flash)I/O Pin define(please use "smp_gpio.h") ***/
#ifdef BSP_MX25L 
#define BSP_MX25L_WRITE_PROTECTON_PIN          PIN0
#define BSP_MX25L_WRITE_PROTECTON_GPIO_PORT    SMP_GPIOE

#define BSP_MX25L_CS_PIN                       PIN15
#define BSP_MX25L_CS_GPIO_PORT                 SMP_GPIOA

#define BSP_MX25L_HOLD_PIN                     PIN9
#define BSP_MX25L_HOLD_GPIO_PORT               SMP_GPIOB

#endif
					
/*********************************BSP_W5500 (SPI to TCPIP)I/O Pin define ***/
#ifdef BSP_W5500
#define BSP_W5500_RST_PORT                     GPIOB
#define BSP_W5500_RST_PIN                      GPIO_PIN_6

#define BSP_W5500_INT_PORT                     GPIOD
#define BSP_W5500_INT_PIN                      GPIO_PIN_8
#endif

/********************************* BSP_IRM I/O Pin define ****************************************/
#ifdef BSP_IRM

#define BSP_IRM_SW_BAT_PORT                    GPIOC        
#define BSP_IRM_SW_BAT_PIN                     GPIO_PIN_9

#define BSP_IRM_SW_E_R_PORT                    GPIOE        
#define BSP_IRM_SW_E_R_PIN                     GPIO_PIN_15

#define BSP_IRM_SW_E_PORT                      GPIOA        
#define BSP_IRM_SW_E_PIN                       GPIO_PIN_8

#define BSP_IRM_SW1_PORT                       BSP_IRM_SW_BAT_PORT        
#define BSP_IRM_SW1_PIN                        BSP_IRM_SW_BAT_PIN
#define BSP_IRM_SW2_PORT                       BSP_IRM_SW_E_R_PORT        
#define BSP_IRM_SW2_PIN                        BSP_IRM_SW_E_R_PIN
#define BSP_IRM_SW3_PORT                       BSP_IRM_SW_E_PORT        
#define BSP_IRM_SW3_PIN                        BSP_IRM_SW_E_PIN
#define BSP_IRM_SW1_ON()                       (BSP_IRM_SW1_PORT->BSRR = BSP_IRM_SW1_PIN) 
#define BSP_IRM_SW1_OFF()                      (BSP_IRM_SW1_PORT->BRR = BSP_IRM_SW1_PIN) 
#define BSP_IRM_SW2_ON()                       (BSP_IRM_SW2_PORT->BSRR = BSP_IRM_SW2_PIN) 
#define BSP_IRM_SW2_OFF()                      (BSP_IRM_SW2_PORT->BRR = BSP_IRM_SW2_PIN)
#define BSP_IRM_SW3_ON()                       (BSP_IRM_SW3_PORT->BSRR = BSP_IRM_SW3_PIN) 
#define BSP_IRM_SW3_OFF()                      (BSP_IRM_SW3_PORT->BRR = BSP_IRM_SW3_PIN)

#endif

/********************************* BSP_UART0 I/O Pin define ****************************************/

/* Definition for BSP_UART0 clock resources */
#define BSP_UART0                        			 USART2
#define BSP_UART0_CLK_ENABLE()           			 __HAL_RCC_USART2_CLK_ENABLE()
#define BSP_UART0_DMA_CLK_ENABLE()          	 __HAL_RCC_DMA1_CLK_ENABLE()
#define BSP_UART0_RX_GPIO_CLK_ENABLE()      	 __HAL_RCC_GPIOD_CLK_ENABLE()
#define BSP_UART0_TX_GPIO_CLK_ENABLE()      	 __HAL_RCC_GPIOD_CLK_ENABLE()

#define BSP_UART0_FORCE_RESET()             	 __HAL_RCC_USART2_FORCE_RESET()
#define BSP_UART0_RELEASE_RESET()           	 __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for BSP_UART0 Pins */
#define BSP_UART0_TX_GPIO_PORT                 GPIOD
#define BSP_UART0_TX_PIN                       GPIO_PIN_5
#define BSP_UART0_TX_AF                        GPIO_AF7_USART2

#define BSP_UART0_RX_GPIO_PORT                 GPIOD
#define BSP_UART0_RX_PIN                       GPIO_PIN_6
#define BSP_UART0_RX_AF                     	 GPIO_AF7_USART2

/* Definition for BSP_UART0 DMA */
#define BSP_UART0_TX_DMA_CHANNEL             	 DMA1_Channel7
#define BSP_UART0_RX_DMA_CHANNEL             	 DMA1_Channel6

/* Definition for BSP_UART0 DMA Request(MCU UART1~UART5 must be setting DMA_REQUEST_2, acorrding STM32L496 p.339) */
#define BSP_UART0_TX_DMA_REQUEST             	 DMA_REQUEST_2
#define BSP_UART0_RX_DMA_REQUEST             	 DMA_REQUEST_2

/* Definition for BSP_UART0 NVIC */
#define BSP_UART0_DMA_TX_IRQn                	 DMA1_Channel7_IRQn
#define BSP_UART0_DMA_RX_IRQn                	 DMA1_Channel6_IRQn
#define BSP_UART0_DMA_TX_IRQHandler          	 DMA1_Channel7_IRQHandler
#define BSP_UART0_DMA_RX_IRQHandler          	 DMA1_Channel6_IRQHandler

/* Definition for BSP UART0 NVIC */
#define BSP_UART0_IRQn                      	 USART2_IRQn
#define BSP_UART0_IRQHandler                	 USART2_IRQHandler

/********************************* BSP_UART1 I/O Pin define ***************************************/
/* Definition for BSP_UART1 clock resources */
#define BSP_UART1                              USART3
#define BSP_UART1_CLK_ENABLE()           			 __HAL_RCC_USART3_CLK_ENABLE()
#define BSP_UART1_DMA_CLK_ENABLE()       			 __HAL_RCC_DMA1_CLK_ENABLE()
#define BSP_UART1_RX_GPIO_CLK_ENABLE()   			 __HAL_RCC_GPIOB_CLK_ENABLE()
#define BSP_UART1_TX_GPIO_CLK_ENABLE()   			 __HAL_RCC_GPIOB_CLK_ENABLE()

#define BSP_UART1_FORCE_RESET()          			 __HAL_RCC_USART3_FORCE_RESET()
#define BSP_UART1_RELEASE_RESET()        			 __HAL_RCC_USART3_RELEASE_RESET()

/* Definition for BSP_UART1 Pins */
#define BSP_UART1_TX_PIN											 GPIO_PIN_10
#define BSP_UART1_TX_GPIO_PORT								 GPIOB
#define BSP_UART1_TX_AF												 GPIO_AF7_USART3
#define BSP_UART1_RX_PIN											 GPIO_PIN_11
#define BSP_UART1_RX_GPIO_PORT								 GPIOB
#define BSP_UART1_RX_AF												 GPIO_AF7_USART3

/* Definition for BSP_UART1 DMA */
#if 0
#define BSP_UART1_TX_DMA_CHANNEL           	   DMA1_Channel2
#define BSP_UART1_RX_DMA_CHANNEL           	   DMA1_Channel3
#endif

/* Definition for BSP_UART1 DMA Request(MCU UART1~UART5 must be setting DMA_REQUEST_2, acorrding STM32L496 p.339) */
#define BSP_UART1_TX_DMA_REQUEST           		 DMA_REQUEST_2
#define BSP_UART1_RX_DMA_REQUEST           		 DMA_REQUEST_2

/* Definition for USARTx's NVIC */
#if 0
#define BSP_UART1_DMA_TX_IRQn              	   DMA1_Channel2_IRQn
#define BSP_UART1_DMA_RX_IRQn              	   DMA1_Channel3_IRQn
#define BSP_UART1_DMA_TX_IRQHandler            DMA1_Channel2_IRQHandler
#define BSP_UART1_DMA_RX_IRQHandler            DMA1_Channel3_IRQHandler
#endif 

/* Definition for SMP UART NVIC */
#define BSP_UART1_IRQn                   			 USART3_IRQn
#define BSP_UART1_IRQHandler             			 USART3_IRQHandler																	 

/********************************* BSP_SPI1 I/O Pin define ***************************************/
/* Definition BSP_SPI1 TX/RX buffer size */
#define BSP_SPI1_TX_BUFFER_SIZE                256
#define BSP_SPI1_RX_BUFFER_SIZE                256

//BSP_SPI1 Pin define
#define BSP_SPI1                             	 SPI1
#define BSP_SPI1_CLK_ENABLE()                	 __HAL_RCC_SPI1_CLK_ENABLE()
#define BSP_SPI1_DMAx_CLK_ENABLE()             __HAL_RCC_DMA1_CLK_ENABLE()
#define BSP_SPI1_FORCE_RESET()               	 __HAL_RCC_SPI1_FORCE_RESET()
#define BSP_SPI1_RELEASE_RESET()             	 __HAL_RCC_SPI1_RELEASE_RESET()
#define BSP_SPI1_SCK_GPIO_CLK_ENABLE()       	 __HAL_RCC_GPIOA_CLK_ENABLE()
#define BSP_SPI1_MISO_GPIO_CLK_ENABLE()      	 __HAL_RCC_GPIOA_CLK_ENABLE()
#define BSP_SPI1_MOSI_GPIO_CLK_ENABLE()      	 __HAL_RCC_GPIOA_CLK_ENABLE()

#define BSP_SPI1_SCK_PIN                     	 GPIO_PIN_5
#define BSP_SPI1_SCK_GPIO_PORT               	 GPIOA
#define BSP_SPI1_SCK_AF                      	 GPIO_AF5_SPI1
#define BSP_SPI1_MISO_PIN                    	 GPIO_PIN_6
#define BSP_SPI1_MISO_GPIO_PORT              	 GPIOA
#define BSP_SPI1_MISO_AF                     	 GPIO_AF5_SPI1
#define BSP_SPI1_MOSI_PIN                    	 GPIO_PIN_7
#define BSP_SPI1_MOSI_GPIO_PORT              	 GPIOA
#define BSP_SPI1_MOSI_AF                     	 GPIO_AF5_SPI1

/* Definition for BSP_SPI1 DMA */
#define BSP_SPI1_TX_DMA_CHANNEL              	 DMA1_Channel3
#define BSP_SPI1_RX_DMA_CHANNEL              	 DMA1_Channel2

#define BSP_SPI1_TX_DMA_REQUEST              	 DMA_REQUEST_1
#define BSP_SPI1_RX_DMA_REQUEST              	 DMA_REQUEST_1

/* Definition for BSP_SPI1 NVIC */
#define BSP_SPI1_DMA_TX_IRQn                 	 DMA1_Channel3_IRQn
#define BSP_SPI1_DMA_RX_IRQn                 	 DMA1_Channel2_IRQn

/********************************* BSP_SPI2 I/O Pin define ***************************************/
/* Definition BSP_SPI2 TX/RX buffer size */
#define BSP_SPI2_TX_BUFFER_SIZE                256
#define BSP_SPI2_RX_BUFFER_SIZE                2000

/* Definition for SPIx clock resources */
#define BSP_SPI2                             	SPI2
#define BSP_SPI2_CLK_ENABLE()                 __HAL_RCC_SPI2_CLK_ENABLE()
#define BSP_SPI2_DMAx_CLK_ENABLE()            __HAL_RCC_DMA1_CLK_ENABLE()
#define BSP_SPI2_FORCE_RESET()               	__HAL_RCC_SPI2_FORCE_RESET()
#define BSP_SPI2_RELEASE_RESET()             	__HAL_RCC_SPI2_RELEASE_RESET()

//BSP_SPI2 Pin define
#define BSP_SPI2_SCK_GPIO_CLK_ENABLE()       	__HAL_RCC_GPIOB_CLK_ENABLE()
#define BSP_SPI2_MISO_GPIO_CLK_ENABLE()      	__HAL_RCC_GPIOB_CLK_ENABLE()
#define BSP_SPI2_MOSI_GPIO_CLK_ENABLE()      	__HAL_RCC_GPIOB_CLK_ENABLE()

#define BSP_SPI2_SCK_PIN                     	GPIO_PIN_13
#define BSP_SPI2_SCK_GPIO_PORT               	GPIOB
#define BSP_SPI2_SCK_AF                      	GPIO_AF5_SPI2
#define BSP_SPI2_MISO_PIN                    	GPIO_PIN_14
#define BSP_SPI2_MISO_GPIO_PORT              	GPIOB
#define BSP_SPI2_MISO_AF                     	GPIO_AF5_SPI2
#define BSP_SPI2_MOSI_PIN                    	GPIO_PIN_15
#define BSP_SPI2_MOSI_GPIO_PORT              	GPIOB
#define BSP_SPI2_MOSI_AF                     	GPIO_AF5_SPI2

/* Definition for BSP_SPI2 DMA */
#define BSP_SPI2_TX_DMA_CHANNEL              	DMA1_Channel5
#define BSP_SPI2_RX_DMA_CHANNEL              	DMA1_Channel4

#define BSP_SPI2_TX_DMA_REQUEST              	DMA_REQUEST_1
#define BSP_SPI2_RX_DMA_REQUEST              	DMA_REQUEST_1

/* Definition for BSP_SPI2 NVIC */
#define BSP_SPI2_DMA_TX_IRQn                 	DMA1_Channel5_IRQn
#define BSP_SPI2_DMA_RX_IRQn                 	DMA1_Channel4_IRQn

/********************************* BSP_SPI3 I/O Pin define ***************************************/
/* Definition BSP_SPI3 TX/RX buffer size */
#define BSP_SPI3_TX_BUFFER_SIZE                256
#define BSP_SPI3_RX_BUFFER_SIZE                256

/* Definition for SPIx clock resources */
#define BSP_SPI3                             	 SPI3
#define BSP_SPI3_CLK_ENABLE()                	 __HAL_RCC_SPI3_CLK_ENABLE()
#define BSP_SPI3_DMAx_CLK_ENABLE()             __HAL_RCC_DMA2_CLK_ENABLE()

#define BSP_SPI3_FORCE_RESET()               	 __HAL_RCC_SPI3_FORCE_RESET()
#define BSP_SPI3_RELEASE_RESET()             	 __HAL_RCC_SPI3_RELEASE_RESET()

#define BSP_SPI3_SCK_GPIO_CLK_ENABLE()       	 __HAL_RCC_GPIOC_CLK_ENABLE()
#define BSP_SPI3_MISO_GPIO_CLK_ENABLE()      	 __HAL_RCC_GPIOC_CLK_ENABLE()
#define BSP_SPI3_MOSI_GPIO_CLK_ENABLE()      	 __HAL_RCC_GPIOC_CLK_ENABLE()

//BSP_SPI3 Pin define
#define BSP_SPI3_SCK_PIN                     	 GPIO_PIN_10
#define BSP_SPI3_SCK_GPIO_PORT               	 GPIOC
#define BSP_SPI3_SCK_AF                      	 GPIO_AF6_SPI3
#define BSP_SPI3_MISO_PIN                    	 GPIO_PIN_11
#define BSP_SPI3_MISO_GPIO_PORT              	 GPIOC
#define BSP_SPI3_MISO_AF                     	 GPIO_AF6_SPI3
#define BSP_SPI3_MOSI_PIN                    	 GPIO_PIN_12
#define BSP_SPI3_MOSI_GPIO_PORT              	 GPIOC
#define BSP_SPI3_MOSI_AF                     	 GPIO_AF6_SPI3

/* Definition for BSP_SPI3 DMA */
#define BSP_SPI3_TX_DMA_CHANNEL              	 DMA2_Channel2
#define BSP_SPI3_RX_DMA_CHANNEL              	 DMA2_Channel1

#define BSP_SPI3_TX_DMA_REQUEST              	 DMA_REQUEST_3
#define BSP_SPI3_RX_DMA_REQUEST              	 DMA_REQUEST_3

/* Definition for BSP_SPI3 NVIC */
#define BSP_SPI3_DMA_TX_IRQn                 	 DMA2_Channel2_IRQn
#define BSP_SPI3_DMA_RX_IRQn                 	 DMA2_Channel1_IRQn

/********************************* BSP_I2C I/O Pin define ****************************************/
/* SMP I2C0 clock and pin define */
/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 80 MHz */
/* This example use TIMING to 0x10A09EE9 to reach 100 KHz speed (Rise time = 10ns, Fall time = 10ns) */
#define BSP_I2C0_100K_TIMING      						0x10A09EE9
#define BSP_I2C0_250K_TIMING      						0x008031FF
#define BSP_I2C0_400K_TIMING      						0x0080298F

#define BSP_I2C0                             	I2C1
#define BSP_I2C0_CLK_ENABLE()                	__HAL_RCC_I2C1_CLK_ENABLE()
#define BSP_I2C0_SDA_GPIO_CLK_ENABLE()       	__HAL_RCC_GPIOB_CLK_ENABLE()
#define BSP_I2C0_SCL_GPIO_CLK_ENABLE()       	__HAL_RCC_GPIOB_CLK_ENABLE() 

#define BSP_I2C0_FORCE_RESET()               	__HAL_RCC_I2C1_FORCE_RESET()
#define BSP_I2C0_RELEASE_RESET()             	__HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define BSP_I2C0_SCL_PIN                    	GPIO_PIN_8
#define BSP_I2C0_SCL_GPIO_PORT              	GPIOB
#define BSP_I2C0_SCL_AF                     	GPIO_AF4_I2C1
#define BSP_I2C0_SDA_PIN                    	GPIO_PIN_7
#define BSP_I2C0_SDA_GPIO_PORT              	GPIOB
#define BSP_I2C0_SDA_AF                     	GPIO_AF4_I2C1

/* Definition for I2Cx's NVIC */
#define BSP_I2C0_EV_IRQn                    	I2C1_EV_IRQn
#define BSP_I2C0_EV_IRQHandler              	I2C1_EV_IRQHandler
#define BSP_I2C0_ER_IRQn                    	I2C1_ER_IRQn
#define BSP_I2C0_ER_IRQHandler              	I2C1_ER_IRQHandler

/********************************* BSP_CAN0 I/O Pin define ***************************************/
/* Definition BSP_CAN0 TX/RX buffer size */
#define BSP_CAN0_TX_BUFFER_SIZE                256
#define BSP_CAN0_RX_BUFFER_SIZE                1

/* Definition for BSP_CAN0 clock resources */
#define BSP_CAN0                               CAN1
#define BSP_CAN0_CLK_ENABLE()                  __HAL_RCC_CAN1_CLK_ENABLE()
#define BSP_CAN0_RX_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOA_CLK_ENABLE()
#define BSP_CAN0_TX_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOA_CLK_ENABLE()

#define BSP_CAN0_FORCE_RESET()                 __HAL_RCC_UART4_FORCE_RESET()
#define BSP_CAN0_RELEASE_RESET()               __HAL_RCC_UART4_RELEASE_RESET()

/* Definition for BSP_CAN0 Pins */
#define BSP_CAN0_TX_PIN                        GPIO_PIN_12
#define BSP_CAN0_TX_GPIO_PORT                  GPIOA
#define BSP_CAN0_TX_AF                         GPIO_AF9_CAN1
#define BSP_CAN0_RX_PIN                        GPIO_PIN_11
#define BSP_CAN0_RX_GPIO_PORT                  GPIOA
#define BSP_CAN0_RX_AF                         GPIO_AF9_CAN1

/* Definition for BSP CAN0 NVIC */
#define BSP_CAN0_RX_IRQn                       CAN1_RX0_IRQn
#define BSP_CAN0_RX_IRQHandler                 CAN1_RX0_IRQHandler

#define BSP_CAN0_TX_IRQn                       CAN1_TX_IRQn
#define BSP_CAN0_TX_IRQHandler                 CAN1_TX_IRQHandler
/********************************* BSP_CAN1 I/O Pin define ***************************************/
/* Definition BSP_CAN1 TX/RX buffer size */
#define BSP_CAN1_TX_BUFFER_SIZE                256
#define BSP_CAN1_RX_BUFFER_SIZE                1 

/* Definition for BSP CAN1 clock resources */
#define BSP_CAN1                               CAN2
#define BSP_CAN1_CLK_ENABLE()                  __HAL_RCC_CAN2_CLK_ENABLE()

#define BSP_CAN1_RX_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()
#define BSP_CAN1_TX_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()

#if 0
#define SMP_CAN1_FORCE_RESET()                 __HAL_RCC_CAN2_FORCE_RESET()
#define SMP_CAN1_RELEASE_RESET()               __HAL_RCC_CAN2_RELEASE_RESET()
#endif 

/* Definition for BSP CAN1 Pins */
#define BSP_CAN1_TX_PIN                        GPIO_PIN_6
#define BSP_CAN1_TX_GPIO_PORT                  GPIOB
#define BSP_CAN1_TX_AF                         GPIO_AF8_CAN2
#define BSP_CAN1_RX_PIN                        GPIO_PIN_5
#define BSP_CAN1_RX_GPIO_PORT                  GPIOB
#define BSP_CAN1_RX_AF                         GPIO_AF3_CAN2

/* Definition for BSP CAN1 NVIC */
#define BSP_CAN1_RX_IRQn                       CAN2_RX0_IRQn
#define BSP_CAN1_RX_IRQHandler                 CAN2_RX0_IRQHandler

#define BSP_CAN1_TX_IRQn                       CAN2_TX_IRQn
#define BSP_CAN1_TX_IRQHandler                 CAN2_TX_IRQHandler

/********************************* BSP_ADC Pin define ********************************************/																	 
/* STM32L496 ADC1 GPIO Configuration
    PC0     ------> MCU ADC1_IN1
    PC1     ------> MCU ADC1_IN2
    PC2     ------> MCU ADC1_IN3
    PC3     ------> MCU ADC1_IN4
		PA0    -------> MCU ADC1_IN5
		PA1    -------> MCU ADC1_IN6
		PA2    -------> MCU ADC1_IN7		
		PA3    -------> MCU ADC1_IN8
		PA4    -------> MCU ADC1_IN9
		PA5    -------> MCU ADC1_IN10
		PA6    -------> MCU ADC1_IN11
		PA7    -------> MCU ADC1_IN12
		PC4    -------> MCU ADC1_IN13
		PC5    -------> MCU ADC1_IN14	
		PB0    -------> MCU ADC1_IN15
		PB1    -------> MCU ADC1_IN16		
*/
	    
#define BSP_MCU_ADC_DISABLE                    0
			
#define BSP_MCU_ADC_GPIO_1                     GPIOA
#define BSP_MCU_ADC_PIN_1                      GPIO_PIN_0

#define BSP_MCU_ADC_GPIO_2                     GPIOA
#define BSP_MCU_ADC_PIN_2                      GPIO_PIN_1

#define BSP_MCU_ADC_GPIO_3                     GPIOA
#define BSP_MCU_ADC_PIN_3                      GPIO_PIN_2

#define BSP_MCU_ADC_GPIO_4                     GPIOA
#define BSP_MCU_ADC_PIN_4                      GPIO_PIN_3

#define BSP_MCU_ADC_GPIO_5                     GPIOC
#define BSP_MCU_ADC_PIN_5                      GPIO_PIN_4

#define BSP_MCU_ADC_GPIO_6                     BSP_MCU_ADC_DISABLE
#define BSP_MCU_ADC_PIN_6                      BSP_MCU_ADC_DISABLE

#define BSP_MCU_ADC_GPIO_7                     BSP_MCU_ADC_DISABLE
#define BSP_MCU_ADC_PIN_7                      BSP_MCU_ADC_DISABLE

#define BSP_MCU_ADC_GPIO_8                     BSP_MCU_ADC_DISABLE
#define BSP_MCU_ADC_PIN_8                      BSP_MCU_ADC_DISABLE												 
																	 
#define BSP_MCU_ADC_GPIO_9                     BSP_MCU_ADC_DISABLE
#define BSP_MCU_ADC_PIN_9                      BSP_MCU_ADC_DISABLE

#define BSP_MCU_ADC_GPIO_10                    BSP_MCU_ADC_DISABLE
#define BSP_MCU_ADC_PIN_10                     BSP_MCU_ADC_DISABLE

#define BSP_MCU_ADC_GPIO_11                    BSP_MCU_ADC_DISABLE
#define BSP_MCU_ADC_PIN_11                     BSP_MCU_ADC_DISABLE

#define BSP_MCU_ADC_GPIO_12                    BSP_MCU_ADC_DISABLE
#define BSP_MCU_ADC_PIN_12                     BSP_MCU_ADC_DISABLE
																	
#define BSP_MCU_ADC_GPIO_13                    BSP_MCU_ADC_DISABLE
#define BSP_MCU_ADC_PIN_13                     BSP_MCU_ADC_DISABLE

#define BSP_MCU_ADC_GPIO_14                    BSP_MCU_ADC_DISABLE
#define BSP_MCU_ADC_PIN_14                     BSP_MCU_ADC_DISABLE

#define BSP_MCU_ADC_GPIO_15                    BSP_MCU_ADC_DISABLE
#define BSP_MCU_ADC_PIN_15                     BSP_MCU_ADC_DISABLE

#define BSP_MCU_ADC_GPIO_16                    BSP_MCU_ADC_DISABLE
#define BSP_MCU_ADC_PIN_16                     BSP_MCU_ADC_DISABLE									 

// Board Internal ADC Pin name
#define BSP_TMP_RLY_1                          {.gpio_port= BSP_MCU_ADC_GPIO_1   , .gpio_pin =  BSP_MCU_ADC_PIN_1 }
#define BSP_TMP_RLY_2                          {.gpio_port= BSP_MCU_ADC_GPIO_2   , .gpio_pin =  BSP_MCU_ADC_PIN_2 }
#define BSP_TMP_RLY_AMB                        {.gpio_port= BSP_MCU_ADC_GPIO_3   , .gpio_pin =  BSP_MCU_ADC_PIN_3 }
#define BSP_TMP_RLY_BBP                        {.gpio_port= BSP_MCU_ADC_GPIO_4   , .gpio_pin =  BSP_MCU_ADC_PIN_4 }
#define BSP_TMP_RLY_BBN                        {.gpio_port= BSP_MCU_ADC_GPIO_5   , .gpio_pin =  BSP_MCU_ADC_PIN_5 }

/* Public macro -------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* _DAVINCI_BSP_REV2_H_ */

/************************ (C) COPYRIGHT *****END OF FILE****/    

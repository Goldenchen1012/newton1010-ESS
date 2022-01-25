/**
  ******************************************************************************
  * @file    smp_TLC6C5912.h 
  * @author  Golden Chen
  * @version V0.0.1
  * @date    2022/01/21
  * @brief   Header for smp_TLC65912.c module
  ******************************************************************************
  */

#ifndef __SMP_TLC6C5912_LIBRARY__
#define __SMP_TLC6C5912_LIBRARY__
#include <stdint.h>

#if 0
#define TLC6C5912_CLK_DELAY_WITH_NOP 
#endif

#define TLC6C5912_CLK_DELAY_NOP                0



typedef enum{
    LED_OFF      = 0,										         /* LED OFF */
    LED_ON                                       /* LED ON  */
}bsp_tlc6c5912_led_type;

typedef union
{
  struct
  {
    uint16_t percent20:1;                      /*!< bit:0  20%                      */
		uint16_t percent40:1;                      /*!< bit:1  40%                      */
		uint16_t percent60:1;                      /*!< bit:2  60%                      */
		uint16_t percent80:1;                      /*!< bit:3  80%                      */
		uint16_t percent100:1;                     /*!< bit:4  100%                     */
		uint16_t _reserved0_:1;                    /*!< bit:5  Reserved                 */
		uint16_t dsg:1;                            /*!< bit:6  Dischrage                */
		uint16_t chg:1;                            /*!< bit:7  Chrage                   */
		uint16_t alarm:1;                          /*!< bit:8  Alarm                    */
		uint16_t comm:1;                           /*!< bit:9  Communiaction            */		
		uint16_t _reserved:6;                      /*!< bit:10..15  Reserved            */
  } b;                                         /*!< Structure used for bit  access  */
  uint16_t w;                                  /*!< Type      used for word access  */
} LED_Display_Indicator_Type;


#define BSP_TLC6C5912_DATA_0()                 (BSP_TLC6C5912_SER_IN_PORT->ODR &= ~BSP_TLC6C5912_SER_IN_PIN)
#define BSP_TLC6C5912_DATA_1()                 (BSP_TLC6C5912_SER_IN_PORT->ODR |=  BSP_TLC6C5912_SER_IN_PIN)

#define BSP_TLC6C5912_CLK_0()                  (BSP_TLC6C5912_SRCK_PORT->ODR &= ~BSP_TLC6C5912_SRCK_PIN)
#define BSP_TLC6C5912_CLK_1()                  (BSP_TLC6C5912_SRCK_PORT->ODR |=  BSP_TLC6C5912_SRCK_PIN)

#define BSP_TLC6C5912_CS_0()                   (BSP_TLC6C5912_RCK_PORT->ODR &= ~BSP_TLC6C5912_RCK_PIN)
#define BSP_TLC6C5912_CS_1()                   (BSP_TLC6C5912_RCK_PORT->ODR |=  BSP_TLC6C5912_RCK_PIN)

void smp_TLC6C5912_Init(void);
LED_Display_Indicator_Type smp_TLC6C5912_Get_Ledstauts(void);
void smp_TLC6C5912_Set_Ledstauts(LED_Display_Indicator_Type led_status);
void smp_TLC6C5912_Display_Ledstauts(void);
void smp_TLC6C5912_OutputLed_Open(void);
void smp_TLC6C5912_SendData(uint16_t dataout);
void smp_TLC6C5912_All_LED_On(void);
void smp_TLC6C5912_All_LED_Off(void);
void smp_TLC6C5912_Percent20_LED(bsp_tlc6c5912_led_type led_status);
void smp_TLC6C5912_Percent40_LED(bsp_tlc6c5912_led_type led_status);
void smp_TLC6C5912_Percent60_LED(bsp_tlc6c5912_led_type led_status);
void smp_TLC6C5912_Percent80_LED(bsp_tlc6c5912_led_type led_status);
void smp_TLC6C5912_Percent100_LED(bsp_tlc6c5912_led_type led_status);
void smp_TLC6C5912_Discharge_LED(bsp_tlc6c5912_led_type led_status);
void smp_TLC6C5912_Charge_LED(bsp_tlc6c5912_led_type led_status);
void smp_TLC6C5912_Alarm_LED(bsp_tlc6c5912_led_type led_status);
void smp_TLC6C5912_Comm_LED(bsp_tlc6c5912_led_type led_status);

#endif // __SMP_TLC6C5912_LIBRARY__

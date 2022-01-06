/**
  ******************************************************************************
  * @file        HalAfeAds7946.h
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/19
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef HAL_AFE_ADS7946_H_
#define HAL_AFE_ADS7946_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/
	
/* Public macro -------------------------------------------------------------*/


/* Public variables ---------------------------------------------------------*/
extern int32_t	ads_7945_adcValue[4];

/* Public function prototypes -----------------------------------------------*/
void HalAfeCurrentSetAdcValue(uint8_t adc_index, int32_t adcvalue);

void halAfeCurrentOpen(void);


#ifdef __cplusplus
}
#endif

#endif /* HAL_AFE_ADS7946_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    

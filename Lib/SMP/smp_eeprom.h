/**
  ******************************************************************************
  * @file    smp_eeprom.h 
  * @author  SMP Randy
  * @version V0.0.1
  * @date    23-December-2016
  * @brief   Header for smp_eeprom.c module
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMP_EEPROM_H
#define __SMP_EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/ 
/* Exported defines --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int8_t smp_eeprom_open(void);
int8_t smp_eeprom_write(uint16_t addr, uint16_t val);
int8_t smp_eeprom_read(uint16_t addr, uint16_t *val);

#endif /* __SMP_EEPROM_H */

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

/**
  ******************************************************************************
  * @file        ApiRamData.h
  * @author      Johnny
  * @version     v0.0.0
  * @date        2021/11/03
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _API_RAM_DATA_H_
#define _API_RAM_DATA_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/

/* Public macro -------------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
void apiRamOpen(void);

void apiRamSaveRtcMagicCode(uint16_t MagicCode);
uint16_t apiRamLoadRtcMagicCode(void);
void apiRamSaveSoc(uint16_t soc);
uint16_t apiRamLoadSoc(void);
void apiRamSaveTotalDisChargeCount(uint32_t count);
uint32_t apiRamLoadTotalDisChargeCount(void);

void apiRamSaveLastChgDhgTime(void);
uint32_t apiRamLoadReleaseTime(void);
void apiRamSaveRtcDateTime(void);
uint32_t apiRamLoadPowerOffDateTime(void);


#ifdef __cplusplus
}
#endif


	

#endif /* _API_RAM_DATA_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



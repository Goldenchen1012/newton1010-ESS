/**
  ******************************************************************************
  * @file        HalBsp.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/10/28
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */
#ifndef _HALI_BSP_H_
#define _HALI_BSP_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/

/* Public macro -------------------------------------------------------------*/
void HalBspInit(void);

void halBspPreDischargeRelayOn(void);
void halBspPreDischargeRelayOff(void);

void HalBslK1Ctrl(uint8_t hi);
void HalBspK2Ctrl(uint8_t hi);


void halBspFanRelayOn(void);
void halBspFanRelayOff(void);

void halBspPostiveRelayOn(void);
void halBspPostiveRelayOff(void);
void halBspNegtiveRelayOn(void);
void halBspNegtiveRelayOff(void);

void HalBspRelayPsCtrl(uint8_t hi);
void HalBspReleaseCtrl(uint8_t hi);


//-----------------------------------
//	Input
uint8_t HalBspGetDi1Status(void);
uint8_t HalBspGetDi2Status(void);
uint8_t HalBspGetEpoStatus(void);
uint8_t HalBspGetSpStatus(void);
uint8_t HalBspGetPs1Status(void);
uint8_t HalBspGetPs2Status(void);
uint8_t HalBspGetPs3Status(void);
uint8_t HalBspGetButtonStatus(void);
uint8_t HalBspGetK1Status(void);
uint8_t HalBspGetK2Status(void);
uint8_t HalBspGetK3Status(void);
uint8_t HalBspGetK4Status(void);
uint8_t HalBspGetDocpLatchStatus(void);
uint8_t HalBspGetCocpLatchStatus(void);
uint8_t HalBspGetOdInStatus(void);
uint8_t halBspGetNfaultStatus(void);
char *halBspGetGpioControlMsg(uint8_t group, uint32_t mask);
void halBspGpioControl(uint8_t group, uint32_t mask, uint32_t dat);


#ifdef __cplusplus
}
#endif


	

#endif /* _HALI_BSP_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


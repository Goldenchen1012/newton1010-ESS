/**
  ******************************************************************************
  * @file        AppProject.h
  * @author      Johnny
  * @version     v0.0
  * @date        2021/9/7
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _APP_PROJECT_H_
#define _APP_PROJECT_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
#define	APP_PROJECT_SIMU_MODE 	0x01
#define	APP_PROJECT_ENG_MODE	0x02

uint8_t appProjectIsSystemReadyFlag(void);
uint8_t appProjectIsInSimuMode(void);
void appProjectEnableSimuMode(void);
void appProjectDisableSimuMode(void);
uint8_t appProjectIsInEngMode(void);
void appProjectEnableEngMode(void);
void appProjectDisableEngMode(void);
uint8_t appProjectGetScuId(void);

void appProjectOpen(void);
uint8_t	appProjectIsRtcValid(void);
void appProjcetGetIrValue(uint32_t *Rp, uint32_t *Rn);


/* Public macro -------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif


	

#endif /* _APP_PROJECT_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



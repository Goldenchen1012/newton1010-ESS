/**
  ******************************************************************************
  * @file        LibProtect.h
  * @author      Norman
  * @version     v1.0
  * @date        2019/9/11
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Norman</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef LIB_PROTECT_H_
#define LIB_PROTECT_H_

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "LibRegister.h"
	
//#if 0	
/* Public define ------------------------------------------------------------*/
#define tLibProtectEvtHandler tLibRegisterEvtHandler
/* Public typedef -----------------------------------------------------------*/
// typedef union {
//   struct  {
//     uint8_t ov:1; 
//     uint8_t uv:1; 
//     uint8_t oc:1; 
//     uint8_t ot:1; 
//     uint8_t ut:1; 
//   } bits;                              
//   uint8_t b;                           
// } tLibProtectEnable;

// typedef union{
//   struct  {
//     uint64_t c1:2; 
//     uint64_t c2:2; 
//     uint64_t c3:2; 
//     uint64_t c4:2; 
//     uint64_t c5:2; 
//     uint64_t c6:2; 
//     uint64_t c7:2; 
//     uint64_t c8:2; 
//     uint64_t c9:2; 
//     uint64_t c10:2; 
//     uint64_t c11:2; 
//     uint64_t c12:2; 
//     uint64_t c13:2; 
//     uint64_t c14:2; 
//     uint64_t c15:2; 
//     uint64_t c16:2; 
//     uint64_t c17:2; 
//     uint64_t c18:2; 
//     uint64_t c19:2; 
//     uint64_t c20:2; 
//     uint64_t c21:2; 
//     uint64_t c22:2; 
//     uint64_t c23:2; 
//     uint64_t c24:2; 
//     uint64_t c25:2; 
//     uint64_t c26:2; 
//     uint64_t c27:2; 
//     uint64_t c28:2; 
//     uint64_t c29:2; 
//     uint64_t c30:2; 
//     uint64_t c31:2; 
//     uint64_t c32:2; 
//   } bits;                              
//   uint64_t u64;                           
// } tLibProtectOvState, tLibProtectUvState;

enum {
  LIB_PROTECT_EVT_LOW_VBAT = 0
};
/* Public macro -------------------------------------------------------------*/

/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/

// void LibProtectOvOpen(tLibProtectEvtHandler handler);
// void LibProtectOvClose(void);
// #endif

void LibProtectOpen(tLibSwTimerEvtHandler handler);

#ifdef __cplusplus
}
#endif

#endif /* LIB_AFE_H_ */

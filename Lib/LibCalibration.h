/**
  ******************************************************************************
  * @file        LibCalibration.h
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/14
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _LIB_CALIBRATION_H_
#define _LIB_CALIBRATION_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Public define ------------------------------------------------------------*/
typedef struct {  
    int32_t A1;    
    int32_t A2;  
    int32_t B;
} tCalibCoef;
/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
int32_t doCalibration(tCalibCoef *par, int32_t dataX);
tCalibCoef calCoef(int32_t valL, int32_t adcL, int32_t valH, int32_t adcH);



#ifdef __cplusplus
}
#endif

#endif /* _LIB_CALIBRATION_H_ */

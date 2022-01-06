/**
  ******************************************************************************
  * @file        LibAvgWeight.h
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

#ifndef _LIB_AVG_WEIGHT_H
#define _LIB_AVG_WEIGHT_H

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

	
/* Public define ------------------------------------------------------------*/
	
/* Public typedef -----------------------------------------------------------*/
typedef struct {
    int32_t *buffer;
	uint8_t bufNum;
	uint8_t window;
}tLibAvgWeightNumI32;

typedef struct {
    int32_t *buffer;
	uint8_t window;
}tLibAvgWeightI32;
/* Public macro -------------------------------------------------------------*/
#define LIB_AVG_WEIGHT_NUM_I32_CREATE(name, win, bufNum) \
    static int32_t name##_buffer[bufNum] = {0}; \
    const tLibAvgWeightNumI32 name = {name##_buffer, bufNum, win};

#define LIB_AVG_WEIGHT_I32_CREATE(name, win) \
    static int32_t name##_buffer = 0; \
    const tLibAvgWeightI32 name = {&name##_buffer, win};
    
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
int32_t libAvgWeightExeNumI32(const tLibAvgWeightNumI32 *cont, uint8_t bufNum, int32_t dataIn);
int32_t libAvgWeightExeI32(const tLibAvgWeightI32 *cont, int32_t dataIn);

#ifdef __cplusplus
}
#endif

#endif /* _LIB_AVG_WEIGHT_H */

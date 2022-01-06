/**
  ******************************************************************************
  * @file        LibAvgWeight.c
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

/* Includes ------------------------------------------------------------------*/
#include "LibAvgWeight.h"

#include <stdint.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
int32_t libAvgWeightExeNumI32(const tLibAvgWeightNumI32 *cont,uint8_t bufNum, int32_t dataIn){
	int32_t avgRes;	
	if(cont->window <= 1){
		return dataIn;
	}
	
	if(bufNum >= cont->bufNum){
		return dataIn;
	}
	
	if(cont->buffer[bufNum] == 0){
		cont->buffer[bufNum] = dataIn * (int32_t)cont->window;
		avgRes = dataIn;
	}else{
	    cont->buffer[bufNum] += dataIn;
	    avgRes = cont->buffer[bufNum] / (int32_t)cont->window;
	}
	cont->buffer[bufNum] -= avgRes;
	return avgRes;
}

int32_t libAvgWeightExeI32(const tLibAvgWeightI32 *cont, int32_t dataIn){
	int32_t avgRes;	
	if(cont->window <= 1){
		return dataIn;
	}
	
	if(*cont->buffer == 0){
		*cont->buffer = dataIn * (int32_t)cont->window;
		avgRes = dataIn;
	}else{
	    *cont->buffer += dataIn;
	    avgRes = *cont->buffer / (int32_t)cont->window;
	}
	*cont->buffer -= avgRes;
	return avgRes;
}

/************************ (C) COPYRIGHT Johnny *****END OF FILE****/    

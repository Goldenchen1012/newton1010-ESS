/**
  ******************************************************************************
  * @file        LibCalibration.c
  * @author      Johnny
  * @version     v0.0
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

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>
#include "LibCalibration.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
int32_t doCalibration(tCalibCoef *par, int32_t dataX){ //Y = A1*X + A2*X/10000 + B
	int32_t dataTemp;
	dataTemp = dataX * par->A1;
	dataTemp += (dataX * par->A2) / 100000;
	dataTemp += par->B;
	return dataTemp;
}

tCalibCoef calCoef(int32_t valL, int32_t adcL, int32_t valH, int32_t adcH){
    tCalibCoef coef;
    double y1, y2, x1, x2;
    double cA, cB;
    
    if(valL == valH){
        coef.A1 = 0;
        coef.A2 = 0;
        coef.B = 0;
    }else{
        y1 = valL;
        y2 = valH;
        x1 = adcL;
        x2 = adcH;
        cA = (y1 - y2) / (x1 - x2);
        cB = y1 - (cA * x1);
        coef.A1 = (int32_t)cA;
        coef.A2 = (int32_t)((cA - (double)coef.A1) * 100000);
        coef.B = (int32_t)cB;
    }
#if 0    
    {
    	char	str[100];
    	sprintf(str,"A1=%d A2=%d B =%d",
    			coef.A1,	coef.A2,coef.B);
    		
    	appSerialCanDavinciSendTextMessage(str);
    }
#endif    
    return coef;
}


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


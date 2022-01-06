/**
  ******************************************************************************
  * @file        LibNtc.c
  * @author      Johnny 
  * @version     v0.0
  * @date        2021/10/25
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
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define	B_VALUE		3435.0
#define	R25_VALUE	10000.0

/*
-40 ~ - 15 := 53752		25157*EXP(-0.05*A2)
-14 ~ +6:		21196	27509*EXP(-0.044*A28)
7 ~ 30 :		8305	26583*EXP(-0.039*A49)
31 ~ 45 : 		4902	23608*EXP(-0.035*A73)
46 ~ 66 : 		2512	20598*EXP(-0.032*A88)
67 ~ 83			1534	15818*EXP(-0.028*A109)
84 ~ 105 		857		13137*EXP(-0.026*A126)

*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
uint16_t LibSetRealTemperatureToInternalValue(int16_t temp)
{
	temp += 40;
	if(temp < 0)
		temp = 0;
	if(temp > 160)
		temp = 160;
	return (uint16_t)temp;
}
uint16_t LibTemperatureToVoltage(int16_t temp)
{
	uint16_t	n;

	double	NtcR;
    double  d1,d2;
    
	temp -= (int16_t)40;		//溫度設定值Offset -40 度
	
	if(temp <= -15)		//-40 ~ - 15 := 25157*EXP(-0.05*A2)
	{
		d1 = (double)temp;
		d1 *= (-5.0);
		d1 /= 100.0;
		d2 = exp(d1);
		NtcR = d2 * 25157.0;
	}
	else if(temp <= 6 )	//-14 ~ +6:27509*EXP(-0.044*A28)
	{
		d1 = (double)temp;
		d1 *= (-44.0);
		d1 /= 1000.0;
		d2 = exp(d1);
		NtcR = d2 * 27509.0;
	}
	else if(temp <= 30 )	//7 ~ 30 :26583*EXP(-0.039*A49)
	{
		d1 = (double)temp;
		d1 *= (-39.0);
		d1 /= 1000.0;
		d2 = exp(d1);
		NtcR = d2 * 26583.0;
	}
	else if(temp <= 45 )	//31 ~ 45 : = 23608*EXP(-0.035*A73)
	{
		d1 = (double)temp;
		d1 *= (-35.0);
		d1 /= 1000.0;
		d2 = exp(d1);
		NtcR = d2 * 23608.0;
	}
	else if(temp <= 66 )	//46 ~ 66 : 20598*EXP(-0.032*A88)
	{
		d1 = (double)temp;
		d1 *= (-32.0);
		d1 /= 1000.0;
		d2 = exp(d1);
		NtcR = d2 * 20598.0;
	}
	else if(temp <= 83 )	//67 ~ 83		=15818*EXP(-0.028*A109)
	{
		d1 = (double)temp;
		d1 *= (-28.0);
		d1 /= 1000.0;
		d2 = exp(d1);
		NtcR = d2 * 15818.0;
	}
	else //if(temp <= 83 )	//84 ~ 105 :=13137*EXP(-0.026*A126)
	{
		d1 = (double)temp;
		d1 *= (-26.0);
		d1 /= 1000.0;
		d2 = exp(d1);
		NtcR = d2 * 13137.0;
	}
	
    d1 = (5000.0 * NtcR);
    d1 /= (NtcR + R25_VALUE);
	n = (uint16_t)d1;
    return n;
}

uint16_t LibNtcRToTemperature(double NtcR)
{
	uint16_t	dd;

    double  d1,d2,d3;   
    
    if(NtcR >= 193278) 			// < -40 
    {
    	d1 = -40.0;
	}
    else if(NtcR >= 53752.0)	//-40 ~ - 15 53752	25157*EXP(-0.05*A2)
    {
		d2 = NtcR / 25157.0;
   		d3 = (double)log(d2);
	    d1 = d3*(-1000.0 / 50.0);   
    }	
    else if(NtcR >= 21196.0)	//-14 ~ +6 	21196	27509*EXP(-0.044*A28)
    {
		d2 = NtcR / 27509.0;
   		d3 = (double)log(d2);
	    d1 = d3*(-1000.0 / 44.0);   
    }
    
    else if(NtcR >= 8305.0)	//7 ~ 30 8305	26583*EXP(-0.039*A49)
    {
		d2 = NtcR / 26583.0;
   		d3 = (double)log(d2);
	    d1 = d3*(-1000.0 / 39.0);
    }
    else if(NtcR >= 4902.0)	//31 ~ 45 4902	23608*EXP(-0.035*A73)
	{
		d2 = NtcR / 23608.0;
   		d3 = (double)log(d2);
	    d1 = d3*(-1000.0 / 35.0);
    }	
    else if(NtcR >= 2512.0)	//46 ~ 66 2512	20598*EXP(-0.032*A88)
	{
		d2 = NtcR / 20598.0;
   		d3 = (double)log(d2);
	    d1 = d3*(-1000.0 / 32.0);
    }
    else if(NtcR >= 1534.0)	//67 ~ 83	1534	15818*EXP(-0.028*A109)
	{
		d2 = NtcR / 15818.0;
   		d3 = (double)log(d2);
	    d1 = d3*(-1000.0 / 28.0);
    }
    else //if(NtcR >= 857)	//84 ~ 105 	857	13137*EXP(-0.026*A126)
	{
		d2 = NtcR / 13137.0;
   		d3 = (double)log(d2);
	    d1 = d3*(-1000.0 / 26.0);
    }
	d1 *= 100.0;
	d1 += 4000.0;
	if(d1<0.0)	
		d1=0;
   	dd = (uint16_t)d1;
    if(dd >= 20000)
    	dd = 20000;

    return	dd;		
}

#define	BAT_NTC_R_BASE		(double)10000.0
uint16_t LibNtcVoltageToTemperature(uint16_t NtcVoltage)
{
	uint16_t	dd;
    double  NtcR;
    double  Adc;

	Adc = (double)NtcVoltage;

    if(Adc >= 5000.0) 
    	Adc = 5000.0;   //R2/R1=V2/V1 => R2xV1=R1xV2
    NtcR = BAT_NTC_R_BASE * Adc;             //R2=(R1xV2)/V1
    
    if(Adc < 5000.0)
    	NtcR /= (5000.0 - Adc);		//計算出來的 NTC 電阻值
    else
    	NtcR = 300000.0;    
	dd = LibNtcRToTemperature(NtcR);	
    return	dd;
}

/************************ (C) COPYRIGHT Johnny *****END OF FILE****/    

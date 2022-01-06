/**
  ******************************************************************************
  * @file        HalAfe.c
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/12
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
#include "halafe.h"
#include "LibSwTimer.h"
#include "LibNtc.h"
#include "HalAfeBq796xx.h"
#include "ApiSysPar.h"

/* Private define ------------------------------------------------------------*/
#define		afeCellNumber()		apiSysParGetCellNumber()
#define		afeNtcNumber()		apiSysParGetNtcNumber()
/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

typedef struct{
	tCellVoltage	CellVoltage[MAX_CELL_NUMBER + 4];
	tNtcAdcData		NtcAdcData[MAX_NTC_NUMBER + 4];
	tCurrent		CurrentValue[2];
	int32_t			CurrentAdcValue[2];
	uint32_t		VBat[2];
	int32_t			VbatAdcValue[2];
	uint16_t		CellNumber;
	uint16_t		MaxCellVoltage;
	uint16_t		MinCellVoltage;
	uint16_t		MaxTempNtcVoltage;
	uint16_t		MinTempNtcVoltage;
}tAfeBuffer;

tAfeBuffer	AfeBuffer;
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
int32_t	halAfeGetCurrentAdcValue(uint8_t CurrentIndex)
{
	if(CurrentIndex >= 2)
		return 0;
	return AfeBuffer.CurrentAdcValue[CurrentIndex];
}

void halAfeSetCurrentAdcValue(uint8_t CurrentIndex,int32_t adcvalue)
{
	if(CurrentIndex >= 2)
		return;
	AfeBuffer.CurrentAdcValue[CurrentIndex] = adcvalue;
}
int32_t	halAfeGetVBatAdcValue(uint8_t VbIndex)
{
	if(VbIndex >= 2)
		return 0;
	return AfeBuffer.VbatAdcValue[VbIndex];
}
void halAfeSetVBatAdcValue(uint8_t VbIndex, int32_t adcvalue)
{
	if(VbIndex >= 2)
		return;
	AfeBuffer.VbatAdcValue[VbIndex] = adcvalue;
}

void halAfeSetCellVoltage(uint16_t cell, tCellVoltage voltage)
{
	AfeBuffer.CellVoltage[cell] = voltage;
}
void halAfeSetNtcAdcData(uint16_t ntcs, tNtcAdcData adcdata)
{
	AfeBuffer.NtcAdcData[ntcs] = adcdata;
}

tBatteryVoltage HalAfeGetBatteryVoltage(void)
{
	tBatteryVoltage	vbat = 0;
	uint16_t		cell;
		 
	for(cell=0; cell<AfeBuffer.CellNumber; cell++)
	{
		vbat += AfeBuffer.CellVoltage[cell];
	}
	return vbat;
}

tCellVoltage halAfeGetCellVoltage(uint16_t CellIndex)
{
	return AfeBuffer.CellVoltage[CellIndex];	
}

tNtcAdcData HalAfeGetNtcAdc(uint16_t NtcIndex)
{
	return AfeBuffer.NtcAdcData[NtcIndex];	
}

tCurrent halAfeGetCurrentValue(uint8_t index)
{
	return AfeBuffer.CurrentValue[index];
}
void HalAfeSetCurrentValue(uint8_t index, int32_t current)
{
	AfeBuffer.CurrentValue[index] = current;
}

uint32_t halAfeGetVBatVoltage(uint8_t index)
{
	return AfeBuffer.VBat[index];
}
void halAfeSetVBatVoltage(uint8_t index, uint32_t voltage)
{
	AfeBuffer.VBat[index] = voltage;
}


tCellVoltage halAfeGetMaxCellVoltage(void)
{
	return AfeBuffer.MaxCellVoltage;
}

tCellVoltage halAfeGetMinCellVoltage(void)
{
	return AfeBuffer.MinCellVoltage;
}

tNtcAdcData HalAfeGetMinNtcTempAdc(void)
{
	return AfeBuffer.MinTempNtcVoltage;
}
tNtcAdcData HalAfeGetMaxNtcTempAdc(void)
{
	return AfeBuffer.MaxTempNtcVoltage;
}


void halAfeUpdateMinMaxCellVoltage(void)
{
	uint16_t	cell;
	tCellVoltage	cv;
	
	AfeBuffer.MaxCellVoltage = 0;
	AfeBuffer.MinCellVoltage = 0xffff;
	
	for(cell=0; cell<afeCellNumber(); cell++)
	{
		cv  = halAfeGetCellVoltage(cell);
		if(cv > AfeBuffer.MaxCellVoltage)
		{		
			AfeBuffer.MaxCellVoltage = cv;
		}
		if(cv < AfeBuffer.MinCellVoltage)
		{		
			AfeBuffer.MinCellVoltage = cv;
		}
	}
}
void halAfeUpdateMinMaxNtcTempVoltage(void)
{
	uint16_t	ntc;
	uint16_t	cv;
	
	AfeBuffer.MinTempNtcVoltage = 0;
	AfeBuffer.MaxTempNtcVoltage = 0xffff;
	
	for(ntc=0; ntc<afeNtcNumber(); ntc++)
	{
		cv = HalAfeGetNtcAdc(ntc);
		if(cv < AfeBuffer.MaxTempNtcVoltage)
		{		
			AfeBuffer.MaxTempNtcVoltage = cv;
		}
		if(cv > AfeBuffer.MinTempNtcVoltage)
		{		
			AfeBuffer.MinTempNtcVoltage = cv;
		}
	}
}

//tNtcAdcData HalAfeGetNtcAdc(uint16_t NtcIndex)

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



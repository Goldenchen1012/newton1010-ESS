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
#define		afeBmuNumber()		apiSysParGetBmuNumber()
#define		afeCellFlag(bmu)	apiSysParGetCellFlag(bmu)
#define		afeNtcFlag(bmu)		apiSysParGetNtcFlag(bmu)

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

typedef struct{
	tCellVoltage	CellVoltage[MAX_CELL_NUMBER + 4];
	tNtcVoltage		NtcVoltage[MAX_NTC_NUMBER + 4];
	tCurrent		CurrentValue[2];
	int32_t			CurrentAdcValue[2];
	uint32_t		VBat[2];
	int32_t			VbatAdcValue[2];
	uint16_t		MaxCellVoltage;
	uint16_t		MinCellVoltage;
	uint16_t		MaxTempNtcVoltage;
	uint16_t		MinTempNtcVoltage;
	uint16_t		MaxNtcTemp;
	uint16_t		MinNtcTemp;
	struct{
		uint8_t		Bmu;
		uint8_t		Channel;
	}MaxVPosition;
	struct{
		uint8_t		Bmu;
		uint8_t		Channel;
	}MinVPosition;
	struct{
		uint8_t		Bmu;
		uint8_t		Channel;
	}MaxTPosition;
	struct{
		uint8_t		Bmu;
		uint8_t		Channel;
	}MinTPosition;
	
}tAfeBuffer;

tAfeBuffer	AfeBuffer;
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


static void logicalCellPositionToPhysicalPosition(uint16_t posi,uint8_t *retbmu, uint8_t *retch)
{	
	uint16_t	index = 0;
	uint8_t		bmu;
	uint32_t	flag;
	uint8_t		i;
	for(bmu=0; bmu<afeBmuNumber(); bmu++)
	{
		flag = afeCellFlag(bmu);
		for(i=0; i<32; i++)
		{
			if(flag & 0x01)
			{
				if(index == posi)
				{
					*retbmu = bmu + 1;
					*retch = i + 1;
					return;
				}
				index++;
			}
			flag >>= 1;
		}		
	}
}
	
	
static void logicalNtcPositionToPhysicalPosition(uint16_t posi,uint8_t *retbmu, uint8_t *retch)
{	
	uint16_t	index = 0;
	uint8_t		bmu;
	uint32_t	flag;
	uint8_t		i;
	for(bmu=0; bmu<afeBmuNumber(); bmu++)
	{
		flag = afeNtcFlag(bmu);
		for(i=0; i<32; i++)
		{
			if(flag & 0x01)
			{
				if(index == posi)
				{
					*retbmu = bmu + 1;
					*retch = i + 1;
					return;
				}
				index++;
			}			
			flag >>= 1;
		}		
	}
}
	
	
/* Public function prototypes -----------------------------------------------*/
void halAfeCalVbatFromCellVoltage(void)
{
	uint16_t	cell;
	uint32_t	vbat = 0;
	if(appProjectIsInSimuMode())
		return;
		
	for(cell = 0; cell<afeCellNumber(); cell++)
	{
		vbat += halAfeGetCellVoltage(cell);
	}
	halAfeSetVBatVoltage(AFE_VBAT_INDEX, vbat);
}
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
void halAfeSetNtcVoltage(uint16_t ntcs, tNtcVoltage voltage)
{
	AfeBuffer.NtcVoltage[ntcs] = voltage;
}

tCellVoltage halAfeGetCellVoltage(uint16_t CellIndex)
{
	return AfeBuffer.CellVoltage[CellIndex];	
}

tNtcVoltage HalAfeGetNtcVoltage(uint16_t NtcIndex)
{
	return AfeBuffer.NtcVoltage[NtcIndex];	
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


tCellVoltage halAfeGetMaxCellVoltage(uint8_t *bmu, uint8_t *posi)
{
	if(bmu)
		*bmu = AfeBuffer.MaxVPosition.Bmu;
	if(posi)
		*posi = AfeBuffer.MaxVPosition.Channel;
	
	return AfeBuffer.MaxCellVoltage;
}

tCellVoltage halAfeGetMinCellVoltage(uint8_t *bmu, uint8_t *posi)
{
	if(bmu)
		*bmu = AfeBuffer.MinVPosition.Bmu;
	if(posi)
		*posi = AfeBuffer.MinVPosition.Channel;
		
	return AfeBuffer.MinCellVoltage;
}

uint16_t HalAfeGetMinNtcTemp(uint8_t *bmu, uint8_t *posi)
{
	if(bmu)
		*bmu = AfeBuffer.MinTPosition.Bmu;
	if(posi)
		*posi = AfeBuffer.MinTPosition.Channel;

	return AfeBuffer.MinNtcTemp;
}
uint16_t HalAfeGetMaxNtcTemp(uint8_t *bmu, uint8_t *posi)
{
	if(bmu)
		*bmu = AfeBuffer.MaxTPosition.Bmu;
	if(posi)
		*posi = AfeBuffer.MaxTPosition.Channel;

	return AfeBuffer.MaxNtcTemp;
}

void halAfeUpdateMinMaxCellVoltage(void)
{
	uint16_t	cell;
	tCellVoltage	cv;
	uint16_t	MaxPosition = 0;
	uint16_t	MinPosition = 0;
	
	AfeBuffer.MaxCellVoltage = 0;
	AfeBuffer.MinCellVoltage = 0xffff;
	
	for(cell=0; cell<afeCellNumber(); cell++)
	{
		cv  = halAfeGetCellVoltage(cell);
		if(cv > AfeBuffer.MaxCellVoltage)
		{		
			AfeBuffer.MaxCellVoltage = cv;
			MaxPosition = cell;
		}
		if(cv < AfeBuffer.MinCellVoltage)
		{		
			AfeBuffer.MinCellVoltage = cv;
			MinPosition = cell;
		}
	}

	logicalCellPositionToPhysicalPosition(MaxPosition,
						&AfeBuffer.MaxVPosition.Bmu,
						&AfeBuffer.MaxVPosition.Channel);
	logicalCellPositionToPhysicalPosition(MinPosition,
						&AfeBuffer.MinVPosition.Bmu,
						&AfeBuffer.MinVPosition.Channel);
#if 1
	{						
		char	str[100];
		
		sprintf(str,"MaxV = %d %d %d, MinV=%d %d %d",
				MaxPosition,
				AfeBuffer.MaxVPosition.Bmu,
				AfeBuffer.MaxVPosition.Channel,
				MinPosition,
				AfeBuffer.MinVPosition.Bmu,
				AfeBuffer.MinVPosition.Channel
				);
		appSerialCanDavinciSendTextMessage(str);
	}
#endif
}
void halAfeUpdateMinMaxNtcTempVoltage(void)
{
	uint16_t	ntc;
	uint16_t	ntcv;
	uint16_t	MaxPosition = 0;
	uint16_t	MinPosition = 0;
	
	
	AfeBuffer.MinTempNtcVoltage = 0;
	AfeBuffer.MaxTempNtcVoltage = 0xffff;
	
	for(ntc=0; ntc<afeNtcNumber(); ntc++)
	{
		ntcv = HalAfeGetNtcVoltage(ntc);
		if(ntcv < AfeBuffer.MaxTempNtcVoltage)
		{		
			AfeBuffer.MaxTempNtcVoltage = ntcv;
			MaxPosition =  ntc;
		}
		if(ntcv > AfeBuffer.MinTempNtcVoltage)
		{		
			AfeBuffer.MinTempNtcVoltage = ntcv;
			MinPosition = ntc;
		}
	}

	logicalNtcPositionToPhysicalPosition(MaxPosition,
						&AfeBuffer.MaxTPosition.Bmu,
						&AfeBuffer.MaxTPosition.Channel);
	logicalNtcPositionToPhysicalPosition(MinPosition,
						&AfeBuffer.MinTPosition.Bmu,
						&AfeBuffer.MinTPosition.Channel);
			
	AfeBuffer.MaxNtcTemp = LibNtcVoltageToTemperature(AfeBuffer.MaxTempNtcVoltage);
	AfeBuffer.MinNtcTemp = LibNtcVoltageToTemperature(AfeBuffer.MinTempNtcVoltage);
	
#if 1
	{
		char	str[100];
		sprintf(str,"MaxT = %d %d %d, MinT=%d %d %d",
				MaxPosition,
				AfeBuffer.MaxTPosition.Bmu,
				AfeBuffer.MaxTPosition.Channel,
				MinPosition,
				AfeBuffer.MinTPosition.Bmu,
				AfeBuffer.MinTPosition.Channel
				);
		appSerialCanDavinciSendTextMessage(str);
	}
#endif	
}


/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



/**
  ******************************************************************************
  * @file        LibFaultLog.c
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
#include "fault.h"
#include "NoClearFault.h"
#include "LibSwTimer.h"

#include <stdint.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void libFaultLogTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr){
    if(evt == LIB_SW_TIMER_EVT_SW_1S){
		if(FaultRecBuf.FaultIdleTime){
			FaultRecBuf.FaultIdleTime--;
			if(!FaultRecBuf.FaultIdleTime){
				SaveFaultDataToSpiRom(FALSE);
			}
		}

        if(NoClearFaultRecBuf.FaultIdleTime){
			NoClearFaultRecBuf.FaultIdleTime--;
			if(!NoClearFaultRecBuf.FaultIdleTime)			{				
				NoClear_SaveFaultDataToSpiRom(FALSE);
			}
		}

	    if(NoClearFaultRecBuf.FaultIdleTime == 0 
		    && FaultRecBuf.FaultIdleTime == 0){
			LibSwTimerClose(libFaultLogTimerHandler, 0);
		}
	}
}

/* Public function prototypes -----------------------------------------------*/
void SaveFaultData(uint8_t type, uint16_t cell){
	LibSwTimerOpen(libFaultLogTimerHandler, 0);
	__SaveFaultData(type, cell);
}

void NoClear_SaveFaultData(uint8_t type, uint16_t cell){
	LibSwTimerOpen(libFaultLogTimerHandler, 0);
	__NoClear_SaveFaultData(type, cell);
}

void libFaultLogOpen(void){
	LoadFaultLogAddress();
	InitialFaultData();
	NoClear_InitialFaultData();
	if(!FaultAddress.IsValidAddress)
	{
		SaveFaultLogAddress(TRUE, 0x20);
	}
}

/************************ (C) COPYRIGHT Johnny *****END OF FILE****/    

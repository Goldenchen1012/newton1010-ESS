/**
  ******************************************************************************
  * @file        LibCellBalance.c
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
#include "AN49503.h"
#include "balancecheck.h"
#include "LibSwTimer.h"
#include <stdint.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LIB_CELL_BALANCE_STATE_DISABLE 0
#define LIB_CELL_BALANCE_STATE_ENABLE 1
#define LIB_CELL_BALANCE_STATE_STOP 2
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t CellBalanceState = LIB_CELL_BALANCE_STATE_DISABLE;
bool runBalanceTimeFlag = true;
/* Private function prototypes -----------------------------------------------*/
void libCellBalanceTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr){
    if(evt == LIB_SW_TIMER_EVT_SW_500MS){
		if(runBalanceTimeFlag == true){
			runBalanceTimeFlag = false;
		    CheckBalanceSet(0);
		}else{
            runBalanceTimeFlag = true;
		    RunBalance();
		}
    }else if(evt == LIB_SW_TIMER_EVT_SW_100MS){
	    if(CellBalanceState == LIB_CELL_BALANCE_STATE_ENABLE){
			if(BalanceSwitch){
				SendBalanceData();
			}else{
				libAfeSetBalance(0);			
			}
		}
	}
}

/* Public function prototypes -----------------------------------------------*/


void libCellBalanceStop(void){
	if(CellBalanceState == LIB_CELL_BALANCE_STATE_ENABLE){
		CellBalanceState = LIB_CELL_BALANCE_STATE_STOP;
	    libAfeSetBalance(0);
	}
}

void libCellBalanceStart(void){
	if(CellBalanceState == LIB_CELL_BALANCE_STATE_STOP ){
		CellBalanceState = LIB_CELL_BALANCE_STATE_ENABLE;	
		if(BalanceSwitch) 
		{
			SendBalanceData();
		}	
	}
}

bool libCellBalanceIsBalancing(void){
    if(CellBalanceState == LIB_CELL_BALANCE_STATE_ENABLE){
		if(BalanceSwitch) 
		{
            return true;
		}
	}
	return false;
}

void libCellBalanceOpen(void){
	if(CellBalanceState == LIB_CELL_BALANCE_STATE_DISABLE){
		libAfeSetBalance(0);
	    CellBalanceState = LIB_CELL_BALANCE_STATE_ENABLE;
	    LibSwTimerOpen(libCellBalanceTimerHandler, 0);
	}
}

void libCellBalanceClose(void){
	CellBalanceState = LIB_CELL_BALANCE_STATE_DISABLE;
	LibSwTimerClose(libCellBalanceTimerHandler, 0);
	libCellBalanceStop();
}
/************************ (C) COPYRIGHT Johnny *****END OF FILE****/    

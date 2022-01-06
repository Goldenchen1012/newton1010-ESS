/**
  ******************************************************************************
  * @file        LibStateMachine.c
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


//#include "LibRegister.h"
#include "LibStateMachine.h"

#include <stdint.h>
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void LibStateMachineEvtExe(const tLibStateMachine *pStateMachine, uint16_t evt, __far void *vDataPtr){
  
  if(pStateMachine->nowMember == 0){
    return;
  }
  if(pStateMachine->nowMember[0]->handler == 0){
    return;
  }
  
  pStateMachine->nowMember[0]->handler(pStateMachine->dest[0], evt, vDataPtr);
}

void LibStateMachineSetState(const tLibStateMachine *pStateMachine, uint8_t newState){
	uint8_t num;
  
    if(pStateMachine->nowMember[0]->id == newState){
        return;
    }
	
    for(num = 0; num < 0xFF; num++){
		if(pStateMachine->memberTab[num].id == 0xFF){
		    return;	
		}
		if(pStateMachine->memberTab[num].id == newState){
		    break;	
		}
	}
	
    if(pStateMachine->nowMember[0]->id != 0xFF){
        pStateMachine->nowMember[0]->handler(pStateMachine->dest[0], STATE_SYS_EVT_UNINITIAL, 0);
    }	
    pStateMachine->nowMember[0] = (const tLibStateMachineMember *)&pStateMachine->memberTab[num];
    pStateMachine->nowMember[0]->handler(pStateMachine->dest[0], STATE_SYS_EVT_INITIAL, 0);
}

void LibStateMachineSetStateInit(const tLibStateMachine *pStateMachine, __far void *dest, uint8_t initState){
	uint8_t num;
	
    pStateMachine->dest[0] = dest;
	
	for(num = 0; num < 0xFF; num++){
		if(pStateMachine->memberTab[num].id == 0xFF){
		    break;	
		}
		if(pStateMachine->memberTab[num].id == initState){
		    break;	
		}
	}
    pStateMachine->nowMember[0] = (const tLibStateMachineMember *)&pStateMachine->memberTab[num];
    pStateMachine->nowMember[0]->handler(pStateMachine->dest[0], STATE_SYS_EVT_INITIAL, 0);
}

uint8_t LibStateMachineGetState(const tLibStateMachine *pStateMachine){
	return pStateMachine->nowMember[0]->id;
}

bool LibStateMachineIsInit(const tLibStateMachine *pStateMachine){
	if(pStateMachine->nowMember[0] == 0){
		return false;
	}else{
		return true;
	}
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    

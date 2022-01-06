/**
  ******************************************************************************
  * @file        LibRegister.c
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

#include "sdk_config.h"
//#include "define.h"
#include "LibRegister.h"
#include "LibDebug.h"
//#include "uart.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static tLibRegisterMember mRegisterRecord[LIB_REGISTER_RECORD_MAX] = {0};
/* Private function prototypes -----------------------------------------------*/

static __far tLibRegisterMember *recordMalloc(void){
#if (LIB_REGISTER_RECORD_DYNAMIC_ENABLE == 1)
    return (__far tLibRegisterMember *)malloc(sizeof(tLibRegisterMember));
#else
    uint8_t u8;

    for(u8=0;u8<LIB_REGISTER_RECORD_MAX;u8++){
        if(mRegisterRecord[u8].handler == 0){
            break;
        }
    }

    if(u8 >= LIB_REGISTER_RECORD_MAX){
        return 0;
    }

    return (__far tLibRegisterMember *)&mRegisterRecord[u8];
#endif
}

static void recordFree(__far tLibRegisterMember *record){
#if (LIB_REGISTER_RECORD_DYNAMIC_ENABLE == 1)
    free(record);
#else
    uint8_t u8;

    for(u8=0;u8<LIB_REGISTER_RECORD_MAX;u8++){
        if(record == (__far tLibRegisterMember *)&mRegisterRecord[u8]){
            break;
        }
    }

    if(u8 < LIB_REGISTER_RECORD_MAX){
        mRegisterRecord[u8].handler = 0;
    }
#endif

}

tErrCode _LibRegisterAdd(__far tLibRegister *head, tLibRegisterEvtHandler handler, __far void *dest){
    __far tLibRegisterMember *nowPtr, *nextPtr;
//	char	str[100];
	
	if(head == 0 || handler == 0){
        return RES_ERROR_INVALID_PARAM;
    }
        
    nowPtr = 0;
    nextPtr = head->next;
    
    //Find Next
    while(nextPtr != 0){
        if((nextPtr->handler == handler) && (nextPtr->dest == dest)){
			if((head->executing->handler == handler) && (head->executing->dest == dest) && (head->removeExecutingHandlerFlag == true)){
			    head->removeExecutingHandlerFlag = false;
			}
            return RES_ERROR_REINIT;
        }
        nowPtr = nextPtr;
        nextPtr = nextPtr->next;
    }
    
    //Allocate memory
    nextPtr = recordMalloc();
 
    if(nextPtr == 0){
 //       NRF_LOG_INFO("SMP_ERROR_MALLOC----------------------\r\n");
        assert_param(0);
        return RES_ERROR_MALLOC;
    }
 //   NRF_LOG_INFO("Add 0x08%X\r\n",(uint32_t)nextPtr);

// sprintf(str,"nextPtr 0x%08X \r\n",nextPtr);
//	SendUartString((BYTE *)str);
 
    //Assign
    nextPtr->handler = handler;    
    nextPtr->dest = dest;    
    nextPtr->next=0;
    if(nowPtr == 0){//Cheak head
        head->next = nextPtr;
    }else{
        nowPtr->next = nextPtr;
    }
    
    return RES_SUCCESS;
}

tErrCode _LibRegisterRm(__far tLibRegister *head, tLibRegisterEvtHandler handler, __far void *dest){ //When "head" is 0,this API will remove all member.
    __far tLibRegisterMember *nowPtr, *nextPtr;
	
	if(head==0 || handler==0){
        return RES_ERROR_INVALID_PARAM;
    }

	//Do not kill self.
	if((head->executing->handler == handler) && (head->executing->dest == dest)){
		head->removeExecutingHandlerFlag = true;
		return RES_SUCCESS;
	}
	
    nowPtr=0;
    nextPtr=head->next;
    
    //Find Next
    while(nextPtr!=0){
        if((nextPtr->handler==handler) && (nextPtr->dest == dest)){
            break;
        }
        nowPtr=nextPtr;
        nextPtr=nextPtr->next;
    }

    if(nextPtr==0){
        return RES_ERROR_NOT_FOUND;
    }else{
        //free memory
        if(nowPtr==0){ //Head
            head->next=nextPtr->next;
        }else{
            nowPtr->next=nextPtr->next;
        }
        recordFree(nextPtr);
    }

    return RES_SUCCESS;
}    

tErrCode _LibRegisterTypeHandlerExe(__far tLibRegister *head, uint16_t evt, __far void * data){
    __far tLibRegisterMember *nowPtr, *lastPtr;
	if(head->next==0){
        return RES_ERROR_NULL;
    }

    nowPtr = head->next;
    while(nowPtr!=0){
        head->executing = nowPtr;
        head->executing->handler(nowPtr->dest, evt, data);    
		head->executing = 0;
		lastPtr = nowPtr;		
        nowPtr = nowPtr->next;
		
		if(head->removeExecutingHandlerFlag == true){
		    _LibRegisterRm(head, lastPtr->handler, lastPtr->dest);
			head->removeExecutingHandlerFlag = false;
		}
    }
    
    return RES_SUCCESS;

}

__far tLibRegisterMember *_LibRegisterGetMemberAddr(__far tLibRegister *head, uint16_t number){
    __far tLibRegisterMember *nowPtr;
	uint8_t i=0;
	
    if(head==0 || head->next==0){
        return 0;
    }
    nowPtr=head->next;

    //Find member
    for(i=0;i<number;i++){
        if(nowPtr->next==0){
            return 0;
        }
        nowPtr=nowPtr->next;
    }
    return nowPtr;
}

bool _LibRegisterIsMemberNull(__far tLibRegister *head){
  if(head->next == 0){
      return true;
  }else{
      return false;
  }
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    

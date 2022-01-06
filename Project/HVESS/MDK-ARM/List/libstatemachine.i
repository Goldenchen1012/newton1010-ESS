#line 1 "..\\..\\..\\Lib\\LibStateMachine.c"














 

 



#line 1 "..\\..\\..\\Lib\\LibStateMachine.h"







 
#line 1 "..\\..\\..\\Lib\\LibRegister.h"




#line 1 "..\\..\\..\\Config_Common\\sdk_config.h"














 





















































































































































#line 171 "..\\..\\..\\Config_Common\\sdk_config.h"






















































#line 6 "..\\..\\..\\Lib\\LibRegister.h"
#line 1 "..\\..\\..\\Lib\\LibDebug.h"



#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 5 "..\\..\\..\\Lib\\LibDebug.h"



 
#line 26 "..\\..\\..\\Lib\\LibDebug.h"




 

#line 7 "..\\..\\..\\Lib\\LibRegister.h"

#line 9 "..\\..\\..\\Lib\\LibRegister.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdbool.h"
 






 





#line 25 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdbool.h"



#line 10 "..\\..\\..\\Lib\\LibRegister.h"








typedef void (* tLibRegisterEvtHandler)( void *dest, uint16_t evt,  void *data);

typedef struct 
{
	 void *dest;
    tLibRegisterEvtHandler handler;
     void *next;
}tLibRegisterMember;

typedef struct 
{
     tLibRegisterMember *next;
	 tLibRegisterMember *executing;
	_Bool removeExecutingHandlerFlag;
}tLibRegister;


int8_t _LibRegisterAdd( tLibRegister *head, tLibRegisterEvtHandler handler,  void *dest); 
int8_t _LibRegisterRm( tLibRegister *head,tLibRegisterEvtHandler handler,  void *dest); 
int8_t _LibRegisterTypeHandlerExe( tLibRegister *head, uint16_t evt,  void * data);
 tLibRegisterMember *_LibRegisterGetMemberAddr( tLibRegister *head, uint16_t number);
_Bool _LibRegisterIsMemberNull( tLibRegister *head);


#line 10 "..\\..\\..\\Lib\\LibStateMachine.h"
#line 11 "..\\..\\..\\Lib\\LibStateMachine.h"
#line 12 "..\\..\\..\\Lib\\LibStateMachine.h"
 



 
typedef struct{
  uint8_t id;
  tLibRegisterEvtHandler handler;
}tLibStateMachineMember;

typedef struct{
   void **dest;
  const tLibStateMachineMember *memberTab;
  const tLibStateMachineMember **nowMember;
}tLibStateMachine;

typedef void (* tLibStateMachineEvtHandler)( void *dest, uint16_t evt,  void *data);
  









 
 

void LibStateMachineEvtExe(const tLibStateMachine *pStateMachine, uint16_t evt,  void *vDataPtr);
void LibStateMachineSetState(const tLibStateMachine *pStateMachine, uint8_t newState);
void LibStateMachineSetStateInit(const tLibStateMachine *pStateMachine,  void *dest, uint8_t initState);
uint8_t LibStateMachineGetState(const tLibStateMachine *pStateMachine);
_Bool LibStateMachineIsInit(const tLibStateMachine *pStateMachine);





#line 22 "..\\..\\..\\Lib\\LibStateMachine.c"

#line 24 "..\\..\\..\\Lib\\LibStateMachine.c"
 

 

 
 
 
void LibStateMachineEvtExe(const tLibStateMachine *pStateMachine, uint16_t evt,  void *vDataPtr){
  
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
        pStateMachine->nowMember[0]->handler(pStateMachine->dest[0], 1, 0);
    }	
    pStateMachine->nowMember[0] = (const tLibStateMachineMember *)&pStateMachine->memberTab[num];
    pStateMachine->nowMember[0]->handler(pStateMachine->dest[0], 0, 0);
}

void LibStateMachineSetStateInit(const tLibStateMachine *pStateMachine,  void *dest, uint8_t initState){
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
    pStateMachine->nowMember[0]->handler(pStateMachine->dest[0], 0, 0);
}

uint8_t LibStateMachineGetState(const tLibStateMachine *pStateMachine){
	return pStateMachine->nowMember[0]->id;
}

_Bool LibStateMachineIsInit(const tLibStateMachine *pStateMachine){
	if(pStateMachine->nowMember[0] == 0){
		return 0;
	}else{
		return 1;
	}
}

     

#line 1 "..\\..\\..\\Lib\\LibSwTimer.c"














 

 
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




 

#line 19 "..\\..\\..\\Lib\\LibSwTimer.c"
#line 1 "..\\..\\..\\Lib\\LibRegister.h"




#line 1 "..\\..\\..\\Config_Common\\sdk_config.h"














 





















































































































































#line 171 "..\\..\\..\\Config_Common\\sdk_config.h"






















































#line 6 "..\\..\\..\\Lib\\LibRegister.h"
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


#line 20 "..\\..\\..\\Lib\\LibSwTimer.c"
#line 1 "..\\..\\..\\Lib\\LibSwTimer.h"














 








 
#line 26 "..\\..\\..\\Lib\\LibSwTimer.h"
#line 27 "..\\..\\..\\Lib\\LibSwTimer.h"
#line 28 "..\\..\\..\\Lib\\LibSwTimer.h"

 

 
typedef enum {
  LIB_SW_TIMER_EVT_SW_10MS_0 = 0,
  LIB_SW_TIMER_EVT_SW_10MS_1,  
  LIB_SW_TIMER_EVT_SW_10MS_2,  
  LIB_SW_TIMER_EVT_SW_10MS_3,  
  LIB_SW_TIMER_EVT_SW_10MS_4,  
  LIB_SW_TIMER_EVT_SW_10MS_5,  
  LIB_SW_TIMER_EVT_SW_10MS_6,  
  LIB_SW_TIMER_EVT_SW_10MS_7,  
  LIB_SW_TIMER_EVT_SW_10MS_8,  
  LIB_SW_TIMER_EVT_SW_10MS_9,  
  LIB_SW_TIMER_EVT_SW_1MS,  
  LIB_SW_TIMER_EVT_SW_100MS,
  LIB_SW_TIMER_EVT_SW_500MS,
  LIB_SW_TIMER_EVT_SW_1S,
  
  LIB_SW_TIMER_EVT_SW_TASK,
  
  LIB_SW_TIMER_EVT_HW_1MS,
  LIB_SW_TIMER_EVT_HW_5MS,

} tLibSwTimerEvt;

 
 
 
uint16_t LibGetSwTimer(void);
void LibSwTimerClearCount(void);
int8_t LibSwTimerOpen(tLibRegisterEvtHandler handler,  void *dest);
int8_t LibSwTimerClose(tLibRegisterEvtHandler handler,  void *dest);
void LibSwTimerHwHandler(tLibSwTimerEvt evt,  void *data);
void LibSwTimerHandle(void);
void LibSwTimerHwDelay(uint16_t ms);

int8_t LibSwTimerTaskOpen(tLibRegisterEvtHandler handler,  void *dest);
int8_t LibSwTimerTaskClose(tLibRegisterEvtHandler handler,  void *dest);





#line 21 "..\\..\\..\\Lib\\LibSwTimer.c"

#line 23 "..\\..\\..\\Lib\\LibSwTimer.c"
 
 
 
 
static uint16_t count1ms = 0;
static uint16_t count1s = 0;
static uint16_t delayCount = 0;


static tLibRegister EvtHandlerRegister={0}, EvtHandlerRegisterTask={0};
 
uint16_t LibGetSwTimer(void)
{
	return count1ms;
}

void LibSwTimerClearCount(void)
{
	count1ms = 0;
	count1s = 0;
	delayCount = 0;
}


int8_t LibSwTimerOpen(tLibRegisterEvtHandler handler,  void *dest){
static _Bool initFlag = 0; 
  if(_LibRegisterIsMemberNull(( tLibRegister *)&EvtHandlerRegister) == 1){
	  if(initFlag == 1){
          count1ms = 0;
          count1s = 0;
	  }
  }
  
  if(initFlag == 0){
	  initFlag = 1;  
  }
  return _LibRegisterAdd(( tLibRegister *)&EvtHandlerRegister, handler, ( void *)dest);
}

int8_t LibSwTimerClose(tLibRegisterEvtHandler handler,  void *dest){
  return _LibRegisterRm(( tLibRegister *)&EvtHandlerRegister, handler, ( void *)dest);
}

void LibSwTimerHwHandler(tLibSwTimerEvt evt,  void *data){
  if(evt == LIB_SW_TIMER_EVT_HW_1MS){
    count1ms++;
    if(delayCount > 0){
      delayCount--;
    }
  }else if(evt == LIB_SW_TIMER_EVT_HW_5MS){
    count1ms+=5;
    if(delayCount > 5){
      delayCount--;
    }else{
      delayCount = 0;
    }
  }
}

void LibSwTimerHwDelay(uint16_t ms){
 
    delayCount = ms;
    while(delayCount >0);
}

void LibSwTimerHandle(void){
	uint8_t i;
	
  
  {
    if(count1ms > 0){

		
		count1ms = 0;
		if(_LibRegisterIsMemberNull(( tLibRegister *)&EvtHandlerRegister) == 1){
		return;
		}

		count1s++;
		_LibRegisterTypeHandlerExe(( tLibRegister *)&EvtHandlerRegister, LIB_SW_TIMER_EVT_SW_1MS, ( void *)0);
		_LibRegisterTypeHandlerExe(( tLibRegister *)&EvtHandlerRegister, count1s%10, ( void *)0);
	
		if(count1s%100 == 0){
		  _LibRegisterTypeHandlerExe(( tLibRegister *)&EvtHandlerRegister, LIB_SW_TIMER_EVT_SW_100MS, ( void *)0);
		}
		if(count1s%500 == 0){
		  _LibRegisterTypeHandlerExe(( tLibRegister *)&EvtHandlerRegister, LIB_SW_TIMER_EVT_SW_500MS, ( void *)0);
		} 

		if(count1s >= 1000){
		_LibRegisterTypeHandlerExe(( tLibRegister *)&EvtHandlerRegister, LIB_SW_TIMER_EVT_SW_1S, ( void *)0);
		count1s -= 1000;
		}
	}
  }
}

int8_t LibSwTimerTaskOpen(tLibRegisterEvtHandler handler,  void *dest){
  return _LibRegisterAdd(( tLibRegister *)&EvtHandlerRegisterTask, handler, ( void *)dest);
}

int8_t LibSwTimerTaskClose(tLibRegisterEvtHandler handler,  void *dest){
  return _LibRegisterRm(( tLibRegister *)&EvtHandlerRegisterTask, handler, ( void *)dest);
}

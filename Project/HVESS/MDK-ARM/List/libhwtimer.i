#line 1 "..\\..\\..\\Lib\\LibHwTimer.c"














 

 
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




 

#line 19 "..\\..\\..\\Lib\\LibHwTimer.c"
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


#line 20 "..\\..\\..\\Lib\\LibHwTimer.c"
#line 1 "..\\..\\..\\Lib\\LibHwTimer.h"














 








 
#line 26 "..\\..\\..\\Lib\\LibHwTimer.h"
#line 27 "..\\..\\..\\Lib\\LibHwTimer.h"
#line 28 "..\\..\\..\\Lib\\LibHwTimer.h"

 

 
typedef enum {
  LIB_HW_TIMER_EVT_1MS = 0,
} tLibHwTimerEvt;

 
 
 
void LibHwTimerHandle(void);
int8_t LibHwTimerOpen(tLibRegisterEvtHandler handler,  void *dest);
int8_t LibHwTimerClose(tLibRegisterEvtHandler handler,  void *dest);




#line 21 "..\\..\\..\\Lib\\LibHwTimer.c"

#line 23 "..\\..\\..\\Lib\\LibHwTimer.c"
 
 
 
 
static tLibRegister HwTimerEvtHandlerRegister;
 

int8_t LibHwTimerOpen(tLibRegisterEvtHandler handler,  void *dest){
static _Bool initFlag = 0; 
  if(_LibRegisterIsMemberNull(( tLibRegister *)&HwTimerEvtHandlerRegister) == 1){
	  if(initFlag == 1){

 
	  }
  }
  
  if(initFlag == 0){
	  initFlag = 1;  
  }
  return _LibRegisterAdd(( tLibRegister *)&HwTimerEvtHandlerRegister, handler, ( void *)dest);
}

int8_t LibHwTimerClose(tLibRegisterEvtHandler handler,  void *dest){
  return _LibRegisterRm(( tLibRegister *)&HwTimerEvtHandlerRegister, handler, ( void *)dest);
}

void LibHwTimerHandle(void){
	_LibRegisterTypeHandlerExe(( tLibRegister *)&HwTimerEvtHandlerRegister, LIB_HW_TIMER_EVT_1MS, ( void *)0);
}


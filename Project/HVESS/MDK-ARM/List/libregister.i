#line 1 "..\\..\\..\\Lib\\LibRegister.c"














 

 

#line 1 "..\\..\\..\\Config_Common\\sdk_config.h"














 





















































































































































#line 171 "..\\..\\..\\Config_Common\\sdk_config.h"






















































#line 20 "..\\..\\..\\Lib\\LibRegister.c"

#line 1 "..\\..\\..\\Lib\\LibRegister.h"




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


#line 22 "..\\..\\..\\Lib\\LibRegister.c"
#line 23 "..\\..\\..\\Lib\\LibRegister.c"


#line 26 "..\\..\\..\\Lib\\LibRegister.c"
#line 27 "..\\..\\..\\Lib\\LibRegister.c"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
 
 
 




 
 



 






   














  


 








#line 54 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


  



    typedef unsigned int size_t;    
#line 70 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"






    



    typedef unsigned short wchar_t;  
#line 91 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"

typedef struct div_t { int quot, rem; } div_t;
    
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
    

typedef struct lldiv_t { long long quot, rem; } lldiv_t;
    


#line 112 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
   



 

   




 
#line 131 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
   


 
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);

   




 

   




 




extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) long long atoll(const char *  ) __attribute__((__nonnull__(1)));
   



 


extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

















 

extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

 

extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



























 
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   


























 

 
extern __declspec(__nothrow) long long strtoll(const char * __restrict  ,
                                  char ** __restrict  , int  )
                          __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) unsigned long long strtoull(const char * __restrict  ,
                                            char ** __restrict  , int  )
                                   __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) int rand(void);
   







 
extern __declspec(__nothrow) void srand(unsigned int  );
   






 

struct _rand_state { int __x[57]; };
extern __declspec(__nothrow) int _rand_r(struct _rand_state *);
extern __declspec(__nothrow) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __declspec(__nothrow) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __declspec(__nothrow) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);
   


 

extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
   



 
extern __declspec(__nothrow) void free(void *  );
   





 
extern __declspec(__nothrow) void *malloc(size_t  );
   



 
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
   













 

extern __declspec(__nothrow) int posix_memalign(void **  , size_t  , size_t  );
   









 

typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
   










 
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
   







 

extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
   




 
#line 436 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
   












 

extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
   







      

extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
   









 

extern __declspec(__nothrow) int  system(const char *  );
   









 

extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
   












 
#line 524 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
   









 

#line 553 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"

extern __declspec(__nothrow) __attribute__((const)) int abs(int  );
   



 

extern __declspec(__nothrow) __attribute__((const)) div_t div(int  , int  );
   









 
extern __declspec(__nothrow) __attribute__((const)) long int labs(long int  );
   



 




extern __declspec(__nothrow) __attribute__((const)) ldiv_t ldiv(long int  , long int  );
   











 







extern __declspec(__nothrow) __attribute__((const)) long long llabs(long long  );
   



 




extern __declspec(__nothrow) __attribute__((const)) lldiv_t lldiv(long long  , long long  );
   











 
#line 634 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"




 
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
    
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;

__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
   

 




 
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
   







 























 
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
   












 
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
   















 
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
   













 





 
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 

extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);

extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);











#line 892 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"





 
#line 28 "..\\..\\..\\Lib\\LibRegister.c"
 

 
 
 
static tLibRegisterMember mRegisterRecord[50] = {0};
 

static  tLibRegisterMember *recordMalloc(void){



    uint8_t u8;

    for(u8=0;u8<50;u8++){
        if(mRegisterRecord[u8].handler == 0){
            break;
        }
    }

    if(u8 >= 50){
        return 0;
    }

    return ( tLibRegisterMember *)&mRegisterRecord[u8];

}

static void recordFree( tLibRegisterMember *record){



    uint8_t u8;

    for(u8=0;u8<50;u8++){
        if(record == ( tLibRegisterMember *)&mRegisterRecord[u8]){
            break;
        }
    }

    if(u8 < 50){
        mRegisterRecord[u8].handler = 0;
    }


}

int8_t _LibRegisterAdd( tLibRegister *head, tLibRegisterEvtHandler handler,  void *dest){
     tLibRegisterMember *nowPtr, *nextPtr;

	
	if(head == 0 || handler == 0){
        return ((int8_t)(0x00) - 3);
    }
        
    nowPtr = 0;
    nextPtr = head->next;
    
    
    while(nextPtr != 0){
        if((nextPtr->handler == handler) && (nextPtr->dest == dest)){
			if((head->executing->handler == handler) && (head->executing->dest == dest) && (head->removeExecutingHandlerFlag == 1)){
			    head->removeExecutingHandlerFlag = 0;
			}
            return ((int8_t)(0x00) - 20);
        }
        nowPtr = nextPtr;
        nextPtr = nextPtr->next;
    }
    
    
    nextPtr = recordMalloc();
 
    if(nextPtr == 0){
 
        ((void)0U);
        return ((int8_t)(0x00) - 17);
    }
 



 
    
    nextPtr->handler = handler;    
    nextPtr->dest = dest;    
    nextPtr->next=0;
    if(nowPtr == 0){
        head->next = nextPtr;
    }else{
        nowPtr->next = nextPtr;
    }
    
    return ((int8_t)(0x00) - 0);
}

int8_t _LibRegisterRm( tLibRegister *head, tLibRegisterEvtHandler handler,  void *dest){ 
     tLibRegisterMember *nowPtr, *nextPtr;
	
	if(head==0 || handler==0){
        return ((int8_t)(0x00) - 3);
    }

	
	if((head->executing->handler == handler) && (head->executing->dest == dest)){
		head->removeExecutingHandlerFlag = 1;
		return ((int8_t)(0x00) - 0);
	}
	
    nowPtr=0;
    nextPtr=head->next;
    
    
    while(nextPtr!=0){
        if((nextPtr->handler==handler) && (nextPtr->dest == dest)){
            break;
        }
        nowPtr=nextPtr;
        nextPtr=nextPtr->next;
    }

    if(nextPtr==0){
        return ((int8_t)(0x00) - 1);
    }else{
        
        if(nowPtr==0){ 
            head->next=nextPtr->next;
        }else{
            nowPtr->next=nextPtr->next;
        }
        recordFree(nextPtr);
    }

    return ((int8_t)(0x00) - 0);
}    

int8_t _LibRegisterTypeHandlerExe( tLibRegister *head, uint16_t evt,  void * data){
     tLibRegisterMember *nowPtr, *lastPtr;
	if(head->next==0){
        return ((int8_t)(0x00) - 11);
    }

    nowPtr = head->next;
    while(nowPtr!=0){
        head->executing = nowPtr;
        head->executing->handler(nowPtr->dest, evt, data);    
		head->executing = 0;
		lastPtr = nowPtr;		
        nowPtr = nowPtr->next;
		
		if(head->removeExecutingHandlerFlag == 1){
		    _LibRegisterRm(head, lastPtr->handler, lastPtr->dest);
			head->removeExecutingHandlerFlag = 0;
		}
    }
    
    return ((int8_t)(0x00) - 0);

}

 tLibRegisterMember *_LibRegisterGetMemberAddr( tLibRegister *head, uint16_t number){
     tLibRegisterMember *nowPtr;
	uint8_t i=0;
	
    if(head==0 || head->next==0){
        return 0;
    }
    nowPtr=head->next;

    
    for(i=0;i<number;i++){
        if(nowPtr->next==0){
            return 0;
        }
        nowPtr=nowPtr->next;
    }
    return nowPtr;
}

_Bool _LibRegisterIsMemberNull( tLibRegister *head){
  if(head->next == 0){
      return 1;
  }else{
      return 0;
  }
}

     

#line 1 "..\\..\\..\\Lib\\SMP\\smp_fifo_flash.c"















 

 
#line 1 "..\\..\\..\\Lib\\SMP\\smp_fifo_flash.h"








 
  
 



 
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






 
#line 17 "..\\..\\..\\Lib\\SMP\\smp_fifo_flash.h"

 
typedef enum{
	SMP_FLASH_EVENT_READ_DONE = 0,
	SMP_FLASH_EVENT_WRITE_DONE,
	SMP_FLASH_EVENT_ERASE_DONE,
	SMP_FLASH_EVENT_BUSY,
	SMP_FLASH_EVENT_ERROR
}smp_flash_evt_type;
typedef void (*smp_flash_event_t)(smp_flash_evt_type p_evt);

typedef struct{
	uint8_t	command;
	uint8_t	addr[3];
	uint8_t	Dummy;
	uint16_t  R_W_bytes;
	uint8_t page_buffer[256];
	uint8_t * read_buffer;
	smp_flash_event_t flash_callback;
}smp_flash_package;

typedef struct{
	smp_flash_package		*buffer_addr;	 
	uint16_t	buffer_size;	 
	uint32_t	in;				 
	uint32_t	out;			 
}smp_fifo_flash_t;

  
 
 
 
int8_t smp_fifo_flash_open(smp_fifo_flash_t *p_fifo);
int8_t smp_fifo_flash_push(smp_fifo_flash_t *p_fifo, smp_flash_package *pFlashPkg);
int8_t smp_fifo_flash_pop(smp_fifo_flash_t *p_fifo, smp_flash_package *pFlashPkg);
int8_t smp_fifo_flash_read(smp_fifo_flash_t *p_fifo, smp_flash_package *pFlashPkg);
int8_t smp_fifo_flash_back(smp_fifo_flash_t *p_fifo);
int8_t smp_fifo_flash_check(smp_fifo_flash_t *p_fifo, char *val, uint32_t index);
int8_t smp_fifo_flash_get_size(smp_fifo_flash_t *p_fifo, uint16_t *size);
int8_t smp_fifo_flash_clean(smp_fifo_flash_t *p_fifo);





 



 



 

 
#line 20 "..\\..\\..\\Lib\\SMP\\smp_fifo_flash.c"
#line 1 "..\\..\\..\\User\\smp_debug.h"








 




 
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdarg.h"
 
 
 





 










#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdarg.h"








 

 
 
#line 57 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdarg.h"
    typedef struct __va_list { void *__ap; } va_list;

   






 


   










 


   















 




   

 


   




 



   





 







#line 138 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdarg.h"



#line 147 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdarg.h"

 

#line 16 "..\\..\\..\\User\\smp_debug.h"
 
 
 
 
 
 

 





	
#line 50 "..\\..\\..\\User\\smp_debug.h"

#line 21 "..\\..\\..\\Lib\\SMP\\smp_fifo_flash.c"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"


  



    typedef unsigned int size_t;    
#line 54 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"




extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 193 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 209 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 232 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 247 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 270 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 502 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"



 

#line 22 "..\\..\\..\\Lib\\SMP\\smp_fifo_flash.c"
 
 
 
 
 
 

int8_t smp_fifo_flash_open(smp_fifo_flash_t *p_fifo)
{
	if(p_fifo!=0){
		smp_fifo_flash_clean(p_fifo);
	}else{
		return ((0x0) - 11);
	}	
	
	return ((0x0) - 0);
}

int8_t smp_fifo_flash_push(smp_fifo_flash_t *p_fifo, smp_flash_package *pFlashPkg)
{
	if((p_fifo->in+1)%p_fifo->buffer_size==p_fifo->out){
		return ((0x0) - 19); 
	}else{
		memcpy(&p_fifo->buffer_addr[p_fifo->in] ,pFlashPkg, sizeof(smp_flash_package)); 
		p_fifo->in=(p_fifo->in+1)%p_fifo->buffer_size;
	}
	return ((0x0) - 0);
}

int8_t smp_fifo_flash_pop(smp_fifo_flash_t *p_fifo, smp_flash_package *pFlashPkg)
{
	if(p_fifo->in==p_fifo->out){
		return ((0x0) - 14); 
	}else{
		memcpy(pFlashPkg, &p_fifo->buffer_addr[p_fifo->out], sizeof(smp_flash_package)); 
		p_fifo->out=(p_fifo->out+1)%p_fifo->buffer_size;
	}		
	return ((0x0) - 0);
}

int8_t smp_fifo_flash_read(smp_fifo_flash_t *p_fifo, smp_flash_package *pFlashPkg)
{
	if(p_fifo->in==p_fifo->out){
		return ((0x0) - 14); 
	}else{
		memcpy(pFlashPkg, &p_fifo->buffer_addr[p_fifo->out], sizeof(smp_flash_package)); 
	}		
	return ((0x0) - 0);
}
#line 94 "..\\..\\..\\Lib\\SMP\\smp_fifo_flash.c"
int8_t smp_fifo_flash_get_size(smp_fifo_flash_t *p_fifo, uint16_t *size)
{
  if(p_fifo->in==p_fifo->out){
		*size = 0;
		return ((0x0) - 14); 
	}else if(p_fifo->in>p_fifo->out){
		*size = p_fifo->in-p_fifo->out;
	}else{
		*size = (p_fifo->buffer_size-p_fifo->out)+(p_fifo->in);
	}	
	return ((0x0) - 0);
}

int8_t smp_fifo_flash_clean(smp_fifo_flash_t *p_fifo)
{
	memset(p_fifo->buffer_addr,0,p_fifo->buffer_size);
	p_fifo->in=0;
	p_fifo->out=0;	
	return ((0x0) - 0);
}



 

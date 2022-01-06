#line 1 "..\\..\\..\\AppProtect\\ApiProtectDocp.c"














 

 
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






 
#line 19 "..\\..\\..\\AppProtect\\ApiProtectDocp.c"
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





 
#line 20 "..\\..\\..\\AppProtect\\ApiProtectDocp.c"
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



 

#line 21 "..\\..\\..\\AppProtect\\ApiProtectDocp.c"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 22 "..\\..\\..\\AppProtect\\ApiProtectDocp.c"
#line 1 "..\\..\\..\\User\\define.h"














 







typedef unsigned char			BYTE;
typedef	char					CHAR;
typedef	unsigned short int		WORD;
typedef short int				INT;
typedef unsigned long int		DWORD;
typedef long int				LONG;
typedef unsigned char			BOOL;
typedef unsigned long long int 	DDWORD;
typedef long long int			DLONG;

typedef union{
	BYTE	b[2];
	WORD	i;
	INT		si;
}tIbyte;
	
typedef union{
	BYTE	b[4];
	WORD	i[2];
	DWORD	l;
	LONG	sl;
}tLbyte;

typedef union{
	BYTE	b[8];
	WORD	i[4];
	DWORD	l[2];
	DDWORD  ll;
	DLONG	sll;
}tLLbyte;











#line 23 "..\\..\\..\\AppProtect\\ApiProtectDocp.c"
#line 1 "..\\..\\..\\HAL\\halafe.h"














 



 
#line 21 "..\\..\\..\\HAL\\halafe.h"
#line 1 "..\\..\\..\\Lib\\LibRegister.h"




#line 1 "..\\..\\..\\Config_Common\\sdk_config.h"














 





















































































































































#line 171 "..\\..\\..\\Config_Common\\sdk_config.h"






















































#line 6 "..\\..\\..\\Lib\\LibRegister.h"
#line 1 "..\\..\\..\\Lib\\LibDebug.h"



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


#line 22 "..\\..\\..\\HAL\\halafe.h"





enum{
	AFE_STATE_NORMAL = 0,
	AFE_STATE_INI,
};
	
enum{
	AFE_EVT_COMM_L1_SET = 1,
	AFE_EVT_COMM_L1_RELEASE,
	AFE_EVT_COMM_L2_SET,
	AFE_EVT_COMM_L2_RELEASE,
	AFE_EVT_END
};




 









int32_t	halAfeGetCurrentAdcValue(uint8_t CurrentIndex);
void halAfeSetCurrentAdcValue(uint8_t CurrentIndex,int32_t adcvalue);
int32_t	halAfeGetVBatAdcValue(uint8_t VbIndex);
void halAfeSetVBatAdcValue(uint8_t VbIndex,int32_t adcvalue);

uint16_t halAfeGetCellVoltage(uint16_t CellIndex);
int32_t halAfeGetCurrentValue(uint8_t index);
uint16_t HalAfeGetNtcAdc(uint16_t NtcIndex);
void halAfeSetCellVoltage(uint16_t cell, uint16_t voltage);
void halAfeSetNtcAdcData(uint16_t ntcs, uint16_t adcdata);
void HalAfeSetCurrentValue(uint8_t index, int32_t current);
uint32_t halAfeGetVBatVoltage(uint8_t index);
void halAfeSetVBatVoltage(uint8_t index, uint32_t voltage);

uint16_t halAfeGetMaxCellVoltage(void);
uint16_t halAfeGetMinCellVoltage(void);
uint16_t HalAfeGetMinNtcTempAdc(void);
uint16_t HalAfeGetMaxNtcTempAdc(void);

void halAfeUpdateMinMaxCellVoltage(void);
void halAfeUpdateMinMaxNtcTempVoltage(void);

void halafeOpen(tLibRegisterEvtHandler evtHandler);
void halAfeSetPhysicalBalancePosition(uint8_t bmuindex, uint16_t position);
void halAfeSetBalanceOnFlag(uint8_t onflag);
uint8_t halAfeGetState(void);
uint8_t halAfeIsL1Protect(void);
uint8_t halAfeIsL2Protect(void);







	



     
#line 24 "..\\..\\..\\AppProtect\\ApiProtectDocp.c"
#line 1 "..\\..\\..\\AppProtect\\AppProtect.h"














 


 
#line 20 "..\\..\\..\\AppProtect\\AppProtect.h"
#line 21 "..\\..\\..\\AppProtect\\AppProtect.h"
#line 1 "..\\..\\..\\AppProtect\\AppProtectEvent.h"














 
  



enum{
	APP_PROTECT_OVP_L1_SET=1,
	APP_PROTECT_OVP_L2_SET,
	APP_PROTECT_OVP_L3_SET,
	APP_PROTECT_OVP_L1_RELEASE,
	APP_PROTECT_OVP_L2_RELEASE,
	APP_PROTECT_OVP_L3_RELEASE,

	APP_PROTECT_UVP_L1_SET,
	APP_PROTECT_UVP_L2_SET,
	APP_PROTECT_UVP_L3_SET,
	APP_PROTECT_UVP_L1_RELEASE,
	APP_PROTECT_UVP_L2_RELEASE,
	APP_PROTECT_UVP_L3_RELEASE,
	
	APP_PROTECT_COTP_L1_SET,
	APP_PROTECT_COTP_L2_SET,
	APP_PROTECT_COTP_L3_SET,
	APP_PROTECT_COTP_L4_SET,
	APP_PROTECT_COTP_L1_RELEASE,
	APP_PROTECT_COTP_L2_RELEASE,
	APP_PROTECT_COTP_L3_RELEASE,
	APP_PROTECT_COTP_L4_RELEASE,		

	APP_PROTECT_CUTP_L1_SET,
	APP_PROTECT_CUTP_L2_SET,
	APP_PROTECT_CUTP_L3_SET,
	APP_PROTECT_CUTP_L4_SET,
	APP_PROTECT_CUTP_L1_RELEASE,		
	APP_PROTECT_CUTP_L2_RELEASE,
	APP_PROTECT_CUTP_L3_RELEASE,
	APP_PROTECT_CUTP_L4_RELEASE,
	
	APP_PROTECT_DOTP_L1_SET,
	APP_PROTECT_DOTP_L2_SET,		
	APP_PROTECT_DOTP_L3_SET,
	APP_PROTECT_DOTP_L1_RELEASE,
	APP_PROTECT_DOTP_L2_RELEASE,
	APP_PROTECT_DOTP_L3_RELEASE,

	APP_PROTECT_DUTP_L1_SET,
	APP_PROTECT_DUTP_L2_SET,		
	APP_PROTECT_DUTP_L3_SET,
	APP_PROTECT_DUTP_L1_RELEASE,
	APP_PROTECT_DUTP_L2_RELEASE,
	APP_PROTECT_DUTP_L3_RELEASE,

	APP_PROTECT_DOCP_L1_SET,		
	APP_PROTECT_DOCP_L2_SET,
	APP_PROTECT_DOCP_L3_SET,
	APP_PROTECT_DOCP_L4_SET,
	APP_PROTECT_DOCP_L1_RELEASE,
	APP_PROTECT_DOCP_L2_RELEASE,
	APP_PROTECT_DOCP_L3_RELEASE,
	APP_PROTECT_DOCP_L4_RELEASE,
	
	APP_PROTECT_COCP_L1_SET,
	APP_PROTECT_COCP_L2_SET,	
	APP_PROTECT_COCP_L3_SET,
	APP_PROTECT_COCP_L4_SET,
	APP_PROTECT_COCP_L1_RELEASE,
	APP_PROTECT_COCP_L2_RELEASE,
	APP_PROTECT_COCP_L3_RELEASE,
	APP_PROTECT_COCP_L4_RELEASE,

	APP_PROTECT_OVP_PF,
	APP_PROTECT_UVP_PF,
	
	APP_PROTECT_EVENT
};





     
#line 22 "..\\..\\..\\AppProtect\\AppProtect.h"





 




























 
typedef struct{
	uint8_t		Mask;
	uint8_t		ClearMask;
	uint8_t		Setting;
	uint8_t		Setted;
	uint8_t		Releasing;
}tProtectFlagValue;

 
 
 
void appProtectGetLevelMask(uint8_t Level, tProtectFlagValue *pProtectFlagValue);
uint8_t	appProtectIsUnderTemperter(uint16_t NtcAdcValue, uint16_t CompareAdcValue);
uint8_t	appProtectIsOverTemperter(uint16_t NtcAdcValue, uint16_t CompareAdcValue);
void appProtectOpen(tLibRegisterEvtHandler evtHandler);
void appProtectHandler(uint16_t evt);

 
 
 
 










     





#line 25 "..\\..\\..\\AppProtect\\ApiProtectDocp.c"
#line 1 "..\\..\\..\\AppProtect\\ApiProtectDocp.h"














 


 
#line 20 "..\\..\\..\\AppProtect\\ApiProtectDocp.h"
#line 21 "..\\..\\..\\AppProtect\\ApiProtectDocp.h"
#line 22 "..\\..\\..\\AppProtect\\ApiProtectDocp.h"




 
 


 
 
 
void apiProtectDocpOpen(tLibRegisterEvtHandler evtHandler);
uint8_t	apiProtectDocpGetFlag(void);
uint8_t apiProtectDocpHandler(uint8_t ProtectLevel);


 
 
 
 










     


#line 26 "..\\..\\..\\AppProtect\\ApiProtectDocp.c"
#line 1 "..\\..\\..\\API\\ApiSysPar.h"














 



 
#line 21 "..\\..\\..\\API\\ApiSysPar.h"
#line 22 "..\\..\\..\\API\\ApiSysPar.h"
#line 1 "..\\..\\..\\Lib\\LibCalibration.h"














 








 
#line 26 "..\\..\\..\\Lib\\LibCalibration.h"
#line 27 "..\\..\\..\\Lib\\LibCalibration.h"

 
typedef struct {  
    int32_t A1;    
    int32_t A2;  
    int32_t B;
} tCalibCoef;
 
 
 
 
 
 
int32_t doCalibration(tCalibCoef *par, int32_t dataX);
tCalibCoef calCoef(int32_t valL, int32_t adcL, int32_t valH, int32_t adcH);







#line 23 "..\\..\\..\\API\\ApiSysPar.h"














typedef struct{
    int32_t 	valL;
    int32_t 	valH;
    int32_t 	adcL;
    int32_t 	adcH;
}tCaliPar;


typedef struct{
	tLbyte			SetValue;
	tLbyte			STime;
	tLbyte			RelValue;
	tLbyte			RTime;
}tScuProtectPar;


typedef struct{
		uint8_t		Level;
		uint16_t	Value;
}tOcvRaTable;

typedef struct{
	uint8_t			HeadInfo[8];
	uint16_t		ParLeng;
	uint16_t		DateCode;
	uint32_t		Checksum;
	tCaliPar		Currentt[2];
	tCaliPar		VBat[2];
	uint32_t	Reserved;
}tCalRomPar;

typedef struct{
	tCalRomPar			RomPar;
	struct{	
		tCalibCoef		Current[2];
		tCalibCoef		VBat[2];
	}RamPar;
}tSysCalPar;

extern tSysCalPar	SysCalPar;

 


 
 

uint8_t apiSysParIsOvpPfSet(void);
uint8_t apiSysParIsUvpPfSet(void);
void apiSysParOvpPfClean(void);
void apiSysParOvpPfSet(void);
void apiSysParUvpPfClean(void);
void apiSysParUvpPfSet(void);


void apiCaliParSetCurrentValue(uint8_t CurrentNum, uint8_t PointIndex,int32_t Value, int32_t Adc);
void apiCaliParGetCurrentValue(uint8_t CurrentNum, uint8_t PointIndex,int32_t *Value, int32_t *Adc);
void apiCaliParSetVbatValue(uint8_t CurrentIndex, uint8_t PointIndex,int32_t Value, int32_t Adc);
void apiCaliParGetVbatValue(uint8_t CurrentNum, uint8_t PointIndex,int32_t *Value, int32_t *Adc);
uint32_t apiCaliParGetChecksum(void);






uint32_t apiSysParGetHwVersion(void);
void apiSysParSetHwVersion(uint32_t version);
uint32_t apiSysParGetFwVersion(void);
void appSysParSetFwVersion(uint32_t version);


uint8_t apiSysParGetBmuNumber(void);
void apiSysParSetBmuNumber(uint8_t BmuNumber);
uint32_t apiSysParGetCellFlag(uint8_t BmuIndex);
void apiSysParSetCellFlag(uint8_t BmuIndex,uint32_t CellFlag);
uint32_t apiSysParGetNtcFlag(uint8_t BmuIndex);
void apiSysParSetNtcFlag(uint8_t BmuIndex,uint32_t NtcFlag);

uint16_t apiSysParGetZeroCurrentValue(void);
void apiSysParSetZeroCurrentValue(uint16_t current);
uint16_t apiSysParGetMinChargeCurrentValue(void);
void apiSysParSetMinChargeCurrentValue(uint16_t current);

uint32_t apiSysParGetDesignedCapacity(void);
void apiSysParSetDesignedCapacity(uint32_t dc);

void apiSysParGetFullChargeCondition(tScuProtectPar *pPar);
void apiSysParSetFullChargeCondition(tScuProtectPar *pPar);


uint16_t apiSysParGetMinFlatVoltage(void);
uint16_t apiSysParGetMaxFlatVoltage(void);

void apiSysParGetFlatVoltage(tScuProtectPar *pPar);
void apiSysParSetFlatVoltage(tScuProtectPar *pPar);



uint16_t apiSysParGetCellNumber(void);
uint16_t apiSysParGetNtcNumber(void);

void apiSysParGetOcvTable(uint8_t index ,tOcvRaTable *pOcvTable);
void apiSysParGetRaTable(uint8_t index ,tOcvRaTable *pOcvTable);
void apiSysParSetOcvTable(uint8_t index ,tOcvRaTable *pOcvTable);
void apiSysParSetRaTable(uint8_t index ,tOcvRaTable *pOcvTable);

void apiSysParGetAfeCommTime(tScuProtectPar *pPar);
void apiSysParSetAfeCommTime(tScuProtectPar *pPar);
void apiSysParGetInsulationResistance(tScuProtectPar *pPar);
void apiSysParSetInsulationResistance(tScuProtectPar *pPar);

uint16_t apiSysParGetTerminateVoltage(void);
void apiSysParSetTerminateVoltage(uint16_t voltage);
uint16_t apiSysParGetPreDischargeTime(void);
void apiSysParSetPreDischargeTime(uint16_t time);
uint16_t apiSysParGetRelayOnDiffVoltage(void);
void apiSysParSetRelayOnDiffVoltage(uint16_t voltage);

uint8_t apiSysParGetScuId(void);
void saveScuIdPar(uint8_t scuid);



uint16_t appSysParGetOvpSetTime(uint8_t level);
uint16_t appSysParGetOvpReleaseValue(uint8_t level);
uint16_t appSysParGetOvpReleaseTime(uint8_t level);

void apiSysParGetOvpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetOvpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetUvpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetUvpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGet2ndOtProtectPar(tScuProtectPar *pPar);
void apiSysParSet2ndOtProtectPar(tScuProtectPar *pPar);
void apiSysParGet2ndUtProtectPar(tScuProtectPar *pPar);
void apiSysParSet2ndUtProtectPar(tScuProtectPar *pPar);

void apiSysParGetCotpProtectPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParGetCotpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetCotpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetCutpProtectPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParGetCutpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetCutpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetDotpProtectPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParGetDotpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetDotpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetDutpProtectPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParGetDutpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetDutpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetDtpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetDtpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetCocpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetCocpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetDocpPar(uint8_t level, tScuProtectPar *pPar);
void apiSysParSetDocpPar(uint8_t level, tScuProtectPar *pPar);

void apiSysParGetOvpPfPar(tScuProtectPar *pPar);
void apiSysParSetOvpPfPar(tScuProtectPar *pPar);

void apiSysParGetUvpPfPar(tScuProtectPar *pPar);
void apiSysParSetUvpPfPar(tScuProtectPar *pPar);

void apiSysParGetBalanceDuty(tScuProtectPar *pPar);
void apiSysParSetBalanceDuty(tScuProtectPar *pPar);
void apiSysParGetBalanceChg(tScuProtectPar *pPar);
void apiSysParSetBalanceChg(tScuProtectPar *pPar);
void apiSysParGetBalanceDhg(tScuProtectPar *pPar);
void apiSysParSetBalanceDhg(tScuProtectPar *pPar);
void apiSysParGetBalanceRlx(tScuProtectPar *pPar);
void apiSysParSetBalanceRlx(tScuProtectPar *pPar);

void apiSysParGetNotwMessageString(uint8_t *pMsg);
void apiSysParSetNotwMessageString(uint8_t *pMsg);

uint32_t apiSysParGetQmax(void);
void apiSysParSetQmax(uint32_t Qmax);
uint16_t apiSysParGetQmaxUpdateTimes(void);
void apiSysParSetQmaxUpdateTimes(uint16_t times);
uint16_t apiSysParGetCycleCount(void);
void apiSysParSetCycleCount(uint16_t count);
uint16_t apiSysParGetPfFlag(void);
void apiSysParSetPfFlag(uint16_t flag);
uint32_t apiSysParGetChecksum(void);

uint16_t apiSysParOpen(void);







	



     


#line 27 "..\\..\\..\\AppProtect\\ApiProtectDocp.c"
#line 1 "..\\..\\..\\APP\\AppGauge.h"














 



 
#line 21 "..\\..\\..\\APP\\AppGauge.h"
#line 22 "..\\..\\..\\APP\\AppGauge.h"
#line 23 "..\\..\\..\\APP\\AppGauge.h"




 







enum{
	APP_GAUGE_UNKNOW = 0,
	APP_GAUGE_STATE_IDLE,
	APP_GAUGE_CHARGING,
	APP_GAUGE_FULL,
	APP_GAUGE_DISCHARGE,
	APP_GAUGE_EMPTY
};

enum{
	APP_GAUGE_EVENT_START_CHG_SOC = 0,
	APP_GAUGE_EVENT_START_DHG_SOC,
	APP_GAUGE_EVENT_IDLE_OVER_5HR,
	APP_GAUGE_EVENT_CANNOT_GET_5HR_SOC,
	APP_GAUGE_EVENT_UPDATE_QMAX_1ST,
	APP_GAUGE_EVENT_UPDATE_QMAX,
	APP_GAUGE_EVENT_GET_5HR_SOC,
	APP_GAUGE_EVENT_CAL_RA1 = 40,
	APP_GAUGE_EVT_END
};

void appGaugeCleanCycleCount(void);
void appGaugeSetSoc0(uint16_t soc);
void appGaugeUpdateSoc0(void);

void appGaugeOpen(tLibRegisterEvtHandler evtHandler);
uint8_t	appGaugeGetCurrentMode(void);
int32_t appGaugeGetCurrentValue(void);

uint32_t appGaugeGetQmax(void);
void appGaugeSetQmax(uint32_t qmax);

uint32_t appGaugeGetQStart(void);
uint32_t appGaugeGetRM(void);
void appGaugeSetRM(uint32_t rm);

int32_t appGaugeGetQPassCharge(void);
int32_t appGaugeGetRPassCharge(void);
uint32_t appGaugeGetFCC(void);
uint16_t appGaugeGetSOH(void);
uint16_t appGaugeGetRSoc(void);
uint16_t appGaugeGetSoc0(void);

uint16_t appGaugeGetRSoc(void);
uint16_t appGaugeGetRamSoc(void);
uint16_t appGaugeGetEndOfSoc(void);
uint16_t appGaugeGetDisplaySoc(void);
uint16_t appGaugeGetCyleCount(void);


 





	



     


#line 28 "..\\..\\..\\AppProtect\\ApiProtectDocp.c"

void appSerialCanDavinciSendTextMessage(uint8_t *str);



 
 
 
typedef struct{
	uint8_t	Flag;
	uint8_t	SetCount[3];
	uint8_t	ReleaseCount[3];
	tLibRegisterEvtHandler  EvtHandler;
}tDocpProtect;

static tDocpProtect	mDocpProtect={0};
 
 
 
 
uint8_t	apiProtectDocpGetFlag(void)
{
	return mDocpProtect.Flag;
}

uint8_t apiProtectDocpHandler(uint8_t ProtectLevel)
{
	char		str[100];
	uint16_t		value;
	int32_t		CurrentValue;
	tProtectFlagValue	ProtectFlagValue;
	tScuProtectPar		ProtectPar;
	
	apiSysParGetDocpPar(ProtectLevel, &ProtectPar);
	appProtectGetLevelMask(ProtectLevel, &ProtectFlagValue);

	CurrentValue = abs(appGaugeGetCurrentValue()) / 1000;

	if(appGaugeGetCurrentMode() != 1)
		CurrentValue = 0;





	




	
		
		
		
		
		
	if(CurrentValue > ProtectPar.SetValue.l)
	{
		if((mDocpProtect.Flag & ProtectFlagValue.Mask) == 0)
		{
			mDocpProtect.Flag &= ProtectFlagValue.ClearMask;
			mDocpProtect.Flag |= ProtectFlagValue.Setting;
			mDocpProtect.SetCount[ProtectLevel] = 1;			
		}
		else if((mDocpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
		{
			mDocpProtect.SetCount[ProtectLevel]++;
			if(mDocpProtect.SetCount[ProtectLevel] >= ProtectPar.STime.l)
			{
				if(mDocpProtect.EvtHandler)
				{
					value = CurrentValue;
					mDocpProtect.EvtHandler(0, APP_PROTECT_DOCP_L1_SET + ProtectLevel, &value);
				}
				mDocpProtect.Flag &= ProtectFlagValue.ClearMask;
				mDocpProtect.Flag |= ProtectFlagValue.Setted;
				mDocpProtect.SetCount[ProtectLevel] = 0;
			}
		}
	}
	else if((mDocpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
	{
		mDocpProtect.Flag &= ProtectFlagValue.ClearMask;
	}	
	
	
	if(CurrentValue < ProtectPar.RelValue.l)
	{
		if((mDocpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Setted)
		{
			mDocpProtect.Flag &= ProtectFlagValue.ClearMask;
			mDocpProtect.Flag |= ProtectFlagValue.Releasing;
			mDocpProtect.ReleaseCount[ProtectLevel] = 1;		
		}
		else if((mDocpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
		{
			mDocpProtect.ReleaseCount[ProtectLevel] ++;
			if(mDocpProtect.ReleaseCount[ProtectLevel]  >=  ProtectPar.RTime.l)
			{
				if(mDocpProtect.EvtHandler)
				{
					value = CurrentValue;
					mDocpProtect.EvtHandler(0, APP_PROTECT_DOCP_L1_RELEASE + ProtectLevel, &value);	
				}	
				mDocpProtect.Flag &= ProtectFlagValue.ClearMask;
				mDocpProtect.ReleaseCount[ProtectLevel] = 0;
			}		
		}
	}
	else if((mDocpProtect.Flag & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
	{
		mDocpProtect.Flag &= ProtectFlagValue.ClearMask;
		mDocpProtect.Flag |= ProtectFlagValue.Setted;
 	}	
	
	return 1;
}

void apiProtectDocpOpen(tLibRegisterEvtHandler evtHandler)
{
	mDocpProtect.EvtHandler = evtHandler;
}

     


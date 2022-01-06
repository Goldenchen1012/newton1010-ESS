#line 1 "..\\..\\..\\User\\main.c"














 

 
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











#line 19 "..\\..\\..\\User\\main.c"
#line 1 "..\\..\\..\\User\\main.h"

















 

 



 
#line 1 "..\\..\\..\\BSP\\Bsp.h"














 



	







#line 1 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"














 


 
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






 
#line 20 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"
#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h"


















 

 







 
#line 1 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"

















 

 







 
 

 


 



 
 

 
 
 
 

 
 
 

 
 
 
 
 


 
 
 
 
 
 

 

 

 
 
 
 

 



 
 


 




 











 








 










 







 








 












 








 





 

 


 
#line 175 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"

 



 
 

 




 



 


 

#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

















 

 







 
#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_def.h"


















 

 







 
#line 1 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l4xx.h"



























 



 



 










 



 






 

#line 92 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l4xx.h"



 
#line 104 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l4xx.h"



 
#line 116 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l4xx.h"



 



 

#line 1 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"























 



 



 










 



 








 



 




 
typedef enum
{
 
  NonMaskableInt_IRQn         = -14,     
  HardFault_IRQn              = -13,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      
 
  WWDG_IRQn                   = 0,       
  PVD_PVM_IRQn                = 1,       
  TAMP_STAMP_IRQn             = 2,       
  RTC_WKUP_IRQn               = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Channel1_IRQn          = 11,      
  DMA1_Channel2_IRQn          = 12,      
  DMA1_Channel3_IRQn          = 13,      
  DMA1_Channel4_IRQn          = 14,      
  DMA1_Channel5_IRQn          = 15,      
  DMA1_Channel6_IRQn          = 16,      
  DMA1_Channel7_IRQn          = 17,      
  ADC1_2_IRQn                 = 18,      
  CAN1_TX_IRQn                = 19,      
  CAN1_RX0_IRQn               = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_TIM15_IRQn         = 24,      
  TIM1_UP_TIM16_IRQn          = 25,      
  TIM1_TRG_COM_TIM17_IRQn     = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTC_Alarm_IRQn              = 41,      
  DFSDM1_FLT3_IRQn            = 42,      
  TIM8_BRK_IRQn               = 43,      
  TIM8_UP_IRQn                = 44,      
  TIM8_TRG_COM_IRQn           = 45,      
  TIM8_CC_IRQn                = 46,      
  ADC3_IRQn                   = 47,      
  FMC_IRQn                    = 48,      
  SDMMC1_IRQn                 = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_DAC_IRQn               = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Channel1_IRQn          = 56,      
  DMA2_Channel2_IRQn          = 57,      
  DMA2_Channel3_IRQn          = 58,      
  DMA2_Channel4_IRQn          = 59,      
  DMA2_Channel5_IRQn          = 60,      
  DFSDM1_FLT0_IRQn            = 61,      
  DFSDM1_FLT1_IRQn            = 62,      
  DFSDM1_FLT2_IRQn            = 63,      
  COMP_IRQn                   = 64,      
  LPTIM1_IRQn                 = 65,      
  LPTIM2_IRQn                 = 66,      
  OTG_FS_IRQn                 = 67,      
  DMA2_Channel6_IRQn          = 68,      
  DMA2_Channel7_IRQn          = 69,      
  LPUART1_IRQn                = 70,      
  QUADSPI_IRQn                = 71,      
  I2C3_EV_IRQn                = 72,      
  I2C3_ER_IRQn                = 73,      
  SAI1_IRQn                   = 74,      
  SAI2_IRQn                   = 75,      
  SWPMI1_IRQn                 = 76,      
  TSC_IRQn                    = 77,      
  LCD_IRQn                    = 78,      
  RNG_IRQn                    = 80,      
  FPU_IRQn                    = 81,      
  CRS_IRQn                    = 82,      
  I2C4_EV_IRQn                = 83,      
  I2C4_ER_IRQn                = 84,      
  DCMI_IRQn                   = 85,      
  CAN2_TX_IRQn                = 86,      
  CAN2_RX0_IRQn               = 87,      
  CAN2_RX1_IRQn               = 88,      
  CAN2_SCE_IRQn               = 89,      
  DMA2D_IRQn                  = 90       
} IRQn_Type;



 

#line 1 "../../../Drivers/CMSIS/Include/core_cm4.h"
 




 
















 










#line 35 "../../../Drivers/CMSIS/Include/core_cm4.h"

















 




 



 

#line 1 "../../../Drivers/CMSIS/Include/cmsis_version.h"
 




 
















 










 
#line 64 "../../../Drivers/CMSIS/Include/core_cm4.h"

 









 
#line 87 "../../../Drivers/CMSIS/Include/core_cm4.h"

#line 161 "../../../Drivers/CMSIS/Include/core_cm4.h"

#line 1 "../../../Drivers/CMSIS/Include/cmsis_compiler.h"
 




 
















 




#line 29 "../../../Drivers/CMSIS/Include/cmsis_compiler.h"



 
#line 1 "../../../Drivers/CMSIS/Include/cmsis_armcc.h"
 




 
















 









 













   
   

 




 
#line 110 "../../../Drivers/CMSIS/Include/cmsis_armcc.h"

 





















 



 





 
 






 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}









 







 







 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}






 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xFFU);
}







 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xFFU);
}






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}






 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1U);
}









 
static __inline uint32_t __get_FPSCR(void)
{


  register uint32_t __regfpscr         __asm("fpscr");
  return(__regfpscr);



}






 
static __inline void __set_FPSCR(uint32_t fpscr)
{


  register uint32_t __regfpscr         __asm("fpscr");
  __regfpscr = (fpscr);



}


 


 



 




 






 







 






 








 










 










 






                  





 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int16_t __REVSH(int16_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 
#line 561 "../../../Drivers/CMSIS/Include/cmsis_armcc.h"







 











 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 


#line 809 "../../../Drivers/CMSIS/Include/cmsis_armcc.h"

   


 



 



#line 880 "../../../Drivers/CMSIS/Include/cmsis_armcc.h"











 


#line 35 "../../../Drivers/CMSIS/Include/cmsis_compiler.h"




 
#line 280 "../../../Drivers/CMSIS/Include/cmsis_compiler.h"




#line 163 "../../../Drivers/CMSIS/Include/core_cm4.h"

















 
#line 207 "../../../Drivers/CMSIS/Include/core_cm4.h"

 






 
#line 223 "../../../Drivers/CMSIS/Include/core_cm4.h"

 




 













 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:16;               
    uint32_t GE:4;                        
    uint32_t _reserved1:7;                
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 





















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:1;                
    uint32_t ICI_IT_1:6;                  
    uint32_t GE:4;                        
    uint32_t _reserved1:4;                
    uint32_t T:1;                         
    uint32_t ICI_IT_2:2;                  
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 

































 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 









 







 



 
typedef struct
{
  volatile uint32_t ISER[8U];                
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];                
        uint32_t RESERVED1[24U];
  volatile uint32_t ISPR[8U];                
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];                
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];                
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];                
        uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                    
}  NVIC_Type;

 



 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
  volatile uint32_t VTOR;                    
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
  volatile uint8_t  SHP[12U];                
  volatile uint32_t SHCSR;                   
  volatile uint32_t CFSR;                    
  volatile uint32_t HFSR;                    
  volatile uint32_t DFSR;                    
  volatile uint32_t MMFAR;                   
  volatile uint32_t BFAR;                    
  volatile uint32_t AFSR;                    
  volatile const  uint32_t PFR[2U];                 
  volatile const  uint32_t DFR;                     
  volatile const  uint32_t ADR;                     
  volatile const  uint32_t MMFR[4U];                
  volatile const  uint32_t ISAR[5U];                
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                   
} SCB_Type;

 















 






























 



 





















 









 


















 










































 









 


















 





















 


















 









 















 







 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;                    
  volatile uint32_t ACTLR;                   
} SCnSCB_Type;

 



 















 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 







 



 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                  
    volatile  uint16_t   u16;                 
    volatile  uint32_t   u32;                 
  }  PORT [32U];                          
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;                     
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;                     
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;                     
        uint32_t RESERVED3[32U];
        uint32_t RESERVED4[43U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
        uint32_t RESERVED5[6U];
  volatile const  uint32_t PID4;                    
  volatile const  uint32_t PID5;                    
  volatile const  uint32_t PID6;                    
  volatile const  uint32_t PID7;                    
  volatile const  uint32_t PID0;                    
  volatile const  uint32_t PID1;                    
  volatile const  uint32_t PID2;                    
  volatile const  uint32_t PID3;                    
  volatile const  uint32_t CID0;                    
  volatile const  uint32_t CID1;                    
  volatile const  uint32_t CID2;                    
  volatile const  uint32_t CID3;                    
} ITM_Type;

 



 



























 









   







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t CYCCNT;                  
  volatile uint32_t CPICNT;                  
  volatile uint32_t EXCCNT;                  
  volatile uint32_t SLEEPCNT;                
  volatile uint32_t LSUCNT;                  
  volatile uint32_t FOLDCNT;                 
  volatile const  uint32_t PCSR;                    
  volatile uint32_t COMP0;                   
  volatile uint32_t MASK0;                   
  volatile uint32_t FUNCTION0;               
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t FUNCTION1;               
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t FUNCTION2;               
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;                   
  volatile uint32_t MASK3;                   
  volatile uint32_t FUNCTION3;               
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   







 



 
typedef struct
{
  volatile const  uint32_t SSPSR;                   
  volatile uint32_t CSPSR;                   
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;                    
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;                    
        uint32_t RESERVED2[131U];
  volatile const  uint32_t FFSR;                    
  volatile uint32_t FFCR;                    
  volatile const  uint32_t FSCR;                    
        uint32_t RESERVED3[759U];
  volatile const  uint32_t TRIGGER;                 
  volatile const  uint32_t FIFO0;                   
  volatile const  uint32_t ITATBCTR2;               
        uint32_t RESERVED4[1U];
  volatile const  uint32_t ITATBCTR0;               
  volatile const  uint32_t FIFO1;                   
  volatile uint32_t ITCTRL;                  
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;                
  volatile uint32_t CLAIMCLR;                
        uint32_t RESERVED7[8U];
  volatile const  uint32_t DEVID;                   
  volatile const  uint32_t DEVTYPE;                 
} TPI_Type;

 



 



 












 






 



 





















 






 





















 






 



 


















 






   








 



 
typedef struct
{
  volatile const  uint32_t TYPE;                    
  volatile uint32_t CTRL;                    
  volatile uint32_t RNR;                     
  volatile uint32_t RBAR;                    
  volatile uint32_t RASR;                    
  volatile uint32_t RBAR_A1;                 
  volatile uint32_t RASR_A1;                 
  volatile uint32_t RBAR_A2;                 
  volatile uint32_t RASR_A2;                 
  volatile uint32_t RBAR_A3;                 
  volatile uint32_t RASR_A3;                 
} MPU_Type;



 









 









 



 









 






























 








 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile uint32_t FPCCR;                   
  volatile uint32_t FPCAR;                   
  volatile uint32_t FPDSCR;                  
  volatile const  uint32_t MVFR0;                   
  volatile const  uint32_t MVFR1;                   
  volatile const  uint32_t MVFR2;                   
} FPU_Type;

 



























 



 












 
























 












 




 







 



 
typedef struct
{
  volatile uint32_t DHCSR;                   
  volatile  uint32_t DCRSR;                   
  volatile uint32_t DCRDR;                   
  volatile uint32_t DEMCR;                   
} CoreDebug_Type;

 




































 






 







































 







 






 







 


 







 

 
#line 1553 "../../../Drivers/CMSIS/Include/core_cm4.h"

#line 1562 "../../../Drivers/CMSIS/Include/core_cm4.h"









 










 


 



 





 

#line 1616 "../../../Drivers/CMSIS/Include/core_cm4.h"

#line 1626 "../../../Drivers/CMSIS/Include/core_cm4.h"




 
#line 1637 "../../../Drivers/CMSIS/Include/core_cm4.h"










 
static __inline void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));  
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U)  );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}






 
static __inline uint32_t __NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}







 
static __inline void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    __memory_changed();
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __memory_changed();
  }
}









 
static __inline uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}







 
static __inline void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);
  }
}









 
static __inline uint32_t __NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}







 
static __inline void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}







 
static __inline void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}









 
static __inline uint32_t __NVIC_GetActive(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}










 
static __inline void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)IRQn)]               = (uint8_t)((priority << (8U - 4)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4)) & (uint32_t)0xFFUL);
  }
}










 
static __inline uint32_t __NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)IRQn)]               >> (8U - 4)));
  }
  else
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4)));
  }
}












 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4)) ? (uint32_t)(4) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}












 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4)) ? (uint32_t)(4) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}










 
static __inline void __NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
  uint32_t vectors = (uint32_t )((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;
  (* (int *) (vectors + ((int32_t)IRQn + 16) * 4)) = vector;
   
}









 
static __inline uint32_t __NVIC_GetVector(IRQn_Type IRQn)
{
  uint32_t vectors = (uint32_t )((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;
  return (uint32_t)(* (int *) (vectors + ((int32_t)IRQn + 16) * 4));
}





 
__declspec(noreturn) static __inline void __NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16U)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U)    );          
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 


 



#line 1 "../../../Drivers/CMSIS/Include/mpu_armv7.h"





 
















 
 





 



#line 62 "../../../Drivers/CMSIS/Include/mpu_armv7.h"

#line 69 "../../../Drivers/CMSIS/Include/mpu_armv7.h"





 












   














 
#line 110 "../../../Drivers/CMSIS/Include/mpu_armv7.h"












                          









  










  












  




 




 




 




 





 
typedef struct {
  uint32_t RBAR; 
  uint32_t RASR; 
} ARM_MPU_Region_t;
    


 
static __inline void ARM_MPU_Enable(uint32_t MPU_Control)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL = MPU_Control | (1UL );

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR |= (1UL << 16U);

  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
  do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);
}


 
static __inline void ARM_MPU_Disable(void)
{
  do { __schedule_barrier(); __dmb(0xF); __schedule_barrier(); } while (0U);

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR &= ~(1UL << 16U);

  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL  &= ~(1UL );
}



 
static __inline void ARM_MPU_ClrRegion(uint32_t rnr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR = rnr;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = 0U;
}




    
static __inline void ARM_MPU_SetRegion(uint32_t rbar, uint32_t rasr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR = rbar;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = rasr;
}





    
static __inline void ARM_MPU_SetRegionEx(uint32_t rnr, uint32_t rbar, uint32_t rasr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR = rnr;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR = rbar;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = rasr;
}





 
static __inline void ARM_MPU_OrderedMemcpy(volatile uint32_t* dst, const uint32_t* __restrict src, uint32_t len)
{
  uint32_t i;
  for (i = 0U; i < len; ++i) 
  {
    dst[i] = src[i];
  }
}




 
static __inline void ARM_MPU_Load(ARM_MPU_Region_t const* table, uint32_t cnt) 
{
  const uint32_t rowWordSize = sizeof(ARM_MPU_Region_t)/4U;
  while (cnt > 4U) {
    ARM_MPU_OrderedMemcpy(&(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR), &(table->RBAR), 4U*rowWordSize);
    table += 4U;
    cnt -= 4U;
  }
  ARM_MPU_OrderedMemcpy(&(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR), &(table->RBAR), cnt*rowWordSize);
}

#line 1956 "../../../Drivers/CMSIS/Include/core_cm4.h"




 





 








 
static __inline uint32_t SCB_GetFPUType(void)
{
  uint32_t mvfr0;

  mvfr0 = ((FPU_Type *) ((0xE000E000UL) + 0x0F30UL) )->MVFR0;
  if      ((mvfr0 & ((0xFUL << 4U) | (0xFUL << 8U))) == 0x020U)
  {
    return 1U;            
  }
  else
  {
    return 0U;            
  }
}


 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  __NVIC_SetPriority (SysTick_IRQn, (1UL << 4) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 





 

extern volatile int32_t ITM_RxBuffer;                               










 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __nop();
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}







 
static __inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                            

  if (ITM_RxBuffer != ((int32_t)0x5AA55AA5U))
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = ((int32_t)0x5AA55AA5U);        
  }

  return (ch);
}







 
static __inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == ((int32_t)0x5AA55AA5U))
  {
    return (0);                               
  }
  else
  {
    return (1);                               
  }
}

 










#line 176 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"
#line 1 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/system_stm32l4xx.h"

















 



 



 



 









 



 




 
  






 
extern uint32_t SystemCoreClock;             

extern const uint8_t  AHBPrescTable[16];     
extern const uint8_t  APBPrescTable[8];      
extern const uint32_t MSIRangeTable[12];     



 



 



 



 



 



 

extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 



 
 
#line 177 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"
#line 178 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 



 

typedef struct
{
  volatile uint32_t ISR;           
  volatile uint32_t IER;           
  volatile uint32_t CR;            
  volatile uint32_t CFGR;          
  volatile uint32_t CFGR2;         
  volatile uint32_t SMPR1;         
  volatile uint32_t SMPR2;         
       uint32_t RESERVED1;     
  volatile uint32_t TR1;           
  volatile uint32_t TR2;           
  volatile uint32_t TR3;           
       uint32_t RESERVED2;     
  volatile uint32_t SQR1;          
  volatile uint32_t SQR2;          
  volatile uint32_t SQR3;          
  volatile uint32_t SQR4;          
  volatile uint32_t DR;            
       uint32_t RESERVED3;     
       uint32_t RESERVED4;     
  volatile uint32_t JSQR;          
       uint32_t RESERVED5[4];  
  volatile uint32_t OFR1;          
  volatile uint32_t OFR2;          
  volatile uint32_t OFR3;          
  volatile uint32_t OFR4;          
       uint32_t RESERVED6[4];  
  volatile uint32_t JDR1;          
  volatile uint32_t JDR2;          
  volatile uint32_t JDR3;          
  volatile uint32_t JDR4;          
       uint32_t RESERVED7[4];  
  volatile uint32_t AWD2CR;        
  volatile uint32_t AWD3CR;        
       uint32_t RESERVED8;     
       uint32_t RESERVED9;     
  volatile uint32_t DIFSEL;        
  volatile uint32_t CALFACT;       

} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CSR;           
  uint32_t      RESERVED;      
  volatile uint32_t CCR;           
  volatile uint32_t CDR;           
} ADC_Common_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SR;        
  volatile uint32_t RISR;      
  volatile uint32_t IER;       
  volatile uint32_t MISR;      
  volatile uint32_t ICR;       
  volatile uint32_t ESCR;      
  volatile uint32_t ESUR;      
  volatile uint32_t CWSTRTR;   
  volatile uint32_t CWSIZER;   
  volatile uint32_t DR;        
} DCMI_TypeDef;



 

typedef struct
{
  volatile uint32_t TIR;   
  volatile uint32_t TDTR;  
  volatile uint32_t TDLR;  
  volatile uint32_t TDHR;  
} CAN_TxMailBox_TypeDef;



 

typedef struct
{
  volatile uint32_t RIR;   
  volatile uint32_t RDTR;  
  volatile uint32_t RDLR;  
  volatile uint32_t RDHR;  
} CAN_FIFOMailBox_TypeDef;



 

typedef struct
{
  volatile uint32_t FR1;  
  volatile uint32_t FR2;  
} CAN_FilterRegister_TypeDef;



 

typedef struct
{
  volatile uint32_t              MCR;                  
  volatile uint32_t              MSR;                  
  volatile uint32_t              TSR;                  
  volatile uint32_t              RF0R;                 
  volatile uint32_t              RF1R;                 
  volatile uint32_t              IER;                  
  volatile uint32_t              ESR;                  
  volatile uint32_t              BTR;                  
  uint32_t                   RESERVED0[88];        
  CAN_TxMailBox_TypeDef      sTxMailBox[3];        
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];      
  uint32_t                   RESERVED1[12];        
  volatile uint32_t              FMR;                  
  volatile uint32_t              FM1R;                 
  uint32_t                   RESERVED2;            
  volatile uint32_t              FS1R;                 
  uint32_t                   RESERVED3;            
  volatile uint32_t              FFA1R;                
  uint32_t                   RESERVED4;            
  volatile uint32_t              FA1R;                 
  uint32_t                   RESERVED5[8];         
  CAN_FilterRegister_TypeDef sFilterRegister[28];  
} CAN_TypeDef;




 

typedef struct
{
  volatile uint32_t CSR;          
} COMP_TypeDef;

typedef struct
{
  volatile uint32_t CSR;          
} COMP_Common_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;           
  volatile uint8_t  IDR;          
  uint8_t       RESERVED0;    
  uint16_t      RESERVED1;    
  volatile uint32_t CR;           
  uint32_t      RESERVED2;    
  volatile uint32_t INIT;         
  volatile uint32_t POL;          
} CRC_TypeDef;



 
typedef struct
{
volatile uint32_t CR;             
volatile uint32_t CFGR;           
volatile uint32_t ISR;            
volatile uint32_t ICR;            
} CRS_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;           
  volatile uint32_t SWTRIGR;      
  volatile uint32_t DHR12R1;      
  volatile uint32_t DHR12L1;      
  volatile uint32_t DHR8R1;       
  volatile uint32_t DHR12R2;      
  volatile uint32_t DHR12L2;      
  volatile uint32_t DHR8R2;       
  volatile uint32_t DHR12RD;      
  volatile uint32_t DHR12LD;      
  volatile uint32_t DHR8RD;       
  volatile uint32_t DOR1;         
  volatile uint32_t DOR2;         
  volatile uint32_t SR;           
  volatile uint32_t CCR;          
  volatile uint32_t MCR;          
  volatile uint32_t SHSR1;        
  volatile uint32_t SHSR2;        
  volatile uint32_t SHHR;         
  volatile uint32_t SHRR;         
} DAC_TypeDef;



 
typedef struct
{
  volatile uint32_t FLTCR1;       
  volatile uint32_t FLTCR2;       
  volatile uint32_t FLTISR;       
  volatile uint32_t FLTICR;       
  volatile uint32_t FLTJCHGR;     
  volatile uint32_t FLTFCR;       
  volatile uint32_t FLTJDATAR;    
  volatile uint32_t FLTRDATAR;    
  volatile uint32_t FLTAWHTR;     
  volatile uint32_t FLTAWLTR;     
  volatile uint32_t FLTAWSR;      
  volatile uint32_t FLTAWCFR;     
  volatile uint32_t FLTEXMAX;     
  volatile uint32_t FLTEXMIN;     
  volatile uint32_t FLTCNVTIMR;   
} DFSDM_Filter_TypeDef;



 
typedef struct
{
  volatile uint32_t CHCFGR1;      
  volatile uint32_t CHCFGR2;      
  volatile uint32_t CHAWSCDR;    
 
  volatile uint32_t CHWDATAR;     
  volatile uint32_t CHDATINR;     
} DFSDM_Channel_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;       
  volatile uint32_t CR;           
  volatile uint32_t APB1FZR1;     
  volatile uint32_t APB1FZR2;     
  volatile uint32_t APB2FZ;       
} DBGMCU_TypeDef;




 

typedef struct
{
  volatile uint32_t CCR;          
  volatile uint32_t CNDTR;        
  volatile uint32_t CPAR;         
  volatile uint32_t CMAR;         
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;          
  volatile uint32_t IFCR;         
} DMA_TypeDef;

typedef struct
{
  volatile uint32_t CSELR;        
} DMA_Request_TypeDef;

 





 

typedef struct
{
  volatile uint32_t CR;             
  volatile uint32_t ISR;            
  volatile uint32_t IFCR;           
  volatile uint32_t FGMAR;          
  volatile uint32_t FGOR;           
  volatile uint32_t BGMAR;          
  volatile uint32_t BGOR;           
  volatile uint32_t FGPFCCR;        
  volatile uint32_t FGCOLR;         
  volatile uint32_t BGPFCCR;        
  volatile uint32_t BGCOLR;         
  volatile uint32_t FGCMAR;         
  volatile uint32_t BGCMAR;         
  volatile uint32_t OPFCCR;         
  volatile uint32_t OCOLR;          
  volatile uint32_t OMAR;           
  volatile uint32_t OOR;            
  volatile uint32_t NLR;            
  volatile uint32_t LWR;            
  volatile uint32_t AMTCR;          
  uint32_t      RESERVED[236];  
  volatile uint32_t FGCLUT[256];    
  volatile uint32_t BGCLUT[256];    
} DMA2D_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR1;         
  volatile uint32_t EMR1;         
  volatile uint32_t RTSR1;        
  volatile uint32_t FTSR1;        
  volatile uint32_t SWIER1;       
  volatile uint32_t PR1;          
  uint32_t      RESERVED1;    
  uint32_t      RESERVED2;    
  volatile uint32_t IMR2;         
  volatile uint32_t EMR2;         
  volatile uint32_t RTSR2;        
  volatile uint32_t FTSR2;        
  volatile uint32_t SWIER2;       
  volatile uint32_t PR2;          
} EXTI_TypeDef;




 

typedef struct
{
  volatile uint32_t CSSA;         
  volatile uint32_t CSL;          
  volatile uint32_t NVDSSA;       
  volatile uint32_t NVDSL;        
  volatile uint32_t VDSSA ;       
  volatile uint32_t VDSL ;        
  uint32_t      RESERVED1;    
  uint32_t      RESERVED2;    
  volatile uint32_t CR ;          
} FIREWALL_TypeDef;




 

typedef struct
{
  volatile uint32_t ACR;               
  volatile uint32_t PDKEYR;            
  volatile uint32_t KEYR;              
  volatile uint32_t OPTKEYR;           
  volatile uint32_t SR;                
  volatile uint32_t CR;                
  volatile uint32_t ECCR;              
  volatile uint32_t RESERVED1;         
  volatile uint32_t OPTR;              
  volatile uint32_t PCROP1SR;          
  volatile uint32_t PCROP1ER;          
  volatile uint32_t WRP1AR;            
  volatile uint32_t WRP1BR;            
       uint32_t RESERVED2[4];      
  volatile uint32_t PCROP2SR;          
  volatile uint32_t PCROP2ER;          
  volatile uint32_t WRP2AR;            
  volatile uint32_t WRP2BR;            
} FLASH_TypeDef;




 

typedef struct
{
  volatile uint32_t BTCR[8];      
} FMC_Bank1_TypeDef;



 

typedef struct
{
  volatile uint32_t BWTR[7];      
} FMC_Bank1E_TypeDef;



 

typedef struct
{
  volatile uint32_t PCR;         
  volatile uint32_t SR;          
  volatile uint32_t PMEM;        
  volatile uint32_t PATT;        
  uint32_t      RESERVED0;   
  volatile uint32_t ECCR;        
} FMC_Bank3_TypeDef;



 

typedef struct
{
  volatile uint32_t MODER;        
  volatile uint32_t OTYPER;       
  volatile uint32_t OSPEEDR;      
  volatile uint32_t PUPDR;        
  volatile uint32_t IDR;          
  volatile uint32_t ODR;          
  volatile uint32_t BSRR;         
  volatile uint32_t LCKR;         
  volatile uint32_t AFR[2];       
  volatile uint32_t BRR;          

} GPIO_TypeDef;




 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t OAR1;         
  volatile uint32_t OAR2;         
  volatile uint32_t TIMINGR;      
  volatile uint32_t TIMEOUTR;     
  volatile uint32_t ISR;          
  volatile uint32_t ICR;          
  volatile uint32_t PECR;         
  volatile uint32_t RXDR;         
  volatile uint32_t TXDR;         
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;           
  volatile uint32_t PR;           
  volatile uint32_t RLR;          
  volatile uint32_t SR;           
  volatile uint32_t WINR;         
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;           
  volatile uint32_t FCR;          
  volatile uint32_t SR;           
  volatile uint32_t CLR;          
  uint32_t RESERVED;          
  volatile uint32_t RAM[16];      
} LCD_TypeDef;



 
typedef struct
{
  volatile uint32_t ISR;          
  volatile uint32_t ICR;          
  volatile uint32_t IER;          
  volatile uint32_t CFGR;         
  volatile uint32_t CR;           
  volatile uint32_t CMP;          
  volatile uint32_t ARR;          
  volatile uint32_t CNT;          
  volatile uint32_t OR;           
} LPTIM_TypeDef;



 

typedef struct
{
  volatile uint32_t CSR;          
  volatile uint32_t OTR;          
  volatile uint32_t LPOTR;        
} OPAMP_TypeDef;

typedef struct
{
  volatile uint32_t CSR;          
} OPAMP_Common_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;    
  volatile uint32_t CR2;    
  volatile uint32_t CR3;    
  volatile uint32_t CR4;    
  volatile uint32_t SR1;    
  volatile uint32_t SR2;    
  volatile uint32_t SCR;    
  uint32_t RESERVED;    
  volatile uint32_t PUCRA;  
  volatile uint32_t PDCRA;  
  volatile uint32_t PUCRB;  
  volatile uint32_t PDCRB;  
  volatile uint32_t PUCRC;  
  volatile uint32_t PDCRC;  
  volatile uint32_t PUCRD;  
  volatile uint32_t PDCRD;  
  volatile uint32_t PUCRE;  
  volatile uint32_t PDCRE;  
  volatile uint32_t PUCRF;  
  volatile uint32_t PDCRF;  
  volatile uint32_t PUCRG;  
  volatile uint32_t PDCRG;  
  volatile uint32_t PUCRH;  
  volatile uint32_t PDCRH;  
  volatile uint32_t PUCRI;  
  volatile uint32_t PDCRI;  
} PWR_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;           
  volatile uint32_t DCR;          
  volatile uint32_t SR;           
  volatile uint32_t FCR;          
  volatile uint32_t DLR;          
  volatile uint32_t CCR;          
  volatile uint32_t AR;           
  volatile uint32_t ABR;          
  volatile uint32_t DR;           
  volatile uint32_t PSMKR;        
  volatile uint32_t PSMAR;        
  volatile uint32_t PIR;          
  volatile uint32_t LPTR;         
} QUADSPI_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;           
  volatile uint32_t ICSCR;        
  volatile uint32_t CFGR;         
  volatile uint32_t PLLCFGR;      
  volatile uint32_t PLLSAI1CFGR;  
  volatile uint32_t PLLSAI2CFGR;  
  volatile uint32_t CIER;         
  volatile uint32_t CIFR;         
  volatile uint32_t CICR;         
  uint32_t      RESERVED0;    
  volatile uint32_t AHB1RSTR;     
  volatile uint32_t AHB2RSTR;     
  volatile uint32_t AHB3RSTR;     
  uint32_t      RESERVED1;    
  volatile uint32_t APB1RSTR1;    
  volatile uint32_t APB1RSTR2;    
  volatile uint32_t APB2RSTR;     
  uint32_t      RESERVED2;    
  volatile uint32_t AHB1ENR;      
  volatile uint32_t AHB2ENR;      
  volatile uint32_t AHB3ENR;      
  uint32_t      RESERVED3;    
  volatile uint32_t APB1ENR1;     
  volatile uint32_t APB1ENR2;     
  volatile uint32_t APB2ENR;      
  uint32_t      RESERVED4;    
  volatile uint32_t AHB1SMENR;    
  volatile uint32_t AHB2SMENR;    
  volatile uint32_t AHB3SMENR;    
  uint32_t      RESERVED5;    
  volatile uint32_t APB1SMENR1;   
  volatile uint32_t APB1SMENR2;   
  volatile uint32_t APB2SMENR;    
  uint32_t      RESERVED6;    
  volatile uint32_t CCIPR;        
  uint32_t      RESERVED7;    
  volatile uint32_t BDCR;         
  volatile uint32_t CSR;          
  volatile uint32_t CRRCR;        
  volatile uint32_t CCIPR2;       
} RCC_TypeDef;



 

typedef struct
{
  volatile uint32_t TR;           
  volatile uint32_t DR;           
  volatile uint32_t CR;           
  volatile uint32_t ISR;          
  volatile uint32_t PRER;         
  volatile uint32_t WUTR;         
       uint32_t reserved;     
  volatile uint32_t ALRMAR;       
  volatile uint32_t ALRMBR;       
  volatile uint32_t WPR;          
  volatile uint32_t SSR;          
  volatile uint32_t SHIFTR;       
  volatile uint32_t TSTR;         
  volatile uint32_t TSDR;         
  volatile uint32_t TSSSR;        
  volatile uint32_t CALR;         
  volatile uint32_t TAMPCR;       
  volatile uint32_t ALRMASSR;     
  volatile uint32_t ALRMBSSR;     
  volatile uint32_t OR;           
  volatile uint32_t BKP0R;        
  volatile uint32_t BKP1R;        
  volatile uint32_t BKP2R;        
  volatile uint32_t BKP3R;        
  volatile uint32_t BKP4R;        
  volatile uint32_t BKP5R;        
  volatile uint32_t BKP6R;        
  volatile uint32_t BKP7R;        
  volatile uint32_t BKP8R;        
  volatile uint32_t BKP9R;        
  volatile uint32_t BKP10R;       
  volatile uint32_t BKP11R;       
  volatile uint32_t BKP12R;       
  volatile uint32_t BKP13R;       
  volatile uint32_t BKP14R;       
  volatile uint32_t BKP15R;       
  volatile uint32_t BKP16R;       
  volatile uint32_t BKP17R;       
  volatile uint32_t BKP18R;       
  volatile uint32_t BKP19R;       
  volatile uint32_t BKP20R;       
  volatile uint32_t BKP21R;       
  volatile uint32_t BKP22R;       
  volatile uint32_t BKP23R;       
  volatile uint32_t BKP24R;       
  volatile uint32_t BKP25R;       
  volatile uint32_t BKP26R;       
  volatile uint32_t BKP27R;       
  volatile uint32_t BKP28R;       
  volatile uint32_t BKP29R;       
  volatile uint32_t BKP30R;       
  volatile uint32_t BKP31R;       
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t GCR;          
} SAI_TypeDef;

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t FRCR;         
  volatile uint32_t SLOTR;        
  volatile uint32_t IMR;          
  volatile uint32_t SR;           
  volatile uint32_t CLRFR;        
  volatile uint32_t DR;           
} SAI_Block_TypeDef;




 

typedef struct
{
  volatile uint32_t POWER;           
  volatile uint32_t CLKCR;           
  volatile uint32_t ARG;             
  volatile uint32_t CMD;             
  volatile const uint32_t  RESPCMD;         
  volatile const uint32_t  RESP1;           
  volatile const uint32_t  RESP2;           
  volatile const uint32_t  RESP3;           
  volatile const uint32_t  RESP4;           
  volatile uint32_t DTIMER;          
  volatile uint32_t DLEN;            
  volatile uint32_t DCTRL;           
  volatile const uint32_t  DCOUNT;          
  volatile const uint32_t  STA;             
  volatile uint32_t ICR;             
  volatile uint32_t MASK;            
  uint32_t      RESERVED0[2];    
  volatile const uint32_t  FIFOCNT;         
  uint32_t      RESERVED1[13];   
  volatile uint32_t FIFO;            
} SDMMC_TypeDef;




 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t SR;           
  volatile uint32_t DR;           
  volatile uint32_t CRCPR;        
  volatile uint32_t RXCRCR;       
  volatile uint32_t TXCRCR;       
} SPI_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;           
  volatile uint32_t BRR;          
    uint32_t  RESERVED1;      
  volatile uint32_t ISR;          
  volatile uint32_t ICR;          
  volatile uint32_t IER;          
  volatile uint32_t RFL;          
  volatile uint32_t TDR;          
  volatile uint32_t RDR;          
  volatile uint32_t OR;           
} SWPMI_TypeDef;




 

typedef struct
{
  volatile uint32_t MEMRMP;       
  volatile uint32_t CFGR1;        
  volatile uint32_t EXTICR[4];    
  volatile uint32_t SCSR;         
  volatile uint32_t CFGR2;        
  volatile uint32_t SWPR;         
  volatile uint32_t SKR;          
  volatile uint32_t SWPR2;        
} SYSCFG_TypeDef;




 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t SMCR;         
  volatile uint32_t DIER;         
  volatile uint32_t SR;           
  volatile uint32_t EGR;          
  volatile uint32_t CCMR1;        
  volatile uint32_t CCMR2;        
  volatile uint32_t CCER;         
  volatile uint32_t CNT;          
  volatile uint32_t PSC;          
  volatile uint32_t ARR;          
  volatile uint32_t RCR;          
  volatile uint32_t CCR1;         
  volatile uint32_t CCR2;         
  volatile uint32_t CCR3;         
  volatile uint32_t CCR4;         
  volatile uint32_t BDTR;         
  volatile uint32_t DCR;          
  volatile uint32_t DMAR;         
  volatile uint32_t OR1;          
  volatile uint32_t CCMR3;        
  volatile uint32_t CCR5;         
  volatile uint32_t CCR6;         
  volatile uint32_t OR2;          
  volatile uint32_t OR3;          
} TIM_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;             
  volatile uint32_t IER;            
  volatile uint32_t ICR;            
  volatile uint32_t ISR;            
  volatile uint32_t IOHCR;          
  uint32_t      RESERVED1;      
  volatile uint32_t IOASCR;         
  uint32_t      RESERVED2;      
  volatile uint32_t IOSCR;          
  uint32_t      RESERVED3;      
  volatile uint32_t IOCCR;          
  uint32_t      RESERVED4;      
  volatile uint32_t IOGCSR;         
  volatile uint32_t IOGXCR[8];      
} TSC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t CR3;          
  volatile uint32_t BRR;          
  volatile uint16_t GTPR;         
  uint16_t  RESERVED2;        
  volatile uint32_t RTOR;         
  volatile uint16_t RQR;          
  uint16_t  RESERVED3;        
  volatile uint32_t ISR;          
  volatile uint32_t ICR;          
  volatile uint16_t RDR;          
  uint16_t  RESERVED4;        
  volatile uint16_t TDR;          
  uint16_t  RESERVED5;        
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CSR;          
  volatile uint32_t CCR;          
} VREFBUF_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;           
  volatile uint32_t CFR;          
  volatile uint32_t SR;           
} WWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;   
  volatile uint32_t SR;   
  volatile uint32_t DR;   
} RNG_TypeDef;



 
typedef struct
{
  volatile uint32_t GOTGCTL;               
  volatile uint32_t GOTGINT;               
  volatile uint32_t GAHBCFG;               
  volatile uint32_t GUSBCFG;               
  volatile uint32_t GRSTCTL;               
  volatile uint32_t GINTSTS;               
  volatile uint32_t GINTMSK;               
  volatile uint32_t GRXSTSR;               
  volatile uint32_t GRXSTSP;               
  volatile uint32_t GRXFSIZ;               
  volatile uint32_t DIEPTXF0_HNPTXFSIZ;    
  volatile uint32_t HNPTXSTS;              
  uint32_t Reserved30[2];              
  volatile uint32_t GCCFG;                 
  volatile uint32_t CID;                   
  volatile uint32_t GSNPSID;               
  volatile uint32_t GHWCFG1;               
  volatile uint32_t GHWCFG2;               
  volatile uint32_t GHWCFG3;               
  uint32_t  Reserved6;                 
  volatile uint32_t GLPMCFG;               
  volatile uint32_t GPWRDN;                
  volatile uint32_t GDFIFOCFG;             
   volatile uint32_t GADPCTL;              
    uint32_t  Reserved43[39];          
  volatile uint32_t HPTXFSIZ;              
  volatile uint32_t DIEPTXF[0x0F];         
} USB_OTG_GlobalTypeDef;



 
typedef struct
{
  volatile uint32_t DCFG;         
  volatile uint32_t DCTL;         
  volatile uint32_t DSTS;         
  uint32_t Reserved0C;        
  volatile uint32_t DIEPMSK;      
  volatile uint32_t DOEPMSK;      
  volatile uint32_t DAINT;        
  volatile uint32_t DAINTMSK;     
  uint32_t Reserved20;        
  uint32_t Reserved24;        
  volatile uint32_t DVBUSDIS;     
  volatile uint32_t DVBUSPULSE;   
  volatile uint32_t DTHRCTL;      
  volatile uint32_t DIEPEMPMSK;   
  volatile uint32_t DEACHINT;     
  volatile uint32_t DEACHMSK;     
  uint32_t Reserved40;        
  volatile uint32_t DINEP1MSK;    
  uint32_t  Reserved44[15];   
  volatile uint32_t DOUTEP1MSK;   
} USB_OTG_DeviceTypeDef;



 
typedef struct
{
  volatile uint32_t DIEPCTL;      
  uint32_t Reserved04;        
  volatile uint32_t DIEPINT;      
  uint32_t Reserved0C;        
  volatile uint32_t DIEPTSIZ;     
  volatile uint32_t DIEPDMA;      
  volatile uint32_t DTXFSTS;      
  uint32_t Reserved18;        
} USB_OTG_INEndpointTypeDef;



 
typedef struct
{
  volatile uint32_t DOEPCTL;      
  uint32_t Reserved04;        
  volatile uint32_t DOEPINT;      
  uint32_t Reserved0C;        
  volatile uint32_t DOEPTSIZ;     
  volatile uint32_t DOEPDMA;      
  uint32_t Reserved18[2];     
} USB_OTG_OUTEndpointTypeDef;



 
typedef struct
{
  volatile uint32_t HCFG;         
  volatile uint32_t HFIR;         
  volatile uint32_t HFNUM;        
  uint32_t Reserved40C;       
  volatile uint32_t HPTXSTS;      
  volatile uint32_t HAINT;        
  volatile uint32_t HAINTMSK;     
} USB_OTG_HostTypeDef;



 
typedef struct
{
  volatile uint32_t HCCHAR;
  volatile uint32_t HCSPLT;
  volatile uint32_t HCINT;
  volatile uint32_t HCINTMSK;
  volatile uint32_t HCTSIZ;
  volatile uint32_t HCDMA;
  uint32_t Reserved[2];
} USB_OTG_HostChannelTypeDef;



 



 
#line 1209 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"






 











 





#line 1239 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 1274 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"


 
#line 1310 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 1319 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"


#line 1329 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"


#line 1339 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"


 
#line 1351 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"













 




 


 


#line 1387 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







 



 
#line 1434 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1468 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"
 
#line 1487 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1504 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"


#line 1514 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"


#line 1524 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"













 



 



 

 
 
 

 
 
 
 
 



 


 
#line 1596 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 1631 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 1644 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 1676 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 1684 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"















#line 1706 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 1722 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





#line 1733 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1752 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1761 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





 
#line 1773 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1780 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1788 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1795 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 1803 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1810 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1817 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1824 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1831 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1838 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1845 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1852 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1859 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1866 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





 
#line 1878 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1885 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1892 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1899 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1906 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1913 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1920 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1927 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1934 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 1951 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1967 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 1980 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 1992 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2005 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2017 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2026 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2035 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2044 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2053 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2062 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2072 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2081 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2090 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2099 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2108 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2118 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2127 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2136 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2145 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2154 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2164 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2173 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2194 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 






#line 2209 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 2224 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2233 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2242 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2251 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2268 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2277 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





 
#line 2298 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2307 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





 
#line 2328 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2337 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





 
#line 2358 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2367 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





 
#line 2392 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2413 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2434 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2455 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2479 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2503 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2527 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2539 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2550 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
#line 2586 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2620 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2630 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2638 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

















#line 2662 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2672 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2693 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2713 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
 
 
 
 
 
#line 2748 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2777 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2827 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2840 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2853 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2867 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2881 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2925 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2936 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2943 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 2950 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 2979 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
#line 2997 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3008 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3022 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3036 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3053 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3064 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3078 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3092 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3109 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3120 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3134 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3148 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3162 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3173 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3187 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3201 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3215 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3226 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3240 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3254 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
#line 3263 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3310 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3357 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3404 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3451 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3549 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3647 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3745 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3843 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 3941 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 4039 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 4137 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 4235 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 4333 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 4431 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 4529 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 4627 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 4725 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 4823 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 4921 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 5019 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 5117 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 5215 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 5313 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 5411 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 5509 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 5607 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 5705 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 5803 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 5901 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 5999 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6097 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6195 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
 
 
 
 




 




 
#line 6228 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 
 
 
 

 
#line 6275 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6283 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 6290 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"











 
#line 6329 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6343 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
 
 
 


 


 
#line 6361 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 6368 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 6382 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 6392 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 6399 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 6406 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 6420 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 6430 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6438 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 




 




 




 




 
#line 6476 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6484 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6492 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 
#line 6513 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 6523 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6531 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6539 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 6546 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 
#line 6564 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6572 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
 
 
 
 
#line 6630 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6641 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6658 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6678 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6695 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6712 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6758 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6804 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6839 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6875 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6921 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
 
 
 

 

 
#line 6972 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6980 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 6996 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
#line 7009 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 

 
#line 7060 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 7089 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 7118 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 7132 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
#line 7151 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 7159 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 7170 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 7178 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 7186 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 7194 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 7202 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 7210 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 7218 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
 
 
 
 

 
#line 7315 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 7401 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 7427 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"























 




 




 





 
#line 7488 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
 
 
 

 

#line 7529 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 

#line 7550 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 

#line 7571 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 





 





 





 





 

#line 7628 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 

#line 7640 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 

#line 7673 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 

#line 7685 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 





 





 

#line 7712 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 

 






 




 





 





 





 





 

#line 7759 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 





 

#line 7774 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 

 

 
 
 
 
 
 
#line 7884 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 7982 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8050 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8118 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8186 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8254 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8286 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8318 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8332 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8346 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8360 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8374 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"


 
 
 
 
 
 
#line 8411 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8449 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8496 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8516 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8571 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
#line 8584 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8592 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8600 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 
#line 8618 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8626 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"


 
 
 
 
 
 
#line 8640 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8648 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"













#line 8685 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8692 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





 
#line 8705 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8713 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8725 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8733 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8741 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8749 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







 
#line 8764 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8772 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8784 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8792 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







 
#line 8809 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"











#line 8827 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8835 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8842 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8865 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8878 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8890 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8902 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8914 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 8927 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8939 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8951 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 8963 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
 
 
 
 
 
#line 9055 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9105 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9155 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9173 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9255 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9305 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9387 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9437 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9487 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9505 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9523 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9573 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9591 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9609 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9707 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9741 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9794 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9852 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9862 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9920 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9930 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9980 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 9998 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 
 
 
 
 
 
#line 10070 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10105 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10116 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10149 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10166 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10183 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10236 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10265 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 




 
 
 
 
 
 




 
#line 10298 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
#line 10314 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
 
 
 
 

 
#line 10345 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10356 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
 
 
 

 

#line 10392 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"


 
#line 10401 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"
 
#line 10417 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"
 
#line 10446 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10475 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 





 
#line 10504 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10530 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10556 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10579 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10626 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10670 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10720 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10767 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10817 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10867 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10917 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 10967 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11017 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11067 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11117 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11167 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11217 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11267 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11317 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11367 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11405 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11443 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"


 
 
 
 
 


 
#line 11460 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11474 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11491 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 11504 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 11517 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 11536 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
#line 11550 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11563 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11576 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11588 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 











 











 
#line 11622 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 11632 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11640 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







 
#line 11654 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"











 
#line 11673 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 11680 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







 
#line 11694 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




#line 11709 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 11716 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 11727 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 11737 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 11752 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 11761 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11773 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 11780 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 11789 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 11798 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 11807 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11819 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 11826 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 11835 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 11844 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11876 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11911 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11946 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 11966 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12007 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12015 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12086 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12100 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12138 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12158 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12199 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12207 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12284 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12298 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12339 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12362 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12406 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12414 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12491 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12505 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12543 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 


































































































 
#line 12653 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 12666 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 12685 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12693 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 12701 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 12729 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12737 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12751 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 






 
 
 
 
 
 
#line 12771 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12788 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
 
 
 


 






 



 
#line 12849 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12893 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 12963 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
#line 13024 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 13032 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
#line 13107 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 13177 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 
#line 13195 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 13238 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 13268 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
#line 13296 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 13368 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 13380 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 13392 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 13400 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"


 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 
 
 
 
 
 












 












#line 13599 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 13606 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 13628 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 13636 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 13644 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 13657 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"


#line 13668 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 13677 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"


 
#line 13691 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 13702 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 13712 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 13722 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 13736 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





 
#line 13763 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 13786 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 13793 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 13816 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
 
 
 
 

 
#line 13835 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 13842 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 13855 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 13866 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 13873 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 13880 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 13887 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 13894 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 13907 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 13927 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 13935 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
 
 
 
 
 






 
#line 13966 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 13979 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 










#line 14008 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 




 




 




 




 




 
#line 14057 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 14065 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 14078 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
#line 14154 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 14192 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 14260 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 
 
 
 
 
 
#line 14286 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 14293 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 14324 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 14366 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 14405 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 




 




 
 
 
 
 
 
#line 14480 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 14494 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 14517 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 14531 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
#line 14588 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 




 




 




 




 




 
 
 
 
 
 
#line 14636 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





 
#line 14678 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 14692 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 
#line 14705 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 
#line 14718 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 
#line 14731 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 
#line 14744 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 14758 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"


 
#line 14770 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 
#line 14783 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 
#line 14796 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 
#line 14809 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 14823 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 
#line 14836 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 
#line 14849 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 
#line 14862 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 
#line 14875 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 14889 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 
#line 14901 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 
#line 14913 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 
#line 14925 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



 
#line 14937 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 14945 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 14962 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 15060 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 15158 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 







 
 
 
 
 
 
#line 15188 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





















 
#line 15219 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 15226 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 15257 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 15265 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 15274 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





#line 15285 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





#line 15297 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 15310 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 15357 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 15407 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"


 
#line 15437 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"


 






#line 15452 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 15460 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"











#line 15477 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 15485 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





 






#line 15504 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 15518 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 






#line 15532 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 15540 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"











#line 15557 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 15565 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





 






#line 15584 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 15598 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 15606 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 15614 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





#line 15625 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 15633 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





 
#line 15696 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 15704 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 




 




 




 




 




 
#line 15753 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
#line 15771 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 15796 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 15803 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 15810 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 15820 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 15829 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
















 
#line 15874 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 15881 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 15904 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
















 
#line 15944 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 15951 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 15974 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 15982 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







 
#line 15996 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 






 
#line 16011 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 










 
#line 16045 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16053 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16076 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 






 
#line 16106 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
 
 
 
 
#line 16134 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16157 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16180 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 






















#line 16210 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 16217 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







#line 16242 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16253 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 




 






 
 
 
 
 
 










#line 16298 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"




















#line 16324 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 16331 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"









 
 
 
 
 
 
#line 16352 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



















#line 16386 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16394 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



















#line 16428 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





 
#line 16440 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"



















#line 16474 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16482 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16490 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16498 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16506 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16514 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16522 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
 
 
 
 
#line 16544 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 16551 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 16558 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 16565 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 16576 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 16584 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 16592 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16600 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16608 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16616 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16714 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16812 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 16910 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17008 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17058 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
 
 
 
 



 


 
#line 17152 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17212 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17282 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17290 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17298 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17306 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17323 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17394 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17435 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 




 
 
 
 
 

 
#line 17479 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
#line 17519 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17542 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17571 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17579 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 
#line 17597 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
 
 
 
 
#line 17616 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
 
 
 
 
 
#line 17638 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"





 
#line 17654 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"











 





 
 
 
 
 
 
#line 17683 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17697 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







 
#line 17750 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17758 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17775 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
 
 
 
 
#line 17809 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17829 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17850 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17913 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 17944 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18027 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18110 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 
#line 18120 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"
 
#line 18135 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"
 
#line 18151 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18165 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
#line 18184 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18200 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 18211 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18243 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
#line 18295 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 
#line 18313 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18321 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18331 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
#line 18344 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18360 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 18372 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 
#line 18411 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 18420 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

#line 18428 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"







 




#line 18453 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"













#line 18485 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18520 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18555 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18571 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
#line 18606 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18641 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18657 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18683 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18712 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18735 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18770 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18778 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18786 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 
#line 18834 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 
#line 18847 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18855 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18902 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18937 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 18948 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 




 




 
#line 18998 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19018 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19031 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19042 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19093 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"




 



 



 

 








 



 





 


 


 


 





#line 19149 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 


 


 
#line 19171 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19182 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
 


 
 


 





 


 


 


 





 


 


 


 


 





 


 





 




 


 



 


 
#line 19266 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 



 






 






 



 
#line 19299 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19308 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19316 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19324 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 



 



 






 
#line 19352 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19363 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19374 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19426 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19447 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19458 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19467 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19475 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19484 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19493 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 



 






 
#line 19512 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19520 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19528 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19536 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 





 
#line 19553 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19562 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19570 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19579 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 






 



 
#line 19599 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 



 


 




 






 






 
#line 19633 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19641 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19649 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 






 
#line 19664 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 






 




 


 


 




 


 
 
 
 
 
 
 

 
#line 19711 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"

 
#line 19724 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l496xx.h"









 

  

 

 
#line 157 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l4xx.h"
#line 178 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l4xx.h"



 



 
typedef enum
{
  RESET = 0,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;


typedef enum
{
  SUCCESS = 0,
  ERROR = !SUCCESS
} ErrorStatus;



 




 



















 

#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h"


















 

 
#line 728 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h"

 
#line 236 "../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l4xx.h"









 



 




 
#line 31 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_def.h"
#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


















 

 







 
 
 



 







 



 
#line 88 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 96 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






 



 





 



 
#line 134 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"




#line 144 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 166 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"




 
 
#line 179 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 189 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 199 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"




 



 



 



 






 



 

#line 237 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"













 



 
#line 269 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"







#line 302 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"












#line 323 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 385 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 

#line 481 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

#line 498 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

#line 523 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 




 
#line 542 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 





 



 


















#line 592 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"





#line 603 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 610 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"










 



 
#line 634 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 643 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 654 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 766 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 783 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 
#line 806 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 





 



 






 



 















 
 





 



 














 



 










 



 







































 



 


#line 948 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






 



 

 
#line 970 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

 












 



 




























#line 1026 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 




 















 




 
#line 1067 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 









#line 1097 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 



#line 1135 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1145 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1164 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"










#line 1191 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 




 



 

























 




 








 



 




 



 
#line 1271 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

#line 1288 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1300 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1331 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 











 






 






#line 1375 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 




 



 

 



 



 



 
#line 1407 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

 





































 



 
#line 1474 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 
#line 1489 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 








#line 1517 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1528 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 

#line 1558 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1566 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






#line 1582 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"





 



 





 



 



 



 
#line 1622 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 



 



 






 




 



 

 



 





 



 
#line 1683 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"









 




 
#line 1711 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1732 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1743 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1752 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1765 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1774 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 







 



 
#line 1810 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1825 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


#line 1858 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 
#line 2025 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"




 
 
 
 




 




 




 




 







 



 

#line 2072 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

#line 2100 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 










 



 














 




 




 




 







 




 
#line 2178 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 




 
#line 2222 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2236 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 




 








#line 2510 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2524 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2741 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2755 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2762 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2783 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2931 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

 



#line 2956 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2977 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3094 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3103 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3120 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3135 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






#line 3164 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

















#line 3190 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"





#line 3217 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"







#line 3232 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3265 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3283 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"












#line 3301 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3322 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3330 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 




 



 
#line 3353 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3381 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3396 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






 



 




#line 3436 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3458 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"
 




#line 3469 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3481 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 

#line 3495 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"








 



 
#line 3516 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 







 



 













 




 









#line 3570 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 












#line 3596 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3605 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3614 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"








 



 








#line 3647 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"




 



 

#line 3664 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






 



 




 



 
#line 3698 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 







 



 
#line 3725 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 





 



 



 







 

#line 32 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_def.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"
 






 

 
 
 





 





#line 34 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"




  typedef signed int ptrdiff_t;



  



    typedef unsigned int size_t;    
#line 57 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



   



      typedef unsigned short wchar_t;  
#line 82 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



    




   




  typedef long double max_align_t;









#line 114 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



 

#line 33 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_def.h"

 



 
typedef enum
{
  HAL_OK       = 0x00,
  HAL_ERROR    = 0x01,
  HAL_BUSY     = 0x02,
  HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;



 
typedef enum
{
  HAL_UNLOCKED = 0x00,
  HAL_LOCKED   = 0x01
} HAL_LockTypeDef;

 




























 


#line 103 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_def.h"








#line 126 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_def.h"


 
#line 155 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_def.h"



 









 


#line 187 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_def.h"



 



 


#line 204 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_def.h"








 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"



 



 

 


 



 
typedef struct
{
  uint32_t PLLState;   
 

  uint32_t PLLSource;  
 

  uint32_t PLLM;       

 

  uint32_t PLLN;       
 


  uint32_t PLLP;       
 


  uint32_t PLLQ;       
 

  uint32_t PLLR;       


 

}RCC_PLLInitTypeDef;



 
typedef struct
{
  uint32_t OscillatorType;       
 

  uint32_t HSEState;             
 

  uint32_t LSEState;             
 

  uint32_t HSIState;             
 

  uint32_t HSICalibrationValue;  



 

  uint32_t LSIState;             
 






  uint32_t MSIState;             
 

  uint32_t MSICalibrationValue;  
 

  uint32_t MSIClockRange;        
 

  uint32_t HSI48State;             
 

  RCC_PLLInitTypeDef PLL;         

}RCC_OscInitTypeDef;



 
typedef struct
{
  uint32_t ClockType;             
 

  uint32_t SYSCLKSource;          
 

  uint32_t AHBCLKDivider;         
 

  uint32_t APB1CLKDivider;        
 

  uint32_t APB2CLKDivider;        
 

}RCC_ClkInitTypeDef;



 

 


 



 




 



 
#line 176 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"


 



 





 



 
#line 200 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"


 



 










 



 




 
#line 238 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"



 






 




 




 
#line 268 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"



 





 




 
#line 318 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"


 




 






 



 






 



 






 



 
#line 366 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"


 




 





 






 
#line 396 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"


 





 
#line 417 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"


 



 






 



 






 



 






 



 
#line 466 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"


 



 







 



 






 



 




 



 
#line 516 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"


 



 







 



 
#line 552 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"


 










 
 
#line 577 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

 



 
#line 592 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"


 




 



 






 



 




 



 

 



 







 

#line 646 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 654 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 664 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 672 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 680 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 688 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 698 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 708 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"


























 







 

#line 751 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 759 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 767 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 777 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 787 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 797 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 807 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 815 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 825 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 835 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 843 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 853 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 863 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 873 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 883 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 891 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 901 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 911 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 921 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"




































































 







 

#line 1008 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1018 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1028 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1038 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"



















 







 

#line 1074 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1084 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1094 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1104 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1112 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1122 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1132 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1142 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1150 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1160 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1170 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1178 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1188 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1198 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1208 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1216 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1226 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1234 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1244 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1254 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1264 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1274 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1284 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1292 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1302 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1310 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1318 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1326 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1336 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1344 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"




































































































 







 

#line 1461 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1469 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1479 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1487 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1495 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1505 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1513 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"


#line 1522 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1530 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1540 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1550 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1560 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1570 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1580 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 1590 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"
















































 







 
















































 







 




































































































































 







 



































 







 








































































































































































































 







 






























































































 




 



















































 




 







































































































































 




 






































 




 





































































































































































































 




 































































































 








 




















































 








 
















































































































































 








 



































 








 








































































































































































































 








 




























































































 








 




















































 








 
















































































































































 








 




































 








 








































































































































































































 








 




























































































 



 






 






 



 








 






 
















 












 









 












 




















 













 




























 
















 


















 













 


























 
#line 4135 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"




















 
#line 4173 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"









 





























 










 









 















 













 









































 


#line 4318 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 4345 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"








 















 














 











 










 















 










 




























 






 




















 





















 






















 






















 






 





























 
#line 4609 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"



 



 

 


 
 
#line 4629 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"




 

 


 

#line 4656 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"




#line 4667 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"















































#line 4720 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 4733 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

#line 4746 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"






























#line 4796 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"














 

 
#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"



 



 

 



 




 
typedef struct
{

  uint32_t PLLSAI1Source;    
 





  uint32_t PLLSAI1M;         
 


  uint32_t PLLSAI1N;         
 

  uint32_t PLLSAI1P;         
 

  uint32_t PLLSAI1Q;         
 

  uint32_t PLLSAI1R;         
 

  uint32_t PLLSAI1ClockOut;  
 
}RCC_PLLSAI1InitTypeDef;





 
typedef struct
{

  uint32_t PLLSAI2Source;    
 





  uint32_t PLLSAI2M;         
 


  uint32_t PLLSAI2N;         
 

  uint32_t PLLSAI2P;         
 






  uint32_t PLLSAI2R;         
 

  uint32_t PLLSAI2ClockOut;  
 
}RCC_PLLSAI2InitTypeDef;





 
typedef struct
{
  uint32_t PeriphClockSelection;   
 


  RCC_PLLSAI1InitTypeDef PLLSAI1;  
 



  RCC_PLLSAI2InitTypeDef PLLSAI2;  
 



  uint32_t Usart1ClockSelection;   
 

  uint32_t Usart2ClockSelection;   
 



  uint32_t Usart3ClockSelection;   
 





  uint32_t Uart4ClockSelection;    
 





  uint32_t Uart5ClockSelection;    
 



  uint32_t Lpuart1ClockSelection;  
 

  uint32_t I2c1ClockSelection;     
 



  uint32_t I2c2ClockSelection;     
 



  uint32_t I2c3ClockSelection;     
 



  uint32_t I2c4ClockSelection;     
 



  uint32_t Lptim1ClockSelection;   
 

  uint32_t Lptim2ClockSelection;   
 


  uint32_t Sai1ClockSelection;     
 




  uint32_t Sai2ClockSelection;     
 





  uint32_t UsbClockSelection;      
 





  uint32_t Sdmmc1ClockSelection;   
 



  uint32_t RngClockSelection;      
 


  uint32_t AdcClockSelection;      
 




  uint32_t Swpmi1ClockSelection;   
 





  uint32_t Dfsdm1ClockSelection;   
 









#line 253 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 260 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 267 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

  uint32_t RTCClockSelection;      
 
}RCC_PeriphCLKInitTypeDef;





 
typedef struct
{
  uint32_t Prescaler;             
 

  uint32_t Source;                
 

  uint32_t Polarity;              
 

  uint32_t ReloadValue;           

 

  uint32_t ErrorLimitValue;       
 

  uint32_t HSI48CalibrationValue; 

 

}RCC_CRSInitTypeDef;



 
typedef struct
{
  uint32_t ReloadValue;           
 

  uint32_t HSI48CalibrationValue; 
 

  uint32_t FreqErrorCapture;      

 

  uint32_t FreqErrorDirection;    


 

}RCC_CRSSynchroInfoTypeDef;




 

 


 



 




 



 
#line 400 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"


 




 






 



 






 




 






 





 






 





 






 




 






 



 





 




 





 




 





 




 





 





 
#line 541 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"


 





 
#line 561 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"


 




 






 



 






 




 
#line 603 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"


 




 
#line 621 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"


 




 
#line 639 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"


 




 
#line 659 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"


 




 




 





 
#line 684 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"


 

#line 700 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 713 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 724 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 736 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"



 



 





 
#line 757 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"


 



 





 



 
#line 782 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"


 



 




 



 




 



 



 



 
#line 824 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"


 



 




 



 
#line 847 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"



 



 
#line 862 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"



 





 

 


 



































 
#line 942 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"



#line 953 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 966 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

















 



#line 1005 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"













 





#line 1030 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"












 














 







 

















 















 






































 

#line 1179 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 1192 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 1200 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 1212 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"


















 



#line 1252 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"












 



#line 1286 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"












 







 



























 

























 


























 
#line 1395 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"













 
























 
#line 1441 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"







 
















 








 












 








 












 








 












 








 














 









 











 









 













 









 















 









 















 









 













 









 











 









 











 









 


































 
#line 1785 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"




















 
#line 1813 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"



















 














 




















 














 
















 











 
#line 1924 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"









 







 











 
#line 1961 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"





 






#line 1995 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"



#line 2022 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 2043 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 2066 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"




 




 




 




 




 




 








 




 




 




 




 








 





 





 





 






 






 






 





 





 









 









 





 





 














 











 










 












 
 


#line 2281 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"













 















 

 


#line 2325 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"





 





 




 





 






 





 











 




 



#line 2412 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"



 

 


 



 

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void              HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
uint32_t          HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);



 



 


HAL_StatusTypeDef HAL_RCCEx_EnablePLLSAI1(RCC_PLLSAI1InitTypeDef  *PLLSAI1Init);
HAL_StatusTypeDef HAL_RCCEx_DisablePLLSAI1(void);





HAL_StatusTypeDef HAL_RCCEx_EnablePLLSAI2(RCC_PLLSAI2InitTypeDef  *PLLSAI2Init);
HAL_StatusTypeDef HAL_RCCEx_DisablePLLSAI2(void);



void              HAL_RCCEx_WakeUpStopCLKConfig(uint32_t WakeUpClk);
void              HAL_RCCEx_StandbyMSIRangeConfig(uint32_t MSIRange);
void              HAL_RCCEx_EnableLSECSS(void);
void              HAL_RCCEx_DisableLSECSS(void);
void              HAL_RCCEx_EnableLSECSS_IT(void);
void              HAL_RCCEx_LSECSS_IRQHandler(void);
void              HAL_RCCEx_LSECSS_Callback(void);
void              HAL_RCCEx_EnableLSCO(uint32_t LSCOSource);
void              HAL_RCCEx_DisableLSCO(void);
void              HAL_RCCEx_EnableMSIPLLMode(void);
void              HAL_RCCEx_DisableMSIPLLMode(void);






 





 

void              HAL_RCCEx_CRSConfig(RCC_CRSInitTypeDef *pInit);
void              HAL_RCCEx_CRSSoftwareSynchronizationGenerate(void);
void              HAL_RCCEx_CRSGetSynchronizationInfo(RCC_CRSSynchroInfoTypeDef *pSynchroInfo);
uint32_t          HAL_RCCEx_CRSWaitSynchronization(uint32_t Timeout);
void              HAL_RCCEx_CRS_IRQHandler(void);
void              HAL_RCCEx_CRS_SyncOkCallback(void);
void              HAL_RCCEx_CRS_SyncWarnCallback(void);
void              HAL_RCCEx_CRS_ExpectedSyncCallback(void);
void              HAL_RCCEx_CRS_ErrorCallback(uint32_t Error);



 





 

 


 




#line 2644 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 2667 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 2801 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"















































































#line 2894 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 2903 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"



#line 2920 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"















#line 2946 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 2962 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"



#line 2977 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 2987 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"




#line 3003 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 3014 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"









#line 3037 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"















#line 3060 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"



#line 3072 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 3080 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"

#line 3089 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc_ex.h"


































































































 



 



 







 
#line 4814 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rcc.h"

 


 




 

 
HAL_StatusTypeDef HAL_RCC_DeInit(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);



 



 

 
void              HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
void              HAL_RCC_EnableCSS(void);
uint32_t          HAL_RCC_GetSysClockFreq(void);
uint32_t          HAL_RCC_GetHCLKFreq(void);
uint32_t          HAL_RCC_GetPCLK1Freq(void);
uint32_t          HAL_RCC_GetPCLK2Freq(void);
void              HAL_RCC_GetOscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
void              HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t *pFLatency);
 
void              HAL_RCC_NMI_IRQHandler(void);
 
void              HAL_RCC_CSSCallback(void);



 



 



 



 







 
#line 199 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"


#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio.h"



 



 

 



 


 
typedef struct
{
  uint32_t Pin;        
 

  uint32_t Mode;       
 

  uint32_t Pull;       
 

  uint32_t Speed;      
 

  uint32_t Alternate;  
 
}GPIO_InitTypeDef;



 
typedef enum
{
  GPIO_PIN_RESET = 0U,
  GPIO_PIN_SET
}GPIO_PinState;


 

 


 


 
#line 101 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio.h"




 










 
#line 130 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio.h"


 




 






 

 


 





 



 

 


 






 







 







 







 







 




 

 


 





#line 232 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio.h"











 

 
#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"



 




 

 
 


 



 

#line 143 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"

#line 259 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"

#line 380 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"

#line 510 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"


 


 







 
#line 530 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"



 
#line 540 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"



 
#line 552 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"



 








 








 






 






 







 





 







 




 
#line 623 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"



 







 
#line 641 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"



 






#line 801 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"

#line 946 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"



 



 

 


 



 
#line 971 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"

#line 981 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"

#line 989 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"

#line 999 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"

#line 1011 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"



#line 1022 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"



#line 1037 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio_ex.h"



 



 

 


 



 







 
#line 247 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_gpio.h"

 


 




 

 
void              HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void              HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);



 



 

 
GPIO_PinState     HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void              HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void              HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void              HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
void              HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);



 



 



 



 







 
#line 203 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"


#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_dma.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_dma.h"



 



 

 


 



 
typedef struct
{
  uint32_t Request;                   
 

  uint32_t Direction;                 

 

  uint32_t PeriphInc;                 
 

  uint32_t MemInc;                    
 

  uint32_t PeriphDataAlignment;       
 

  uint32_t MemDataAlignment;          
 

  uint32_t Mode;                      


 

  uint32_t Priority;                  
 
} DMA_InitTypeDef;



 
typedef enum
{
  HAL_DMA_STATE_RESET             = 0x00U,   
  HAL_DMA_STATE_READY             = 0x01U,   
  HAL_DMA_STATE_BUSY              = 0x02U,   
  HAL_DMA_STATE_TIMEOUT           = 0x03U,   
}HAL_DMA_StateTypeDef;



 
typedef enum
{
  HAL_DMA_FULL_TRANSFER      = 0x00U,     
  HAL_DMA_HALF_TRANSFER      = 0x01U      
}HAL_DMA_LevelCompleteTypeDef;




 
typedef enum
{
  HAL_DMA_XFER_CPLT_CB_ID          = 0x00U,     
  HAL_DMA_XFER_HALFCPLT_CB_ID      = 0x01U,     
  HAL_DMA_XFER_ERROR_CB_ID         = 0x02U,     
  HAL_DMA_XFER_ABORT_CB_ID         = 0x03U,     
  HAL_DMA_XFER_ALL_CB_ID           = 0x04U      
}HAL_DMA_CallbackIDTypeDef;



 
typedef struct __DMA_HandleTypeDef
{
  DMA_Channel_TypeDef    *Instance;                                                      

  DMA_InitTypeDef       Init;                                                            

  HAL_LockTypeDef       Lock;                                                            

  volatile HAL_DMA_StateTypeDef  State;                                                      

  void                  *Parent;                                                         

  void                  (* XferCpltCallback)(struct __DMA_HandleTypeDef * hdma);         

  void                  (* XferHalfCpltCallback)(struct __DMA_HandleTypeDef * hdma);     

  void                  (* XferErrorCallback)(struct __DMA_HandleTypeDef * hdma);        

  void                  (* XferAbortCallback)(struct __DMA_HandleTypeDef * hdma);        

  volatile uint32_t         ErrorCode;                                                       

  DMA_TypeDef           *DmaBaseAddress;                                                 

  uint32_t              ChannelIndex;                                                    

#line 153 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_dma.h"

}DMA_HandleTypeDef;


 

 



 



 
#line 175 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_dma.h"



 



 


#line 193 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_dma.h"



#line 432 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_dma.h"



 



 





 



 




 



 




 



 





 



 





 



 




 



 






 




 





 



 
#line 547 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_dma.h"


 



 

 


 




 






 






 



 





 

#line 603 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_dma.h"





 
#line 623 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_dma.h"





 
#line 643 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_dma.h"





 
#line 663 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_dma.h"












 














 












 











 











 






 




 






 



 



 
 
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_DeInit (DMA_HandleTypeDef *hdma);


 



 
 
HAL_StatusTypeDef HAL_DMA_Start (DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, HAL_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)( DMA_HandleTypeDef * _hdma));
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID);



 



 
 
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
uint32_t             HAL_DMA_GetError(DMA_HandleTypeDef *hdma);


 



 

 


 















#line 821 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_dma.h"

























 

 



 



 







 
#line 207 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"





   








#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_cortex.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_cortex.h"



 




 

 


 





 
typedef struct
{
  uint8_t                Enable;                
 
  uint8_t                Number;                
 
  uint32_t               BaseAddress;            
  uint8_t                Size;                  
 
  uint8_t                SubRegionDisable;      
 
  uint8_t                TypeExtField;          
 
  uint8_t                AccessPermission;      
 
  uint8_t                DisableExec;           
 
  uint8_t                IsShareable;           
 
  uint8_t                IsCacheable;           
 
  uint8_t                IsBufferable;          
 
}MPU_Region_InitTypeDef;


 




 

 



 



 
#line 102 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_cortex.h"


 



 





 




 






 



 




 



 




 



 




 



 




 



 




 



 






 



 
#line 215 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_cortex.h"


 



 
#line 228 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_cortex.h"


 



 
#line 243 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_cortex.h"


 




 

 


 



 

 


 




 
 
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
void HAL_NVIC_SystemReset(void);
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);



 




 
 
uint32_t HAL_NVIC_GetPriorityGrouping(void);
void HAL_NVIC_GetPriority(IRQn_Type IRQn, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority);
uint32_t HAL_NVIC_GetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type IRQn);
uint32_t HAL_NVIC_GetActive(IRQn_Type IRQn);
void HAL_SYSTICK_CLKSourceConfig(uint32_t CLKSource);
void HAL_SYSTICK_IRQHandler(void);
void HAL_SYSTICK_Callback(void);


void HAL_MPU_Enable(uint32_t MPU_Control);
void HAL_MPU_Disable(void);
void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *MPU_Init);



 



 

  
 
 
 


 




































#line 359 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_cortex.h"

#line 368 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_cortex.h"

#line 397 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_cortex.h"






 

 



 



 








 
#line 223 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"


#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"

 
#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"



 





 

 
 

 


 

 
 
 
 

 
 










 
 
#line 84 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"



 
 
 
 

 
 










 
 







 
 
 
 


 
 
 





 
 
 





 





 
 
 
 


 
 
 





 
 
 





 








 
 
 
 
 
 
 
 





 


 




 
 








 
 
#line 225 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"

 
 
#line 247 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"

 
 
#line 269 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 
 
 
 
 
 
#line 284 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"

 
 
 
 
 
 
 
 

 




 
 











 
#line 321 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"

 
 
#line 330 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 







 



 
 


 
#line 358 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"



 


 


 








 





 


 
#line 582 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"

 


 




 
#line 627 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 




 
#line 646 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 



 
 
 
 






 



 
#line 682 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 



 
 
 
 
 
 






 



 






 



 




 



 




 



 






 



 




 



 





 



 
#line 797 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 



 
#line 821 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 



 





 



 




 



 





 




 




 





 




 




 




 



 
#line 904 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 



 
#line 920 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 



 
#line 943 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 



 
#line 967 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 



 





 



 




 



 





 



 






 



 




 



 






 



 
#line 1042 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 



 





 



 





 



 
#line 1162 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 



 





 



 







 



 




 



 
#line 1208 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 



 
#line 1224 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 




 
#line 1240 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 



 







 



 
#line 1271 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


 



 





 





 
#line 1297 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"

#line 1305 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"

#line 1315 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"



 








 

 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

 
 
 
 


 
 
 
 


 
 
 
 


 
 
 
 
 
 
 




 



 


 


 



 







 







 



 



 

















































 
#line 1475 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"

















































 
#line 1539 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"


























































 










































































 




































 
#line 1765 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"
























































































































































 
#line 1927 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"




















 





















 














 















 
















 
















 
#line 2047 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"










 
#line 2068 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"

















 
#line 2099 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"













 




















 
#line 2141 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"
















 
#line 2164 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"

 


























 
#line 2200 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"













































 
#line 2259 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"












































 
#line 2320 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"



 



 


 


 



 
 
 
 






























 

static __inline uint32_t LL_ADC_DMA_GetRegAddr(ADC_TypeDef *ADCx, uint32_t Register)
{
  uint32_t data_reg_addr;

  if (Register == (0x00000000UL))
  {
     
    data_reg_addr = (uint32_t) &(ADCx->DR);
  }
  else  
  {
     
    data_reg_addr = (uint32_t) &(((((ADC_Common_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08040300UL))))->CDR);
  }

  return data_reg_addr;
}
#line 2400 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"



 



 


































 
static __inline void LL_ADC_SetCommonClock(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t CommonClock)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x3UL << (16U)) | (0xFUL << (18U))))) | (CommonClock))));
}























 
static __inline uint32_t LL_ADC_GetCommonClock(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x3UL << (16U)) | (0xFUL << (18U)))));
}































 
static __inline void LL_ADC_SetCommonPathInternalCh(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t PathInternal)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x1UL << (22U)) | (0x1UL << (23U)) | (0x1UL << (24U))))) | (PathInternal))));
}






























 
static __inline void LL_ADC_SetCommonPathInternalChAdd(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t PathInternal)
{
  ((ADCxy_COMMON->CCR) |= (PathInternal));
}



















 
static __inline void LL_ADC_SetCommonPathInternalChRem(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t PathInternal)
{
  ((ADCxy_COMMON->CCR) &= ~(PathInternal));
}

















 
static __inline uint32_t LL_ADC_GetCommonPathInternalCh(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x1UL << (22U)) | (0x1UL << (23U)) | (0x1UL << (24U)))));
}



 



 































 
static __inline void LL_ADC_SetCalibrationFactor(ADC_TypeDef *ADCx, uint32_t SingleDiff, uint32_t CalibrationFactor)
{
  (((ADCx->CALFACT)) = ((((((ADCx->CALFACT))) & (~(SingleDiff & ((0x7FUL << (16U)) | (0x7FUL << (0U)))))) | (CalibrationFactor << (((SingleDiff & (0x00010000UL)) >> ((16UL) - 4UL)) & ~(SingleDiff & (0x7FUL << (0U))))))));


}
















 
static __inline uint32_t LL_ADC_GetCalibrationFactor(ADC_TypeDef *ADCx, uint32_t SingleDiff)
{
   
   
   
   
  return (uint32_t)(((ADCx->CALFACT) & ((SingleDiff & ((0x7FUL << (16U)) | (0x7FUL << (0U)))))) >> ((SingleDiff & (0x00010000UL)) >>

                                                                                  ((16UL) - 4UL)));
}

















 
static __inline void LL_ADC_SetResolution(ADC_TypeDef *ADCx, uint32_t Resolution)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x3UL << (3U))))) | (Resolution))));
}












 
static __inline uint32_t LL_ADC_GetResolution(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x3UL << (3U)))));
}















 
static __inline void LL_ADC_SetDataAlignment(ADC_TypeDef *ADCx, uint32_t DataAlignment)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (5U))))) | (DataAlignment))));
}










 
static __inline uint32_t LL_ADC_GetDataAlignment(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (5U)))));
}



















































 
static __inline void LL_ADC_SetLowPowerMode(ADC_TypeDef *ADCx, uint32_t LowPowerMode)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (14U))))) | (LowPowerMode))));
}














































 
static __inline uint32_t LL_ADC_GetLowPowerMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (14U)))));
}
















































































 
static __inline void LL_ADC_SetOffset(ADC_TypeDef *ADCx, uint32_t Offsety, uint32_t Channel, uint32_t OffsetLevel)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));

  (((*preg)) = ((((((*preg))) & (~((0x1UL << (31U)) | (0x1FUL << (26U)) | (0xFFFUL << (0U))))) | ((0x1UL << (31U)) | (Channel & ((0x1FUL << (26U)))) | OffsetLevel))));


}






































































 
static __inline uint32_t LL_ADC_GetOffsetChannel(ADC_TypeDef *ADCx, uint32_t Offsety)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));

  return (uint32_t) ((*preg) & ((0x1FUL << (26U))));
}



















 
static __inline uint32_t LL_ADC_GetOffsetLevel(ADC_TypeDef *ADCx, uint32_t Offsety)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));

  return (uint32_t) ((*preg) & ((0xFFFUL << (0U))));
}


























 
static __inline void LL_ADC_SetOffsetState(ADC_TypeDef *ADCx, uint32_t Offsety, uint32_t OffsetState)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));

  (((*preg)) = ((((((*preg))) & (~((0x1UL << (31U))))) | (OffsetState))));


}

















 
static __inline uint32_t LL_ADC_GetOffsetState(ADC_TypeDef *ADCx, uint32_t Offsety)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));

  return (uint32_t) ((*preg) & ((0x1UL << (31U))));
}















 
static __inline void LL_ADC_SetSamplingTimeCommonConfig(ADC_TypeDef *ADCx, uint32_t SamplingTimeCommonConfig)
{
  (((ADCx->SMPR1)) = ((((((ADCx->SMPR1))) & (~((0x1UL << (31U))))) | (SamplingTimeCommonConfig))));
}









 
static __inline uint32_t LL_ADC_GetSamplingTimeCommonConfig(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->SMPR1) & ((0x1UL << (31U)))));
}




 



 







































 
static __inline void LL_ADC_REG_SetTriggerSource(ADC_TypeDef *ADCx, uint32_t TriggerSource)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x3UL << (10U)) | (0xFUL << (6U))))) | (TriggerSource))));
}


































 
static __inline uint32_t LL_ADC_REG_GetTriggerSource(ADC_TypeDef *ADCx)
{
  volatile uint32_t TriggerSource = ((ADCx->CFGR) & ((0xFUL << (6U)) | (0x3UL << (10U))));

   
   
  uint32_t ShiftExten = ((TriggerSource & (0x3UL << (10U))) >> ((10UL) - 2UL));

   
   
  return ((TriggerSource
           & (((((0x00000000UL) & (0xFUL << (6U))) << (4U * 0UL)) | (((0xFUL << (6U))) << (4U * 1UL)) | (((0xFUL << (6U))) << (4U * 2UL)) | (((0xFUL << (6U))) << (4U * 3UL)) ) >> ShiftExten) & (0xFUL << (6U)))
          | ((((((0x00000000UL) & (0x3UL << (10U))) << (4U * 0UL)) | ((((0x1UL << (10U)))) << (4U * 1UL)) | ((((0x1UL << (10U)))) << (4U * 2UL)) | ((((0x1UL << (10U)))) << (4U * 3UL)) ) >> ShiftExten) & (0x3UL << (10U)))
         );
}











 
static __inline uint32_t LL_ADC_REG_IsTriggerSourceSWStart(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CFGR) & ((0x3UL << (10U)))) == ((0x00000000UL) & (0x3UL << (10U)))) ? 1UL : 0UL);
}















 
static __inline void LL_ADC_REG_SetTriggerEdge(ADC_TypeDef *ADCx, uint32_t ExternalTriggerEdge)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x3UL << (10U))))) | (ExternalTriggerEdge))));
}










 
static __inline uint32_t LL_ADC_REG_GetTriggerEdge(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x3UL << (10U)))));
}






















































 
static __inline void LL_ADC_REG_SetSequencerLength(ADC_TypeDef *ADCx, uint32_t SequencerNbRanks)
{
  (((ADCx->SQR1)) = ((((((ADCx->SQR1))) & (~((0xFUL << (0U))))) | (SequencerNbRanks))));
}

















































 
static __inline uint32_t LL_ADC_REG_GetSequencerLength(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->SQR1) & ((0xFUL << (0U)))));
}



























 
static __inline void LL_ADC_REG_SetSequencerDiscont(ADC_TypeDef *ADCx, uint32_t SeqDiscont)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (16U)) | (0x7UL << (17U))))) | (SeqDiscont))));
}


















 
static __inline uint32_t LL_ADC_REG_GetSequencerDiscont(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (16U)) | (0x7UL << (17U)))));
}






























































































 
static __inline void LL_ADC_REG_SetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank, uint32_t Channel)
{
   
   
   
   
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SQR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));

  (((*preg)) = ((((((*preg))) & (~(((0x1FUL << (0U))) << (Rank & (((0x1FUL << (0U)))))))) | (((Channel & ((0x1FUL << (26U)))) >> (26UL)) << (Rank & (((0x1FUL << (0U)))))))));


}
































































































 
static __inline uint32_t LL_ADC_REG_GetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SQR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));

  return (uint32_t)((((*preg) & (((0x1FUL << (0U))) << (Rank & (((0x1FUL << (0U)))))))

                     >> (Rank & (((0x1FUL << (0U)))))) << (26UL)
                   );
}



















 
static __inline void LL_ADC_REG_SetContinuousMode(ADC_TypeDef *ADCx, uint32_t Continuous)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (13U))))) | (Continuous))));
}












 
static __inline uint32_t LL_ADC_REG_GetContinuousMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (13U)))));
}



































 
static __inline void LL_ADC_REG_SetDMATransfer(ADC_TypeDef *ADCx, uint32_t DMATransfer)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (0U)) | (0x1UL << (1U))))) | (DMATransfer))));
}






























 
static __inline uint32_t LL_ADC_REG_GetDMATransfer(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (0U)) | (0x1UL << (1U)))));
}


















 
static __inline void LL_ADC_REG_SetDFSDMTransfer(ADC_TypeDef *ADCx, uint32_t DFSDMTransfer)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (2U))))) | (DFSDMTransfer))));
}








 
static __inline uint32_t LL_ADC_REG_GetDFSDMTransfer(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (2U)))));
}





















 
static __inline void LL_ADC_REG_SetOverrun(ADC_TypeDef *ADCx, uint32_t Overrun)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (12U))))) | (Overrun))));
}









 
static __inline uint32_t LL_ADC_REG_GetOverrun(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (12U)))));
}



 



 







































 
static __inline void LL_ADC_INJ_SetTriggerSource(ADC_TypeDef *ADCx, uint32_t TriggerSource)
{
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0xFUL << (2U)) | (0x3UL << (6U))))) | (TriggerSource))));
}


































 
static __inline uint32_t LL_ADC_INJ_GetTriggerSource(ADC_TypeDef *ADCx)
{
  volatile uint32_t TriggerSource = ((ADCx->JSQR) & ((0xFUL << (2U)) | (0x3UL << (6U))));

   
   
  uint32_t ShiftJexten = ((TriggerSource & (0x3UL << (6U))) >> (( 6UL) - 2UL));

   
   
  return ((TriggerSource
           & (((((0x00000000UL) & (0xFUL << (2U))) << (4U * 0UL)) | (((0xFUL << (2U))) << (4U * 1UL)) | (((0xFUL << (2U))) << (4U * 2UL)) | (((0xFUL << (2U))) << (4U * 3UL)) ) >> ShiftJexten) & (0xFUL << (2U)))
          | ((((((0x00000000UL) & (0x3UL << (6U))) << (4U * 0UL)) | ((((0x1UL << (6U)))) << (4U * 1UL)) | ((((0x1UL << (6U)))) << (4U * 2UL)) | ((((0x1UL << (6U)))) << (4U * 3UL)) ) >> ShiftJexten) & (0x3UL << (6U)))
         );
}











 
static __inline uint32_t LL_ADC_INJ_IsTriggerSourceSWStart(ADC_TypeDef *ADCx)
{
  return ((((ADCx->JSQR) & ((0x3UL << (6U)))) == ((0x00000000UL) & (0x3UL << (6U)))) ? 1UL : 0UL);
}















 
static __inline void LL_ADC_INJ_SetTriggerEdge(ADC_TypeDef *ADCx, uint32_t ExternalTriggerEdge)
{
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0x3UL << (6U))))) | (ExternalTriggerEdge))));
}










 
static __inline uint32_t LL_ADC_INJ_GetTriggerEdge(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->JSQR) & ((0x3UL << (6U)))));
}





















 
static __inline void LL_ADC_INJ_SetSequencerLength(ADC_TypeDef *ADCx, uint32_t SequencerNbRanks)
{
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0x3UL << (0U))))) | (SequencerNbRanks))));
}
















 
static __inline uint32_t LL_ADC_INJ_GetSequencerLength(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->JSQR) & ((0x3UL << (0U)))));
}













 
static __inline void LL_ADC_INJ_SetSequencerDiscont(ADC_TypeDef *ADCx, uint32_t SeqDiscont)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (20U))))) | (SeqDiscont))));
}










 
static __inline uint32_t LL_ADC_INJ_GetSequencerDiscont(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (20U)))));
}

































































 
static __inline void LL_ADC_INJ_SetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank, uint32_t Channel)
{
   
   
   
   
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((((0x1FUL << (26U))) >> (26UL)) << (Rank & (((0x1FUL << (0U)))))))) | (((Channel & ((0x1FUL << (26U)))) >> (26UL)) << (Rank & (((0x1FUL << (0U)))))))));


}




































































 
static __inline uint32_t LL_ADC_INJ_GetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank)
{
  return (uint32_t)((((ADCx->JSQR) & ((((0x1FUL << (26U))) >> (26UL)) << (Rank & (((0x1FUL << (0U)))))))

                     >> (Rank & (((0x1FUL << (0U)))))) << (26UL)
                   );
}






























 
static __inline void LL_ADC_INJ_SetTrigAuto(ADC_TypeDef *ADCx, uint32_t TrigAuto)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (25U))))) | (TrigAuto))));
}









 
static __inline uint32_t LL_ADC_INJ_GetTrigAuto(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (25U)))));
}









































 
static __inline void LL_ADC_INJ_SetQueueMode(ADC_TypeDef *ADCx, uint32_t QueueMode)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (21U)) | (0x1UL << (31U))))) | (QueueMode))));
}










 
static __inline uint32_t LL_ADC_INJ_GetQueueMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (21U)) | (0x1UL << (31U)))));
}



























































































































































































































 
static __inline void LL_ADC_INJ_ConfigQueueContext(ADC_TypeDef *ADCx,
                                                   uint32_t TriggerSource,
                                                   uint32_t ExternalTriggerEdge,
                                                   uint32_t SequencerNbRanks,
                                                   uint32_t Rank1_Channel,
                                                   uint32_t Rank2_Channel,
                                                   uint32_t Rank3_Channel,
                                                   uint32_t Rank4_Channel)
{
   
   
   
   
   
   
  uint32_t is_trigger_not_sw = (uint32_t)((TriggerSource != (0x00000000UL)) ? 1UL : 0UL);
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0xFUL << (2U)) | (0x3UL << (6U)) | (0x1FUL << (26U)) | (0x1FUL << (20U)) | (0x1FUL << (14U)) | (0x1FUL << (8U)) | (0x3UL << (0U))))) | ((TriggerSource & (0xFUL << (2U))) | (ExternalTriggerEdge * (is_trigger_not_sw)) | (((Rank4_Channel & ((0x1FUL << (26U)))) >> (26UL)) << (((0x00000300UL) | (26UL)) & (((0x1FUL << (0U)))))) | (((Rank3_Channel & ((0x1FUL << (26U)))) >> (26UL)) << (((0x00000200UL) | (20UL)) & (((0x1FUL << (0U)))))) | (((Rank2_Channel & ((0x1FUL << (26U)))) >> (26UL)) << (((0x00000100UL) | (14UL)) & (((0x1FUL << (0U)))))) | (((Rank1_Channel & ((0x1FUL << (26U)))) >> (26UL)) << (((0x00000000UL) | ( 8UL)) & (((0x1FUL << (0U)))))) | SequencerNbRanks))));
#line 4605 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"
}



 



 



































































































 
static __inline void LL_ADC_SetChannelSamplingTime(ADC_TypeDef *ADCx, uint32_t Channel, uint32_t SamplingTime)
{
   
   
   
   
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SMPR1)) + ((((Channel & ((0x00000000UL) | (0x02000000UL))) >> (25UL))) << 2UL))));

  (((*preg)) = ((((((*preg))) & (~((0x7UL << (0U)) << ((Channel & (0x01F00000UL)) >> (20UL))))) | (SamplingTime << ((Channel & (0x01F00000UL)) >> (20UL))))));


}



















































































 
static __inline uint32_t LL_ADC_GetChannelSamplingTime(ADC_TypeDef *ADCx, uint32_t Channel)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SMPR1)) + ((((Channel & ((0x00000000UL) | (0x02000000UL))) >> (25UL))) << 2UL))));

  return (uint32_t)(((*preg) & ((0x7UL << (0U)) << ((Channel & (0x01F00000UL)) >> (20UL))))

                    >> ((Channel & (0x01F00000UL)) >> (20UL))
                   );
}
















































 
static __inline void LL_ADC_SetChannelSingleDiff(ADC_TypeDef *ADCx, uint32_t Channel, uint32_t SingleDiff)
{
   
   
   
  (((ADCx->DIFSEL)) = ((((((ADCx->DIFSEL))) & (~(Channel & (((0x7FFFFUL << (0U))))))) | ((Channel & (((0x7FFFFUL << (0U))))) & ((0x7FFFFUL << (0U)) >> (SingleDiff & ((0x10UL << (0U)) | (0x08UL << (0U)))))))));


}








































 
static __inline uint32_t LL_ADC_GetChannelSingleDiff(ADC_TypeDef *ADCx, uint32_t Channel)
{
  return (uint32_t)(((ADCx->DIFSEL) & ((Channel & (((0x7FFFFUL << (0U))))))));
}



 



 
















































































































































 
static __inline void LL_ADC_SetAnalogWDMonitChannels(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t AWDChannelGroup)
{
   
   
   
   
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->CFGR)) + ((((AWDy & ((0x00000000UL) | (0x00100000UL) | (0x00200000UL))) >> (20UL)) + ((AWDy & ((0x00001UL << (0U)))) * (0x00000024UL))) << 2UL))));


  (((*preg)) = ((((((*preg))) & (~((AWDy & (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | ((0x7FFFFUL << (0U)))))))) | (AWDChannelGroup & AWDy))));


}


























































































































 
static __inline uint32_t LL_ADC_GetAnalogWDMonitChannels(ADC_TypeDef *ADCx, uint32_t AWDy)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->CFGR)) + ((((AWDy & ((0x00000000UL) | (0x00100000UL) | (0x00200000UL))) >> (20UL)) + ((AWDy & ((0x00001UL << (0U)))) * (0x00000024UL))) << 2UL))));


  uint32_t AnalogWDMonitChannels = (((*preg) & (AWDy)) & AWDy & (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | ((0x7FFFFUL << (0U)))));

   
   
   
   
  if (AnalogWDMonitChannels != 0UL)
  {
    if (AWDy == (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | (0x00000000UL)))
    {
      if ((AnalogWDMonitChannels & (0x1UL << (22U))) == 0UL)
      {
         
        AnalogWDMonitChannels = ((AnalogWDMonitChannels
                                  | (((0x7FFFFUL << (0U))))
                                 )
                                 & (~((0x1FUL << (26U))))
                                );
      }
      else
      {
         
        AnalogWDMonitChannels = (AnalogWDMonitChannels
                                 | ((0x00001UL << (0U)) << (AnalogWDMonitChannels >> (26U)))
                                );
      }
    }
    else
    {
      if ((AnalogWDMonitChannels & ((0x7FFFFUL << (0U)))) == ((0x7FFFFUL << (0U))))
      {
         
        AnalogWDMonitChannels = (((0x7FFFFUL << (0U)))
                                 | (((0x1UL << (24U)) | (0x1UL << (23U))))
                                );
      }
      else
      {
         
         
        AnalogWDMonitChannels = (AnalogWDMonitChannels
                                 | ((0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U)))
                                 | (((((AnalogWDMonitChannels) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( ((AnalogWDMonitChannels) & ((0x1FUL << (26U)))) >> (26UL) ) : ( (uint32_t)(__clz(__rbit((AnalogWDMonitChannels)))) ) ) << (26U))
                                );
      }
    }
  }

  return AnalogWDMonitChannels;
}




















































 
static __inline void LL_ADC_ConfigAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t AWDThresholdHighValue,
                                                     uint32_t AWDThresholdLowValue)
{
   
   
   
   
   
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->TR1)) + ((((AWDy & (((0x00000000UL)) | ((0x00100000UL)) | ((0x00200000UL)))) >> ((20UL)))) << 2UL))));

  (((*preg)) = ((((((*preg))) & (~((0xFFFUL << (16U)) | (0xFFFUL << (0U))))) | ((AWDThresholdHighValue << (16UL)) | AWDThresholdLowValue))));


}






















































 
static __inline void LL_ADC_SetAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t AWDThresholdsHighLow,
                                                  uint32_t AWDThresholdValue)
{
   
   
   
   
   
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->TR1)) + ((((AWDy & (((0x00000000UL)) | ((0x00100000UL)) | ((0x00200000UL)))) >> ((20UL)))) << 2UL))));


  (((*preg)) = ((((((*preg))) & (~(AWDThresholdsHighLow))) | (AWDThresholdValue << ((AWDThresholdsHighLow & (0x00010000UL)) >> ((16UL) - 4UL))))));


}




























 
static __inline uint32_t LL_ADC_GetAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t AWDThresholdsHighLow)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->TR1)) + ((((AWDy & (((0x00000000UL)) | ((0x00100000UL)) | ((0x00200000UL)))) >> ((20UL)))) << 2UL))));


  return (uint32_t)(((*preg) & ((AWDThresholdsHighLow | (0xFFFUL << (0U)))))

                    >> (((AWDThresholdsHighLow & (0x00010000UL)) >> ((16UL) - 4UL))
                        & ~(AWDThresholdsHighLow & (0xFFFUL << (0U)))));
}



 



 

























 
static __inline void LL_ADC_SetOverSamplingScope(ADC_TypeDef *ADCx, uint32_t OvsScope)
{
  (((ADCx->CFGR2)) = ((((((ADCx->CFGR2))) & (~((0x1UL << (0U)) | (0x1UL << (1U)) | (0x1UL << (10U))))) | (OvsScope))));
}




















 
static __inline uint32_t LL_ADC_GetOverSamplingScope(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR2) & ((0x1UL << (0U)) | (0x1UL << (1U)) | (0x1UL << (10U)))));
}






















 
static __inline void LL_ADC_SetOverSamplingDiscont(ADC_TypeDef *ADCx, uint32_t OverSamplingDiscont)
{
  (((ADCx->CFGR2)) = ((((((ADCx->CFGR2))) & (~((0x1UL << (9U))))) | (OverSamplingDiscont))));
}














 
static __inline uint32_t LL_ADC_GetOverSamplingDiscont(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR2) & ((0x1UL << (9U)))));
}


































 
static __inline void LL_ADC_ConfigOverSamplingRatioShift(ADC_TypeDef *ADCx, uint32_t Ratio, uint32_t Shift)
{
  (((ADCx->CFGR2)) = ((((((ADCx->CFGR2))) & (~(((0xFUL << (5U)) | (0x7UL << (2U)))))) | ((Shift | Ratio)))));
}















 
static __inline uint32_t LL_ADC_GetOverSamplingRatio(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR2) & ((0x7UL << (2U)))));
}
















 
static __inline uint32_t LL_ADC_GetOverSamplingShift(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR2) & ((0xFUL << (5U)))));
}



 



 



























 
static __inline void LL_ADC_SetMultimode(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t Multimode)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x1FUL << (0U))))) | (Multimode))));
}



















 
static __inline uint32_t LL_ADC_GetMultimode(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x1FUL << (0U)))));
}














































 
static __inline void LL_ADC_SetMultiDMATransfer(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t MultiDMATransfer)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x3UL << (14U)) | (0x1UL << (13U))))) | (MultiDMATransfer))));
}









































 
static __inline uint32_t LL_ADC_GetMultiDMATransfer(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x3UL << (14U)) | (0x1UL << (13U)))));
}



































 
static __inline void LL_ADC_SetMultiTwoSamplingDelay(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t MultiTwoSamplingDelay)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0xFUL << (8U))))) | (MultiTwoSamplingDelay))));
}























 
static __inline uint32_t LL_ADC_GetMultiTwoSamplingDelay(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0xFUL << (8U)))));
}




 


 
 
 
static __inline void LL_ADC_REG_SetTrigSource(ADC_TypeDef *ADCx, uint32_t TriggerSource)
{
  LL_ADC_REG_SetTriggerSource(ADCx, TriggerSource);
}
static __inline void LL_ADC_INJ_SetTrigSource(ADC_TypeDef *ADCx, uint32_t TriggerSource)
{
  LL_ADC_INJ_SetTriggerSource(ADCx, TriggerSource);
}



 



 













 
static __inline void LL_ADC_EnableDeepPowerDown(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (29U))))));


}













 
static __inline void LL_ADC_DisableDeepPowerDown(ADC_TypeDef *ADCx)
{
   
   
   
  ((ADCx->CR) &= ~(((0x1UL << (29U)) | ((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U))))));
}






 
static __inline uint32_t LL_ADC_IsDeepPowerDownEnabled(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (29U)))) == ((0x1UL << (29U)))) ? 1UL : 0UL);
}














 
static __inline void LL_ADC_EnableInternalRegulator(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (28U))))));


}









 
static __inline void LL_ADC_DisableInternalRegulator(ADC_TypeDef *ADCx)
{
  ((ADCx->CR) &= ~(((0x1UL << (28U)) | ((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U))))));
}






 
static __inline uint32_t LL_ADC_IsInternalRegulatorEnabled(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (28U)))) == ((0x1UL << (28U)))) ? 1UL : 0UL);
}
















 
static __inline void LL_ADC_Enable(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (0U))))));


}










 
static __inline void LL_ADC_Disable(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (1U))))));


}









 
static __inline uint32_t LL_ADC_IsEnabled(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsDisableOngoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (1U)))) == ((0x1UL << (1U)))) ? 1UL : 0UL);
}























 
static __inline void LL_ADC_StartCalibration(ADC_TypeDef *ADCx, uint32_t SingleDiff)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~((0x1UL << (30U)) | ((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (31U)) | (SingleDiff & ((0x1UL << (30U))))))));


}






 
static __inline uint32_t LL_ADC_IsCalibrationOnGoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (31U)))) == ((0x1UL << (31U)))) ? 1UL : 0UL);
}



 



 


















 
static __inline void LL_ADC_REG_StartConversion(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (2U))))));


}










 
static __inline void LL_ADC_REG_StopConversion(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (4U))))));


}






 
static __inline uint32_t LL_ADC_REG_IsConversionOngoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (2U)))) == ((0x1UL << (2U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_REG_IsStopConversionOngoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (4U)))) == ((0x1UL << (4U)))) ? 1UL : 0UL);
}









 
static __inline uint32_t LL_ADC_REG_ReadConversionData32(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}










 
static __inline uint16_t LL_ADC_REG_ReadConversionData12(ADC_TypeDef *ADCx)
{
  return (uint16_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}










 
static __inline uint16_t LL_ADC_REG_ReadConversionData10(ADC_TypeDef *ADCx)
{
  return (uint16_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}










 
static __inline uint8_t LL_ADC_REG_ReadConversionData8(ADC_TypeDef *ADCx)
{
  return (uint8_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}










 
static __inline uint8_t LL_ADC_REG_ReadConversionData6(ADC_TypeDef *ADCx)
{
  return (uint8_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}






















 
static __inline uint32_t LL_ADC_REG_ReadMultiConversionData32(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t ConversionData)
{
  return (uint32_t)(((ADCxy_COMMON->CDR) & (ConversionData))

                    >> ((__clz(__rbit(ConversionData))) & 0x1FUL)
                   );
}




 



 


















 
static __inline void LL_ADC_INJ_StartConversion(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (3U))))));


}










 
static __inline void LL_ADC_INJ_StopConversion(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (5U))))));


}






 
static __inline uint32_t LL_ADC_INJ_IsConversionOngoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (3U)))) == ((0x1UL << (3U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_INJ_IsStopConversionOngoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (5U)))) == ((0x1UL << (5U)))) ? 1UL : 0UL);
}

















 
static __inline uint32_t LL_ADC_INJ_ReadConversionData32(ADC_TypeDef *ADCx, uint32_t Rank)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));

  return (uint32_t)(((*preg) & ((0xFFFFUL << (0U))))

                   );
}


















 
static __inline uint16_t LL_ADC_INJ_ReadConversionData12(ADC_TypeDef *ADCx, uint32_t Rank)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));

  return (uint16_t)(((*preg) & ((0xFFFFUL << (0U))))

                   );
}


















 
static __inline uint16_t LL_ADC_INJ_ReadConversionData10(ADC_TypeDef *ADCx, uint32_t Rank)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));

  return (uint16_t)(((*preg) & ((0xFFFFUL << (0U))))

                   );
}


















 
static __inline uint8_t LL_ADC_INJ_ReadConversionData8(ADC_TypeDef *ADCx, uint32_t Rank)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));

  return (uint8_t)(((*preg) & ((0xFFFFUL << (0U))))

                  );
}


















 
static __inline uint8_t LL_ADC_INJ_ReadConversionData6(ADC_TypeDef *ADCx, uint32_t Rank)
{
  const volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));

  return (uint8_t)(((*preg) & ((0xFFFFUL << (0U))))

                  );
}



 



 









 
static __inline uint32_t LL_ADC_IsActiveFlag_ADRDY(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_EOC(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (2U)))) == ((0x1UL << (2U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_EOS(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (3U)))) == ((0x1UL << (3U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_OVR(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (4U)))) == ((0x1UL << (4U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_EOSMP(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (1U)))) == ((0x1UL << (1U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_JEOC(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (5U)))) == ((0x1UL << (5U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_JEOS(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (6U)))) == ((0x1UL << (6U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_JQOVF(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (10U)))) == ((0x1UL << (10U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_AWD1(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (7U)))) == ((0x1UL << (7U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_AWD2(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (8U)))) == ((0x1UL << (8U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_AWD3(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (9U)))) == ((0x1UL << (9U)))) ? 1UL : 0UL);
}









 
static __inline void LL_ADC_ClearFlag_ADRDY(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (0U))));
}






 
static __inline void LL_ADC_ClearFlag_EOC(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (2U))));
}






 
static __inline void LL_ADC_ClearFlag_EOS(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (3U))));
}






 
static __inline void LL_ADC_ClearFlag_OVR(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (4U))));
}






 
static __inline void LL_ADC_ClearFlag_EOSMP(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (1U))));
}






 
static __inline void LL_ADC_ClearFlag_JEOC(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (5U))));
}






 
static __inline void LL_ADC_ClearFlag_JEOS(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (6U))));
}






 
static __inline void LL_ADC_ClearFlag_JQOVF(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (10U))));
}






 
static __inline void LL_ADC_ClearFlag_AWD1(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (7U))));
}






 
static __inline void LL_ADC_ClearFlag_AWD2(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (8U))));
}






 
static __inline void LL_ADC_ClearFlag_AWD3(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (9U))));
}








 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_ADRDY(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_ADRDY(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (16U)))) == ((0x1UL << (16U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_EOC(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (18U)))) == ((0x1UL << (18U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_EOC(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (18U)))) == ((0x1UL << (18U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_EOS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (3U)))) == ((0x1UL << (3U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_EOS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (19U)))) == ((0x1UL << (19U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_OVR(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (4U)))) == ((0x1UL << (4U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_OVR(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (20U)))) == ((0x1UL << (20U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_EOSMP(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (1U)))) == ((0x1UL << (1U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_EOSMP(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (17U)))) == ((0x1UL << (17U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_JEOC(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (5U)))) == ((0x1UL << (5U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_JEOC(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (21U)))) == ((0x1UL << (21U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_JEOS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (6U)))) == ((0x1UL << (6U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_JEOS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (22U)))) == ((0x1UL << (22U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_JQOVF(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (10U)))) == ((0x1UL << (10U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_JQOVF(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (26U)))) == ((0x1UL << (26U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_AWD1(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (7U)))) == ((0x1UL << (7U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_AWD1(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (23U)))) == ((0x1UL << (23U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_AWD2(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (8U)))) == ((0x1UL << (8U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_AWD2(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (24U)))) == ((0x1UL << (24U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_AWD3(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (9U)))) == ((0x1UL << (9U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_AWD3(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (25U)))) == ((0x1UL << (25U)))) ? 1UL : 0UL);
}




 



 






 
static __inline void LL_ADC_EnableIT_ADRDY(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (0U))));
}






 
static __inline void LL_ADC_EnableIT_EOC(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (2U))));
}






 
static __inline void LL_ADC_EnableIT_EOS(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (3U))));
}






 
static __inline void LL_ADC_EnableIT_OVR(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (4U))));
}






 
static __inline void LL_ADC_EnableIT_EOSMP(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (1U))));
}






 
static __inline void LL_ADC_EnableIT_JEOC(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (5U))));
}






 
static __inline void LL_ADC_EnableIT_JEOS(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (6U))));
}






 
static __inline void LL_ADC_EnableIT_JQOVF(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (10U))));
}






 
static __inline void LL_ADC_EnableIT_AWD1(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (7U))));
}






 
static __inline void LL_ADC_EnableIT_AWD2(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (8U))));
}






 
static __inline void LL_ADC_EnableIT_AWD3(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (9U))));
}






 
static __inline void LL_ADC_DisableIT_ADRDY(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (0U))));
}






 
static __inline void LL_ADC_DisableIT_EOC(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (2U))));
}






 
static __inline void LL_ADC_DisableIT_EOS(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (3U))));
}






 
static __inline void LL_ADC_DisableIT_OVR(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (4U))));
}






 
static __inline void LL_ADC_DisableIT_EOSMP(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (1U))));
}






 
static __inline void LL_ADC_DisableIT_JEOC(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (5U))));
}






 
static __inline void LL_ADC_DisableIT_JEOS(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (6U))));
}






 
static __inline void LL_ADC_DisableIT_JQOVF(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (10U))));
}






 
static __inline void LL_ADC_DisableIT_AWD1(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (7U))));
}






 
static __inline void LL_ADC_DisableIT_AWD2(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (8U))));
}






 
static __inline void LL_ADC_DisableIT_AWD3(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (9U))));
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_ADRDY(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_EOC(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (2U)))) == ((0x1UL << (2U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_EOS(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (3U)))) == ((0x1UL << (3U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_OVR(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (4U)))) == ((0x1UL << (4U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_EOSMP(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (1U)))) == ((0x1UL << (1U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_JEOC(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (5U)))) == ((0x1UL << (5U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_JEOS(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (6U)))) == ((0x1UL << (6U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_JQOVF(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (10U)))) == ((0x1UL << (10U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_AWD1(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (7U)))) == ((0x1UL << (7U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_AWD2(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (8U)))) == ((0x1UL << (8U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_AWD3(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (9U)))) == ((0x1UL << (9U)))) ? 1UL : 0UL);
}



 

#line 7439 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h"



 



 





 







 
#line 33 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"



 



 

 


 



 
typedef struct
{
  uint32_t Ratio;                         
 

  uint32_t RightBitShift;                 
 

  uint32_t TriggeredMode;                 
 

  uint32_t OversamplingStopReset;         





 

} ADC_OversamplingTypeDef;
















 
typedef struct
{
  uint32_t ClockPrescaler;        








 

  uint32_t Resolution;            
 

  uint32_t DataAlign;             

 

  uint32_t ScanConvMode;          





 

  uint32_t EOCSelection;          
 

  FunctionalState LowPowerAutoWait; 









 

  FunctionalState ContinuousConvMode; 

 

  uint32_t NbrOfConversion;       



 

  FunctionalState DiscontinuousConvMode; 



 

  uint32_t NbrOfDiscConversion;   

 

  uint32_t ExternalTrigConv;      


 

  uint32_t ExternalTrigConvEdge;  

 

  FunctionalState DMAContinuousRequests; 


 

  uint32_t Overrun;               








 

  FunctionalState OversamplingMode;       

 

  ADC_OversamplingTypeDef Oversampling;   
 


  uint32_t DFSDMConfig;           

 


} ADC_InitTypeDef;











 
typedef struct
{
  uint32_t Channel;                

 

  uint32_t Rank;                   


 

  uint32_t SamplingTime;           








 

  uint32_t SingleDiff;             









 

  uint32_t OffsetNumber;           

 

  uint32_t Offset;                 




 

} ADC_ChannelConfTypeDef;






 
typedef struct
{
  uint32_t WatchdogNumber;    


 

  uint32_t WatchdogMode;      


 

  uint32_t Channel;           


 

  FunctionalState ITMode;     
 

  uint32_t HighThreshold;     







 

  uint32_t LowThreshold;      







 
} ADC_AnalogWDGConfTypeDef;




 
typedef struct
{
  uint32_t ContextQueue;                 

 

  uint32_t ChannelCount;                  
} ADC_InjectionConfigTypeDef;



 








 
 





 




 






 





 




 




 



 



typedef struct

{
  ADC_TypeDef                   *Instance;               
  ADC_InitTypeDef               Init;                    
  DMA_HandleTypeDef             *DMA_Handle;             
  HAL_LockTypeDef               Lock;                    
  volatile uint32_t                 State;                   
  volatile uint32_t                 ErrorCode;               
  ADC_InjectionConfigTypeDef    InjectionConfig ;        
#line 385 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"
} ADC_HandleTypeDef;

#line 412 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"



 


 



 



 
#line 436 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"


 



 




#line 459 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"


 



 






 



 




 



 




 



 
 
#line 513 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"


 



 






 



 




 



 




 



 
#line 565 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"


 



 
#line 583 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"


 



 
 
 
#line 625 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"


 



 





 



 
#line 649 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"


 



 
#line 664 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"


 



 
#line 680 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"


 



 




 



 




 



 
#line 711 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"


 




 
#line 730 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"





 



 
#line 751 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"







 






 



 

 



 
 
 





 







 






 









 









 







 







 







 







 
#line 866 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"





 









 







 







 







 










 
#line 934 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"





 







 







 
#line 976 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"





 
#line 998 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"



 


 



 

 
 
 
 
 
 
 


 
 
 




 

 



 
 
 



 




 
#line 1055 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"


















 




















 



















 




















 




















 
 





 



 

















































 



















































 




























































 










































































 




































 
















 













 



















 















 




















 
#line 1537 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"
















 
#line 1560 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"

























 

















































 
#line 1642 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"












































 
#line 1699 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"



 



 

 
#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"



 



 

 


 



 
typedef struct
{
  uint32_t Ratio;                         
 

  uint32_t RightBitShift;                 
 
} ADC_InjOversamplingTypeDef;
















 
typedef struct
{
  uint32_t InjectedChannel;               

 

  uint32_t InjectedRank;                  


 

  uint32_t InjectedSamplingTime;          








 

  uint32_t InjectedSingleDiff;            









 

  uint32_t InjectedOffsetNumber;          

 

  uint32_t InjectedOffset;                




 

  uint32_t InjectedNbrOfConversion;       



 

  FunctionalState InjectedDiscontinuousConvMode; 







 

  FunctionalState AutoInjectedConv;       






 

  FunctionalState QueueInjectedContext;   








 

  uint32_t ExternalTrigInjecConv;         



 

  uint32_t ExternalTrigInjecConvEdge;     



 

  FunctionalState InjecOversamplingMode;         

 

  ADC_InjOversamplingTypeDef  InjecOversampling; 

 
} ADC_InjectionConfTypeDef;






 
typedef struct
{
  uint32_t Mode;              
 

  uint32_t DMAAccessMode;     

 

  uint32_t TwoSamplingDelay;  



 
} ADC_MultiModeTypeDef;




 

 



 



 
 
#line 228 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"


 



 






 



 




 



 







 



 






 




 
#line 287 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"



 





 



 
#line 313 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"


 



 




 





 



 
#line 350 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"


 



 
#line 368 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"


 



 

 







 




 




 




 

 




 














 





 


 



 
 
 






 







 








 








 
#line 483 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"






 







 






 






 






 






 






 






 






 






 






 







 






 







 














 














 













 










 
#line 659 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"








 









 




 








 




 








 
 






 






 








 
#line 897 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"






 
#line 921 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"



 
#line 954 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"





 







 










 










 
#line 1007 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"





 










 
#line 1032 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"





 








 
#line 1060 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"





 








 
#line 1082 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"





 








 
#line 1103 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"





 
#line 1117 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"





 
#line 1132 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"





 







 










 
#line 1163 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc_ex.h"








 








 


 


 



 
 

 
HAL_StatusTypeDef       HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *hadc, uint32_t SingleDiff);
uint32_t                HAL_ADCEx_Calibration_GetValue(ADC_HandleTypeDef *hadc, uint32_t SingleDiff);
HAL_StatusTypeDef       HAL_ADCEx_Calibration_SetValue(ADC_HandleTypeDef *hadc, uint32_t SingleDiff,
                                                       uint32_t CalibrationFactor);

 
HAL_StatusTypeDef       HAL_ADCEx_InjectedStart(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADCEx_InjectedStop(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADCEx_InjectedPollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout);

 
HAL_StatusTypeDef       HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef *hadc);


 
HAL_StatusTypeDef       HAL_ADCEx_MultiModeStart_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);
HAL_StatusTypeDef       HAL_ADCEx_MultiModeStop_DMA(ADC_HandleTypeDef *hadc);
uint32_t                HAL_ADCEx_MultiModeGetValue(ADC_HandleTypeDef *hadc);


 
uint32_t                HAL_ADCEx_InjectedGetValue(ADC_HandleTypeDef *hadc, uint32_t InjectedRank);

 
void                    HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);
void                    HAL_ADCEx_InjectedQueueOverflowCallback(ADC_HandleTypeDef *hadc);
void                    HAL_ADCEx_LevelOutOfWindow2Callback(ADC_HandleTypeDef *hadc);
void                    HAL_ADCEx_LevelOutOfWindow3Callback(ADC_HandleTypeDef *hadc);
void                    HAL_ADCEx_EndOfSamplingCallback(ADC_HandleTypeDef *hadc);

 
HAL_StatusTypeDef HAL_ADCEx_RegularStop(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADCEx_RegularStop_IT(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADCEx_RegularStop_DMA(ADC_HandleTypeDef *hadc);

HAL_StatusTypeDef HAL_ADCEx_RegularMultiModeStop_DMA(ADC_HandleTypeDef *hadc);




 



 
 
HAL_StatusTypeDef       HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef *hadc,
                                                        ADC_InjectionConfTypeDef *sConfigInjected);

HAL_StatusTypeDef       HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef *hadc, ADC_MultiModeTypeDef *multimode);

HAL_StatusTypeDef       HAL_ADCEx_EnableInjectedQueue(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADCEx_DisableInjectedQueue(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADCEx_DisableVoltageRegulator(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADCEx_EnterADCDeepPowerDownMode(ADC_HandleTypeDef *hadc);



 



 



 



 








 
#line 1710 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"

 


 




 
 
HAL_StatusTypeDef       HAL_ADC_Init(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADC_DeInit(ADC_HandleTypeDef *hadc);
void                    HAL_ADC_MspInit(ADC_HandleTypeDef *hadc);
void                    HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc);

#line 1732 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_adc.h"


 




 
 

 
HAL_StatusTypeDef       HAL_ADC_Start(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADC_Stop(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout);
HAL_StatusTypeDef       HAL_ADC_PollForEvent(ADC_HandleTypeDef *hadc, uint32_t EventType, uint32_t Timeout);

 
HAL_StatusTypeDef       HAL_ADC_Start_IT(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef       HAL_ADC_Stop_IT(ADC_HandleTypeDef *hadc);

 
HAL_StatusTypeDef       HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);
HAL_StatusTypeDef       HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc);

 
uint32_t                HAL_ADC_GetValue(ADC_HandleTypeDef *hadc);

 
void                    HAL_ADC_IRQHandler(ADC_HandleTypeDef *hadc);
void                    HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void                    HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc);
void                    HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc);
void                    HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);


 




 
 
HAL_StatusTypeDef       HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *sConfig);
HAL_StatusTypeDef       HAL_ADC_AnalogWDGConfig(ADC_HandleTypeDef *hadc, ADC_AnalogWDGConfTypeDef *AnalogWDGConfig);



 

 


 
uint32_t                HAL_ADC_GetState(ADC_HandleTypeDef *hadc);
uint32_t                HAL_ADC_GetError(ADC_HandleTypeDef *hadc);



 



 

 


 
HAL_StatusTypeDef ADC_ConversionStop(ADC_HandleTypeDef *hadc, uint32_t ConversionGroup);
HAL_StatusTypeDef ADC_Enable(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef ADC_Disable(ADC_HandleTypeDef *hadc);
void ADC_DMAConvCplt(DMA_HandleTypeDef *hdma);
void ADC_DMAHalfConvCplt(DMA_HandleTypeDef *hdma);
void ADC_DMAError(DMA_HandleTypeDef *hdma);



 



 



 








 
#line 227 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"


#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_can.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_can.h"



 




 

 


 


 
typedef enum
{
  HAL_CAN_STATE_RESET             = 0x00U,   
  HAL_CAN_STATE_READY             = 0x01U,   
  HAL_CAN_STATE_LISTENING         = 0x02U,   
  HAL_CAN_STATE_SLEEP_PENDING     = 0x03U,   
  HAL_CAN_STATE_SLEEP_ACTIVE      = 0x04U,   
  HAL_CAN_STATE_ERROR             = 0x05U    

} HAL_CAN_StateTypeDef;



 
typedef struct
{
  uint32_t Prescaler;                  
 

  uint32_t Mode;                       
 

  uint32_t SyncJumpWidth;              

 

  uint32_t TimeSeg1;                   
 

  uint32_t TimeSeg2;                   
 

  FunctionalState TimeTriggeredMode;   
 

  FunctionalState AutoBusOff;          
 

  FunctionalState AutoWakeUp;          
 

  FunctionalState AutoRetransmission;  
 

  FunctionalState ReceiveFifoLocked;   
 

  FunctionalState TransmitFifoPriority;
 

} CAN_InitTypeDef;



 
typedef struct
{
  uint32_t FilterIdHigh;          

 

  uint32_t FilterIdLow;           

 

  uint32_t FilterMaskIdHigh;      


 

  uint32_t FilterMaskIdLow;       


 

  uint32_t FilterFIFOAssignment;  
 

  uint32_t FilterBank;            



 

  uint32_t FilterMode;            
 

  uint32_t FilterScale;           
 

  uint32_t FilterActivation;      
 

  uint32_t SlaveStartFilterBank;  




 

} CAN_FilterTypeDef;



 
typedef struct
{
  uint32_t StdId;    
 

  uint32_t ExtId;    
 

  uint32_t IDE;      
 

  uint32_t RTR;      
 

  uint32_t DLC;      
 

  FunctionalState TransmitGlobalTime; 



 

} CAN_TxHeaderTypeDef;



 
typedef struct
{
  uint32_t StdId;    
 

  uint32_t ExtId;    
 

  uint32_t IDE;      
 

  uint32_t RTR;      
 

  uint32_t DLC;      
 

  uint32_t Timestamp; 

 

  uint32_t FilterMatchIndex; 
 

} CAN_RxHeaderTypeDef;



 
typedef struct __CAN_HandleTypeDef
{
  CAN_TypeDef                 *Instance;                  

  CAN_InitTypeDef             Init;                       

  volatile HAL_CAN_StateTypeDef   State;                      

  volatile uint32_t               ErrorCode;                 
 

#line 239 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_can.h"
} CAN_HandleTypeDef;

#line 272 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_can.h"


 

 



 



 
#line 308 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_can.h"








 



 




 



 






 




 






 



 
#line 369 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_can.h"


 



 
#line 384 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_can.h"


 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 





 



 
 
#line 483 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_can.h"

 





 






 





 




 
 


 
#line 519 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_can.h"

 



 







 



 

 


 




 
#line 556 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_can.h"







 








 







 







 
#line 595 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_can.h"
























 








 

 


 




 

 
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef *hcan);
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan);
void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan);

#line 652 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_can.h"


 




 

 
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig);



 




 

 
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_RequestSleep(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_WakeUp(CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_IsSleepActive(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan, uint32_t TxMailboxes);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *hcan, uint32_t TxMailboxes);
uint32_t HAL_CAN_GetTxTimestamp(CAN_HandleTypeDef *hcan, uint32_t TxMailbox);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *hcan, uint32_t RxFifo);



 




 
 
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs);
HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *hcan, uint32_t InactiveITs);
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan);



 




 
 

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);



 




 
 
HAL_CAN_StateTypeDef HAL_CAN_GetState(CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *hcan);



 



 

 


 



 

 


 



 

 


 



 

 


 

#line 827 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_can.h"



 
 



 





 








 
#line 231 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"






























#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"



 



 

 


 



 
typedef struct
{
  uint32_t TypeErase;   
 
  uint32_t Banks;       

 
  uint32_t Page;        

 
  uint32_t NbPages;     
 
} FLASH_EraseInitTypeDef;



 
typedef struct
{
  uint32_t OptionType;     
 
  uint32_t WRPArea;        

 
  uint32_t WRPStartOffset; 

 
  uint32_t WRPEndOffset;   
 
  uint32_t RDPLevel;       
 
  uint32_t USERType;       
 
  uint32_t USERConfig;     






 
  uint32_t PCROPConfig;    

 
  uint32_t PCROPStartAddr; 

 
  uint32_t PCROPEndAddr;   
 
} FLASH_OBProgramInitTypeDef;



 
typedef enum
{
  FLASH_PROC_NONE = 0,
  FLASH_PROC_PAGE_ERASE,
  FLASH_PROC_MASS_ERASE,
  FLASH_PROC_PROGRAM,
  FLASH_PROC_PROGRAM_LAST
} FLASH_ProcedureTypeDef;



 
typedef enum
{
  FLASH_CACHE_DISABLED = 0,
  FLASH_CACHE_ICACHE_ENABLED,
  FLASH_CACHE_DCACHE_ENABLED,
  FLASH_CACHE_ICACHE_DCACHE_ENABLED
} FLASH_CacheTypeDef;



 
typedef struct
{
  HAL_LockTypeDef             Lock;               
  volatile uint32_t               ErrorCode;          
  volatile FLASH_ProcedureTypeDef ProcedureOnGoing;   
  volatile uint32_t               Address;            
  volatile uint32_t               Bank;               
  volatile uint32_t               Page;               
  volatile uint32_t               NbPagesToErase;     
  volatile FLASH_CacheTypeDef     CacheToReactivate;  
}FLASH_ProcessTypeDef;



 

 


 



 
#line 167 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"


 



 




 



 
#line 192 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"


 




 







 



 






 



 
#line 231 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"


 



 






 



 
#line 280 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"


 



 







 



 




 



 




 



 




 



 




 



 




 



 




 



 




 






 




 
#line 380 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"


 




 



#line 398 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"


 


 




 



 




 



 




 







 




 



 




 




 






 



 
#line 483 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"


 



 













 



 
#line 544 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"







 




 






 

 



 











 











 





 





 





 





 





 





 






 








 








 









 








 





 




 




 










 













 
























 























 





 

 
#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash_ex.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash_ex.h"



 



 

 

 
#line 59 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash_ex.h"

 

 


 

 


 
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *PageError);
HAL_StatusTypeDef HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit);
HAL_StatusTypeDef HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);
void              HAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit);


 

#line 88 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash_ex.h"



 

 


 
void FLASH_PageErase(uint32_t Page, uint32_t Banks);
void FLASH_FlushCaches(void);


 

 


 





 



 



 







 
#line 773 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"
#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash_ramfunc.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash_ramfunc.h"



 



 

 
 
 


 



 
 
  HAL_StatusTypeDef HAL_FLASHEx_EnableRunPowerDown(void);
  HAL_StatusTypeDef HAL_FLASHEx_DisableRunPowerDown(void);





 



 



 



 







 
#line 774 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"

 


 

 


 
HAL_StatusTypeDef  HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
HAL_StatusTypeDef  HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
 
void               HAL_FLASH_IRQHandler(void);
 
void               HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void               HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);


 

 


 
HAL_StatusTypeDef  HAL_FLASH_Unlock(void);
HAL_StatusTypeDef  HAL_FLASH_Lock(void);
 
HAL_StatusTypeDef  HAL_FLASH_OB_Unlock(void);
HAL_StatusTypeDef  HAL_FLASH_OB_Lock(void);
HAL_StatusTypeDef  HAL_FLASH_OB_Launch(void);


 

 


 
uint32_t HAL_FLASH_GetError(void);


 



 

 


 
extern FLASH_ProcessTypeDef pFlash;


 

 


 
HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);


 

 


 
#line 851 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"

#line 858 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"




 

 


 




#line 878 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"

#line 886 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"





#line 900 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"





#line 921 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"



#line 932 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"





#line 944 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"
















































#line 1008 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"


 



 



 



 







 
#line 263 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"














#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c.h"



 



 

 


 




 
typedef struct
{
  uint32_t Timing;              

 

  uint32_t OwnAddress1;         
 

  uint32_t AddressingMode;      
 

  uint32_t DualAddressMode;     
 

  uint32_t OwnAddress2;         
 

  uint32_t OwnAddress2Masks;    
 

  uint32_t GeneralCallMode;     
 

  uint32_t NoStretchMode;       
 

} I2C_InitTypeDef;



 



























 
typedef enum
{
  HAL_I2C_STATE_RESET             = 0x00U,    
  HAL_I2C_STATE_READY             = 0x20U,    
  HAL_I2C_STATE_BUSY              = 0x24U,    
  HAL_I2C_STATE_BUSY_TX           = 0x21U,    
  HAL_I2C_STATE_BUSY_RX           = 0x22U,    
  HAL_I2C_STATE_LISTEN            = 0x28U,    
  HAL_I2C_STATE_BUSY_TX_LISTEN    = 0x29U,   
 
  HAL_I2C_STATE_BUSY_RX_LISTEN    = 0x2AU,   
 
  HAL_I2C_STATE_ABORT             = 0x60U,    
  HAL_I2C_STATE_TIMEOUT           = 0xA0U,    
  HAL_I2C_STATE_ERROR             = 0xE0U     

} HAL_I2C_StateTypeDef;



 


















 
typedef enum
{
  HAL_I2C_MODE_NONE               = 0x00U,    
  HAL_I2C_MODE_MASTER             = 0x10U,    
  HAL_I2C_MODE_SLAVE              = 0x20U,    
  HAL_I2C_MODE_MEM                = 0x40U     

} HAL_I2C_ModeTypeDef;



 




 
#line 178 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c.h"


 




 
typedef struct __I2C_HandleTypeDef
{
  I2C_TypeDef                *Instance;       

  I2C_InitTypeDef            Init;            

  uint8_t                    *pBuffPtr;       

  uint16_t                   XferSize;        

  volatile uint16_t              XferCount;       

  volatile uint32_t              XferOptions;    
 

  volatile uint32_t              PreviousState;   

  HAL_StatusTypeDef(*XferISR)(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);   

  DMA_HandleTypeDef          *hdmatx;         

  DMA_HandleTypeDef          *hdmarx;         

  HAL_LockTypeDef            Lock;            

  volatile HAL_I2C_StateTypeDef  State;           

  volatile HAL_I2C_ModeTypeDef   Mode;            

  volatile uint32_t              ErrorCode;       

  volatile uint32_t              AddrEventCount;  

#line 236 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c.h"
} I2C_HandleTypeDef;

#line 266 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c.h"


 



 
 



 



 
#line 288 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c.h"



 




 



 




 



 




 



 
#line 327 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c.h"


 



 




 



 




 



 




 



 




 



 





 



 






 






 
#line 401 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c.h"


 



 
#line 424 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c.h"


 



 

 



 




 
#line 451 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c.h"














 















 















 

























 




















 






 





 





 



 

 
#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c_ex.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c_ex.h"



 



 

 
 


 



 




 



 
#line 79 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c_ex.h"


 



 

 


 



 

 


 



 
 
HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *hi2c, uint32_t AnalogFilter);
HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *hi2c, uint32_t DigitalFilter);


 



 
HAL_StatusTypeDef HAL_I2CEx_EnableWakeUp(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2CEx_DisableWakeUp(I2C_HandleTypeDef *hi2c);


 



 
void HAL_I2CEx_EnableFastModePlus(uint32_t ConfigFastModePlus);
void HAL_I2CEx_DisableFastModePlus(uint32_t ConfigFastModePlus);


 




 

 


 



 

 


 





#line 161 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c_ex.h"


 

 


 
 


 



 



 







 
#line 571 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c.h"

 


 



 
 
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c);

 
#line 595 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c.h"


 



 
 
 
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size,
                                          uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size,
                                         uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Receive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                    uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                   uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials,
                                        uint32_t Timeout);

 
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                             uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                            uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                       uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                      uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                                 uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                                uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size,
                                                uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size,
                                               uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DisableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);

 
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                              uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                             uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                        uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                       uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                                  uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                                 uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size,
                                                 uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size,
                                                uint32_t XferOptions);


 



 
 
void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c);


 



 
 
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c);
HAL_I2C_ModeTypeDef  HAL_I2C_GetMode(I2C_HandleTypeDef *hi2c);
uint32_t             HAL_I2C_GetError(I2C_HandleTypeDef *hi2c);



 



 

 


 



 

 


 







#line 729 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c.h"



















#line 755 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_i2c.h"




























 

 


 
 


 



 



 








 
#line 279 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"


















#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr.h"



 



 

 



 



 
typedef struct
{
  uint32_t PVDLevel;   
 

  uint32_t Mode;      
 
}PWR_PVDTypeDef;




 

 



 




 
#line 80 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr.h"


 



 
#line 94 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr.h"


 






 




 



 




 



 




 




 



 



 



 



 

 


 









































 





















 






 





 





 





 





 





 





 






 






 









 









 





 





 




 


 


 






#line 330 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr.h"










 

 
#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"



 



 


 



 




 
typedef struct
{
  uint32_t PVMType;   






 

  uint32_t Mode;      
 
}PWR_PVMTypeDef;



 

 



 



 



 




 
#line 102 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"


 



 
#line 117 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"


 



 
#line 131 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"


 





 







 




 




 



 




 



 
#line 187 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"


 



 
#line 213 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"


 



 
#line 228 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"


 



 
#line 243 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"


 










 
#line 268 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"

#line 281 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"


 



 
#line 295 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"


 



 

 


 





 





 





 





 





 





 





 






 






 









 









 





 





 









 





 





 





 





 





 





 






 






 









 









 





 





 








 





 





 





 





 





 





 






 






 









 









 





 





 








 





 





 





 





 





 





 






 






 









 









 





 





 


















 
#line 695 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"



 

 


 

#line 720 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"

#line 734 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"

#line 743 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"

#line 751 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"

#line 760 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"











#line 812 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"

#line 821 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"



 




 



 


 
uint32_t HAL_PWREx_GetVoltageRange(void);
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t VoltageScaling);
void HAL_PWREx_EnableBatteryCharging(uint32_t ResistorSelection);
void HAL_PWREx_DisableBatteryCharging(void);

void HAL_PWREx_EnableVddUSB(void);
void HAL_PWREx_DisableVddUSB(void);


void HAL_PWREx_EnableVddIO2(void);
void HAL_PWREx_DisableVddIO2(void);

void HAL_PWREx_EnableInternalWakeUpLine(void);
void HAL_PWREx_DisableInternalWakeUpLine(void);
HAL_StatusTypeDef HAL_PWREx_EnableGPIOPullUp(uint32_t GPIO, uint32_t GPIONumber);
HAL_StatusTypeDef HAL_PWREx_DisableGPIOPullUp(uint32_t GPIO, uint32_t GPIONumber);
HAL_StatusTypeDef HAL_PWREx_EnableGPIOPullDown(uint32_t GPIO, uint32_t GPIONumber);
HAL_StatusTypeDef HAL_PWREx_DisableGPIOPullDown(uint32_t GPIO, uint32_t GPIONumber);
void HAL_PWREx_EnablePullUpPullDownConfig(void);
void HAL_PWREx_DisablePullUpPullDownConfig(void);
void HAL_PWREx_EnableSRAM2ContentRetention(void);
void HAL_PWREx_DisableSRAM2ContentRetention(void);
HAL_StatusTypeDef HAL_PWREx_SetSRAM2ContentRetention(uint32_t SRAM2Size);
#line 869 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"
void HAL_PWREx_EnablePVM1(void);
void HAL_PWREx_DisablePVM1(void);


void HAL_PWREx_EnablePVM2(void);
void HAL_PWREx_DisablePVM2(void);

void HAL_PWREx_EnablePVM3(void);
void HAL_PWREx_DisablePVM3(void);
void HAL_PWREx_EnablePVM4(void);
void HAL_PWREx_DisablePVM4(void);
HAL_StatusTypeDef HAL_PWREx_ConfigPVM(PWR_PVMTypeDef *sConfigPVM);
#line 889 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr_ex.h"


 
void HAL_PWREx_EnableLowPowerRunMode(void);
HAL_StatusTypeDef HAL_PWREx_DisableLowPowerRunMode(void);
void HAL_PWREx_EnterSTOP0Mode(uint8_t STOPEntry);
void HAL_PWREx_EnterSTOP1Mode(uint8_t STOPEntry);
void HAL_PWREx_EnterSTOP2Mode(uint8_t STOPEntry);
void HAL_PWREx_EnterSHUTDOWNMode(void);

void HAL_PWREx_PVD_PVM_IRQHandler(void);

void HAL_PWREx_PVM1Callback(void);


void HAL_PWREx_PVM2Callback(void);

void HAL_PWREx_PVM3Callback(void);
void HAL_PWREx_PVM4Callback(void);



 



 



 



 








 
#line 344 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_pwr.h"

 



 



 

 
void HAL_PWR_DeInit(void);
void HAL_PWR_EnableBkUpAccess(void);
void HAL_PWR_DisableBkUpAccess(void);



 



 

 
HAL_StatusTypeDef HAL_PWR_ConfigPVD(PWR_PVDTypeDef *sConfigPVD);
void HAL_PWR_EnablePVD(void);
void HAL_PWR_DisablePVD(void);


 
void HAL_PWR_EnableWakeUpPin(uint32_t WakeUpPinPolarity);
void HAL_PWR_DisableWakeUpPin(uint32_t WakeUpPinx);

 
void HAL_PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry);
void HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry);
void HAL_PWR_EnterSTANDBYMode(void);

void HAL_PWR_EnableSleepOnExit(void);
void HAL_PWR_DisableSleepOnExit(void);
void HAL_PWR_EnableSEVOnPend(void);
void HAL_PWR_DisableSEVOnPend(void);

void HAL_PWR_PVDCallback(void);




 



 



 



 








 
#line 299 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"










#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"



 



 

 


 



 
typedef enum
{
  HAL_RTC_STATE_RESET             = 0x00U,   
  HAL_RTC_STATE_READY             = 0x01U,   
  HAL_RTC_STATE_BUSY              = 0x02U,   
  HAL_RTC_STATE_TIMEOUT           = 0x03U,   
  HAL_RTC_STATE_ERROR             = 0x04U    

} HAL_RTCStateTypeDef;



 
typedef struct
{
  uint32_t HourFormat;      
 

  uint32_t AsynchPrediv;    
 

  uint32_t SynchPrediv;     
 

  uint32_t OutPut;          
 

  uint32_t OutPutRemap;     
 

  uint32_t OutPutPolarity;  
 

  uint32_t OutPutType;      
 





#line 94 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"
} RTC_InitTypeDef;



 
typedef struct
{
  uint8_t Hours;            

 

  uint8_t Minutes;          
 

  uint8_t Seconds;          
 

  uint8_t TimeFormat;       
 

#line 122 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"
  uint32_t SubSeconds;      

 


  uint32_t SecondFraction;  



 

  uint32_t DayLightSaving;   

  uint32_t StoreOperation;   
} RTC_TimeTypeDef;



 
typedef struct
{
  uint8_t WeekDay;  
 

  uint8_t Month;    
 

  uint8_t Date;     
 

  uint8_t Year;     
 

} RTC_DateTypeDef;



 
typedef struct
{
  RTC_TimeTypeDef AlarmTime;      

  uint32_t AlarmMask;            
 

#line 178 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"
  uint32_t SubSeconds;            


  uint32_t AlarmSubSecondMask;   
 

  uint32_t AlarmDateWeekDaySel;  
 

  uint8_t AlarmDateWeekDay;      

 

  uint32_t Alarm;                
 
} RTC_AlarmTypeDef;



 



typedef struct

{
  RTC_TypeDef               *Instance;   




  RTC_InitTypeDef           Init;        

  HAL_LockTypeDef           Lock;        

  volatile HAL_RTCStateTypeDef  State;       

#line 233 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"

} RTC_HandleTypeDef;

#line 262 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"



 

 


 



 




 



 
#line 291 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"


 



 




 



 
#line 314 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"


 



 






 



 
#line 339 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"


 



 




 



 





 



 




 

#line 385 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"



 




 




 

 
#line 413 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"



 



 
#line 428 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"



 



 





 



 
#line 453 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"



 



 





 




 
#line 505 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"


 



 
#line 519 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"


 

#line 577 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"



 
#line 595 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"


 




 

 


 




 
#line 622 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"





 










 















 
#line 662 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"











 
#line 681 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"





 






 






 






 










 










 










 













 


#line 770 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"











 











 
#line 800 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"




 





 





 





 





 





 





 





 





 








 








 





 





 




 

 
#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"



 



 

 


 



 
typedef struct
{
  uint32_t Tamper;                      
 

  uint32_t Interrupt;                   
 

  uint32_t Trigger;                     
 

  uint32_t NoErase;                     
 

  uint32_t MaskFlag;                    
 

  uint32_t Filter;                      
 

  uint32_t SamplingFrequency;           
 

  uint32_t PrechargeDuration;           
 

  uint32_t TamperPullUp;                
 

  uint32_t TimeStampOnTamperDetection;  
 
} RTC_TamperTypeDef;


 



 

 



 

 
 
 




 




 



 



 

 
 
 



 
#line 128 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


 

 
 
 



 
#line 145 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


 



 







 

#line 176 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"



 




 



 




 


 
 
 



 
#line 225 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


 



 
#line 243 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


 



 
#line 257 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


 



 
#line 271 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


 



 
#line 289 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"

#line 297 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"



 



 
#line 325 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"

#line 345 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


 



 
#line 362 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"

#line 374 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


 



 
#line 388 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"



 



 
#line 403 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


 



 

#line 427 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


 



 
#line 448 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


 

 
 
 



 
#line 465 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


 



 
#line 504 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


 






 
#line 533 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


#line 645 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"




 

 


 

#line 694 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"

 


 




 






 









 









 










 













 










 













 







 
 



 





 





 





 





 





 





 





 





 









 








 





 





 




 

 


 




 






 









 









 









 












 










 














 










 






 









 













 







#line 1047 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"

 


 





 






 






 






 









 








 


 


 





 










 
#line 1127 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"





 










 











 






 



 










 















 







 










 















 















 














 









 





 





 





 





 





 





 





 





 








 








 





 





 




 

#line 1452 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"



 

 



 

 
 
 

 



 
HAL_StatusTypeDef HAL_RTCEx_SetTimeStamp(RTC_HandleTypeDef *hrtc, uint32_t TimeStampEdge, uint32_t RTC_TimeStampPin);
HAL_StatusTypeDef HAL_RTCEx_SetTimeStamp_IT(RTC_HandleTypeDef *hrtc, uint32_t TimeStampEdge, uint32_t RTC_TimeStampPin);
HAL_StatusTypeDef HAL_RTCEx_DeactivateTimeStamp(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_SetInternalTimeStamp(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DeactivateInternalTimeStamp(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_GetTimeStamp(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTimeStamp, RTC_DateTypeDef *sTimeStampDate, uint32_t Format);
void              HAL_RTCEx_TamperTimeStampIRQHandler(RTC_HandleTypeDef *hrtc);
void              HAL_RTCEx_TimeStampEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForTimeStampEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


 

 
 
 

 



 
HAL_StatusTypeDef HAL_RTCEx_SetWakeUpTimer(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock);



HAL_StatusTypeDef HAL_RTCEx_SetWakeUpTimer_IT(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock);

HAL_StatusTypeDef HAL_RTCEx_DeactivateWakeUpTimer(RTC_HandleTypeDef *hrtc);
uint32_t          HAL_RTCEx_GetWakeUpTimer(RTC_HandleTypeDef *hrtc);
void              HAL_RTCEx_WakeUpTimerIRQHandler(RTC_HandleTypeDef *hrtc);
void              HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForWakeUpTimerEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


 

 
 
 

 



 
HAL_StatusTypeDef HAL_RTCEx_SetSmoothCalib(RTC_HandleTypeDef *hrtc, uint32_t SmoothCalibPeriod, uint32_t SmoothCalibPlusPulses, uint32_t SmoothCalibMinusPulsesValue);



HAL_StatusTypeDef HAL_RTCEx_SetSynchroShift(RTC_HandleTypeDef *hrtc, uint32_t ShiftAdd1S, uint32_t ShiftSubFS);
HAL_StatusTypeDef HAL_RTCEx_SetCalibrationOutPut(RTC_HandleTypeDef *hrtc, uint32_t CalibOutput);
HAL_StatusTypeDef HAL_RTCEx_DeactivateCalibrationOutPut(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_SetRefClock(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DeactivateRefClock(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_EnableBypassShadow(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DisableBypassShadow(RTC_HandleTypeDef *hrtc);
#line 1535 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


 

 


 

void              HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForAlarmBEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


 



 
HAL_StatusTypeDef HAL_RTCEx_SetTamper(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef *sTamper);
HAL_StatusTypeDef HAL_RTCEx_SetTamper_IT(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef *sTamper);
HAL_StatusTypeDef HAL_RTCEx_DeactivateTamper(RTC_HandleTypeDef *hrtc, uint32_t Tamper);


HAL_StatusTypeDef HAL_RTCEx_PollForTamper1Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);

HAL_StatusTypeDef HAL_RTCEx_PollForTamper2Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);

HAL_StatusTypeDef HAL_RTCEx_PollForTamper3Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);



void              HAL_RTCEx_Tamper1EventCallback(RTC_HandleTypeDef *hrtc);

void              HAL_RTCEx_Tamper2EventCallback(RTC_HandleTypeDef *hrtc);

void              HAL_RTCEx_Tamper3EventCallback(RTC_HandleTypeDef *hrtc);





 



 
void              HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister, uint32_t Data);
uint32_t          HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister);


 



 

 
 
 


 










 

 


 



 







#line 1630 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"

















#line 1653 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


















#line 1679 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"






















#line 1721 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc_ex.h"


 



 



 



 







 
#line 891 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"

 


 



 
 
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_DeInit(RTC_HandleTypeDef *hrtc);

void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc);
void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc);

 






 



 
 
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
void              HAL_RTC_DST_Add1Hour(RTC_HandleTypeDef *hrtc);
void              HAL_RTC_DST_Sub1Hour(RTC_HandleTypeDef *hrtc);
void              HAL_RTC_DST_SetStoreOperation(RTC_HandleTypeDef *hrtc);
void              HAL_RTC_DST_ClearStoreOperation(RTC_HandleTypeDef *hrtc);
uint32_t          HAL_RTC_DST_ReadStoreOperation(RTC_HandleTypeDef *hrtc);


 



 
 
HAL_StatusTypeDef HAL_RTC_SetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetAlarm_IT(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_DeactivateAlarm(RTC_HandleTypeDef *hrtc, uint32_t Alarm);
HAL_StatusTypeDef HAL_RTC_GetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Alarm, uint32_t Format);
void              HAL_RTC_AlarmIRQHandler(RTC_HandleTypeDef *hrtc);
void              HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_PollForAlarmAEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


 



 
 
HAL_StatusTypeDef   HAL_RTC_WaitForSynchro(RTC_HandleTypeDef *hrtc);


 



 
 
HAL_RTCStateTypeDef HAL_RTC_GetState(RTC_HandleTypeDef *hrtc);


 



 

 
 
 


 
 








#line 991 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"







 

 


 



 
#line 1020 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"





































#line 1064 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"



#line 1074 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_rtc.h"





























 



 

 


 
HAL_StatusTypeDef  RTC_EnterInitMode(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef  RTC_ExitInitMode(RTC_HandleTypeDef *hrtc);
uint8_t            RTC_ByteToBcd2(uint8_t Value);
uint8_t            RTC_Bcd2ToByte(uint8_t Value);


 




 



 



 







 

#line 311 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"














#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi.h"



 



 

 


 



 
typedef struct
{
  uint32_t Mode;                
 

  uint32_t Direction;           
 

  uint32_t DataSize;            
 

  uint32_t CLKPolarity;         
 

  uint32_t CLKPhase;            
 

  uint32_t NSS;                 

 

  uint32_t BaudRatePrescaler;   



 

  uint32_t FirstBit;            
 

  uint32_t TIMode;              
 

  uint32_t CRCCalculation;      
 

  uint32_t CRCPolynomial;       
 

  uint32_t CRCLength;           

 

  uint32_t NSSPMode;            




 
} SPI_InitTypeDef;



 
typedef enum
{
  HAL_SPI_STATE_RESET      = 0x00U,     
  HAL_SPI_STATE_READY      = 0x01U,     
  HAL_SPI_STATE_BUSY       = 0x02U,     
  HAL_SPI_STATE_BUSY_TX    = 0x03U,     
  HAL_SPI_STATE_BUSY_RX    = 0x04U,     
  HAL_SPI_STATE_BUSY_TX_RX = 0x05U,     
  HAL_SPI_STATE_ERROR      = 0x06U,     
  HAL_SPI_STATE_ABORT      = 0x07U      
} HAL_SPI_StateTypeDef;



 
typedef struct __SPI_HandleTypeDef
{
  SPI_TypeDef                *Instance;       

  SPI_InitTypeDef            Init;            

  uint8_t                    *pTxBuffPtr;     

  uint16_t                   TxXferSize;      

  volatile uint16_t              TxXferCount;     

  uint8_t                    *pRxBuffPtr;     

  uint16_t                   RxXferSize;      

  volatile uint16_t              RxXferCount;     

  uint32_t                   CRCSize;         

  void (*RxISR)(struct __SPI_HandleTypeDef *hspi);    

  void (*TxISR)(struct __SPI_HandleTypeDef *hspi);    

  DMA_HandleTypeDef          *hdmatx;         

  DMA_HandleTypeDef          *hdmarx;         

  HAL_LockTypeDef            Lock;            

  volatile HAL_SPI_StateTypeDef  State;           

  volatile uint32_t              ErrorCode;       

#line 163 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi.h"
} SPI_HandleTypeDef;

#line 190 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi.h"


 

 


 



 
#line 213 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi.h"


 



 




 



 





 



 
#line 252 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi.h"


 



 




 



 




 



 





 



 




 



 
#line 304 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi.h"


 



 




 



 




 



 




 







 





 








 





 



 





 



 
#line 388 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi.h"


 



 







 



 






 



 

 


 





 
#line 438 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi.h"










 











 











 


















 






 






 
#line 513 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi.h"





 
#line 526 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi.h"





 
#line 538 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi.h"





 






 




 

 


 





 






 






 

















 











 







 







 







 





 







 
#line 662 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi.h"





 







 







 








 







 
#line 709 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi.h"





 







 







 







 








 







 




 

 
#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi_ex.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi_ex.h"



 



 

 
 
 
 


 

 
 


 
HAL_StatusTypeDef HAL_SPIEx_FlushRxFifo(SPI_HandleTypeDef *hspi);


 



 



 



 







 
#line 764 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_spi.h"

 


 



 
 
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);

 






 



 
 
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                             uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size);
HAL_StatusTypeDef HAL_SPI_DMAPause(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAResume(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *hspi);
 
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_Abort_IT(SPI_HandleTypeDef *hspi);

void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi);


 



 
 
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *hspi);
uint32_t             HAL_SPI_GetError(SPI_HandleTypeDef *hspi);


 



 



 



 







 
#line 327 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"






#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"



 



 

 


 



 
typedef struct
{
  uint32_t Prescaler;         
 

  uint32_t CounterMode;       
 

  uint32_t Period;            

 

  uint32_t ClockDivision;     
 

  uint32_t RepetitionCounter;  






 

  uint32_t AutoReloadPreload;  
 
} TIM_Base_InitTypeDef;



 
typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         
 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 

  uint32_t OCFastMode;    

 


  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 
} TIM_OC_InitTypeDef;



 
typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         
 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 

  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 

  uint32_t ICPolarity;    
 

  uint32_t ICSelection;   
 

  uint32_t ICFilter;      
 
} TIM_OnePulse_InitTypeDef;



 
typedef struct
{
  uint32_t  ICPolarity;  
 

  uint32_t ICSelection;  
 

  uint32_t ICPrescaler;  
 

  uint32_t ICFilter;     
 
} TIM_IC_InitTypeDef;



 
typedef struct
{
  uint32_t EncoderMode;   
 

  uint32_t IC1Polarity;   
 

  uint32_t IC1Selection;  
 

  uint32_t IC1Prescaler;  
 

  uint32_t IC1Filter;     
 

  uint32_t IC2Polarity;   
 

  uint32_t IC2Selection;  
 

  uint32_t IC2Prescaler;  
 

  uint32_t IC2Filter;     
 
} TIM_Encoder_InitTypeDef;



 
typedef struct
{
  uint32_t ClockSource;     
 
  uint32_t ClockPolarity;   
 
  uint32_t ClockPrescaler;  
 
  uint32_t ClockFilter;     
 
} TIM_ClockConfigTypeDef;



 
typedef struct
{
  uint32_t ClearInputState;      
 
  uint32_t ClearInputSource;     
 
  uint32_t ClearInputPolarity;   
 
  uint32_t ClearInputPrescaler;  
 
  uint32_t ClearInputFilter;     
 
} TIM_ClearInputConfigTypeDef;





 
typedef struct
{
  uint32_t  MasterOutputTrigger;   
 
  uint32_t  MasterOutputTrigger2;  
 
  uint32_t  MasterSlaveMode;       





 
} TIM_MasterConfigTypeDef;



 
typedef struct
{
  uint32_t  SlaveMode;         
 
  uint32_t  InputTrigger;      
 
  uint32_t  TriggerPolarity;   
 
  uint32_t  TriggerPrescaler;  
 
  uint32_t  TriggerFilter;     
 

} TIM_SlaveConfigTypeDef;





 
typedef struct
{
  uint32_t OffStateRunMode;      
 
  uint32_t OffStateIDLEMode;     
 
  uint32_t LockLevel;            
 
  uint32_t DeadTime;             
 
  uint32_t BreakState;           
 
  uint32_t BreakPolarity;        
 
  uint32_t BreakFilter;          
 
  uint32_t Break2State;          
 
  uint32_t Break2Polarity;       
 
  uint32_t Break2Filter;         
 
  uint32_t AutomaticOutput;      
 
} TIM_BreakDeadTimeConfigTypeDef;



 
typedef enum
{
  HAL_TIM_STATE_RESET             = 0x00U,     
  HAL_TIM_STATE_READY             = 0x01U,     
  HAL_TIM_STATE_BUSY              = 0x02U,     
  HAL_TIM_STATE_TIMEOUT           = 0x03U,     
  HAL_TIM_STATE_ERROR             = 0x04U      
} HAL_TIM_StateTypeDef;



 
typedef enum
{
  HAL_TIM_CHANNEL_STATE_RESET             = 0x00U,     
  HAL_TIM_CHANNEL_STATE_READY             = 0x01U,     
  HAL_TIM_CHANNEL_STATE_BUSY              = 0x02U,     
} HAL_TIM_ChannelStateTypeDef;



 
typedef enum
{
  HAL_DMA_BURST_STATE_RESET             = 0x00U,     
  HAL_DMA_BURST_STATE_READY             = 0x01U,     
  HAL_DMA_BURST_STATE_BUSY              = 0x02U,     
} HAL_TIM_DMABurstStateTypeDef;



 
typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_1        = 0x01U,     
  HAL_TIM_ACTIVE_CHANNEL_2        = 0x02U,     
  HAL_TIM_ACTIVE_CHANNEL_3        = 0x04U,     
  HAL_TIM_ACTIVE_CHANNEL_4        = 0x08U,     
  HAL_TIM_ACTIVE_CHANNEL_5        = 0x10U,     
  HAL_TIM_ACTIVE_CHANNEL_6        = 0x20U,     
  HAL_TIM_ACTIVE_CHANNEL_CLEARED  = 0x00U      
} HAL_TIM_ActiveChannel;



 



typedef struct

{
  TIM_TypeDef                        *Instance;          
  TIM_Base_InitTypeDef               Init;               
  HAL_TIM_ActiveChannel              Channel;            
  DMA_HandleTypeDef                  *hdma[7];          
 
  HAL_LockTypeDef                    Lock;               
  volatile HAL_TIM_StateTypeDef          State;              
  volatile HAL_TIM_ChannelStateTypeDef   ChannelState[6];    
  volatile HAL_TIM_ChannelStateTypeDef   ChannelNState[4];   
  volatile HAL_TIM_DMABurstStateTypeDef  DMABurstState;      

#line 391 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"
} TIM_HandleTypeDef;

#line 436 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"



 
 

 


 



 





 



 
#line 486 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"


 



 
#line 502 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"


 



 





 



 




 



 






 



 







 



 




 



 





 



 




 



 





 



 




 



 




 



 




 



 




 



 




 



 




 



 





 



 




 



 







 



 






 



 




 



 





 



 
#line 712 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"


 



 




 



 
#line 735 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"


 



 
#line 758 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"


 



 
#line 772 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"


 



 
#line 789 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"


 



 







 



 






 



 




 



 






 



 




 



 




 


 






 



 




 



 




 



 




 



 




 



 





 



 






 



 
#line 932 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"


 



 
#line 955 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"


 



 




 



 
#line 977 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"


 



 
#line 998 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"


 



 
#line 1014 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"


 



 







 



 






 



 




 



 
#line 1071 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"


 



 
#line 1085 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"


 



 






 



 






 



 
 

 


 




 
#line 1170 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"





 






 






 
#line 1200 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"






 
#line 1217 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"






 















 















 














 














 























 























 
















 
















 








 







 







 








 







 









 






 







 










 











 
#line 1464 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"








 


















 




















 



















 
#line 1540 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"













 
#line 1561 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"













 
#line 1582 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"













 
#line 1603 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"

















 
#line 1628 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"

















 
#line 1653 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"








 












 
















 








 
 

 


 

 




 
 

 


 




#line 1748 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"



























































#line 1814 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"








#line 1832 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"























































#line 1895 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"

#line 1913 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"




#line 1923 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"

#line 1930 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"

#line 1939 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"

#line 1948 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"























#line 1989 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"







































#line 2035 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"

#line 2043 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"

#line 2052 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"













#line 2071 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"



 
 

 
#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim_ex.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim_ex.h"



 



 

 


 



 

typedef struct
{
  uint32_t IC1Polarity;         
 

  uint32_t IC1Prescaler;        
 

  uint32_t IC1Filter;           
 

  uint32_t Commutation_Delay;   
 
} TIM_HallSensor_InitTypeDef;



 
typedef struct
{
  uint32_t Source;         
 
  uint32_t Enable;         
 
  uint32_t Polarity;       

 
}
TIMEx_BreakInputConfigTypeDef;



 
 

 


 



 
#line 108 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim_ex.h"

#line 133 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim_ex.h"

#line 142 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim_ex.h"

#line 154 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim_ex.h"

#line 161 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim_ex.h"

#line 172 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim_ex.h"

#line 182 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim_ex.h"

#line 189 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim_ex.h"


 



 




 



 
#line 211 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim_ex.h"


 



 




 



 




 



 
 

 


 



 
 

 


 





#line 267 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim_ex.h"









 
 

 


 




 
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(TIM_HandleTypeDef *htim, TIM_HallSensor_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(TIM_HandleTypeDef *htim);

void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef *htim);

 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(TIM_HandleTypeDef *htim);


 




 
 
 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);


 




 
 
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent(TIM_HandleTypeDef *htim, uint32_t  InputTrigger,
                                              uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent_IT(TIM_HandleTypeDef *htim, uint32_t  InputTrigger,
                                                 uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent_DMA(TIM_HandleTypeDef *htim, uint32_t  InputTrigger,
                                                  uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *htim,
                                                        TIM_MasterConfigTypeDef *sMasterConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef *htim,
                                                TIM_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakInput(TIM_HandleTypeDef *htim, uint32_t BreakInput,
                                             TIMEx_BreakInputConfigTypeDef *sBreakInputConfig);
HAL_StatusTypeDef HAL_TIMEx_GroupChannel5(TIM_HandleTypeDef *htim, uint32_t Channels);
HAL_StatusTypeDef HAL_TIMEx_RemapConfig(TIM_HandleTypeDef *htim, uint32_t Remap);


 




 
 
void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_CommutHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_Break2Callback(TIM_HandleTypeDef *htim);


 




 
 
HAL_TIM_StateTypeDef HAL_TIMEx_HallSensor_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_ChannelStateTypeDef HAL_TIMEx_GetChannelNState(TIM_HandleTypeDef *htim,  uint32_t ChannelN);


 



 
 

 


 
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma);
void TIMEx_DMACommutationHalfCplt(DMA_HandleTypeDef *hdma);


 
 



 



 








 
#line 2079 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"

 


 




 
 
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Base_Stop_DMA(TIM_HandleTypeDef *htim);


 




 
 
HAL_StatusTypeDef HAL_TIM_OC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_OC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_OC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
HAL_StatusTypeDef HAL_TIM_IC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_IC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_IC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Init(TIM_HandleTypeDef *htim, uint32_t OnePulseMode);
HAL_StatusTypeDef HAL_TIM_OnePulse_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);


 




 
 
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim,  TIM_Encoder_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_TIM_Encoder_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData1,
                                            uint32_t *pData2, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);


 




 
 
HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_IC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OnePulse_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OnePulse_InitTypeDef *sConfig,
                                                 uint32_t OutputChannel,  uint32_t InputChannel);
HAL_StatusTypeDef HAL_TIM_ConfigOCrefClear(TIM_HandleTypeDef *htim, TIM_ClearInputConfigTypeDef *sClearInputConfig,
                                           uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *htim, TIM_ClockConfigTypeDef *sClockSourceConfig);
HAL_StatusTypeDef HAL_TIM_ConfigTI1Input(TIM_HandleTypeDef *htim, uint32_t TI1_Selection);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchro(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef *sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchro_IT(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef *sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                              uint32_t BurstRequestSrc, uint32_t  *BurstBuffer, uint32_t  BurstLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_MultiWriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                                   uint32_t BurstRequestSrc, uint32_t *BurstBuffer, uint32_t BurstLength,
                                                   uint32_t DataLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                             uint32_t BurstRequestSrc, uint32_t  *BurstBuffer, uint32_t  BurstLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_MultiReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                                  uint32_t BurstRequestSrc, uint32_t  *BurstBuffer, uint32_t  BurstLength,
                                                  uint32_t  DataLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *htim, uint32_t EventSource);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);

 








 




 
 
HAL_TIM_StateTypeDef HAL_TIM_Base_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_PWM_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_IC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OnePulse_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_Encoder_GetState(TIM_HandleTypeDef *htim);

 
HAL_TIM_ActiveChannel HAL_TIM_GetActiveChannel(TIM_HandleTypeDef *htim);
HAL_TIM_ChannelStateTypeDef HAL_TIM_GetChannelState(TIM_HandleTypeDef *htim,  uint32_t Channel);
HAL_TIM_DMABurstStateTypeDef HAL_TIM_DMABurstState(TIM_HandleTypeDef *htim);


 



 
 

 


 
void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure);
void TIM_TI1_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection, uint32_t TIM_ICFilter);
void TIM_OC2_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
void TIM_ETR_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ExtTRGPrescaler,
                       uint32_t TIM_ExtTRGPolarity, uint32_t ExtTRGFilter);

void TIM_DMADelayPulseHalfCplt(DMA_HandleTypeDef *hdma);
void TIM_DMAError(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureCplt(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureHalfCplt(DMA_HandleTypeDef *hdma);
void TIM_CCxChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelState);







 
 



 



 







 
#line 335 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"


#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"




 



 

 


 



 
typedef enum
{
  HAL_TSC_STATE_RESET  = 0x00UL,  
  HAL_TSC_STATE_READY  = 0x01UL,  
  HAL_TSC_STATE_BUSY   = 0x02UL,  
  HAL_TSC_STATE_ERROR  = 0x03UL   
} HAL_TSC_StateTypeDef;



 
typedef enum
{
  TSC_GROUP_ONGOING   = 0x00UL,  
  TSC_GROUP_COMPLETED = 0x01UL  
} TSC_GroupStatusTypeDef;



 
typedef struct
{
  uint32_t CTPulseHighLength;       
 
  uint32_t CTPulseLowLength;        
 
  FunctionalState SpreadSpectrum;   
 
  uint32_t SpreadSpectrumDeviation; 
 
  uint32_t SpreadSpectrumPrescaler; 
 
  uint32_t PulseGeneratorPrescaler; 
 
  uint32_t MaxCountValue;           
 
  uint32_t IODefaultMode;           
 
  uint32_t SynchroPinPolarity;      
 
  uint32_t AcquisitionMode;         
 
  FunctionalState MaxCountInterrupt;
 
  uint32_t ChannelIOs;               
  uint32_t ShieldIOs;                
  uint32_t SamplingIOs;              
} TSC_InitTypeDef;



 
typedef struct
{
  uint32_t ChannelIOs;   
  uint32_t ShieldIOs;    
  uint32_t SamplingIOs;  
} TSC_IOConfigTypeDef;



 



typedef struct

{
  TSC_TypeDef               *Instance;   
  TSC_InitTypeDef           Init;        
  volatile HAL_TSC_StateTypeDef State;       
  HAL_LockTypeDef           Lock;        
  volatile uint32_t             ErrorCode;   

#line 130 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"
} TSC_HandleTypeDef;

enum
{
  TSC_GROUP1_IDX = 0x00UL,
  TSC_GROUP2_IDX,
  TSC_GROUP3_IDX,
  TSC_GROUP4_IDX,

  TSC_GROUP5_IDX,


  TSC_GROUP6_IDX,


  TSC_GROUP7_IDX,


  TSC_GROUP8_IDX,

  TSC_NB_OF_GROUPS
};

#line 173 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"



 

 


 




 






 



 
#line 214 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"


 



 
#line 237 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"


 



 




 



 
#line 261 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"


 



 
#line 275 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"


 



 




 



 




 



 




 



 




 



 




 



 
#line 343 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"
























#line 379 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"

#line 392 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"

#line 405 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"

#line 417 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"


 



 

 



 




 
#line 444 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"





 






 






 






 






 






 






 






 







 







 






 







 







 







 







 







 







 







 







 







 







 







 







 






 





 

 



 

#line 654 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"

#line 671 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"







#line 686 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"





#line 698 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"












#line 744 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tsc.h"



 

 


 



 
 
HAL_StatusTypeDef HAL_TSC_Init(TSC_HandleTypeDef *htsc);
HAL_StatusTypeDef HAL_TSC_DeInit(TSC_HandleTypeDef *htsc);
void HAL_TSC_MspInit(TSC_HandleTypeDef *htsc);
void HAL_TSC_MspDeInit(TSC_HandleTypeDef *htsc);

 






 



 
 
HAL_StatusTypeDef HAL_TSC_Start(TSC_HandleTypeDef *htsc);
HAL_StatusTypeDef HAL_TSC_Start_IT(TSC_HandleTypeDef *htsc);
HAL_StatusTypeDef HAL_TSC_Stop(TSC_HandleTypeDef *htsc);
HAL_StatusTypeDef HAL_TSC_Stop_IT(TSC_HandleTypeDef *htsc);
HAL_StatusTypeDef HAL_TSC_PollForAcquisition(TSC_HandleTypeDef *htsc);
TSC_GroupStatusTypeDef HAL_TSC_GroupGetStatus(TSC_HandleTypeDef *htsc, uint32_t gx_index);
uint32_t HAL_TSC_GroupGetValue(TSC_HandleTypeDef *htsc, uint32_t gx_index);


 



 
 
HAL_StatusTypeDef HAL_TSC_IOConfig(TSC_HandleTypeDef *htsc, TSC_IOConfigTypeDef *config);
HAL_StatusTypeDef HAL_TSC_IODischarge(TSC_HandleTypeDef *htsc, FunctionalState choice);


 



 
 
HAL_TSC_StateTypeDef HAL_TSC_GetState(TSC_HandleTypeDef *htsc);


 



 
 
void HAL_TSC_IRQHandler(TSC_HandleTypeDef *htsc);
void HAL_TSC_ConvCpltCallback(TSC_HandleTypeDef *htsc);
void HAL_TSC_ErrorCallback(TSC_HandleTypeDef *htsc);


 



 



 



 







 
#line 339 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"


#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"



 



 

 


 



 
typedef struct
{
  uint32_t BaudRate;                  














 

  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint32_t Parity;                    




 

  uint32_t Mode;                      
 

  uint32_t HwFlowCtl;                 

 

  uint32_t OverSampling;              

 

  uint32_t OneBitSampling;            

 






} UART_InitTypeDef;



 
typedef struct
{
  uint32_t AdvFeatureInit;        


 

  uint32_t TxPinLevelInvert;      
 

  uint32_t RxPinLevelInvert;      
 

  uint32_t DataInvert;            

 

  uint32_t Swap;                  
 

  uint32_t OverrunDisable;        
 

  uint32_t DMADisableonRxError;   
 

  uint32_t AutoBaudRateEnable;    
 

  uint32_t AutoBaudRateMode;      

 

  uint32_t MSBFirst;              
 
} UART_AdvFeatureInitTypeDef;








































 
typedef uint32_t HAL_UART_StateTypeDef;



 
typedef enum
{
  UART_CLOCKSOURCE_PCLK1      = 0x00U,     
  UART_CLOCKSOURCE_PCLK2      = 0x01U,     
  UART_CLOCKSOURCE_HSI        = 0x02U,     
  UART_CLOCKSOURCE_SYSCLK     = 0x04U,     
  UART_CLOCKSOURCE_LSE        = 0x08U,     
  UART_CLOCKSOURCE_UNDEFINED  = 0x10U      
} UART_ClockSourceTypeDef;









 
typedef uint32_t HAL_UART_RxTypeTypeDef;



 
typedef struct __UART_HandleTypeDef
{
  USART_TypeDef            *Instance;                 

  UART_InitTypeDef         Init;                      

  UART_AdvFeatureInitTypeDef AdvancedInit;            

  uint8_t                  *pTxBuffPtr;               

  uint16_t                 TxXferSize;                

  volatile uint16_t            TxXferCount;               

  uint8_t                  *pRxBuffPtr;               

  uint16_t                 RxXferSize;                

  volatile uint16_t            RxXferCount;               

  uint16_t                 Mask;                      

#line 241 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"
  volatile HAL_UART_RxTypeTypeDef ReceptionType;          

  void (*RxISR)(struct __UART_HandleTypeDef *huart);  

  void (*TxISR)(struct __UART_HandleTypeDef *huart);  

  DMA_HandleTypeDef        *hdmatx;                   

  DMA_HandleTypeDef        *hdmarx;                   

  HAL_LockTypeDef           Lock;                     

  volatile HAL_UART_StateTypeDef    gState;              

 

  volatile HAL_UART_StateTypeDef    RxState;             
 

  volatile uint32_t                 ErrorCode;            

#line 281 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"

} UART_HandleTypeDef;

#line 316 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"



 

 


 



 
#line 346 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"


 



 
#line 360 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"






 



 






 



 





 



 






 



 





 



 




 



 




 



 




 

#line 458 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"


 
#line 469 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"


 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 







 



 
#line 560 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"


 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 






 



 




 



 




 



 



 



 






 



 




 



 




 



 




 



 



 



 



 





 
#line 760 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"


 


















 
#line 804 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"








 



 
#line 831 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"


 



 






 



 

 


 




 
#line 872 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"




 























 





 





 





 





 





 


#line 940 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"
































 
























 
#line 1007 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"























 
#line 1040 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"























 

























 
#line 1097 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"



















 












 





 





 





 





 














 


















 


















 


















 







 

 


 
#line 1287 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"





 






 






 






 








 









 





 






 









 







 








 










 






 







 







 







 









 







 






 







 







 







 







 







 







 










 
#line 1508 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"





 







 







 







 







 







 








 







 







 







 







 








 



#line 1626 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"



 

 
#line 1 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart_ex.h"

















 

 







 
#line 30 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart_ex.h"



 



 

 


 



 
typedef struct
{
  uint32_t WakeUpEvent;        


 

  uint16_t AddressLength;      
 

  uint8_t Address;              
} UART_WakeUpTypeDef;



 

 


 



 





 



 




 

#line 128 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart_ex.h"


 

 
 


 



 

 
HAL_StatusTypeDef HAL_RS485Ex_Init(UART_HandleTypeDef *huart, uint32_t Polarity, uint32_t AssertionTime,
                                   uint32_t DeassertionTime);



 



 

void HAL_UARTEx_WakeupCallback(UART_HandleTypeDef *huart);








 



 

 
HAL_StatusTypeDef HAL_UARTEx_StopModeWakeUpSourceConfig(UART_HandleTypeDef *huart, UART_WakeUpTypeDef WakeUpSelection);
HAL_StatusTypeDef HAL_UARTEx_EnableStopMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UARTEx_DisableStopMode(UART_HandleTypeDef *huart);


HAL_StatusTypeDef HAL_UARTEx_EnableClockStopMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UARTEx_DisableClockStopMode(UART_HandleTypeDef *huart);


HAL_StatusTypeDef HAL_MultiProcessorEx_AddressLength_Set(UART_HandleTypeDef *huart, uint32_t AddressLength);

#line 187 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart_ex.h"

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint16_t *RxLen, uint32_t Timeout);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);




 



 

 


 





 
#line 630 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart_ex.h"









 
#line 680 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart_ex.h"





 








 



#line 724 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart_ex.h"


 

 



 



 







 
#line 1633 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"

#line 1641 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"

 


 



 

 
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_LIN_Init(UART_HandleTypeDef *huart, uint32_t BreakDetectLength);
HAL_StatusTypeDef HAL_MultiProcessor_Init(UART_HandleTypeDef *huart, uint8_t Address, uint32_t WakeUpMethod);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);

 
#line 1669 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_uart.h"



 



 

 
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_DMAPause(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAResume(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart);
 
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *huart);

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);



 



 

 
void HAL_UART_ReceiverTimeout_Config(UART_HandleTypeDef *huart, uint32_t TimeoutValue);
HAL_StatusTypeDef HAL_UART_EnableReceiverTimeout(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DisableReceiverTimeout(UART_HandleTypeDef *huart);

HAL_StatusTypeDef HAL_LIN_SendBreak(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_EnableMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_DisableMuteMode(UART_HandleTypeDef *huart);
void HAL_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart);



 



 

 
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart);
uint32_t              HAL_UART_GetError(UART_HandleTypeDef *huart);



 



 

 


 



HAL_StatusTypeDef UART_SetConfig(UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_CheckIdleState(UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status,
                                              uint32_t Tickstart, uint32_t Timeout);
void              UART_AdvFeatureConfig(UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_Start_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef UART_Start_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);



 



 



 







 
#line 343 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"


























 
#line 385 "..\\..\\..\\User\\stm32l4xx_hal_conf.h"








 
#line 31 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h"



 



 

 


 



 
typedef enum
{
  HAL_TICK_FREQ_10HZ         = 100U,
  HAL_TICK_FREQ_100HZ        = 10U,
  HAL_TICK_FREQ_1KHZ         = 1U,
  HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;


 



 

 



 



 



 



#line 85 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h"
        
        
        



#line 97 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h"



 



 
#line 111 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h"



 



 
#line 153 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h"



 




 
#line 194 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h"



 





 





 



 





 




 






 



 


 
#line 247 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h"



 



 



 

 


 



 


 
















































































































 



 


 



 



 








 



        
        
        

#line 423 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h"


 


























 





 








 







 






 




 











 





 





 





 








 



 








 










 



 

 


 



 

#line 571 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h"


















#line 606 "../../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h"


 



 

 



 
extern volatile uint32_t uwTick;
extern uint32_t uwTickPrio;
extern HAL_TickFreqTypeDef uwTickFreq;


 

 



 



 

 
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void              HAL_MspInit(void);
void              HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority);



 



 

 
void               HAL_IncTick(void);
void               HAL_Delay(uint32_t Delay);
uint32_t           HAL_GetTick(void);
uint32_t           HAL_GetTickPrio(void);
HAL_StatusTypeDef  HAL_SetTickFreq(HAL_TickFreqTypeDef Freq);
HAL_TickFreqTypeDef HAL_GetTickFreq(void);
void               HAL_SuspendTick(void);
void               HAL_ResumeTick(void);
uint32_t           HAL_GetHalVersion(void);
uint32_t           HAL_GetREVID(void);
uint32_t           HAL_GetDEVID(void);
uint32_t           HAL_GetUIDw0(void);
uint32_t           HAL_GetUIDw1(void);
uint32_t           HAL_GetUIDw2(void);



 



 

 
void              HAL_DBGMCU_EnableDBGSleepMode(void);
void              HAL_DBGMCU_DisableDBGSleepMode(void);
void              HAL_DBGMCU_EnableDBGStopMode(void);
void              HAL_DBGMCU_DisableDBGStopMode(void);
void              HAL_DBGMCU_EnableDBGStandbyMode(void);
void              HAL_DBGMCU_DisableDBGStandbyMode(void);



 



 

 
void              HAL_SYSCFG_SRAM2Erase(void);
void              HAL_SYSCFG_EnableMemorySwappingBank(void);
void              HAL_SYSCFG_DisableMemorySwappingBank(void);


void              HAL_SYSCFG_VREFBUF_VoltageScalingConfig(uint32_t VoltageScaling);
void              HAL_SYSCFG_VREFBUF_HighImpedanceConfig(uint32_t Mode);
void              HAL_SYSCFG_VREFBUF_TrimmingConfig(uint32_t TrimmingValue);
HAL_StatusTypeDef HAL_SYSCFG_EnableVREFBUF(void);
void              HAL_SYSCFG_DisableVREFBUF(void);


void              HAL_SYSCFG_EnableIOAnalogSwitchBooster(void);
void              HAL_SYSCFG_DisableIOAnalogSwitchBooster(void);



 



 



 



 







 
#line 21 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"
#line 1 "../../../Drivers/BSP/STM32L4xx_Nucleo_144/stm32l4xx_nucleo_144.h"





















 

 







 
#line 34 "../../../Drivers/BSP/STM32L4xx_Nucleo_144/stm32l4xx_nucleo_144.h"



 



 



 
typedef enum
{
  LED1 = 0,
  LED_GREEN = LED1,
  LED2 = 1,
  LED_BLUE = LED2,
  LED3 = 2,
  LED_RED = LED3
}
Led_TypeDef;

typedef enum
{
  BUTTON_USER = 0,
   
  BUTTON_KEY = BUTTON_USER
} Button_TypeDef;

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

typedef enum
{
  JOY_NONE  = 0,
  JOY_SEL   = 1,
  JOY_DOWN  = 2,
  JOY_LEFT  = 3,
  JOY_RIGHT = 4,
  JOY_UP    = 5
} JOYState_TypeDef;



 



 



 






 
























 



 




 
#line 138 "../../../Drivers/BSP/STM32L4xx_Nucleo_144/stm32l4xx_nucleo_144.h"




 
#line 149 "../../../Drivers/BSP/STM32L4xx_Nucleo_144/stm32l4xx_nucleo_144.h"



 




 










 




 





 

  



 
 











#line 204 "../../../Drivers/BSP/STM32L4xx_Nucleo_144/stm32l4xx_nucleo_144.h"




 












 





 







 







 







 







 



 

















 






















 



 



 
uint32_t         BSP_GetVersion(void);



 
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_DeInit(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);


 



 
void             BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void             BSP_PB_DeInit(Button_TypeDef Button);
uint32_t         BSP_PB_GetState(Button_TypeDef Button);

uint8_t          BSP_JOY_Init(void);
void             BSP_JOY_DeInit(void);
JOYState_TypeDef BSP_JOY_GetState(void);



 




 
uint32_t         BSP_SMPS_Init(uint32_t VoltageRange);
uint32_t         BSP_SMPS_DeInit(void);
uint32_t         BSP_SMPS_Enable(uint32_t Delay, uint32_t Power_Good_Check);
uint32_t         BSP_SMPS_Disable(void);
uint32_t         BSP_SMPS_Supply_Enable(uint32_t Delay, uint32_t Power_Good_Check);
uint32_t         BSP_SMPS_Supply_Disable(void);


 




 



 



 



 







 
#line 22 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"
#line 1 "..\\..\\..\\User\\smp_debug.h"








 




 
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdarg.h"
 
 
 





 










#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdarg.h"








 

 
 
#line 57 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdarg.h"
    typedef struct __va_list { void *__ap; } va_list;

   






 


   










 


   















 




   

 


   




 



   





 







#line 138 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdarg.h"



#line 147 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdarg.h"

 

#line 16 "..\\..\\..\\User\\smp_debug.h"
 
 
 
 
 
 

 





	
#line 50 "..\\..\\..\\User\\smp_debug.h"

#line 23 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"
#line 1 "..\\..\\..\\Drivers\\SMP\\smp_gpio.h"








 
  
 



 
#line 17 "..\\..\\..\\Drivers\\SMP\\smp_gpio.h"
#line 18 "..\\..\\..\\Drivers\\SMP\\smp_gpio.h"
 
typedef enum{
	SMP_GPIOA = 0,		 
	SMP_GPIOB,			 
	SMP_GPIOC,			 
	SMP_GPIOD,			 
	SMP_GPIOE,			 
	SMP_GPIOF,			 
	SMP_GPIOG			 
}gpio_port;

typedef enum{
	PIN0 = 0,	 
	PIN1,		 
	PIN2,		 
	PIN3,		 
	PIN4,		 
	PIN5,		 
	PIN6,		 
	PIN7,		 
	PIN8,		 
	PIN9,		 
	PIN10,		 
	PIN11,		 
	PIN12,		 
	PIN13,		 
	PIN14,		 
	PIN15		 
}gpio_pin;

typedef enum{
	SMP_GPIO_MODE_INPUT = 0,		 
	SMP_GPIO_MODE_OUTPUT_PP,		 
	SMP_GPIO_MODE_OUTPUT_OD,		 
	SMP_GPIO_MODE_ANALOG,			 
	SMP_GPIO_MODE_IT_RISING,		 
	SMP_GPIO_MODE_IT_FALLING,		 
	SMP_GPIO_MODE_IT_RISING_FALLING	 
}gpio_mode;

typedef enum{
	GPIO_ACTIVE_LOW = 0,	 
	GPIO_ACTIVE_HIGH,		   
	GPIO_ACTIVE_TOGGLE     
}smp_gpio_state;

typedef void(*gpio_handler_t)( void *p_context);

typedef struct{
	gpio_port			port;			 
	gpio_pin			pin;			 
	gpio_mode			mode;			 
	gpio_handler_t		gpio_handler;	 
}smp_gpio_t;
  
 
 
 
int8_t smp_gpio_init(smp_gpio_t *p_gpio);
int8_t smp_gpio_deinit(smp_gpio_t *p_gpio);
int8_t smp_gpio_set_state(smp_gpio_t *p_gpio, smp_gpio_state state);
int8_t smp_gpio_get_state(smp_gpio_t *p_gpio, smp_gpio_state *state);






 



 



 

 
#line 24 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"





typedef struct{
GPIO_TypeDef  *gpio_port;
uint16_t      gpio_pin;
}bsp_adc_init_io_config;

 








void HalBspSetGpio(GPIO_TypeDef  *GPIOx, uint16_t Pin, uint32_t mode,uint32_t pull, uint32_t speed);



#line 53 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 60 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"
				
#line 67 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"
		
#line 74 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 81 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 88 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 95 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 102 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 109 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 116 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 123 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 130 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 137 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 144 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 151 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"




#line 162 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"
										
#line 170 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"
										
#line 178 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 186 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 194 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 202 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 210 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 218 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 226 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 234 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 242 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 250 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 258 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 266 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 274 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"


 	
#line 285 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"
																							 
 







																				
 
#line 310 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

 

 









 








 



 



 





 



 
 









 
#line 368 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

 





 



 
#line 386 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

 



 
 




#line 405 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

#line 415 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

 






 



 
 



 











#line 453 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

 






 



 
 



 












#line 492 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

 






 



 
 
 
 
 












 
#line 528 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

 





 
 



 








 
#line 556 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

 





 
 



 











 
#line 587 "..\\..\\..\\BSP\\DavinciBsp_Rev1.h"

 






 																	 

















 
	    

			























																	 











																	



















 






     
#line 29 "..\\..\\..\\BSP\\Bsp.h"










     
#line 26 "..\\..\\..\\User\\main.h"

 
 
 
 
void Error_Handler(void);



 
#line 20 "..\\..\\..\\User\\main.c"
#line 1 "..\\..\\..\\APP\\AppProject.h"














 



 
#line 21 "..\\..\\..\\APP\\AppProject.h"





 



uint8_t appProjectIsSystemReadyFlag(void);
uint8_t appProjectIsInSimuMode(void);
void appProjectEnableSimuMode(void);
void appProjectDisableSimuMode(void);
uint8_t appProjectIsInEngMode(void);
void appProjectEnableEngMode(void);
void appProjectDisableEngMode(void);
uint8_t appProjectGetScuId(void);

void appProjectOpen(void);
uint8_t	appProjectIsRtcValid(void);

 





	



     


#line 21 "..\\..\\..\\User\\main.c"
#line 1 "..\\..\\..\\Lib\\LibSwTimer.h"














 








 
#line 26 "..\\..\\..\\Lib\\LibSwTimer.h"
#line 1 "..\\..\\..\\Lib\\LibDebug.h"



#line 5 "..\\..\\..\\Lib\\LibDebug.h"



 
#line 26 "..\\..\\..\\Lib\\LibDebug.h"




 

#line 27 "..\\..\\..\\Lib\\LibSwTimer.h"
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





#line 22 "..\\..\\..\\User\\main.c"
#line 1 "..\\..\\..\\HAL\\HalBsp.h"














 


 
#line 20 "..\\..\\..\\HAL\\HalBsp.h"





 

 
void HalBspInit(void);

void halBspPreDischargeRelayOn(void);
void halBspPreDischargeRelayOff(void);

void HalBslK1Ctrl(uint8_t hi);
void HalBspK2Ctrl(uint8_t hi);


void halBspFanRelayOn(void);
void halBspFanRelayOff(void);

void halBspPostiveRelayOn(void);
void halBspPostiveRelayOff(void);
void halBspNegtiveRelayOn(void);
void halBspNegtiveRelayOff(void);

void HalBspRelayPsCtrl(uint8_t hi);
void HalBspReleaseCtrl(uint8_t hi);




uint8_t HalBspGetDi1Status(void);
uint8_t HalBspGetDi2Status(void);
uint8_t HalBspGetEpoStatus(void);
uint8_t HalBspGetSpStatus(void);
uint8_t HalBspGetPs1Status(void);
uint8_t HalBspGetPs2Status(void);
uint8_t HalBspGetPs3Status(void);
uint8_t HalBspGetButtonStatus(void);
uint8_t HalBspGetK1Status(void);
uint8_t HalBspGetK2Status(void);
uint8_t HalBspGetK3Status(void);
uint8_t HalBspGetK4Status(void);
uint8_t HalBspGetDocpLatchStatus(void);
uint8_t HalBspGetCocpLatchStatus(void);
uint8_t HalBspGetOdInStatus(void);








	



     

#line 23 "..\\..\\..\\User\\main.c"
#line 1 "..\\..\\..\\API\\ApiFu.h"














 



 
#line 21 "..\\..\\..\\API\\ApiFu.h"




 
typedef void(*tApiFuCallbackFunction)(uint16_t evt, uint8_t *pMsgBuf);

 
enum{
	API_FU_FW_CHECK_INI = 0,
	API_FU_FW_CHECKING,
	API_FU_FW_CHECK_SUCCESS,
	API_FU_FW_CHECK_FAIL,
	API_FU_FW_CHECK_END
};

enum{
	API_FU_EVT_PROGRESS = 0,
	API_FU_EVT_START_FW_CHECK,
	API_FU_EVT_CHECK_RESULT,
	API_FU_EVT_FW_CHECKING,
	API_FU_EVT_CHECK_FINISH,
	API_FU_EVT_END
};

enum{
	MAGIC_CODE_FROM_APP = 0,
	MAGIC_CODE_FROM_ISP,
	MAGIC_CODE_FROM_APP_RESET,
	MAGIC_CODE_FROM_ISP_RESET,
	MAGIC_CODE_UPDATE_FW1,
	MAGIC_CODE_UPDATE_FW2,
	MAGIC_CODE_APP_NORMAL	
};

 
extern const uint8_t	SmpFwHeadInfo1[];
extern const uint8_t	SmpFwHeadInfo2[];

 
 
uint8_t	apiFuGetFwCheckStatus(void);
void apiFuStartUp(tApiFuCallbackFunction CbFunction);
void apiFuSetTotalPackageNum(uint32_t num);
void apiFuRcvSetVersion(uint32_t Version);
void apiFuRcvSetBaseAddr(uint32_t BaseAddr);
void apiFuSetUpgradeData(uint32_t addr, uint8_t *pDatBuf, uint16_t leng);

void apiFuSetMagicCode(uint8_t type);
uint8_t apiFuGetMagicModeIndex(void);
void apiFuJumpToBootloader(void);
void apiFuUpdateFw(void);
void apiFuResetAndUpdate(void);
void apiFuResetApp(void);








	



     


#line 24 "..\\..\\..\\User\\main.c"

#line 1 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"








 
  
 



 
#line 17 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"
#line 18 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"
#line 19 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"
#line 1 "..\\..\\..\\Drivers\\SMP\\smp_uart.h"








 
  
 



 
#line 17 "..\\..\\..\\Drivers\\SMP\\smp_uart.h"
#line 18 "..\\..\\..\\Drivers\\SMP\\smp_uart.h"
 
typedef enum{
	UART_DATA_READY = 0,					 
	UART_BUFFER_FULL,       			 
	UART_COMMUNICATION_ERR,				 
	UART_TX_EMPTY									 
}uart_evt_type;

typedef void(*smp_uart_event_t)(uart_evt_type p_evt);

typedef enum{
	UART0 = 0,										 
	UART1													 
}uart_module_number;

typedef enum{
	UART_FLOW_CTRL_DISABLE = 0,		 
	UART_FLOW_CTRL_ENABLE					 
}uart_flow_ctrl;

typedef enum{
	PARITY_NONE = 0,							 
	PARITY_EVEN,									 
	PARITY_ODD										 
}uart_parity;

typedef struct{
	uint8_t			*rx_buf;					 
	uint32_t		rx_buf_size;			 
	uint8_t			*tx_buf;					 
	uint32_t		tx_buf_size;			 
}uart_buffer_t;

typedef struct{
	uart_module_number	num;			 
	uint32_t			baud_rate;			 
	uart_flow_ctrl		flow_ctrl;	 
	uart_parity			use_parity;		 
	uart_buffer_t		buffers;			 
}smp_uart_t;

  







 
 
 
int8_t smp_uart_init(smp_uart_t *p_uart, smp_uart_event_t smp_uart_event_handler);
int8_t smp_uart_deinit(smp_uart_t *p_uart);
int8_t smp_uart_put(smp_uart_t *p_uart, uint8_t byte);
int8_t smp_uart_get(smp_uart_t *p_uart, uint8_t *p_byte);



 
#line 20 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"
 
typedef enum{
	BQ_GPIO          = 0,										 
	BQ_UART 							     						   
}bq796xx_io_type;

typedef enum{
	HIGH_BYTE        = 0,					 					
	LOW_BYTE  							     						
}byte_h_l;

enum{
    DIR_NORTH = 0,
    DIR_SOUTH  =1,
};

typedef enum{
	WAKE_TONE_DISABLE        = 0,
	WAKE_TONE_ENABLE
}bq796xx_wake_tone_switch;

enum {
    SINGLE_READ     = 0x80,
    SINGLE_WRITE    = 0x90,
    STACK_READ      = 0xA0,
    STACK_WRITE     = 0xB0,
    BROAD_READ      = 0xC0,
    BROAD_WRITE     = 0xD0,
    BROAD_WRITE_REVERSE = 0xE0
};

typedef enum{
    GPIO_HI_Z,
    GPIO_ADC_OTUT,
    GPIO_ADC,
    GPIO_INPUT,
    GPIO_OUT_H,
    GPIO_OUT_L,
    GPIO_ADC_PULL_H,
    GPIO_ADC_PULL_L,
}bq796xx_AFE_GPIO_Type;

typedef enum{
    AFE_GPIO1,
    AFE_GPIO2,
    AFE_GPIO3,
    AFE_GPIO4,
    AFE_GPIO5,
    AFE_GPIO6,
    AFE_GPIO7,
    AFE_GPIO8,
}bq796xx_AFE_GPIO;

typedef enum{
	SINGLE    = 0,					 					
	STACK  							     						
}bq796xx_AFE_GPIO_stack;

typedef enum{
	BQ_DISABLE   = 0,					 					
	BQ_ENABLE  							     						
}bq796xx_fun_switch;

typedef enum{
	OV_2700MV = 0x02,					 					
	OV_3500MV = 0x12,
	OV_4175MV = 0x22
}bq796xx_ov_range;

typedef enum{
	OV_STEP_0MV   = 0,					 					
	OV_STEP_25MV  ,
	OV_STEP_50MV  ,
	OV_STEP_75MV  ,
	OV_STEP_100MV ,
	OV_STEP_125MV ,
	OV_STEP_150MV ,
	OV_STEP_175MV ,
	OV_STEP_200MV ,
	OV_STEP_225MV ,
	OV_STEP_250MV ,	
	OV_STEP_275MV ,	
	OV_STEP_300MV ,	
}bq796xx_ov_step;

typedef enum{
	UV_1200MV = 0,					 					
	UV_1250MV ,
	UV_1300MV ,
	UV_1350MV ,
	UV_1400MV ,
	UV_1450MV ,
	UV_1500MV ,
	UV_1550MV ,
	UV_1600MV ,
	UV_1650MV ,
	UV_1700MV ,
	UV_1750MV ,
	UV_1800MV ,
	UV_1850MV ,	
	UV_1900MV ,
	UV_1950MV ,
	UV_2000MV ,
	UV_2050MV ,
	UV_2100MV ,
	UV_2150MV ,
	UV_2200MV ,
	UV_2250MV ,
	UV_2300MV ,
	UV_2350MV ,
	UV_2400MV ,
	UV_2450MV ,
	UV_2500MV ,
	UV_2550MV ,
	UV_2600MV ,
	UV_2650MV ,
	UV_2700MV ,
	UV_2750MV ,
	UV_2800MV ,
	UV_2850MV ,
	UV_2900MV ,
	UV_2950MV ,
	UV_3000MV ,
	UV_3050MV ,
	UV_3100MV ,
}bq796xx_uv_mv;

typedef enum{
	OT_10_PCT = 0,					 					
	OT_11_PCT ,
  OT_12_PCT ,
  OT_13_PCT ,
  OT_14_PCT ,
  OT_15_PCT ,
  OT_16_PCT ,
  OT_17_PCT ,
  OT_18_PCT ,
  OT_19_PCT ,
  OT_20_PCT ,
  OT_21_PCT ,
  OT_22_PCT ,
  OT_23_PCT ,
  OT_24_PCT ,
  OT_25_PCT ,
  OT_26_PCT ,
  OT_27_PCT ,
  OT_28_PCT ,
  OT_29_PCT ,
  OT_30_PCT ,
  OT_31_PCT ,
  OT_32_PCT ,
  OT_33_PCT ,
  OT_34_PCT ,
  OT_35_PCT ,
  OT_36_PCT ,
  OT_37_PCT ,
  OT_38_PCT ,
  OT_39_PCT ,	
}bq796xx_ot_threshold_p;

typedef enum{
	UT_66_PCT = 0,					 					
	UT_68_PCT ,
  UT_70_PCT ,
  UT_72_PCT ,
  UT_74_PCT ,
  UT_76_PCT ,
  UT_78_PCT ,
  UT_80_PCT ,
}bq796xx_ut_threshold_p;

typedef enum{
	CB_STOP = 0  ,					 					
	CB_TIME_10S  ,
	CB_TIME_30S  ,
	CB_TIME_60S  ,
	CB_TIME_300S ,
	CB_TIME_10MIN,
	CB_TIME_20MIN,
	CB_TIME_30MIN,
	CB_TIME_40MIN,
	CB_TIME_50MIN,
	CB_TIME_60MIN,
	CB_TIME_70MIN,
	CB_TIME_80MIN,
	CB_TIME_90MIN,
	CB_TIME_100MIN,
	CB_TIME_110MIN,
	CB_TIME_120MIN,
	CB_TIME_150MIN,
	CB_TIME_180MIN,
	CB_TIME_210MIN,
	CB_TIME_240MIN,
	CB_TIME_270MIN,
	CB_TIME_300MIN,
	CB_TIME_330MIN,
	CB_TIME_360MIN,
	CB_TIME_390MIN,
	CB_TIME_420MIN,
	CB_TIME_450MIN,
	CB_TIME_480MIN,
	CB_TIME_510MIN,
	CB_TIME_540MIN,
	CB_TIME_570MIN,
	CB_TIME_600MIN,
}bq796xx_cellbalance_time;

typedef enum{
	CB_CYC_T_5S = 0  ,					 					
 	CB_CYC_T_10S     ,
	CB_CYC_T_30S     ,
	CB_CYC_T_60S     ,
  CB_CYC_T_5MIN    ,
  CB_CYC_T_10MIN   ,
  CB_CYC_T_20MIN   ,
  CB_CYC_T_30MIN   ,	
}bq796xx_cellbalance_cycle_time;

typedef enum{
	CB_MANUAL = 0  ,					 					
	CB_AUTO        ,
}bq796xx_cellbalance_control;

enum {
    BQ796XX_READ_GOOD,
    BQ796XX_TYPE_FAIL,
    BQ796XX_READ_LEN_FAIL,
    BQ796XX_READ_ID_FAIL,
    BQ796XX_READ_REG_FAIL,
    BQ796XX_READ_DATA_FAIL, 
    BQ796XX_READ_CRC_FAIL,
};

typedef enum{
   BQ_EVENT_VCELL      =0  ,
	 BQ_EVENT_GPIO_ADC       ,
	 BQ_EVENT_FAULT          ,	
   BQ_EVENT_FAULTOVUV      ,
   BQ_EVENT_FAULTOTUT      ,
	 BQ_EVENT_DIR_ADDR       ,
	 BQ_EVENT_ACTIVE_CELL    ,
	 BQ_EVENT_OTHER_ERR      ,
}bq796xx_event_cb_type;

typedef enum{
    AFE_INIT_IDLE = 0,
    AFE_INIT_WAKE_UP,                               
    AFE_INIT_WAKE_UP_WAIT,                          
	  AFE_INIT_BRORAD_SET_BMU_STACK1,
	  AFE_INIT_SET_BASE1,
	  AFE_INIT_WAKE_DEV,                              
    AFE_INIT_FAULT_DET_ENABLE,                      
    AFE_INIT_WAIT_LONG,                             
    AFE_INIT_OTP_ECC_TEST_DISABLE_BASE,             
    AFE_INIT_SET_DIR,                               
    AFE_INIT_OTP_ECC_TEST_DISABLE_BROAD,            
    AFE_INIT_DUMMY_WRITE,                           

    AFE_INIT_AUTO_ADDR,                             
    AFE_INIT_SET_ID,                                
    AFE_INIT_SET_ID_WAIT,                           
    AFE_INIT_DUMMY_READ,                            
    AFE_INIT_WAIT_ID_RESPONSE,                      
	  AFE_INIT_BRORAD_SET_BMU_STACK2,
    AFE_INIT_CHECK_ID_RESPONSE,                     
    AFE_INIT_SET_STACK,                             
    AFE_INIT_SET_BASE,                              
    AFE_INIT_SET_TOP,                               

    AFE_INIT_CHECK_IS_RING,                         
    AFE_INIT_SET_DIRECTION_FLAG_ACTIVE_CELL,        

    AFE_INIT_FAULT_RESET_1,                         
    AFE_INIT_FAULT_RESET_2,
    AFE_INIT_FAULT_RESET,
    AFE_INIT_OTP_SPARE,
    AFE_INIT_BQ79600_TIMEOUT,

    AFE_INIT_SWITCH_GPIO,                            
    AFE_INIT_TSREF_ENABLE,                           

    AFE_INIT_ADC_CONF1,
    AFE_INIT_ADC_CTRL1,

    AFE_INIT_BQ796XX_TIMEOUT,
		
		AFE_INIT_MASK_FAULT_ALL,                         
		AFE_INIT_BRIDGE_FAULT_MSK,                       
		AFE_INIT_CLEAR_RST_ALL,
		AFE_INIT_SET_RUN_OVUV_FUNC,
		AFE_INIT_SET_RUN_OTUT_FUNC,
		AFE_INIT_SET_CELL_BALANCE_FUNC_DISABLE,
		AFE_INIT_SET_GPIO_FUNC,
		AFE_INIT_SET_BMU_FAULT_MSK,
		AFE_RUN_AUX_ADC,
}bq796xx_init_steps_enum;

typedef enum{
  SETDIR_BRORAD_SET_BMU_STACK1 = 0      ,	
	SETDIR_SET_BASE1                      ,
	SETDIR_BASE_DIR_CHG                   ,
	SETDIR_BRORAD_REVERSE_DIRECTION       ,
	SETDIR_BRORAD_OTP_ECC_TEST_W          ,
	SETDIR_INIT_DUMMY_WRITE               ,
	SETDIR_AUTO_ADDR                      ,
	SETDIR_SET_ID                         ,
	SETDIR_INIT_DUMMY_READ                ,
	SETDIR_BRORAD_SET_BMU_STACK2          ,
	SETDIR_CHECK_ID_RESPONSE              ,	
	SETDIR_SET_STACK                      ,
	SETDIR_SET_BASE2                      ,
	SETDIR_SET_TOP                        ,
	SETDIR_CHECK_IS_RING                  ,
	SETDIR_SET_DIRECTION_FLAG_ACTIVE_CELL ,        
	SETDIR_FAULT_RESET_2                  ,
	SETDIR_CLEAR_RST_ALL                  ,
	SETDIR_AFE_RUN_AUX_ADC                ,
}bq796xx_dir_set_steps_enum;

  






















































#line 402 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"

#line 421 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"







#line 440 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"







#line 495 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"















#line 518 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"











#line 537 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"


#line 547 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"






#line 559 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"


#line 569 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"


#line 579 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"


#line 588 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"


#line 598 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"


#line 607 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"


#line 616 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"


#line 625 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"


















































 


 
#line 691 "..\\..\\..\\Drivers\\SMP\\smp_drv_bq796xx.h"
 

typedef struct{ 
	uint16_t vcell_data[0x3F][16];
	uint16_t gpio_data[0x3F][8];  
	uint8_t  fault_summary[0x3F];
	uint16_t fault_ov[0x3F];
	uint16_t fault_uv[0x3F];
	uint8_t  fault_ot[0x3F];
	uint8_t  fault_ut[0x3F];
	uint8_t  paser_id;
  uint8_t  comm_dir;
	uint8_t  top_stack_north_id;
	uint8_t  top_stack_south_id;
	uint8_t  ns_flag;
}bq796xx_data_t;													
					
typedef struct{
    bq796xx_ov_range                  ov_threshold;
	  bq796xx_ov_step                   ov_step_threshold;
	  bq796xx_uv_mv                     uv_threshold;
	  bq796xx_ot_threshold_p            ot_threshold;
	  bq796xx_ut_threshold_p            ut_threshold;
	  bq796xx_fun_switch                ov_enable;
	  bq796xx_fun_switch                uv_enable;
		bq796xx_fun_switch                ot_enable;
	  bq796xx_fun_switch                ut_enable;
	  bq796xx_cellbalance_cycle_time    cb_cycle_time;
	  uint8_t                           bmu_total_num;
}bq796xx_init_default_t;

typedef uint8_t(*bq796xx_CB_Fun_t)(bq796xx_data_t *bq_data, bq796xx_event_cb_type bq_event);    

uint16_t smp_time_count_get(void);
void smp_time_count_set(uint16_t val);

void drv_bq796xx_delay_ms(uint32_t ms_c);
void drv_bq796xx_switch_rx_pin_type_setting(bq796xx_io_type type);													
void drv_bq796xx_rx_pin_wakeup(void);													
void drv_bq796xx_uart_puts(uint8_t *d_bytes,int16_t d_size);
void drv_bq796xx_clear_fifobuffer(void);

void drv_bq796xx_command(uint8_t cmd_type, uint8_t dev_id, uint16_t reg_addr, uint8_t datalen, uint8_t *data_array, uint32_t delayms);
void drv_bq796xx_command_framing(uint8_t cmd_type, uint8_t dev_id, uint16_t reg_addr, uint8_t datalen, uint8_t *data_array, uint32_t delayms);
	
uint8_t drv_bq796xx_init(void);
uint8_t drv_bq796xx_start_setting(uint8_t maxcnt, uint8_t dir);													
uint8_t drv_bq796xx_Read_AFE_ALL_VCELL(bq796xx_AFE_GPIO_stack is_stack,uint8_t dev_id,uint32_t delays);
uint8_t drv_bq796xx_Set_AFE_GPIO_type(bq796xx_AFE_GPIO_stack is_stack,uint8_t dev_id,bq796xx_AFE_GPIO_Type GPIO_type,bq796xx_AFE_GPIO GPIO_Num,uint32_t delays);
uint8_t drv_bq796xx_Start_AFE_ADC(bq796xx_AFE_GPIO_stack is_stack,uint8_t dev_id,uint32_t delays);
uint8_t drv_bq796xx_Stop_AFE_ADC(bq796xx_AFE_GPIO_stack is_stack,uint8_t dev_id,uint32_t delays);			
uint8_t drv_bq796xx_Read_AFE_ALL_ADC(bq796xx_AFE_GPIO_stack is_stack,uint8_t dev_id,uint32_t delays);

uint8_t drv_bq796xx_Goto_ShutDownMode(uint32_t delays);
uint8_t drv_bq796xx_Goto_SleepMode(uint32_t delays);
uint8_t drv_bq796xx_Send_WakeupAll(uint32_t delays);
uint8_t drv_bq796xx_Set_Mask_FaultAll(bq796xx_AFE_GPIO_stack is_stack, uint8_t dev_id, uint32_t delays);
uint8_t drv_bq796xx_Set_BroadCast_Mask_FaultAll(uint32_t delays);
uint8_t drv_bq796xx_Set_BroadCast_Mask_FaultSel(uint8_t msk1,uint8_t msk2,uint32_t delays);
uint8_t drv_bq796xx_Clear_FaultRstAll(uint32_t delays);

uint8_t drv_bq796xx_Clear_Mask_Fault_OVUV(bq796xx_AFE_GPIO_stack is_stack, uint8_t dev_id, uint32_t delays);
uint8_t drv_bq796xx_Clear_Mask_Fault_OTUT(bq796xx_AFE_GPIO_stack is_stack, uint8_t dev_id, uint32_t delays);

uint8_t drv_bq796xx_Read_Stack_FaultSummary(uint32_t delays);
uint8_t drv_bq796xx_Read_Base_FaultSummary(uint32_t delays);

uint8_t drv_bq796xx_Set_Stack_OV(bq796xx_ov_range ov_range_mv,bq796xx_ov_step ov_step_mv ,uint32_t delays);
uint8_t drv_bq796xx_Set_Stack_UV(bq796xx_uv_mv uv_mv,uint32_t delays);
uint8_t drv_bq796xx_Run_OVUV(uint32_t delays);
uint8_t drv_bq796xx_Read_Stack_FaultOVUV(uint32_t delays);

uint8_t drv_bq796xx_Set_Stack_OTUT(bq796xx_ot_threshold_p ot_th_pct,bq796xx_ut_threshold_p ut_th_pct,uint32_t delays);
uint8_t drv_bq796xx_Set_Stack_OTUT_Associate(bq796xx_AFE_GPIO afe_gpio,uint32_t delays);
uint8_t drv_bq796xx_Run_OTUT(uint32_t delays);
uint8_t drv_bq796xx_Read_Stack_FaultOTUT(uint32_t delays);

uint8_t drv_bq796xx_Set_Stack_CellBalanceTime(bq796xx_cellbalance_time cb_time,uint32_t delays);
uint8_t drv_bq796xx_Set_Stack_CellBalanceCycleTime(bq796xx_cellbalance_cycle_time cb_cyc_time,uint32_t delays);
uint8_t drv_bq796xx_Stack_CellBalanceStarting(bq796xx_cellbalance_control cb_ctrl, uint32_t delays);

void drv_bq796xx_uart_event_handler(uart_evt_type p_evt);
uint8_t drv_bq796xx_data_frame_parser(void);													
uint8_t drv_bq796xx_check_respone_event(void);


int8_t bq796xx_event_RegisteCB(bq796xx_CB_Fun_t callbackfunc);
int8_t bq796xx_event_UnregisteCB(void);

uint8_t drv_bq796xx_Init_Steps(bq796xx_wake_tone_switch wake_tone_sw, bq796xx_init_steps_enum *afe_phase,uint8_t maxcnt, uint8_t dir, uint8_t *step_complete_f, uint8_t *before_delay_ms);

int8_t drv_bq796xx_init_default_load(bq796xx_init_default_t in_load_data);

uint8_t drv_bq796xx_CellBalance_1to8_set(uint8_t bmu_id, uint8_t cell_d_en, bq796xx_cellbalance_time cb_time, uint32_t delays);
uint8_t drv_bq796xx_CellBalance_9to16_set(uint8_t bmu_id, uint8_t cell_d_en, bq796xx_cellbalance_time cb_time, uint32_t delays);
uint8_t drv_bq796xx_Stack_CellBalance_1to8_clear(bq796xx_AFE_GPIO_stack is_stack, uint8_t dev_id, uint32_t delays);
uint8_t drv_bq796xx_Stack_CellBalance_9to16_clear(bq796xx_AFE_GPIO_stack is_stack, uint8_t dev_id, uint32_t delays);
uint8_t drv_bq796xx_direction_set_steps(uint8_t ns_dir_bmu_cnt,bq796xx_dir_set_steps_enum *afe_phase,uint8_t maxcnt, uint8_t dir, uint8_t *step_complete_f, uint8_t *before_delay_ms);

void fill_data4_payload(uint8_t *payload,uint8_t b1,uint8_t b2,uint8_t b3,uint8_t b4);
													
extern uint8_t bq796xx_res_buf[(128+6)];													
extern uint16_t bq796xx_res_buf_c;
													


 
#line 26 "..\\..\\..\\User\\main.c"
#line 1 "..\\..\\..\\Drivers\\SMP\\smp_adc.h"















 
 



 
#line 23 "..\\..\\..\\Drivers\\SMP\\smp_adc.h"
#line 24 "..\\..\\..\\Drivers\\SMP\\smp_adc.h"
#line 1 "..\\..\\..\\BSP\\BSP.h"














 



	
#line 38 "..\\..\\..\\BSP\\BSP.h"

     
#line 25 "..\\..\\..\\Drivers\\SMP\\smp_adc.h"

 






#line 39 "..\\..\\..\\Drivers\\SMP\\smp_adc.h"
 

 
 


#line 52 "..\\..\\..\\Drivers\\SMP\\smp_adc.h"

 
 

typedef enum{
	adc1 = 0,
	adc2,
	adc3, 	
}adc_module_number;

  
extern volatile uint16_t SMPADCVALUE1[16];
extern ADC_HandleTypeDef smp_adc_1;
extern DMA_HandleTypeDef smp_dma_adc1;
 
 


 
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void Enable_DMA_Control_Clock(void);
void smp_adc_gpio_init(ADC_HandleTypeDef* hadc);
void smp_adc_deinit(adc_module_number num);
int8_t smp_adc_adc_para_init(adc_module_number num);

int8_t hal_internal_adc_get(uint16_t *adc_data,adc_module_number num, bsp_adc_init_io_config adc_confing);
uint32_t GetADCChParaSTM32L496(adc_module_number num, GPIO_TypeDef *Port, uint16_t Pin);




 


#line 27 "..\\..\\..\\User\\main.c"
#line 1 "..\\..\\..\\Drivers\\SMP\\smp_ADS7946_Driver.h"








 
	
#line 12 "..\\..\\..\\Drivers\\SMP\\smp_ADS7946_Driver.h"
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



 

#line 13 "..\\..\\..\\Drivers\\SMP\\smp_ADS7946_Driver.h"
#line 1 "..\\..\\..\\Drivers\\SMP\\smp_spi_DMA.h"








 
  
 



 
#line 17 "..\\..\\..\\Drivers\\SMP\\smp_spi_DMA.h"
#line 18 "..\\..\\..\\Drivers\\SMP\\smp_spi_DMA.h"
#line 19 "..\\..\\..\\Drivers\\SMP\\smp_spi_DMA.h"
 
extern SPI_HandleTypeDef smp_spi1_handle;
extern SPI_HandleTypeDef smp_spi2_handle;
extern SPI_HandleTypeDef smp_spi3_handle;

typedef enum{
	SPI_module1 = 0,											 
	SPI_module2,													 
	SPI_module3														 
}spi_module_number;

typedef enum{
	SPI_mode0 = 0,												 
	SPI_mode1,														 
	SPI_mode2,														 
	SPI_mode3															 
}spi_mode_number;

typedef struct{
	spi_module_number		num;					 
	spi_mode_number			mode;					
}smp_spi_t;

typedef struct{
	spi_module_number		spi_num;					 
	smp_gpio_t					cs_handler;
}smp_spi_cs_t;



typedef enum{
	SMP_SPI_EVENT_DONE = 0,		 
	SMP_SPI_EVENT_TRANSFER_BUSY,
	SMP_SPI_EVENT_TRANSFERR_ERROR
}smp_spi_evt_type;
typedef void (*smp_spi_event_t)(smp_spi_evt_type p_evt);
  
extern uint8_t uSPIFlag[3];
 
 
 

int8_t smp_spi_get_status(smp_spi_t *spi);
int8_t smp_spi_master_cs_init(smp_spi_cs_t *p_cs);
int8_t smp_spi_master_cs_deinit(smp_spi_cs_t *p_cs);
int8_t smp_spi_master_cs_set(smp_spi_cs_t *p_cs, uint8_t status);
int8_t smp_spi_master_init(smp_spi_t *p_spi, smp_spi_event_t smp_spi_event_handler, const _Bool lsb);
int8_t smp_spi_master_deinit(smp_spi_t *p_spi);
int8_t smp_spi_master_send_recv(smp_spi_t *spi, uint8_t *tx_data,uint16_t tx_size, uint8_t *rx_data, uint16_t rx_size,smp_spi_cs_t *p_cs);
int8_t smp_spi_master_send_recv_blocking(smp_spi_t *spi, uint8_t *tx_data,uint16_t tx_size, uint8_t *rx_data, uint16_t rx_size,smp_spi_cs_t *p_cs);


 
#line 14 "..\\..\\..\\Drivers\\SMP\\smp_ADS7946_Driver.h"



typedef enum{
	channel_0 = 0,												
	channel_1						
}smp_ADS7946_channel_num;
	
typedef enum{
	CS_0 = 0,												
	CS_1						
}smp_ADS7946_CS_num;

typedef struct{
	smp_ADS7946_channel_num channel_sel;
	smp_spi_cs_t ADS7946_CS0;
}smp_ADS7946_control;
	
typedef void(*ads7946_CB_Fun_t)(uint8_t *pDatBuf,uint8_t bufsize);


int8_t smp_ADS7946_init(void);
int8_t smp_ADS7946_deinit(void);
int8_t smp_ADS7946_get_data(smp_ADS7946_channel_num channel_sel,smp_ADS7946_CS_num CS,ads7946_CB_Fun_t evt_callback);

#line 28 "..\\..\\..\\User\\main.c"
#line 1 "..\\..\\..\\API\\ApiIRMonitoring.h"














 



 
#line 21 "..\\..\\..\\API\\ApiIRMonitoring.h"




 
typedef struct{ 
	float Vo_stack;
	float V_stack;
	float Vo_n;
	float Vo_en;
	float Vn;
	float Vp;
	
	float Vo_n_l;
	float Vn_l;
	float Vp_l;
	
	uint8_t  exe_interval_s;
	uint16_t sw_delay_ms;
}IRMonitoring_Data_t;

typedef struct{ 
	uint16_t Rp_kohm;
	uint16_t Rn_kohm;
	float V_stack;   
}IRMonitoring_Resistor_t;		

typedef enum{
	 IRM_EVENT_BALANCE     =0 ,
	 IRM_EVENT_UNBALANCE      ,
   IRM_EVENT_GET_VSTACK     ,	
	 IRM_EVENT_OTHER_ERR      ,
}IRMonitoring_event_cb_type;


typedef enum{
	 IRM_BUSY              =0 ,
	 IRM_OK                   ,
	 IRM_OTHER_ERR            ,
}IRMonitoring_event_read_cb_type;

typedef enum{
	 IRM_STEP_OK                  =0       ,
	 IRM_STEP_READ_TIMEOUT                 ,	
	 IRM_STEP_DATA_READY_TIMEOUT           ,	 
	 IRM_STEP_OUT_VSTACK_VAL               ,
	 IRM_STEP_OTHER_ERR                    ,
}IRMonitoring_step_ret_type;

typedef enum{
    IRM_S1             = 0          ,
	  IRM_S2                          ,
	  IRM_S3                          ,
	  IRM_S4                          ,	
	  IRM_S5                          ,
	  IRM_S6                          ,
		IRM_IO_WAITTING                 ,
	  IRM_DEVICE_WAITTING             ,
	  IRM_DATAREADY_WAITTING          ,
	  IRM_FINISH                      ,
}IRMonitoring_steps_enum;

typedef enum{
    IRM_SW_OFF         = 0          ,
	  IRM_SW_ON                       ,
}IRMonitoring_SW_enum;

typedef uint8_t(*apiIRMonitoring_CB_Fun_t)(IRMonitoring_Resistor_t *irm_res_data, IRMonitoring_event_cb_type irm_event);    

typedef void(*IRM_Recv_CB_t)(float *pDatBuf);

typedef struct{
	 void  (*SW_gpioinit_cb)(void);
   void  (*SW_gpio_crtl_cb[3])(IRMonitoring_SW_enum on_off);
	 void  (*GetVoltDeviceInit_cb)(void);
	 IRMonitoring_event_read_cb_type (*TriggerData_cb)(void);                                 
   apiIRMonitoring_CB_Fun_t irm_outdata;
	 void (*DataReady_cb)(IRM_Recv_CB_t rcv_cb); 
}apiIRMonitoring_cb_t;	

 
























 














 

 
IRMonitoring_Resistor_t apiIRMonitoringGetResistor(void);

uint8_t apiIRMonitoringOpen(uint8_t exe_interval_s, uint16_t sw_delay_ms, apiIRMonitoring_cb_t callbackfunc);
void apiIRMonitoringGetVstack(void);







     
#line 29 "..\\..\\..\\User\\main.c"
#line 1 "..\\..\\..\\HAL\\HalAfeADS7946.h"














 



 
#line 21 "..\\..\\..\\HAL\\HalAfeADS7946.h"






 
 
	
 


 
extern int32_t	ads_7945_adcValue[4];

 
void HalAfeCurrentSetAdcValue(uint8_t adc_index, int32_t adcvalue);

void halAfeCurrentOpen(void);








     
#line 30 "..\\..\\..\\User\\main.c"

#line 1 "..\\..\\..\\API\\ApiModbusTCPIP.h"















 



#line 1 "..\\..\\..\\Drivers\\SMP\\smp_w5500_DMA.h"















 
  



#line 1 "..\\..\\..\\Drivers\\W5500\\wizchip_conf.h"



















































 








#line 62 "..\\..\\..\\Drivers\\W5500\\wizchip_conf.h"




 


























#line 137 "..\\..\\..\\Drivers\\W5500\\wizchip_conf.h"
  












 

   



   typedef   uint8_t   iodata_t;
#line 1 "..\\..\\..\\Drivers\\W5500\\w5500.h"






















































#line 56 "..\\..\\..\\Drivers\\W5500\\w5500.h"
#line 1 "..\\..\\..\\Drivers\\W5500\\wizchip_conf.h"



















































 

#line 57 "..\\..\\..\\Drivers\\W5500\\w5500.h"



































 
 
 








 









































 


















 
 
  
 











 
 
 
 
 



 





 





 
 
















 






 






 






 






 






 
















 

















 









 








 









 









 








 






 






 






 






 








 








 






 

















 

































 


















 

















 






















 







 







 









 









 






 









 






 


















 










 










 











 













 








 












 








 









 






 












 







 



 










 







 







 







 


 



 






 





 





 



 
#line 782 "..\\..\\..\\Drivers\\W5500\\w5500.h"

 




 






 






 






 


 







 








 









 







 






 







 





 





 


 









 








 







 


 





 


 


 




 



 











 










 











 












 





 







 









 







 







 


 



 





 





 





 





 


 




 







 







 








 







 








 







 







 







 







 






 







 









 




 
#line 1170 "..\\..\\..\\Drivers\\W5500\\w5500.h"












 
















 












 
uint8_t  WIZCHIP_READ (uint32_t AddrSel);







 
void     WIZCHIP_WRITE(uint32_t AddrSel, uint8_t wb );







 
void     WIZCHIP_READ_BUF (uint32_t AddrSel, uint8_t* pBuf, uint16_t len);







 
void     WIZCHIP_WRITE_BUF(uint32_t AddrSel, uint8_t* pBuf, uint16_t len);









 









 








 








 








 









 








 








 








 








 








 











 




 








 








 







 








 








 








 







 








 








 










 




 









 








 










 








 








 








 








 








 








 










 





 








 










 




 







 




 







 




 








 








 







 














 









 









 









 









 









 









 









 








 









 











 




 









 









 









 









 









 











 




 









 











 




 









 









 









 










 










 










 









 









 








 
uint16_t getSn_TX_FSR(uint8_t sn);






 




 









 











 




 









 
uint16_t getSn_RX_RSR(uint8_t sn);








 











 




 		








 




 		









 











 




 		









 









 













 




 		








 




 		
















 
void wiz_send_data(uint8_t sn, uint8_t *wizdata, uint16_t len);














 
void wiz_recv_data(uint8_t sn, uint8_t *wizdata, uint16_t len);







 
void wiz_recv_ignore(uint8_t sn, uint16_t len);









#line 158 "..\\..\\..\\Drivers\\W5500\\wizchip_conf.h"
#line 189 "..\\..\\..\\Drivers\\W5500\\wizchip_conf.h"










 
#line 206 "..\\..\\..\\Drivers\\W5500\\wizchip_conf.h"






















 



 
typedef struct __WIZCHIP
{
   uint16_t  if_mode;               
   uint8_t   id[8];                 
   

 
   struct _CRIS
   {
      void (*_enter)  (void);       
      void (*_exit) (void);         
   }CRIS;  
   

 
   struct _CS
   {
      void (*_select)  (void);      
      void (*_deselect)(void);      
   }CS;  
   

 
   union _IF
   {	 
      

 
      
      
      
      
      
      
      struct
      {
         iodata_t  (*_read_data)   (uint32_t AddrSel);
         void      (*_write_data)  (uint32_t AddrSel, iodata_t wb);
      }BUS;      

      

 
      struct
      {
         uint8_t (*_read_byte)   (void);
         void    (*_write_byte)  (uint8_t wb);
         void    (*_read_burst)  (uint8_t* pBuf, uint16_t len);
         void    (*_write_burst) (uint8_t* pBuf, uint16_t len);
      }SPI;
      
      
   }IF;
}_WIZCHIP;

extern _WIZCHIP  WIZCHIP;




 
typedef enum
{
   CW_RESET_WIZCHIP,   
   CW_INIT_WIZCHIP,    
   CW_GET_INTERRUPT,   
   CW_CLR_INTERRUPT,   
   CW_SET_INTRMASK,    
   CW_GET_INTRMASK,    
   CW_SET_INTRTIME,    
   CW_GET_INTRTIME,    
   CW_GET_ID,          



   CW_RESET_PHY,       
   CW_SET_PHYCONF,     
   CW_GET_PHYCONF,     
   CW_GET_PHYSTATUS,   
   CW_SET_PHYPOWMODE,  



   CW_GET_PHYPOWMODE,  
   CW_GET_PHYLINK      

}ctlwizchip_type;




 
typedef enum
{
   CN_SET_NETINFO,  
   CN_GET_NETINFO,  
   CN_SET_NETMODE,  
   CN_GET_NETMODE,  
   CN_SET_TIMEOUT,  
   CN_GET_TIMEOUT,  
}ctlnetwork_type;






 
typedef enum
{

   IK_WOL               = (1 << 4),   




   IK_PPPOE_TERMINATED  = (1 << 5),   


   IK_DEST_UNREACH      = (1 << 6),   


   IK_IP_CONFLICT       = (1 << 7),   

   IK_SOCK_0            = (1 << 8),   
   IK_SOCK_1            = (1 << 9),   
   IK_SOCK_2            = (1 << 10),  
   IK_SOCK_3            = (1 << 11),  

   IK_SOCK_4            = (1 << 12),  
   IK_SOCK_5            = (1 << 13),  
   IK_SOCK_6            = (1 << 14),  
   IK_SOCK_7            = (1 << 15),  



   IK_SOCK_ALL          = (0xFF << 8) 



}intr_kind;

#line 387 "..\\..\\..\\Drivers\\W5500\\wizchip_conf.h"








 
typedef struct wiz_PhyConf_t
{
      uint8_t by;       
      uint8_t mode;     
      uint8_t speed;    
      uint8_t duplex;   
      
      
   }wiz_PhyConf;





 
typedef enum
{
   NETINFO_STATIC = 1,    
   NETINFO_DHCP           
}dhcp_mode;




 
typedef struct wiz_NetInfo_t
{
   uint8_t mac[6];  
   uint8_t ip[4];   
   uint8_t sn[4];   
   uint8_t gw[4];   
   uint8_t dns[4];  
   dhcp_mode dhcp;  
}wiz_NetInfo;




 
typedef enum
{

   NM_FORCEARP    = (1<<1),  

   NM_WAKEONLAN   = (1<<5),  
   NM_PINGBLOCK   = (1<<4),  
   NM_PPPOE       = (1<<3),  
}netmode_type;




 
typedef struct wiz_NetTimeout_t
{
   uint8_t  retry_cnt;     
   uint16_t time_100us;    
}wiz_NetTimeout;








 
void reg_wizchip_cris_cbfunc(void(*cris_en)(void), void(*cris_ex)(void));








 
void reg_wizchip_cs_cbfunc(void(*cs_sel)(void), void(*cs_desel)(void));








 


void reg_wizchip_bus_cbfunc(iodata_t (*bus_rb)(uint32_t addr), void (*bus_wb)(uint32_t addr, iodata_t wb));








 
void reg_wizchip_spi_cbfunc(uint8_t (*spi_rb)(void), void (*spi_wb)(uint8_t wb));








 
void reg_wizchip_spiburst_cbfunc(void (*spi_rb)(uint8_t* pBuf, uint16_t len), void (*spi_wb)(uint8_t* pBuf, uint16_t len));










           
int8_t ctlwizchip(ctlwizchip_type cwtype, void* arg);









           
int8_t ctlnetwork(ctlnetwork_type cntype, void* arg);





 
 



  
void   wizchip_sw_reset(void);








 
int8_t wizchip_init(uint8_t* txsize, uint8_t* rxsize);





 
void wizchip_clrinterrupt(intr_kind intr);





 
intr_kind wizchip_getinterrupt(void);





 
void wizchip_setinterruptmask(intr_kind intr);





 
intr_kind wizchip_getinterruptmask(void);



   int8_t wizphy_getphylink(void);              
   int8_t wizphy_getphypmode(void);             



   void   wizphy_reset(void);                   




 
   void   wizphy_setphyconf(wiz_PhyConf* phyconf);  
 



 
   void   wizphy_getphyconf(wiz_PhyConf* phyconf); 
 



  
   void   wizphy_getphystat(wiz_PhyConf* phyconf);
 



    
   int8_t wizphy_setphypmode(uint8_t pmode);    






 
void wizchip_setnetinfo(wiz_NetInfo* pnetinfo);





 
void wizchip_getnetinfo(wiz_NetInfo* pnetinfo);





 
int8_t wizchip_setnetmode(netmode_type netmode);





 
netmode_type wizchip_getnetmode(void);






 
void wizchip_settimeout(wiz_NetTimeout* nettime);






 
void wizchip_gettimeout(wiz_NetTimeout* nettime);




#line 22 "..\\..\\..\\Drivers\\SMP\\smp_w5500_DMA.h"
#line 1 "..\\..\\..\\Drivers\\W5500\\socket.h"



















































































 






#line 92 "..\\..\\..\\Drivers\\W5500\\socket.h"







#line 113 "..\\..\\..\\Drivers\\W5500\\socket.h"





 





#line 130 "..\\..\\..\\Drivers\\W5500\\socket.h"










 























 
int8_t  socket(uint8_t sn, uint8_t protocol, uint16_t port, uint8_t flag);










 
int8_t  close(uint8_t sn);











 
int8_t  listen(uint8_t sn);





















 
int8_t  connect(uint8_t sn, uint8_t * addr, uint16_t port);















 
int8_t  disconnect(uint8_t sn);


















 
int32_t send(uint8_t sn, uint8_t * buf, uint16_t len);




















 
int32_t recv(uint8_t sn, uint8_t * buf, uint16_t len);


























 
int32_t sendto(uint8_t sn, uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t port);




























 
int32_t recvfrom(uint8_t sn, uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t *port);










 





 
typedef enum
{
   SIK_CONNECTED     = (1 << 0),    
   SIK_DISCONNECTED  = (1 << 1),    
   SIK_RECEIVED      = (1 << 2),    
   SIK_TIMEOUT       = (1 << 3),    
   SIK_SENT          = (1 << 4),    
   
   
   SIK_ALL           = 0x1F         
}sockint_kind;




 
typedef enum
{
   CS_SET_IOMODE,          
   CS_GET_IOMODE,          
   CS_GET_MAXTXBUF,        
   CS_GET_MAXRXBUF,        
   CS_CLR_INTERRUPT,       
   CS_GET_INTERRUPT,       

   CS_SET_INTMASK,         
   CS_GET_INTMASK          

}ctlsock_type;





  
typedef enum
{
   SO_FLAG,           
   SO_TTL,              
   SO_TOS,              
   SO_MSS,              
   SO_DESTIP,           
   SO_DESTPORT,         

   SO_KEEPALIVESEND,    

      SO_KEEPALIVEAUTO, 


   SO_SENDBUF,          
   SO_RECVBUF,          
   SO_STATUS,           
   SO_REMAINSIZE,       
   SO_PACKINFO          
}sockopt_type;

















 
int8_t  ctlsocket(uint8_t sn, ctlsock_type cstype, void* arg);


























 
int8_t  setsockopt(uint8_t sn, sockopt_type sotype, void* arg);
































 
int8_t  getsockopt(uint8_t sn, sockopt_type sotype, void* arg);





#line 23 "..\\..\\..\\Drivers\\SMP\\smp_w5500_DMA.h"
#line 24 "..\\..\\..\\Drivers\\SMP\\smp_w5500_DMA.h"
#line 25 "..\\..\\..\\Drivers\\SMP\\smp_w5500_DMA.h"
#line 26 "..\\..\\..\\Drivers\\SMP\\smp_w5500_DMA.h"
#line 27 "..\\..\\..\\Drivers\\SMP\\smp_w5500_DMA.h"
#line 28 "..\\..\\..\\Drivers\\SMP\\smp_w5500_DMA.h"
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



 

#line 29 "..\\..\\..\\Drivers\\SMP\\smp_w5500_DMA.h"
#line 30 "..\\..\\..\\Drivers\\SMP\\smp_w5500_DMA.h"












 





#line 57 "..\\..\\..\\Drivers\\SMP\\smp_w5500_DMA.h"
 
#line 69 "..\\..\\..\\Drivers\\SMP\\smp_w5500_DMA.h"



typedef struct{
	uint32_t AddrSel;
	uint16_t Len;
	uint8_t *pData;
}R_W_NetInfo;

typedef enum{
	W5500_DATA_RECV = 0,
	W5500_Socket_REG_Success,
	W5500_COMMUNICATE_ERR
}W5500_cb_type;

typedef struct{
	uint8_t					*rx_buf_Ptr;
	uint16_t				rx_buf_size;			 
	uint8_t					*tx_buf_Ptr;
	uint16_t				tx_buf_size;			 
}w5500_socket_buffer_t;

typedef struct{
	uint8_t					Num;
	uint8_t					Protocol;	
	uint16_t 				PortNum;
	uint8_t					DeviceID;
	w5500_socket_buffer_t 	Memory;
	uint16_t				DestPort;
}W5500_Socket_parm;

typedef uint16_t (*smp_w5500_event_t)(W5500_cb_type p_W5500event, uint16_t DataLen);

typedef struct{
	uint8_t SocketStatus;
	smp_w5500_event_t cbFunPtr;	
	W5500_Socket_parm Parm;

}W5500_RegisterList;	
 
typedef enum{
	Init_HW_Reset_1,
	Init_HW_Reset_2,
	Init_SW_Reset,
	Init_PHY_Conf,
	Init_PHY_Read,
	Init_PHY_Rst,
	Init_PHY_Verify_Read,
	Init_PHY_Verify,
	Init_SET_NetInfo,
	Init_Read_NetInfo,
	Init_Verify_NetInfo,
	Init_SET_SUBMask,
	Init_SET_LocalIP,
	Init_INT_MASK_Config,
	Init_SocketNum_INT_MASK_Config,
	Init_SocketEvent_INT_MASK_Config,
	Init_End
}W5500_init_step;
 
 
typedef enum{
	 
	Server_Read_Socket_Status,
	Server_Check_Socket_Status,
	Socket_Open_Read_IP,
	Socket_Open_Check_IP,
	Set_Socket_Close,
	Read_Cmd_Register_Status_0,
	Check_Close_Cmd_Recv,
	RST_All_INT_Flag,
	Read_Socket_Status_0,
	Verify_Socket_Close_0,
	Set_Socket_Mode,
	Set_Socket_Port_Highbyte,
	Set_Socket_Port_Lowbyte,
	Set_Socket_Open,
	Read_Cmd_Register_Status_1,
	Check_Open_Cmd_Recv,
	Read_Socket_Status_1,
	Verify_Socket_Close_1,
	 	
	Socket_Listen_Read_Mode,
	Socket_Listen_Verify_Mode,
	Set_Socket_Listen,
	Read_Cmd_Register_Status_2,
	Check_Listen_Cmd_Recv,
	Read_Socket_Status_2,
	Verify_Socket_Listen,
	 		
	Read_Socket_INT_event,
	Verify_Connect_Event,
	Clear_CON_INT_Flag,
	Read_Remote_IP,
	Read_Remote_Port_HighByte,
	Save_High_Byte_Read_Remote_Port_LowByte,
	Save_Port_Low_Byte,
	Read_Recv_Cnt_HighByte,
	Read_Recv_Cnt_LowByte,
	Check_Recv_Data,
	Read_Buf_Max_Len,
	Check_Oversize_Event,
	Read_Socket_Buf_Addr_HighByte,
	Read_Socket_Buf_Addr_LowByte,
	Read_Socket_Buf_data,
	ParserData_and_Update_Buf_Offset_Highbyte,
	Update_Buf_Offset_Lowbyte,
	Set_Scoket_Recv,
	Read_Cmd_Register_Status_3,
	Check_Recv_Cmd_Recv,
	Clear_Recv_INT_Flag,
	Read_Tx_Buf_Max_Len,
	Check_Response_Oversize,
	Read_W5500_Tx_FSR_High,
	Read_W5500_Tx_FSR_Low,
	Read_W5500_Tx_Ptr_Highbyte,
	Read_W5500_Tx_Ptr_Lowbyte,
	Get_W5500_Tx_Ptr_Addr_and_Trans,
	Update_Tx_Ptr_Highbyte,
	Update_Tx_Ptr_Lowbyte,
	Set_Socket_Send,
	Read_Cmd_Register_Status_4,
	Check_Send_Cmd_Recv,
	Read_Socket_INT_event_1,
	Clear_SendOK_INT_Flag,
	Verify_Send_End,
	 		
	Set_Socket_Disconnect,
	Read_Cmd_Register_Status_5,
	Check_Discon_Cmd_Recv,
	Read_Socket_INT_event_2,
	Verify_Disconnct_End,
	Clear_DISCON_INT_Flag,
	Server_End
}W5500_server_step;

 
enum{
	W5500_SPI_Idle = 0,
	W5500_SPI_Busy,
	W5500_SPI_Done
};

enum{
	Socket_Disable = 0,
	Socket_Enable
};
enum{
	Event_Handle_Done,
	Event_Handle_Ing
};

int8_t W5500_Init_Step(uint8_t Step);
int8_t W5500_Server_Step(uint8_t Step, uint8_t SocketNum);
int8_t W5500_Socket_Register(W5500_Socket_parm *parm, smp_w5500_event_t w5500_event_Handler);

void Hal_W5500_Open(void);

extern smp_gpio_t		PB12;
extern smp_spi_cs_t 	W5500_CS;
extern smp_spi_t 		SPI_W5500;
extern _Bool W5500INT_Low;
#line 21 "..\\..\\..\\API\\ApiModbusTCPIP.h"


 






 








 
#line 48 "..\\..\\..\\API\\ApiModbusTCPIP.h"
 


 











 




enum{
	HIGH_Byte = 0,
	LOW_Byte
};

enum{
    Normal = 0,
    Illegal_Function,
    Illegal_DataAddr,
    Illegal_DataValue,
    Slave_DeviceFail,
    Acknowledge,
    Slave_Device_Busy,
    Negative_Acknowledge,
    Memory_Parity_Error,
    Gateway_Path_Unavail = 10,
    Gateway_Target_ResponseFail
};

void Modbus_TCPIP_Socket_Open(void);
uint16_t Modbus_TCPIP_Parser(W5500_Socket_parm Socket, uint16_t DataLen);
uint8_t ModbusTCPIP_Func_ReadHoldReg( uint16_t xu16StartRegAddr, uint16_t xu16ReadRegCnt, W5500_Socket_parm Socket);
uint8_t ModbusTCPIP_Func_WriteMultiReg(uint16_t xu16StartRegAddr, uint16_t xu16WriteRegCnt, W5500_Socket_parm Socket);
extern uint8_t ModbusTCPIP_buffer[2000];
#line 32 "..\\..\\..\\User\\main.c"
#line 1 "..\\..\\..\\APP\\AppProjectTest.h"



#line 1 "..\\..\\..\\HAL\\halTimer.h"














 



 
#line 21 "..\\..\\..\\HAL\\halTimer.h"
#line 22 "..\\..\\..\\HAL\\halTimer.h"
#line 23 "..\\..\\..\\HAL\\halTimer.h"






 

 
typedef struct{
	uint8_t		TimerNo;
	uint32_t	IntervalUs;
}tHalTimer;



 
 
 
int8_t HalTimerOpen(tHalTimer *pHalTimer, tLibRegisterEvtHandler evtHandler);
int8_t HalSpiClose(tHalTimer *pHalTimer);




     


#line 5 "..\\..\\..\\APP\\AppProjectTest.h"
#line 6 "..\\..\\..\\APP\\AppProjectTest.h"
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




#line 7 "..\\..\\..\\APP\\AppProjectTest.h"
#line 8 "..\\..\\..\\APP\\AppProjectTest.h"
#line 1 "..\\..\\..\\Drivers\\SMP\\smp_ADS7946_Driver.h"








 
	
#line 12 "..\\..\\..\\Drivers\\SMP\\smp_ADS7946_Driver.h"
#line 13 "..\\..\\..\\Drivers\\SMP\\smp_ADS7946_Driver.h"
#line 14 "..\\..\\..\\Drivers\\SMP\\smp_ADS7946_Driver.h"

#line 9 "..\\..\\..\\APP\\AppProjectTest.h"

void appTestProjectOpen(void);


#line 33 "..\\..\\..\\User\\main.c"
#line 1 "..\\..\\..\\Drivers\\SMP\\smp_MX25L_Driver.h"



#line 5 "..\\..\\..\\Drivers\\SMP\\smp_MX25L_Driver.h"
#line 6 "..\\..\\..\\Drivers\\SMP\\smp_MX25L_Driver.h"
#line 1 "..\\..\\..\\Lib\\SMP\\smp_fifo_flash.h"








 
  
 



 
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





 



 



 

 
#line 7 "..\\..\\..\\Drivers\\SMP\\smp_MX25L_Driver.h"

 




typedef struct{
	smp_flash_package		*rx_buf;				 
	uint32_t				rx_buf_size;			 
	
	
}flash_buffer_t;

typedef struct{
	flash_buffer_t		buffers;			
}smp_flash_t;

typedef struct{
	uint8_t Manufacture_ID;
	uint16_t Device_ID;
}smp_mx25l_ID;






typedef struct{
	uint8_t status1;
	uint8_t status2;
}smp_mx25l_status;




typedef struct{
	uint8_t config1;
	uint8_t config2;
}smp_mx25l_config;

int8_t smp_mx25l_flash_init(void);
int8_t smp_mx25l_flash_write_enable(void);
int8_t smp_mx25l_flash_write_disable(void);
int8_t smp_mx25l_flash_read_ID(smp_mx25l_ID *mx251_ID);
int8_t smp_mx25l_flash_read_status(smp_mx25l_status *mx251_status);
int8_t smp_mx25l_flash_read_configuration(smp_mx25l_config *mx251_config);
int8_t smp_mx25l_flash_write_status(smp_mx25l_status *mx251_status,smp_mx25l_config *mx251_config);
int8_t smp_mx25l_flash_read_data_bytes(uint8_t *flash_addr,uint16_t read_byte_num,smp_flash_event_t smp_flash_event_handle);
int8_t smp_mx25l_flash_fast_read_data_bytes_addr(uint8_t *flash_addr ,uint8_t *buffer, uint16_t read_byte_num , smp_flash_event_t smp_flash_event_handle);
int8_t smp_mx25l_flash_fast_read_data_bytes_page(uint16_t page, uint8_t *buffer,uint16_t read_byte_num , smp_flash_event_t smp_flash_event_handle);
int8_t smp_mx25l_flash_sector_erase(uint8_t *flash_addr,smp_flash_event_t smp_flash_event_handle);
int8_t smp_mx25l_flash_sector_erase_sectornum(uint16_t sector_num , smp_flash_event_t smp_flash_event_handle);
int8_t smp_mx25l_flash_block_erase(uint8_t *flash_addr,smp_flash_event_t smp_flash_event_handle);
int8_t smp_mx25l_flash_chip_erase(smp_flash_event_t smp_flash_event_handle);
int8_t smp_mx25l_flash_page_program(uint16_t page,uint8_t *buffer,uint16_t write_byte_num,smp_flash_event_t smp_flash_event_handle);
int8_t smp_mx25l_flash_deep_power_down(void);
int8_t smp_mx25l_flash_release_deep_power_down(void);
void MX25L_SPI_send_command(void);


#line 34 "..\\..\\..\\User\\main.c"
#line 1 "..\\..\\..\\Drivers\\SMP\\smp_max7219.h"








 

#line 14 "..\\..\\..\\Drivers\\SMP\\smp_max7219.h"


















void maxPowerUp(uint8_t bPowerUp);




void maxSetIntensity(uint8_t bIntensity);



void maxSetSegmentMode(uint8_t bMode);





void maxSendImage(uint8_t *pImage, int iPitch);





void maxSetTestMode(uint8_t bOn);




void maxSetLimit(uint8_t bLimit);



void maxSegmentString(char *pString);





void maxDrawString(char *pString, uint8_t *pImage, uint8_t iPitch, uint8_t bSmall);






void maxScrollBitmap(uint8_t *pBitmap, int iPitch, int iScroll);




int maxInit(uint8_t iNum, uint8_t bDecodeMode);



void maxShutdown(void);

void MAX7219_All_Display(uint8_t data);

#line 35 "..\\..\\..\\User\\main.c"
#line 1 "..\\..\\..\\APP\\smp_log_managment.h"














 



#line 20 "..\\..\\..\\APP\\smp_log_managment.h"
#line 21 "..\\..\\..\\APP\\smp_log_managment.h"
#line 1 "..\\..\\..\\Drivers\\BSP\\STM32L496V_Davinci\\stm32l4xx_Davinci.h"







 

 







 
#line 20 "..\\..\\..\\Drivers\\BSP\\STM32L496V_Davinci\\stm32l4xx_Davinci.h"
 	 
#line 31 "..\\..\\..\\Drivers\\BSP\\STM32L496V_Davinci\\stm32l4xx_Davinci.h"

 



 


													
 



 



 



 


 

 
void BSP_EXT_Power_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
int8_t smp_timer_handler(uint32_t sample_rate);
int8_t user_timer_handler(uint32_t sample_rate);
int8_t led_timer_handler(uint32_t sample_rate);
int smp_timer_deinit(void);
int user_timer_deinit(void);
int led_timer_deinit(void);

uint16_t smp_time_count_get(void);
void smp_time_count_set(uint16_t val);






    
 
#line 22 "..\\..\\..\\APP\\smp_log_managment.h"

























#line 55 "..\\..\\..\\APP\\smp_log_managment.h"
	typedef enum{
		SMP_REFLASH_MEMORY = 0,
		SMP_FIX_MEMORY,
	}smp_flash_type;
	
	typedef struct{
		uint8_t header[4];
		uint16_t reflash_memory_head_page;
		uint16_t reflash_memory_current_page;
		uint16_t reflash_total_log_cnt;
		uint16_t fix_memory_current_page;
		uint16_t fix_total_log_cnt;
		uint8_t dummy[2];
	}smp_sector_header_package;
	
	typedef struct{
		uint8_t package_num;
		uint8_t page_usage_size;
	}smp_page_header_package;
	
	typedef struct{
		uint8_t ID;
		uint8_t SMP_RTC[4];
		uint8_t data[2];
		uint8_t sum;
	}smp_log_package;
	
	typedef enum{
		SMP_LOG_EVENT_SECTOR_HEADER_SAVE_DONE = 0,
		SMP_LOG_EVENT_SECTOR_HEADER_LOAD_DONE,
		SMP_LOG_EVENT_PAGE_SAVE_DONE,
		SMP_LOG_EVENT_PAGE_LOAD_DONE,
		SMP_LOG_EVENT_MEMORY_FULL,
		SMP_LOG_EVENT_BUSY,
		SMP_LOG_EVENT_ERROR
	}smp_log_evt_type;
	typedef void (*smp_log_event_t)(smp_log_evt_type p_evt);
	
	
	int8_t app_flash_log_managment_init(smp_log_event_t smp_log_event_handle);
	void app_flash_log_managment_clean_all_memory(void);
	void app_flash_log_managment_clean_head(void);
	void app_flash_log_managment_clean_reflash_memory(void);
	void app_flash_log_managment_clean_fix_memory(void);
	void app_flash_sector_header_save(smp_sector_header_package * sector_header);
	void app_flash_sector_header_load(smp_sector_header_package * sector_header);
	void app_flash_page_data_push(smp_log_package log_package,smp_flash_type flash_type);
	void app_flash_page_data_save(smp_flash_type flash_type);
	void app_flash_page_data_load(uint8_t * RX_buffer , uint16_t log_start_position, uint16_t log_length,smp_flash_type flash_type);






#line 36 "..\\..\\..\\User\\main.c"
#line 1 "..\\..\\..\\RTT\\SEGGER_RTT.h"























































 




#line 1 "..\\..\\..\\RTT\\SEGGER_RTT_Conf.h"























































 













 


















































 
































 
#line 241 "..\\..\\..\\RTT\\SEGGER_RTT_Conf.h"




 
#line 296 "..\\..\\..\\RTT\\SEGGER_RTT_Conf.h"




 
#line 310 "..\\..\\..\\RTT\\SEGGER_RTT_Conf.h"




 
#line 324 "..\\..\\..\\RTT\\SEGGER_RTT_Conf.h"




 
#line 351 "..\\..\\..\\RTT\\SEGGER_RTT_Conf.h"










 
#line 383 "..\\..\\..\\RTT\\SEGGER_RTT_Conf.h"




 
#line 398 "..\\..\\..\\RTT\\SEGGER_RTT_Conf.h"





 
#line 415 "..\\..\\..\\RTT\\SEGGER_RTT_Conf.h"




 









 
#line 62 "..\\..\\..\\RTT\\SEGGER_RTT.h"






 
#line 125 "..\\..\\..\\RTT\\SEGGER_RTT.h"
    
    
    
#line 146 "..\\..\\..\\RTT\\SEGGER_RTT.h"
  
  
  
#line 158 "..\\..\\..\\RTT\\SEGGER_RTT.h"










#line 175 "..\\..\\..\\RTT\\SEGGER_RTT.h"





#line 192 "..\\..\\..\\RTT\\SEGGER_RTT.h"

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





 
#line 195 "..\\..\\..\\RTT\\SEGGER_RTT.h"
#line 196 "..\\..\\..\\RTT\\SEGGER_RTT.h"






 








#line 218 "..\\..\\..\\RTT\\SEGGER_RTT.h"






 





typedef struct {
  const     char*    sName;         
            char*    pBuffer;       
            unsigned SizeOfBuffer;  
            unsigned WrOff;         
  volatile  unsigned RdOff;         
            unsigned Flags;         
} SEGGER_RTT_BUFFER_UP;





typedef struct {
  const     char*    sName;         
            char*    pBuffer;       
            unsigned SizeOfBuffer;  
  volatile  unsigned WrOff;         
            unsigned RdOff;         
            unsigned Flags;         
} SEGGER_RTT_BUFFER_DOWN;






typedef struct {
  char                    acID[16];                                 
  int                     MaxNumUpBuffers;                          
  int                     MaxNumDownBuffers;                        
  SEGGER_RTT_BUFFER_UP    aUp[(3)];       
  SEGGER_RTT_BUFFER_DOWN  aDown[(3)];   



} SEGGER_RTT_CB;






 
extern SEGGER_RTT_CB _SEGGER_RTT;






 



int          SEGGER_RTT_AllocDownBuffer         (const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags);
int          SEGGER_RTT_AllocUpBuffer           (const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags);
int          SEGGER_RTT_ConfigUpBuffer          (unsigned BufferIndex, const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags);
int          SEGGER_RTT_ConfigDownBuffer        (unsigned BufferIndex, const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags);
int          SEGGER_RTT_GetKey                  (void);
unsigned     SEGGER_RTT_HasData                 (unsigned BufferIndex);
int          SEGGER_RTT_HasKey                  (void);
unsigned     SEGGER_RTT_HasDataUp               (unsigned BufferIndex);
void         SEGGER_RTT_Init                    (void);
unsigned     SEGGER_RTT_Read                    (unsigned BufferIndex,       void* pBuffer, unsigned BufferSize);
unsigned     SEGGER_RTT_ReadNoLock              (unsigned BufferIndex,       void* pData,   unsigned BufferSize);
int          SEGGER_RTT_SetNameDownBuffer       (unsigned BufferIndex, const char* sName);
int          SEGGER_RTT_SetNameUpBuffer         (unsigned BufferIndex, const char* sName);
int          SEGGER_RTT_SetFlagsDownBuffer      (unsigned BufferIndex, unsigned Flags);
int          SEGGER_RTT_SetFlagsUpBuffer        (unsigned BufferIndex, unsigned Flags);
int          SEGGER_RTT_WaitKey                 (void);
unsigned     SEGGER_RTT_Write                   (unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned     SEGGER_RTT_WriteNoLock             (unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned     SEGGER_RTT_WriteSkipNoLock         (unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned     SEGGER_RTT_ASM_WriteSkipNoLock     (unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned     SEGGER_RTT_WriteString             (unsigned BufferIndex, const char* s);
void         SEGGER_RTT_WriteWithOverwriteNoLock(unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned     SEGGER_RTT_PutChar                 (unsigned BufferIndex, char c);
unsigned     SEGGER_RTT_PutCharSkip             (unsigned BufferIndex, char c);
unsigned     SEGGER_RTT_PutCharSkipNoLock       (unsigned BufferIndex, char c);
unsigned     SEGGER_RTT_GetAvailWriteSpace      (unsigned BufferIndex);
unsigned     SEGGER_RTT_GetBytesInBuffer        (unsigned BufferIndex);














 
unsigned     SEGGER_RTT_ReadUpBuffer            (unsigned BufferIndex, void* pBuffer, unsigned BufferSize);
unsigned     SEGGER_RTT_ReadUpBufferNoLock      (unsigned BufferIndex, void* pData, unsigned BufferSize);
unsigned     SEGGER_RTT_WriteDownBuffer         (unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned     SEGGER_RTT_WriteDownBufferNoLock   (unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);








 
int     SEGGER_RTT_SetTerminal        (unsigned char TerminalId);
int     SEGGER_RTT_TerminalOut        (unsigned char TerminalId, const char* s);






 
int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);
int SEGGER_RTT_vprintf(unsigned BufferIndex, const char * sFormat, va_list * pParamList);












 
















#line 388 "..\\..\\..\\RTT\\SEGGER_RTT.h"

#line 397 "..\\..\\..\\RTT\\SEGGER_RTT.h"

#line 406 "..\\..\\..\\RTT\\SEGGER_RTT.h"

#line 415 "..\\..\\..\\RTT\\SEGGER_RTT.h"




 
#line 37 "..\\..\\..\\User\\main.c"
#line 1 "..\\..\\..\\RTT\\RTT_Log.h"

#line 5 "..\\..\\..\\RTT\\RTT_Log.h"






#line 17 "..\\..\\..\\RTT\\RTT_Log.h"

 


 


 
#line 33 "..\\..\\..\\RTT\\RTT_Log.h"

#line 42 "..\\..\\..\\RTT\\RTT_Log.h"

#line 38 "..\\..\\..\\User\\main.c"









































extern uint8_t app_afe_cb(bq796xx_data_t *bq_data, bq796xx_event_cb_type bq_event);

extern void ads7946_callBack(uint8_t *pDat, uint8_t size);

#line 96 "..\\..\\..\\User\\main.c"




																
extern bq796xx_init_default_t bq796xx_default;

uint8_t num=0;
uint8_t d_payload[4] = {0};
uint8_t null_payload[4] = {0};
uint8_t cb_en1,cb_en2;
uint16_t test_cont = 0;

uint8_t bmu_dir=0;
uint16_t bmu_dir_cnt = 0;

extern uint8_t step_com_f;
extern uint8_t before_d_ms;
extern uint16_t bq_count_temp;
extern uint8_t  cnt_delay;
extern bq796xx_init_steps_enum afe_steps;
uint8_t res;

bq796xx_dir_set_steps_enum dir_afe_steps;

uint8_t dir_step_com_f;
uint8_t dir_before_d_ms;
uint16_t dir_bq_count_temp;
uint8_t  dir_cnt_delay;

uint8_t north_res = 0,south_res = 0, dir_res=0, dir_state;
uint8_t ns_bmu_cnt = 0;
uint8_t bmu_is_ring = 0;
uint8_t ns_recheck_cnt = 0;
extern bq796xx_wake_tone_switch wake_sw;
uint8_t wake_cnt = 0;

uint16_t app_adc_temp[5]={0};

long int test_dir_ok[2] ={0};
long int test_dir_fail[2] ={0};
long int irm_get_vstack_cont = 0;

uint16_t  test_init_rec_bmu_num[2][40]={0};

 
 

																				
 


 
 
static volatile uint32_t TimingDelay;
static volatile uint32_t button_pressed = 0;
 
void SystemClock_Config_80MHz(void);
void SystemClock_Config_HSE_80MHz(void);
void SystemClock_Config_24MHz (void);
 





apiIRMonitoring_cb_t app_irm_event_cb;



uint8_t app_irm_rxdata_cb(IRMonitoring_Resistor_t *irm_res_data, IRMonitoring_event_cb_type irm_event){

  static IRMonitoring_Resistor_t temp_irm_data;
  static float temp_irm_vstack;
	
 	switch(irm_event)
 	{
 	case IRM_EVENT_BALANCE:
    temp_irm_data = *irm_res_data;
		break;
 	case IRM_EVENT_UNBALANCE:
    temp_irm_data = *irm_res_data;
		break;
 	case IRM_EVENT_GET_VSTACK:
    temp_irm_vstack = irm_res_data->V_stack;
		break;	
 	case IRM_EVENT_OTHER_ERR:

		break;
  }
	

	SEGGER_RTT_printf(0,"IRM EVENT=%d, Vstack=%d, Rn=%d, Rp=%d\r\n", irm_event, (int)(temp_irm_data.V_stack), temp_irm_data.Rn_kohm, temp_irm_data.Rp_kohm);

	
	return 0;
}


void app_irm_sw_gpio_init_cb(void){
    GPIO_InitTypeDef GPIO_InitStructure;
	
	  
	  
	  GPIO_InitStructure.Pin   = ((uint16_t)0x4000);
	  GPIO_InitStructure.Mode  = (0x00000001u);
	  GPIO_InitStructure.Pull  = (0x00000001u);
	  GPIO_InitStructure.Speed = (0x00000002u);
	  HAL_GPIO_Init(((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0C00UL)), &GPIO_InitStructure); 	

 	  GPIO_InitStructure.Pin   = ((uint16_t)0x8000);
	  GPIO_InitStructure.Mode  = (0x00000001u);
	  GPIO_InitStructure.Pull  = (0x00000001u);
	  GPIO_InitStructure.Speed = (0x00000002u);
	  HAL_GPIO_Init(((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0C00UL)), &GPIO_InitStructure); 	
	
 	  GPIO_InitStructure.Pin   = ((uint16_t)0x0040);
	  GPIO_InitStructure.Mode  = (0x00000001u);
	  GPIO_InitStructure.Pull  = (0x00000001u);
	  GPIO_InitStructure.Speed = (0x00000002u);
	  HAL_GPIO_Init(((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0800UL)), &GPIO_InitStructure); 	
	  
}

void app_irm_sw1_gpio(IRMonitoring_SW_enum on_off){
    if(on_off == IRM_SW_ON){
	      (((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0C00UL))->ODR |= ((uint16_t)0x4000));
	  }else{
		    (((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0C00UL))->ODR &= ~((uint16_t)0x4000));
		}
}

void app_irm_sw2_gpio(IRMonitoring_SW_enum on_off){
    if(on_off == IRM_SW_ON){
	      (((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0C00UL))->ODR |= ((uint16_t)0x8000));
	  }else{
		    (((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0C00UL))->ODR &= ~((uint16_t)0x8000));
		}
}

void app_irm_sw3_gpio(IRMonitoring_SW_enum on_off){
    if(on_off == IRM_SW_ON){
	      (((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0800UL))->ODR |= ((uint16_t)0x0040));
	  }else{
		    (((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0800UL))->ODR &= ~((uint16_t)0x0040));
		}
}

IRM_Recv_CB_t irm_fun_ptr = 0;
static void app_imr_ads7946_callBack(uint8_t *pDat, uint8_t size)
{
	tIbyte	AdcValue;
	static float volt_data;
	static float adc_bits;
  adc_bits = (float)(1<<14); 	
	
	AdcValue.b[1] = pDat[2];
	AdcValue.b[0] = pDat[3];
	AdcValue.i >>= 3;
	
	if(irm_fun_ptr !=0){
	    volt_data = (float)(AdcValue.i)/adc_bits * 5.0f;
		  irm_fun_ptr(&volt_data);
	}
	
}

void irm_data_ready_cb(IRM_Recv_CB_t rcv_ptr){
	irm_fun_ptr = rcv_ptr;
}

IRMonitoring_event_read_cb_type app_irm_trigger_voltage_data_cb(void){
	  static int8_t res;
		
    res = smp_ADS7946_get_data(channel_0,CS_0,app_imr_ads7946_callBack);
	  
	  if(res == ((0x0) - 0)){
		    return(IRM_OK);	
		}else{
		    return(IRM_BUSY);
		}
	
}

void app_irm_get_device_init_cb(void){
    smp_ADS7946_init();
}


static void smp_DMA_Init(void)
{
	 
	do { volatile uint32_t tmpreg; ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->AHB1ENR) |= ((0x1UL << (0U)))); tmpreg = ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->AHB1ENR) & ((0x1UL << (0U)))); (void)tmpreg; } while(0);
	do { volatile uint32_t tmpreg; ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->AHB1ENR) |= ((0x1UL << (0U)))); tmpreg = ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->AHB1ENR) & ((0x1UL << (0U)))); (void)tmpreg; } while(0);
	do { volatile uint32_t tmpreg; ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->AHB1ENR) |= ((0x1UL << (1U)))); tmpreg = ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->AHB1ENR) & ((0x1UL << (1U)))); (void)tmpreg; } while(0);
	 
	 
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	 
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	 
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	 
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	 
	HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
	 
	HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

}


extern uint16_t Davinci_uart_rx_cnt;
extern smp_uart_t mDavinci_uart;
smp_sector_header_package	header_package;
uint8_t page_data_buffer[256];



#line 433 "..\\..\\..\\User\\main.c"



uint16_t kk = 0;
void test_uart_rx_process(void)
{
static uint8_t rx_data = 0;
static int8_t fifo_res;
	
	if(Davinci_uart_rx_cnt>0){
		fifo_res = smp_uart_get(&mDavinci_uart, &rx_data);
		--Davinci_uart_rx_cnt;
		if(fifo_res == ((0x0) - 0))
		{	
      switch(rx_data){
			case 'A':	
				SEGGER_RTT_printf(0,"%s%s" "EVENT_LOG Clean ALL Memory\r\n" "%s", "\x1B[1;33m", "", "\x1B[0m");
				app_flash_log_managment_clean_all_memory();
			  break; 
			case 'B':
				SEGGER_RTT_printf(0,"%s%s" "EVENT_LOG Clean REFLASH Memory\r\n" "%s", "\x1B[1;33m", "", "\x1B[0m");
				app_flash_log_managment_clean_reflash_memory();
			  break;
			case 'C':		
				SEGGER_RTT_printf(0,"%s%s" "EVENT_LOG Clean FIX Memory\r\n" "%s", "\x1B[1;33m", "", "\x1B[0m");
				app_flash_log_managment_clean_fix_memory();
			  break;
			case 'D':
				SEGGER_RTT_printf(0,"%s%s" "EVENT_LOG Clean HEAD\r\n" "%s", "\x1B[1;33m", "", "\x1B[0m");
				app_flash_log_managment_clean_head();
			  break;
		  }
		}
	}
}

void app_flash_log_event_handler(smp_log_evt_type p_evt)
{
	switch(p_evt){
		case SMP_LOG_EVENT_SECTOR_HEADER_LOAD_DONE:
				SEGGER_RTT_printf(0,"%s%s" "load head\r\n" "%s", "\x1B[1;36m", "", "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "header %x\r\n" "%s", "\x1B[1;36m", "",header_package . header[0], "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "header %x\r\n" "%s", "\x1B[1;36m", "",header_package . header[1], "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "header %x\r\n" "%s", "\x1B[1;36m", "",header_package . header[2], "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "header %x\r\n" "%s", "\x1B[1;36m", "",header_package . header[3], "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "reflash head page%x\r\n" "%s", "\x1B[1;36m", "",header_package . reflash_memory_head_page, "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "reflash current page%x\r\n" "%s", "\x1B[1;36m", "",header_package . reflash_memory_current_page, "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "reflash cnt%d\r\n" "%s", "\x1B[1;36m", "",header_package . reflash_total_log_cnt, "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "fix curent page%x\r\n" "%s", "\x1B[1;36m", "",header_package . fix_memory_current_page, "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "fix cnt%d\r\n" "%s", "\x1B[1;36m", "",header_package . fix_total_log_cnt, "\x1B[0m");
		break;
		case SMP_LOG_EVENT_SECTOR_HEADER_SAVE_DONE:
				SEGGER_RTT_printf(0,"%s%s" "save head\r\n" "%s", "\x1B[1;36m", "", "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "header %x\r\n" "%s", "\x1B[1;36m", "",header_package . header[0], "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "header %x\r\n" "%s", "\x1B[1;36m", "",header_package . header[1], "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "header %x\r\n" "%s", "\x1B[1;36m", "",header_package . header[2], "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "header %x\r\n" "%s", "\x1B[1;36m", "",header_package . header[3], "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "reflash head page%x\r\n" "%s", "\x1B[1;36m", "",header_package . reflash_memory_head_page, "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "reflash cnt%x\r\n" "%s", "\x1B[1;36m", "",header_package . reflash_memory_current_page, "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "reflash cnt%d\r\n" "%s", "\x1B[1;36m", "",header_package . reflash_total_log_cnt, "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "fix curent page%x\r\n" "%s", "\x1B[1;36m", "",header_package . fix_memory_current_page, "\x1B[0m");
				SEGGER_RTT_printf(0,"%s%s" "fix cnt%d\r\n" "%s", "\x1B[1;36m", "",header_package . fix_total_log_cnt, "\x1B[0m");
		break;
		case SMP_LOG_EVENT_PAGE_LOAD_DONE:
				SEGGER_RTT_printf(0,"%s%s" "page load\r\n" "%s", "\x1B[1;36m", "", "\x1B[0m");
				for(int i = 0; i < 256;i++){				
					SEGGER_RTT_printf(0,"%s%s" "%d,%x\r\n" "%s", "\x1B[1;36m", "",i,page_data_buffer[i], "\x1B[0m");
				}
		break;
		case SMP_LOG_EVENT_PAGE_SAVE_DONE:
				SEGGER_RTT_printf(0,"%s%s" "page save\r\n" "%s", "\x1B[1;36m", "", "\x1B[0m");
		break;
		case SMP_LOG_EVENT_MEMORY_FULL:
				SEGGER_RTT_printf(0,"%s%s" "fix memory full\r\n" "%s", "\x1B[1;36m", "", "\x1B[0m");
		break;
		case SMP_LOG_EVENT_ERROR:
				SEGGER_RTT_printf(0,"%s%s" "Error\r\n" "%s", "\x1B[1;36m", "", "\x1B[0m");
		break;
		default:
		break;
	}
}






 
int main(void)
{

  static uint32_t counter = 0; 

  
	uint32_t		del;
  GPIO_InitTypeDef GPIO_InitStructure;

	uint32_t SMPS_status = 0;
	uint32_t resumed_from_standby = 0;
	
	








 
		
	((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0]=0xFFFFFFFF;
	((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[1]=0xFFFFFFFF;
	((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[2]=0xFFFFFFFF;
	
  HalBspInit();




   



  SystemClock_Config_HSE_80MHz();
	
	smp_DMA_Init();
	do { volatile uint32_t tmpreg; ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->AHB2ENR) |= ((0x1UL << (0U)))); tmpreg = ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->AHB2ENR) & ((0x1UL << (0U)))); (void)tmpreg; } while(0);
	do { volatile uint32_t tmpreg; ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->AHB2ENR) |= ((0x1UL << (1U)))); tmpreg = ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->AHB2ENR) & ((0x1UL << (1U)))); (void)tmpreg; } while(0);
	do { volatile uint32_t tmpreg; ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->AHB2ENR) |= ((0x1UL << (2U)))); tmpreg = ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->AHB2ENR) & ((0x1UL << (2U)))); (void)tmpreg; } while(0);
	do { volatile uint32_t tmpreg; ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->AHB2ENR) |= ((0x1UL << (3U)))); tmpreg = ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->AHB2ENR) & ((0x1UL << (3U)))); (void)tmpreg; } while(0);
	do { volatile uint32_t tmpreg; ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->AHB2ENR) |= ((0x1UL << (4U)))); tmpreg = ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->AHB2ENR) & ((0x1UL << (4U)))); (void)tmpreg; } while(0);	
	do { volatile uint32_t tmpreg; ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->APB1ENR1) |= ((0x1UL << (10U)))); tmpreg = ((((RCC_TypeDef *) (((0x40000000UL) + 0x00020000UL) + 0x1000UL))->APB1ENR1) & ((0x1UL << (10U)))); (void)tmpreg; } while(0);




	
	GPIO_InitStructure.Pin = 0x1F;                                               
	GPIO_InitStructure.Mode = (0x00000001u);
	GPIO_InitStructure.Pull = (0x00000001u);
	GPIO_InitStructure.Speed = (0x00000002u);



	
	GPIO_InitStructure.Pin = ((uint16_t)0x2000) | ((uint16_t)0x4000) |((uint16_t)0x8000);
	GPIO_InitStructure.Mode = (0x00000001u);
	GPIO_InitStructure.Pull = (0x00000001u);
	GPIO_InitStructure.Speed = (0x00000002u);
	HAL_GPIO_Init(((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0C00UL)), &GPIO_InitStructure); 

	GPIO_InitStructure.Pin = ((uint16_t)0x0400) | ((uint16_t)0x0800);
	GPIO_InitStructure.Mode = (0x00000001u);
	GPIO_InitStructure.Pull = (0x00000001u);
	GPIO_InitStructure.Speed = (0x00000002u);



	
	GPIO_InitStructure.Pin = ((uint16_t)0x0040);
	GPIO_InitStructure.Mode = (0x00000001u);
	GPIO_InitStructure.Pull = (0x00000001u);
	GPIO_InitStructure.Speed = (0x00000002u);
	HAL_GPIO_Init(((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0800UL)), &GPIO_InitStructure); 
	
	GPIO_InitStructure.Pin = ((uint16_t)0x0020) | ((uint16_t)0x0040);
	GPIO_InitStructure.Mode = (0x00000001u);
	GPIO_InitStructure.Pull = (0x00000001u);
	GPIO_InitStructure.Speed = (0x00000002u);



	
	GPIO_InitStructure.Pin = ((uint16_t)0x0800) | ((uint16_t)0x1000) | ((uint16_t)0x2000);
	GPIO_InitStructure.Mode = (0x00000001u);
	GPIO_InitStructure.Pull = (0x00000001u);
	GPIO_InitStructure.Speed = (0x00000002u);
	HAL_GPIO_Init(((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x1000UL)), &GPIO_InitStructure); 
  HAL_Delay(200);
	
	SEGGER_RTT_Init();
	SEGGER_RTT_printf(0,"  start\r\n" );
	
  
	

  maxInit(4,0);
	HAL_Delay(500);
	MAX7219_All_Display(0xFF);
	HAL_Delay(500);
	MAX7219_All_Display(0x00);
	HAL_Delay(500);
	MAX7219_Display_1();
	HAL_Delay(1000);

  

	appProjectOpen();
	LibSwTimerClearCount();
	
	
	
  appSerialUartSendMessage("12345678");
	
	
	
	

	res = smp_mx25l_flash_init();

	
	
	
	

	smp_ADS7946_init();

	
	
	
	

	static	uint16_t	app_adc_temp[10];
	extern bsp_adc_init_io_config bsp_in_adc_ch[5];
	
	test_cont = 0;
	smp_adc_adc_para_init(adc1);                            
	while(1){
		hal_internal_adc_get(&app_adc_temp[0] ,adc1 , bsp_in_adc_ch[0]);
		hal_internal_adc_get(&app_adc_temp[1] ,adc1 , bsp_in_adc_ch[1]);
		hal_internal_adc_get(&app_adc_temp[2] ,adc1 , bsp_in_adc_ch[2]);
		hal_internal_adc_get(&app_adc_temp[3] ,adc1 , bsp_in_adc_ch[3]);
		hal_internal_adc_get(&app_adc_temp[4] ,adc1 , bsp_in_adc_ch[4]);
		
		SEGGER_RTT_printf(0,"%s%s" "TEST ADC#%04d %04d,%04d,%04d,%04d,%04d\r\n" "%s", "\x1B[1;34m", "",test_cont,app_adc_temp[0],app_adc_temp[1],app_adc_temp[2],app_adc_temp[3],app_adc_temp[4], "\x1B[0m");
		
		drv_bq796xx_delay_ms(50);
		
		test_cont++;
		if(test_cont>=50) break;
	}

  
	
  
  

  drv_bq796xx_init_default_load(bq796xx_default);

	drv_bq796xx_init();

	bq796xx_event_RegisteCB(app_afe_cb);
	((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0C00UL))->ODR &= ~((uint16_t)0x4000);
	
	bq_count_temp = smp_time_count_get();
	
	res = drv_bq796xx_start_setting(13, DIR_NORTH);
  



	
	drv_bq796xx_Read_Stack_FaultSummary(0);
	drv_bq796xx_delay_ms(10);
				
  for(int k=0; k<10*13; k++){			
	    res = drv_bq796xx_data_frame_parser(); 	
	}

	

  
	for(int k = 0; k<10 ; k++){
	    afe_steps = 0;
	    cnt_delay = 0;
		  res = 0;
		 
		  smp_time_count_set(0);
		
		  if((k%2)==0){			
          dir_state = DIR_NORTH;
			}else{
			    dir_state = DIR_SOUTH;
			}	
			
			++wake_cnt;
			if(wake_cnt>=3){
			   wake_sw = WAKE_TONE_DISABLE;
			}else{
			   wake_sw = WAKE_TONE_ENABLE;
			}
		
			
			wake_sw = WAKE_TONE_ENABLE;
			



			
	    while(1){	
		      ((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0C00UL))->ODR ^= ((uint16_t)0x2000);
		  
		      if(cnt_delay==0){
	            res = drv_bq796xx_Init_Steps(wake_sw,&afe_steps, bq796xx_default.bmu_total_num , dir_state, &step_com_f , &before_d_ms);
				      cnt_delay = before_d_ms;
					    if(step_com_f == 1){
		              ++afe_steps;
			        }
			    }
  		    ((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0C00UL))->ODR ^= ((uint16_t)0x2000);
 			
          if(bq_count_temp !=smp_time_count_get()){	   
				     bq_count_temp = smp_time_count_get();
				     if(cnt_delay>0){
					         --cnt_delay;
				     }
			    }
		
		      if((res>=0) && (afe_steps > AFE_RUN_AUX_ADC)){
				    break;
		      }
	    }
	
   	  ((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0C00UL))->ODR |= ((uint16_t)0x4000);

			if(dir_state == DIR_NORTH){
			    north_res = res & 0x7F;
			}else{
			    south_res = res & 0x7F;
			}
			
			test_init_rec_bmu_num[dir_state][res & 0x7F]++;
			
			bmu_is_ring = ((res & 0x80)>> 7);                                        

			SEGGER_RTT_printf(0,"BMU Init#%04d N=%d,S=%d\r\n", k, north_res, south_res);
			
	    test_cont = 0;
	    while(1){
				 drv_bq796xx_clear_fifobuffer();
	       drv_bq796xx_Read_AFE_ALL_VCELL(STACK, 0, 0);                          
				 drv_bq796xx_delay_ms(1*bq796xx_default.bmu_total_num);                
				 
				 drv_bq796xx_Read_AFE_ALL_ADC(STACK, 0, 0);                            
	       drv_bq796xx_delay_ms(1*bq796xx_default.bmu_total_num);                
		
				 drv_bq796xx_Read_Stack_FaultSummary(0);                               
				 drv_bq796xx_delay_ms(1*bq796xx_default.bmu_total_num);                

				 for(int ki =0; ki<10*13; ki++){
				     res = drv_bq796xx_data_frame_parser();
				 }					 
		     
				 if(test_cont >=2) break;
		     test_cont++;
	    }	
  }
  

	
	
	
#line 819 "..\\..\\..\\User\\main.c"
	
  
	
	

	north_res = 0;
	south_res = 0;
	bmu_is_ring = 0;

  for(long int k = 0; k<100; k++){
	    dir_afe_steps = 0;
	    dir_cnt_delay = 0;
		  dir_res = 0;
		
		  drv_bq796xx_clear_fifobuffer();
		  smp_time_count_set(0);
		  
		  if((k%2)==0){			
          dir_state = DIR_NORTH;
			}else{
			    dir_state = DIR_SOUTH;
			}		
	    
      
      
			if( (north_res ==bq796xx_default.bmu_total_num) || (south_res ==bq796xx_default.bmu_total_num) || ((north_res+ south_res)==bq796xx_default.bmu_total_num)){
		      if((k%2)==0){			
              ns_bmu_cnt = north_res;    
			    }else{
			        ns_bmu_cnt = south_res;    
			    }
					
					++ns_recheck_cnt;
					
					if(ns_recheck_cnt>=50){
						  ns_bmu_cnt = 0;   
						 if(ns_recheck_cnt>=51) ns_recheck_cnt = 0;
					}
			}else{
					ns_bmu_cnt = 0;   
			}	
			
	    while(1){	
		      ((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0C00UL))->ODR ^= ((uint16_t)0x2000);
		 
		      if(dir_cnt_delay==0){
						
						  ns_bmu_cnt = 0;  
						
              dir_res = drv_bq796xx_direction_set_steps(ns_bmu_cnt, &dir_afe_steps, bq796xx_default.bmu_total_num , dir_state, &dir_step_com_f , &dir_before_d_ms);							
						  dir_cnt_delay=dir_before_d_ms;
					    if(dir_step_com_f == 1){
		              ++dir_afe_steps;
			        }
			    }
  		    ((GPIO_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x0C00UL))->ODR ^= ((uint16_t)0x2000);
 			
          if(dir_bq_count_temp !=smp_time_count_get()){	   
				     dir_bq_count_temp = smp_time_count_get();
				     if(dir_cnt_delay>0){
					     --dir_cnt_delay;
				     }
			    }
		
		      if((dir_res>=0) && (dir_afe_steps > SETDIR_AFE_RUN_AUX_ADC)){
				    break;
		      }
	    }
			
			if(dir_state == DIR_NORTH){
			    north_res = dir_res & 0x7F;
			}else{
			    south_res = dir_res & 0x7F;
			}
			
			bmu_is_ring = ((dir_res & 0x80)>> 7);                                       
			
			SEGGER_RTT_printf(0,"%s%s" "BMU CHK#%04d N=%d,S=%d,Ring=%d\r\n" "%s", "\x1B[1;32m", "",k, north_res, south_res , bmu_is_ring, "\x1B[0m");
			
			if(dir_res>=2){
			   test_dir_ok[dir_state]++;
			}else{
			   test_dir_fail[dir_state]++;
			}
			
			drv_bq796xx_delay_ms(dir_state+1);
			
			dir_step_com_f = 0;
			dir_before_d_ms = 0;
			
			test_cont = 0;
	    while(1){
				 drv_bq796xx_clear_fifobuffer();
				
	       drv_bq796xx_Read_AFE_ALL_VCELL(STACK, 0, 0);                               
				 drv_bq796xx_delay_ms(1*bq796xx_default.bmu_total_num);                     
				
				 drv_bq796xx_Read_AFE_ALL_ADC(STACK, 0, 0);                                 
		     drv_bq796xx_delay_ms(1*bq796xx_default.bmu_total_num);                     
				
				 drv_bq796xx_Read_Stack_FaultSummary(0);                                    
				 drv_bq796xx_delay_ms(1*bq796xx_default.bmu_total_num);                     
				
				 for(int ki =0; ki<10*13; ki++){
				     res = drv_bq796xx_data_frame_parser();
				 }					 
		     
				 if(test_cont >=2) break;
		     test_cont++;
	    }
			
			dir_step_com_f = 0;
			dir_before_d_ms = 0;
  }
  


  

  apiFuCheckMagicCode();
	
  for(int k=0 ;k<64; k++){
	    drv_bq796xx_Read_Stack_FaultSummary(0);                                    
	    drv_bq796xx_delay_ms(2*bq796xx_default.bmu_total_num);                     
  }

	drv_bq796xx_clear_fifobuffer();
	 
	
	

	
	app_irm_event_cb.SW_gpioinit_cb = app_irm_sw_gpio_init_cb;
	app_irm_event_cb.SW_gpio_crtl_cb[0]=app_irm_sw1_gpio; 
	app_irm_event_cb.SW_gpio_crtl_cb[1]=app_irm_sw2_gpio;
	app_irm_event_cb.SW_gpio_crtl_cb[2]=app_irm_sw3_gpio; 
	app_irm_event_cb.GetVoltDeviceInit_cb = app_irm_get_device_init_cb;
	app_irm_event_cb.TriggerData_cb = app_irm_trigger_voltage_data_cb;
	app_irm_event_cb.irm_outdata = app_irm_rxdata_cb;
	app_irm_event_cb.DataReady_cb = irm_data_ready_cb;
	
	
	res = apiIRMonitoringOpen(2, 100, app_irm_event_cb);
	
	
	apiIRMonitoringGetVstack();

	
	
	
	

	smp_mx25l_status mx251_status;
	smp_mx25l_flash_init();
	
	uint16_t test_event_log_cnt = 0;
	
	app_flash_log_managment_init(app_flash_log_event_handler);
	HAL_Delay(1000);
	while(test_event_log_cnt<300)
	{
		
	  smp_mx25l_flash_read_status(&mx251_status);
		if((mx251_status.status1&1<<0)==0){		
			 MX25L_SPI_send_command();
		}
    
		test_uart_rx_process();
		
		test_event_log_cnt++;
		HAL_Delay(500);
		SEGGER_RTT_printf(0,"%s%s" "Event log#%d transfer\r\n" "%s", "\x1B[1;37m", "",test_event_log_cnt, "\x1B[0m");
	}

	
	
	while(1)
	{



		
		LibSwTimerHandle();
		
#line 1016 "..\\..\\..\\User\\main.c"
	}

#line 1259 "..\\..\\..\\User\\main.c"
}












 

void SystemClock_Config_24MHz(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

   
  RCC_ClkInitStruct.ClockType = 0x00000001U;
  RCC_ClkInitStruct.SYSCLKSource = (0x00000000UL); 
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, (0x00000003UL)) != HAL_OK)
  {
     
    Error_Handler();
  }

   
  RCC_OscInitStruct.OscillatorType = 0x00000010U;
  RCC_OscInitStruct.MSIState = (0x1UL << (0U));
  RCC_OscInitStruct.MSIClockRange = (0x9UL << (4U));
  RCC_OscInitStruct.MSICalibrationValue = 0U;
  RCC_OscInitStruct.PLL.PLLState = 0x00000001U;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
     
    Error_Handler();
  }
}



















 
void SystemClock_Config_80MHz(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

   
  RCC_OscInitStruct.OscillatorType = 0x00000010U;
  RCC_OscInitStruct.MSIState = (0x1UL << (0U));
  RCC_OscInitStruct.MSIClockRange = (0x6UL << (4U));
  RCC_OscInitStruct.MSICalibrationValue = 0U;
  RCC_OscInitStruct.PLL.PLLState = 0x00000002U;
  RCC_OscInitStruct.PLL.PLLSource = (0x1UL << (0U));
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
     
    Error_Handler();
  }
  
  
 
  RCC_ClkInitStruct.ClockType = (0x00000001U | 0x00000002U | 0x00000004U | 0x00000008U);
  RCC_ClkInitStruct.SYSCLKSource = (0x00000003UL);
  RCC_ClkInitStruct.AHBCLKDivider = (0x00000000UL);
  RCC_ClkInitStruct.APB1CLKDivider = (0x00000000UL);  
  RCC_ClkInitStruct.APB2CLKDivider = (0x00000000UL);  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, (0x00000004UL)) != HAL_OK)
  {
     
    Error_Handler();
  }
}

void SystemClock_Config_HSE_80MHz(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  
 
  if (HAL_PWREx_ControlVoltageScaling((0x1UL << (9U))) != HAL_OK)
  {
    Error_Handler();
  }
  

 
  RCC_OscInitStruct.OscillatorType = 0x00000008U|0x00000001U;
  RCC_OscInitStruct.HSEState = (0x1UL << (16U));
  RCC_OscInitStruct.LSIState = (0x1UL << (0U));
  RCC_OscInitStruct.PLL.PLLState = 0x00000002U;
  RCC_OscInitStruct.PLL.PLLSource = (0x3UL << (0U));
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 0x00000002U;
  RCC_OscInitStruct.PLL.PLLQ = 0x00000002U;
  RCC_OscInitStruct.PLL.PLLR = 0x00000004U;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
 
  RCC_ClkInitStruct.ClockType = 0x00000002U|0x00000001U
                              |0x00000004U|0x00000008U;
  RCC_ClkInitStruct.SYSCLKSource = (0x00000003UL);
  RCC_ClkInitStruct.AHBCLKDivider = (0x00000000UL);
  RCC_ClkInitStruct.APB1CLKDivider = (0x00000000UL);
  RCC_ClkInitStruct.APB2CLKDivider = (0x00000000UL);

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, (0x00000004UL)) != HAL_OK)
  {
    Error_Handler();
  }
}








 
void Error_Handler(void)
{
   
 
  
   
  
  
 
  
	return;
}

void SysTick_Handler(void)
{
	HAL_IncTick();
}




 
void HAL_SYSTICK_Callback(void)
{
  HAL_IncTick();

  if (TimingDelay != 0)
  {
    TimingDelay--;
  }
  else
  {
     
    BSP_LED_Toggle(LED1);
    TimingDelay = 100;
  }
}






 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ((uint16_t)0x2000))
  {
     
    BSP_LED_Init(LED1); 
     
    BSP_LED_On(LED1);
    
     
    button_pressed = 1;
    
  }
}

#line 1487 "..\\..\\..\\User\\main.c"



 



 

     


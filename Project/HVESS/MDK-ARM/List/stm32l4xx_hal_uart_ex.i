#line 1 "..\\..\\..\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_uart_ex.c"








































 

 
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



 



 



 



 







 
#line 45 "..\\..\\..\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_uart_ex.c"



 




 



 
 
#line 72 "..\\..\\..\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_uart_ex.c"

 
 
 


 
static void UARTEx_Wakeup_AddressConfig(UART_HandleTypeDef *huart, UART_WakeUpTypeDef WakeUpSelection);





 

 



 

























































 



















 
HAL_StatusTypeDef HAL_RS485Ex_Init(UART_HandleTypeDef *huart, uint32_t Polarity, uint32_t AssertionTime,
                                   uint32_t DeassertionTime)
{
  uint32_t temp;

   
  if (huart == 0)
  {
    return HAL_ERROR;
  }
   
  ((void)0U);

   
  ((void)0U);

   
  ((void)0U);

   
  ((void)0U);

  if (huart->gState == 0x00000000U)
  {
     
    huart->Lock = HAL_UNLOCKED;

#line 208 "..\\..\\..\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_uart_ex.c"
     
    HAL_UART_MspInit(huart);

  }

  huart->gState = 0x00000024U;

   
  ((huart)->Instance ->CR1 &= ~(0x1UL << (0U)));

   
  if (UART_SetConfig(huart) == HAL_ERROR)
  {
    return HAL_ERROR;
  }

  if (huart->AdvancedInit.AdvFeatureInit != 0x00000000U)
  {
    UART_AdvFeatureConfig(huart);
  }

   
  ((huart->Instance ->CR3) |= ((0x1UL << (14U))));

   
  (((huart->Instance ->CR3)) = ((((((huart->Instance ->CR3))) & (~((0x1UL << (15U))))) | (Polarity))));

   
  temp = (AssertionTime << 21U);
  temp |= (DeassertionTime << 16U);
  (((huart->Instance ->CR1)) = ((((((huart->Instance ->CR1))) & (~(((0x1FUL << (16U)) | (0x1FUL << (21U)))))) | (temp))));

   
  ((huart)->Instance ->CR1 |= (0x1UL << (0U)));

   
  return (UART_CheckIdleState(huart));
}



 



















 





 
__weak void HAL_UARTEx_WakeupCallback(UART_HandleTypeDef *huart)
{
   
  (void)huart;

  

 
}

#line 317 "..\\..\\..\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_uart_ex.c"



 



























































 










 
HAL_StatusTypeDef HAL_UARTEx_EnableClockStopMode(UART_HandleTypeDef *huart)
{
   
  do{ if((huart)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (huart)->Lock = HAL_LOCKED; } }while (0);

   
  ((huart->Instance ->CR3) |= ((0x1UL << (23U))));

   
  do{ (huart)->Lock = HAL_UNLOCKED; }while (0);

  return HAL_OK;
}





 
HAL_StatusTypeDef HAL_UARTEx_DisableClockStopMode(UART_HandleTypeDef *huart)
{
   
  do{ if((huart)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (huart)->Lock = HAL_LOCKED; } }while (0);

   
  ((huart->Instance ->CR3) &= ~((0x1UL << (23U))));

   
  do{ (huart)->Lock = HAL_UNLOCKED; }while (0);

  return HAL_OK;
}














 
HAL_StatusTypeDef HAL_MultiProcessorEx_AddressLength_Set(UART_HandleTypeDef *huart, uint32_t AddressLength)
{
   
  if (huart == 0)
  {
    return HAL_ERROR;
  }

   
  ((void)0U);

  huart->gState = 0x00000024U;

   
  ((huart)->Instance ->CR1 &= ~(0x1UL << (0U)));

   
  (((huart->Instance ->CR2)) = ((((((huart->Instance ->CR2))) & (~((0x1UL << (4U))))) | (AddressLength))));

   
  ((huart)->Instance ->CR1 |= (0x1UL << (0U)));

   
  return (UART_CheckIdleState(huart));
}












 
HAL_StatusTypeDef HAL_UARTEx_StopModeWakeUpSourceConfig(UART_HandleTypeDef *huart, UART_WakeUpTypeDef WakeUpSelection)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t tickstart;

   
  ((void)0U);
   
  ((void)0U);

   
  do{ if((huart)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (huart)->Lock = HAL_LOCKED; } }while (0);

  huart->gState = 0x00000024U;

   
  ((huart)->Instance ->CR1 &= ~(0x1UL << (0U)));

   
  (((huart->Instance ->CR3)) = ((((((huart->Instance ->CR3))) & (~((0x3UL << (20U))))) | (WakeUpSelection . WakeUpEvent))));

  if (WakeUpSelection.WakeUpEvent == 0x00000000U)
  {
    UARTEx_Wakeup_AddressConfig(huart, WakeUpSelection);
  }

   
  ((huart)->Instance ->CR1 |= (0x1UL << (0U)));

   
  tickstart = HAL_GetTick();

   
  if (UART_WaitOnFlagUntilTimeout(huart, (0x1UL << (22U)), RESET, tickstart, 0x1FFFFFFU) != HAL_OK)
  {
    status = HAL_TIMEOUT;
  }
  else
  {
     
    huart->gState = 0x00000020U;
  }

   
  do{ (huart)->Lock = HAL_UNLOCKED; }while (0);

  return status;
}






 
HAL_StatusTypeDef HAL_UARTEx_EnableStopMode(UART_HandleTypeDef *huart)
{
   
  do{ if((huart)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (huart)->Lock = HAL_LOCKED; } }while (0);

   
  ((huart->Instance ->CR1) |= ((0x1UL << (1U))));

   
  do{ (huart)->Lock = HAL_UNLOCKED; }while (0);

  return HAL_OK;
}





 
HAL_StatusTypeDef HAL_UARTEx_DisableStopMode(UART_HandleTypeDef *huart)
{
   
  do{ if((huart)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (huart)->Lock = HAL_LOCKED; } }while (0);

   
  ((huart->Instance ->CR1) &= ~((0x1UL << (1U))));

   
  do{ (huart)->Lock = HAL_UNLOCKED; }while (0);

  return HAL_OK;
}

#line 744 "..\\..\\..\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_uart_ex.c"


















 
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint16_t *RxLen, uint32_t Timeout)
{
  uint8_t  *pdata8bits;
  uint16_t *pdata16bits;
  uint16_t uhMask;
  uint32_t tickstart;

   
  if (huart->RxState == 0x00000020U)
  {
    if ((pData == 0) || (Size == 0U))
    {
      return  HAL_ERROR;
    }

    do{ if((huart)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (huart)->Lock = HAL_LOCKED; } }while (0);

    huart->ErrorCode = (0x00000000U);
    huart->RxState = 0x00000022U;
    huart->ReceptionType = (0x00000001U);

     
    tickstart = HAL_GetTick();

    huart->RxXferSize  = Size;
    huart->RxXferCount = Size;

     
    do { if ((huart)->Init . WordLength == (0x1UL << (12U))) { if ((huart)->Init . Parity == 0x00000000U) { (huart)->Mask = 0x01FFU ; } else { (huart)->Mask = 0x00FFU ; } } else if ((huart)->Init . WordLength == 0x00000000U) { if ((huart)->Init . Parity == 0x00000000U) { (huart)->Mask = 0x00FFU ; } else { (huart)->Mask = 0x007FU ; } } else if ((huart)->Init . WordLength == (0x1UL << (28U))) { if ((huart)->Init . Parity == 0x00000000U) { (huart)->Mask = 0x007FU ; } else { (huart)->Mask = 0x003FU ; } } else { (huart)->Mask = 0x0000U; } } while(0U);
    uhMask = huart->Mask;

     
    if ((huart->Init.WordLength == (0x1UL << (12U))) && (huart->Init.Parity == 0x00000000U))
    {
      pdata8bits  = 0;
      pdata16bits = (uint16_t *) pData;
    }
    else
    {
      pdata8bits  = pData;
      pdata16bits = 0;
    }

    do{ (huart)->Lock = HAL_UNLOCKED; }while (0);

     
    *RxLen = 0U;

     
    while (huart->RxXferCount > 0U)
    {
       
      if ((((huart)->Instance ->ISR & ((0x1UL << (4U)))) == ((0x1UL << (4U)))))
      {
         
        ((huart)->Instance ->ICR = ((0x1UL << (4U))));

         
         
        if (*RxLen > 0U)
        {
          huart->RxState = 0x00000020U;

          return HAL_OK;
        }
      }

       
      if ((((huart)->Instance ->ISR & ((0x1UL << (5U)))) == ((0x1UL << (5U)))))
      {
        if (pdata8bits == 0)
        {
          *pdata16bits = (uint16_t)(huart->Instance->RDR & uhMask);
          pdata16bits++;
        }
        else
        {
          *pdata8bits = (uint8_t)(huart->Instance->RDR & (uint8_t)uhMask);
          pdata8bits++;
        }
         
        *RxLen += 1U;
        huart->RxXferCount--;
      }

       
      if (Timeout != 0xFFFFFFFFU)
      {
        if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
        {
          huart->RxState = 0x00000020U;

          return HAL_TIMEOUT;
        }
      }
    }

     
    *RxLen = huart->RxXferSize - huart->RxXferCount;
     
    huart->RxState = 0x00000020U;

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}













 
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef status;

   
  if (huart->RxState == 0x00000020U)
  {
    if ((pData == 0) || (Size == 0U))
    {
      return HAL_ERROR;
    }

    do{ if((huart)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (huart)->Lock = HAL_LOCKED; } }while (0);

     
    huart->ReceptionType = (0x00000001U);

    status =  UART_Start_Receive_IT(huart, pData, Size);

     
    if (status == HAL_OK)
    {
      if (huart->ReceptionType == (0x00000001U))
      {
        ((huart)->Instance ->ICR = ((0x1UL << (4U))));
        ((huart->Instance ->CR1) |= ((0x1UL << (4U))));
      }
      else
      {
        


 
        status = HAL_ERROR;
      }
    }

    return status;
  }
  else
  {
    return HAL_BUSY;
  }
}
















 
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef status;

   
  if (huart->RxState == 0x00000020U)
  {
    if ((pData == 0) || (Size == 0U))
    {
      return HAL_ERROR;
    }

    do{ if((huart)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (huart)->Lock = HAL_LOCKED; } }while (0);

     
    huart->ReceptionType = (0x00000001U);

    status =  UART_Start_Receive_DMA(huart, pData, Size);

     
    if (status == HAL_OK)
    {
      if (huart->ReceptionType == (0x00000001U))
      {
        ((huart)->Instance ->ICR = ((0x1UL << (4U))));
        ((huart->Instance ->CR1) |= ((0x1UL << (4U))));
      }
      else
      {
        


 
        status = HAL_ERROR;
      }
    }

    return status;
  }
  else
  {
    return HAL_BUSY;
  }
}



 



 



 






 
static void UARTEx_Wakeup_AddressConfig(UART_HandleTypeDef *huart, UART_WakeUpTypeDef WakeUpSelection)
{
  ((void)0U);

   
  (((huart->Instance ->CR2)) = ((((((huart->Instance ->CR2))) & (~((0x1UL << (4U))))) | (WakeUpSelection . AddressLength))));

   
  (((huart->Instance ->CR2)) = ((((((huart->Instance ->CR2))) & (~((0xFFUL << (24U))))) | (((uint32_t)WakeUpSelection . Address << 24U)))));
}

#line 1056 "..\\..\\..\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_uart_ex.c"


 





 



 

 

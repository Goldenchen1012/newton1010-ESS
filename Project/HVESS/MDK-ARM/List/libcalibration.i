#line 1 "..\\..\\..\\Lib\\LibCalibration.c"














 

 

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






 
#line 20 "..\\..\\..\\Lib\\LibCalibration.c"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdbool.h"
 






 





#line 25 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdbool.h"



#line 21 "..\\..\\..\\Lib\\LibCalibration.c"
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







#line 22 "..\\..\\..\\Lib\\LibCalibration.c"

 
 
 
 
 
 
int32_t doCalibration(tCalibCoef *par, int32_t dataX){ 
	int32_t dataTemp;
	dataTemp = dataX * par->A1;
	dataTemp += (dataX * par->A2) / 100000;
	dataTemp += par->B;
	return dataTemp;
}

tCalibCoef calCoef(int32_t valL, int32_t adcL, int32_t valH, int32_t adcH){
    tCalibCoef coef;
    double y1, y2, x1, x2;
    double cA, cB;
    
    if(valL == valH){
        coef.A1 = 0;
        coef.A2 = 0;
        coef.B = 0;
    }else{
        y1 = valL;
        y2 = valH;
        x1 = adcL;
        x2 = adcH;
        cA = (y1 - y2) / (x1 - x2);
        cB = y1 - (cA * x1);
        coef.A1 = (int32_t)cA;
        coef.A2 = (int32_t)((cA - (double)coef.A1) * 100000);
        coef.B = (int32_t)cB;
    }
#line 66 "..\\..\\..\\Lib\\LibCalibration.c"
    return coef;
}


     


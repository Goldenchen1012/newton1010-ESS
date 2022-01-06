#line 1 "..\\..\\..\\HAL\\HalAfe.c"














 

 
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






 
#line 19 "..\\..\\..\\HAL\\HalAfe.c"
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







	



     
#line 20 "..\\..\\..\\HAL\\HalAfe.c"
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





#line 21 "..\\..\\..\\HAL\\HalAfe.c"
#line 1 "..\\..\\..\\Lib\\LibNtc.h"














 







	
 
#line 26 "..\\..\\..\\Lib\\LibNtc.h"

	
 
	
 

 

 
 
uint16_t LibTemperatureToVoltage(int16_t temp);
uint16_t LibNtcRToTemperature(double NtcR);
uint16_t LibNtcVoltageToTemperature(uint16_t NtcVoltage);
uint16_t LibSetRealTemperatureToInternalValue(int16_t temp);






#line 22 "..\\..\\..\\HAL\\HalAfe.c"
#line 1 "..\\..\\..\\HAL\\HalAfeBq796xx.h"














 



 
#line 21 "..\\..\\..\\HAL\\HalAfeBq796xx.h"






 
 
	
 


 
 
int8_t HalAfeBq796xx_Init(void);








     
#line 23 "..\\..\\..\\HAL\\HalAfe.c"
#line 1 "..\\..\\..\\API\\ApiSysPar.h"














 



 
#line 21 "..\\..\\..\\API\\ApiSysPar.h"
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







	



     


#line 24 "..\\..\\..\\HAL\\HalAfe.c"

 


 

 

typedef struct{
	uint16_t	CellVoltage[400 + 4];
	uint16_t		NtcAdcData[400 + 4];
	int32_t		CurrentValue[2];
	int32_t			CurrentAdcValue[2];
	uint32_t		VBat[2];
	int32_t			VbatAdcValue[2];
	uint16_t		CellNumber;
	uint16_t		MaxCellVoltage;
	uint16_t		MinCellVoltage;
	uint16_t		MaxTempNtcVoltage;
	uint16_t		MinTempNtcVoltage;
}tAfeBuffer;

tAfeBuffer	AfeBuffer;
 
 
 
 
int32_t	halAfeGetCurrentAdcValue(uint8_t CurrentIndex)
{
	if(CurrentIndex >= 2)
		return 0;
	return AfeBuffer.CurrentAdcValue[CurrentIndex];
}

void halAfeSetCurrentAdcValue(uint8_t CurrentIndex,int32_t adcvalue)
{
	if(CurrentIndex >= 2)
		return;
	AfeBuffer.CurrentAdcValue[CurrentIndex] = adcvalue;
}
int32_t	halAfeGetVBatAdcValue(uint8_t VbIndex)
{
	if(VbIndex >= 2)
		return 0;
	return AfeBuffer.VbatAdcValue[VbIndex];
}
void halAfeSetVBatAdcValue(uint8_t VbIndex, int32_t adcvalue)
{
	if(VbIndex >= 2)
		return;
	AfeBuffer.VbatAdcValue[VbIndex] = adcvalue;
}

void halAfeSetCellVoltage(uint16_t cell, uint16_t voltage)
{
	AfeBuffer.CellVoltage[cell] = voltage;
}
void halAfeSetNtcAdcData(uint16_t ntcs, uint16_t adcdata)
{
	AfeBuffer.NtcAdcData[ntcs] = adcdata;
}

uint32_t HalAfeGetBatteryVoltage(void)
{
	uint32_t	vbat = 0;
	uint16_t		cell;
		 
	for(cell=0; cell<AfeBuffer.CellNumber; cell++)
	{
		vbat += AfeBuffer.CellVoltage[cell];
	}
	return vbat;
}

uint16_t halAfeGetCellVoltage(uint16_t CellIndex)
{
	return AfeBuffer.CellVoltage[CellIndex];	
}

uint16_t HalAfeGetNtcAdc(uint16_t NtcIndex)
{
	return AfeBuffer.NtcAdcData[NtcIndex];	
}

int32_t halAfeGetCurrentValue(uint8_t index)
{
	return AfeBuffer.CurrentValue[index];
}
void HalAfeSetCurrentValue(uint8_t index, int32_t current)
{
	AfeBuffer.CurrentValue[index] = current;
}

uint32_t halAfeGetVBatVoltage(uint8_t index)
{
	return AfeBuffer.VBat[index];
}
void halAfeSetVBatVoltage(uint8_t index, uint32_t voltage)
{
	AfeBuffer.VBat[index] = voltage;
}


uint16_t halAfeGetMaxCellVoltage(void)
{
	return AfeBuffer.MaxCellVoltage;
}

uint16_t halAfeGetMinCellVoltage(void)
{
	return AfeBuffer.MinCellVoltage;
}

uint16_t HalAfeGetMinNtcTempAdc(void)
{
	return AfeBuffer.MinTempNtcVoltage;
}
uint16_t HalAfeGetMaxNtcTempAdc(void)
{
	return AfeBuffer.MaxTempNtcVoltage;
}


void halAfeUpdateMinMaxCellVoltage(void)
{
	uint16_t	cell;
	uint16_t	cv;
	
	AfeBuffer.MaxCellVoltage = 0;
	AfeBuffer.MinCellVoltage = 0xffff;
	
	for(cell=0; cell<apiSysParGetCellNumber(); cell++)
	{
		cv  = halAfeGetCellVoltage(cell);
		if(cv > AfeBuffer.MaxCellVoltage)
		{		
			AfeBuffer.MaxCellVoltage = cv;
		}
		if(cv < AfeBuffer.MinCellVoltage)
		{		
			AfeBuffer.MinCellVoltage = cv;
		}
	}
}
void halAfeUpdateMinMaxNtcTempVoltage(void)
{
	uint16_t	ntc;
	uint16_t	cv;
	
	AfeBuffer.MinTempNtcVoltage = 0;
	AfeBuffer.MaxTempNtcVoltage = 0xffff;
	
	for(ntc=0; ntc<apiSysParGetNtcNumber(); ntc++)
	{
		cv = HalAfeGetNtcAdc(ntc);
		if(cv < AfeBuffer.MaxTempNtcVoltage)
		{		
			AfeBuffer.MaxTempNtcVoltage = cv;
		}
		if(cv > AfeBuffer.MinTempNtcVoltage)
		{		
			AfeBuffer.MinTempNtcVoltage = cv;
		}
	}
}



     



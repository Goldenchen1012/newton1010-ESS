#line 1 "..\\..\\..\\AppProtect\\ApiProtectDotp.c"














 

 
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






 
#line 19 "..\\..\\..\\AppProtect\\ApiProtectDotp.c"
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











#line 20 "..\\..\\..\\AppProtect\\ApiProtectDotp.c"
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







	



     
#line 21 "..\\..\\..\\AppProtect\\ApiProtectDotp.c"
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

 
 
 
 










     





#line 22 "..\\..\\..\\AppProtect\\ApiProtectDotp.c"
#line 1 "..\\..\\..\\AppProtect\\ApiProtectDotp.h"














 


 
#line 20 "..\\..\\..\\AppProtect\\ApiProtectDotp.h"
#line 21 "..\\..\\..\\AppProtect\\ApiProtectDotp.h"
#line 22 "..\\..\\..\\AppProtect\\ApiProtectDotp.h"




 
 


 
 
 
void apiProtectDotpOpen(tLibRegisterEvtHandler evtHandler);
uint8_t	apiProtectDotpGetFlag(uint16_t NtcIndex);
uint8_t apiProtectDotpHandler(uint8_t ProtectLevel);

 
 
 
 










     


#line 23 "..\\..\\..\\AppProtect\\ApiProtectDotp.c"
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







	



     


#line 24 "..\\..\\..\\AppProtect\\ApiProtectDotp.c"

void appSerialCanDavinciSendTextMessage(char *str);



 


 
 
typedef struct{
	uint8_t	Flag[400];
	uint8_t	SetCount[3][400];
	uint8_t	ReleaseCount[3][400];
	tLibRegisterEvtHandler  EvtHandler;
}tDotpProtect;

static tDotpProtect	mDotpProtect={0};
static	uint16_t	DotpNtcIndex;
 
 
static void dotpProtectIni(void)
{
	DotpNtcIndex = 0;	
}
 

 
uint8_t	apiProtectDotpGetFlag(uint16_t NtcIndex)
{
	return mDotpProtect.Flag[NtcIndex];
}


uint8_t apiProtectDotpHandler(uint8_t ProtectLevel)
{
	BYTE	str[200];
	uint8_t	checkcount;


	static 	uint8_t		flag = 0;
	uint16_t		NtcAdcValue;
	tProtectFlagValue	ProtectFlagValue;
	tScuProtectPar		ProtectPar;
	
	apiSysParGetDotpProtectPar(ProtectLevel, &ProtectPar);
	appProtectGetLevelMask(ProtectLevel, &ProtectFlagValue);








	while(1)
	{			
		NtcAdcValue = HalAfeGetNtcAdc(DotpNtcIndex);





		if(appProtectIsOverTemperter(NtcAdcValue, ProtectPar.SetValue.l))
		{
			if((mDotpProtect.Flag[DotpNtcIndex] & ProtectFlagValue.Mask) == 0)
			{
				mDotpProtect.Flag[DotpNtcIndex] &= ProtectFlagValue.ClearMask;
				mDotpProtect.Flag[DotpNtcIndex] |= ProtectFlagValue.Setting;
				mDotpProtect.SetCount[ProtectLevel][DotpNtcIndex] = 1;
			}	
			else if((mDotpProtect.Flag[DotpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
			{
				mDotpProtect.SetCount[ProtectLevel][DotpNtcIndex]++;
				if(mDotpProtect.SetCount[ProtectLevel][DotpNtcIndex] >= ProtectPar.STime.l)
				{
					mDotpProtect.Flag[DotpNtcIndex] &= ProtectFlagValue.ClearMask;
					mDotpProtect.Flag[DotpNtcIndex] |= ProtectFlagValue.Setted;
					mDotpProtect.SetCount[ProtectLevel][DotpNtcIndex] = 0;

					if(mDotpProtect.EvtHandler)
					{
						mDotpProtect.EvtHandler(0, APP_PROTECT_DOTP_L1_SET + ProtectLevel, &DotpNtcIndex);
					}
				}
			}
		}
		else if((mDotpProtect.Flag[DotpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setting)
		{
			mDotpProtect.Flag[DotpNtcIndex] &= ProtectFlagValue.ClearMask;
		}
		
		
		if(appProtectIsUnderTemperter(NtcAdcValue, ProtectPar.RelValue.l))
		{
			if((mDotpProtect.Flag[DotpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Setted)
			{
				mDotpProtect.Flag[DotpNtcIndex] &= ProtectFlagValue.ClearMask;
				mDotpProtect.Flag[DotpNtcIndex] |= ProtectFlagValue.Releasing;
				mDotpProtect.ReleaseCount[ProtectLevel][DotpNtcIndex] = 1;
			}	
			else if((mDotpProtect.Flag[DotpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
			{
				mDotpProtect.ReleaseCount[ProtectLevel][DotpNtcIndex]++;
				if(mDotpProtect.ReleaseCount[ProtectLevel][DotpNtcIndex] >= ProtectPar.RTime.l)
				{
					mDotpProtect.Flag[DotpNtcIndex] &= ProtectFlagValue.ClearMask;
					mDotpProtect.ReleaseCount[ProtectLevel][DotpNtcIndex] = 0;
					if(mDotpProtect.EvtHandler)
					{
						mDotpProtect.EvtHandler(0, APP_PROTECT_DOTP_L1_RELEASE + ProtectLevel, &DotpNtcIndex);
					}
				}
			}
		}
		else if((mDotpProtect.Flag[DotpNtcIndex] & ProtectFlagValue.Mask) == ProtectFlagValue.Releasing)
		{
			mDotpProtect.Flag[DotpNtcIndex] &= ProtectFlagValue.ClearMask;
			mDotpProtect.Flag[DotpNtcIndex] |= ProtectFlagValue.Setted;
		}
		DotpNtcIndex++;
		if(DotpNtcIndex >= apiSysParGetNtcNumber())
		{
			DotpNtcIndex = 0;
			return 1;
		}
		checkcount++;
		if(checkcount >= 20)
			break;
	}

	return 0;
}

void apiProtectDotpOpen(tLibRegisterEvtHandler evtHandler)
{
	dotpProtectIni();
	
	mDotpProtect.EvtHandler = evtHandler;
}

     


/*
  ******************************************************************************
  * @file        HalAfeBq796xx.c
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/12/07
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "LibDebug.h"
#include "halafe.h"
#include "smp_drv_bq796xx.h"
#include "smp_uart.h"
#include "LibSwTimer.h"
#include "LibHwTimer.h"
#include "LibNtc.h"
#include "ApiSysPar.h"
#include "AppProject.h"

//#define	BALANCE_DEBUG		
//#define	AFE_DEBUG_MODE
#define	AFE_DEBUG_N_NUM		2
#define	AFE_DEBUG_S_NUM		2

void appSerialCanDavinciSendTextMessage(char *msg);
#define	halAfeBq796xxDebugMsg(str)	//appSerialCanDavinciSendTextMessage(str)

/* Private define ------------------------------------------------------------*/
#define	afeCommL1Time()					5
#define	afeCommL2Time()					20

#define	NTC_CH_NUM						16
#define	AFE_READ_CYCLE_COUNT_10MS		50

#define	PASER_COUNT_PER_ONE_TIMES		3

#define	afeBmuNumber()					apiSysParGetBmuNumber()

//#define	CHANGE_BRIDGE_DIRECTION()		{BridgeDirection ^= 0x01,GPIOD->ODR ^= GPIO_PIN_13;}
#define	CHANGE_BRIDGE_DIRECTION()		{BridgeDirection ^= 0x01;}

#define	AFE_COMM_FLAG_L1		0x01
#define	AFE_COMM_FLAG_L2		0x02
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
tAfeEvtHandler afeEvtHandler;


typedef void (* tAfeFunctionTable)(void);

tAfeFunctionTable	subFunctionPointer;
void (*AfeTimerHandlerProcessor)(uint16_t evt) = {0};

void (*AfeFunctionProcessor)(void) = {0};

static void AfeBq796xxIniHandler(uint16_t evt);
static void changeToBq796xxIniHandler(void);

/* Public variables ---------------------------------------------------------*/
extern bq796xx_data_t bq796xx_data;

/* Private variables ---------------------------------------------------------*/
#define AFE_PARAM_LOAD_VALUE    {                                                          \
							          .ov_threshold         =OV_4175MV,                      \
	                                  .ov_step_threshold    =OV_STEP_100MV,                  \
	                                  .uv_threshold         =UV_2800MV,                      \
	                                  .ot_threshold         =OT_10_PCT,                      \
	                                  .ut_threshold         =UT_80_PCT,                      \
	                                  .ov_enable            =BQ_ENABLE,                      \
	                                  .uv_enable            =BQ_ENABLE,                      \
		                                .ot_enable            =BQ_DISABLE,                      \
	                                  .ut_enable            =BQ_DISABLE,                      \
		                                .cb_cycle_time        =CB_CYC_T_10S,                   \
                                }	

bq796xx_init_default_t afe_load_data = AFE_PARAM_LOAD_VALUE;

bq796xx_init_steps_enum afe_steps = AFE_INIT_IDLE;
uint8_t step_com_f=0;
uint8_t before_d_ms=0;
uint16_t bq_count_temp=0;
uint16_t  cnt_delay=0;
static uint8_t	AfeCommTimeOutCount = 0;
static uint8_t	AfeCommFlag = 0;
static uint16_t	BalanceData[64];

static uint16_t	AfeFailTimerCount = 0;
static uint8_t	IsBalanceEnable = 1;

static uint8_t	PaserCount;
static uint8_t	PaserIndex;
static uint8_t	BalanceOnFlag = 0;
static uint8_t	BalanceOddFlag = 0;
static uint8_t	BridgeDirection = 0;

static uint8_t	BridgeSwitchCount =0;
static uint8_t	AfeState = AFE_STATE_INI;
static uint8_t	StackCountForIni[2] = {0};
static uint8_t	PaserCountPerOneTimes;
static uint8_t	NorthAndSouthIniFlag = 0;
static uint8_t	RealPaserCount[2] = {0};
static uint8_t	PaserCountFailCount[2] ={0};
static uint16_t	ReaAfeCycleCount10Ms = AFE_READ_CYCLE_COUNT_10MS;
static uint8_t	MainFunIndex = 0;
static uint8_t	IsRingDaisyChain;
bq796xx_wake_tone_switch wake_sw = WAKE_TONE_ENABLE;

//static uint
static uint32_t	ReadCount = 0;
static uint32_t	NortnResponseCount = 0;
static uint32_t	SouthResponseCount = 0;
static uint32_t	AfeIniCount = 0;

/* Private function prototypes -----------------------------------------------*/
uint8_t drv_bq796xx_clean_fifo(void);

static void AfeBq796xxNormalHandler(uint16_t evt);

static void finishAfeReadProcessor(void);
static void changeBmuDirForDataReadProcessor(void);

static void startBalance(void);
static void stopBalanceLoCell(void);
static void stopBalanceHiCell(void);
static void stopBalanceActive(void);

const tAfeFunctionTable	afeMainFunctionTable[];

static uint8_t	delayCount;

static void getNextMainFunctionPointer(void)
{
	MainFunIndex++;
	AfeFunctionProcessor = afeMainFunctionTable[MainFunIndex];
	if(AfeFunctionProcessor)
		AfeFunctionProcessor();
}
static void getNextSubFunctionPointer(void)
{
	subFunctionPointer =(tAfeFunctionTable)((uint32_t)subFunctionPointer+4);
	AfeFunctionProcessor = (tAfeFunctionTable )(*((uint32_t *)subFunctionPointer));
} 

static void afeFunNone(void)
{
	getNextMainFunctionPointer();
}


const tAfeFunctionTable	stopBalanceFunctionTable[]={
	stopBalanceLoCell,
	stopBalanceHiCell,
	stopBalanceActive,
	getNextMainFunctionPointer
};

static void mainFunStopBalance(void)
{
	ReadCount++;

	/*
	if(IsBalanceEnable == 0)
	{
		halAfeBq796xxDebugMsg("mainFunStopBalance");
		getNextMainFunctionPointer();
	}
	*/
	//halAfeBq796xxDebugMsg("mainFunStopBalance");
	subFunctionPointer = (tAfeFunctionTable )stopBalanceFunctionTable;	
	AfeFunctionProcessor = (tAfeFunctionTable )(*((uint32_t *)subFunctionPointer));
}
static uint8_t	ntcIoIndex = 0;
static uint8_t	ntcChannelStartIndex = 0;
static uint8_t	ntcChannelOffset = 0;
static uint8_t	NtcChannelSelect = 0;
static void setNtcChannel(void)
{
	char	str[100];

	uint16_t test_gpio_HL=0;
	
	NtcChannelSelect = ntcChannelStartIndex + ntcChannelOffset;
	if(NtcChannelSelect >= NTC_CH_NUM)
		NtcChannelSelect -= NTC_CH_NUM;

	if(ntcIoIndex == 0)
	{
		//sprintf(str,"Dir = %d NTC Channel = %d",BridgeDirection, NtcChannelSelect);
		//halAfeBq796xxDebugMsg(str);
		
	    test_gpio_HL = 5- (NtcChannelSelect & 0x01);	
		drv_bq796xx_Set_AFE_GPIO_type(STACK, 0x00, test_gpio_HL, AFE_GPIO4,0);//BQ796XX_CMD_DELAY_MS);
	}
	else if(ntcIoIndex == 1)
	{
		test_gpio_HL = 5-((NtcChannelSelect & 0x02)>>1);
		drv_bq796xx_Set_AFE_GPIO_type(STACK, 0x00, test_gpio_HL, AFE_GPIO5,0);//BQ796XX_CMD_DELAY_MS);	
	}
	else if(ntcIoIndex == 2)
	{	
		test_gpio_HL = 5-((NtcChannelSelect & 0x04)>>2);
		drv_bq796xx_Set_AFE_GPIO_type(STACK, 0x00, test_gpio_HL, AFE_GPIO6,0);//BQ796XX_CMD_DELAY_MS);
	}
	else if(ntcIoIndex == 3)
	{
		test_gpio_HL = 5-((NtcChannelSelect & 0x08)>>3);
		drv_bq796xx_Set_AFE_GPIO_type(STACK, 0x00, test_gpio_HL, AFE_GPIO7,0);//BQ796XX_CMD_DELAY_MS);
	}
	
	ntcIoIndex++;
	if(ntcIoIndex >= 4)
	{
		ntcChannelOffset++;
		ntcIoIndex = 0;
		delayCount = 10;
	}
	getNextSubFunctionPointer();
}
static void delayFunction(void)
{
	if(delayCount)
		delayCount--;
	if(!delayCount)
		getNextSubFunctionPointer();
}

static void sendOutReadGpioCommand(void)
{
	//halAfeBq796xxDebugMsg("sendOutReadGpioCommand");
	
	drv_bq796xx_clean_fifo();
	drv_bq796xx_Read_AFE_ALL_ADC(STACK, 0, 0);//BQ796XX_CMD_DELAY_MS);         //Stack(BMU #1,#2) Read all AUX ADC   (GPIO1~8).
	
	delayCount = 1 + StackCountForIni[BridgeDirection] / 4;
	PaserCount = StackCountForIni[BridgeDirection];
	PaserIndex = 0;
	PaserCountPerOneTimes = PaserCount;
	RealPaserCount[BridgeDirection] = 0;
	getNextSubFunctionPointer();
}
static void paserAfeResponse(void)
{
	uint8_t res,i;
	
	if(PaserCountPerOneTimes == 0) 
	{
		getNextSubFunctionPointer();
		return;
	}
	for(i=0; i<PaserCountPerOneTimes; i++)
	{
//		GPIOD->ODR |= GPIO_PIN_15;
		//GPIOD->ODR ^= GPIO_PIN_14;
		res = drv_bq796xx_data_frame_parser();
//		GPIOD->ODR &= ~GPIO_PIN_15;

		if(res == BQ796XX_RES_OK)
		{
			RealPaserCount[BridgeDirection]++;
		}
		//	break;
		PaserIndex++;
		if(PaserIndex >= PaserCount)	//
		{
			getNextSubFunctionPointer();
			return;
		}
	}
}



static void mainFunStartGetNtc(void)
{
	ntcChannelOffset = 0;
	getNextMainFunctionPointer();
}
static void mainFunChangeChannel(void)
{
	if(StackCountForIni[BridgeDirection] < afeBmuNumber())
	{
		if(BridgeDirection == DIR_NORTH)
		{
			ntcChannelStartIndex += ntcChannelOffset;
			if(ntcChannelStartIndex >= NTC_CH_NUM)
				ntcChannelStartIndex -= NTC_CH_NUM;
			ntcChannelOffset = 0;
		}
	}
	else
	{
		ntcChannelStartIndex += ntcChannelOffset;
		if(ntcChannelStartIndex >= NTC_CH_NUM)
			ntcChannelStartIndex -= NTC_CH_NUM;
		ntcChannelOffset = 0;
	}			
	getNextMainFunctionPointer();
}

const tAfeFunctionTable	getNtcFunctionTable[]={
	setNtcChannel,
	setNtcChannel,
	setNtcChannel,
	setNtcChannel,
	delayFunction,
	sendOutReadGpioCommand,
	delayFunction,
	paserAfeResponse,
	getNextMainFunctionPointer
};
static void mainFunGetNtc(void)
{
	ntcIoIndex = 0;
	//halAfeBq796xxDebugMsg("mainFunGetNtc");
	subFunctionPointer = (tAfeFunctionTable )getNtcFunctionTable;	
	AfeFunctionProcessor = (tAfeFunctionTable )(*((uint32_t *)subFunctionPointer));
}

static void sendOutReadCellVoltageCommand(void)
{
	drv_bq796xx_clean_fifo();
	drv_bq796xx_Read_AFE_ALL_VCELL(STACK, 0, 0);//BQ796XX_CMD_DELAY_MS);       //Stack(BMU #1,#2) Read all VCell 1~16(Main ADC).
		
	delayCount = 1 + (StackCountForIni[BridgeDirection] / 2);
	PaserCount = StackCountForIni[BridgeDirection];
	PaserIndex = 0;
	PaserCountPerOneTimes = PASER_COUNT_PER_ONE_TIMES;
	RealPaserCount[BridgeDirection] = 0;
	getNextSubFunctionPointer();
}


const tAfeFunctionTable	getCellVoltageFunctionTable[]={
	sendOutReadCellVoltageCommand,
	delayFunction,
	paserAfeResponse,
	getNextMainFunctionPointer
};

static void mainFunGetCellVoltage(void)
{
	//halAfeBq796xxDebugMsg("mainFunGetCellVoltage");
	subFunctionPointer = (tAfeFunctionTable )getCellVoltageFunctionTable;	
	AfeFunctionProcessor = (tAfeFunctionTable )(*((uint32_t *)subFunctionPointer));
}

static void sendOutReadSummaryCommand(void)
{
	drv_bq796xx_clean_fifo();
	drv_bq796xx_Read_Stack_FaultSummary(0);//BQ796XX_CMD_DELAY_MS);            //Stack Read fault summary.

	delayCount = 1 + (StackCountForIni[BridgeDirection] / 8);
	PaserCount = StackCountForIni[BridgeDirection];
	PaserIndex = 0;
	PaserCountPerOneTimes = PaserCount;
	RealPaserCount[BridgeDirection] = 0;
	getNextSubFunctionPointer();
}


const tAfeFunctionTable	getSummaryFunctionTable[]={
	sendOutReadSummaryCommand,
	delayFunction,
	paserAfeResponse,
	getNextMainFunctionPointer
};

static void mainFunGetSummary(void)
{
	//halAfeBq796xxDebugMsg("mainFunGetSummary");
	subFunctionPointer = (tAfeFunctionTable )getSummaryFunctionTable;	
	AfeFunctionProcessor = (tAfeFunctionTable )(*((uint32_t *)subFunctionPointer));
}


static void sendOutReadOvUvFlagCommand(void)
{	
	drv_bq796xx_clean_fifo();
	drv_bq796xx_Read_Stack_FaultOVUV(0);//BQ796XX_CMD_DELAY_MS);               //Stack Reaf fault OV, UV Cell Detection.

	delayCount = 1 + (StackCountForIni[BridgeDirection] / 8);
	PaserCount = StackCountForIni[BridgeDirection];
	PaserIndex = 0;
	PaserCountPerOneTimes = PaserCount;
	RealPaserCount[BridgeDirection] = 0;
	getNextSubFunctionPointer();
}

const tAfeFunctionTable	getOvUvFlagFunctionTable[]={
	sendOutReadOvUvFlagCommand,
	delayFunction,
	paserAfeResponse,
	getNextMainFunctionPointer
};

static void mainFunGetOvUvFlag(void)
{
	//halAfeBq796xxDebugMsg("mainFunGetOvUvFlag");
	subFunctionPointer = (tAfeFunctionTable )getOvUvFlagFunctionTable;	
	AfeFunctionProcessor = (tAfeFunctionTable )(*((uint32_t *)subFunctionPointer));
}

static void sendOutReadOtUtFlagCommand(void)
{
	drv_bq796xx_clean_fifo();
	drv_bq796xx_Read_Stack_FaultOTUT(0);//BQ796XX_CMD_DELAY_MS);

	delayCount = 1 + (StackCountForIni[BridgeDirection] / 8);
	PaserCount = StackCountForIni[BridgeDirection];
	PaserIndex = 0;
	PaserCountPerOneTimes = PaserCount;
	RealPaserCount[BridgeDirection] = 0;
	getNextSubFunctionPointer();
}


const tAfeFunctionTable	getOtUtFlagFunctionTable[]={
	sendOutReadOtUtFlagCommand,
	delayFunction,
	paserAfeResponse,
	getNextMainFunctionPointer
};

static void mainFunGetOtUtFlag(void)
{
	//halAfeBq796xxDebugMsg("mainFunGetOtUtFlag");
	subFunctionPointer = (tAfeFunctionTable )getOtUtFlagFunctionTable;	
	AfeFunctionProcessor = (tAfeFunctionTable )(*((uint32_t *)subFunctionPointer));
}

const tAfeFunctionTable	changeDirectionFunctionTable[]={
	finishAfeReadProcessor,
	changeBmuDirForDataReadProcessor,
	
	getNextMainFunctionPointer,
};


static void mainFunFinishAfeRead(void)
{
	//halAfeBq796xxDebugMsg("mainFunFinishAfeRead");
	subFunctionPointer = (tAfeFunctionTable )changeDirectionFunctionTable;	
	AfeFunctionProcessor = (tAfeFunctionTable )(*((uint32_t *)subFunctionPointer));
}


static uint8_t	BalanceBmuIndex = 0;

static uint8_t isLastBmuForBalance(void)
{
	BalanceBmuIndex++;
	if(BalanceBmuIndex >= StackCountForIni[BridgeDirection])
	{
		BalanceBmuIndex = 0;
		return 1;
	}
	return 0;
}


static void sendOutBalanceDataLo(void)
{
	char	str[100];
	uint8_t		dat;
	uint8_t		bmuindex;
	uint8_t		RealBalanceIndex;
	//BalanceOnFlag = 1;
	if(BalanceOnFlag == 0)
	{
		IsBalanceEnable = 0;
		getNextMainFunctionPointer();
		return;
	}
	for(bmuindex=0; bmuindex<afeBmuNumber(); bmuindex++)
	{
		if(BridgeDirection == DIR_NORTH)
			RealBalanceIndex = BalanceBmuIndex;
		else
			RealBalanceIndex = afeBmuNumber() - BalanceBmuIndex - 1;
		
#ifdef AFE_DEBUG_MODE			
		dat = (uint8_t)(BalanceData[RealBalanceIndex] & 0x00FF);
#else
		if(BalanceOddFlag)
			dat = (uint8_t)(BalanceData[RealBalanceIndex] & 0x0055);
		else
			dat = (uint8_t)(BalanceData[RealBalanceIndex] & 0x00AA);
#endif			
		sprintf(str, "Dir=%d Bmu#=%d Odd=%d Bmu=%d Balance Lo = %.2X",
					BridgeDirection,
					StackCountForIni[BridgeDirection],
					BalanceOddFlag,
					BalanceBmuIndex + 1, 
					dat);
		//halAfeBq796xxDebugMsg(str);
		if(dat)
		{
			IsBalanceEnable = 1;
			drv_bq796xx_CellBalance_1to8_set(BalanceBmuIndex + 1, dat, CB_TIME_10S, 0);
		}
		if(isLastBmuForBalance())
		{
			//halAfeBq796xxDebugMsg("Next sub fun-L");
			getNextSubFunctionPointer();
			break;
		}
		if(dat)
			break;
	}
}

static void sendOutBalanceDataHi(void)
{
	char	str[100];
	uint8_t		dat;
	uint8_t		bmuindex;
	uint8_t		RealBalanceIndex;
	
	if(BalanceOnFlag == 0)
	{
		IsBalanceEnable = 0;
		getNextMainFunctionPointer();
		return;
	}
	for(bmuindex=0; bmuindex<afeBmuNumber(); bmuindex++)
	{
		if(BridgeDirection == DIR_NORTH)
			RealBalanceIndex = BalanceBmuIndex;
		else
			RealBalanceIndex = afeBmuNumber() - BalanceBmuIndex - 1;	
#ifdef AFE_DEBUG_MODE
		dat =(uint8_t)((BalanceData[RealBalanceIndex] & 0xFF00) / 0x100);
#else		
		if(BalanceOddFlag)
			dat =(uint8_t)((BalanceData[RealBalanceIndex] & 0x5500) / 0x100);
		else
			dat =(uint8_t)((BalanceData[RealBalanceIndex] & 0xAA00) / 0x100);
#endif			
		sprintf(str, "Dir=%d Bmu#=%d Odd=%d Bmu=%d Balance Hi = %.2X",
					BridgeDirection,
					StackCountForIni[BridgeDirection],
					BalanceOddFlag,
					BalanceBmuIndex + 1, 
					dat);
		//halAfeBq796xxDebugMsg(str);
		if(dat)
		{
			IsBalanceEnable = 1;
			drv_bq796xx_CellBalance_9to16_set(BalanceBmuIndex + 1, dat, CB_TIME_10S, 0);
		}
		if(isLastBmuForBalance())
		{
			//halAfeBq796xxDebugMsg("Next sub fun-H");
			getNextSubFunctionPointer();
			break;
		}
		if(dat)
			break;
	}
	
}



const tAfeFunctionTable	startBalanceFunctionTable[]={
	sendOutBalanceDataLo,
	sendOutBalanceDataHi,
	startBalance,
	getNextMainFunctionPointer
};

static void mainFunStartBalance(void)
{
	if(StackCountForIni[BridgeDirection] < afeBmuNumber())
	{
		if(BridgeDirection == DIR_NORTH)
		{
			BalanceOddFlag ^= 0x01;
			//halAfeBq796xxDebugMsg("Setup Odd Even...1");
		}
	}
	else
	{
		BalanceOddFlag ^= 0x01;
		//halAfeBq796xxDebugMsg("Setup Odd Even...2");
	}			
	BalanceBmuIndex = 0;
	
	//halAfeBq796xxDebugMsg("mainFunStartBalance");
	subFunctionPointer = (tAfeFunctionTable )startBalanceFunctionTable;	
	AfeFunctionProcessor = (tAfeFunctionTable )(*((uint32_t *)subFunctionPointer));
}






uint16_t CvtCellAdcToMv(uint16_t adc)
{
	double	d;
	d = adc;
	d *= BQ79656_RESOLUTION_CELL_MAIN;
	return (uint16_t) d;	
}
uint16_t CvtNtcAdcToMv(uint16_t adc)
{
	double	d;
	d = adc;
	d *= BQ79656_RESOLUTION_GPIO;
	return (uint16_t) d;	
}

static void DecodeCellVoltageByCellFlag(bq796xx_data_t *bq_data)
{
	uint8_t		BmuIndex;
	uint8_t		CellChannel;
	uint32_t	CellFlag;
	uint16_t	LogicCellIndex = 0;
	uint16_t	adc;
	uint16_t	voltage;
	uint8_t		paser_id;
	
	if(bq_data->paser_id == 0)
		return;
	if(appProjectIsInSimuMode())
		return;

	paser_id = bq_data->paser_id -1;
	
	for(BmuIndex=0; BmuIndex<afeBmuNumber(); BmuIndex++)
	{
		if(BmuIndex > bq_data->paser_id)
			break;
			
		CellFlag = apiSysParGetCellFlag(BmuIndex);
		for(CellChannel=0; CellChannel<16; CellChannel++)
		{
			if(CellFlag & 0x01)
			{
				if(BmuIndex == paser_id)
				{
					adc = bq_data->vcell_data[BmuIndex][CellChannel];
					voltage = CvtCellAdcToMv(adc);
					halAfeSetCellVoltage(LogicCellIndex, voltage);
				}
				LogicCellIndex++;
			}
			CellFlag >>= 1;
		}
	}
}
static void DecodeNtcVoltageByNtcFlag(bq796xx_data_t *bq_data,uint8_t NtcDataChannel)
{
	uint8_t		BmuIndex;
	static uint8_t		NtcChannel;
	uint32_t	NtcFlag;
	uint16_t	LogicNtcIndex = 0;
	uint16_t	adc;
	uint16_t	voltage;
	uint16_t	mask;
	uint16_t	MaskChk;
	uint8_t		paser_id;
	char	str[100];
	
	if(bq_data->paser_id == 0)
		return;
	if(appProjectIsInSimuMode())
		return;
		
	paser_id = bq_data->paser_id -1;
	mask = 1 << NtcDataChannel;
	for(BmuIndex=0; BmuIndex<afeBmuNumber(); BmuIndex++)
	{
		if(BmuIndex > bq_data->paser_id)
			break;
		
		NtcFlag = apiSysParGetNtcFlag(BmuIndex);
		for(NtcChannel=0; NtcChannel<16; NtcChannel++)
		{
			MaskChk = 1 << NtcChannel;
			if(NtcFlag & MaskChk)
			{				
				if(BmuIndex == paser_id && (MaskChk & mask))
				{
					adc = bq_data->gpio_data[BmuIndex][0];
					voltage = CvtNtcAdcToMv(adc);
					halAfeSetNtcAdcData(LogicNtcIndex, voltage);
				}
				LogicNtcIndex++;
			}
		}
	}
}

uint8_t app_afe_cb(bq796xx_data_t *bq_data, bq796xx_event_cb_type bq_event){

//	static bq796xx_event_cb_type temp;
//  	static bq796xx_data_t temp_afe_data;
//	static uint8_t bq_event_buf[256]={0};
//	static uint8_t bq_event_buf_c = 0;
	char	str[100];
//	uint8_t		temp_ch;

//	halAfeBq796xxDebugMsg("Bq796xx CB");
	GPIOD->ODR ^= GPIO_PIN_14;
 	switch(bq_event)
 	{
 	case BQ_EVENT_VCELL:
		//halAfeBq796xxDebugMsg("Bq796xx CB VCell");
		DecodeCellVoltageByCellFlag(bq_data);
		break;
 	case BQ_EVENT_GPIO_ADC:
 		//halAfeBq796xxDebugMsg("Gpio");
 		DecodeNtcVoltageByNtcFlag(bq_data, NtcChannelSelect);
// 	          bq796xx_data.gpio_data[device_id-1][i]=bq796xx_res_buf[BQ796XX_DF_REG_PAYLOAD+(i*2)] * 256.0f + bq796xx_res_buf[BQ796XX_DF_REG_PAYLOAD+(i*2+1)];
//
// 		temp_ch = AfeFunctionStep - 1;
// 		temp_ch &= 0x0f;
// 		sprintf(str,"
//		halAfeBq796xxDebugMsg("Bq796xx CB Gpio");
		break;
 	case BQ_EVENT_FAULT:
 	//	sprintf(str,"Fault Flag = %.4X %.4X", 
 	//			bq_data->fault_summary[0],
 	//			bq_data->fault_summary[1]);
		//halAfeBq796xxDebugMsg(str);
//		halAfeBq796xxDebugMsg("Bq796xx CB Fault");
		break;
 	case BQ_EVENT_FAULTOVUV:
  		sprintf(str,"OV UP Flag = %.4X %.4X %.4X %.4X", 
 				bq_data->fault_ov[0],
 				bq_data->fault_uv[0],
 				bq_data->fault_ov[1],
 				bq_data->fault_uv[1]);
 		//halAfeBq796xxDebugMsg(str);
		break;
 	case BQ_EVENT_FAULTOTUT:
  		sprintf(str,"OT UT Flag = %.4X %.4X %.4X %.4X", 
 				bq_data->fault_ot[0],
 				bq_data->fault_ut[0],
 				bq_data->fault_ot[1],
 				bq_data->fault_ut[1]);
 		//halAfeBq796xxDebugMsg(str);
		break;
 	case BQ_EVENT_OTHER_ERR:
		//halAfeBq796xxDebugMsg("Bq796xx CB Other");
		break;
	default:
		//halAfeBq796xxDebugMsg("Bq796xx CB Undefine");
		break;
	}	 
	 
//	temp = bq_event;
//	temp_afe_data = *bq_data;
	
//	bq_event_buf[ bq_event_buf_c++ ] = temp;
//	if(bq_event_buf_c > 255) 
//		bq_event_buf_c=0;
	
	return 0;
}

#define	CHECK_AFE_TIMES		6

static void changeDirIni(void)
{
	cnt_delay = 0;
	afe_steps = AFE_INIT_IDLE;
}

static void finishAfeReadProcessor(void)
{
	char	str[100];
	static	uint8_t	fail_count = 0;
	static  uint8_t	run_count = 0;
	
	sprintf(str,"finish afe read:Dir=%d Error:%d/%d Num=%d %d" , 
			BridgeDirection, fail_count, run_count,
			StackCountForIni[0],
			StackCountForIni[1]
			
			);		
	halAfeBq796xxDebugMsg(str);

#ifdef	AFE_DEBUG_MODE
	RealPaserCount[DIR_NORTH] = AFE_DEBUG_N_NUM;
	RealPaserCount[DIR_SOUTH] = AFE_DEBUG_S_NUM;
#endif
	run_count++;

	if(StackCountForIni[BridgeDirection] < afeBmuNumber())
	{	
		if((StackCountForIni[DIR_NORTH]+StackCountForIni[DIR_SOUTH]) <
			afeBmuNumber())
		{
			fail_count++;
			if(run_count >= CHECK_AFE_TIMES && fail_count >= (CHECK_AFE_TIMES/2))
			{
#if	1				
				halAfeBq796xxDebugMsg("重新 Ini AFE");

				//AfeState = AFE_STATE_INI;
				//cnt_delay = 0;
				//afe_steps = AFE_INIT_IDLE;
				//BridgeDirection = DIR_SOUTH;//DIR_SOUTH;//DIR_NORTH;
				//BridgeSwitchCount =0;
				//NorthAndSouthIniFlag = 0;
				changeToBq796xxIniHandler();
				//AfeTimerHandlerProcessor = AfeBq796xxIniHandler;
				AfeFailTimerCount = 0;
				fail_count = 0;
				run_count = 0;
				return;
#endif				
			}
		}
		else
		{
			if(BridgeDirection == DIR_NORTH)
			{
				if(RealPaserCount[DIR_NORTH] == StackCountForIni[DIR_NORTH])
					NortnResponseCount++;					
			}
			else
			{
				if(RealPaserCount[DIR_SOUTH] == StackCountForIni[DIR_SOUTH])
					SouthResponseCount++;
			}				
			AfeCommTimeOutCount = 0;
			fail_count = 0;
			run_count = 0;
		}
		ReaAfeCycleCount10Ms = AFE_READ_CYCLE_COUNT_10MS / 2;
		if(BridgeDirection == DIR_NORTH)
		{
			halAfeBq796xxDebugMsg("北橋資料不足");
		}
		else
		{
			halAfeBq796xxDebugMsg("南橋資料不足");
		}
		CHANGE_BRIDGE_DIRECTION();
		
		changeDirIni();
		getNextSubFunctionPointer();
	}
	else
	{		
		ReaAfeCycleCount10Ms = AFE_READ_CYCLE_COUNT_10MS;
		sprintf(str,"Paser Count %d = %d",BridgeDirection, RealPaserCount[BridgeDirection]);
		halAfeBq796xxDebugMsg(str);
		if(RealPaserCount[BridgeDirection] != StackCountForIni[BridgeDirection])
		{
			PaserCountFailCount[BridgeDirection]++;
			if(PaserCountFailCount[BridgeDirection] >= CHECK_AFE_TIMES)
			{
				StackCountForIni[BridgeDirection] = 0;
				CHANGE_BRIDGE_DIRECTION();
				halAfeBq796xxDebugMsg("切換方向");
				changeDirIni();
				getNextSubFunctionPointer();
			}
#if	0			
			fail_count++;
			if(run_count >= CHECK_AFE_TIMES && fail_count >= (CHECK_AFE_TIMES/2))
			{
				halAfeBq796xxDebugMsg("重新 Ini ");
				//afe_steps = AFE_INIT_IDLE;
				//BridgeDirection = DIR_SOUTH;//DIR_SOUTH;//DIR_NORTH;
				//BridgeSwitchCount =0;
				//NorthAndSouthIniFlag = 0;
				//AfeState = AFE_STATE_INI;
				//cnt_delay = 0;
				changeToBq796xxIniHandler();
				//AfeTimerHandlerProcessor = AfeBq796xxIniHandler;
				AfeFailTimerCount = 0;
				fail_count = 0;
				run_count = 0;
//				GPIOD->ODR |= GPIO_PIN_14;
				return;
			}
			else
			{
				CHANGE_BRIDGE_DIRECTION();
				changeDirIni();
				AfeFunctionProcessor = changeBmuDirForDataReadProcessor;
			}
#endif			
		}
		else
		{
			if(BridgeDirection == DIR_NORTH)
				NortnResponseCount++;
			else
				SouthResponseCount++;
			AfeCommTimeOutCount = 0;
			getNextMainFunctionPointer();
			
			PaserCountFailCount[BridgeDirection] = 0;
			fail_count = 0;
			run_count = 0;
		}		
	}
}

static void changeBmuDirForDataReadProcessor(void)
{
	static uint8_t	ChangeDirCount = 0;	
	uint8_t	res;
	char	str[100];
	
	if(cnt_delay >= 2)
	{
		cnt_delay--;
		if(cnt_delay)
			return;
	}
	if(afe_steps == 0)
	{
		drv_bq796xx_clean_fifo();
		
		sprintf(str,"Dir = %d,初始化數量= %d", 
					BridgeDirection,
					StackCountForIni[BridgeDirection]);
		halAfeBq796xxDebugMsg(str);
		GPIOD->ODR |= GPIO_PIN_14;
		ChangeDirCount++;
		if(ChangeDirCount >= 10)
		{
			halAfeBq796xxDebugMsg(".........檢查是否接線正常");
			StackCountForIni[BridgeDirection] = 0;
			ChangeDirCount = 0;
		}
	}
	res = drv_bq796xx_direction_set_steps(StackCountForIni[BridgeDirection],  
										  &afe_steps, 
										  afeBmuNumber()+1 , 
										  BridgeDirection, 
										  &step_com_f , 
										  &before_d_ms);
	cnt_delay = before_d_ms;
	if(step_com_f == 1)
	{
		afe_steps++;
	}
	if(afe_steps > SETDIR_AFE_RUN_AUX_ADC)
	{
		if(AfeCommTimeOutCount >= 10)	//over 10 sec not ready
		{
			changeToBq796xxIniHandler();
			return;		
		}	
		
		if(res & 0x80)
		{
			IsRingDaisyChain = 1;
			halAfeBq796xxDebugMsg("................Ring");
		}
		else
		{
			halAfeBq796xxDebugMsg("..........Not...Ring");
			IsRingDaisyChain = 0;
		}
		res &= 0x7f;
				
		GPIOD->ODR &= ~GPIO_PIN_14;
		StackCountForIni[BridgeDirection] = res;

#ifdef AFE_DEBUG_MODE
		StackCountForIni[DIR_NORTH] = AFE_DEBUG_N_NUM;
		StackCountForIni[DIR_SOUTH] = AFE_DEBUG_S_NUM;
		if(BridgeDirection == DIR_NORTH)
			res = AFE_DEBUG_N_NUM;
		else
			res = AFE_DEBUG_S_NUM;
#endif

		if(BridgeDirection == DIR_NORTH)
			sprintf(str,"Get North BMU# = %d", res);
		else
			sprintf(str,"Get South BMU# = %d", res);
		halAfeBq796xxDebugMsg(str);
		if((IsRingDaisyChain && res == afeBmuNumber()) ||
		   (!IsRingDaisyChain && res))
		{
			getNextSubFunctionPointer();
		}
		else
		{
			halAfeBq796xxDebugMsg("重新切方向");
			cnt_delay = 100;
			afe_steps = 0;
			CHANGE_BRIDGE_DIRECTION();
		}
  	}
}

static void stopBalanceLoCell(void)
{
	//halAfeBq796xxDebugMsg("stopBalanceLoCell");
	drv_bq796xx_CellBalance_1to8_clear(STACK, 0x00, 0);
	getNextSubFunctionPointer();
}
static void stopBalanceHiCell(void)
{
	//halAfeBq796xxDebugMsg("stopBalanceHiCell");
	drv_bq796xx_CellBalance_9to16_clear(STACK, 0x00, 0);
	getNextSubFunctionPointer();
}

static void stopBalanceActive(void)
{
	//halAfeBq796xxDebugMsg("stopBalanceActive");
	drv_bq796xx_Stack_CellBalanceStarting(CB_MANUAL, 0);
	getNextSubFunctionPointer();
}


static void startBalance(void)
{
	//halAfeBq796xxDebugMsg("startBalance");

	drv_bq796xx_Stack_CellBalanceStarting(CB_MANUAL, 0);
	getNextSubFunctionPointer();
}



static void mainFunEnd(void)
{
	//halAfeBq796xxDebugMsg("afeFunEnd");
	AfeFunctionProcessor = 0;
}

const tAfeFunctionTable	afeMainFunctionTable[]={
	mainFunStopBalance,
	mainFunStartGetNtc,
	mainFunGetNtc,
	mainFunGetNtc,
	mainFunChangeChannel,
	mainFunGetCellVoltage,
	mainFunStartBalance,
	mainFunGetSummary,
	mainFunGetOvUvFlag,
	mainFunGetOtUtFlag,
	mainFunFinishAfeRead,
	mainFunEnd
};


static void AfeBq796xxNormalHandler(uint16_t evt)
{
//	static uint16_t test_gpio_HL=0;
//	uint8_t res, res_c=0;
	static uint8_t	count;
	//static uint8_t	static;
//	uint8_t	state;
	//GPIOD->ODR |= GPIO_PIN_14;
 
    if(evt == LIB_SW_TIMER_EVT_SW_1MS)
    {
    	if(AfeFunctionProcessor)
			AfeFunctionProcessor();
		//GPIOC->ODR ^= GPIO_PIN_6;
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_9)
	{
    	count++;
   		AfeFailTimerCount++;
		//if(AfeFailTimerCount >= 200)	//time out
		if(0)
		{
			AfeState = AFE_STATE_INI;

			//afe_steps = AFE_INIT_IDLE;
			//cnt_delay = 0;
			changeToBq796xxIniHandler();
			//AfeTimerHandlerProcessor = AfeBq796xxIniHandler;
			AfeFailTimerCount = 0;
		}
		else if(count  >= ReaAfeCycleCount10Ms)
		{
			count = 0;
			if(!AfeFunctionProcessor)
			{
				halAfeBq796xxDebugMsg("Start read AFE");
				MainFunIndex = 0;
				AfeFunctionProcessor = afeMainFunctionTable[0];
			}
		}
	}
	//GPIOD->ODR &= ~GPIO_PIN_14;
}


#define	INI_RETRY_TIMES		5

static void changeToBq796xxIniHandler(void)
{
	BridgeDirection = DIR_SOUTH;//DIR_SOUTH;//DIR_NORTH;
	BridgeSwitchCount =0;
	NorthAndSouthIniFlag = 0;
	AfeState = AFE_STATE_INI;
	cnt_delay = 0;
	wake_sw = WAKE_TONE_ENABLE;
	AfeTimerHandlerProcessor = AfeBq796xxIniHandler;
}

static uint32_t IniCountForTest[2][40] = {0};


static void AfeBq796xxIniHandler(uint16_t evt)
{
	char	str[100];
	uint8_t res;
	//wake_sw
	//WAKE_TONE_DISABLE
	if(appProjectIsInEngMode() && afe_steps == AFE_INIT_IDLE)
		return;
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{  
		GPIOD->ODR ^= GPIO_PIN_14;
		
		if(cnt_delay <= 1)
		{
			if(afe_steps == AFE_INIT_IDLE)
			{
				drv_bq796xx_clean_fifo();
				AfeIniCount++;
			}
			
	        res = drv_bq796xx_Init_Steps(wake_sw,
	        							 &afe_steps, 
	        							 afeBmuNumber() + 1, 
	        						  	 BridgeDirection, 
	        							 &step_com_f , 
	        							 &before_d_ms);
			
	        //sprintf(str,"%d %d %d %d", afe_steps, res,step_com_f, before_d_ms); 
			//halAfeBq796xxDebugMsg(str);
			
			cnt_delay = before_d_ms;
			if(step_com_f == 1)
			{
		   		afe_steps++;
			}
			if(afe_steps > AFE_RUN_AUX_ADC)
			{
				sprintf(str,"Wakeup: %d", wake_sw);
				halAfeBq796xxDebugMsg(str);

				if(res&0x80)
				{
					IsRingDaisyChain = 1;
					halAfeBq796xxDebugMsg("...........Ring...1");
				}
				else
					IsRingDaisyChain = 0;
					
				res &= 0x7f;
			
				IniCountForTest[BridgeDirection][res]++;
				
	//			if(res > 12)
//					GPIOD->ODR ^= GPIO_PIN_15;
				StackCountForIni[BridgeDirection] = res; 
				if(res == afeBmuNumber())
					wake_sw = WAKE_TONE_DISABLE;
				//else
				//	wake_sw = WAKE_TONE_ENABLE;
				
				if(BridgeDirection == DIR_NORTH)
				{
					if(res == afeBmuNumber() ||
					   BridgeSwitchCount >= (INI_RETRY_TIMES - 1))
					{
						NorthAndSouthIniFlag |= 0x01;
					}
					sprintf(str,"North Number:%d", res);
					halAfeBq796xxDebugMsg(str);
				}
				else
				{
					if(res == afeBmuNumber() ||
					   BridgeSwitchCount >= (INI_RETRY_TIMES - 1))
					{
						NorthAndSouthIniFlag |= 0x02;
					}
					sprintf(str,"South Number:%d", res);
					halAfeBq796xxDebugMsg(str);
				}
				if((NorthAndSouthIniFlag&0x03) == 0x03 && 
					( (StackCountForIni[DIR_NORTH] == afeBmuNumber()) ||
					  (StackCountForIni[DIR_SOUTH] == afeBmuNumber()) ||
					  ((StackCountForIni[DIR_NORTH] + StackCountForIni[DIR_SOUTH]) ==
					    afeBmuNumber())))
				{					
					halAfeBq796xxDebugMsg("Start run....");
					if(BridgeDirection == DIR_NORTH && StackCountForIni[DIR_NORTH] == 0)
					{
						afe_steps = AFE_INIT_IDLE;
						cnt_delay = 50;
						BridgeSwitchCount = 0;
						CHANGE_BRIDGE_DIRECTION();
						return;
					}
					else if(BridgeDirection == DIR_SOUTH && StackCountForIni[DIR_SOUTH] == 0)
					{
						afe_steps = AFE_INIT_IDLE;
						cnt_delay = 50;
						BridgeSwitchCount = 0;
						CHANGE_BRIDGE_DIRECTION();
						return;
					}
					if((NorthAndSouthIniFlag & 0x80) == 0)
					{
						NorthAndSouthIniFlag |= 0x80;
						if(StackCountForIni[DIR_NORTH] == StackCountForIni[DIR_SOUTH] &&
						   StackCountForIni[DIR_NORTH] == (afeBmuNumber() / 2))
						{
							halAfeBq796xxDebugMsg("檢查是否為 ring.......");
							afe_steps = AFE_INIT_IDLE;
							cnt_delay = 50;
							wake_sw = WAKE_TONE_DISABLE;
							CHANGE_BRIDGE_DIRECTION();
							return;
						}
					}
					else
					{
						if(IsRingDaisyChain && 
						  (StackCountForIni[DIR_NORTH] != afeBmuNumber() ||
					       StackCountForIni[DIR_SOUTH] != afeBmuNumber()))
					  	{
				  			changeToBq796xxIniHandler();
					  		halAfeBq796xxDebugMsg("接線錯誤，重新Ini......");
					  		return;
					  	}					
					}					
					if(StackCountForIni[DIR_NORTH] == afeBmuNumber() ||
					   StackCountForIni[DIR_SOUTH] == afeBmuNumber())
					{
						ReaAfeCycleCount10Ms = AFE_READ_CYCLE_COUNT_10MS;
					}
					else
					{
						ReaAfeCycleCount10Ms = AFE_READ_CYCLE_COUNT_10MS / 2;
					}
					AfeTimerHandlerProcessor = AfeBq796xxNormalHandler;
					AfeFunctionProcessor = 0;
					AfeFailTimerCount = 0;
					//RealBmuNumber = res;//afeBmuNumber();
					AfeState = AFE_STATE_NORMAL;
				}
				else
				{
					if(StackCountForIni[BridgeDirection] == afeBmuNumber())
					{
						BridgeSwitchCount = INI_RETRY_TIMES;
					}
					
					afe_steps = AFE_INIT_IDLE;
					cnt_delay = 50;
					BridgeSwitchCount++;
					if(BridgeSwitchCount >= INI_RETRY_TIMES)
					{
						BridgeSwitchCount = 0;
						if(BridgeDirection == DIR_NORTH)
							NorthAndSouthIniFlag |= 0x01;
						else
							NorthAndSouthIniFlag |= 0x02;
						CHANGE_BRIDGE_DIRECTION();
						//halAfeBq796xxDebugMsg("AFE Dir Change");
						sprintf(str,"Ini Dir = %d", BridgeDirection);
						halAfeBq796xxDebugMsg(str);
					}
				}
			}
		}
		else 
			cnt_delay--;		
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		static uint8_t	cnt = 0;
		cnt++;
		//if(cnt >= 5)
		{
		//	cnt = 0;
			uint8_t	n;
			
			for(n=0; n< 40; n++)
			{
				if(IniCountForTest[0][n])
				{
					sprintf(str,"N: %d = %d", n, IniCountForTest[0][n]);
					halAfeBq796xxDebugMsg(str);
				}
				if(IniCountForTest[1][n])
				{
					sprintf(str,"S: %d = %d", n, IniCountForTest[1][n]);
					halAfeBq796xxDebugMsg(str);
				}
			}
		}
	}
}

static void afeSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	char	str[100];
	if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		if(appProjectIsInSimuMode())
			AfeCommTimeOutCount  = 0;
		if(AfeCommTimeOutCount<200)
			AfeCommTimeOutCount++;
		if(AfeCommTimeOutCount >= afeCommL1Time())
		{
			 if((AfeCommFlag & AFE_COMM_FLAG_L1) == 0)
			 {
			 	AfeCommFlag |= AFE_COMM_FLAG_L1;
			 	if(afeEvtHandler)
			 		afeEvtHandler(0, AFE_EVT_COMM_L1_SET, 0);
			 }
		}
		else
		{
			if((AfeCommFlag & AFE_COMM_FLAG_L1) != 0 )
			{
			 	AfeCommFlag &= ~AFE_COMM_FLAG_L1;
			 	if(afeEvtHandler)
			 		afeEvtHandler(0, AFE_EVT_COMM_L1_RELEASE, 0);
			}
		}
		if(AfeCommTimeOutCount >= afeCommL2Time())
		{
			if((AfeCommFlag & AFE_COMM_FLAG_L2) == 0)
			{
			 	AfeCommFlag |= AFE_COMM_FLAG_L2;
			 	if(afeEvtHandler)
			 		afeEvtHandler(0, AFE_EVT_COMM_L2_SET, 0);
			}
		}
		else
		{
			if((AfeCommFlag & AFE_COMM_FLAG_L2) != 0)
			{
			 	AfeCommFlag &= ~AFE_COMM_FLAG_L2;
			 	if(afeEvtHandler)
			 		afeEvtHandler(0, AFE_EVT_COMM_L2_RELEASE, 0);
			}
		}
		sprintf(str,"Afe IDLE TIME : %d", AfeCommTimeOutCount);
		halAfeBq796xxDebugMsg(str);
	
		sprintf(str, "N=%d S=%d Read=%d AFEIni=%d",	
				NortnResponseCount, SouthResponseCount,
				ReadCount,
				AfeIniCount
				);
		halAfeBq796xxDebugMsg(str);
	}
	
//	GPIOD->ODR |= GPIO_PIN_15;	//over 1ms per 15ms 
	AfeTimerHandlerProcessor(evt);
//	GPIOD->ODR &= ~GPIO_PIN_15;
}
#define	HW_2ND_PROTECT_PAR_INDEX	0x10	

static void setupAfe2ndOvValue(void)
{
	#define	OV_STEP_MV					25
	tScuProtectPar	ProtectPar;
	uint16_t		setvalue;	
	apiSysParGetOvpPar(HW_2ND_PROTECT_PAR_INDEX, &ProtectPar);

	if(ProtectPar.SetValue.l >= 2700 && ProtectPar.SetValue.l <= 3000)
	{
		setvalue = ProtectPar.SetValue.l - 2700;
		setvalue += (OV_STEP_MV - 1);
		setvalue /= OV_STEP_MV;
		setvalue += OV_2700MV;
	}
	else if(ProtectPar.SetValue.l >= 3500 && ProtectPar.SetValue.l <= 3800)
	{
		setvalue = ProtectPar.SetValue.l - 3500;
		setvalue += (OV_STEP_MV - 1);
		setvalue /= OV_STEP_MV;
		setvalue += OV_3500MV;
	}
	else if(ProtectPar.SetValue.l >= 4175 && ProtectPar.SetValue.l <= 4475)
	{
		setvalue = ProtectPar.SetValue.l - 4175;
		setvalue += (OV_STEP_MV - 1);
		setvalue /= OV_STEP_MV;
		setvalue += OV_4175MV;
	}
	else
		setvalue = 0x3f;
	afe_load_data.ov_threshold = setvalue;		  
}
static void setupAfe2ndUvValue(void)
{
	#define	UV_STEP_MV			50
	tScuProtectPar	ProtectPar;
	uint16_t		setvalue;	
	apiSysParGetUvpPar(HW_2ND_PROTECT_PAR_INDEX, &ProtectPar);

	if(ProtectPar.SetValue.l >= 1200 && ProtectPar.SetValue.l <= 3100)
	{
		setvalue = ProtectPar.SetValue.l - 1200;
		setvalue += (UV_STEP_MV - 1);
		setvalue /= UV_STEP_MV;
	}
	else
		setvalue = UV_2500MV;
	afe_load_data.uv_threshold = setvalue;
}
uint8_t	converterNtcVoltageToPercentage(uint16_t voltage)
{
	uint32_t	r;
	
	r = voltage;
	r *= 100;
	r /= 5000;
	return r;
}

static void setupAfe2ndOtValue(void)
{
	uint16_t	ntc_voltage;
	uint8_t		Percentage;
	tScuProtectPar	ProtectPar;
	char	str[100];
	
	apiSysParGet2ndOtProtectPar(&ProtectPar);
	ntc_voltage = LibTemperatureToVoltage(ProtectPar.SetValue.i[0]);
	
	Percentage = converterNtcVoltageToPercentage(ntc_voltage);
	if(Percentage >= 10 && Percentage<= 39)
	{
		Percentage -= 10;
		Percentage += OT_10_PCT;
	}
	else 
		Percentage = OT_10_PCT;
	afe_load_data.ot_threshold = Percentage;
	
	sprintf(str,"Sen OT = %d",10 + Percentage);
	halAfeBq796xxDebugMsg(str);
// .ot_threshold         =OT_10_PCT,                      \
// .ut_threshold         =UT_80_PCT,                      \

}
static void setupAfe2ndUtValue(void)
{
	uint16_t	ntc_voltage;
	uint8_t		Percentage;
	tScuProtectPar	ProtectPar;
	char	str[100];
	
	apiSysParGet2ndUtProtectPar(&ProtectPar);
	ntc_voltage = LibTemperatureToVoltage(ProtectPar.SetValue.i[0]);
	
	Percentage = converterNtcVoltageToPercentage(ntc_voltage);
	if(Percentage >= 66 && Percentage<= 80)
	{
		Percentage -= 66;
		Percentage /= 2;
		Percentage += UT_66_PCT;
	}
	else 
		Percentage = UT_80_PCT;
	afe_load_data.ut_threshold = Percentage;
	
	sprintf(str,"Sen UT = %d",66 + (Percentage * 2));
	halAfeBq796xxDebugMsg(str);
}

/* Public function prototypes -----------------------------------------------*/
uint8_t halAfeIsL1Protect(void)
{
	if(AfeCommFlag & AFE_COMM_FLAG_L1)
		return 1;
	return 0;
}
uint8_t halAfeIsL2Protect(void)
{
	if(AfeCommFlag & AFE_COMM_FLAG_L2)
		return 1;
	return 0;
}
void halAfeSetPhysicalBalancePosition(uint8_t bmuindex, uint16_t position)
{
#ifdef AFE_DEBUG_MODE	
	return;
#endif	
	BalanceData[bmuindex] = position;
}
void halAfeSetBalanceOnFlag(uint8_t onflag)
{
	BalanceOnFlag = onflag;
}

uint8_t halAfeGetState(void)
{
	if(appProjectIsInSimuMode())
		return AFE_STATE_NORMAL;
	return AfeState;
}

void halAfeClearTestCount(void)
{
	ReadCount = 0;
	NortnResponseCount = 0;
	SouthResponseCount = 0;
	AfeIniCount = 0;
}

void halafeOpen(tAfeEvtHandler evtHandler)
{
	afeEvtHandler = evtHandler;
	AfeState = AFE_STATE_INI;
	AfeFailTimerCount  = 0;
	afe_load_data.bmu_total_num = afeBmuNumber();
	setupAfe2ndOvValue();
	setupAfe2ndUvValue();
	setupAfe2ndOtValue();
	setupAfe2ndUtValue();
                                
  	drv_bq796xx_init_default_load(afe_load_data);

	drv_bq796xx_init();
	bq796xx_event_RegisteCB(app_afe_cb);
	afe_steps = AFE_INIT_IDLE;
	BridgeDirection = DIR_SOUTH;//DIR_SOUTH;//DIR_NORTH;
	BridgeSwitchCount =0;
	NorthAndSouthIniFlag = 0;
#ifdef	AFE_DEBUG_MODE	
	AfeState = AFE_STATE_NORMAL;
	AfeTimerHandlerProcessor = AfeBq796xxNormalHandler;
	StackCountForIni[DIR_NORTH] = AFE_DEBUG_N_NUM;
	StackCountForIni[DIR_SOUTH] = AFE_DEBUG_S_NUM;
	BalanceData[0] = 0x0102;
	BalanceData[1] = 0x0304;
	BalanceData[2] = 0x0506;
	BalanceData[3] = 0x0708;
	BridgeDirection = DIR_SOUTH;
#else	
	changeToBq796xxIniHandler();
	//AfeTimerHandlerProcessor = AfeBq796xxIniHandler;
#endif
	LibSwTimerOpen(afeSwTimerHandler, 0);
	{	
		char	str[100];
		subFunctionPointer = (tAfeFunctionTable )stopBalanceFunctionTable;			
		sprintf(str,"Sub Fun Pointer=%.8lX",(uint32_t)subFunctionPointer);
		halAfeBq796xxDebugMsg(str);
		
		sprintf(str,"Sub Fun Pointer=%.8lX",*(uint32_t *)((uint32_t)subFunctionPointer+4));
		halAfeBq796xxDebugMsg(str);
		
		subFunctionPointer = (tAfeFunctionTable )stopBalanceFunctionTable[0];		
		sprintf(str,"Sub Fun Pointer=%.8lX",(uint32_t)subFunctionPointer);
		halAfeBq796xxDebugMsg(str);
	}
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



/**
  ******************************************************************************
  * @file        HalRtcSTM32L4xx.c
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/15
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
#include "define.h"
#include "main.h"
#include "LibDebug.h"
#include "halRtc.h"

void appSerialCanDavinciSendTextMessage(char *msg);
#define	HalRtcDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

//#define RTC_CLOCK_SOURCE_LSI
#define RTC_CLOCK_SOURCE_LSE

#ifdef RTC_CLOCK_SOURCE_LSI
#define RTC_ASYNCH_PREDIV    0x7F
#define RTC_SYNCH_PREDIV     0xF9    
#endif

#ifdef RTC_CLOCK_SOURCE_LSE
#define RTC_ASYNCH_PREDIV  0x7F
#define RTC_SYNCH_PREDIV   0x00FF
#endif
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static RTC_HandleTypeDef RtcHandle;

/* Public variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc)
{
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  
  /*##-1- Enables the PWR Clock and Enables access to the backup domain ###################################*/
  /* To change the source clock of the RTC feature (LSE, LSI), You have to:
     - Enable the power clock using __HAL_RCC_PWR_CLK_ENABLE()
     - Enable write access using HAL_PWR_EnableBkUpAccess() function before to 
       configure the RTC clock source (to be done once after reset).
     - Reset the Back up Domain using __HAL_RCC_BACKUPRESET_FORCE() and 
       __HAL_RCC_BACKUPRESET_RELEASE().
     - Configure the needed RTC clock source */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  /*##-2- Configure LSE/LSI as RTC clock source ###############################*/
#ifdef RTC_CLOCK_SOURCE_LSE
  
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  { 
    Error_Handler();
  }
  
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  { 
    Error_Handler();
  }
#elif defined (RTC_CLOCK_SOURCE_LSI)  
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  { 
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  { 
    Error_Handler();
  }
#else
#error Please select the RTC Clock source inside the main.h file
#endif /*RTC_CLOCK_SOURCE_LSE*/
  
  /*##-3- Enable RTC peripheral Clocks #######################################*/
  /* Enable RTC Clock */ 
  __HAL_RCC_RTC_ENABLE(); 
  
  /*##-4- Configure the NVIC for RTC TimeStamp ###############################*/
  HAL_NVIC_SetPriority(TAMP_STAMP_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(TAMP_STAMP_IRQn);
}

/**
  * @brief RTC MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  * @param hrtc: RTC handle pointer
  * @retval None
  */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc)
{
  /*##-1- Reset peripherals ##################################################*/
   __HAL_RCC_RTC_DISABLE();
}

void HalRtcGetDateTime(tHalRtcDateTime *pDateTime)
{
	RTC_DateTypeDef sdatestructureget;
  	RTC_TimeTypeDef stimestructureget;
  
  	/* Get the RTC current Time */
  	HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN);
  	/* Get the RTC current Date */
  	HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);

	pDateTime->Year = 2000 + sdatestructureget.Year;
	pDateTime->Month = sdatestructureget.Month;
	pDateTime->Day = sdatestructureget.Date;
	pDateTime->Hour = stimestructureget.Hours;
	pDateTime->Minute = stimestructureget.Minutes;
	pDateTime->Second = stimestructureget.Seconds;
}


tErrCode HalRtcSetupDate(uint16_t year, uint8_t mon, uint8_t day)
{
	RTC_DateTypeDef sdatestructure;

	sdatestructure.Year    = (year % 100);
  	sdatestructure.Month   = mon;
  	sdatestructure.Date    = day;
  	sdatestructure.WeekDay = 0;
  
  	if(HAL_RTC_SetDate(&RtcHandle, &sdatestructure, RTC_FORMAT_BIN) != HAL_OK)
  	{
    	/* Initialization Error */
    	//Error_Handler(); 
    	return RES_ERROR_FAIL;
  	} 
  	return RES_SUCCESS;
}

tErrCode HalRtcSetupTime(uint8_t hour, uint8_t min, uint8_t sec)
{
  	RTC_TimeTypeDef stimestructure;
  	
	stimestructure.Hours          = hour;
  	stimestructure.Minutes        = min;
	stimestructure.Seconds        = sec;
	stimestructure.SubSeconds     = 0x00;
	stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
  	if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BIN) != HAL_OK)
  	{
    	/* Initialization Error */
    	//Error_Handler();
    	return RES_ERROR_FAIL; 
  	}
  	return RES_SUCCESS;
}

void HalRtcSmpUnixTimeToDateTime(uint32_t sec, tHalRtcDateTime *pRtcDateTime)
{
 	BYTE	iMonthDay[12] ={31,28,31,30,31,30,31,31,30,31,30,31};
    WORD	yearoff;
    CHAR    str[20];
	BYTE	i;
	DWORD	dayoffset;
	DWORD	yearday;
//	tHalRtcDateTime	mRtcDateTime;

	dayoffset = sec / 86400L;

    pRtcDateTime->Second = sec % 60;
    pRtcDateTime->Minute = (sec % 3600) / 60;
    pRtcDateTime->Hour = (sec % 86400L) / 3600;

    yearoff = 2000;

    while(dayoffset > 36524L)	//100年
    {
    	dayoffset -= 36524L;
    	yearoff += 100;
    }

  	while(dayoffset >= 1461)	//年
    {
    	yearoff += 4;
    	dayoffset -= 1461;
    }

    while(1)
    {
    	if((yearoff % 400) == 0)
    		yearday = 366;
    	else if((yearoff % 100) == 0)
    		yearday = 365;
    	else if((yearoff % 4) == 0)
    		yearday = 366;
    	else
    		yearday = 365;

    	if(dayoffset < yearday)
    		break;
    	dayoffset -= yearday;
    	yearoff++;
    }
    if((yearoff % 400) == 0)
    	iMonthDay[1] = 29;
   	else if((yearoff%100) == 0)
 		iMonthDay[1] = 28;
   	else if((yearoff%4) == 0)
		iMonthDay[1] = 29;
   	else
		iMonthDay[1] = 28;
    for (i = 0; i < 12; i++)
    {
        if(dayoffset < iMonthDay[i])
            break;
        dayoffset -= iMonthDay[i];
    }
    pRtcDateTime->Year = yearoff;
    pRtcDateTime->Month = i + 1;
    pRtcDateTime->Day = dayoffset + 1;
}


uint32_t HalRtcGetSmpUnixTime(void)
{
	DWORD	baseday, day, sec, yearoff;
	BYTE	i;
    BYTE    iMonthDay[12] ={31,28,31,30,31,30,31,31,30,31,30,31};

	tHalRtcDateTime	mRtcDateTime;

	HalRtcGetDateTime(&mRtcDateTime);


    yearoff = (mRtcDateTime.Year % 400);                //  146097*4+135140
    baseday = (mRtcDateTime.Year / 400) * 146097L;// - 716241L;
    if (yearoff > 0)
        baseday += yearoff*365u + ((yearoff + 3) / 4) - (yearoff - 1) / 100;

	baseday -= (146097L * 5L);

    if(yearoff & 3)
        iMonthDay[1] = 28;
    else
    {
        if((yearoff % 400)==0)
            iMonthDay[1] = 29;
        else if((yearoff % 100)==0)
            iMonthDay[1] = 28;
        else
            iMonthDay[1] = 29;
    }
    for (day = 0, i = 1; i <mRtcDateTime.Month; i++)
        day += iMonthDay[i-1];
    day +=(baseday +mRtcDateTime.Day-1);

//	DateTime->WeekIndex=((day+6)/7);	//2000.1.1 起第幾周

	day *= 86400L;
  	sec = mRtcDateTime.Hour * 3600;
  	sec += mRtcDateTime.Minute * 60;
  	sec += mRtcDateTime.Second;
	sec += day;
    return sec;
}

tErrCode HalRtcOpen(void)
{
	/*##-1- Configure the RTC peripheral #######################################*/
  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
      - Hour Format    = Format 12
      - Asynch Prediv  = Value according to source clock
      - Synch Prediv   = Value according to source clock
      - OutPut         = Output Disable
      - OutPutPolarity = High Polarity
      - OutPutType     = 
      Open Drain */
  	__HAL_RTC_RESET_HANDLE_STATE(&RtcHandle);
  	RtcHandle.Instance            = RTC;
  	RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
  	RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
  	RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
  	RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  	RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  	RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

  	if(HAL_RTC_Init(&RtcHandle) != HAL_OK)
  	{
    	/* Initialization Error */
    	//Error_Handler(); 
    	return RES_ERROR_FAIL;
	}
	
  	return RES_SUCCESS;
}
tErrCode HalRtcClose(void)
{
  	return RES_SUCCESS;
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/


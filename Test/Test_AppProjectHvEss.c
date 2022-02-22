/**
  ******************************************************************************
  * @file    Test_AppProjectHvEss.c
  * @author  Golden Chen
  * @version V0.0.1
  * @date    2022/02/11
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 SMP</center></h2>
  *
  * 
  *
  ******************************************************************************
  */

#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "LibDebug.h"
#include "LibSwTimer.h"
#include "LibHwTimer.h"
#include "halafe.h"
#include "halUart.h"
#include "HalCan.h"
#include "HalBsp.h"
#include "HalTimer.h"
#include "HalRtc.h"
#include "HalEeprom.h"
#include "AppProject.h"
#include "ApiSysPar.h"
#include "AppProtect.h"
#include "AppSerialCanDavinci.h"
#include "AppSerialUartDavinci.h"
#include "AppBalance.h"
#include "AppGauge.h"
#include "ApiSignalFeedback.h"
#include "LibNtc.h"
#include "LibData.h"
#include "ApiRamData.h"
#include "HalRtc.h"
#include "ApiSysPar.h"
#include "ApiRelayControl.h"
#include "AppBms.h"
#include "ApiScuTemp.h"
#include "ApiSystemFlag.h"
#include "HalAfeADS7946.h"
#include "ApiEventLog.h"
#include "SmpEventType.h"
#include "AppButton.h"
#include "smp_w5500_DMA.h"
#include "smp_drv_bq796xx.h"
#include "Test_drv_bq796xx.h"
#include "ApiIRMonitoring.h"
#include "SEGGER_RTT.h"
#include "RTT_Log.h"


// Disable/Enable test funcuion.
//---------------------------------------------------------------------------------------
#if 1
#define TEST_TLC6C5912_FUNC
#include "Test_TLC6C5912.h"

#endif

#if 0
#define TEST_MAX7219_LCD_MARITEX
#include "Test_max7219.h"
#endif

#if 0
#define TEST_MX25LXX_FLASH 
#include "Test_MX25L_Driver.h"
#endif

#if 1
#define TEST_ADS7946_FUNC
#include "Test_ADS7946_Driver.h" 
#endif 

#if 1
#define TEST_IRM_FUNCTION
#include "Test_ApiIRMonitoring.h"
#endif


#if 1
#define TEST_INT_ADC_FUNCTION
#include "Test_ADC.h"
#endif


#if 1
#define TEST_EVENT_LOG_FUNC
#include "Test_log_managment.h"
#endif


#if 1
#define TEST_BQ796XX_SETTING_INIT_WITHOUT_STEP
#endif 

#if 1
#define TEST_BQ796XX_SETTING_INIT_WITH_STEP
#endif

#if 1
#define TEST_BQ796XX_DIRECTION_CHECK_BMU_WITH_STEP
#endif

#if 1
#define TEST_BQ796XX_OVUVOTUT_FUNC
#endif

#if 1
#define TEST_BQ796XX_CELL_BALANCE_FUNC 

#endif

#if 1
#define TEST_BQ796XX_GPIO_SELECT_READ_ADC_FUNC
#endif
//---------------------------------------------------------------------------------------

#define	Test_appProjectDebugMsg(str)	appSerialCanDavinciSendTextMessage(str)
#define	Test_saveEventLog(type, par)		apiEventLogSaveLogData(type, par)

tHalTimer	test_mHalTimer3={3, 1000};
static uint8_t	test_RelayOnFlag = 0;
static uint8_t	test_SystemReadyFlag = 0;
static uint8_t	test_RtcValid;

static void Test_relayOff(void)
{
	apiRelayControlMainRelayOff();
	test_RelayOnFlag = 0;
}

static void Test_appProjectHwTimerHandler(void *pin, uint16_t evt, void *pData)
{
	LibSwTimerHwHandler(LIB_SW_TIMER_EVT_HW_1MS, 0);
	LibHwTimerHandle();
}

static void Test_protectEventHandler(void *pDest, uint16_t evt, void *pData)
{
	switch(evt)
	{
	case APP_PROTECT_OVP_L1_SET:
		Test_saveEventLog(EVENT_TYPE_OVP_L1_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_OVP_L2_SET:
		Test_saveEventLog(EVENT_TYPE_OVP_L2_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_OVP_L3_SET:
		Test_relayOff();
		Test_appProjectDebugMsg("OVP L3 set");
		Test_saveEventLog(EVENT_TYPE_OVP_L3_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_OVP_L1_RELEASE:
		Test_saveEventLog(EVENT_TYPE_OVP_L1_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_OVP_L2_RELEASE:
		Test_saveEventLog(EVENT_TYPE_OVP_L2_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_OVP_L3_RELEASE:
		Test_saveEventLog(EVENT_TYPE_OVP_L3_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_UVP_L1_SET:
		Test_saveEventLog(EVENT_TYPE_UVP_L1_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_UVP_L2_SET:
		Test_saveEventLog(EVENT_TYPE_UVP_L2_SET, libGetUint16((uint8_t *)pData));	
		break;
	case APP_PROTECT_UVP_L3_SET:
		Test_saveEventLog(EVENT_TYPE_UVP_L3_SET, libGetUint16((uint8_t *)pData));
		Test_relayOff();
		break;
	case APP_PROTECT_UVP_L1_RELEASE:
		Test_saveEventLog(EVENT_TYPE_UVP_L1_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_UVP_L2_RELEASE:
		Test_saveEventLog(EVENT_TYPE_UVP_L2_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_UVP_L3_RELEASE:
		Test_saveEventLog(EVENT_TYPE_UVP_L3_RELEASE, libGetUint16((uint8_t *)pData));
		Test_relayOff();
		break;
	case APP_PROTECT_COTP_L1_SET:
		Test_saveEventLog(EVENT_TYPE_COTP_L1_SET, libGetUint16((uint8_t *)pData));
		break;	
	case APP_PROTECT_COTP_L2_SET:
		Test_saveEventLog(EVENT_TYPE_COTP_L2_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COTP_L3_SET:
		Test_saveEventLog(EVENT_TYPE_COTP_L3_SET, libGetUint16((uint8_t *)pData));
		Test_relayOff();
		break;
	case APP_PROTECT_COTP_L4_SET:
		Test_saveEventLog(EVENT_TYPE_COTP_L4_SET, libGetUint16((uint8_t *)pData));
		Test_relayOff();
		break;
	case APP_PROTECT_COTP_L1_RELEASE:
		Test_saveEventLog(EVENT_TYPE_COTP_L1_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COTP_L2_RELEASE:
		Test_saveEventLog(EVENT_TYPE_COTP_L2_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COTP_L3_RELEASE:
		Test_saveEventLog(EVENT_TYPE_COTP_L3_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COTP_L4_RELEASE:
		Test_saveEventLog(EVENT_TYPE_COTP_L4_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_CUTP_L1_SET:
		Test_saveEventLog(EVENT_TYPE_CUTP_L1_SET, libGetUint16((uint8_t *)pData));
		break;	
	case APP_PROTECT_CUTP_L2_SET:
		Test_saveEventLog(EVENT_TYPE_CUTP_L2_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_CUTP_L3_SET:
		Test_saveEventLog(EVENT_TYPE_CUTP_L3_SET, libGetUint16((uint8_t *)pData));
		Test_relayOff();
		break;
	case APP_PROTECT_CUTP_L4_SET:
		Test_saveEventLog(EVENT_TYPE_CUTP_L4_SET, libGetUint16((uint8_t *)pData));
		Test_relayOff();
		break;
	case APP_PROTECT_CUTP_L1_RELEASE:
		Test_saveEventLog(EVENT_TYPE_CUTP_L1_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_CUTP_L2_RELEASE:
		Test_saveEventLog(EVENT_TYPE_CUTP_L2_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_CUTP_L3_RELEASE:
		Test_saveEventLog(EVENT_TYPE_CUTP_L3_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_CUTP_L4_RELEASE:
		Test_saveEventLog(EVENT_TYPE_CUTP_L4_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOTP_L1_SET:
		Test_saveEventLog(EVENT_TYPE_DOTP_L1_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOTP_L2_SET:		
		Test_saveEventLog(EVENT_TYPE_DOTP_L2_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOTP_L3_SET:
		Test_relayOff();
		Test_saveEventLog(EVENT_TYPE_DOTP_L3_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOTP_L1_RELEASE:
		Test_saveEventLog(EVENT_TYPE_DOTP_L1_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOTP_L2_RELEASE:
		Test_saveEventLog(EVENT_TYPE_DOTP_L2_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOTP_L3_RELEASE:
		Test_saveEventLog(EVENT_TYPE_DOTP_L3_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DUTP_L1_SET:
		Test_saveEventLog(EVENT_TYPE_DUTP_L1_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DUTP_L2_SET:		
		Test_saveEventLog(EVENT_TYPE_DUTP_L2_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DUTP_L3_SET:
		Test_relayOff();
		Test_saveEventLog(EVENT_TYPE_DUTP_L3_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DUTP_L1_RELEASE:
		Test_saveEventLog(EVENT_TYPE_DUTP_L1_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DUTP_L2_RELEASE:
		Test_saveEventLog(EVENT_TYPE_DUTP_L2_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DUTP_L3_RELEASE:
		Test_saveEventLog(EVENT_TYPE_DUTP_L3_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOCP_L1_SET:
		Test_saveEventLog(EVENT_TYPE_DOCP_L1_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOCP_L2_SET:
		Test_saveEventLog(EVENT_TYPE_DOCP_L2_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOCP_L3_SET:
		Test_relayOff();
		Test_saveEventLog(EVENT_TYPE_DOCP_L3_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOCP_L4_SET:
		Test_relayOff();
		Test_saveEventLog(EVENT_TYPE_DOCP_L4_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOCP_L1_RELEASE:
		Test_saveEventLog(EVENT_TYPE_DOCP_L1_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOCP_L2_RELEASE:
		Test_saveEventLog(EVENT_TYPE_DOCP_L2_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOCP_L3_RELEASE:
		Test_saveEventLog(EVENT_TYPE_DOCP_L3_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_DOCP_L4_RELEASE:
		Test_saveEventLog(EVENT_TYPE_DOCP_L4_RELEASE, libGetUint16((uint8_t *)pData));
		break;	
	case APP_PROTECT_COCP_L1_SET:
		Test_saveEventLog(EVENT_TYPE_COCP_L1_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COCP_L2_SET:	
		Test_saveEventLog(EVENT_TYPE_COCP_L2_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COCP_L3_SET:
		Test_relayOff();
		Test_saveEventLog(EVENT_TYPE_COCP_L3_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COCP_L4_SET:
		Test_relayOff();
		Test_saveEventLog(EVENT_TYPE_COCP_L4_SET, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COCP_L1_RELEASE:
		Test_saveEventLog(EVENT_TYPE_COCP_L1_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COCP_L2_RELEASE:
		Test_saveEventLog(EVENT_TYPE_COCP_L2_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COCP_L3_RELEASE:
		Test_saveEventLog(EVENT_TYPE_COCP_L3_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_COCP_L4_RELEASE:
		Test_saveEventLog(EVENT_TYPE_COCP_L4_RELEASE, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_OVP_PF:
		Test_relayOff();
		apiSysParOvpPfSet();
		Test_saveEventLog(EVENT_TYPE_OVP_PF, libGetUint16((uint8_t *)pData));
		break;
	case APP_PROTECT_UVP_PF:
		Test_relayOff();
		apiSysParUvpPfSet();
		Test_saveEventLog(EVENT_TYPE_UVP_PF, libGetUint16((uint8_t *)pData));
		break;
	}
}

static void Test_gaugeEventHandler(void *pDest, uint16_t evt, void *pData)
{
	switch(evt)
	{
	case APP_GAUGE_EVENT_START_CHG_SOC:
	case APP_GAUGE_EVENT_START_DHG_SOC:
	case APP_GAUGE_EVENT_IDLE_OVER_5HR:
	case APP_GAUGE_EVENT_CANNOT_GET_5HR_SOC:
	case APP_GAUGE_EVENT_UPDATE_QMAX_1ST:
	case APP_GAUGE_EVENT_UPDATE_QMAX:
	case APP_GAUGE_EVENT_GET_5HR_SOC:
		break;
	}
	if(evt >= APP_GAUGE_EVENT_CAL_RA1 &&
	   evt <= (APP_GAUGE_EVENT_CAL_RA1+30))
	{
		;
	}

}

static void Test_appProjectSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	static	uint8_t		SystemReadyCount = 10;
//	tHalRtcDateTime	mHalRtcDateTime;
	char	str[100];

    if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
			;
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		if(SystemReadyCount)
		{
			SystemReadyCount--;
			if(!SystemReadyCount)
				test_SystemReadyFlag = 1;
		}
		apiRamSaveRtcDateTime();
		
		//sprintf(str,"EPO Status ......%d",	HalBspGetEpoStatus());
		//appProjectDebugMsg(str);
	}
}

static uint8_t	test_ocpCount;
static void ocpReleaseSwTimer(__far void *dest, uint16_t evt, void *vDataPtr)
{
//	GPIOD->ODR |= GPIO_PIN_14;
    if(evt == LIB_SW_TIMER_EVT_SW_1MS)
    {
    	test_ocpCount++;
    	if(test_ocpCount >= 20)
    	{
			HalBspReleaseCtrl(0);
	  		LibSwTimerClose(ocpReleaseSwTimer, 0);
	  	}
	}
//	GPIOD->ODR &= ~GPIO_PIN_14;
}

static void test_releaseOCP(void)
{
	test_ocpCount = 0;
	HalBspReleaseCtrl(1);
  LibSwTimerOpen(ocpReleaseSwTimer, 0);
}


static void Test_buttonEventHandler(void *pDest, uint16_t evt, void *pData)
{
	switch(evt)
	{
	case BUTTON_EVT_CLICK_KEY_0:
		Test_appProjectDebugMsg("BUTTON_EVT_CLICK_KEY_0");
		break;
	case BUTTON_EVT_LONG_PRESS_KEY_0:
		Test_appProjectDebugMsg("BUTTON_EVT_LONG_PRESS_KEY_0");
		break;
	case BUTTON_EVT_PRESS_1S_KEY_0:
		Test_appProjectDebugMsg("BUTTON_EVT_PRESS_1S_KEY_0");
		break;
	case BUTTON_EVT_PRESS_KEY_0:
		Test_appProjectDebugMsg("BUTTON_EVT_PRESS_KEY_0");
		break;
	case BUTTON_EVT_RELEASE_KEY_0:	
		Test_appProjectDebugMsg("BUTTON_EVT_RELEASE_KEY_0");
		break;
	}
}

//--------------------------------------------------------------------------------
void Test_appProjectOpen(void){
	char	str[100];
	uint32_t	len;
	 
	HalRtcOpen();
	apiRamOpen();
	HalTimerOpen(&test_mHalTimer3, Test_appProjectHwTimerHandler);
	appSerialUartDavinciOpen();
	appSerialCanDavinciOpen();
	
  	//------------------------------------------
	Test_appProjectDebugMsg("--------- Start Run -----------...9");
	len = apiSysParOpen();
	
	appProtectOpen(Test_protectEventHandler);
	appGaugeOpen(Test_gaugeEventHandler);
		
	sprintf(str,"Par Len = %d", len);
	Test_appProjectDebugMsg(str);
  
	apiSignalFeedbackOpen(0);//signalFeedbackEventHandler);
  
	LibSwTimerOpen(Test_appProjectSwTimerHandler, 0);
  test_releaseOCP(); 
  apiRelayControlOpen();
  appBmsOpen();
  apiScuTempOpen();
  apiSystemFlagOpen();
  apiEventLogOpen();
	appButtonOpen(Test_buttonEventHandler);
}

void Test_main(void){
	SEGGER_RTT_Init();
	SEGGER_RTT_printf(0,"Test Main Code Start...\r\n" );
	
  Test_appProjectOpen();
	LibSwTimerClearCount();
	
	//Put into test code 
	//--------------------------------------------------------------------------------------------------------------------------------- 
	
	// MCU UART3 Test
	//-------------------------------------------
  appSerialUartSendMessage((uint8_t*)"12345678");
	//-------------------------------------------
	
	//LED Display Indicator Test
	//-------------------------------------------
	#ifdef TEST_TLC6C5912_FUNC
	Test_TLC6C5912_FUNC();
	#endif
	
  // MAX7219 LCD Maritex Test
	//-------------------------------------------
	#ifdef TEST_MAX7219_LCD_MARITEX
  Test_MAX7219_LCD_Maxtrix_Func();
	#endif
  //-------------------------------------------

  //Internal ADC Test
	#ifdef TEST_INT_ADC_FUNCTION
	Test_Internal_ADC();
  #endif

	// MX25LXX Flash Test
	//------------------------------------------
	#ifdef TEST_MX25LXX_FLASH
	Test_Event_MX25L_Driver();
	#endif
	//------------------------------------------

  #ifdef TEST_ADS7946_FUNC
	Test_ADS7946_Init();
	#endif

  // BQ96XX Setting Init without step Test
  //----------------------------------------------------------------------------------
	#ifdef TEST_BQ796XX_SETTING_INIT_WITHOUT_STEP
  Test_BQ796XX_Setting_Init_Without_Step();
	#endif
	//----------------------------------------------------------------------------------

	// BQ96XX Setting Init with step Test
	//----------------------------------------------------------------------------------
	#ifdef TEST_BQ796XX_SETTING_INIT_WITH_STEP
  Test_BQ796XX_Setting_Init_With_Step(); 
  #endif
	//----------------------------------------------------------------------------------
	
	// BQ796XX Cell Balance function Test
	//----------------------------------------------------------------------------------
	#ifdef TEST_BQ796XX_CELL_BALANCE_FUNC
  Test_BQ796XX_Cell_balance_func();
	#endif
	//----------------------------------------------------------------------------------

	// BQ796XX GPIO select read ADC function Test
	//----------------------------------------------------------------------------------	
  #ifdef TEST_BQ796XX_GPIO_SELECT_READ_ADC_FUNC
  Test_BQ796XX_Gpio_Select_Read_ADC_FUNC();
	#endif
	//----------------------------------------------------------------------------------

	// BQ796XX  Direction check North/South BMU number Test with step.
	//----------------------------------------------------------------------------------
	#ifdef TEST_BQ796XX_DIRECTION_CHECK_BMU_WITH_STEP
  Test_BQ796XX_Direction_Chechk_BMU_with_Step();
	#endif
  //----------------------------------------------------------------------------------

	// BQ796XX TEST OV/UV/OT/UT nFault and nFault setting at this data struct "bq796xx_init_default_t".
	//----------------------------------------------------------------------------------
	#ifdef TEST_BQ796XX_OVUVOTUT_FUNC
  Test_BQ796XX_OVUVOTUT_FUNC();
	#endif
  //----------------------------------------------------------------------------------	

	//API IRMonitoring Test
	//----------------------------------------------------------------------------------------- 
	#ifdef TEST_IRM_FUNCTION
  Test_IRM_Function_Exe();
	#endif
	//-------------------------------------------------------------------------------------- 	
	
	//Log managment test
	#ifdef TEST_EVENT_LOG_FUNC
	Test_Event_Log_FUNC();
	#endif
	
  //Test main code while loop
  //-------------------------------------------
	while(1){
	    LibSwTimerHandle();
	}
	//-------------------------------------------
	//---------------------------------------------------------------------------------------------------------------------------------

}

//--------FILE END------------------------------------------------------------------------------------------

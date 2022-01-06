/**
  ******************************************************************************
  * @file        sdk_config.h
  * @author      Johnny
  * @version     v1.0
  * @date        2021/9/4
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef SDK_CONFIG_H_
#define SDK_CONFIG_H_

#define __far
//
//#include "project_config.h"

// <h> Application 

//==========================================================
// <q> APP PROJECT 
#define APP_PROJECT_LED_SHOW_CAPACITY_TIME 5000 //ms
#define APP_PROJECT_VPACK_THRES 36000  //mv
#define APP_PROJECT_VPACK_DROP_DELTA 20  //%
#define APP_PROJECT_VPACK_NOMINAL_VOL 36500  //mV

#ifndef APP_PROJECT_LED_POWER_OFF_TIMES
#define APP_PROJECT_LED_POWER_OFF_TIMES 2
#endif

#ifndef APP_PROJECT_LED_POWER_ON_ENABLE
#define APP_PROJECT_LED_POWER_ON_ENABLE 0
#endif

#define APP_PROJECT_STATE_ALARM_SUPPOER 1

#ifndef APP_PROJECT_SHUTDOWN_TIMER_DEF
#define APP_PROJECT_SHUTDOWN_TIMER_DEF 86400
#endif

#ifndef APP_PROJECT_LED_COMM_ENABLE
#define APP_PROJECT_LED_COMM_ENABLE 0
#endif

//==========================================================
// <q> APP CAPACITY
#ifndef APP_CAPACITY_SELF_CURRENT
#define APP_CAPACITY_SELF_CURRENT  4   //mA
#endif
//==========================================================
// <q> APP SERIAL

#define APP_SERIAL_KEEGLO_HAL &mHalSerialUart1
#define APP_SERIAL_KEEGLO_APP_DATA_COMM &mAppSerialDat
#define APP_SERIAL_KEEGLO_BMU 0

#define APP_SERIAL_CUSTOM_HAL &mHalSerialUart0
#define APP_SERIAL_CUSTOM_APP_DATA_COMM &mAppSerialDat
#define APP_SERIAL_CUSTOM_BMU 0


#define APP_SERIAL_CAN_KEEGLO_HAL &mHalSerialCan1
#define APP_SERIAL_CAN_KEEGLO_APP_DATA_COMM &mAppSerialDat
#define APP_SERIAL_CAN_KEEGLO_BMU 0

//UART
#ifndef APP_SERIAL_CUSTOM_ENABLE
#define APP_SERIAL_CUSTOM_ENABLE 0
#endif


//CAN
#ifndef APP_SERIAL_CAN1_ENABLE
#define APP_SERIAL_CAN1_ENABLE 1
#endif

#ifndef APP_SERIAL_CAN_CUSTOM_ENABLE
#define APP_SERIAL_CAN_CUSTOM_ENABLE  1
#endif

#if (APP_SERIAL_CAN_CUSTOM_ENABLE == 1)
#define APP_SERIAL_CAN_CUSTOM_HAL &mHalSerialCan1
#define APP_SERIAL_CAN_CUSTOM_APP_DATA_COMM &mAppSerialDat
#define APP_SERIAL_CAN_CUSTOM_BMU 0

#endif

// </h> 
//==========================================================

// <h> AFE 

//==========================================================
// <q> AFE Parameter 

#ifndef AFE_MAX
#define AFE_MAX 1
#endif

#define AFE_ADC_DATA_LEN_16 16
#define AFE_ADC_DATA_LEN_32 32

#ifndef AFE_ADC_DATA_LEN
#define AFE_ADC_DATA_LEN AFE_ADC_DATA_LEN_16
#endif

#ifndef AFE_ADC_GAIN_CELL_VOL
#define AFE_ADC_GAIN_CELL_VOL 305
#endif

#ifndef AFE_ADC_GAIN_PACK_VOL
#define AFE_ADC_GAIN_PACK_VOL 6104
#endif

#ifndef AFE_ADC_GAIN_CURR
#define AFE_ADC_GAIN_CURR 5493
#endif

#define AFE_AVG_WINDOW_CELL_VOL 2
#define AFE_AVG_WINDOW_NTC 2
#define AFE_AVG_WINDOW_CURRENT 1
#define AFE_AVG_WINDOW_VB 0
#define AFE_AVG_WINDOW_VP 0

#define AFE_CALIB_VB   0
#define AFE_CALIB_VP   1
#define AFE_CALIB_CURRENT   0

#define AFE_ADC_BALANCE_RELEASE_TIME 80
//==========================================================
// <q> LIB MOS

#ifndef LIB_MOS_PRE_DSG_TIME_DEF   //ms
#define LIB_MOS_PRE_DSG_TIME_DEF 2000
#endif

#ifndef LIB_MOS_PRE_DSG_RETRY_TIME   //ms
#define LIB_MOS_PRE_DSG_RETRY_TIME 1000
#endif

#ifndef LIB_MOS_PRE_DSG_RETRY_COUNT   
#define LIB_MOS_PRE_DSG_RETRY_COUNT 10
#endif

#ifndef LIB_MOS_PRE_DSG_VP_THRESHOLD   //uV
#define LIB_MOS_PRE_DSG_VP_THRESHOLD 10000000
#endif

//==========================================================
// <q> LIB PROTECT

#ifndef LIB_PROTECT_OV_ENABLE   
#define LIB_PROTECT_OV_ENABLE 1
#endif

// OVP
#define LIB_OVP_LEVEL_MAX 3

#define LIB_OVP_THRESHOLD_L1   (3500000/AFE_ADC_GAIN_CELL_VOL)  //LSB
#define LIB_OVP_THRESHOLD_L2   (3600000/AFE_ADC_GAIN_CELL_VOL)  //LSB
#define LIB_OVP_THRESHOLD_L3   (3650000/AFE_ADC_GAIN_CELL_VOL)  //LSB
#define LIB_OVP_THRESHOLD_RELEASE   (3450000/AFE_ADC_GAIN_CELL_VOL)  //LSB
#define LIB_OVP_TIME 50  //unit 100ms
#define LIB_OVP_TIME_RELEASE 50  //unit 100ms

//==========================================================
// <q> LIB Register

#ifndef LIB_REGISTER_RECORD_DYNAMIC_ENABLE
#define LIB_REGISTER_RECORD_DYNAMIC_ENABLE 0
#endif

#ifndef LIB_REGISTER_RECORD_MAX
#define LIB_REGISTER_RECORD_MAX 	50
#endif

//==========================================================
// <q> LIB Button

#ifndef LIB_BUTTON_LONG_PRESS_TIME
#define LIB_BUTTON_LONG_PRESS_TIME 3000  //ms
#endif


#ifndef LIB_BUTTON_USE_ADC
#define LIB_BUTTON_USE_ADC 0  
#endif


//==========================================================
// <q> NTC
#ifndef HAL_NTC_CHECK_TABLE
#define HAL_NTC_CHECK_TABLE {1,0,0,0,0,0,0}
#endif

//==========================================================
// <q> BMU

#ifndef APP_BMU_SELF_TEST_TIME    //Unit:100ms 
#define APP_BMU_SELF_TEST_TIME (LIB_BUTTON_LONG_PRESS_TIME / 100)
#endif

#ifndef APP_BMU_PROTECT_DUTP_ENABLE
#define	APP_BMU_PROTECT_DUTP_ENABLE	0
#endif 

#ifndef APP_BMU_PRE_DISCHARGE_ENABLE
#define	APP_BMU_PRE_DISCHARGE_ENABLE	0
#endif

//==========================================================
// <q> BMS
#ifndef BMS_MODE_SLAVE_ENABLE
#define BMS_MODE_SLAVE_ENABLE false
#endif

#endif


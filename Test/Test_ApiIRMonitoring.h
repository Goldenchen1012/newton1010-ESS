/**
  ******************************************************************************
  * @file    Test_ApiIRMonitoring.h 
  * @author  Golden Chen
  * @version V0.0.1
  * @date    2022/02/11
  * @brief   Header for Test_ApiIRMonitoring.c module
  ******************************************************************************
  */

#ifndef __TEST_API_IRMONITORING__
#define __TEST_API_IRMONITORING__
#include <stdint.h>

#define TEST_IRM_FUNCTION_TEST_CYCLE_NUM                                     50
#define TEST_IRM_FUNCTION_TEST_CYCLE_DELAY_TIME                              100

//IRM GPIO Call Back Function--------------------
#define ADS7946_VREF                           4.096f
#define ADS7946_RESOLUTION_B                   14
#define VSTACK_CAL_P1                          696.64f
#define VSTACK_CAL_P2                          -0.2192f
#define VO_CAL_V_P1                            0.002f   //unit:V


void Test_IRM_Function_Init(void);
void Test_IRM_Function_Exe(void);

#endif // __TEST_API_IRMONITORING__

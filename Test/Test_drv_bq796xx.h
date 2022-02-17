/**
  ******************************************************************************
  * @file    Test_drv_bq796xx.h 
  * @author  Golden Chen
  * @version V0.0.1
  * @date    2022/02/11
  * @brief   Header for Test_drv_bq796xx.c module
  ******************************************************************************
  */

#ifndef __TEST_DRV_BQ796XX__
#define __TEST_DRV_BQ796XX__
#include <stdint.h>

#define TEST_BQ796XX_SETTING_INIT_WITH_STEP_TEST_CYCLE_NUM                   10
#define TEST_BQ796XX_SETTING_INIT_WITH_STEP_WAKE_CNT                         10
#define TEST_BQ796XX_DIRECTION_CHECK_BMU_WITH_STEP_TEST_CYCLE_NUM            10
#define TEST_BQ796XX_OVUVOTUT_FUNC_TEST_CYCLE_NUM                            200
#define TEST_BQ796XX_CELL_BALANCE_FUNC_TEST_CYCLE_NUM                        100
#define TEST_BQ796XX_GPIO_SELECT_READ_ADC_FUNC_TEST_CYCLE_NUM                5

void Test_BQ796XX_Setting_Init_Without_Step(void);
void Test_BQ796XX_Setting_Init_With_Step(void);
void Test_BQ796XX_Gpio_Select_Read_ADC_FUNC(void);
void Test_BQ796XX_Cell_balance_func(void);
void Test_BQ796XX_Direction_Chechk_BMU_with_Step(void);
void Test_BQ796XX_Direction_Chechk_BMU_with_Step_Print_MeasureData(void);
void Test_BQ796XX_Direction_Chechk_BMU_with_Step_Print_Statistics(long int k_cnt);
void Test_BQ796XX_SET_GPIO_Channel(uint8_t gpio_ch);
uint8_t Test_BQ796XX_DIRECTION_SET_STEPS_NORTH(void);
void Test_BQ796XX_OVUVOTUT_FUNC(void);

#endif // __TEST_DRV_BQ796XX__

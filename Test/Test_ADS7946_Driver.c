/**
  ******************************************************************************
  * @file    Test_ADS7946_Driver.c
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

#include "Bsp.h"
#include "smp_debug.h"
#include "smp_gpio.h"
#include "smp_ADS7946_Driver.h"
#include <stdbool.h>

void Test_ADS7946_Init(void){
	smp_ADS7946_init();
}
//--------FILE END------------------------------------------------------------------------------------------

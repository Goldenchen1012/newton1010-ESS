/**
  ******************************************************************************
  * @file    smp_drv_bq796xx.h 
  * @author  Golden Chen
  * @version V0.0.12
  * @date    2021/12/23
  * @brief   Header for smp_drv_bq796xx.c module
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMP_DRV_BQ796XX_H
#define __SMP_DRV_BQ796XX_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "smp_gpio.h"
#include "smp_uart.h"
/* Exported types ------------------------------------------------------------*/
typedef enum{
	BQ_GPIO          = 0,										/* GPIO */
	BQ_UART 							     						  /* UART */
}bq796xx_io_type;

typedef enum{
	HIGH_BYTE        = 0,					 					
	LOW_BYTE  							     						
}byte_h_l;

enum{
    DIR_NORTH = 0,
    DIR_SOUTH  =1,
};

typedef enum{
	WAKE_TONE_DISABLE        = 0,
	WAKE_TONE_ENABLE
}bq796xx_wake_tone_switch;

enum {
    SINGLE_READ     = 0x80,
    SINGLE_WRITE    = 0x90,
    STACK_READ      = 0xA0,
    STACK_WRITE     = 0xB0,
    BROAD_READ      = 0xC0,
    BROAD_WRITE     = 0xD0,
    BROAD_WRITE_REVERSE = 0xE0
};

typedef enum{
    GPIO_HI_Z,
    GPIO_ADC_OTUT,
    GPIO_ADC,
    GPIO_INPUT,
    GPIO_OUT_H,
    GPIO_OUT_L,
    GPIO_ADC_PULL_H,
    GPIO_ADC_PULL_L,
}bq796xx_AFE_GPIO_Type;

typedef enum{
    AFE_GPIO1,
    AFE_GPIO2,
    AFE_GPIO3,
    AFE_GPIO4,
    AFE_GPIO5,
    AFE_GPIO6,
    AFE_GPIO7,
    AFE_GPIO8,
}bq796xx_AFE_GPIO;

typedef enum{
	SINGLE    = 0,					 					
	STACK  							     						
}bq796xx_AFE_GPIO_stack;

typedef enum{
	BQ_DISABLE   = 0,					 					
	BQ_ENABLE  							     						
}bq796xx_fun_switch;

typedef enum{
	OV_2700MV = 0x02,					 					
	OV_3500MV = 0x12,
	OV_4175MV = 0x22
}bq796xx_ov_range;

typedef enum{
	OV_STEP_0MV   = 0,					 					
	OV_STEP_25MV  ,
	OV_STEP_50MV  ,
	OV_STEP_75MV  ,
	OV_STEP_100MV ,
	OV_STEP_125MV ,
	OV_STEP_150MV ,
	OV_STEP_175MV ,
	OV_STEP_200MV ,
	OV_STEP_225MV ,
	OV_STEP_250MV ,	
	OV_STEP_275MV ,	
	OV_STEP_300MV ,	
}bq796xx_ov_step;

typedef enum{
	UV_1200MV = 0,					 					
	UV_1250MV ,
	UV_1300MV ,
	UV_1350MV ,
	UV_1400MV ,
	UV_1450MV ,
	UV_1500MV ,
	UV_1550MV ,
	UV_1600MV ,
	UV_1650MV ,
	UV_1700MV ,
	UV_1750MV ,
	UV_1800MV ,
	UV_1850MV ,	
	UV_1900MV ,
	UV_1950MV ,
	UV_2000MV ,
	UV_2050MV ,
	UV_2100MV ,
	UV_2150MV ,
	UV_2200MV ,
	UV_2250MV ,
	UV_2300MV ,
	UV_2350MV ,
	UV_2400MV ,
	UV_2450MV ,
	UV_2500MV ,
	UV_2550MV ,
	UV_2600MV ,
	UV_2650MV ,
	UV_2700MV ,
	UV_2750MV ,
	UV_2800MV ,
	UV_2850MV ,
	UV_2900MV ,
	UV_2950MV ,
	UV_3000MV ,
	UV_3050MV ,
	UV_3100MV ,
}bq796xx_uv_mv;

typedef enum{
	OT_10_PCT = 0,					 					
	OT_11_PCT ,
  OT_12_PCT ,
  OT_13_PCT ,
  OT_14_PCT ,
  OT_15_PCT ,
  OT_16_PCT ,
  OT_17_PCT ,
  OT_18_PCT ,
  OT_19_PCT ,
  OT_20_PCT ,
  OT_21_PCT ,
  OT_22_PCT ,
  OT_23_PCT ,
  OT_24_PCT ,
  OT_25_PCT ,
  OT_26_PCT ,
  OT_27_PCT ,
  OT_28_PCT ,
  OT_29_PCT ,
  OT_30_PCT ,
  OT_31_PCT ,
  OT_32_PCT ,
  OT_33_PCT ,
  OT_34_PCT ,
  OT_35_PCT ,
  OT_36_PCT ,
  OT_37_PCT ,
  OT_38_PCT ,
  OT_39_PCT ,	
}bq796xx_ot_threshold_p;

typedef enum{
	UT_66_PCT = 0,					 					
	UT_68_PCT ,
  UT_70_PCT ,
  UT_72_PCT ,
  UT_74_PCT ,
  UT_76_PCT ,
  UT_78_PCT ,
  UT_80_PCT ,
}bq796xx_ut_threshold_p;

typedef enum{
	CB_STOP = 0  ,					 					
	CB_TIME_10S  ,
	CB_TIME_30S  ,
	CB_TIME_60S  ,
	CB_TIME_300S ,
	CB_TIME_10MIN,
	CB_TIME_20MIN,
	CB_TIME_30MIN,
	CB_TIME_40MIN,
	CB_TIME_50MIN,
	CB_TIME_60MIN,
	CB_TIME_70MIN,
	CB_TIME_80MIN,
	CB_TIME_90MIN,
	CB_TIME_100MIN,
	CB_TIME_110MIN,
	CB_TIME_120MIN,
	CB_TIME_150MIN,
	CB_TIME_180MIN,
	CB_TIME_210MIN,
	CB_TIME_240MIN,
	CB_TIME_270MIN,
	CB_TIME_300MIN,
	CB_TIME_330MIN,
	CB_TIME_360MIN,
	CB_TIME_390MIN,
	CB_TIME_420MIN,
	CB_TIME_450MIN,
	CB_TIME_480MIN,
	CB_TIME_510MIN,
	CB_TIME_540MIN,
	CB_TIME_570MIN,
	CB_TIME_600MIN,
}bq796xx_cellbalance_time;

typedef enum{
	CB_CYC_T_5S = 0  ,					 					
 	CB_CYC_T_10S     ,
	CB_CYC_T_30S     ,
	CB_CYC_T_60S     ,
  CB_CYC_T_5MIN    ,
  CB_CYC_T_10MIN   ,
  CB_CYC_T_20MIN   ,
  CB_CYC_T_30MIN   ,	
}bq796xx_cellbalance_cycle_time;

typedef enum{
	CB_MANUAL = 0  ,					 					
	CB_AUTO        ,
}bq796xx_cellbalance_control;

enum {
    BQ796XX_READ_GOOD,
    BQ796XX_TYPE_FAIL,
    BQ796XX_READ_LEN_FAIL,
    BQ796XX_READ_ID_FAIL,
    BQ796XX_READ_REG_FAIL,
    BQ796XX_READ_DATA_FAIL, // if we need to check fixed data
    BQ796XX_READ_CRC_FAIL,
};

typedef enum{
   BQ_EVENT_VCELL      =0  ,
	 BQ_EVENT_GPIO_ADC       ,
	 BQ_EVENT_FAULT          ,	
   BQ_EVENT_FAULTOVUV      ,
   BQ_EVENT_FAULTOTUT      ,
	 BQ_EVENT_DIR_ADDR       ,
	 BQ_EVENT_ACTIVE_CELL    ,
	 BQ_EVENT_OTHER_ERR      ,
}bq796xx_event_cb_type;

typedef enum{
    AFE_INIT_IDLE = 0,
    AFE_INIT_WAKE_UP,                               // send reset pulse(~2.5ms)
    AFE_INIT_WAKE_UP_WAIT,                          // wait for all devices wake up
	  AFE_INIT_BRORAD_SET_BMU_STACK1,
	  AFE_INIT_SET_BASE1,
	  AFE_INIT_WAKE_DEV,                              // wake up base
    AFE_INIT_FAULT_DET_ENABLE,                      // enable fault detection
    AFE_INIT_WAIT_LONG,                             // delay 100ms
    AFE_INIT_OTP_ECC_TEST_DISABLE_BASE,             // disable base OTP test
    AFE_INIT_SET_DIR,                               // set direction(for south only)
    AFE_INIT_OTP_ECC_TEST_DISABLE_BROAD,            // disable stacks OTP test
    AFE_INIT_DUMMY_WRITE,                           // dummy write

    AFE_INIT_AUTO_ADDR,                             // enable auto address
    AFE_INIT_SET_ID,                                // set devices ID
    AFE_INIT_SET_ID_WAIT,                           // delay 10ms
    AFE_INIT_DUMMY_READ,                            // dummy read
    AFE_INIT_WAIT_ID_RESPONSE,                      // delay 10ms(reset RX buffer...)
	  AFE_INIT_BRORAD_SET_BMU_STACK2,
    AFE_INIT_CHECK_ID_RESPONSE,                     // check which one is the last id(need to read)
    AFE_INIT_SET_STACK,                             // set stacks
    AFE_INIT_SET_BASE,                              // set base
    AFE_INIT_SET_TOP,                               // set top

    AFE_INIT_CHECK_IS_RING,                         // check is ring? to check reg address(0x0003) contant 0x8X= North direction assigned , 0x4X= Sorth direction assigned.
    AFE_INIT_SET_DIRECTION_FLAG_ACTIVE_CELL,        // direction flag + active cell setting

    AFE_INIT_FAULT_RESET_1,                         // fault reset setting
    AFE_INIT_FAULT_RESET_2,
    AFE_INIT_FAULT_RESET,
    AFE_INIT_OTP_SPARE,
    AFE_INIT_BQ79600_TIMEOUT,

    AFE_INIT_SWITCH_GPIO,                            // switch to 1st index for multiplexer
    AFE_INIT_TSREF_ENABLE,                           // enable 5V pull high voltage

    AFE_INIT_ADC_CONF1,
    AFE_INIT_ADC_CTRL1,

    AFE_INIT_BQ796XX_TIMEOUT,
		
		AFE_INIT_MASK_FAULT_ALL,                         // mask fault all
		AFE_INIT_BRIDGE_FAULT_MSK,                       
		AFE_INIT_CLEAR_RST_ALL,
		AFE_INIT_SET_RUN_OVUV_FUNC,
		AFE_INIT_SET_RUN_OTUT_FUNC,
		AFE_INIT_SET_CELL_BALANCE_FUNC_DISABLE,
		AFE_INIT_SET_GPIO_FUNC,
		AFE_INIT_SET_BMU_FAULT_MSK,
		AFE_RUN_AUX_ADC,
}bq796xx_init_steps_enum;

typedef enum{
  SETDIR_BRORAD_SET_BMU_STACK1 = 0      ,	
	SETDIR_SET_BASE1                      ,
	SETDIR_BASE_DIR_CHG                   ,
	SETDIR_BRORAD_REVERSE_DIRECTION       ,
	SETDIR_BRORAD_OTP_ECC_TEST_W          ,
	SETDIR_INIT_DUMMY_WRITE               ,
	SETDIR_AUTO_ADDR                      ,
	SETDIR_SET_ID                         ,
	SETDIR_INIT_DUMMY_READ                ,
	SETDIR_BRORAD_SET_BMU_STACK2          ,
	SETDIR_CHECK_ID_RESPONSE              ,	
	SETDIR_SET_STACK                      ,
	SETDIR_SET_BASE2                      ,
	SETDIR_SET_TOP                        ,
	SETDIR_CHECK_IS_RING                  ,
	SETDIR_SET_DIRECTION_FLAG_ACTIVE_CELL ,        // direction flag + active cell setting
	SETDIR_FAULT_RESET_2                  ,
	SETDIR_CLEAR_RST_ALL                  ,
	SETDIR_AFE_RUN_AUX_ADC                ,
}bq796xx_dir_set_steps_enum;

/* Exported constants --------------------------------------------------------*/ 
#define BMU_TOTAL_BOARDS															11
#define BMU_CELL_SERIES             									16

#define BQ796XX_TX_BUFFER_SIZE												512
#define BQ796XX_RX_BUFFER_SIZE												(512*1)
#define BQ796XX_RESPONE_BUFFER_SIZE										(128+6)

#define BQ796XX_UART_BAUD_RATE												1000000

#define BQ796XX_DF_INIT																0
#define BQ796XX_DF_RES_PAYLOAD_LEN										0

#define BQ796XX_DF_DEV_ADR														1
#define BQ796XX_DF_REG_ADR_MSB												2
#define BQ796XX_DF_REG_ADR_LSB												3
#define BQ796XX_DF_REG_PAYLOAD												4

#define BQ796XX_PAYLOAD_MAX_LEN												128
#define BQ796XX_DATAFRAME_MIN_LEN											5


#define BQ796XX_RES_OK																0x01
#define BQ796XX_RES_ERR																0x02

#define BQ796XX_RES_TIMEOUT													  0x00
#define BQ796XX_RES_RCV_DAT													  0x01
#define BQ796XX_RES_WAITTING                          0x02
#define BQ796XX_RES_ERR_CRC													  0xFE


#define BQ796XX_NCS_PIN																BSP_BQ796XX_NCS_PIN
#define BQ796XX_NCS_PORT															BSP_BQ796XX_NCS_PORT

#define BQ796XX_RX_PIN																BSP_BQ796XX_RX_PIN
#define BQ796XX_RX_PORT																BSP_BQ796XX_RX_PORT

// ******************** AFE registers ********************
#define BQ796XX_DEV_CONF															0x0002
#define BQ79600_ACTIVE_CELL             							0x0003

#define BQ796XX_OV_THRESH															0x0009
#define BQ796XX_UV_THRESH															0x000A
#define BQ796XX_OTUT_THRESH														0x000B


#define BQ79600_ADC_CONF1               							0x0007
#define BQ79600_GPIO_CONF1              							0x000E
#define BQ79600_OTP_SPARE13             							0x0013

#define BQ796XX_FAULT_MSK1             								0x0016
#define BQ796XX_FAULT_MSK2             								0x0017

#define BQ79600_COMM_TIMEOUT_CONF       							0x0019

#define BQ79600_DIR0_ADDR               							0x0306
#define BQ79600_DIR1_ADDR               							0x0307
#define BQ79600_COMM_CTRL               							0x0308
#define BQ79600_CONTROL1                							0x0309
#define BQ79600_CONTROL2                							0x030A
#define BQ79600_ADC_CTRL1               							0x030D
#define BQ79600_ADC_CTRL2               							0x030E
#define BQ79600_ADC_CTRL3               							0x030F

#define BQ796XX_COMM_CTRL                							0x0308
#define BQ796XX_CONTROL1                							0x0309
#define BQ796XX_CB_CELL16_CTRL												0x0318
#define BQ796XX_CB_CELL15_CTRL												0x0319
#define BQ796XX_CB_CELL14_CTRL												0x031A
#define BQ796XX_CB_CELL13_CTRL												0x031B
#define BQ796XX_CB_CELL12_CTRL												0x031C
#define BQ796XX_CB_CELL11_CTRL												0x031D
#define BQ796XX_CB_CELL10_CTRL												0x031E
#define BQ796XX_CB_CELL09_CTRL												0x031F
#define BQ796XX_CB_CELL08_CTRL												0x0320
#define BQ796XX_CB_CELL07_CTRL												0x0321
#define BQ796XX_CB_CELL06_CTRL												0x0322
#define BQ796XX_CB_CELL05_CTRL												0x0323
#define BQ796XX_CB_CELL04_CTRL												0x0324
#define BQ796XX_CB_CELL03_CTRL												0x0325
#define BQ796XX_CB_CELL02_CTRL												0x0326
#define BQ796XX_CB_CELL01_CTRL												0x0327

#define BQ796XX_BAL_CTRL1															0x032E
#define BQ796XX_BAL_CTRL2															0x032F

#define B796XX_OVUV_CTRL															0x032C
#define B796XX_OTUT_CTRL															0x032D

#define BQ79600_FAULT_RST1              							0x0331
#define BQ79600_FAULT_RST2              							0x0332
#define BQ79600_OTP_ECC_DATAIN1         							0x0343
#define BQ79600_OTP_ECC_TEST            							0x034C
#define BQ79600_DEV_STAT															0x052C
#define BQ796XX_FAULT_SUMMARY													0x052D
#define BQ796XX_FAULT_OV1															0x053C
#define BQ796XX_FAULT_OV2															0x053D
#define BQ796XX_FAULT_UV1															0x053E
#define BQ796XX_FAULT_UV2															0x053F
#define BQ796XX_FAULT_OT															0x0540
#define BQ796XX_FAULT_UT															0x0541


#define BQ79600_VCELL16_H                							0x0568
#define BQ79600_VCELL16_L                							0x0569
#define BQ79600_VCELL15_H                							0x056A
#define BQ79600_VCELL15_L                							0x056B

#define BQ79600_VCELL14_H                							0x056C
#define BQ79600_VCELL14_L                							0x056D
#define BQ79600_VCELL13_H                							0x056E
#define BQ79600_VCELL13_L                							0x056F
#define BQ79600_VCELL12_H                							0x0570
#define BQ79600_VCELL12_L                							0x0571
#define BQ79600_VCELL11_H                							0x0572
#define BQ79600_VCELL11_L                							0x0573
#define BQ79600_VCELL10_H                							0x0574
#define BQ79600_VCELL10_L                							0x0575
#define BQ79600_VCELL09_H                							0x0576
#define BQ79600_VCELL09_L                							0x0577
#define BQ79600_VCELL08_H                							0x0578
#define BQ79600_VCELL08_L                							0x0579
#define BQ79600_VCELL07_H                							0x057A
#define BQ79600_VCELL07_L                							0x057B
#define BQ79600_VCELL06_H                							0x057C
#define BQ79600_VCELL06_L                							0x057D
#define BQ79600_VCELL05_H                							0x057E
#define BQ79600_VCELL05_L                							0x057F
#define BQ79600_VCELL04_H                							0x0580
#define BQ79600_VCELL04_L                							0x0581
#define BQ79600_VCELL03_H                							0x0582
#define BQ79600_VCELL03_L                							0x0583
#define BQ79600_VCELL02_H                							0x0584
#define BQ79600_VCELL02_L                							0x0585
#define BQ79600_VCELL01_H                							0x0586
#define BQ79600_VCELL01_L                							0x0587
#define BQ79600_BUSBAR_H                							0x0588
#define BQ79600_BUSBAR_L                							0x0589
#define BQ79600_TSREF_H                							  0x058C
#define BQ79600_TSREF_L                							  0x058D
#define BQ79600_GPIO1_RES_H               						0x058E
#define BQ79600_GPIO1_RES_L               						0x058F
#define BQ79600_GPIO2_RES_H               						0x0590
#define BQ79600_GPIO2_RES_L               						0x0591
#define BQ79600_GPIO3_RES_H               						0x0592
#define BQ79600_GPIO3_RES_L               						0x0593
#define BQ79600_GPIO4_RES_H               						0x0594
#define BQ79600_GPIO4_RES_L               						0x0595
#define BQ79600_GPIO5_RES_H               						0x0596
#define BQ79600_GPIO5_RES_L               						0x0597
#define BQ79600_GPIO6_RES_H               						0x0598
#define BQ79600_GPIO6_RES_L               						0x0599
#define BQ79600_GPIO7_RES_H               						0x059A
#define BQ79600_GPIO7_RES_L               						0x059B
#define BQ79600_GPIO8_RES_H               						0x059C
#define BQ79600_GPIO8_RES_L               						0x059D

#define BQ79600_DEV_CONF1               							0x2001
#define BQ79600_COMM_TIMEOUT            							0x2005
#define BQ79600_FAULT_MSK                             0x2020
#define BQ79600_FAULT_RST               							0x2030
#define BQ79600_FAULT_SUMMARY													0x2100


// BQ79600_COMM_CTRL
#define BQ79600_BASE                    							0
#define BQ79600_STACK                   							2
#define BQ79600_TOP                     							3

// BQ79616_CONTROL1(REG Address=0x309)
//----------------------------------------------------------
#define BQ79616_ADDR_WR                  							0x01
#define BQ79616_SOFT_RESET               							0x02
#define BQ79616_GOTO_SLEEP               							0x04
#define BQ79616_GOTO_SHUTDOWN            							0x08
#define BQ79616_SEND_SLPTOACT            							0x10
#define BQ79616_SEND_WAKE           									0x20
#define BQ79616_SEND_SHUTDOWN          								0x40
#define BQ79616_DIR_SEL                  							0x80
//----------------------------------------------------------

// BQ79600_CONTROL2
#define BQ79600_TSREF_EN                							0x01

// BQ79600_DEV_CONF1
#define NFAULT_EN                       							0x04
#define FCOMM_EN                        							0x10
#define NO_ADJ_CB                                     0x40

//BQ79600_FAULT_RST
#define BQ79600_RST_PWR                 							0x01
#define BQ79600_RST_SYS                 							0x02
#define BQ79600_RST_REG                 							0x04
#define BQ79600_RST_HB                  							0x08
#define BQ79600_RST_FTONE_DET           							0x10
#define BQ79600_RST_FCOMM_DET           							0x20
#define BQ79600_RST_UART_SPI            							0x40
#define BQ79600_RST_COML_H              							0x80

//BQ79600_FAULT_MSK
#define BQ79600_MSK_PWR                               0x01
#define BQ79600_MSK_SYS                               0x02
#define BQ79600_MSK_REG                               0x04
#define BQ79600_MSK_HB                                0x08
#define BQ79600_MSK_FTONE_DET                         0x10
#define BQ79600_MSK_FCOMM_DET                         0x20
#define BQ79600_MSK_UART_SPI                          0x40
#define BQ79600_MSK_COML_H                            0x80

#define MAX_AFE_CNT                     							0x3F        //6 bits for address ID

#define BQ79616_GPIO_NUM													    8

//BQ79616 ADC_CTRL1(REG Address=0x30D)
#define BQ79616_MAIN_MODE_STOP												0x00
#define BQ79616_MAIN_MODE_B0													0x01
#define BQ79616_MAIN_MODE_B1													0x02
#define BQ79616_MAIN_GO																0x04
#define BQ79616_LPF_VCELL_EN													0x08
#define BQ79616_LPF_BB_EN													    0x10

//BQ79616 ADC_CTRL3(REG Address=0x30F)
#define BQ79616_AUX_MODE_STOP												  0x00
#define BQ79616_AUX_MODE_B0													  0x01
#define BQ79616_AUX_MODE_B1													  0x02
#define BQ79616_AUX_GO																0x04
#define BQ79616_AUX_GPIO_SEL_B0												0x08
#define BQ79616_AUX_GPIO_SEL_B1										    0x10
#define BQ79616_AUX_GPIO_SEL_B2										    0x20
#define BQ79616_AUX_GPIO_SEL_B3										    0x40

//BQ79616 OVUV_CTRL(REG Address=0x032C)
#define BQ79616_OVUV_MODE_B0													0x01
#define BQ79616_OVUV_MODE_B1													0x02
#define BQ79616_OVUV_GO																0x04
#define BQ79616_OVUV_LOCK_B0 												  0x08
#define BQ79616_OVUV_LOCK_B1 												  0x10
#define BQ79616_OVUV_LOCK_B2 												  0x20
#define BQ79616_OVUV_LOCK_B3 												  0x40
#define BQ79616_VCBDONE_THR_LOCK  									  0x80

//BQ79616 OTUT_CTRL(REG Address=0x032D)
#define BQ79616_OTUT_MODE_B0													0x01
#define BQ79616_OTUT_MODE_B1													0x02
#define BQ79616_OTUT_GO																0x04
#define BQ79616_OTUT_LOCK_B0 												  0x08
#define BQ79616_OTUT_LOCK_B1 												  0x10
#define BQ79616_OTUT_LOCK_B2 												  0x20
#define BQ79616_OTCB_THR_LOCK												  0x40

//BQ79616 FAULT_MSK1(REG Address=0x0016)
#define BQ79616_MSK_PWR 	  													0x01
#define BQ79616_MSK_SYS																0x02
#define BQ79616_MSK_COMP															0x04
#define BQ79616_MSK_OV			 												  0x08
#define BQ79616_MSK_UV			 												  0x10
#define BQ79616_MSK_OT      												  0x20
#define BQ79616_MSK_UT			 												  0x40
#define BQ79616_MSK_PROT				  									  0x80

//BQ79616 FAULT_MSK2(REG Address=0x0017)
#define BQ79616_MSK_COMM1	  													0x01
#define BQ79616_MSK_COMM2															0x02
#define BQ79616_MSK_COMM3_HB													0x04
#define BQ79616_MSK_COMM3_FTONE											  0x08
#define BQ79616_MSK_COMM3_FCOMM 										  0x10
#define BQ79616_MSK_OTP_DATA												  0x20
#define BQ79616_MSK_OTP_CRC  												  0x40

//BQ79616 BAL_CTRL2(REG Address=0x032F)
#define BQ79616_AUTO_BAL	  													0x01
#define BQ79616_BAL_GO  															0x02
#define BQ79616_BAL_ACT0     													0x04
#define BQ79616_BAL_ACT1      											  0x08
#define BQ79616_OTCB_EN         										  0x10
#define BQ79616_FLTSTOP_EN    											  0x20
#define BQ79616_CB_PAUSE     												  0x40

// ******************** ADC resolution ********************
#define BQ79656_RESOLUTION_CELL_MAIN    							0.19073f // mV
#define BQ79656_RESOLUTION_BB           							0.03052f // mV
#define BQ79656_RESOLUTION_DIE          							0.025f   // DegC for die1 and die2
#define BQ79656_RESOLUTION_BAT          							3.05f    // mV
#define BQ79656_RESOLUTION_GPIO         							0.15259f // mV
#define BQ79656_RESOLUTION_DIAG         							0.15259f // mV
#define BQ79656_RESOLUTION_TSREF        							0.16954f // mV

// ******************** V/I/T address and length ********************
#define AFE_READ_SINGLEBYTE_LEN         							7
#define AFE_READ_OVERHEAD               							6                              							// len(1) + ID(1) + Addr(2) + CRC(2)
#define CELL_V16_ADDR                   							0x0568
#define SINGLE_CELL_V_DATA_LEN          							32                                          // (16*2+AFE_READ_OVERHEAD)   always read 16s even it is not 16 cells project
#define SINGLE_CELL_V_TOTAL_LEN         							(SINGLE_CELL_V_DATA_LEN+AFE_READ_OVERHEAD)  // (16*2+AFE_READ_OVERHEAD)   always read 16s even it is not 16 cells project

#define CELL_T_ADDR                     							0x058E	                                    // GPIO1 high byte address
#define SINGLE_CELL_T_DATA_LEN          							22                                          // (2 bytes for GPIO 1)
#define SINGLE_CELL_T_TOTAL_LEN         							(SINGLE_CELL_T_DATA_LEN+AFE_READ_OVERHEAD)  // (16*2+AFE_READ_OVERHEAD)   always read 16s even it is not 16 cells project

#define PACK_I_ADDR                     							0x0506                                       // current high address
#define PACK_I_DATA_LEN                 							3                                           // Pack_I(3 bytes)
#define SINGLE_PACK_I_LEN               							(PACK_I_DATA_LEN+AFE_READ_OVERHEAD)

//cell V       I    Rvd  TSRef  GPIO 1~8
#define SINGLE_ADC_DATA_LEN             							54                                          //(16*2 + 2 + 2 + 2 + 2)         0x568~59D
#define SINGLE_ADC_TOTAL_LEN            							(SINGLE_ADC_DATA_LEN+AFE_READ_OVERHEAD)     // 54 + 6 bytes header

#define GPIO_TOTAL_PHASE    													14
#define TEMP_AVG_PHASE  															4
#define MAXDELTATEMP 																	1000L
#define INITIALTEMPADCTIME 														(TEMP_AVG_PHASE*GPIO_TOTAL_PHASE)

#define AFE_WAKE_WAIT_TIME_BASE                       10
//#define AFE_WAKE_WAIT_TIME          									(10*(BMU_TOTAL_BOARDS+1))                     //ms, base on TI tool.  base on 79616 Spec. Max wakeup time is 10ms(+1 for base device)
#define AFE_READ_WAIT_TIME          									10  //ms
#define AFE_SEND_CMD_DELAY      											500 // unit ms

#define ADDR_WR     																	1
#define SOFT_RESET   																	0x02
#define WAKEUP      																	0x20

#define BQ796XX_CMD_DELAY_MS													1
#define BQ796XX_TIMEOUT_10MS													5

#define BQ796XX_GPIOCONF_BIT_MARK_1										0xF8
#define BQ796XX_GPIOCONF_BIT_MARK_2										0xC7


#define BQ796XX_OV_MIN_VAL														2700		//mV
#define BQ796XX_OV_MAX_VAL														4475		//mV
#define BQ796XX_OV_STEP_MV														25  		//mV

#define BQ796XX_UV_MIN_VAL														1200		//mV
#define BQ796XX_UV_MAX_VAL														3100		//mV
#define BQ796XX_UV_STEP_MV														50  		//mV

#define BQ796XX_CHECK_ID_CMD_RETRY                    0
/* Exported defines --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/
#define BQ796XX_NCS_GPIO	{																										\
																.port  			= BQ796XX_NCS_PORT,								\
																.pin 				= BQ796XX_NCS_PIN,								\
																.mode				= SMP_GPIO_MODE_OUTPUT_PP,				\
																.gpio_handler	= NULL													\
										      }												
#define BQ796XX_RX_GPIO	 {																										\
																.port  			= BQ796XX_RX_PORT,								\
																.pin 				= BQ796XX_RX_PIN,									\
																.mode				= SMP_GPIO_MODE_OUTPUT_PP,				\
																.gpio_handler	= NULL													\
										      }															
/* Exported functions ------------------------------------------------------- */

typedef struct{ 
	uint16_t vcell_data[MAX_AFE_CNT][BMU_CELL_SERIES];
	uint16_t gpio_data[MAX_AFE_CNT][BQ79616_GPIO_NUM];  
	uint8_t  fault_summary[MAX_AFE_CNT];
	uint16_t fault_ov[MAX_AFE_CNT];
	uint16_t fault_uv[MAX_AFE_CNT];
	uint8_t  fault_ot[MAX_AFE_CNT];
	uint8_t  fault_ut[MAX_AFE_CNT];
	uint8_t  paser_id;
  uint8_t  comm_dir;
	uint8_t  top_stack_north_id;
	uint8_t  top_stack_south_id;
	uint8_t  ns_flag;
}bq796xx_data_t;													
					
typedef struct{
    bq796xx_ov_range                  ov_threshold;
	  bq796xx_ov_step                   ov_step_threshold;
	  bq796xx_uv_mv                     uv_threshold;
	  bq796xx_ot_threshold_p            ot_threshold;
	  bq796xx_ut_threshold_p            ut_threshold;
	  bq796xx_fun_switch                ov_enable;
	  bq796xx_fun_switch                uv_enable;
		bq796xx_fun_switch                ot_enable;
	  bq796xx_fun_switch                ut_enable;
	  bq796xx_cellbalance_cycle_time    cb_cycle_time;
	  uint8_t                           bmu_total_num;
}bq796xx_init_default_t;

typedef uint8_t(*bq796xx_CB_Fun_t)(bq796xx_data_t *bq_data, bq796xx_event_cb_type bq_event);    //BQ796XX Callback function

uint16_t smp_time_count_get(void);
void smp_time_count_set(uint16_t val);

void drv_bq796xx_delay_ms(uint32_t ms_c);
void drv_bq796xx_switch_rx_pin_type_setting(bq796xx_io_type type);													
void drv_bq796xx_rx_pin_wakeup(void);													
void drv_bq796xx_uart_puts(uint8_t *d_bytes,int16_t d_size);
void drv_bq796xx_clear_fifobuffer(void);

void drv_bq796xx_command(uint8_t cmd_type, uint8_t dev_id, uint16_t reg_addr, uint8_t datalen, uint8_t *data_array, uint32_t delayms);
void drv_bq796xx_command_framing(uint8_t cmd_type, uint8_t dev_id, uint16_t reg_addr, uint8_t datalen, uint8_t *data_array, uint32_t delayms);
	
uint8_t drv_bq796xx_init(void);
uint8_t drv_bq796xx_start_setting(uint8_t maxcnt, uint8_t dir);													
uint8_t drv_bq796xx_Read_AFE_ALL_VCELL(bq796xx_AFE_GPIO_stack is_stack,uint8_t dev_id,uint32_t delays);
uint8_t drv_bq796xx_Set_AFE_GPIO_type(bq796xx_AFE_GPIO_stack is_stack,uint8_t dev_id,bq796xx_AFE_GPIO_Type GPIO_type,bq796xx_AFE_GPIO GPIO_Num,uint32_t delays);
uint8_t drv_bq796xx_Start_AFE_ADC(bq796xx_AFE_GPIO_stack is_stack,uint8_t dev_id,uint32_t delays);
uint8_t drv_bq796xx_Stop_AFE_ADC(bq796xx_AFE_GPIO_stack is_stack,uint8_t dev_id,uint32_t delays);			
uint8_t drv_bq796xx_Read_AFE_ALL_ADC(bq796xx_AFE_GPIO_stack is_stack,uint8_t dev_id,uint32_t delays);

uint8_t drv_bq796xx_Goto_ShutDownMode(uint32_t delays);
uint8_t drv_bq796xx_Goto_SleepMode(uint32_t delays);
uint8_t drv_bq796xx_Send_WakeupAll(uint32_t delays);
uint8_t drv_bq796xx_Set_Mask_FaultAll(bq796xx_AFE_GPIO_stack is_stack, uint8_t dev_id, uint32_t delays);
uint8_t drv_bq796xx_Set_BroadCast_Mask_FaultAll(uint32_t delays);
uint8_t drv_bq796xx_Set_BroadCast_Mask_FaultSel(uint8_t msk1,uint8_t msk2,uint32_t delays);
uint8_t drv_bq796xx_Clear_FaultRstAll(uint32_t delays);

uint8_t drv_bq796xx_Clear_Mask_Fault_OVUV(bq796xx_AFE_GPIO_stack is_stack, uint8_t dev_id, uint32_t delays);
uint8_t drv_bq796xx_Clear_Mask_Fault_OTUT(bq796xx_AFE_GPIO_stack is_stack, uint8_t dev_id, uint32_t delays);

uint8_t drv_bq796xx_Read_Stack_FaultSummary(uint32_t delays);
uint8_t drv_bq796xx_Read_Base_FaultSummary(uint32_t delays);

uint8_t drv_bq796xx_Set_Stack_OV(bq796xx_ov_range ov_range_mv,bq796xx_ov_step ov_step_mv ,uint32_t delays);
uint8_t drv_bq796xx_Set_Stack_UV(bq796xx_uv_mv uv_mv,uint32_t delays);
uint8_t drv_bq796xx_Run_OVUV(uint32_t delays);
uint8_t drv_bq796xx_Read_Stack_FaultOVUV(uint32_t delays);

uint8_t drv_bq796xx_Set_Stack_OTUT(bq796xx_ot_threshold_p ot_th_pct,bq796xx_ut_threshold_p ut_th_pct,uint32_t delays);
uint8_t drv_bq796xx_Set_Stack_OTUT_Associate(bq796xx_AFE_GPIO afe_gpio,uint32_t delays);
uint8_t drv_bq796xx_Run_OTUT(uint32_t delays);
uint8_t drv_bq796xx_Read_Stack_FaultOTUT(uint32_t delays);

uint8_t drv_bq796xx_Set_Stack_CellBalanceTime(bq796xx_cellbalance_time cb_time,uint32_t delays);
uint8_t drv_bq796xx_Set_Stack_CellBalanceCycleTime(bq796xx_cellbalance_cycle_time cb_cyc_time,uint32_t delays);
uint8_t drv_bq796xx_Stack_CellBalanceStarting(bq796xx_cellbalance_control cb_ctrl, uint32_t delays);

void drv_bq796xx_uart_event_handler(uart_evt_type p_evt);
uint8_t drv_bq796xx_data_frame_parser(void);													
uint8_t drv_bq796xx_check_respone_event(void);

//Callback Register
int8_t bq796xx_event_RegisteCB(bq796xx_CB_Fun_t callbackfunc);
int8_t bq796xx_event_UnregisteCB(void);

uint8_t drv_bq796xx_Init_Steps(bq796xx_wake_tone_switch wake_tone_sw, bq796xx_init_steps_enum *afe_phase,uint8_t maxcnt, uint8_t dir, uint8_t *step_complete_f, uint8_t *before_delay_ms);

int8_t drv_bq796xx_init_default_load(bq796xx_init_default_t in_load_data);

uint8_t drv_bq796xx_CellBalance_1to8_set(uint8_t bmu_id, uint8_t cell_d_en, bq796xx_cellbalance_time cb_time, uint32_t delays);
uint8_t drv_bq796xx_CellBalance_9to16_set(uint8_t bmu_id, uint8_t cell_d_en, bq796xx_cellbalance_time cb_time, uint32_t delays);
uint8_t drv_bq796xx_Stack_CellBalance_1to8_clear(bq796xx_AFE_GPIO_stack is_stack, uint8_t dev_id, uint32_t delays);
uint8_t drv_bq796xx_Stack_CellBalance_9to16_clear(bq796xx_AFE_GPIO_stack is_stack, uint8_t dev_id, uint32_t delays);
uint8_t drv_bq796xx_direction_set_steps(uint8_t ns_dir_bmu_cnt,bq796xx_dir_set_steps_enum *afe_phase,uint8_t maxcnt, uint8_t dir, uint8_t *step_complete_f, uint8_t *before_delay_ms);

void fill_data4_payload(uint8_t *payload,uint8_t b1,uint8_t b2,uint8_t b3,uint8_t b4);
													
extern uint8_t bq796xx_res_buf[BQ796XX_RESPONE_BUFFER_SIZE];													
extern uint16_t bq796xx_res_buf_c;
													
#endif /* __SMP_DRV_BQ796XX_H */

/************************ (C) COPYRIGHT Simplo all right reserved *****END OF FILE****/

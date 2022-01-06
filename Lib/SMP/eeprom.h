/**
  ******************************************************************************
  * @file    EEPROM_Emulation/inc/eeprom.h 
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    29-April-2016
  * @brief   This file contains all the functions prototypes for the EEPROM 
  *          emulation firmware library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Define of the return value */
typedef enum {
  EE_OK = 0,
  EE_ERROR,
  EE_NO_VALID_PAGE,
  EE_ERASE_ERROR,
  EE_FORMAT_ERROR,
  EE_WRITE_ERROR,
  EE_ERROR_NOVALID_PAGE,
  EE_NO_DATA,
  EE_INVALID_VIRTUALADRESS,
  EE_TRANSFER_ERROR,
 
  /* Internal return code */
  EE_PAGE_NOTERASED,
  EE_PAGE_ERASED,
  EE_PAGE_FULL,
} EE_Status;

/* Variables' number */
#define NB_OF_VAR             ((uint8_t)0xff)


//For WGE-101 or IRB.
//Boot code use
#define EE_JUMP_BOOT_DOING		((uint16_t)0x5555)

#define EE_ADDB_MAIN_CHECKSUM_H	((uint16_t)0x0000)
#define EE_ADDB_MAIN_CHECKSUM_L	((uint16_t)0x0001)
#define EE_ADDB_MAIN_SIZE_H		((uint16_t)0x0002)
#define EE_ADDB_MAIN_SIZE_L		((uint16_t)0x0003)
#define EE_ADDB_JUMP_BOOT		((uint16_t)0x0004)

//Main code use
#define EE_ADDM_SYS_MAC_H		((uint16_t)0x0020)
#define EE_ADDM_SYS_MAC_M		((uint16_t)0x0021)
#define EE_ADDM_SYS_MAC_L		((uint16_t)0x0022)


#define EE_ADDM_HR_ECG_SR		((uint16_t)0x0040) //Sample rate
#define EE_ADDM_HR_APOT			((uint16_t)0x0041) //Auto Power Off Time
#define EE_ADDM_HR_FCI			((uint16_t)0x0042) //Cut off interval of ECG logging file
#define HR_EE_ADDM_MIN				EE_ADDM_HR_ECG_SR
#define HR_EE_ADDM_MAX				EE_ADDM_HR_FCI

#define EE_ADDM_SYS_ERR_MSG			((uint16_t)0x0050)
#define EE_ADDM_SYS_BT_TIME			((uint16_t)0x0051)
#define EE_ADDM_SYS_CHG_VAL			((uint16_t)0x0052)
#define EE_ADDM_SYS_W_SENSOR_SWITCH	((uint16_t)0x0053)
#define EE_ADDM_SYS_ACC_RANGE		((uint16_t)0x0054)
#define EE_ADDM_SYS_GYRO_RANGE		((uint16_t)0x0055)
#define SYS_EE_ADDM_MIN				EE_ADDM_SYS_ERR_MSG
#define SYS_EE_ADDM_MAX				EE_ADDM_SYS_GYRO_RANGE

// MFG code use
#define EE_ADDM_MFG_SN_0			((uint16_t)0x0060) //Serial number
#define EE_ADDM_MFG_SN_1			((uint16_t)0x0061) //Serial number
#define EE_ADDM_MFG_SN_2			((uint16_t)0x0062) //Serial number
#define EE_ADDM_MFG_SN_3			((uint16_t)0x0063) //Serial number
#define EE_ADDM_MFG_SN_4			((uint16_t)0x0064) //Serial number
#define EE_ADDM_MFG_SN_5			((uint16_t)0x0065) //Serial number
#define EE_ADDM_MFG_SN_6			((uint16_t)0x0066) //Serial number
#define EE_ADDM_MFG_SN_7			((uint16_t)0x0067) //Serial number
#define EE_ADDM_MFG_SN_8			((uint16_t)0x0068) //Serial number
#define EE_ADDM_MFG_SN_9			((uint16_t)0x0069) //Serial number
#define EE_ADDM_MFG_SN_10			((uint16_t)0x006A) //Serial number
#define EE_ADDM_MFG_SN_11			((uint16_t)0x006B) //Serial number
#define EE_ADDM_MFG_SN_12			((uint16_t)0x006C) //Serial number
#define EE_ADDM_MFG_SN_13			((uint16_t)0x006D) //Serial number
#define EE_ADDM_MFG_SN_14			((uint16_t)0x006E) //Serial number
#define EE_ADDM_MFG_SN_15			((uint16_t)0x006F) //Serial number
#define EE_ADDM_MFG_HW_VER_0		((uint16_t)0x0070) //Hardware version
#define EE_ADDM_MFG_HW_VER_1		((uint16_t)0x0071) //Hardware version
#define EE_ADDM_MFG_HW_VER_2		((uint16_t)0x0072) //Hardware version
#define EE_ADDM_MFG_HW_VER_3		((uint16_t)0x0073) //Hardware version
#define EE_ADDM_MFG_HW_VER_4		((uint16_t)0x0074) //Hardware version
#define EE_ADDM_MFG_HW_VER_5		((uint16_t)0x0075) //Hardware version
#define EE_ADDM_MFG_HW_VER_6		((uint16_t)0x0076) //Hardware version
#define EE_ADDM_MFG_HW_VER_7		((uint16_t)0x0077) //Hardware version
#define EE_ADDM_MFG_HW_VER_8		((uint16_t)0x0078) //Hardware version
#define EE_ADDM_MFG_HW_VER_9		((uint16_t)0x0079) //Hardware version
#define EE_ADDM_MFG_HW_VER_10		((uint16_t)0x007A) //Hardware version
#define EE_ADDM_MFG_HW_VER_11		((uint16_t)0x007B) //Hardware version
#define EE_ADDM_MFG_HW_VER_12		((uint16_t)0x007C) //Hardware version
#define EE_ADDM_MFG_HW_VER_13		((uint16_t)0x007D) //Hardware version
#define EE_ADDM_MFG_HW_VER_14		((uint16_t)0x007E) //Hardware version
#define EE_ADDM_MFG_HW_VER_15		((uint16_t)0x007F) //Hardware version
#define EE_ADDM_MFG_PPID_0			((uint16_t)0x0080) //PPID
#define EE_ADDM_MFG_PPID_1			((uint16_t)0x0081) //PPID
#define EE_ADDM_MFG_PPID_2			((uint16_t)0x0082) //PPID
#define EE_ADDM_MFG_PPID_3			((uint16_t)0x0083) //PPID
#define EE_ADDM_MFG_PPID_4			((uint16_t)0x0084) //PPID
#define EE_ADDM_MFG_PPID_5			((uint16_t)0x0085) //PPID
#define EE_ADDM_MFG_PPID_6			((uint16_t)0x0086) //PPID
#define EE_ADDM_MFG_PPID_7			((uint16_t)0x0087) //PPID
#define EE_ADDM_MFG_PPID_8			((uint16_t)0x0088) //PPID
#define EE_ADDM_MFG_PPID_9			((uint16_t)0x0089) //PPID
#define EE_ADDM_MFG_PPID_10			((uint16_t)0x008A) //PPID
#define EE_ADDM_MFG_PPID_11			((uint16_t)0x008B) //PPID
#define EE_ADDM_MFG_PPID_12			((uint16_t)0x008C) //PPID
#define EE_ADDM_MFG_PPID_13			((uint16_t)0x008D) //PPID
#define EE_ADDM_MFG_PPID_14			((uint16_t)0x008E) //PPID
#define EE_ADDM_MFG_PPID_15			((uint16_t)0x008F) //PPID
#define EE_ADDM_MFG_MPD_0			((uint16_t)0x0090) //MP Date
#define EE_ADDM_MFG_MPD_1			((uint16_t)0x0091) //MP Date
#define EE_ADDM_MFG_MPD_2			((uint16_t)0x0092) //MP Date
#define EE_ADDM_MFG_MPD_3			((uint16_t)0x0093) //MP Date
#define EE_ADDM_MFG_MPD_4			((uint16_t)0x0094) //MP Date
#define EE_ADDM_MFG_FUD_0			((uint16_t)0x0095) //FUD
#define EE_ADDM_MFG_FUD_1			((uint16_t)0x0096) //FUD
#define EE_ADDM_MFG_FUD_2			((uint16_t)0x0097) //FUD
#define EE_ADDM_MFG_FUD_3			((uint16_t)0x0098) //FUD
#define EE_ADDM_MFG_FUD_4			((uint16_t)0x0099) //FUD
#define EE_ADDM_MFG_MODEL_NAME_0	((uint16_t)0x009A) //Model name
#define EE_ADDM_MFG_MODEL_NAME_1 	((uint16_t)0x009B) //Model name
#define EE_ADDM_MFG_MODEL_NAME_2	((uint16_t)0x009C) //Model name
#define EE_ADDM_MFG_MODEL_NAME_3 	((uint16_t)0x009D) //Model name
#define EE_ADDM_MFG_MODEL_NAME_4	((uint16_t)0x009E) //Model name
#define EE_ADDM_MFG_MODEL_NAME_5	((uint16_t)0x009F) //Model name
#define EE_ADDM_MFG_MODEL_NAME_6	((uint16_t)0x00A0) //Model name
#define EE_ADDM_MFG_MODEL_NAME_7	((uint16_t)0x00A1) //Model name
#define EE_ADDM_MFG_MODEL_NAME_8	((uint16_t)0x00A2) //Model name
#define EE_ADDM_MFG_MODEL_NAME_9	((uint16_t)0x00A3) //Model name
#define EE_ADDM_MFG_MODEL_NAME_10	((uint16_t)0x00A4) //Model name
#define EE_ADDM_MFG_MODEL_NAME_11	((uint16_t)0x00A5) //Model name
#define EE_ADDM_MFG_MODEL_NAME_12	((uint16_t)0x00A6) //Model name
#define EE_ADDM_MFG_MODEL_NAME_13	((uint16_t)0x00A7) //Model name
#define EE_ADDM_MFG_MODEL_NAME_14	((uint16_t)0x00A8) //Model name
#define EE_ADDM_MFG_MODEL_NAME_15	((uint16_t)0x00A9) //Model name
#define EE_ADDM_MFG_ADC_CALIB_MAX_0	((uint16_t)0x00AA) //ADC calibration maximum value 0
#define EE_ADDM_MFG_ADC_CALIB_MAX_1	((uint16_t)0x00AB) //ADC calibration maximum value 1
#define EE_ADDM_MFG_ADC_CALIB_MIN_0	((uint16_t)0x00AC) //ADC calibration minimum value 0
#define EE_ADDM_MFG_ADC_CALIB_MIN_1	((uint16_t)0x00AD) //ADC calibration minimum value 1
#define EE_ADDM_MFG_BTMAC_0			((uint16_t)0x00AE) //BT MAC
#define EE_ADDM_MFG_BTMAC_1			((uint16_t)0x00AF) //BT MAC
#define EE_ADDM_MFG_BTMAC_2			((uint16_t)0x00B0) //BT MAC
#define EE_ADDM_MFG_BTMAC_3			((uint16_t)0x00B1) //BT MAC
#define EE_ADDM_MFG_BTMAC_4			((uint16_t)0x00B2) //BT MAC
#define EE_ADDM_MFG_BTMAC_5			((uint16_t)0x00B3) //BT MAC
#define MFG_EE_ADDM_MIN				EE_ADDM_MFG_SN_0
#define MFG_EE_ADDM_MAX				EE_ADDM_MFG_BTMAC_5

// error code use (Common)
#define EE_ADDM_ERR_OSCILLATOR		((uint16_t)0x1000) //oscillator config initial
#define EE_ADDM_ERR_CLOCK			((uint16_t)0x1001) //clock config initial
#define EE_ADDM_ERR_RTC				((uint16_t)0x1002) //RTC initial
#define EE_ADDM_ERR_TIMER			((uint16_t)0x1003) //Timer initial
#define EE_ADDM_ERR_UART			((uint16_t)0x1004) //UART initial
#define EE_ADDM_ERR_I2C				((uint16_t)0x1005) //I2C initial
#define EE_ADDM_ERR_SPI				((uint16_t)0x1006) //SPI initial
#define EE_ADDM_ERR_ADC				((uint16_t)0x1007) //ADC initial
#define EE_ADDM_ERR_SDIO			((uint16_t)0x1008) //SDIO initial
#define ERR_COM_EE_ADDM_MIN			EE_ADDM_ERR_OSCILLATOR
#define ERR_COM_EE_ADDM_MAX			EE_ADDM_ERR_SDIO

// error code use (IRB)
#define EE_ADDM_ERR_GAUGE			((uint16_t)0x1100) //Gauge initial
#define EE_ADDM_ERR_BT				((uint16_t)0x1101) //BT initial
#define EE_ADDM_ERR_BMI160			((uint16_t)0x1102) //BMI160 initial
#define ERR_IRB_EE_ADDM_MIN			EE_ADDM_ERR_GAUGE
#define ERR_IRB_EE_ADDM_MAX			EE_ADDM_ERR_BMI160

// error code use (common time base)
#define EE_ADDM_ERR_OS_Y_M			((uint16_t)0x1500) //Err oscillator time year and month
#define EE_ADDM_ERR_OS_D_H			((uint16_t)0x1501) //Err oscillator time date and  hour
#define EE_ADDM_ERR_OS_M_S			((uint16_t)0x1502) //Err oscillator time minute and second
#define EE_ADDM_ERR_CLK_Y_M			((uint16_t)0x1503) //Err CLK time year and month
#define EE_ADDM_ERR_CLK_D_H			((uint16_t)0x1504) //Err CLK time date and  hour
#define EE_ADDM_ERR_CLK_M_S			((uint16_t)0x1505) //Err CLK time minute and second
#define EE_ADDM_ERR_RTC_Y_M			((uint16_t)0x1506) //Err RTC time year and month
#define EE_ADDM_ERR_RTC_D_H			((uint16_t)0x1507) //Err RTC time date and  hour
#define EE_ADDM_ERR_RTC_M_S			((uint16_t)0x1508) //Err RTC time minute and second
#define EE_ADDM_ERR_TIMER_Y_M		((uint16_t)0x1509) //Err Timer time year and month
#define EE_ADDM_ERR_TIMER_D_H		((uint16_t)0x150A) //Err Timer time date and  hour
#define EE_ADDM_ERR_TIMER_M_S		((uint16_t)0x150B) //Err Timer time minute and second
#define EE_ADDM_ERR_UART_Y_M		((uint16_t)0x150C) //Err UART time year and month
#define EE_ADDM_ERR_UART_D_H		((uint16_t)0x150D) //Err UART time date and  hour
#define EE_ADDM_ERR_UART_M_S		((uint16_t)0x150E) //Err UART time minute and second
#define EE_ADDM_ERR_I2C_Y_M			((uint16_t)0x150F) //Err I2C time year and month
#define EE_ADDM_ERR_I2C_D_H			((uint16_t)0x1510) //Err I2C time date and  hour
#define EE_ADDM_ERR_I2C_M_S			((uint16_t)0x1511) //Err I2C time minute and second
#define EE_ADDM_ERR_SPI_Y_M			((uint16_t)0x1512) //Err SPI time year and month
#define EE_ADDM_ERR_SPI_D_H			((uint16_t)0x1513) //Err SPI time date and  hour
#define EE_ADDM_ERR_SPI_M_S			((uint16_t)0x1514) //Err SPI time minute and second
#define EE_ADDM_ERR_ADC_Y_M			((uint16_t)0x1515) //Err ADC time year and month
#define EE_ADDM_ERR_ADC_D_H			((uint16_t)0x1516) //Err ADC time date and  hour
#define EE_ADDM_ERR_ADC_M_S			((uint16_t)0x1517) //Err ADC time minute and second
#define EE_ADDM_ERR_SDIO_Y_M		((uint16_t)0x1518) //Err SDIO time year and month
#define EE_ADDM_ERR_SDIO_D_H		((uint16_t)0x1519) //Err SDIO time date and  hour
#define EE_ADDM_ERR_SDIO_M_S		((uint16_t)0x151A) //Err SDIO time minute and second
#define ERR_COM_T_EE_ADDM_MIN		EE_ADDM_ERR_OS_Y_M
#define ERR_COM_T_EE_ADDM_MAX		EE_ADDM_ERR_SDIO_M_S
// error code use (IRB time base)
#define EE_ADDM_ERR_GAUGE_Y_M		((uint16_t)0x1600) //Err Gauge time year and month
#define EE_ADDM_ERR_GAUGE_D_H		((uint16_t)0x1601) //Err Gauge time date and  hour
#define EE_ADDM_ERR_GAUGE_M_S		((uint16_t)0x1602) //Err Gauge time minute and second
#define EE_ADDM_ERR_BT_Y_M			((uint16_t)0x1603) //Err BT time year and month
#define EE_ADDM_ERR_BT_D_H			((uint16_t)0x1604) //Err BT time date and  hour
#define EE_ADDM_ERR_BT_M_S			((uint16_t)0x1605) //Err BT time minute and second
#define EE_ADDM_ERR_BMI160_Y_M		((uint16_t)0x1606) //Err BMI160 time year and month
#define EE_ADDM_ERR_BMI160_D_H		((uint16_t)0x1607) //Err BMI160 time date and  hour
#define EE_ADDM_ERR_BMI160_M_S		((uint16_t)0x1608) //Err BMI160 time minute and second
#define ERR_IRB_T_EE_ADDM_MIN		EE_ADDM_ERR_GAUGE_Y_M
#define ERR_IRB_T_EE_ADDM_MAX		EE_ADDM_ERR_BMI160_M_S
/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
EE_Status EE_Init(void);
EE_Status EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data);
EE_Status EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);

#endif /* __EEPROM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

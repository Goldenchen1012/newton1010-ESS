/**
  ******************************************************************************
  * @file        SmpParameterId.h
  * @author      Johnny
  * @version     v1.0
  * @date        2021/12/14
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */


#ifndef _SMP_PARAMETER_ID_H
#define	_SMP_PARAMETER_ID_H

#define	SMP_PAR_MAGIC_CODE               0x00 
#define	SMP_PAR_ID_HW_VERSION            0x01
#define	SMP_PAR_ID_FW_VERSION            0x02
#define	SMP_PAR_ID_FW_BUILD_DATE_TIME    0x03
#define SMP_PAR_ID_MID                   0x04
#define	SMP_PAR_ID_MODEL_NAME            0x05
#define	SMP_PAR_ID_MODULE_NUMBER         0x06
#define	SMP_PAR_ID_PART_NUMBER           0x07
#define SMP_PAR_ID_CELL_TYPE             0x08
#define SMP_PAR_ID_BAUDRATE              0x09
#define SMP_PAR_ID_NOTE_MESSAGE			0x0A

#define	SMP_PAR_ID_CURRENT_BOARD_ID      0x20
#define	SMP_PAR_ID_ZERO_CURRENT          0x21
#define	SMP_PAR_ID_PRE_DHG_TIME          0x22
#define	SMP_PAR_ID_MOS_ON_DIFF_VOLTAGE   0x23
#define	SMP_PAR_ID_DESIGNED_CAPACITY     0x24
#define	SMP_PAR_ID_MAX_CHG_DHG_CURRENT   0x25
#define	SMP_PAR_ID_PEAK_CURRENT          0x26
#define	SMP_PAR_ID_BMU_NUMBER        	  	0x27
#define	SMP_PAR_ID_TAPER_CURRENT         0x28
#define	SMP_PAR_ID_FLAT_VOLTAGE          0x29
#define	SMP_PAR_ID_TERMINATE_VOLTAGE     0x2A
#define	SMP_PAR_ID_QMAX                  0x2B
#define	SMP_PAR_ID_QUPDATE_TIMES         0x2C
#define	SMP_PAR_ID_RM                    0x2D
#define	SMP_PAR_ID_CYCLE_COUNT           0x2E
#define	SMP_PAR_ID_IDLE_TIME             0x2F
#define	SMP_PAR_ID_SHUTDOWN              0x30
#define	SMP_PAR_ID_SHORT_CIRCUIT         0x31
#define	SMP_PAR_ID_COMMUNICATION         0x32
#define	SMP_PAR_ID_CELL_NTC_FLAG             0x33
#define	SMP_PAR_ID_N_TERMINAL            0x34
#define	SMP_PAR_ID_ENABLE_CHG_SOC        0x35
#define	SMP_PAR_ID_RATE_VOLTAGE          0x36
#define	SMP_PAR_ID_PACK_ERROR				0x37
#define	SMP_PAR_ID_INSULATION_RESISTANCE	0x38

#define	SMP_PAR_ID_OVP_PROTECT           0x60
#define	SMP_PAR_ID_UVP_PROTECT           0x61
#define	SMP_PAR_ID_COTP_PROTECT          0x62
#define	SMP_PAR_ID_CUTP_PROTECT          0x63
#define	SMP_PAR_ID_DOTP_PROTECT          0x64
#define	SMP_PAR_ID_DUTP_PROTECT          0x65
#define	SMP_PAR_ID_DTP_PROTECT           0x66
#define	SMP_PAR_ID_DOCP_PROTECT          0x67
#define	SMP_PAR_ID_COCP_PROTECT          0x68
#define	SMP_PAR_ID_MOSOT_PROTECT         0x69
#define	SMP_PAR_ID_SOC_PROTECT           0x6A
#define	SMP_PAR_ID_PF_PROTECT            0x6B
#define	SMP_PAR_ID_DVP_PROTECT           0x6C
#define	SMP_PAR_ID_CURRENT_DIFF          0x6D
#define	SMP_PAR_ID_DVB_PROTECT           0x6E


#define	SMP_PAR_ID_BALANCE_DUTY          0x80
#define	SMP_PAR_ID_BALANCE_CHG           0x81
#define	SMP_PAR_ID_BALANCE_DHG           0x82
#define	SMP_PAR_ID_BALANCE_RLX           0x83


#define	SMP_PAR_ID_CALIB_RACK            0x90
#define	SMP_PAR_ID_CALIB_VB              0x91
#define	SMP_PAR_ID_CALIB_CURR            0x92
#define	SMP_PAR_ID_CALIB_CELLVOL         0x93


#define SMP_PAR_ID_OCV_INDEX             0xA0
#define SMP_PAR_ID_OCV_TABLE             0xA1


#define SMP_PAR_ID_RA_INDEX              0xB0
#define SMP_PAR_ID_RA_TABLE              0xB1

#define SMP_PAR_ID_CUSTOM_MAX            0xC0
#define SMP_PAR_ID_CUSTOM_DATA           0xC1

#endif




/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/   



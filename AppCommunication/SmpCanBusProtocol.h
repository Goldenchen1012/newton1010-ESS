/**
  ******************************************************************************
  * @file        SmpCanBusProtocol.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/10/25
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */
  
#ifndef _SMP_CANBUS_PROTOCOL_H_
#define _SMP_CANBUS_PROTOCOL_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/

//	Function define
#define SMP_CAN_FUN_BASE_RX			0x00
#define SMP_CAN_FUN_BASE_TX			0x01

#define	SMP_CAN_FUN_CMD_RX			0x02
#define	SMP_CAN_FUN_CMD_TX			0x03

#define	SMP_CAN_FUN_COMMON_RX		0x04
#define	SMP_CAN_FUN_COMMON_TX		0x05

#define	SMP_CAN_FUN_DEBUG_RX		0x0A
#define	SMP_CAN_FUN_DEBUG_TX		0x0B

#define SMP_CAN_FUN_FU_RX	 		0x0C
#define SMP_CAN_FUN_FU_TX			0x0D

#define SMP_CAN_FUN_DETAIL_RX	 	0x0E
#define SMP_CAN_FUN_DETAIL_TX		0x0F

//----------------------------------------------------
//	SMP_CAN_FUN_BASE_RX
//	SMP_CAN_FUN_BASE_TX
#define	SMP_BASE_SCU_ID_OBJ_INDEX				0x00
#define	SMP_BASE_SYSTEM_FLAG_OBJ_INDEX			0x01
#define	SMP_BASE_RM_QMAX_OBJ_INDEX				0x02
#define	SMP_BASE_CURRENT_OBJ_INDEX				0x03
#define	SMP_BASE_FCC_OBJ_INDEX					0x04
#define	SMP_BASE_VB_OBJ_INDEX					0x05
#define	SMP_BASE_MIN_MAX_VALUE_OBJ_INDEX		0x06
#define	SMP_BASE_VALID_BMU_OBJ_INDEX			0x07
#define	SMP_BASE_SCU_NTC_VOLTAGE_OBJ_INDEX		0x08
#define	SMP_BASE_SCU_NTC_TEMP_OBJ_INDEX			0x09
#define	SMP_BASE_QSTART_RSOC_SOH_OBJ_INDEX		0x0A
#define	SMP_BASE_RAMSOC_ENDSOC_DSOC_SOC0_OBJ_INDEX	0x0B
#define	SMP_BASE_QPASS_RPASS_OBJ_INDEX				0x0C
#define	SMP_BASE_CYCLE_COUNT_OBJ_INDEX				0x0D

#define	SMP_BASE_DETAIL_MSG_SEND_END_OBJ_INDEX		0x10

//----------------------------------------------
//	SMP_CAN_FUN_CMD_RX
#define	SMP_CMD_PAR_WR_OBJ_INDEX             	0x00
#define	SMP_CMD_PAR_RD_OBJ_INDEX             	0x01
#define	SMP_CMD_RTC_OBJ_INDEX					0x02
#define	SMP_CMD_ADC_OBJ_INDEX					0x03
#define	SMP_CMD_RELAY_ON_OBJ_INDEX				0x04
#define	SMP_CMD_RELAY_OFF_OBJ_INDEX				0x05
#define	SMP_CMD_CHECKSUM_OBJ_INDEX				0x06
#define	SMP_CMD_PF_FLAG_OBJ_INDEX				0x07
#define	SMP_CMD_CLEAN_DATA_OBJ_INDEX			0x08
#define	SMP_CMD_FAULT_LOG_OBJ_INDEX				0x09
#define	SMP_CMD_SOC_OBJ_INDEX					0x0A


#define	SMP_WR_RTC_SUB_INDEX					0x00
#define	SMP_RD_RTC_SUB_INDEX					0x01

#define	SMP_FW_CHECKSUM_SUB_INDEX				0x01
#define	SMP_PAR_CHECKSUM_SUB_INDEX				0x02
#define	SMP_CAL_PAR_CHECKSUM_SUB_INDEX			0x03

#define	SMP_PF_FLAG_CLEAN_ALL_SUB_INDEX			0x00
#define	SMP_PF_FLAG_CLEAN_OVP_SUB_INDEX			0x01
#define	SMP_PF_FLAG_CLEAN_UVP_SUB_INDEX			0x02

#define	SMP_CLEAN_ALL_DATA_SUB_INDEX			0x00
#define	SMP_CLEAN_CYCLE_COUNT_SUB_INDEX			0x01
#define	SMP_CLEAN_FAULT_LOG1_SUB_INDEX			0x02
#define	SMP_CLEAN_FAULT_LOG2_SUB_INDEX			0x03

#define	SMP_READ_FAULT_LOG1_NUM_SUB_INDEX		0x01
#define	SMP_READ_FAULT_LOG2_NUM_SUB_INDEX		0x02
#define	SMP_READ_FAULT_LOG1_DATA_SUB_INDEX		0x10
#define	SMP_READ_FAULT_LOG2_DATA_SUB_INDEX		0x11
#define	SMP_RET_FAULT_LOG1_DATA_SUB_INDEX		0x20
#define	SMP_RET_FAULT_LOG2_DATA_SUB_INDEX		0x40

#define	SMP_UPDATE_SOC_SUB_INDEX				0x01
#define	SMP_SET_SOC_VALUE_SUB_INDEX				0x02

//--------------------------------------------------
//	SMP_CAN_FUN_COMMON_RX
#define	SMP_COMMON_FIND_FIRST_SCU_OBJ_INDEX			0x00
#define	SMP_COMMON_RESET_SCU_ID_OBJ_INDEX			0x01
#define	SMP_COMMON_GET_SCUID_OBJ_INDEX				0x02
#define	SMP_COMMON_STOP_ID_ASSIGN_OBJ_INDEX			0x03

//	SMP_CAN_FUN_DEBUG_RX
#define	SMP_DEBUG_MODE_CONTROL_OBJ_INDEX			0x00
#define	SMP_DEBUG_SIMU_CURR_ADC_OBJ_INDEX			0x01
#define	SMP_DEBUG_SIMU_VBAT_ADC_OBJ_INDEX			0x02
#define	SMP_DEBUG_PKG_RX_OBJ_INDEX					0x03
#define	SMP_DEBUG_PKG_TX_OBJ_INDEX					0x04
#define	SMP_DEBUG_CELLV_SIMU_OBJ_INDEX				0x05
#define	SMP_DEBUG_NTCV_SIMU_OBJ_INDEX				0x06
#define	SMP_DEBUG_NTCT_SIMU_OBJ_INDEX				0x07
#define	SMP_DEBUG_GPIO_OBJ_INDEX					0x08
#define	SMP_DEBUG_SIMU_CURRENT_VALUE_OBJ_INDEX		0x09
#define	SMP_DEBUG_SIMU_VBAT_VALUE_OBJ_INDEX			0x0A
#define	SMP_DEBUG_RELAY_CONTROL_OBJ_INDEX			0x0B
#define	SMP_DEBUG_SCU_TEMP_SIMU_OBJ_INDEX			0x0C

#define	SMP_DEBUG_CLEAR_TEST_COUNT_OBJ_INDEX		0x80

#define	SMP_SIMU_MODE_SUB_INDEX						0x01
#define	SMP_ENG_MODE_SUB_INDEX						0x02

#define	SMP_PS_RELAY_SUB_INDEX						0x01
#define	SMP_P_MAIN_RELAY_SUB_INDEX					0x02
#define	SMP_N_MAIN_RELAY_SUB_INDEX					0x03
#define	SMP_PRE_RELAY_SUB_INDEX						0x04
#define	SMP_FAN_RELAY_SUB_INDEX						0x05

//-----------------------------------------------------------
//	SMP_CAN_FUN_DETAIL_TX
#define	SMP_DETAIL_CELL_VOLTAGE_OBJ_INDEX			0x00
#define	SMP_DETAIL_CELL_NTC_VOLTAGE_OBJ_INDEX		0x01
#define	SMP_DETAIL_CELL_NTC_TEMP_OBJ_INDEX			0x02
#define	SMP_DETAIL_OVP_FLAG_OBJ_INDEX				0x03
#define	SMP_DETAIL_UVP_FLAG_OBJ_INDEX				0x04
#define	SMP_DETAIL_COTP_FLAG_OBJ_INDEX				0x05
#define	SMP_DETAIL_CUTP_FLAG_OBJ_INDEX				0x06
#define	SMP_DETAIL_DOTP_FLAG_OBJ_INDEX				0x07
#define	SMP_DETAIL_DUTP_FLAG_OBJ_INDEX				0x08

//SMP_CAN_FUN_FU_RX
#define	SMP_FU_INFO_OBJ_INDEX						0x00
#define	SMP_FU_DATA_START_OBJ_INDEX					0x01
#define	SMP_FU_DATA_END_OBJ_INDEX					(SMP_FU_DATA_START_OBJ_INDEX + 30)

#define	SMP_FU_INFO_STARTUP_SUB_INDEX					0x01
#define	SMP_FU_INFO_BASE_ADDR_SUB_INDEX					0x02
#define	SMP_FU_INFO_PROGRESS_SUB_INDEX					0x03
#define	SMP_FU_INFO_FW_CHECKING_SUB_INDEX				0x04
#define	SMP_FU_INFO_FW_CHECK_AND_UPDATE_SUB_INDEX		0x05
#define SMP_FU_INFO_FW_RESET_AND_UPDATE_SUB_INDEX		0x06
#define	SMP_FU_INFO_FW_APP_RESET						0x07
#define	SMP_FU_INFO_GOTO_BOOTLOADER_SUB_INDEX			0x08
/* Public macro -------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

	

#endif /* _SMP_CANBUS_PROTOCOL_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    






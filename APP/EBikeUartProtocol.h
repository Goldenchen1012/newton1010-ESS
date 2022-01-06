#ifndef _EBIKE_UART_PROTOCOL_H
#define	_EBIKE_UART_PROTOCOL_H

#define UART_PROTOCOL_VERSION_HIGH  0
#define UART_PROTOCOL_VERSION_LOW  8


#define	MESSAGE_PROTOCOL			'T'
#define	EBIKE_UART_PROTOCOL			'U'
#define	EBIKE_UART_BMS_PROTOCOL		'u'
#define BLE_PROTOCOL                'B'
#define MOTOR_CONTROLLER_PROTOCOL   'M'
#define CAN_PROTOCOL                'C'
#define EXTEND_PROTOCOL             'E'

//-------------------------------------
//BLE_PROTOCOL
#define	BLE_SUB_CMD_SET			    'S'
#define	BLE_SUB_CMD_GET				'G'
//-------------------------------------
#define	BLE_B0_MAC                  'M'
#define	BLE_B0_ADV_INFO             'A'
#define	BLE_B0_ADV_INTERVAL         'I'

//-------------------------------------
//MOTOR_CONTROLLER_PROTOCOL------------
//BLE_PROTOCOL
#define	MOTOR_SUB_CMD_SET			'S'
#define	MOTOR_SUB_CMD_GET		    'G'
#define	MOTOR_SUB_CMD_GET_ACK		'A'
#define	MOTOR_SUB_CMD_SET_ACK		'a'
#define	MOTOR_SUB_CMD_SET_POWER	    'P'
//SUB_CMD_RETURN_INFORMATION
#define	B0_INFO_MOTOR_BIKE_INFO	    'B'
#define	B0_INFO_MOTOR_OUTPUT	    'P'
#define	B0_INFO_MOTOR_REMAINING_RANGE	'R'
//SUB_CMD_WRITE_PARAMETER
#define B0_INFO_MOTOR_SET_ASSIST_LEVEL  'A'
#define B0_INFO_MOTOR_STATUS        'S'
#define B0_INFO_MOTOR_WHEEL_DIAMETER    'W'

//-------------------------------------
//EBIKE_UART_PROTOCOL------------------

#define	SUB_CMD_SIMU_MODE			'S'
#define	SUB_CMD_ENGMODE				'E'
#define	SUB_CMD_RETURN_INFORMATION	'I'
#define	SUB_CMD_RETURN_PARAMETER	'p'
#define	SUB_CMD_WRITE_PARAMETER		'P'
#define	SUB_CMD_READ_VERSION		'V'
#define	SUB_CMD_RTC					'R'
#define	SUB_CMD_RAM					'F'
#define	SUB_CMD_ASK_DATA			'A'
#define	SUB_CMD_BOOT_MODE			'U'
//------------------------------------
#define	B0_INFO_MAX_MIN				'M'
#define B0_INFO_CELL_VOLTAGE		'C'
#define	B0_INFO_CELL_ADC			'A'
#define B0_INFO_CURRENT				'I'
#define	B0_INFO_VPACK				'P'
#define	B0_INFO_VBAT				'B'
#define	B0_INFO_REAL_CURRENT		'r'
#define	B0_INFO_SOC					'S'
#define	B0_INFO_OTHER_MESSAGE		'i'
#define	B0_INFO_OVP_UVP				'c'
#define	B0_INFO_SYSTEMFLAG			'f'
#define	B0_INFO_COTP_CUTP_DOTP		'n'
#define	B0_INFO_NTC_TEMP			'N'
#define	B0_INFO_NTC_ADC				'a'
#define	B0_INFO_LAST_CHG_TIME		'l'

//------------------------------------
//SUB_CMD_RAM
#define	B0_CLEAR_FAULT_LOG			'C'
#define	B0_CLEAR_RMA_FLAG			'S'
#define	B0_READ_FAULT_LOG1_NUMBER	'N'
#define	B0_READ_FAULT_LOG2_NUMBER	'n'
#define	B0_READ_FAULT_LOG1_DATA		'R'
#define	B0_READ_FAULT_LOG2_DATA		'r'
//------------------------------------
//------------------------------------
#define	B0_WRITE_RTC				'W'
#define	B0_READ_RTC					'R'
//------------------------------------
#define	B0_RETURN_HW_VERSION		'H'
#define	B0_RETURN_FW_VERSION		'S'
#define	B0_RETURN_PAR1_CHECKSUM		'1'
#define	B0_RETURN_PAR2_CHECKSUM		'2'
#define	B0_RETURN_PAR3_CHECKSUM		'3'
#define	B0_RETURN_PAR4_CHECKSUM		'4'
#define	B0_RETURN_PROTOCOL_VERSION	'P'
#define	B0_RETURN_BOOT_VERSION		'B'
#define	B0_RETURN_FW_CHECKSUM		'C'
//------------------------------------
//	eng mode
#define	B0_ENTER_EMG_MODE			'E'
#define	B0_EXIT_ENG_MODE			'X'
#define	B0_MOS_CONTROL				'M'
#define	B0_PF_CONTROL				'P'
#define	B0_PRE_DISCHARGE_CONTROL	'p'
#define	B0_LED_CONTROL				'L'
#define	B0_BALANCE_CONTROL			'B'
#define	B0_READ_SPI_ID				'S'
#define	B0_WHILE_LOOP				'l'
#define	B0_CLEAR_FLAG				'C'
#define	B0_SPI_TEST					's'
#define	B0_POWER_DOWN				'd'
#define	B0_GET_SOC					'G'
#define	B0_CLEAR_BATINFO_AREA		'b'
#define	B1_SPI_ERASE				'E'
//------------------------------------
//	simulator mode
#define	B0_ENTER_SIMU_MODE			'E'
#define	B0_EXIT_SIMU_MODE			'X'
#define	B0_CELL_SIMU_DATA			'C'
#define	B0_NTC_SIMU_DATA			'N'
#define	B0_CURRENT_SIMU_DATE		'I'
#define	B0_VPACK_SIMU_DATE			'P'
#define	B0_VB_SIMU_DATA				'B'
//------------------------------------
//	write	parameter
#define	B0_WRITE_SYSTEMPAR			'S'
#define	B0_WRITE_OCV_TABLE			'O'
#define	B0_WRITE_NOTE_PAR			'N'
#define	B0_WRITE_QMAX_RM			'Q'
#define	B0_WRITE_CELL_CAL_PAR		'C'
#define	B0_WRITE_CURRENT_CAL_PAR	'I'
#define	B0_WRITE_VPACK_CAL_PAR		'P'
#define	B0_WRITE_VBAT_CAL_PAR		'B'
#define	B0_WRITE_CALIB_PAR_CURRENT	'A'
#define	B0_WRITE_CALIB_PAR_CELL	    'E'
#define	B0_WRITE_CALIB_PAR_VB	    'T'
#define	B0_WRITE_CALIB_PAR_VP	    'K'
#define	B0_WRITE_CUSTOMER_PARAMETER	    'D'
//------------------------------------
//------------------------------------

//------------------------------------
//	read parameter
#define	B0_RETURN_CELL_CAL_PAR		'C'
#define	B0_RETURN_CURRENT_CAL_PAR	'I'
#define	B0_RETURN_VB_CAL_PAR		'B'
#define	B0_RETURN_VP_CAL_PAR		'P'
#define	B0_RETURN_SYSTEM_PAR		'S'
#define	B0_RETURN_OCV_TABLE			'O'
#define	B0_RETURN_NOTE_PAR			'N'
#define	B0_RETUEN_CELL_ADC			'c'
#define	B0_RETURN_CURRENT_ADC		'i'
#define	B0_RETURN_VB_ADC			'b'
#define	B0_RETURN_VPACK_ADC			'p'
#define	B0_RETURN_CELL_NUMBER		'F'
#define	B0_RETURN_CALIB_PAR_CURRENT	'A'
#define	B0_RETURN_CALIB_PAR_CELL	'E'
#define	B0_RETURN_CALIB_PAR_VB	    'T'
#define	B0_RETURN_CALIB_PAR_VP	    'K'
#define	B0_RETURN_CURRENT_ADC_I32	'U'
#define	B0_RETURN_CUSTOMER_PARAMETER	'D'
#define	B0_RETURN_CUSTOMER_PARAMETER_DATA_LEN_MAX	'M'
//---------------------------------------
//	Ask Command
#define	B0_ASK_CELL_VOLTAGE			'C'		
#define	B0_ASK_ALL_CELL_VOLTAGE		'c'
#define	B0_ASK_NTC_TEMP				'N'
#define	B0_ASK_ALL_NTC_TEMP			'n'
#define	B0_ASK_CURRENT				'I'
#define	B0_ASK_VPACK				'P'
#define	B0_ASK_VBAT					'B'
#define	B0_ASK_SOC					'S'
#define	B0_ASK_STOP_MESSAGE			'M'
//---------------------------------------
#define	B0_BOOT_ENTER_BOOT_MODE		'B'
#define	B0_BOOT_START_RUN_APP		'u'
#define	B0_BOOT_CHECK_MCU_FW		'M'
#define	B0_BOOT_CHECK_SPI_FW		'S'
#define	B0_BOOT_COPY_SPI_TO_MCU		'C'
#define	B0_BOOT_READ_SPI_ID			'I'
#define	B0_BOOT_WRITE_MCU_DATA		'w'
#define	B0_BOOT_WRITE_SPI_DATA		'W'
#define	B0_BOOT_READ_SPI_DATA		'R'
#define	B0_BOOT_READ_MCU_DATA		'r'
#define	B0_BOOT_ERASE_SPI_SECTOR	'E'
#define	B0_BOOT_ERASE_MCU_PAGE		'e'

#endif

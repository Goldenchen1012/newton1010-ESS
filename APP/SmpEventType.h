/**
  ******************************************************************************
  * @file        SmpEventType.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/12/01
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */
  
#ifndef _SMP_EVENT_TYPE_H_
#define _SMP_EVENT_TYPE_H_

enum{
	EVENT_TYPE_OVP_L1_SET = 1,
	EVENT_TYPE_OVP_L2_SET,
	EVENT_TYPE_OVP_L3_SET,
	EVENT_TYPE_OVP_L1_RELEASE,
	EVENT_TYPE_OVP_L2_RELEASE,
	EVENT_TYPE_OVP_L3_RELEASE,
	EVENT_TYPE_OVP_HW_SET,
	EVENT_TYPE_OVP_PF,

	EVENT_TYPE_UVP_L1_SET	= 10,
	EVENT_TYPE_UVP_L2_SET,
	EVENT_TYPE_UVP_L3_SET,
	EVENT_TYPE_UVP_L1_RELEASE,
	EVENT_TYPE_UVP_L2_RELEASE,
	EVENT_TYPE_UVP_L3_RELEASE,
	EVENT_TYPE_UVP_HW_SET,
	EVENT_TYPE_UVP_PF,

	EVENT_TYPE_COTP_L1_SET	= 20,
	EVENT_TYPE_COTP_L2_SET,
	EVENT_TYPE_COTP_L3_SET,
	EVENT_TYPE_COTP_L4_SET,
	EVENT_TYPE_COTP_L1_RELEASE,
	EVENT_TYPE_COTP_L2_RELEASE,
	EVENT_TYPE_COTP_L3_RELEASE,
	EVENT_TYPE_COTP_L4_RELEASE,		
	
	EVENT_TYPE_CUTP_L1_SET	= 30,
	EVENT_TYPE_CUTP_L2_SET,
	EVENT_TYPE_CUTP_L3_SET,
	EVENT_TYPE_CUTP_L4_SET,
	EVENT_TYPE_CUTP_L1_RELEASE,		
	EVENT_TYPE_CUTP_L2_RELEASE,
	EVENT_TYPE_CUTP_L3_RELEASE,
	EVENT_TYPE_CUTP_L4_RELEASE,
	
	EVENT_TYPE_OT_HW_SET	= 40,
	EVENT_TYPE_UT_HW_SET,
	EVENT_TYPE_DOTP_L1_SET,
	EVENT_TYPE_DOTP_L2_SET,
	EVENT_TYPE_DOTP_L3_SET,
	EVENT_TYPE_DOTP_L4_SET,
	EVENT_TYPE_DOTP_L1_RELEASE,
	EVENT_TYPE_DOTP_L2_RELEASE,
	EVENT_TYPE_DOTP_L3_RELEASE,
	EVENT_TYPE_DOTP_L4_RELEASE,		
	
	EVENT_TYPE_DUTP_L1_SET	= 50,
	EVENT_TYPE_DUTP_L2_SET,
	EVENT_TYPE_DUTP_L3_SET,
	EVENT_TYPE_DUTP_L4_SET,
	EVENT_TYPE_DUTP_L1_RELEASE,		
	EVENT_TYPE_DUTP_L2_RELEASE,
	EVENT_TYPE_DUTP_L3_RELEASE,
	EVENT_TYPE_DUTP_L4_RELEASE,

	EVENT_TYPE_DOCP_L1_SET	= 60,		
	EVENT_TYPE_DOCP_L2_SET,
	EVENT_TYPE_DOCP_L3_SET,
	EVENT_TYPE_DOCP_L4_SET,
	EVENT_TYPE_DOCP_L1_RELEASE,	
	EVENT_TYPE_DOCP_L2_RELEASE,	
	EVENT_TYPE_DOCP_L3_RELEASE,
	EVENT_TYPE_DOCP_L4_RELEASE,
	
	EVENT_TYPE_COCP_L1_SET	= 70,
	EVENT_TYPE_COCP_L2_SET,	
	EVENT_TYPE_COCP_L3_SET,	
	EVENT_TYPE_COCP_L4_SET,
	EVENT_TYPE_COCP_L1_RELEASE,
	EVENT_TYPE_COCP_L2_RELEASE,
	EVENT_TYPE_COCP_L3_RELEASE,	
	EVENT_TYPE_COCP_L4_RELEASE,

	EVENT_TYPE_DTP_L1_SET	= 80,
	EVENT_TYPE_DTP_L2_SET,
	EVENT_TYPE_DTP_L3_SET,
	EVENT_TYPE_DTP_L1_RELEASE,
	EVENT_TYPE_DTP_L2_RELEASE,
	EVENT_TYPE_DTP_L3_RELEASE,

	EVENT_TYPE_DVP_L1_SET	= 90,
	EVENT_TYPE_DVP_L2_SET,
	EVENT_TYPE_DVP_L3_SET,
	EVENT_TYPE_DVP_L1_RELEASE,
	EVENT_TYPE_DVP_L2_RELEASE,
	EVENT_TYPE_DVP_L3_RELEASE,

	EVENT_TYPE_CHG_BALANCE_SET	= 100,		
	EVENT_TYPE_DHG_BALANCE_SET,		
	EVENT_TYPE_RLX_BALANCE_SET,		
	EVENT_TYPE_CHG_BALANCE_RELEASE,
	EVENT_TYPE_DHG_BALANCE_RELEASE,
	EVENT_TYPE_RLX_BALANCE_RELEASE,

	EVENT_TYPE_AFE_COMM_L1_SET	= 110,
	EVENT_TYPE_AFE_COMM_L2_SET,
	EVENT_TYPE_AFE_COMM_L1_RELEASE,
	EVENT_TYPE_AFE_COMM_L2_RELEASE,
	EVENT_TYPE_AFE_RE_INI,
	
	EVENT_TYPE_SOC_L1_SET	= 120,
	EVENT_TYPE_SOC_L1_RELEASE,
	EVENT_TYPE_SOC_L2_SET,
	EVENT_TYPE_SOC_L2_RELEASE,
	
	EVENT_TYPE_CAL_RA1 = 130,
	EVENT_TYPE_CAL_RA2,
	EVENT_TYPE_CAL_RA3,
	EVENT_TYPE_CAL_RA4,
	EVENT_TYPE_CAL_RA5,
	EVENT_TYPE_CAL_RA6,
	EVENT_TYPE_CAL_RA7,
	EVENT_TYPE_CAL_RA8,
	EVENT_TYPE_CAL_RA9,
	EVENT_TYPE_CAL_RA10,
	EVENT_TYPE_CAL_RA11,
	EVENT_TYPE_CAL_RA12,
	EVENT_TYPE_CAL_RA13,
	EVENT_TYPE_CAL_RA14,
	EVENT_TYPE_CAL_RA15,
	EVENT_TYPE_CAL_RA16,
	EVENT_TYPE_CAL_RA17,
	EVENT_TYPE_CAL_RA18,
	EVENT_TYPE_CAL_RA19,
	EVENT_TYPE_CAL_RA20,
	EVENT_TYPE_CAL_RA21,
	EVENT_TYPE_CAL_RA22,
	EVENT_TYPE_CAL_RA23,
	EVENT_TYPE_CAL_RA24,
	EVENT_TYPE_CAL_RA25,
	
	//----------------------------
	//	Relay 相關事件
	EVENT_TYPE_RELAY_ON	= 160,
	EVENT_TYPE_RELAY_OFF,
	EVENT_TYPE_RELAY_PF,
	EVENT_TYPE_RELAY_ON_FAIL,
	EVENT_TYPE_RELAY_OFF_FAIL,
	EVENT_TYPE_PRE_CHG_ON,
	EVENT_TYPE_PRE_CHG_OFF,
	//---------------------------
	//	SCU　溫度
	EVENT_TYPE_RLY1_OT_L1_SET,
	EVENT_TYPE_RLY1_OT_L2_SET,
	EVENT_TYPE_RLY1_OT_L3_SET,
	EVENT_TYPE_RLY1_OT_L1_RELEASE,
	EVENT_TYPE_RLY1_OT_L2_RELEASE,
	EVENT_TYPE_RLY1_OT_L3_RELEASE,

	EVENT_TYPE_RLY2_OT_L1_SET,
	EVENT_TYPE_RLY2_OT_L2_SET,
	EVENT_TYPE_RLY2_OT_L3_SET,
	EVENT_TYPE_RLY2_OT_L1_RELEASE,
	EVENT_TYPE_RLY2_OT_L2_RELEASE,
	EVENT_TYPE_RLY2_OT_L3_RELEASE,

	EVENT_TYPE_AMBI_OT_L1_SET,
	EVENT_TYPE_AMBI_OT_L2_SET,
	EVENT_TYPE_AMBI_OT_L3_SET,
	EVENT_TYPE_AMBI_OT_L1_RELEASE,
	EVENT_TYPE_AMBI_OT_L2_RELEASE,
	EVENT_TYPE_AMBI_OT_L3_RELEASE,
	
	EVENT_TYPE_BUSBAR_P_OT_L1_SET,
	EVENT_TYPE_BUSBAR_P_OT_L2_SET,
	EVENT_TYPE_BUSBAR_P_OT_L3_SET,
	EVENT_TYPE_BUSBAR_P_OT_L1_RELEASE,
	EVENT_TYPE_BUSBAR_P_OT_L2_RELEASE,
	EVENT_TYPE_BUSBAR_P_OT_L3_RELEASE,

	EVENT_TYPE_BUSBAR_N_OT_L1_SET,
	EVENT_TYPE_BUSBAR_N_OT_L2_SET,
	EVENT_TYPE_BUSBAR_N_OT_L3_SET,
	EVENT_TYPE_BUSBAR_N_OT_L1_RELEASE,
	EVENT_TYPE_BUSBAR_N_OT_L2_RELEASE,
	EVENT_TYPE_BUSBAR_N_OT_L3_RELEASE,	
	//---------------------------
	EVENT_TYPE_POWEROFF_OVER_5HR,
	EVENT_TYPE_IDLE_OVER_5HR,
	EVENT_TYPE_UPDATE_QMAX_1ST,
	EVENT_TYPE_UPDATE_QMAX,
	EVENT_TYPE_GET_5HR_SOC,
	EVENT_TYPE_CANNOT_GET_5HR_SOC,
	EVENT_TYPE_START_CHG_SOC,
	EVENT_TYPE_START_DHG_SOC,

	EVENT_TYPE_POWER_ON,
	EVENT_TYPE_POWER_OFF,
	EVENT_TYPE_RELAY_FAIL_VB_DIFF,
	//---------------------------------
	EVENT_TYPE_SHORT_CIRCUIT_DETECT,
	
	EVENT_TYPE_EPO_ENABLE,
	EVENT_TYPE_EPO_DISABLE,
	
	EVENT_TYPE_NFAULT_ENABLE,
	EVENT_TYPE_NFAULT_DISABLE,
	



	EVENT_TYPE_END
};


#endif /* _SMP_EVENT_TYPE_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    

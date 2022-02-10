/**
  ******************************************************************************
  * @file        ApiIRMonitoring.h
  * @author      Golden Chen
  * @version     v0.0.4
  * @date        2021/12/21
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 </center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _API_IRMONITORING_H_
#define _API_IRMONITORING_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/* Public typedef -----------------------------------------------------------*/
typedef struct{ 
	float Vo_stack;
	float V_stack;
	float Vo_n;
	float Vo_en;
	float Vn;
	float Vp;
	
	float Vo_n_l;
	float Vn_l;
	float Vp_l;
	
	uint8_t  exe_interval_s;
	uint16_t sw_delay_ms;
}IRMonitoring_Data_t;

typedef struct{ 
	uint16_t Rp_kohm;
	uint16_t Rn_kohm;
	float V_stack;   
}IRMonitoring_Resistor_t;		

typedef enum{
	 IRM_EVENT_BALANCE     =0 ,
	 IRM_EVENT_UNBALANCE      ,
   IRM_EVENT_GET_VSTACK     ,	
	 IRM_EVENT_OTHER_ERR      ,
}IRMonitoring_event_cb_type;


typedef enum{
	 IRM_BUSY              =0 ,
	 IRM_OK                   ,
	 IRM_OTHER_ERR            ,
}IRMonitoring_event_read_cb_type;

typedef enum{
	 IRM_STEP_OK                  =0       ,
	 IRM_STEP_READ_TIMEOUT                 ,	
	 IRM_STEP_DATA_READY_TIMEOUT           ,	 
	 IRM_STEP_OUT_VSTACK_VAL               ,
	 IRM_STEP_OTHER_ERR                    ,
}IRMonitoring_step_ret_type;

typedef enum{
    IRM_S1             = 0          ,
	  IRM_S2                          ,
	  IRM_S3                          ,
	  IRM_S4                          ,	
	  IRM_S5                          ,
	  IRM_S6                          ,
		IRM_IO_WAITTING                 ,
	  IRM_DEVICE_WAITTING             ,
	  IRM_DATAREADY_WAITTING          ,
	  IRM_FINISH                      ,
}IRMonitoring_steps_enum;

typedef enum{
    IRM_SW_OFF         = 0          ,
	  IRM_SW_ON                       ,
}IRMonitoring_SW_enum;

typedef uint8_t(*apiIRMonitoring_CB_Fun_t)(IRMonitoring_Resistor_t *irm_res_data, IRMonitoring_event_cb_type irm_event);    //IRM Callback function

typedef void(*IRM_Recv_CB_t)(float *pDatBuf);

typedef struct{
	 void  (*SW_gpioinit_cb)(void);
   void  (*SW_gpio_crtl_cb[3])(IRMonitoring_SW_enum on_off);
	 void  (*GetVoltDeviceInit_cb)(void);
	 IRMonitoring_event_read_cb_type (*TriggerData_cb)(void);                                 //Unit:Voltage ,type:float
   apiIRMonitoring_CB_Fun_t irm_outdata;
	 void (*DataReady_cb)(IRM_Recv_CB_t rcv_cb); 
}apiIRMonitoring_cb_t;	

/* Public define ------------------------------------------------------------*/

/*
  ------------------------------------------o Stack+
  |            |                              +
  |           SW1
  |            |                           Rp  Vp
-----          |
 ---          IRM_RA
-----          |
 ---           |
-----          |                               -
 ---           -----IRM_RD-----SW2----------O  +
-----          |  |                  |      |
 ---           |  |------------SW3---|      | 
  |            |                           //// 
 ---          IRM_RB                   
-----          |
 ---           |______ +                              
-----          |                            Rn  Vn
 ---          IRM_RC 
  |            |        Vo
  |            |
  |            |       -                          -
  -------------------------------------------O  Stack-
*/

#define IRM_RA							                   800000.0f                                            //Unit:Ohm
#define IRM_RB							                   800000.0f                                            //Unit:Ohm
#define IRM_RC							                   2200.0f	                                            //Unit:Ohm
#define IRM_RD							                   500000.0f                                            //Unit:Ohm
#define IRM_RSW							                   0.0f		                                              //Unit:Ohm

#define IRM_K1                                 ((IRM_RA + IRM_RB + IRM_RC + IRM_RSW)/IRM_RC)        //Unit: NA
#define IRM_K2                                 ((IRM_RB + IRM_RC + IRM_RSW)/IRM_RC)                 //Unit: NA
#define IRM_K3                                 ((IRM_RB + IRM_RC + IRM_RD + IRM_RSW)/IRM_RC)        //Unit: NA
#define IRM_KTHD                               2.0f                                                 //Unit: %

#define IRM_READ_TIMEOUT                       10                                                   //Uint: ms
#define IRM_DATA_READY_TIMEOUT                 5                                                    //Uint: ms

#define ITRM_TESTPARAM_DET_CYCLE_TIME          2                                                    //Uint: second               
#define ITRM_TESTPARAM_RELAY_WAIT_TIME         100                                                  //Uint: ms      
/* Public macro -------------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
IRMonitoring_Resistor_t apiIRMonitoringGetResistor(void);

uint8_t apiIRMonitoringOpen(uint8_t exe_interval_s, uint16_t sw_delay_ms, apiIRMonitoring_cb_t callbackfunc);
void apiIRMonitoringGetVstack(void);

#ifdef __cplusplus
}
#endif

#endif /* _API_IRMONITORING_H_ */

/************************ (C) COPYRIGHT *****END OF FILE****/    

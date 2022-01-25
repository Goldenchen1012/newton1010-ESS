/**
  ******************************************************************************
  * @file        ApiIRMonitoring.c
  * @author      Golden Chen
  * @version     v0.0.4
  * @date        2021/12/21
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021</center></h2>
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "define.h"
#include "main.h"
#include "ApiIRMonitoring.h"
#include "LibSwTimer.h"
#include "LibHwTimer.h"
#include "RTT_Log.h"

/* Private define ------------------------------------------------------------*/

//Test mode, parameter assign fixed value to test IRM formula, you can select one to testing.
//-------------------------------------------------------------------------------------------------
//Test  Balance output results: 
//                                   Rp_kohm = 29999
//                                   Rn_kohm = 29686
//      UnBalance output results:
//                                   Rp_kohm = 20000
//                                   Rn_kohm = 9000

//#define IRM_TEST_S5                //Balance
//#define IRM_TEST_S6                //UnBalance
//#define IRM_TEST_OUT_VSTACK        //Simulation to output Vstack = 1000V
//-------------------------------------------------------------------------------------------------

/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static IRMonitoring_Data_t irm_data;
static IRMonitoring_Resistor_t irm_res_out;
static IRMonitoring_steps_enum irm_mes_step = IRM_S1;
static uint8_t irm_sub_step = 0;

static float temp_rp, temp_rn;
static float ra,rb,rc,rd;
static float temp_a,temp_b, temp_c, temp_d;
static float Ra_l, Rb_l;

static apiIRMonitoring_cb_t irm_event_cb;

static void IRMonitoring_CtrlSW(IRMonitoring_SW_enum sw1, IRMonitoring_SW_enum sw2, IRMonitoring_SW_enum sw3);
static void IRMonitoring_StateReset(void);

static uint16_t	irm_count = 0;
static uint8_t irm_flag = 1;
static uint8_t irm_vstack_f = 0;

static float irm_adc_data;
static uint8_t irm_data_ready_f = 0; 

/* Private function prototypes -----------------------------------------------*/
static void getIRMonitoringVoValue_cb(float *read_volt_data)
{
	  irm_adc_data = *read_volt_data;  //Get Voltage data
	  irm_data_ready_f = 1;	           //Setting data ready falg
}
static uint8_t IRMonitoring_Init(uint8_t exe_interval_s, uint16_t sw_delay_ms, apiIRMonitoring_cb_t callbackfunc)
{
		//IRM Paparmeter
		irm_data.exe_interval_s  = exe_interval_s;
		irm_data.sw_delay_ms     = sw_delay_ms;
	 
    if(callbackfunc.DataReady_cb==NULL){
		  return(0);
		}	
	
    if(callbackfunc.TriggerData_cb==NULL){
		   return(0);
		}else{
		   irm_event_cb.TriggerData_cb = callbackfunc.TriggerData_cb;
		}			
		
	  if(callbackfunc.GetVoltDeviceInit_cb==NULL){
	      return(0);
	  }else{
		    irm_event_cb.GetVoltDeviceInit_cb = callbackfunc.GetVoltDeviceInit_cb;
	      irm_event_cb.GetVoltDeviceInit_cb ();
		}	
			
    if(callbackfunc.irm_outdata==NULL){
        return(0);
		}else{   
		    irm_event_cb.irm_outdata = callbackfunc.irm_outdata;
		}
    
    if(callbackfunc.SW_gpioinit_cb==NULL){
        return(0);
		}else{   
		    irm_event_cb.SW_gpioinit_cb  = callbackfunc.SW_gpioinit_cb;
		}

    for(int i=0; i<3 ; i++){		
        if(callbackfunc.SW_gpio_crtl_cb[i]==NULL){
            return(0);
		    }else{   
		        irm_event_cb.SW_gpio_crtl_cb[i] = callbackfunc.SW_gpio_crtl_cb[i];
		    }
	  }
	
    //GPIO Init Callback function
	  if(irm_event_cb.SW_gpioinit_cb!=NULL){
		    irm_event_cb.SW_gpioinit_cb();
		}
		
		// GPIO SW Init------------------------------------
		IRMonitoring_CtrlSW(IRM_SW_OFF, IRM_SW_OFF ,IRM_SW_OFF);
		
		callbackfunc.DataReady_cb(getIRMonitoringVoValue_cb);
		
    return(1);	
}

static void IRMonitoring_CtrlSW(IRMonitoring_SW_enum sw1, IRMonitoring_SW_enum sw2, IRMonitoring_SW_enum sw3){
    
	  if(irm_event_cb.SW_gpio_crtl_cb[0]!=NULL){
	      if(sw1 == IRM_SW_ON){
	          irm_event_cb.SW_gpio_crtl_cb[0](IRM_SW_ON);
	      }else{
		        irm_event_cb.SW_gpio_crtl_cb[0](IRM_SW_OFF);
		    }
	  }

	  if(irm_event_cb.SW_gpio_crtl_cb[1]!=NULL){
	      if(sw2 == IRM_SW_ON){
	          irm_event_cb.SW_gpio_crtl_cb[1](IRM_SW_ON);
	      }else{
		        irm_event_cb.SW_gpio_crtl_cb[1](IRM_SW_OFF);
		    }
	  }

	  if(irm_event_cb.SW_gpio_crtl_cb[2]!=NULL){
	      if(sw3 == IRM_SW_ON){
	          irm_event_cb.SW_gpio_crtl_cb[2](IRM_SW_ON);
	      }else{
		        irm_event_cb.SW_gpio_crtl_cb[2](IRM_SW_OFF);
		    }
	  }		

}

static uint16_t  IRM_Balance_formula(IRMonitoring_Data_t data, IRMonitoring_Resistor_t *res_out){
	
	ra = ((float)IRM_RA)*1.0f;
	rb = ((float)IRM_RB)*1.0f;
	rc = ((float)IRM_RC)*1.0f;
	
	temp_rp = (((data.Vn / data.Vn_l) * (data.Vp_l / data.Vp))-1.0f)*ra;
	temp_rn = 1.0f/(((data.Vp_l / data.Vn_l)*(1.0f/temp_rp))-(1.0f/(rb + rc)));
	
	res_out->Rp_kohm = (uint16_t)(temp_rp / 1000.0f);
	res_out->Rn_kohm = (uint16_t)(temp_rn / 1000.0f);
	
	return 0;
}

static uint16_t  IRM_Unbalance_formula(IRMonitoring_Data_t data, IRMonitoring_Resistor_t *res_out){
	
	ra = ((float)IRM_RA)*1.0f;
	rb = ((float)IRM_RB)*1.0f;
	rc = ((float)IRM_RC)*1.0f;
	rd = ((float)IRM_RD)*1.0f;

	temp_a = ((data.Vp_l / data.Vp) * (data.Vn / data.Vn_l))-1.0f;
	
	temp_c = 1.0f/(rb+ rc + rd);
  Ra_l = (ra * rd + (rd *( rb+ rc)) + (( rb+ rc) * ra)) / (rb+ rc);  
	Rb_l = (ra * rd + (rd *( rb + rc)) + ((rb+ rc) * ra)) / (ra);
	temp_b = temp_c + (data.Vp_l/ data.Vn_l) * (1.0f/Ra_l - (data.Vn/data.Vp * (1.0f/Rb_l)));
	
	temp_rn = temp_a/temp_b;	
	
	temp_d = 1.0f/(rb + rc + rd);
  temp_rp = 1.0f/((data.Vn_l/data.Vp_l)*(temp_d + (1.0f/temp_rn)));
	
	res_out->Rp_kohm = (uint16_t)(temp_rp / 1000.0f);
	res_out->Rn_kohm = (uint16_t)(temp_rn / 1000.0f);
	
	return 0;
}

static IRMonitoring_step_ret_type IRMonitoringMeasure_Steps(IRMonitoring_steps_enum steps)
{
	static uint16_t sub_step_count = 0;
	static IRMonitoring_event_read_cb_type read_fun_res;
	static IRMonitoring_steps_enum n_next_step;
	
    switch(steps){
			case IRM_S1:
		      IRMonitoring_CtrlSW(IRM_SW_ON, IRM_SW_OFF , IRM_SW_OFF);
			    irm_sub_step = 0;
			    irm_mes_step = IRM_IO_WAITTING;
			    n_next_step =IRM_S2;
		      break;
			case IRM_S2:
				  if(irm_data_ready_f == 1){
						  irm_data_ready_f = 0;
						
				      irm_data.Vo_stack = irm_adc_data;
			        irm_data.V_stack  = (irm_data.Vo_stack * IRM_K1);
			        irm_res_out.V_stack = irm_data.V_stack;
						
              //If callback function exist then execution this.
	            if((irm_event_cb.irm_outdata != NULL) && (irm_vstack_f==1)){	
								
								  irm_vstack_f = 0;
					      
  								//Test 
							    #ifdef IRM_TEST_OUT_VSTACK
							    irm_data.V_stack = 1000.0f;
							    #endif   
								
							    irm_res_out.V_stack = irm_data.V_stack;
	                irm_event_cb.irm_outdata(&irm_res_out, IRM_EVENT_GET_VSTACK);
							    IRMonitoring_StateReset();
						      return(IRM_STEP_OUT_VSTACK_VAL);
              }   			
				   }

		      IRMonitoring_CtrlSW(IRM_SW_ON, IRM_SW_OFF , IRM_SW_ON);
			    irm_sub_step = 0;
			    irm_mes_step = IRM_IO_WAITTING;
			    n_next_step =IRM_S3;
		      break;
			case IRM_S3:
				  if(irm_data_ready_f == 1){
						 irm_data_ready_f = 0;
						
						 irm_data.Vo_n = irm_adc_data;
						 irm_data.Vn   = (irm_data.Vo_n * IRM_K2);  					
					}
			
          irm_data.Vp = (irm_data.V_stack - irm_data.Vn);  
			    irm_sub_step = 0;
			    irm_mes_step = IRM_S4;    											
		      break;
			case IRM_S4:	

          LOG_GREEN("Vp=%d , Vn=%d , Vstack=%d\r\n", (int)(irm_data.Vp),(int)(irm_data.Vn),(int)(irm_data.V_stack));
			
			    if((((fabs(irm_data.Vp - irm_data.Vn))*100)/irm_data.V_stack) < IRM_KTHD){
					    irm_sub_step = 1;
					    irm_mes_step = IRM_S5;
					}else{
					    irm_sub_step = 1;
					    irm_mes_step = IRM_S6;
					}
					
					#ifdef IRM_TEST_S5 
					irm_mes_step = IRM_S5;
					#endif 
					
					#ifdef IRM_TEST_S6 
					irm_mes_step = IRM_S6;
					#endif 
					
					LOG_GREEN("STEP=%d\r\n",irm_mes_step);
							
		      break;
			case IRM_S5:      //Balance resistor detection 
		      switch(irm_sub_step){
						case 1:
							  IRMonitoring_CtrlSW(IRM_SW_OFF, IRM_SW_OFF, IRM_SW_ON);
			          irm_mes_step = IRM_IO_WAITTING;
			          n_next_step =IRM_S5;
                irm_sub_step++;						
						    break;
						case 2:
				        if(irm_data_ready_f == 1){
						        irm_data_ready_f = 0;
						        irm_data.Vo_n_l = irm_adc_data;
						        irm_data.Vn_l   = (irm_data.Vo_n_l * IRM_K2);  								
								}	
						
							  irm_data.Vp_l = (irm_data.V_stack - irm_data.Vn_l);  
			          irm_sub_step++;    											
						    break;
						case 3:
						    IRMonitoring_CtrlSW(IRM_SW_OFF, IRM_SW_OFF ,IRM_SW_OFF);	
										
			          //Test balance formula IRM use
					      //----------------
					      #ifdef IRM_TEST_S5
	              irm_data.V_stack = 1000;
	              irm_data.Vn   = 500.60f;
                irm_data.Vn_l = 25.37606553f;
					      irm_data.Vp   = 499.40f; 
	              irm_data.Vp_l = 974.62f;
					      #endif
					      //----------------					
						
							  // balance formula
						    IRM_Balance_formula(irm_data, &irm_res_out);
						
						    //If callback function exist then execution this.
	              if(irm_event_cb.irm_outdata != NULL){
	                  irm_event_cb.irm_outdata(&irm_res_out, IRM_EVENT_BALANCE);
                } 						
						
						    irm_mes_step = IRM_FINISH;
						    break;						
					}         
		      break;
			case IRM_S6:      //Unbalance resistor detection 
		      switch(irm_sub_step){
						case 1:
							  IRMonitoring_CtrlSW(IRM_SW_ON, IRM_SW_ON , IRM_SW_OFF);
			          irm_mes_step = IRM_IO_WAITTING;
			          n_next_step =IRM_S6;
                irm_sub_step++;							     
				        break;
						case 2:
				        if(irm_data_ready_f == 1){
						        irm_data_ready_f = 0;
										irm_data.Vo_en = irm_adc_data;
						        irm_data.Vn   =(irm_data.Vo_en*((IRM_K2*(1+IRM_RD/IRM_RA))+(IRM_RD/IRM_RC)))-((IRM_RD/IRM_RA)*irm_data.V_stack);
								}							
						
						    irm_data.Vp = (irm_data.V_stack - irm_data.Vn);  	
						    irm_sub_step++;			
				        break;
						case 3:
							  IRMonitoring_CtrlSW(IRM_SW_OFF, IRM_SW_ON , IRM_SW_OFF);
			          irm_mes_step = IRM_IO_WAITTING;
			          n_next_step =IRM_S6;
                irm_sub_step++;
				        break;
						case 4:
								if(irm_data_ready_f == 1){
						        irm_data_ready_f = 0;	
						        irm_data.Vo_n_l = irm_adc_data;
						        irm_data.Vn_l   = (irm_data.Vo_n_l * IRM_K3);   
								}

							  irm_data.Vp_l = (irm_data.V_stack - irm_data.Vn_l);  
						    irm_sub_step++;
				        break;
						case 5:
						    IRMonitoring_CtrlSW(IRM_SW_OFF, IRM_SW_OFF ,IRM_SW_OFF);		

			          //Test Unbalance formula IRM use
					      //----------------
					      #ifdef IRM_TEST_S6
	              irm_data.V_stack = 1000.0f;
	              irm_data.Vn   = 476.5691798f;
                irm_data.Vn_l = 53.81882327f;
					      irm_data.Vp   = 523.43f; 
	              irm_data.Vp_l = 946.18f;
					      #endif
					      //----------------							
						
						    //Unbalance formula
						    IRM_Unbalance_formula(irm_data, &irm_res_out); 
		
						    //If callback function exist then execution this.
	              if(irm_event_cb.irm_outdata != NULL){
	                  irm_event_cb.irm_outdata(&irm_res_out, IRM_EVENT_UNBALANCE);
                } 						
								
    						irm_mes_step = IRM_FINISH;
				        break;						
					}							
		      break;	
		  case IRM_IO_WAITTING:
				  ++sub_step_count;
			    if(sub_step_count >= irm_data.sw_delay_ms){
						  sub_step_count = 0;
					    irm_mes_step = IRM_DEVICE_WAITTING;   
					}
					
					//Test 2022.01.10
					GPIOD->ODR ^= GPIO_PIN_14;
			    break;
			case IRM_DEVICE_WAITTING:
          ++sub_step_count;
					if(sub_step_count>IRM_READ_TIMEOUT){
					    sub_step_count = 0;
              return(IRM_STEP_READ_TIMEOUT);
					}			
					
				  if(irm_event_cb.TriggerData_cb !=NULL){
	           read_fun_res = irm_event_cb.TriggerData_cb();
					   
					   if(read_fun_res == IRM_OK){ 
							   sub_step_count = 0;
						     irm_mes_step = IRM_DATAREADY_WAITTING;   
						 }
		      }
			    break;
			case IRM_DATAREADY_WAITTING:
          ++sub_step_count;
					if(sub_step_count>IRM_DATA_READY_TIMEOUT){
					    sub_step_count = 0;
              return(IRM_STEP_DATA_READY_TIMEOUT);
					}			
					
				  if(irm_data_ready_f == 1){
						  sub_step_count = 0;
						  irm_mes_step = n_next_step;   
		      }			
			    break;
			case IRM_FINISH:		
          break;						
		}
	  //-------------------------------------------------
		
		return(IRM_STEP_OK);
}
static void IRMonitoring_StateReset(void){
   irm_sub_step = 0;
   irm_mes_step = IRM_S1;  
   irm_count = 0;
   irm_flag = 0;   	
	 IRMonitoring_CtrlSW(IRM_SW_OFF, IRM_SW_OFF ,IRM_SW_OFF); 
}

static void IRMonitoringSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	static IRMonitoring_step_ret_type res;
	
	#if 1
	GPIOD->ODR ^= GPIO_PIN_13;
	#endif 
	
	if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
      if(irm_flag ==1){
          res =IRMonitoringMeasure_Steps(irm_mes_step);
      
				  if((res == IRM_STEP_READ_TIMEOUT) || (res == IRM_STEP_DATA_READY_TIMEOUT)){
					    IRMonitoring_CtrlSW(IRM_SW_OFF, IRM_SW_OFF, IRM_SW_OFF);
						  irm_count = 0;
			        irm_flag =0;            //end IR measure
			        irm_mes_step=IRM_S1;    //reset step to start S1
					}
				
 				  if(irm_mes_step == IRM_FINISH){
						  IRMonitoring_CtrlSW(IRM_SW_OFF, IRM_SW_OFF, IRM_SW_OFF);
					    irm_flag = 0;
					}
      }
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_10MS_5)
	{	
		irm_count++;
		if(irm_count >= irm_data.exe_interval_s*100)
		{
		    irm_count = 0;
			  irm_flag =1;            //start IR measure
			  irm_mes_step=IRM_S1;    //reset step to start S1
		}
	}
}

/* Public function prototypes -----------------------------------------------*/
IRMonitoring_Resistor_t apiIRMonitoringGetResistor(void)
{
    return irm_res_out;
}

uint8_t apiIRMonitoringOpen(uint8_t exe_interval_s, uint16_t sw_delay_ms, apiIRMonitoring_cb_t callbackfunc)
{
  uint8_t res;
		
	res = IRMonitoring_Init( exe_interval_s, sw_delay_ms, callbackfunc);
	
	LibSwTimerOpen(IRMonitoringSwTimerHandler, 0);

	return(res);
}

void apiIRMonitoringGetVstack(void){
    irm_vstack_f = 1;
	  irm_flag = 1;
	  irm_mes_step = IRM_S1; 
}

/************************ (C) COPYRIGHT ***END OF FILE****/

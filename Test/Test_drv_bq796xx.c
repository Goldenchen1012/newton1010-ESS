/**
  ******************************************************************************
  * @file    Test_drv_bq796xx.c
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
#include "smp_drv_bq796xx.h"
#include "Test_drv_bq796xx.h"
#include "smp_debug.h"
#include "smp_fifo.h"
#include "stm32l4xx_Davinci.h"
#include "main.h"
#include "smp_uart.h"
#include "LibHwTimer.h"
#include "RTT_Log.h"

extern uint8_t app_afe_cb(bq796xx_data_t *bq_data, bq796xx_event_cb_type bq_event);

#define TEST_AFE_PARAM_LOAD_VALUE {                                                        \
													          .ov_threshold         =OV_4175MV,                      \
	                                  .ov_step_threshold    =OV_STEP_100MV,                  \
	                                  .uv_threshold         =UV_3000MV,                      \
	                                  .ot_threshold         =OT_10_PCT,                      \
	                                  .ut_threshold         =UT_80_PCT,                      \
	                                  .ov_enable            =BQ_ENABLE,                      \
	                                  .uv_enable            =BQ_ENABLE,                      \
		                                .ot_enable            =BQ_DISABLE,                     \
	                                  .ut_enable            =BQ_DISABLE,                     \
		                                .cb_cycle_time        =CB_CYC_T_10S,                   \
	                                  .bmu_total_num        =BMU_TOTAL_BOARDS,               \
                                 }	

#if 1
bq796xx_init_default_t unit_testonly_afe_load_data = TEST_AFE_PARAM_LOAD_VALUE;																
#endif
																
extern bq796xx_init_default_t bq796xx_default;
extern bq796xx_data_t bq796xx_data;
																												
uint8_t num=0;
uint8_t d_payload[4] = {0};
uint8_t null_payload[4] = {0};
uint8_t cb_en1,cb_en2;
uint16_t test_cont = 0;

uint8_t bmu_dir=0;
uint16_t bmu_dir_cnt = 0;

extern uint8_t step_com_f;
extern uint8_t before_d_ms;
extern uint16_t bq_count_temp;
extern uint8_t  cnt_delay;
extern bq796xx_init_steps_enum afe_steps;
uint8_t res;

bq796xx_dir_set_steps_enum dir_afe_steps;

uint8_t dir_step_com_f;
uint8_t dir_before_d_ms;
uint16_t dir_bq_count_temp;
uint8_t  dir_cnt_delay;

uint8_t north_res = 0,south_res = 0, dir_res=0, dir_state;
uint8_t ns_bmu_cnt = 0;
uint8_t bmu_is_ring = 0;
uint8_t ns_recheck_cnt = 0;
extern bq796xx_wake_tone_switch wake_sw;
uint8_t wake_cnt = 0;

long int test_bmu_statistics_num[2][BMU_TOTAL_BOARDS+1] ={0};
long int test_bmu_ring_total_count = 0;
uint16_t  test_init_rec_bmu_num[2][40]={0};

uint16_t test_gpio_HL=0;
uint8_t	test_ntcIoIndex =0;
uint8_t	test_ntcChannelStartIndex = 0;
uint8_t	test_ntcChannelOffset = 0;
uint8_t	test_NtcChannelSelect = 0;

void Test_BQ796XX_Setting_Init_Without_Step(void){
  uint8_t res;
	
  drv_bq796xx_init_default_load(unit_testonly_afe_load_data);

	drv_bq796xx_init();

	bq796xx_event_RegisteCB(app_afe_cb);
	GPIOD->ODR &= ~GPIO_PIN_14;
	
	bq_count_temp = smp_time_count_get();
	
	res = drv_bq796xx_start_setting(BMU_TOTAL_BOARDS, DIR_NORTH);
  
	#if 0
	res = drv_bq796xx_start_setting(BMU_TOTAL_BOARDS, DIR_SOUTH);
  #endif 
	
	drv_bq796xx_Read_Stack_FaultSummary(0);
	HAL_Delay(10);
				
  for(int k=0; k<10*BMU_TOTAL_BOARDS; k++){			
	    res = drv_bq796xx_data_frame_parser(); 	
	}
}

void Test_BQ796XX_Setting_Init_With_Step(void){
	for(int k = 0; k < TEST_BQ796XX_SETTING_INIT_WITH_STEP_TEST_CYCLE_NUM; k++){
	    afe_steps = 0;
	    cnt_delay = 0;
		  res = 0;
		 
		  smp_time_count_set(0);
		
		  if((k%2)==0){			
          dir_state = DIR_NORTH;
			}else{
			    dir_state = DIR_SOUTH;
			}	
			
			++wake_cnt;
			if(wake_cnt >= TEST_BQ796XX_SETTING_INIT_WITH_STEP_WAKE_CNT){
			   wake_sw = WAKE_TONE_DISABLE;
			}else{
			   wake_sw = WAKE_TONE_ENABLE;
			}
		
			
			wake_sw = WAKE_TONE_ENABLE;
			
			#if 0
			dir_state = DIR_NORTH;
			#endif 
			
	    while(1){	
		      GPIOD->ODR ^= GPIO_PIN_13;
		  
		      if(cnt_delay==0){
	            res = drv_bq796xx_Init_Steps(wake_sw,&afe_steps, bq796xx_default.bmu_total_num , dir_state, &step_com_f , &before_d_ms);
				      cnt_delay = before_d_ms;
					    if(step_com_f == 1){
		              ++afe_steps;
			        }
			    }
  		    GPIOD->ODR ^= GPIO_PIN_13;
 			
          if(bq_count_temp !=smp_time_count_get()){	   
				     bq_count_temp = smp_time_count_get();
				     if(cnt_delay>0){
					         --cnt_delay;
				     }
			    }
		
		      if((res>=0) && (afe_steps > AFE_RUN_AUX_ADC)){
				    break;
		      }
	    }
	
   	  GPIOD->ODR |= GPIO_PIN_14;

			if(dir_state == DIR_NORTH){
			    north_res = res & (~BQ796XX_BMU_RING_MASK);
			}else{
			    south_res = res & (~BQ796XX_BMU_RING_MASK);
			}
			
			test_init_rec_bmu_num[dir_state][res & (~BQ796XX_BMU_RING_MASK)]++;
			
			bmu_is_ring = ((res & BQ796XX_BMU_RING_MASK)>> BQ796XX_BMU_COUNT_BITS);                                        //Results MSB is  0 = Non Ring, 1= Ring. 

			SEGGER_RTT_printf(0,"BMU Init#%04d N=%d,S=%d\r\n", k, north_res, south_res);
			
	    test_cont = 0;
	    while(1){
				 drv_bq796xx_clear_fifobuffer();
	       drv_bq796xx_Read_AFE_ALL_VCELL(STACK, 0, 0);                          //Stack(BMU #1,#2) Read all VCell 1~16(Main ADC).
				 drv_bq796xx_delay_ms(1*bq796xx_default.bmu_total_num);                //Please set  delay > (n X BMU conut) ms
				 
				 drv_bq796xx_Read_AFE_ALL_ADC(STACK, 0, 0);                            //Stack(BMU #1,#2) Read all AUX ADC   (GPIO1~8).
	       drv_bq796xx_delay_ms(1*bq796xx_default.bmu_total_num);                //Please set  delay > (n X BMU conut) ms
		
				 drv_bq796xx_Read_Stack_FaultSummary(0);                               //Stack read Fault summary status.
				 drv_bq796xx_delay_ms(1*bq796xx_default.bmu_total_num);                //Please set  delay > (n X BMU conut) ms

				 for(int ki =0; ki<10*BMU_TOTAL_BOARDS; ki++){
				     res = drv_bq796xx_data_frame_parser();
				 }					 
		     
				 if(test_cont >=2) break;
		     test_cont++;
	    }	
  }
}

void Test_BQ796XX_Gpio_Select_Read_ADC_FUNC(void){
  test_cont = 0;
	while(1){
			test_NtcChannelSelect = test_ntcChannelStartIndex + test_ntcChannelOffset;
	    if(test_NtcChannelSelect >= 16){	
		     test_NtcChannelSelect = 0;
				 test_ntcChannelOffset =0;
			}
			
			//test_NtcChannelSelect = 1;
			LOG_GREEN("Stack set BMU GPIO select Ch=%02d\r\n", test_NtcChannelSelect);
			
			//Set BMU GPIO 
			//------------------------------
	    for(int i = 0 ;i<4 ; i++){   
	        if(test_ntcIoIndex == 0)
	        {
	           test_gpio_HL = 5- (test_NtcChannelSelect & 0x01);	
		         drv_bq796xx_Set_AFE_GPIO_type(STACK, 0x00, test_gpio_HL, AFE_GPIO4,0);//BQ796XX_CMD_DELAY_MS);
	        }
	        else if(test_ntcIoIndex == 1)
	        {
		         test_gpio_HL = 5-((test_NtcChannelSelect & 0x02)>>1);
	 	         drv_bq796xx_Set_AFE_GPIO_type(STACK, 0x00, test_gpio_HL, AFE_GPIO5,0);//BQ796XX_CMD_DELAY_MS);	
	       }
	       else if(test_ntcIoIndex == 2)
	      {	
		         test_gpio_HL = 5-((test_NtcChannelSelect & 0x04)>>2);
		         drv_bq796xx_Set_AFE_GPIO_type(STACK, 0x00, test_gpio_HL, AFE_GPIO6,0);//BQ796XX_CMD_DELAY_MS);
	      }
	      else if(test_ntcIoIndex == 3)
	      {
		         test_gpio_HL = 5-((test_NtcChannelSelect & 0x08)>>3);
		         drv_bq796xx_Set_AFE_GPIO_type(STACK, 0x00, test_gpio_HL, AFE_GPIO7,0);//BQ796XX_CMD_DELAY_MS);
	      }

    	  test_ntcIoIndex++;
	      if(test_ntcIoIndex >= 4)
	      {
		       test_ntcIoIndex = 0;
					 drv_bq796xx_delay_ms(10); 
	      } 				
	      drv_bq796xx_delay_ms(2); 
		 }
		 	
		 test_ntcChannelOffset++;
     //------------------------------
		 
	  drv_bq796xx_clean_fifo();
	  drv_bq796xx_Read_AFE_ALL_ADC(STACK, 0, 0);    //Stack(BMU #1,#2) Read all AUX ADC   (GPIO1~8).
			
    drv_bq796xx_delay_ms(10); 
		 
		for(int ki =0; ki<10*BMU_TOTAL_BOARDS; ki++){
          res = drv_bq796xx_data_frame_parser();
		}
			
		drv_bq796xx_delay_ms(10);
		LOG_GREEN("Stack Read GPIO1=");
		
		for(int ki =0; ki<BMU_TOTAL_BOARDS; ki++){ 
        LOG_GREEN("%d,", bq796xx_data.gpio_data[ki][0]); 
		}
		LOG_GREEN("\r\n");
		
		test_cont++;
		if(test_cont > TEST_BQ796XX_GPIO_SELECT_READ_ADC_FUNC_TEST_CYCLE_NUM) break;
		
		HAL_Delay(500); 
  }
}

void Test_BQ796XX_Cell_balance_func(void){
	uint8_t res;
	uint8_t bmu_set_cb_id;
	
	//Daisy Chain direction : North
	res = drv_bq796xx_start_setting(BMU_TOTAL_BOARDS, DIR_NORTH);
	LOG_GREEN("BMU Count:%02d...\r\n", res);
	
	//clear all BMU cell balance func.
  drv_bq796xx_CellBalance_1to8_clear(STACK, 0 , 1);
  drv_bq796xx_CellBalance_9to16_clear(STACK, 0 , 1);
	 
	//from North direction frist BMU ID.
	bmu_set_cb_id = 0x01; 
	
  //Test Cell balance
	#if 1
	//Cell 1,3,5,7,9,11,13,15 enable cell balance func =10s.
  cb_en1 =((1 << 6) | (1 << 4) | (1 << 2) | (1 << 0));
  cb_en2 =((1 << 6) | (1 << 4) | (1 << 2) | (1 << 0));				
	#endif
	
	#if 0
	//Cell 2,4,6,8,10,12,14,16 enable cell balance func =10s.
  cb_en1 =((1 << 7) | (1 << 5) | (1 << 3) | (1 << 1));
  cb_en2 =((1 << 7) | (1 << 5) | (1 << 3) | (1 << 1));				
	#endif

  //Setting BMU=0x01
	drv_bq796xx_CellBalance_1to8_set (bmu_set_cb_id, cb_en1,CB_TIME_10S,1);
  drv_bq796xx_CellBalance_9to16_set(bmu_set_cb_id, cb_en2,CB_TIME_10S,1);		
	
	test_cont = 0;
	
	while(1){
	  //Starting cell balance, cell balance active until CB_CELLXX TIME, then this bit be self-clearing.
	  LOG_GREEN("BMU Cell balance start #%06d...\r\n", test_cont);
		drv_bq796xx_Stack_CellBalanceStarting(CB_MANUAL, 1);
		drv_bq796xx_delay_ms(20);
		test_cont++;
		if(test_cont > TEST_BQ796XX_CELL_BALANCE_FUNC_TEST_CYCLE_NUM) break;
	}
}

// BQ796XX  Direction check North/South BMU number Test with step.
void Test_BQ796XX_Direction_Chechk_BMU_with_Step(void){
	north_res = 0;
	south_res = 0;
	bmu_is_ring = 0;

  for(long int k = 0; k < TEST_BQ796XX_DIRECTION_CHECK_BMU_WITH_STEP_TEST_CYCLE_NUM; k++){
	    dir_afe_steps = 0;
	    dir_cnt_delay = 0;
		  dir_res = 0;
		
		  drv_bq796xx_clear_fifobuffer();
		  smp_time_count_set(0);
		  
		  if((k%2)==0){			
          dir_state = DIR_NORTH;
			}else{
			    dir_state = DIR_SOUTH;
			}		
	    
      //Check these condition, if it is true then skip auto addressing.
      //-------------------------------------------------------------			
			if( (north_res ==bq796xx_default.bmu_total_num) || (south_res ==bq796xx_default.bmu_total_num) || ((north_res+ south_res)==bq796xx_default.bmu_total_num)){
		      if((k%2)==0){			
              ns_bmu_cnt = north_res;    //Skip autoaddressing	
			    }else{
			        ns_bmu_cnt = south_res;    //Skip autoaddressing	
			    }
					
					++ns_recheck_cnt;
					
					if(ns_recheck_cnt>=50){
						  ns_bmu_cnt = 0;   //Re autoaddressing	
						 if(ns_recheck_cnt>=51) ns_recheck_cnt = 0;
					}
			}else{
					ns_bmu_cnt = 0;   //Re autoaddressing	
			}	
			//-------------------------------------------------------------
	    while(1){	
		      GPIOD->ODR ^= GPIO_PIN_13;
		 
		      if(dir_cnt_delay==0){
						
						  ns_bmu_cnt = 0;  //Re autoaddressing
						
              dir_res = drv_bq796xx_direction_set_steps(ns_bmu_cnt, &dir_afe_steps, bq796xx_default.bmu_total_num , dir_state, &dir_step_com_f , &dir_before_d_ms);							
						  dir_cnt_delay=dir_before_d_ms;
					    if(dir_step_com_f == 1){
		              ++dir_afe_steps;
			        }
			    }
  		    GPIOD->ODR ^= GPIO_PIN_13;
 			
          if(dir_bq_count_temp !=smp_time_count_get()){	   
				     dir_bq_count_temp = smp_time_count_get();
				     if(dir_cnt_delay>0){
					     --dir_cnt_delay;
				     }
			    }
		
		      if((dir_res>=0) && (dir_afe_steps > SETDIR_AFE_RUN_AUX_ADC)){
				    break;
		      }
	    }
			
			if(dir_state == DIR_NORTH){
			    north_res = dir_res & (~BQ796XX_BMU_RING_MASK);
				  test_bmu_statistics_num[0][north_res]++;
			}else{
			    south_res = dir_res & (~BQ796XX_BMU_RING_MASK);
				  test_bmu_statistics_num[1][south_res]++;
			}
			
			bmu_is_ring = ((dir_res & BQ796XX_BMU_RING_MASK)>> BQ796XX_BMU_COUNT_BITS);                //Results MSB is  0 = Non Ring, 1= Ring. 
			
			test_bmu_ring_total_count += bmu_is_ring;
			
			LOG_GREEN("BMU CHK#%06d N=%d,S=%d,Ring=%d  ", k, north_res, south_res , bmu_is_ring);
			
			drv_bq796xx_delay_ms(dir_state+1);
			dir_step_com_f = 0;
			dir_before_d_ms = 0;
			
			test_cont = 0;
	    while(1){
				 drv_bq796xx_clear_fifobuffer();
				
	       drv_bq796xx_Read_AFE_ALL_VCELL(STACK, 0, 0);                               //Stack(BMU #1,#2) Read all VCell 1~16(Main ADC).
				 drv_bq796xx_delay_ms(1*bq796xx_default.bmu_total_num);                     //Please set  delay > (n X BMU conut) ms
				
				 drv_bq796xx_Read_AFE_ALL_ADC(STACK, 0, 0);                                 //Stack(BMU #1,#2) Read all AUX ADC   (GPIO1~8).
		     drv_bq796xx_delay_ms(1*bq796xx_default.bmu_total_num);                     //Please set  delay > (n X BMU conut) ms
				
				 drv_bq796xx_Read_Stack_FaultSummary(0);                                    //Stack read Fault summary status.
				 drv_bq796xx_delay_ms(1*bq796xx_default.bmu_total_num);                     //Please set  delay > (n X BMU conut) ms
				
				 for(int ki =0; ki<10*BMU_TOTAL_BOARDS; ki++){
				     res = drv_bq796xx_data_frame_parser();
				 }					 
		     
				 if(test_cont >=2) break;
		     test_cont++;
	    }
			
		  Test_BQ796XX_Direction_Chechk_BMU_with_Step_Print_MeasureData();
			
			if((k%100)==0){
				  HAL_Delay(100);
			    Test_BQ796XX_Direction_Chechk_BMU_with_Step_Print_Statistics(k+1);
				  HAL_Delay(100);
			}
			
			dir_step_com_f = 0;
			dir_before_d_ms = 0;
  }
	
	HAL_Delay(100);
	Test_BQ796XX_Direction_Chechk_BMU_with_Step_Print_Statistics(TEST_BQ796XX_DIRECTION_CHECK_BMU_WITH_STEP_TEST_CYCLE_NUM);
	HAL_Delay(100);
}

void Test_BQ796XX_Direction_Chechk_BMU_with_Step_Print_MeasureData(void){
			LOG_GREEN("VCell16=");
			for(int ki =0; ki<BMU_TOTAL_BOARDS; ki++){
			    LOG_GREEN("%04d,", ((int)((float)(bq796xx_data.vcell_data[ki][15])*BQ79656_RESOLUTION_CELL_MAIN)));
			}
      LOG_GREEN("mV  "); 
			
			LOG_GREEN("VBusBar=");
			for(int ki =0; ki<BMU_TOTAL_BOARDS; ki++){
			    LOG_GREEN("%04d,", ((int)((float)(bq796xx_data.busbar_data[ki])*BQ79656_RESOLUTION_BB)));
			}
      LOG_GREEN("mV  "); 			
			
			LOG_GREEN("GPIO1 ADC=");
		  for(int ki =0; ki<BMU_TOTAL_BOARDS; ki++){ 
        LOG_GREEN("%05d,", bq796xx_data.gpio_data[ki][0]); 
		  }
			LOG_GREEN("  ");
	
			LOG_GREEN("Fault summary(hex)=");
		  for(int ki =0; ki<BMU_TOTAL_BOARDS; ki++){ 
        LOG_GREEN("%02x,", bq796xx_data.fault_summary[ki]); 
		  }
			LOG_GREEN("\r\n");	
}

void Test_BQ796XX_Direction_Chechk_BMU_with_Step_Print_Statistics(long int k_cnt){
	LOG_GREEN("BMU CHK statistics Read BMU Count:\r\n");
	LOG_GREEN("Total Test = %06d, Current#%06d \r\n",TEST_BQ796XX_DIRECTION_CHECK_BMU_WITH_STEP_TEST_CYCLE_NUM, k_cnt);
	LOG_GREEN("North Direction BMU Count= ");
	for(int ki =0; ki<(BMU_TOTAL_BOARDS+1); ki++){ 
      LOG_GREEN("%05d,", test_bmu_statistics_num[0][ki]); 
  }
	LOG_GREEN("\r\n");
	
	LOG_GREEN("South Direction BMU Count= ");
	for(int ki =0; ki<(BMU_TOTAL_BOARDS+1); ki++){ 
      LOG_GREEN("%05d,", test_bmu_statistics_num[1][ki]); 
  }
	LOG_GREEN("\r\n");	
	LOG_GREEN("BMU Ring Count= %d \r\n",test_bmu_ring_total_count);
}

void Test_BQ796XX_SET_GPIO_Channel(uint8_t gpio_ch){
    uint16_t tgpio_HL=0;
    uint8_t	tIndex =0;    
   
  	//Set BMU GPIO 
    //------------------------------
    for(int i = 0 ;i<4 ; i++){   
	        if(tIndex == 0)
	        {
	           tgpio_HL = 5- (gpio_ch & 0x01);	
		         drv_bq796xx_Set_AFE_GPIO_type(STACK, 0x00, tgpio_HL, AFE_GPIO4,0);//BQ796XX_CMD_DELAY_MS);
	        }
	        else if(tIndex == 1)
	        {
		         tgpio_HL = 5-((gpio_ch & 0x02)>>1);
	 	         drv_bq796xx_Set_AFE_GPIO_type(STACK, 0x00, tgpio_HL, AFE_GPIO5,0);//BQ796XX_CMD_DELAY_MS);	
	       }
	       else if(tIndex == 2)
	      {	
		         tgpio_HL = 5-((gpio_ch & 0x04)>>2);
		         drv_bq796xx_Set_AFE_GPIO_type(STACK, 0x00, tgpio_HL, AFE_GPIO6,0);//BQ796XX_CMD_DELAY_MS);
	      }
	      else if(tIndex == 3)
	      {
		         tgpio_HL = 5-((gpio_ch & 0x08)>>3);
		         drv_bq796xx_Set_AFE_GPIO_type(STACK, 0x00, tgpio_HL, AFE_GPIO7,0);//BQ796XX_CMD_DELAY_MS);
	      }

    	  tIndex++;
	      if(tIndex >= 4)
	      {
		       tIndex = 0;
					 drv_bq796xx_delay_ms(10); 
	      } 				
	      drv_bq796xx_delay_ms(2); 
    }
	 
}

uint8_t Test_BQ796XX_DIRECTION_SET_STEPS_NORTH(void){
    dir_afe_steps = 0;
	  dir_cnt_delay = 0;
    dir_res = 0;
		dir_step_com_f = 0;
		dir_before_d_ms = 0;
	
    drv_bq796xx_clear_fifobuffer();
    smp_time_count_set(0);

	  //---------------------------------------------------
    while(1){	
		    GPIOD->ODR ^= GPIO_PIN_13;
		 
			  ns_bmu_cnt =0; //Re addressing & wakeup
			
        if(dir_cnt_delay==0){
            dir_res = drv_bq796xx_direction_set_steps(ns_bmu_cnt, &dir_afe_steps, bq796xx_default.bmu_total_num , DIR_NORTH, &dir_step_com_f , &dir_before_d_ms);							
            dir_cnt_delay=dir_before_d_ms;
            if(dir_step_com_f == 1){
                ++dir_afe_steps;
            }
        }

        GPIOD->ODR ^= GPIO_PIN_13;
 			
        if(dir_bq_count_temp !=smp_time_count_get()){	   
            dir_bq_count_temp = smp_time_count_get();
            if(dir_cnt_delay>0){
                --dir_cnt_delay;
            }
        }
		
        if((dir_res>=0) && (dir_afe_steps > SETDIR_AFE_RUN_AUX_ADC)){
            break;
        }
    }	
		
    north_res = dir_res & 0x7F;
			
    bmu_is_ring = ((dir_res & 0x80)>> 7);                                       //Results MSB is  0 = Non Ring, 1= Ring. 		
		
		LOG_GREEN("BMU CHK N=%d\r\n", north_res);
	  //---------------------------------------------------
		
		return(north_res);
}

void Test_BQ796XX_OVUVOTUT_FUNC(void){
  uint8_t d_payload[4]={0};
	north_res = 0;
	south_res = 0;
	bmu_is_ring = 0;

	
    LOG_GREEN("TEST BMU BQ795XX OVUVOTUT(FXIED_Channel CH=0)\r\n"); 
	  HAL_Delay(1000);
	
	  //Init BQ796XX
	  Test_BQ796XX_DIRECTION_SET_STEPS_NORTH();
	
	  //Select NTC Channel = 0 (NTC0 ~ NTC15), BMU with multipxer control NTC to GPIO1 input. 
	  Test_BQ796XX_SET_GPIO_Channel(0);

	  //Clear All nFault.
	  fill_data4_payload(d_payload,0xFF,0xFF,0,0); //Clear all=x0FF
	  drv_bq796xx_command_framing(STACK_WRITE, 0x00, BQ79600_FAULT_RST1, 2, d_payload, 2);
	  drv_bq796xx_command_framing(SINGLE_WRITE, 0x00, BQ79600_FAULT_RST, 1, d_payload, 2);
		
	  test_cont = 0;
	  while(1){
				 drv_bq796xx_clear_fifobuffer();
				
	       drv_bq796xx_Read_AFE_ALL_VCELL(STACK, 0, 0);                               // All Stack(BMU) Read all VCell 1~16(Main ADC).
				 drv_bq796xx_delay_ms(1*bq796xx_default.bmu_total_num);                     //Please set  delay > (n X BMU conut) ms
				
			   drv_bq796xx_Read_AFE_ALL_Busbar(STACK, 0, 0);                              /// All Stack(BMU) Read Busbar Data(Main ADC).
			   drv_bq796xx_delay_ms(1*bq796xx_default.bmu_total_num);                     //Please set  delay > (n X BMU conut) ms   
			
				 drv_bq796xx_Read_AFE_ALL_ADC(STACK, 0, 0);                                 //Stack(BMU #1,#2) Read all AUX ADC   (GPIO1~8).
		     drv_bq796xx_delay_ms(1*bq796xx_default.bmu_total_num);                     //Please set  delay > (n X BMU conut) ms
				
				 drv_bq796xx_Read_Stack_FaultSummary(0);                                    //Stack read Fault summary status.
				 drv_bq796xx_delay_ms(1*bq796xx_default.bmu_total_num);                     //Please set  delay > (n X BMU conut) ms
				
			   fill_data4_payload(d_payload,1,0,0,0);
	       drv_bq796xx_command_framing(BROAD_READ, 0x00, BQ796XX_FAULT_MSK1, 1, d_payload, 0);
	       drv_bq796xx_delay_ms(10);	
		
		     drv_bq796xx_Read_Stack_FaultOVUV(0);
         drv_bq796xx_delay_ms(10);			
		     drv_bq796xx_Read_Stack_FaultOTUT(0);
         drv_bq796xx_delay_ms(10);		
		
				 for(int ki =0; ki<12*BMU_TOTAL_BOARDS; ki++){
				     res = drv_bq796xx_data_frame_parser();
				 }					 
		     
				 if(test_cont >= TEST_BQ796XX_OVUVOTUT_FUNC_TEST_CYCLE_NUM) break;
		     test_cont++; 
				 
				 Test_BQ796XX_Direction_Chechk_BMU_with_Step_Print_MeasureData();          //Show Read BUM Information
				 
				 HAL_Delay(1000);
	  }
			  
}
//--------FILE END------------------------------------------------------------------------------------------

/**
  ******************************************************************************
  * @file    smp_drv_bq796xx.c
  * @author  Golden Chen
  * @version V0.0.13
  * @date    2022/01/13
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

/* Includes ------------------------------------------------------------------*/
#include "Bsp.h"
#include "smp_drv_bq796xx.h"
#include "smp_debug.h"
#include "smp_fifo.h"
#include "stm32l4xx_Davinci.h"
#include "main.h"
#include "smp_uart.h"
#include "LibHwTimer.h"
#include "RTT_Log.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint8_t  bq796xx_tx_buffer[BQ796XX_TX_BUFFER_SIZE] = {0};
uint8_t  bq796xx_rx_buffer[BQ796XX_RX_BUFFER_SIZE] = {0};
uint8_t  bq796xx_res_buf[BQ796XX_RESPONE_BUFFER_SIZE]={0};
uint16_t bq796xx_res_buf_c = 0;

#define BQ796XX_UART        {                                                                   \
                                .num                        = UART0,                            \
                                .baud_rate                  = BQ796XX_UART_BAUD_RATE,           \
                                .flow_ctrl                  = UART_FLOW_CTRL_DISABLE,           \
                                .use_parity                 = PARITY_NONE,                      \
                                .buffers.rx_buf             = bq796xx_rx_buffer,                \
                                .buffers.rx_buf_size        = BQ796XX_RX_BUFFER_SIZE,           \
                                .buffers.tx_buf             = bq796xx_tx_buffer,                \
                                .buffers.tx_buf_size        = BQ796XX_TX_BUFFER_SIZE,           \
                             }	  

smp_gpio_t      bq796xx_gpio_ncs = BQ796XX_NCS_GPIO;
smp_gpio_state  bq796xx_gpio_ncs_state;    
                                
smp_gpio_t      bq796xx_gpio_rx  = BQ796XX_RX_GPIO;
smp_gpio_state  bq796xx_gpio_rx_state;    
      

#define BQ796XX_DEAULT_VALUE    {                                                            \
													          .ov_threshold            =OV_4175MV,                     \
	                                  .ov_step_threshold       =OV_STEP_50MV,                  \
	                                  .uv_threshold            =UV_3000MV,                     \
	                                  .ot_threshold            =OT_10_PCT,                     \
	                                  .ut_threshold            =UT_80_PCT,                     \
	                                  .ov_enable               =BQ_ENABLE,                     \
	                                  .uv_enable               =BQ_ENABLE,                     \
		                                .ot_enable               =BQ_ENABLE,                     \
	                                  .ut_enable               =BQ_ENABLE,                     \
	                                  .cb_cycle_time           =CB_CYC_T_10S,                  \
	                                  .bmu_total_num           =BMU_TOTAL_BOARDS,              \
                                }	
													
bq796xx_init_default_t bq796xx_default=BQ796XX_DEAULT_VALUE;
	                   												
uint8_t bq796xx_afe_gpio_conf[4]={0};                          //[0]=GPIO2[5:3],GPIO1[2:0],[1]=GPIO4[5:3],GPIO3[2:0],[2]=GPIO6[5:3],GPIO5[2:0],[3]=GPIO8[5:3],GPIO7[2:0]
uint8_t bq796xx_afe_fault_msk1=0x00;
uint8_t bq796xx_afe_fault_msk2=0x00;
uint8_t bq796xx_afe_fault_rst1=0x00;

bq796xx_data_t bq796xx_data;

smp_uart_t bq796xx_uart = BQ796XX_UART;

uint16_t smp_time_count = 0;
bq796xx_CB_Fun_t bq796xx_event_cb;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

//uint16_t smp_time_count = 0;
void drv_bq796xx_HwTimerHanlder(__far void *dest, uint16_t evt, void *vDataPtr)
{
	smp_time_count++;
}


uint16_t smp_time_count_get(void){
    return(smp_time_count);
}

void smp_time_count_set(uint16_t val){
    smp_time_count = val;
}

void drv_bq796xx_delay_ms(uint32_t ms_c){
  uint32_t nop_c=0;
  uint32_t time_out=0;
    
  time_out = HAL_GetTick()+ ms_c;
  while(HAL_GetTick()< time_out){
    nop_c++;
  }  
}
                                                                                            
void drv_bq796xx_switch_rx_pin_type_setting(bq796xx_io_type type){

    switch(type){
    case BQ_GPIO:
        // BQ796XX rx pin init and set pin to pull high.
        smp_gpio_init(&bq796xx_gpio_rx);
        bq796xx_gpio_rx_state = GPIO_ACTIVE_HIGH;
        smp_gpio_set_state(&bq796xx_gpio_rx, bq796xx_gpio_rx_state);
        break;
    case BQ_UART:
        // BQ796XX to MCU UART init      
        if(smp_uart_init(&bq796xx_uart, drv_bq796xx_uart_event_handler)==SMP_SUCCESS){
            //LOG_MAGENTA("BMU UART initial success!\r\n");
        }else{
            LOG_MAGENTA("BMU UART initial fail!\r\n");
        }  
        break;    
    }
}

void drv_bq796xx_uart_puts(uint8_t *d_bytes,int16_t d_size){
  uint16_t i = 0;
  
  for(i=0; i<d_size ;i++){
    smp_uart_put(&bq796xx_uart,d_bytes[i]);
  }
}

void drv_bq796xx_clear_fifobuffer(void){
	uint8_t rx_data = 0;
	static uint8_t aaaa;
//	  GPIOD->ODR |= GPIO_PIN_14;
    while(smp_uart_get(&bq796xx_uart, &rx_data)==SMP_SUCCESS){
        
    }
		
		bq796xx_res_buf_c=0;
		
}

void drv_bq796xx_rx_pin_wakeup(void){

  bq796xx_gpio_rx_state = GPIO_ACTIVE_HIGH;
  smp_gpio_set_state(&bq796xx_gpio_rx, bq796xx_gpio_rx_state);
  drv_bq796xx_delay_ms(10);
  
  bq796xx_gpio_rx_state = GPIO_ACTIVE_LOW;
  smp_gpio_set_state(&bq796xx_gpio_rx, bq796xx_gpio_rx_state);
  drv_bq796xx_delay_ms(3);   //keep low for wake base device(base on Spec. for BQ79600 wakeup timing is 2.5ms ~ 3ms) 
  
  bq796xx_gpio_rx_state = GPIO_ACTIVE_HIGH;
  smp_gpio_set_state(&bq796xx_gpio_rx, bq796xx_gpio_rx_state);
}    
                                                                
uint8_t drv_bq796xx_init(void)
{
  // BQ796XX nCS pin init and set pin to pull low.
  smp_gpio_init(&bq796xx_gpio_ncs);
  bq796xx_gpio_ncs_state = GPIO_ACTIVE_HIGH;
  smp_gpio_set_state(&bq796xx_gpio_ncs, bq796xx_gpio_ncs_state);
  //drv_bq796xx_delay_ms(10);
	
  bq796xx_gpio_ncs_state = GPIO_ACTIVE_LOW;
  smp_gpio_set_state(&bq796xx_gpio_ncs, bq796xx_gpio_ncs_state);
 	
  drv_bq796xx_switch_rx_pin_type_setting(BQ_UART);
  
	
	LibHwTimerOpen(drv_bq796xx_HwTimerHanlder, 0);

//	#include "LibSwTimer.h"

//	#define SMP_TIMER_SAMPLE_RATE                          100         // Unit : Hz

  //Setup timer 10ms
//  smp_timer_handler(SMP_TIMER_SAMPLE_RATE);
//  smp_time_count_set(0);
  
  return 0;
}

uint8_t Word2u8(uint16_t word_data, uint8_t type){
    uint8_t data;
    if ( type == HIGH_BYTE){
        data = (uint8_t)((word_data&0xFF00)>>8);
    }else{
        data = (uint8_t)(word_data&0xFF);
    }
    return data;
}

//Remove this CRC function to Uliti
uint16_t Func_Cal_ModbusCRC16(uint8_t *data, uint8_t len)
{
  uint8_t  j;
  uint16_t crc = 0xFFFF;

  while(len--){
        crc ^= ((*data)&0xFF);
        data++;
        for(j=0;j<8;j++){
            if(crc & 0x01){ /* LSB(b0)=1 */
                crc = (crc>>1) ^ 0xA001;
            }else{
                crc = crc >>1;
            }    
        }
  }
  return crc;  // this is correct for Modbus

} 

void drv_bq796xx_command_framing(uint8_t cmd_type, uint8_t dev_id, uint16_t reg_addr, uint8_t datalen, uint8_t *data_array, uint32_t delayms){
    uint8_t i, write[20], data_size, addr_h, addr_l, index = 1;
    uint16_t  crc;
  
    // ---------- reset RX buffer before send out command ----------
    data_size = 5 + datalen;                               // initial(1) + address(2) + CRC(2) + databytes(xu8DataLen)
    addr_h = Word2u8(reg_addr,HIGH_BYTE);
    addr_l = Word2u8(reg_addr,LOW_BYTE);

    // ---------- reset check initial and ID byte for single R/W command ----------
    switch (cmd_type){
        case SINGLE_WRITE:
            cmd_type = cmd_type + datalen-1;                // add length for write command
        case SINGLE_READ:
            data_size++;                                    // one more device ID byte for single R/W
            write[index++] = dev_id;
            break;
        case STACK_WRITE:
        case BROAD_WRITE:
            cmd_type = cmd_type + datalen-1;               // add length for write command
            break;
    }

    // ---------- fill initial and address bytes ----------
    write[0] = cmd_type;
    write[index++] = addr_h;
    write[index++] = addr_l;

    // ---------- fill data bytes ----------
    for ( i = 0; i < datalen; i++ ){
        write[data_size-3-i] = data_array[datalen-i-1];
    }

    // ---------- fill CRC bytes ----------
    crc = Func_Cal_ModbusCRC16(write,data_size-2);
        
    write[data_size-2] = (uint8_t)(crc&0xFF);
    write[data_size-1] = (uint8_t)((crc&0xFF00)>>8);

    // ---------- send out packet ----------
    drv_bq796xx_uart_puts(write, data_size);

		if(delayms!=0){
        drv_bq796xx_delay_ms(delayms);
		}
}

void fill_data4_payload(uint8_t *payload,uint8_t b1,uint8_t b2,uint8_t b3,uint8_t b4){
  payload[0]=b1;
  payload[1]=b2;
  payload[2]=b3;
  payload[3]=b4;
}

uint8_t drv_bq796xx_start_setting(uint8_t maxcnt, uint8_t dir){

    uint8_t    d_payload[4] = {0};
    uint8_t    null_payload[4] = {0};
    uint8_t    i, bmu_cnt = 0, dir_sel;
    uint16_t   dir_addr;
    uint8_t    top_no;
    
    uint8_t res=0;
    
		top_no = maxcnt;
		
    fill_data4_payload(null_payload,0x00,0,0,0);
    
    if ( dir == DIR_NORTH ){
        dir_addr = BQ79600_DIR0_ADDR;
        dir_sel  = DIR_NORTH;
    }else{
        dir_addr = BQ79600_DIR1_ADDR;
        dir_sel = (DIR_SOUTH*BQ79616_DIR_SEL);
    }
    
    //Step 1: bq79600 Wakeup 
    //--------------------------------------------------------------------------------------------------------
		drv_bq796xx_switch_rx_pin_type_setting(BQ_GPIO);
    //if ( dir == DIR_NORTH ){
    drv_bq796xx_rx_pin_wakeup();
    drv_bq796xx_delay_ms(10);                                                                                                           // v1.0.4Q2 reduce delay time for bq79600 from 100ms to 10ms(follow Spec.)    wait for 79600 wakeup(<10ms)
    //}
    //--------------------------------------------------------------------------------------------------------  
  
    //MCU pin switch to UART function, wait 30ms UART Ready
    drv_bq796xx_switch_rx_pin_type_setting(BQ_UART);
    drv_bq796xx_delay_ms(30); 
    
    //Step 2: Setting Wakeup, FCOMMNFAULT + OTP ECC TEST( to syn the DLL delay-locked loop)
    //--------------------------------------------------------------------------------------------------------
    fill_data4_payload(d_payload,WAKEUP | dir_sel,0,0,0);
    drv_bq796xx_command_framing(SINGLE_WRITE, 0x00, BQ79600_CONTROL1, 1, d_payload, BQ796XX_CMD_DELAY_MS);
    drv_bq796xx_delay_ms(5*bq796xx_default.bmu_total_num);                                                                                            //Each device need Max 10ms for wakeup from reset
    
    fill_data4_payload(d_payload,FCOMM_EN+NFAULT_EN,0,0,0);
    drv_bq796xx_command_framing(SINGLE_WRITE, 0x00, BQ79600_DEV_CONF1, 1, d_payload, BQ796XX_CMD_DELAY_MS);                               //bq79600 only, Device config1 => enable fault detection and NFault function
    drv_bq796xx_command_framing(SINGLE_WRITE, 0x00, BQ79600_OTP_ECC_TEST, 1, null_payload,   BQ796XX_CMD_DELAY_MS);                       //OTP_ECC_TEST disable
    //--------------------------------------------------------------------------------------------------------

    //Step 3: If direction set to SOUTH, then send BW Reverse (if direct is North(COMH), BQ79600 default: [DIR_SEL] bit=0 ,North, so it can't seting)
    //--------------------------------------------------------------------------------------------------------
    if ( dir == DIR_SOUTH ){
         fill_data4_payload(d_payload,dir_sel,0,0,0);
         drv_bq796xx_command_framing(BROAD_WRITE_REVERSE, 0, BQ79600_CONTROL1, 1, d_payload, BQ796XX_CMD_DELAY_MS);     //South direction for all devices	
    }else{
         //fill_data4_payload(d_payload,dir_sel,0,0,0);
        // drv_bq796xx_command_framing(BROAD_WRITE, 0, BQ79600_CONTROL1, 1, d_payload, BQ796XX_CMD_DELAY_MS);             //North direction for all devices						
		}		
		
		bq796xx_data.comm_dir = dir;
		
    drv_bq796xx_command_framing(BROAD_WRITE, 0x00, BQ79600_OTP_ECC_TEST, 1, null_payload, BQ796XX_CMD_DELAY_MS);                          // OTP_ECC_TEST disable
    //--------------------------------------------------------------------------------------------------------
    
    //Step 4: Dummy broadcast write 1~8
    //--------------------------------------------------------------------------------------------------------
    for ( i = 0; i < 8; i++ ){                                                                                                            // 8 is fixed value according to Spec. 
      drv_bq796xx_command_framing(BROAD_WRITE, 0, BQ79600_OTP_ECC_DATAIN1+i, 1, null_payload, BQ796XX_CMD_DELAY_MS);
    }
    //--------------------------------------------------------------------------------------------------------
    
    //Step 5: (1) Auto addressing enable, (2) Send address to each devices, (3) Check each devices address
    //--------------------------------------------------------------------------------------------------------
    //(1) Auto addressing enable
    fill_data4_payload(d_payload,ADDR_WR | dir_sel,0,0,0);
    drv_bq796xx_command_framing(BROAD_WRITE, 0, BQ79600_CONTROL1, 1, d_payload, BQ796XX_CMD_DELAY_MS);

    //(2) Send address to each devices
    for ( i = 0; i <= MAX_AFE_CNT; i++ ){                                                                                      // 6 bits for address ID
        fill_data4_payload(d_payload,i,0,0,0);
        drv_bq796xx_command_framing(BROAD_WRITE, 0, dir_addr, 1, d_payload, BQ796XX_CMD_DELAY_MS);                             // 3000 is base on TI's log file
    }
    drv_bq796xx_delay_ms(10);
    
    //Dummy broadcast read 1~8
    for ( i = 0; i < 8; i++ ){                                                                                                 // 8 is fixed value according to Spec.
        drv_bq796xx_command_framing(BROAD_READ, 0, BQ79600_OTP_ECC_DATAIN1+i, 1, null_payload, BQ796XX_CMD_DELAY_MS);
    }
    drv_bq796xx_delay_ms(10);
    
    //(3) Check each devices address
    for ( bmu_cnt = 1; bmu_cnt <= top_no; bmu_cnt++){
        drv_bq796xx_command_framing(SINGLE_READ,  bmu_cnt, dir_addr, 1, null_payload, BQ796XX_CMD_DELAY_MS);                  // 10000 is base on TI's log file
        res = drv_bq796xx_check_respone_event();
        if (res == 0){                                                                                                         // no return data => no this ID
            break;
        }
    }
    
    --bmu_cnt;
    if (bmu_cnt == 0 ){   //bmu_cnt==0 is not any device detection.
        return 0;  
    }
    drv_bq796xx_delay_ms(10);
    //-------------------------------------------------------------------------------------------------------- 

    //Step 6: Set stack/base/top (i should set it after get device count)
    //--------------------------------------------------------------------------------------------------------
    fill_data4_payload(d_payload,BQ79600_STACK,0,0,0);
    drv_bq796xx_command_framing(BROAD_WRITE,   0, BQ79600_COMM_CTRL, 1, d_payload, BQ796XX_CMD_DELAY_MS);                                 // write stack
    fill_data4_payload(d_payload,BQ79600_BASE,0,0,0);
    drv_bq796xx_command_framing(SINGLE_WRITE,  0x00, BQ79600_COMM_CTRL, 1, d_payload, BQ796XX_CMD_DELAY_MS);                              // write base
    if (bmu_cnt > maxcnt ){
        bmu_cnt = maxcnt;
    }
    
    fill_data4_payload(d_payload,BQ79600_TOP,0,0,0);
    drv_bq796xx_command_framing(SINGLE_WRITE,  bmu_cnt, BQ79600_COMM_CTRL, 1, d_payload, BQ796XX_CMD_DELAY_MS);                          // write top to last BMU
    //--------------------------------------------------------------------------------------------------------
        
    //Step 7: Clear fault
    //--------------------------------------------------------------------------------------------------------
    fill_data4_payload(d_payload,0xFF,0,0,0);
    drv_bq796xx_command_framing(BROAD_WRITE,   0, BQ79600_FAULT_RST1, 1, d_payload, BQ796XX_CMD_DELAY_MS);                               // clear fault1
    drv_bq796xx_command_framing(BROAD_WRITE,   0, BQ79600_FAULT_RST2, 1, d_payload, BQ796XX_CMD_DELAY_MS);                               // clear fault2
    
    fill_data4_payload(d_payload,BQ79600_RST_FCOMM_DET | BQ79600_RST_SYS,0,0,0);
    drv_bq796xx_command_framing(SINGLE_WRITE,  0x00, BQ79600_FAULT_RST, 1,d_payload , BQ796XX_CMD_DELAY_MS);                             // bq79600 only,  fault reset => reset comm and fault syserr
    
    drv_bq796xx_command_framing(STACK_WRITE,   0, BQ79600_OTP_SPARE13, 1, null_payload, BQ796XX_CMD_DELAY_MS);                           // OTP_SPARE 13
    drv_bq796xx_command_framing(SINGLE_WRITE,  0x00, BQ79600_COMM_TIMEOUT, 1, null_payload, BQ796XX_CMD_DELAY_MS);                       // bq79600 only, command timeout => disable CTS/CTL(no command timeout for short/long)


    //Step 8: Enable TSREF
    //--------------------------------------------------------------------------------------------------------
    fill_data4_payload(d_payload,BQ79600_TSREF_EN,0,0,0);
    drv_bq796xx_command_framing(STACK_WRITE,   0, BQ79600_CONTROL2, 1, d_payload, BQ796XX_CMD_DELAY_MS);                                  // Enable TSREF(5V)
    //--------------------------------------------------------------------------------------------------------

    //Step 9: Setting main ADC
    //--------------------------------------------------------------------------------------------------------
    fill_data4_payload(d_payload,BMU_CELL_SERIES-6,0,0,0);
    drv_bq796xx_command_framing(BROAD_WRITE,   0, BQ79600_ACTIVE_CELL, 1, d_payload, BQ796XX_CMD_DELAY_MS);                              // Set active cell No. to 16s(0=6s)
    
    fill_data4_payload(d_payload,0x03,0,0,0);
    drv_bq796xx_command_framing(BROAD_WRITE,   0, BQ79600_ADC_CONF1, 1, d_payload, BQ796XX_CMD_DELAY_MS);                                // 26Hz LPF_Vcell (38ms average)     2:26hz(38ms), 3:53hz(19ms)
    
    fill_data4_payload(d_payload,0x1E,0,0,0);
    drv_bq796xx_command_framing(STACK_WRITE,   0, BQ79600_ADC_CTRL1, 1, d_payload, BQ796XX_CMD_DELAY_MS);                                // [LPF BB EN]=1, Cell filter([3]=1), start ADC([2]=1), continue run([1:0]=2)
    drv_bq796xx_delay_ms(38+5*bmu_cnt);                                                                                                  // Initial delay to allow LPF to average for 38ms (26Hz LPF setting used)
    
    // set comparator threshold for OV/UV/OT/UT WriteReg(0, ACTIVE_CELL, 0x0A, 1, FRMWRT_ALL_W);                                         // Set all cells to active
    fill_data4_payload(d_payload,0x0D,0,0,0);
    drv_bq796xx_command_framing(STACK_WRITE,   0, BQ79600_COMM_TIMEOUT_CONF, 1, d_payload, BQ796XX_CMD_DELAY_MS);                         // Let BMU shutdown after no communication 10 s
    //--------------------------------------------------------------------------------------------------------
    
		//Step 10: //All BMU Device will allow two adjacent CB FETs to be enabled
		//--------------------------------------------------------------------------------------------------------
		fill_data4_payload(d_payload,((FCOMM_EN | NFAULT_EN) &(~(NO_ADJ_CB))), 0, 0, 0);   
    drv_bq796xx_command_framing(STACK_WRITE, 0x00, BQ796XX_DEV_CONF,1, d_payload, 0);
		//--------------------------------------------------------------------------------------------------------
		
    return bmu_cnt;
}

uint8_t drv_bq796xx_Read_AFE_ALL_VCELL(bq796xx_AFE_GPIO_stack is_stack,uint8_t dev_id,uint32_t delays){
  uint8_t    d_payload[4] = {0};
  
  fill_data4_payload(d_payload,(BMU_CELL_SERIES*2)-1,0,0,0); //read 32 bytes 
  
  if(is_stack == STACK)  
      drv_bq796xx_command_framing(STACK_READ, 0, BQ79600_VCELL16_H, 1, d_payload, delays);
  else
      drv_bq796xx_command_framing(SINGLE_READ, dev_id, BQ79600_VCELL16_H, 1, d_payload, delays);
  
  return 0;
}

uint8_t drv_bq796xx_Set_AFE_GPIO_type(bq796xx_AFE_GPIO_stack is_stack,uint8_t dev_id,bq796xx_AFE_GPIO_Type GPIO_type,bq796xx_AFE_GPIO GPIO_Num,uint32_t delays){
  
  static uint8_t    d_payload[4] = {0};
  static uint8_t    set_gpio =0;
  static uint8_t    set_reg_addr =0;
  
  set_gpio = bq796xx_afe_gpio_conf[(GPIO_Num>>1)];
  
  if((GPIO_Num %2)==0){
      set_gpio &= BQ796XX_GPIOCONF_BIT_MARK_1;
      set_gpio |= (GPIO_type);
  }else{
      set_gpio &= BQ796XX_GPIOCONF_BIT_MARK_2; 
      set_gpio |= (GPIO_type<<3);
  }
    
  set_reg_addr = BQ79600_GPIO_CONF1+(GPIO_Num>>1);
  
  fill_data4_payload(d_payload,set_gpio,0,0,0);
  
  if(is_stack == STACK)  
      drv_bq796xx_command_framing(STACK_WRITE, 0, set_reg_addr, 1, d_payload, delays);
  else
      drv_bq796xx_command_framing(SINGLE_WRITE, dev_id, set_reg_addr, 1, d_payload, delays);
  
  bq796xx_afe_gpio_conf[(GPIO_Num>>1)] = set_gpio;
  
  return 0;
}

uint8_t drv_bq796xx_Start_AFE_ADC(bq796xx_AFE_GPIO_stack is_stack,uint8_t dev_id,uint32_t delays){
  
  uint8_t   adc_ctrl_1_d[4] = {0};
	
  fill_data4_payload(adc_ctrl_1_d,(BQ79616_LPF_BB_EN | BQ79616_LPF_VCELL_EN | BQ79616_MAIN_GO | BQ79616_MAIN_MODE_B1),0x00,(BQ79616_AUX_GO | BQ79616_AUX_MODE_B1),0);
  
  if(is_stack == STACK){  
      drv_bq796xx_command_framing(STACK_WRITE, 0, BQ79600_ADC_CTRL1, 3, adc_ctrl_1_d, delays);
  }else{
      drv_bq796xx_command_framing(SINGLE_WRITE, dev_id, BQ79600_ADC_CTRL1, 3, adc_ctrl_1_d, delays);
  }
	
  return 0;
}

uint8_t drv_bq796xx_Stop_AFE_ADC(bq796xx_AFE_GPIO_stack is_stack,uint8_t dev_id,uint32_t delays){
  
  uint8_t   adc_ctrl_1_d[4] = {0};
	
  fill_data4_payload(adc_ctrl_1_d,(BQ79616_LPF_BB_EN | BQ79616_LPF_VCELL_EN | BQ79616_MAIN_MODE_B1), 0x00, (BQ79616_AUX_MODE_B1),0);
  
  if(is_stack == STACK){  
      drv_bq796xx_command_framing(STACK_WRITE, 0, BQ79600_ADC_CTRL1, 3, adc_ctrl_1_d, delays);
  }else{
      drv_bq796xx_command_framing(SINGLE_WRITE, dev_id, BQ79600_ADC_CTRL1, 3, adc_ctrl_1_d, delays);
  }
  
  return 0;
}

uint8_t drv_bq796xx_Read_AFE_ALL_ADC(bq796xx_AFE_GPIO_stack is_stack,uint8_t dev_id,uint32_t delays){
  uint8_t    d_payload[4] = {0};
  
  fill_data4_payload(d_payload,(BQ79616_GPIO_NUM*2)-1,0,0,0); //read 16 bytes 
  
  if(is_stack == STACK)  
      drv_bq796xx_command_framing(STACK_READ, 0, BQ79600_GPIO1_RES_H, 1, d_payload, delays);
  else
      drv_bq796xx_command_framing(SINGLE_READ, dev_id, BQ79600_GPIO1_RES_H, 1, d_payload, delays);
  
  return 0;
}

uint8_t drv_bq796xx_Goto_ShutDownMode(uint32_t delays){
  uint8_t    d_payload[4] = {0};
  
  fill_data4_payload(d_payload,BQ79616_GOTO_SHUTDOWN,0,0,0); //[GOTO_SHUTDOWN]=1
  drv_bq796xx_command_framing(BROAD_WRITE, 0x00, BQ79600_CONTROL1, 1, d_payload, delays);
  
  return 0;
}

uint8_t drv_bq796xx_Goto_SleepMode(uint32_t delays){
  uint8_t    d_payload[4] = {0};
  
  fill_data4_payload(d_payload,BQ79616_GOTO_SLEEP,0,0,0); //[BQ79616_GOTO_SLEEP]=1
  drv_bq796xx_command_framing(SINGLE_WRITE, 0x00, BQ79600_CONTROL1, 1, d_payload, delays);
  
  return 0;
}

uint8_t drv_bq796xx_Send_WakeupAll(uint32_t delays){
  uint8_t    d_payload[4] = {0};
  
  drv_bq796xx_switch_rx_pin_type_setting(BQ_GPIO);
  drv_bq796xx_rx_pin_wakeup();
  drv_bq796xx_delay_ms(10);
  
  drv_bq796xx_switch_rx_pin_type_setting(BQ_UART);
  drv_bq796xx_delay_ms(30);
  fill_data4_payload(d_payload,BQ79616_SEND_WAKE,0,0,0); //[SEND_WAKE]=1
  drv_bq796xx_command_framing(SINGLE_WRITE, 0x00, BQ79600_CONTROL1, 1, d_payload, delays);
  
  return 0;
}

uint8_t drv_bq796xx_Set_Mask_FaultAll(bq796xx_AFE_GPIO_stack is_stack, uint8_t dev_id, uint32_t delays){
  uint8_t    d_payload[4] = {0};
  
  fill_data4_payload(d_payload,0xFF,0xFF,0,0); //Mask all=x0FF
	
	if(is_stack == STACK){
      drv_bq796xx_command_framing(STACK_WRITE, 0, BQ796XX_FAULT_MSK1, 2, d_payload, delays);

		  bq796xx_afe_fault_msk1=0xFF;
		  bq796xx_afe_fault_msk2=0xFF;
  }else{
      drv_bq796xx_command_framing(SINGLE_WRITE, dev_id, BQ796XX_FAULT_MSK1, 2, d_payload, delays);
	}
	
  //BROAD CAST READ t to check
	//fill_data4_payload(d_payload,2,0,0,0);
	//drv_bq796xx_command_framing(BROAD_READ, 0x00, BQ796XX_FAULT_MSK1, 1, d_payload, delays);
	//drv_bq796xx_delay_ms(500);
	
  return 0;
}

uint8_t drv_bq796xx_Clear_Mask_Fault_OVUV(bq796xx_AFE_GPIO_stack is_stack, uint8_t dev_id, uint32_t delays){
  static uint8_t    d_payload[4] = {0};
  
	bq796xx_afe_fault_msk1 &= ~(BQ79616_MSK_OV|BQ79616_MSK_UV);
	
  fill_data4_payload(d_payload,bq796xx_afe_fault_msk1,0,0,0); //MSK_OV=0, MSK_UV=0
  
	if(is_stack == STACK){
	    drv_bq796xx_command_framing(STACK_WRITE, 0, BQ796XX_FAULT_MSK1, 1, d_payload, delays);
		  
	}else{
	    drv_bq796xx_command_framing(SINGLE_WRITE, dev_id, BQ796XX_FAULT_MSK1, 1, d_payload, delays);
	}
	
  //BROAD CAST READ t to check
	//fill_data4_payload(d_payload,1,0,0,0);
	//drv_bq796xx_command_framing(BROAD_READ, 0x00, BQ796XX_FAULT_MSK1, 1, d_payload, delays);
	//drv_bq796xx_delay_ms(500);	
	
  return 0;
}

uint8_t drv_bq796xx_Clear_Mask_Fault_OTUT(bq796xx_AFE_GPIO_stack is_stack, uint8_t dev_id, uint32_t delays){
  static uint8_t    d_payload[4] = {0};
  
	bq796xx_afe_fault_msk1 &= ~(BQ79616_MSK_OT|BQ79616_MSK_UT);
	
  fill_data4_payload(d_payload,bq796xx_afe_fault_msk1,0,0,0); //MSK_OT=0, MSK_UT=0
	
	if(is_stack == STACK){
      drv_bq796xx_command_framing(STACK_WRITE, dev_id, BQ796XX_FAULT_MSK1, 1, d_payload, delays);
	}else{
      drv_bq796xx_command_framing(SINGLE_WRITE, dev_id, BQ796XX_FAULT_MSK1, 1, d_payload, delays);
	}
	
  //BROAD CAST READ t to check
	//fill_data4_payload(d_payload,1,0,0,0);
	//drv_bq796xx_command_framing(BROAD_READ, 0x00, BQ796XX_FAULT_MSK1, 1, d_payload, delays);
	//drv_bq796xx_delay_ms(500);	
	
  return 0;
}

uint8_t drv_bq796xx_Set_BroadCast_Mask_FaultAll(uint32_t delays){
  uint8_t    d_payload[4] = {0};
  
  fill_data4_payload(d_payload,0xFF,0xFF,0,0); //Mask all=x0FF
  drv_bq796xx_command_framing(BROAD_WRITE, 0x00, BQ796XX_FAULT_MSK1, 2, d_payload, delays);
  
	bq796xx_afe_fault_msk1= 0xFF;
	bq796xx_afe_fault_msk2= 0xFF;
	
	//BROAD CAST READ t to check
//	drv_bq796xx_delay_ms(1);
//	fill_data4_payload(d_payload,2,0,0,0);
//	drv_bq796xx_command_framing(BROAD_READ, 0x00, BQ796XX_FAULT_MSK1, 1, d_payload, delays);
//	drv_bq796xx_delay_ms(5);
	
  return 0;
}

uint8_t drv_bq796xx_Set_BroadCast_Mask_FaultSel(uint8_t msk1,uint8_t msk2,uint32_t delays){
  uint8_t    d_payload[4] = {0};

  fill_data4_payload(d_payload,msk1,msk2,0,0); //Mask all=x0FF
  drv_bq796xx_command_framing(BROAD_WRITE, 0x00, BQ796XX_FAULT_MSK1, 2, d_payload, delays);
	
	bq796xx_afe_fault_msk1= msk1;
	bq796xx_afe_fault_msk2= msk2;  
		
  return 0;
}


uint8_t drv_bq796xx_Clear_FaultRstAll(uint32_t delays){
  uint8_t    d_payload[4] = {0};
  
  fill_data4_payload(d_payload,0xFF,0xFF,0,0); //Clear all=x0FF
  drv_bq796xx_command_framing(STACK_WRITE, 0x00, BQ79600_FAULT_RST1, 2, d_payload, delays);
  drv_bq796xx_command_framing(SINGLE_WRITE, 0x00, BQ79600_FAULT_RST, 1, d_payload, delays);
	
	bq796xx_afe_fault_rst1= 0xFF;

  return 0;
}

uint8_t drv_bq796xx_Read_Stack_FaultSummary(uint32_t delays){
  uint8_t    d_payload[4] = {0};
  
  fill_data4_payload(d_payload,0,0,0,0);
  drv_bq796xx_command_framing(STACK_READ, 0x00, BQ796XX_FAULT_SUMMARY,1, d_payload, delays);
 
  return 0;
}

uint8_t drv_bq796xx_Read_Base_FaultSummary(uint32_t delays){
  uint8_t    d_payload[4] = {0};
  
  fill_data4_payload(d_payload,0,0,0,0);
  drv_bq796xx_command_framing(SINGLE_READ, 0x00, BQ79600_FAULT_SUMMARY,1, d_payload, delays);
 
  return 0;
}

//OVUV Function
//------------------------------------------------------------
uint8_t drv_bq796xx_Set_Stack_OV(bq796xx_ov_range ov_range_mv,bq796xx_ov_step ov_step_mv ,uint32_t delays){
  uint8_t    d_payload[4] = {0};
    
  fill_data4_payload(d_payload,ov_range_mv + ov_step_mv,0,0,0);
  drv_bq796xx_command_framing(STACK_WRITE, 0x00, BQ796XX_OV_THRESH,1, d_payload, delays);
 
  return 0;
}

uint8_t drv_bq796xx_Set_Stack_UV(bq796xx_uv_mv uv_mv,uint32_t delays){
  uint8_t    d_payload[4] = {0};
    
  fill_data4_payload(d_payload,uv_mv,0,0,0);
  drv_bq796xx_command_framing(STACK_WRITE, 0x00, BQ796XX_UV_THRESH,1, d_payload, delays);
 
  return 0;
}

uint8_t drv_bq796xx_Run_OVUV(uint32_t delays){
  uint8_t    d_payload[4] = {0};
    
  fill_data4_payload(d_payload,BQ79616_OVUV_GO | BQ79616_OVUV_MODE_B0,0,0,0);   //Start OV_UV + OVUV_MODE=0x01: Run the OV and UV round robin with all active cells.
  drv_bq796xx_command_framing(STACK_WRITE, 0x00, B796XX_OVUV_CTRL,1, d_payload, delays);
 
  return 0;
}

uint8_t drv_bq796xx_Read_Stack_FaultOVUV(uint32_t delays){
  uint8_t    d_payload[4] = {0};
  
  fill_data4_payload(d_payload,3,0,0,0);    //Read 4Bytes(Fault_OV1,Fault_OV2,Fault_UV1,Fault_OV2)
  drv_bq796xx_command_framing(STACK_READ, 0x00, BQ796XX_FAULT_OV1,1, d_payload, delays);
 
  return 0;
}
//------------------------------------------------------------

//OTUT Function
//------------------------------------------------------------
uint8_t drv_bq796xx_Set_Stack_OTUT(bq796xx_ot_threshold_p ot_th_pct,bq796xx_ut_threshold_p ut_th_pct,uint32_t delays){
  static uint8_t    d_payload[4] = {0};
    
  fill_data4_payload(d_payload,ot_th_pct + (ut_th_pct<<5),0,0,0);
  drv_bq796xx_command_framing(STACK_WRITE, 0x00, BQ796XX_OTUT_THRESH,1, d_payload, delays);
 
  return 0;
}

uint8_t drv_bq796xx_Set_Stack_OTUT_Associate(bq796xx_AFE_GPIO afe_gpio,uint32_t delays){
  drv_bq796xx_Set_AFE_GPIO_type(STACK,0x00,GPIO_ADC_OTUT,afe_gpio,delays); 
  return 0;
}

uint8_t drv_bq796xx_Run_OTUT(uint32_t delays){
  uint8_t    d_payload[4] = {0};
    
  fill_data4_payload(d_payload,BQ79616_OTUT_GO | BQ79616_OTUT_MODE_B0,0,0,0);   //Start OT_UT + OTUT_MODE=0x01: Run the OT and UT round robin with all active cells.
  drv_bq796xx_command_framing(STACK_WRITE, 0x00, B796XX_OTUT_CTRL,1, d_payload, delays);
 
  return 0;
}

uint8_t drv_bq796xx_Read_Stack_FaultOTUT(uint32_t delays){
  uint8_t    d_payload[4] = {0};
  
  fill_data4_payload(d_payload,1,0,0,0);    //Read 2Bytes(Fault_OT,Fault_UT)
  drv_bq796xx_command_framing(STACK_READ, 0x00, BQ796XX_FAULT_OT,1, d_payload, delays);
 
  return 0;
}
//------------------------------------------------------------


//Celll Balance Function
//------------------------------------------------------------
uint8_t drv_bq796xx_Set_Stack_CellBalanceTime(bq796xx_cellbalance_time cb_time,uint32_t delays){
  uint8_t    d_payload[4] = {0};  
  uint8_t     cell_cnt=0;

  fill_data4_payload(d_payload, cb_time,0,0,0);
  
  cell_cnt = 0;
  
  while(cell_cnt < BMU_CELL_SERIES){
     if((cell_cnt%2)==0){
         fill_data4_payload(d_payload, cb_time,0,0,0);
     }else{
         fill_data4_payload(d_payload, 0,0,0,0); 
     }
     
     drv_bq796xx_command_framing(STACK_WRITE, 0x00, BQ796XX_CB_CELL01_CTRL-cell_cnt,1, d_payload, delays);
     ++cell_cnt;
  }
  
  return 0;
}

uint8_t drv_bq796xx_Set_Stack_CellBalanceCycleTime(bq796xx_cellbalance_cycle_time cb_cyc_time,uint32_t delays){
  static uint8_t    d_payload[4] = {0};  

  fill_data4_payload(d_payload, cb_cyc_time,0,0,0);
  drv_bq796xx_command_framing(STACK_WRITE, 0x00, BQ796XX_BAL_CTRL1,1, d_payload, delays);
  return 0;
}

uint8_t drv_bq796xx_Stack_CellBalanceStarting(bq796xx_cellbalance_control cb_ctrl, uint32_t delays){
  static uint8_t    d_payload[4] = {0};  

	if(cb_ctrl==CB_AUTO){
      fill_data4_payload(d_payload, (BQ79616_BAL_GO | BQ79616_AUTO_BAL),0,0,0);
	}else{
      fill_data4_payload(d_payload, (BQ79616_BAL_GO),0,0,0);	
	}
	
  drv_bq796xx_command_framing(STACK_WRITE, 0x00, BQ796XX_BAL_CTRL2,1, d_payload, delays);
  
	return 0;
}
//------------------------------------------------------------

uint8_t drv_bq796xx_check_respone_event(void){
  static uint16_t rcv_temp_cnt=0;
  
  smp_time_count_set(0);
  do{
  
    if(bq796xx_res_buf_c >= BQ796XX_DATAFRAME_MIN_LEN){ 
        rcv_temp_cnt = bq796xx_res_buf_c;
        break;
    }   
    if(smp_time_count_get() >= BQ796XX_TIMEOUT_10MS) return(BQ796XX_RES_TIMEOUT);      
  }while(1);
  
  smp_time_count_set(0);
  do{  
    if(bq796xx_res_buf_c > rcv_temp_cnt){ //keep waiting RX data
        smp_time_count_set(0);
    }else{
				rcv_temp_cnt = bq796xx_res_buf_c;
		}   
		
    if(smp_time_count_get()>=1) break;      
  }while(1);
  
  drv_bq796xx_clear_fifobuffer();
  
  return(BQ796XX_RES_RCV_DAT);
}

uint8_t drv_bq796xx_check_respone_event2(void){
  static uint16_t rcv_temp_cnt=0;
  static uint8_t steps=0;
		
	switch(steps){
		case 0:
			  smp_time_count_set(0);
		    ++steps;
	      break;
    case 1:
        if(bq796xx_res_buf_c >= BQ796XX_DATAFRAME_MIN_LEN){ 
            rcv_temp_cnt = bq796xx_res_buf_c;
            ++steps;                                          //next step
        } 		
        if(smp_time_count_get() >= BQ796XX_TIMEOUT_10MS){ 
					steps =0;
					rcv_temp_cnt = 0;
					return(BQ796XX_RES_TIMEOUT); 		
				}
			  break;
	  case 2:
			  smp_time_count_set(0);
		    ++steps;
		    break;
	  case 3:
        if(bq796xx_res_buf_c > rcv_temp_cnt){ //keep waiting RX data
            smp_time_count_set(0);
        }
				rcv_temp_cnt = bq796xx_res_buf_c;    
        
				if(smp_time_count_get()>=1){
					++steps;                                          //next step
			  }
				break;	
    case 4:
        drv_bq796xx_clear_fifobuffer();		
				steps =0;
				return(BQ796XX_RES_RCV_DAT);	
	}
	
	return(BQ796XX_RES_WAITTING); 
}

void appSerialCanDavinciSendTextMessage(uint8_t *str);
void DumpBuffer(uint8_t *pBuf,uint16_t len);
extern smp_fifo_t 								uart0_rx_fifo;// = {0};

uint8_t drv_bq796xx_data_frame_parser(void)
{
  uint8_t rx_data = 0;
  uint8_t i=0;
  static uint16_t rcv_temp_cnt=0;
  static uint16_t read_reg_adr = 0;
  static uint8_t device_id = 0;
  static uint8_t data_len=0;
  static uint16_t  crc,rcv_crc;
	static int8_t fifo_res;
  
  bq796xx_event_cb_type bq_event_type = BQ_EVENT_OTHER_ERR;
  
  // Capture data and check data frame
  //----------------------------------------------------------------------------
  //Short data frame
  //----------------------------------------
	#if 0
  smp_time_count_set(0);
  do{  
    if(bq796xx_res_buf_c >= BQ796XX_DATAFRAME_MIN_LEN){
        rcv_temp_cnt = bq796xx_res_buf_c;
        break;
    }    
    if(smp_time_count_get()>=BQ796XX_TIMEOUT_10MS) return(BQ796XX_RES_TIMEOUT);      
  }while(1);    
  //----------------------------------------

  smp_time_count_set(1);
  do{  
    if(bq796xx_res_buf_c> rcv_temp_cnt){ //keep waiting RX data
        smp_time_count_set(0);
    }    
    if(smp_time_count_get()>=1) break;      
  }while(1);
  //----------------------------------------
  #endif
	
  if(smp_uart_get(&bq796xx_uart, &rx_data)==SMP_SUCCESS){
    --bq796xx_res_buf_c;
    if(rx_data < BQ796XX_PAYLOAD_MAX_LEN){
        bq796xx_res_buf[BQ796XX_DF_RES_PAYLOAD_LEN] = rx_data;
        data_len=rx_data;        
      
        for(i=0;i <(data_len+6);i++){
          fifo_res = smp_uart_get(&bq796xx_uart, &rx_data);
			if(fifo_res != 0)
			{
	//			appSerialCanDavinciSendTextMessage("empty ");
			}
          bq796xx_res_buf[BQ796XX_DF_RES_PAYLOAD_LEN+1+i] = rx_data;
        }
        bq796xx_res_buf_c = bq796xx_res_buf_c - (data_len+6);
    }else{
        return(BQ796XX_RES_ERR);
    }        
  }else{
    bq796xx_res_buf_c = 0;
    return(BQ796XX_RES_ERR);
  }
  
  //Check CRC
  //-----------------------------------------------------------------------------------------------------
  //DumpBuffer(bq796xx_res_buf, data_len+5);
  
  crc = bq796xx_res_buf[BQ796XX_DF_RES_PAYLOAD_LEN+data_len+5] + bq796xx_res_buf[BQ796XX_DF_RES_PAYLOAD_LEN+data_len+6]*256; 
  rcv_crc = Func_Cal_ModbusCRC16(bq796xx_res_buf,data_len+5);
  if(rcv_crc != crc) 	  
  {
	//	GPIOD->ODR ^= GPIO_PIN_15;
	//  appSerialCanDavinciSendTextMessage("Crc error");
	  return(BQ796XX_RES_ERR_CRC);  
  }
  //-----------------------------------------------------------------------------------------------------
  
  //Paser data frame and stroage data
  //-----------------------------------------------------------------------------------------------------
  read_reg_adr = bq796xx_res_buf[BQ796XX_DF_REG_ADR_MSB]*256+ bq796xx_res_buf[BQ796XX_DF_REG_ADR_LSB];
  
	device_id = bq796xx_res_buf[BQ796XX_DF_DEV_ADR];
	
	if(bq796xx_data.comm_dir == DIR_SOUTH){
	    device_id = (bq796xx_default.bmu_total_num+1) - device_id;
	}
	
  switch(read_reg_adr){
    case BQ79600_VCELL16_H:
      for( i=0 ;i < (data_len+1)/2; i++){
          bq796xx_data.vcell_data[device_id-1][15-i]=bq796xx_res_buf[BQ796XX_DF_REG_PAYLOAD+(i*2)] * 256.0f + bq796xx_res_buf[BQ796XX_DF_REG_PAYLOAD+(i*2+1)];
      }
			
			bq_event_type = BQ_EVENT_VCELL;
			
      break;
    case BQ79600_GPIO1_RES_H:  
      for( i=0 ;i < (data_len+1)/2; i++){
          bq796xx_data.gpio_data[device_id-1][i]=bq796xx_res_buf[BQ796XX_DF_REG_PAYLOAD+(i*2)] * 256.0f + bq796xx_res_buf[BQ796XX_DF_REG_PAYLOAD+(i*2+1)];
      }    
			
			bq_event_type = BQ_EVENT_GPIO_ADC;
			
      break;
    case BQ796XX_FAULT_SUMMARY:  
      bq796xx_data.fault_summary[device_id-1] = bq796xx_res_buf[BQ796XX_DF_REG_PAYLOAD];
		  
		  bq_event_type = BQ_EVENT_FAULT;
		
      break;
    case BQ796XX_FAULT_OV1:  
      bq796xx_data.fault_ov[device_id-1] = bq796xx_res_buf[BQ796XX_DF_REG_PAYLOAD]*256+ bq796xx_res_buf[BQ796XX_DF_REG_PAYLOAD+1];
      bq796xx_data.fault_uv[device_id-1] = bq796xx_res_buf[BQ796XX_DF_REG_PAYLOAD+2]*256+ bq796xx_res_buf[BQ796XX_DF_REG_PAYLOAD+3];
    
      bq_event_type = BQ_EVENT_FAULTOVUV;		
		
		  break;      
    case BQ796XX_FAULT_OT:  
      bq796xx_data.fault_ot[device_id-1] = bq796xx_res_buf[BQ796XX_DF_REG_PAYLOAD];
      bq796xx_data.fault_ut[device_id-1] = bq796xx_res_buf[BQ796XX_DF_REG_PAYLOAD+1];
		
		  bq_event_type = BQ_EVENT_FAULTOTUT;		
      break;    
    case BQ79600_DIR0_ADDR:  
      bq796xx_data.top_stack_north_id = bq796xx_res_buf[BQ796XX_DF_REG_PAYLOAD];
      bq796xx_data.top_stack_south_id = bq796xx_res_buf[BQ796XX_DF_REG_PAYLOAD+1];
		
		  bq_event_type = BQ_EVENT_DIR_ADDR;		
      break; 
    case BQ79600_ACTIVE_CELL:  
      bq796xx_data.ns_flag = (bq796xx_res_buf[BQ796XX_DF_REG_PAYLOAD] & 0xC0);
      
		  bq_event_type = BQ_EVENT_ACTIVE_CELL;		
      break; 		
  }
  //-----------------------------------------------------------------------------------------------------
  
	bq796xx_data.paser_id = device_id;
	
	//Execution callback function
	if(bq796xx_event_cb != NULL){
	    bq796xx_event_cb(&bq796xx_data, bq_event_type);
  }
	
	//bq796xx_uart.
  //int8_t smp_fifo_clean(smp_fifo_t *p_fifo)
  //smp_fifo_t 								uart0_rx_fifo = {0};
  //uart0_rx_fifo.in =0;
  //uart0_rx_fifo.out =0;
	
	return(BQ796XX_RES_OK);
}

void drv_bq796xx_uart_event_handler(uart_evt_type p_evt)
{
  
  switch(p_evt){
    case UART_DATA_READY:
      /* received data handle */
      bq796xx_res_buf_c++;
    break;
    case UART_TX_EMPTY:
      /* Data transmission complete handle */
    break;
    case UART_COMMUNICATION_ERR:
      /* occurred during reception */
    break;
    case UART_BUFFER_FULL:
      /* occurred UART buffer full */
    break;    
		case UART_TX_READY_TO_SEND:
			
		break;
    default:
    break;
  }
}

int8_t bq796xx_event_RegisteCB(bq796xx_CB_Fun_t callbackfunc){

    if(callbackfunc==NULL){
        return(SMP_ERROR_NULL);
		}   
		
    bq796xx_event_cb = callbackfunc;
	
    return(SMP_SUCCESS);	
}

int8_t bq796xx_event_UnregisteCB(void){
    bq796xx_event_cb = NULL;
    
	  if(bq796xx_event_cb==NULL){
        return(SMP_SUCCESS);	
		}else{
        return(SMP_ERROR_NOT_SUPPORTED);		
		}
}

uint8_t drv_bq796xx_Init_Steps(bq796xx_wake_tone_switch wake_tone_sw, bq796xx_init_steps_enum *afe_phase,uint8_t maxcnt, uint8_t dir, uint8_t *step_complete_f, uint8_t *before_delay_ms){
    uint8_t    d_payload[4] = {0};
    uint8_t    null_payload[4] = {0};
    
		static uint8_t    bmu_cnt = 0, dir_sel;
    static uint16_t   dir_addr;
    static uint8_t    top_no = 0;
    static uint8_t    sub_step=0;
		static uint8_t    rcv_check_step = 0;
		static uint8_t    is_ring = 0;
			
    uint8_t res=0;		
		static uint8_t wait_index=0;
		static uint8_t cell_cnt=0;
		static uint8_t set_val;
		static uint8_t retry_cnt = 0;
			
		*step_complete_f = 0;
		*before_delay_ms = 0;
		
		top_no = maxcnt;
		is_ring = 0;
		
    switch(*afe_phase){
        case AFE_INIT_IDLE:
				    if(wake_tone_sw == WAKE_TONE_ENABLE){
		            drv_bq796xx_switch_rx_pin_type_setting(BQ_GPIO);
							  sub_step = 0;
		        }else{	
							  sub_step = 3;
						}
					  *before_delay_ms = 0;
					  *step_complete_f = 1;
            break;
        case AFE_INIT_WAKE_UP:                                              // send reset pulse(~2.5ms)
            switch(sub_step){
				    case 0:
						    bq796xx_gpio_rx_state = GPIO_ACTIVE_HIGH;
                smp_gpio_set_state(&bq796xx_gpio_rx, bq796xx_gpio_rx_state);
					      *before_delay_ms = 20;
								++sub_step;
                break;
						case 1:
                bq796xx_gpio_rx_state = GPIO_ACTIVE_LOW;
                smp_gpio_set_state(&bq796xx_gpio_rx, bq796xx_gpio_rx_state);
                *before_delay_ms = 2;   //keep low for wake base device(base on Spec. for BQ79600 wakeup timing is 2.5ms ~ 3ms) 
								++sub_step;
                break;
						case 2:
                bq796xx_gpio_rx_state = GPIO_ACTIVE_HIGH;
                smp_gpio_set_state(&bq796xx_gpio_rx, bq796xx_gpio_rx_state);
								*before_delay_ms = 20;
								++sub_step;
								break;								
						case 3:
							  if(wake_tone_sw == WAKE_TONE_ENABLE){
						        drv_bq796xx_switch_rx_pin_type_setting(BQ_UART);                //MCU pin switch to UART function, wait 30ms UART Ready
                    //drv_bq796xx_delay_ms(2); 
								}
						 
   				      if ( dir == DIR_NORTH ){
                    dir_addr = BQ79600_DIR0_ADDR;
                    dir_sel  = DIR_NORTH;							  
                }else{                                                          // need not to wakeup, just jump to wakeup device(Setting SOUTH direction)
                    dir_addr = BQ79600_DIR1_ADDR;
                    dir_sel = (DIR_SOUTH*BQ79616_DIR_SEL);
                }
							  wait_index = 0;
								++sub_step;
						    break;
					  }
						
					  if(sub_step>=4){
							  sub_step = 0;
							  *before_delay_ms = 0;
						    *step_complete_f = 1;
						}
            break;
        case AFE_INIT_WAKE_UP_WAIT:                                                                                    // wait for all devices wake up
						*before_delay_ms = (AFE_WAKE_WAIT_TIME_BASE * maxcnt);
						*step_complete_f = 1;						
            break;
        case AFE_INIT_BRORAD_SET_BMU_STACK1:                                                                           // Before use "Broadcast Write Revese" cmd, you must be have to clear TOP stack role device BMU.
				    fill_data4_payload(d_payload, BQ79600_STACK,0,0,0);
            drv_bq796xx_command_framing(BROAD_WRITE, 0x00, BQ796XX_COMM_CTRL, 1, d_payload, 0);                        // Set everything as a stack device first.
            *before_delay_ms = 1;
            *step_complete_f = 1;
            break;		
        case AFE_INIT_SET_BASE1:
            fill_data4_payload(d_payload,BQ79600_BASE,0,0,0);
            drv_bq796xx_command_framing(SINGLE_WRITE,  0x00, BQ79600_COMM_CTRL, 1, d_payload, 0); 
				
						*before_delay_ms = 1;
						*step_complete_f = 1;	
				    
				    retry_cnt = 0;
            break;	
        case AFE_INIT_WAKE_DEV:                                             																					 // wake up base 				
				    if(wake_tone_sw == WAKE_TONE_ENABLE){
				         fill_data4_payload(d_payload, WAKEUP | dir_sel,0,0,0);
							   *before_delay_ms = 10 * maxcnt;
						}else{
						     fill_data4_payload(d_payload, dir_sel,0,0,0);
							   *before_delay_ms = 1;
							   retry_cnt=3;
						}
				
            drv_bq796xx_command_framing(SINGLE_WRITE, 0x00, BQ79600_CONTROL1, 1, d_payload, 0);
            
						++retry_cnt;
						if(retry_cnt>=1){
							  retry_cnt = 0;
						    *step_complete_f = 1;
						}							
            break;	
        case AFE_INIT_FAULT_DET_ENABLE:
            fill_data4_payload(d_payload,FCOMM_EN | NFAULT_EN,0,0,0);
            drv_bq796xx_command_framing(SINGLE_WRITE, 0x00, BQ79600_DEV_CONF1, 1, d_payload, 0);                       //bq79600 only, Device config1 => enable fault detection and NFault function

						*before_delay_ms = 1;
						*step_complete_f = 1;					
    				break;
        case AFE_INIT_WAIT_LONG:
            wait_index++;
            if (wait_index >= (maxcnt*AFE_WAKE_WAIT_TIME_BASE) ){
                wait_index = 0;
            }
						
						*before_delay_ms = 0;
						*step_complete_f = 1;	
            break;
        case AFE_INIT_OTP_ECC_TEST_DISABLE_BASE:
            drv_bq796xx_command_framing(SINGLE_WRITE, 0x00, BQ79600_OTP_ECC_TEST, 1, null_payload,   0); 
				
						*before_delay_ms = 1;
						*step_complete_f = 1;
            break;
        case AFE_INIT_SET_DIR:
            fill_data4_payload(d_payload, dir_sel,0,0,0);                                                //Must to do this North direction=BWR 0x00, South direction=BWR 0x80 
            drv_bq796xx_command_framing(BROAD_WRITE_REVERSE, 0, BQ79600_CONTROL1, 1, d_payload, 0);      //If a device receives communication frame opposite to the [DIR_SEL] setting, change direction for all devices	
		
						bq796xx_data.comm_dir = dir;
											
            *before_delay_ms = 2;
            *step_complete_f = 1;
            break;
        case AFE_INIT_OTP_ECC_TEST_DISABLE_BROAD:
            drv_bq796xx_command_framing(BROAD_WRITE, 0x00, BQ79600_OTP_ECC_TEST, 1, null_payload, 0); 
				
						*before_delay_ms = 1;
						*step_complete_f = 1;
            break;
        case AFE_INIT_DUMMY_WRITE:					
            drv_bq796xx_command_framing(BROAD_WRITE, 0, BQ79600_OTP_ECC_DATAIN1+sub_step, 1, null_payload, 0);                // 8 is fixed value according to Spec. 
            ++sub_step;
						
						*before_delay_ms = 1;
						
						if(sub_step>=8){
							  sub_step = 0; 
						    *step_complete_f = 1;
						}
            break;
        case AFE_INIT_AUTO_ADDR:
            fill_data4_payload(d_payload,ADDR_WR | dir_sel,0,0,0);                                                            // Auto addressing enable
            drv_bq796xx_command_framing(BROAD_WRITE, 0, BQ79600_CONTROL1, 1, d_payload, 0);
            
						*before_delay_ms = 1;
						*step_complete_f = 1;
				    sub_step = 0;
				    break;
        case AFE_INIT_SET_ID:
                                                                                                        
            fill_data4_payload(d_payload,sub_step,0,0,0);                                                                     // 6 bits for address ID
            drv_bq796xx_command_framing(BROAD_WRITE, 0, dir_addr, 1, d_payload, 0);                                           // 3000 is base on TI's log file
            ++sub_step;				
						*before_delay_ms = 1;
						
						if(sub_step > top_no){
							  sub_step = 0; 
						    *step_complete_f = 1;
						}						
            break;
        case AFE_INIT_SET_ID_WAIT:
            wait_index++;
            if(wait_index >= AFE_READ_WAIT_TIME ){
                wait_index = 0;
            }
						
						*before_delay_ms = 0;
						*step_complete_f = 1;
            break;
        case AFE_INIT_DUMMY_READ:         
            drv_bq796xx_command_framing(BROAD_READ, 0, BQ79600_OTP_ECC_DATAIN1+sub_step, 1, null_payload, 0);                  // 8 is fixed value according to Spec.
            ++sub_step;
            
						*before_delay_ms = 1;
						
						if(sub_step>=8){
							  sub_step = 0; 
						    *step_complete_f = 1;
						}
            break;
        case AFE_INIT_WAIT_ID_RESPONSE:
					  sub_step = 1;
						*before_delay_ms = 0;
						*step_complete_f = 1;
            break;
        case AFE_INIT_BRORAD_SET_BMU_STACK2:
					  fill_data4_payload(d_payload, BQ79600_STACK,0,0,0);
            drv_bq796xx_command_framing(BROAD_WRITE, 0x00, BQ796XX_COMM_CTRL, 1, d_payload, 0);                   // Set everything as a stack device first.
            *before_delay_ms = 1;
            *step_complete_f = 1;			

            retry_cnt = 0;				
            break;		
				case AFE_INIT_CHECK_ID_RESPONSE:			
            //--------------------------------------------------------------	
            *before_delay_ms = 1;                                                                                 //Msut have, avoid device can not recv command.				
						if(rcv_check_step == 0){
						    bmu_cnt = sub_step;
                drv_bq796xx_clear_fifobuffer();
						    drv_bq796xx_command_framing(SINGLE_READ,  bmu_cnt, dir_addr, 1, null_payload, 0);                 // 10000 is base on TI's log file
							  rcv_check_step = 1;
						}else{
						    res = drv_bq796xx_check_respone_event2();
							  
                if (res == BQ796XX_RES_TIMEOUT){  								                                                // no return data => no this ID    							      
									  ++retry_cnt;
									  if(retry_cnt <= BQ796XX_CHECK_ID_CMD_RETRY){
                        rcv_check_step = 0;                                                                       //Retry this ID is exist?											
										}else{
											  retry_cnt = 0;
									      sub_step = top_no + 1;                                                                    //When device no resopne, then break check device ID
									      if(bmu_cnt>0){ 
										        --bmu_cnt;
									      }
									}
                }else if(res == BQ796XX_RES_RCV_DAT){
								    rcv_check_step = 0;     
									  ++sub_step;             //Next device check
								}
						}
						//--------------------------------------------------------------
						
						if(sub_step > top_no){
							  rcv_check_step = 0;
							  sub_step = 0;
						    *before_delay_ms = 1;
						    *step_complete_f = 1;
						}
				    break;
        case AFE_INIT_SET_STACK:
            fill_data4_payload(d_payload,BQ79600_STACK,0,0,0);
            drv_bq796xx_command_framing(BROAD_WRITE,   0, BQ79600_COMM_CTRL, 1, d_payload, 0);

						*before_delay_ms = 1;
						*step_complete_f = 1;				
            break;
        case AFE_INIT_SET_BASE:
            fill_data4_payload(d_payload,BQ79600_BASE,0,0,0);
            drv_bq796xx_command_framing(SINGLE_WRITE,  0x00, BQ79600_COMM_CTRL, 1, d_payload, 0); 
				
						*before_delay_ms = 1;
						*step_complete_f = 1;	
            break;
        case AFE_INIT_SET_TOP:
            if (bmu_cnt > maxcnt ){
                bmu_cnt = maxcnt;
            }
						
						if(bmu_cnt>=1){
						    fill_data4_payload(d_payload,BQ79600_TOP,0,0,0);
                drv_bq796xx_command_framing(SINGLE_WRITE,  bmu_cnt, BQ79600_COMM_CTRL, 1, d_payload, 0); 
						}
						*before_delay_ms = 1;
						*step_complete_f = 1;
            rcv_check_step = 0;						
						sub_step = 0;
            break;
				case AFE_INIT_CHECK_IS_RING:			
            //--------------------------------------------------------------	
            *before_delay_ms = 1;                                                                                   //Msut have, avoid device can not recv command.				
						if(rcv_check_step == 0){
   			        fill_data4_payload(d_payload,0,0,0,0);
                drv_bq796xx_command_framing(SINGLE_READ,  bmu_cnt, BQ79600_ACTIVE_CELL, 1, d_payload, 0);            // 10000 is base on TI's log file
							  rcv_check_step = 1;
						}else{
						    res = drv_bq796xx_data_frame_parser();
							  
                ++sub_step;							                   		  
                if(res == BQ796XX_RES_OK){
								    ++sub_step;
									  if ( dir == DIR_NORTH ){
										   if(bq796xx_data.ns_flag == 0x40){    //South dir
											     is_ring = 0x80;
											 }
										}else{
										   if(bq796xx_data.ns_flag == 0x80){    //North dir
											     is_ring = 0x80;
											 }											
										}
										
										bmu_cnt |= is_ring;
								}
						}
						//--------------------------------------------------------------
						
						if(sub_step > 0){
							  rcv_check_step = 0;
							  sub_step = 0;
						    *before_delay_ms = 1;
						    *step_complete_f = 1;
						}
				    break;			
        case AFE_INIT_SET_DIRECTION_FLAG_ACTIVE_CELL:						
						if(bmu_cnt>= 0x01){
							  if(dir == DIR_NORTH )
								    fill_data4_payload(d_payload,(BMU_CELL_SERIES-6) | 0x80,0,0,0);
								else
									  fill_data4_payload(d_payload,(BMU_CELL_SERIES-6) | 0x40,0,0,0);
								drv_bq796xx_command_framing(STACK_WRITE, 0 , BQ79600_ACTIVE_CELL, 1, d_payload, 0);
						}
						
						*before_delay_ms = 1;
						*step_complete_f = 1;
						rcv_check_step = 0;
						sub_step = 0;
            break;	
        case AFE_INIT_FAULT_RESET_1:
            fill_data4_payload(d_payload,0xFF,0,0,0);
            drv_bq796xx_command_framing(BROAD_WRITE,   0, BQ79600_FAULT_RST1, 1, d_payload, 0); 
						*before_delay_ms = 1;
						*step_complete_f = 1;	
            break;
        case AFE_INIT_FAULT_RESET_2:
            fill_data4_payload(d_payload,0xFF,0,0,0);
				    drv_bq796xx_command_framing(BROAD_WRITE,   0, BQ79600_FAULT_RST2, 1, d_payload, 0); 
						*before_delay_ms = 1;
						*step_complete_f = 1;
            break;
        case AFE_INIT_FAULT_RESET:
            fill_data4_payload(d_payload,BQ79600_RST_FCOMM_DET | BQ79600_RST_SYS,0,0,0);
				    drv_bq796xx_command_framing(SINGLE_WRITE,  0x00, BQ79600_FAULT_RST, 1,d_payload , 0);
						*before_delay_ms = 1;
						*step_complete_f = 1;
            break;
        case AFE_INIT_OTP_SPARE:
            drv_bq796xx_command_framing(STACK_WRITE,   0, BQ79600_OTP_SPARE13, 1, null_payload, 0);
						*before_delay_ms = 1;
						*step_complete_f = 1;
            break;
        case AFE_INIT_BQ79600_TIMEOUT:
            drv_bq796xx_command_framing(SINGLE_WRITE,  0x00, BQ79600_COMM_TIMEOUT, 1, null_payload, 0);
						*before_delay_ms = 1;
						*step_complete_f = 1;
            break;
        case AFE_INIT_SWITCH_GPIO:
						*before_delay_ms = 1;
						*step_complete_f = 1;
            break;
        case AFE_INIT_TSREF_ENABLE:
            fill_data4_payload(d_payload,BQ79600_TSREF_EN,0,0,0);
            drv_bq796xx_command_framing(STACK_WRITE,   0, BQ79600_CONTROL2, 1, d_payload, 0);
						*before_delay_ms = 1;
						*step_complete_f = 1;
     				break;
        case AFE_INIT_ADC_CONF1:
            fill_data4_payload(d_payload,0x03,0,0,0);
            drv_bq796xx_command_framing(BROAD_WRITE,   0, BQ79600_ADC_CONF1, 1, d_payload, 0);
						*before_delay_ms = 1;
						*step_complete_f = 1;
            break;
        case AFE_INIT_ADC_CTRL1:
            fill_data4_payload(d_payload,0x1E,0,0,0);
            drv_bq796xx_command_framing(STACK_WRITE,   0, BQ79600_ADC_CTRL1, 1, d_payload, 0);                                // [LPF BB EN]=1, Cell filter([3]=1), start ADC([2]=1), continue run([1:0]=2)

						*before_delay_ms = 38+5*bmu_cnt;
						*step_complete_f = 1;				
				    break;
        case AFE_INIT_BQ796XX_TIMEOUT:               //BMU deivice CTL_ACT =1 is sends the device to SHUTDOWN. FAULT_SYS[CTL] bit will not be set, CTL_TIME = 101 (10 mintues long communcation) 
            fill_data4_payload(d_payload,0x0D,0,0,0);   
				    drv_bq796xx_command_framing(STACK_WRITE,   0, BQ79600_COMM_TIMEOUT_CONF, 1, d_payload, 0);
						*before_delay_ms = 1;
						*step_complete_f = 1;				
            break;
       case AFE_INIT_MASK_FAULT_ALL:      //Broad cast Mask Fault all + Clear fault rst all
            drv_bq796xx_Set_BroadCast_Mask_FaultAll(0);
			      
			      *before_delay_ms = 20;
						*step_complete_f = 1;				 
			      sub_step=0;
            break;
			 case AFE_INIT_BRIDGE_FAULT_MSK:
  					fill_data4_payload(d_payload, (BQ79600_MSK_SYS | BQ79600_MSK_REG| BQ79600_MSK_FTONE_DET | BQ79600_MSK_UART_SPI | BQ79600_MSK_COML_H | BQ79600_MSK_HB | BQ79600_MSK_PWR), 0, 0, 0);
						drv_bq796xx_command_framing(SINGLE_WRITE, 0x00, BQ79600_FAULT_MSK, 1, d_payload, 0);

				    *before_delay_ms = 1;
				    *step_complete_f = 1;		
            sub_step=0;			 
       case AFE_INIT_CLEAR_RST_ALL:
			       *before_delay_ms = 20; 
			 
			      fill_data4_payload(d_payload,0xFF,0xFF,0,0); //Clear all=x0FF
	          switch(sub_step){
						    case 0:
			              drv_bq796xx_command_framing(STACK_WRITE, 0x00, BQ79600_FAULT_RST1, 2, d_payload, 0);
                    break;
								case 1:
									  drv_bq796xx_command_framing(SINGLE_WRITE, 0x00, BQ79600_FAULT_RST, 1, d_payload, 0);
								    break;
						}
			      
	          bq796xx_afe_fault_rst1= 0xFF;
	          
			      ++sub_step;
						
						if(sub_step>=2){
						    *before_delay_ms = 1;
						    *step_complete_f = 1;		
                sub_step=0;			 
						}
            break;			
       case AFE_INIT_SET_RUN_OVUV_FUNC:   //STACK Write, first set mask fault all=> clear mask fault OVUV(enable)=> use OVUV thresh + start OVUV function
				    *before_delay_ms = 1; 
			 
	          switch(sub_step){
						    case 0:
								    drv_bq796xx_Set_Mask_FaultAll(STACK, 0, 0);
							      break;
								case 1:
                    drv_bq796xx_Clear_Mask_Fault_OVUV(STACK, 0,0);
                    break;
								case 2:
								    drv_bq796xx_Set_Stack_OV(bq796xx_default.ov_threshold, bq796xx_default.ov_step_threshold,0);
                    break;
								case 3:
								    drv_bq796xx_Set_Stack_UV(bq796xx_default.uv_threshold,0);	
                    break;
								case 4:								
						    		drv_bq796xx_Run_OVUV(0);
								    break;
						}
						
						++sub_step;
						
						if(sub_step>=5){
						    sub_step = 0;
						    *before_delay_ms = 1;
						    *step_complete_f = 1;
						}				 
            break;			 
       case AFE_INIT_SET_RUN_OTUT_FUNC:	//STACK Write, use OTUT thresh + start OTUT
				    *before_delay_ms = 1; 
			 
	          switch(sub_step){				 
							case 0:
							    drv_bq796xx_Clear_Mask_Fault_OTUT(STACK, 0,0);
	                break;
							case 1:
							    drv_bq796xx_Set_Stack_OTUT(bq796xx_default.ot_threshold, bq796xx_default.ut_threshold,0);
	                break;
							case 2:
							    drv_bq796xx_Set_Stack_OTUT_Associate(AFE_GPIO1,0);
	                break;
							case 3:
							    drv_bq796xx_Run_OTUT(0);
							    break;
						}

						++sub_step;
						
					  if(sub_step>=4){
						    sub_step = 0;
						    *before_delay_ms = 1;
						    *step_complete_f = 1;
							  cell_cnt = 0;
					 }						
		       break;				
			case AFE_INIT_SET_CELL_BALANCE_FUNC_DISABLE:  //Disable Cell balance
				    *before_delay_ms = 1; 
			 
	          switch(sub_step){				 
						    case 0:				
                    fill_data4_payload(d_payload, 0,0,0,0); 
                    drv_bq796xx_command_framing(STACK_WRITE, 0x00, BQ796XX_CB_CELL01_CTRL-cell_cnt,1, d_payload, 0);
							      ++cell_cnt;
  
								    if(cell_cnt >= BMU_CELL_SERIES){
								        ++sub_step;   
								    }
							
							      break;
							case 1:
	                  drv_bq796xx_Set_Stack_CellBalanceCycleTime(bq796xx_default.cb_cycle_time, 0);
							      ++sub_step;
							      break;
							case 2:
                    fill_data4_payload(d_payload,((FCOMM_EN | NFAULT_EN) &(~(NO_ADJ_CB))), 0, 0, 0);   //All BMU Device will allow two adjacent CB FETs to be enabled
                    drv_bq796xx_command_framing(STACK_WRITE, 0x00, BQ796XX_DEV_CONF,1, d_payload, 0);							      
							      ++sub_step;
							      break;				 
						}
						
					  if(sub_step>=3){
						    sub_step = 0;
						    *before_delay_ms = 1;
						    *step_complete_f = 1;
					   }						
					   break;
		  case AFE_INIT_SET_GPIO_FUNC:	//Setting ALL BMU ,GPIO1~GPIO3 ADC input.   GPIO4~GPIO8 Output Hi/Lo.	 
				    *before_delay_ms = 1; 
			 
	          switch(sub_step){				 
							case 0:	
				          drv_bq796xx_Set_AFE_GPIO_type(STACK,0,GPIO_ADC_OTUT,AFE_GPIO1,0);
	                break;
							case 1:
								  drv_bq796xx_Set_AFE_GPIO_type(STACK,0,GPIO_ADC_OTUT,AFE_GPIO2,0);
	                break;
							case 2:
								  drv_bq796xx_Set_AFE_GPIO_type(STACK,0,GPIO_OUT_L,AFE_GPIO3,0);	
	                break;
							case 3:
	                drv_bq796xx_Set_AFE_GPIO_type(STACK,0,GPIO_OUT_L,AFE_GPIO4,0);
	                break;
							case 4:
							    drv_bq796xx_Set_AFE_GPIO_type(STACK,0,GPIO_OUT_L,AFE_GPIO5,0);
	                break;
							case 5:
							    drv_bq796xx_Set_AFE_GPIO_type(STACK,0,GPIO_OUT_L,AFE_GPIO6,0);
 	                break;
							case 6:
							    drv_bq796xx_Set_AFE_GPIO_type(STACK,0,GPIO_OUT_L,AFE_GPIO7,0);
	                break;
							case 7:
							    drv_bq796xx_Set_AFE_GPIO_type(STACK,0,GPIO_OUT_H,AFE_GPIO8,0);
							    break;
						}
						
						++sub_step;
						
					  if(sub_step>=8){
						    sub_step = 0;
						    *before_delay_ms = 1;
						    *step_complete_f = 1;
					 }						
					 break;			
				case AFE_INIT_SET_BMU_FAULT_MSK:
					
				   set_val = 0xFF;
				   if(bq796xx_default.ov_enable== 1){
				      set_val &= ~(BQ79616_MSK_OV);   
				   }
				   
					 if(bq796xx_default.uv_enable== 1){
				      set_val &= ~(BQ79616_MSK_UV);   
				   }
					 
				   if(bq796xx_default.ot_enable== 1){
				      set_val &= ~(BQ79616_MSK_OT);   
				   }				
					 
					 if(bq796xx_default.ut_enable== 1){
				      set_val &= ~(BQ79616_MSK_UT);   
				   }
					 
					 drv_bq796xx_Set_BroadCast_Mask_FaultSel(set_val,0xFF,0);
				   sub_step = 0;
					 *before_delay_ms = 1;
					 *step_complete_f = 1;            
				   break;				
				case AFE_RUN_AUX_ADC:	 //Start Main and AUX ADC 
					 drv_bq796xx_Start_AFE_ADC(STACK,0,0);
				
    			 sub_step = 0;
					 *before_delay_ms = 1;
					 *step_complete_f = 1;
				   break;
		}
	  
		return bmu_cnt;
}

int8_t drv_bq796xx_init_default_load(bq796xx_init_default_t in_load_data){
     
	   if(&(in_load_data)== 0) return SMP_ERROR_NULL;
         
	   bq796xx_default = in_load_data;
 return SMP_SUCCESS;
}

uint8_t drv_bq796xx_CellBalance_1to8_set(uint8_t bmu_id, uint8_t cell_d_en, bq796xx_cellbalance_time cb_time, uint32_t delays){
	  uint8_t i;
    static uint8_t mask_en;
    static uint8_t payload_temp[8] = {0};

	  if(bq796xx_data.comm_dir == DIR_SOUTH){
	    bmu_id = (bq796xx_default.bmu_total_num+1) - bmu_id;
	  } 		
		
		for(i=0; i<8; i++){
			  mask_en = (cell_d_en &(0x01 << i));
		    if(mask_en == 0x00){
				    payload_temp[7-i] = 0;
				}else{
				    payload_temp[7-i] = cb_time;
				}
		}
		
	  drv_bq796xx_command_framing(SINGLE_WRITE, bmu_id, BQ796XX_CB_CELL08_CTRL, 8, payload_temp, delays);
	
    return 0;
}

uint8_t drv_bq796xx_CellBalance_9to16_set(uint8_t bmu_id, uint8_t cell_d_en, bq796xx_cellbalance_time cb_time, uint32_t delays){
	  uint8_t i;
    static uint8_t mask_en;
    static uint8_t payload_temp[8] = {0};

	  if(bq796xx_data.comm_dir == DIR_SOUTH){
	    bmu_id = (bq796xx_default.bmu_total_num+1) - bmu_id;
	  }		
		
		for(i=0; i<8; i++){
			  mask_en = (cell_d_en &(0x01 << i));
		    if(mask_en == 0x00){
				    payload_temp[7-i] = 0;
				}else{
				    payload_temp[7-i] = cb_time;
				}
		}
		
	  drv_bq796xx_command_framing(SINGLE_WRITE, bmu_id, BQ796XX_CB_CELL16_CTRL, 8, payload_temp, delays);
	
    return 0;
}

uint8_t drv_bq796xx_CellBalance_1to8_clear(bq796xx_AFE_GPIO_stack is_stack, uint8_t dev_id, uint32_t delays){
  static uint8_t    d_payload[8] = {0};  

	if(bq796xx_data.comm_dir == DIR_SOUTH){
	    dev_id = (bq796xx_default.bmu_total_num+1) - dev_id;
	}	
	
	fill_data4_payload(d_payload, 0,0,0,0);
	fill_data4_payload(&d_payload[4], 0,0,0,0);
	
	if(is_stack == STACK)
      drv_bq796xx_command_framing(STACK_WRITE, 0x00, BQ796XX_CB_CELL08_CTRL, 8, d_payload, delays);
  else
		  drv_bq796xx_command_framing(SINGLE_WRITE, dev_id, BQ796XX_CB_CELL08_CTRL, 8, d_payload, delays);
	
	return 0;
}

uint8_t drv_bq796xx_CellBalance_9to16_clear(bq796xx_AFE_GPIO_stack is_stack, uint8_t dev_id, uint32_t delays){
  static uint8_t    d_payload[8] = {0};  

	if(bq796xx_data.comm_dir == DIR_SOUTH){
	    dev_id = (bq796xx_default.bmu_total_num+1) - dev_id;
	}	
	
	fill_data4_payload(d_payload, 0,0,0,0);
	fill_data4_payload(&d_payload[4], 0,0,0,0);
  
	if(is_stack == STACK)
      drv_bq796xx_command_framing(STACK_WRITE, 0x00, BQ796XX_CB_CELL16_CTRL, 8, d_payload, delays);
  else
		  drv_bq796xx_command_framing(SINGLE_WRITE, dev_id, BQ796XX_CB_CELL16_CTRL, 8, d_payload, delays);
  
	return 0;
}

#if 0
#define TEST_BMU_AUTO_ID
#endif

#ifdef  TEST_BMU_AUTO_ID
uint8_t test_afe_auto_id[5]={0,5,4,3,2};
#endif

//------------------------------------------------------------------------------------------
// Description : Set north/south direction to communicate with AFE
//   Version   : 1
//    Date     : 2021/12/02
// Written By  : Golden Chen
//  Parameter  : 
//               ns_dir_bmu_cnt             = 0 is swtich North or South direction, and auto addressing , check BMU ID
//                                          > 0 is swtich North or South direction, and check BMU ID=(ns_dir_bmu_cnt) is exist, if exist, BMU ID=(ns_dir_bmu_cnt) as TOP.
//               *afe_phase (by address)    = set steps.
//               maxcnt                     = System MAX BMU number.
//               dir                        = North or South direction.
//               step_complete_f(by address)= is complete this sub step action.
//               before_delay_ms(by address)= next sub step delay time. 
//   Return    : uint8_t  BMU count
//------------------------------------------------------------------------------------------
uint8_t drv_bq796xx_direction_set_steps(uint8_t ns_dir_bmu_cnt,bq796xx_dir_set_steps_enum *afe_phase,uint8_t maxcnt, uint8_t dir, uint8_t *step_complete_f, uint8_t *before_delay_ms){
    uint8_t    d_payload[4] = {0};		
    uint8_t    null_payload[4] = {0};
    
		static uint8_t    bmu_cnt = 0, dir_sel;
    static uint16_t   dir_addr;
    static uint8_t    top_no = 0;
    static uint8_t    sub_step=0;
		static uint8_t    rcv_check_step = 0;
		static uint8_t    is_ring=0;
		
		#ifdef  TEST_BMU_AUTO_ID
    static uint8_t    test_id_p_c=0;		
		#endif
		
    uint8_t res=0;
		
		top_no = maxcnt;
		
		*step_complete_f = 0;
		*before_delay_ms = 0;
    is_ring =0;
		
    switch(*afe_phase){
        case SETDIR_BRORAD_SET_BMU_STACK1:                                                                             // Before use "Broadcast Write Revese" cmd, you must be have to clear TOP stack role device BMU.
				    fill_data4_payload(d_payload, BQ79600_STACK,0,0,0);
            drv_bq796xx_command_framing(BROAD_WRITE, 0x00, BQ796XX_COMM_CTRL, 1, d_payload, 0);                        // Set everything as a stack device first.
            *before_delay_ms = 1;
            *step_complete_f = 1;

            break;		
        case SETDIR_SET_BASE1:
            fill_data4_payload(d_payload,BQ79600_BASE,0,0,0);
            drv_bq796xx_command_framing(SINGLE_WRITE,  0x00, BQ79600_COMM_CTRL, 1, d_payload, 0); 
				
						*before_delay_ms = 1;
						*step_complete_f = 1;	
            break;				
        case SETDIR_BASE_DIR_CHG:         
   			    if ( dir == DIR_NORTH ){
                dir_addr = BQ79600_DIR0_ADDR;
                dir_sel  = DIR_NORTH;							  
            }else{                                                                                               // Need not to wakeup, just jump to wakeup device(Setting SOUTH direction)
                dir_addr = BQ79600_DIR1_ADDR;
                dir_sel = (DIR_SOUTH*BQ79616_DIR_SEL);
            }

            fill_data4_payload(d_payload, dir_sel,0,0,0);						
				    drv_bq796xx_command_framing(SINGLE_WRITE, 0x00, BQ79600_CONTROL1, 1, d_payload, 0);

            *before_delay_ms = 2;
						*step_complete_f = 1;						
						
            break;
        case SETDIR_BRORAD_REVERSE_DIRECTION:				
            fill_data4_payload(d_payload, dir_sel,0,0,0);                                                //Must to do this North direction=BWR 0x00, South direction=BWR 0x80 
            drv_bq796xx_command_framing(BROAD_WRITE_REVERSE, 0, BQ79600_CONTROL1, 1, d_payload, 0);      //If a device receives communication frame opposite to the [DIR_SEL] setting, change direction for all devices	
		
						bq796xx_data.comm_dir = dir;
											
            *before_delay_ms = 2;
            *step_complete_f = 1;
				
            if(ns_dir_bmu_cnt > 0){
						    *afe_phase = SETDIR_INIT_DUMMY_READ;     //next phase jump to this.
							  rcv_check_step = 0;
							  sub_step = ns_dir_bmu_cnt;               //direct TOP stack is exist.
						}
				
            break;		
        case SETDIR_BRORAD_OTP_ECC_TEST_W:
					  fill_data4_payload(d_payload, 0x00,0,0,0);
            drv_bq796xx_command_framing(BROAD_WRITE, 0x00, BQ79600_OTP_ECC_TEST, 1, d_payload, 0);                 // Dummy write to snchronize all daisy chain device DLL.
            *before_delay_ms = 2;
            *step_complete_f = 1;			
            sub_step = 0;				
            break;		
        case SETDIR_INIT_DUMMY_WRITE:					                                                                                                                        
            drv_bq796xx_command_framing(BROAD_WRITE, 0, BQ79600_OTP_ECC_DATAIN1+sub_step, 1, null_payload, 0);     // 8 is fixed value according to Spec. 
            ++sub_step;
						
						*before_delay_ms = 1;
						
						if(sub_step>=8){
							  sub_step = 0; 
						    *step_complete_f = 1;
						}
            break;				
        case SETDIR_AUTO_ADDR:
		        fill_data4_payload(d_payload,dir_sel + ADDR_WR,0,0,0);						
				    drv_bq796xx_command_framing(BROAD_WRITE, 0x00, BQ79600_CONTROL1, 1, d_payload, 0);          // enable auto addressing(0x01)
            *before_delay_ms = 1;
            *step_complete_f = 1;
            sub_step = 0; 				
            break;
        case SETDIR_SET_ID:                                                                                                     
            
				    #ifdef TEST_BMU_AUTO_ID
				    if((dir == DIR_NORTH) && (test_id_p_c>=2)){
				        if(sub_step<5)     
				            fill_data4_payload(d_payload,test_afe_auto_id[sub_step],0,0,0);
				        else
							      fill_data4_payload(d_payload, (8+sub_step), 0, 0, 0);
					  }else{
     				    fill_data4_payload(d_payload,sub_step,0,0,0);                                 						// 6 bits for address ID
						}
				    #else
				    fill_data4_payload(d_payload,sub_step,0,0,0);                                 						// 6 bits for address ID
				    #endif
						drv_bq796xx_command_framing(BROAD_WRITE, 0, dir_addr, 1, d_payload, 0);                     // 3000 is base on TI's log file
								
				    ++sub_step;				
						*before_delay_ms = 1;
						
						if(sub_step>top_no){
							  #ifdef TEST_BMU_AUTO_ID
							      test_id_p_c++;
							  #endif  
							
							  sub_step = 1; 
						    *step_complete_f = 1;
						}						
            break;
        case SETDIR_INIT_DUMMY_READ:                                                                                                    
            drv_bq796xx_command_framing(BROAD_READ, 0, BQ79600_OTP_ECC_DATAIN1+sub_step, 1, null_payload, 0);                  // 8 is fixed value according to Spec.
            ++sub_step;
            
						*before_delay_ms = 2;
						
						if(sub_step>=8){
							  sub_step = 1; 
						    *step_complete_f = 1;
						}
            break;
        case SETDIR_BRORAD_SET_BMU_STACK2:
					  fill_data4_payload(d_payload, BQ79600_STACK,0,0,0);
            drv_bq796xx_command_framing(BROAD_WRITE, 0x00, BQ796XX_COMM_CTRL, 1, d_payload, 0);                   // Set everything as a stack device first.
            *before_delay_ms = 1;
            *step_complete_f = 1;						
            break;												
				case SETDIR_CHECK_ID_RESPONSE:			
            //--------------------------------------------------------------	
            *before_delay_ms = 1;                                                                                   //Msut have, avoid device can not recv command.				
						if(rcv_check_step == 0){
						    bmu_cnt = sub_step; 
						    drv_bq796xx_command_framing(SINGLE_READ,  bmu_cnt, dir_addr, 1, null_payload, 0);                  // 10000 is base on TI's log file
							  rcv_check_step = 1;
						}else{
						    res = drv_bq796xx_check_respone_event2();
							  
                if (res == BQ796XX_RES_TIMEOUT){                                                                   // no return data => no this ID    
							      sub_step = top_no + 1;  //When device no resopne, then break check device ID
									  
									  if(ns_dir_bmu_cnt > 0){
									      bmu_cnt = 0;
									  }else{
									      if(bmu_cnt>0){ 
											    --bmu_cnt;
									      }
									 }
                }else if(res == BQ796XX_RES_RCV_DAT){
									  if(ns_dir_bmu_cnt == 0){
								        rcv_check_step = 0;     
									      ++sub_step;             //Next device check
										}else{
										    sub_step = top_no + 1;  //When device no resopne, then break check device ID
										}	
							}
						}
						//--------------------------------------------------------------
						
						if(sub_step > top_no){
							  rcv_check_step = 0;
							  sub_step = 0;
						    *before_delay_ms = 1;
						    *step_complete_f = 1;
						}
				    break;							
        case SETDIR_SET_STACK:
					  fill_data4_payload(d_payload,BQ79600_STACK,0,0,0);
				    drv_bq796xx_command_framing(BROAD_WRITE, 0x00, BQ79600_COMM_CTRL, 1, d_payload, 0);  // write stack to BMUs
				
						*before_delay_ms = 1;
						*step_complete_f = 1;
				    break;
        case SETDIR_SET_BASE2:
            fill_data4_payload(d_payload,BQ79600_BASE,0,0,0);
            drv_bq796xx_command_framing(SINGLE_WRITE,  0x00, BQ79600_COMM_CTRL, 1, d_payload, 0); 
				
						*before_delay_ms = 1;
						*step_complete_f = 1;	
            break;				
        case SETDIR_SET_TOP:
            if (bmu_cnt > maxcnt ){
                bmu_cnt = maxcnt;
            }
						
						if(bmu_cnt>= 0x01){
								fill_data4_payload(d_payload,BQ79600_TOP,0,0,0);
							  #ifdef TEST_BMU_AUTO_ID
							  if((dir == DIR_NORTH) && (test_id_p_c>=3)){
									  drv_bq796xx_command_framing(SINGLE_WRITE,  test_afe_auto_id[4], BQ79600_COMM_CTRL, 1, d_payload, 0);
								}else{
                    drv_bq796xx_command_framing(SINGLE_WRITE,  bmu_cnt, BQ79600_COMM_CTRL, 1, d_payload, 0);
								}									
							  #else	
								drv_bq796xx_command_framing(SINGLE_WRITE,  bmu_cnt, BQ79600_COMM_CTRL, 1, d_payload, 0);
								#endif
						}
						
						*before_delay_ms = 1;
						*step_complete_f = 1;
						rcv_check_step = 0;
						sub_step = 0;
            break;
				case SETDIR_CHECK_IS_RING:			
            //--------------------------------------------------------------	
            *before_delay_ms = 1;                                                                                   //Msut have, avoid device can not recv command.				
						if(rcv_check_step == 0){
   			        fill_data4_payload(d_payload,0,0,0,0);
                drv_bq796xx_command_framing(SINGLE_READ,  bmu_cnt, BQ79600_ACTIVE_CELL, 1, d_payload, 0);            // 10000 is base on TI's log file
							  rcv_check_step = 1;
						}else{
						    res = drv_bq796xx_data_frame_parser();
							
                ++sub_step;							
                if(res == BQ796XX_RES_OK){

									  if ( dir == DIR_NORTH ){
										   if(bq796xx_data.ns_flag == 0x40){    //South dir
											     is_ring = 0x80;
											 }
										}else{
										   if(bq796xx_data.ns_flag == 0x80){    //North dir
											     is_ring = 0x80;
											 }											
										}
										
										bmu_cnt |= is_ring;
								}
						}
						//--------------------------------------------------------------
						
						if(sub_step > 0){
							  rcv_check_step = 0;
							  sub_step = 0;
						    *before_delay_ms = 1;
						    *step_complete_f = 1;
						}
				    break;			
        case SETDIR_SET_DIRECTION_FLAG_ACTIVE_CELL:						
						if(bmu_cnt>= 0x01){
							  if(dir == DIR_NORTH )
								    fill_data4_payload(d_payload,(BMU_CELL_SERIES-6) | 0x80,0,0,0);
								else
									  fill_data4_payload(d_payload,(BMU_CELL_SERIES-6) | 0x40,0,0,0);
								drv_bq796xx_command_framing(STACK_WRITE, 0 , BQ79600_ACTIVE_CELL, 1, d_payload, 0);
						}
						
						*before_delay_ms = 1;
						*step_complete_f = 1;
						rcv_check_step = 0;
						sub_step = 0;
            break;						
        case SETDIR_FAULT_RESET_2:
            fill_data4_payload(d_payload,0xFF,0,0,0);
				    drv_bq796xx_command_framing(BROAD_WRITE,   0, BQ79600_FAULT_RST2, 1, d_payload, 0); 
						*before_delay_ms = 1;
						*step_complete_f = 1;
            break;				
       case SETDIR_CLEAR_RST_ALL:
			       *before_delay_ms = 2; 
			 
			      fill_data4_payload(d_payload,0xFF,0xFF,0,0); //Clear all=x0FF
	          switch(sub_step){
						    case 0:
			              drv_bq796xx_command_framing(STACK_WRITE, 0x00, BQ79600_FAULT_RST1, 2, d_payload, 0);
                    break;
								case 1:
									  drv_bq796xx_command_framing(SINGLE_WRITE, 0x00, BQ79600_FAULT_RST, 1, d_payload, 0);
								    break;
						}
			      
	          bq796xx_afe_fault_rst1= 0xFF;
	          
			      ++sub_step;
						
						if(sub_step>=2){
						    *before_delay_ms = 1;
						    *step_complete_f = 1;		
                sub_step=0;			 
						}
            break;
				case SETDIR_AFE_RUN_AUX_ADC:	 //Start Main and AUX ADC 
					 drv_bq796xx_Start_AFE_ADC(STACK,0,0);
				
    			 sub_step = 0;
					 *before_delay_ms = 1;
					 *step_complete_f = 1;
				   break;											
        default:
					  return(0);
    }
    return bmu_cnt;
	
}

extern smp_fifo_t uart0_rx_fifo;
extern int8_t smp_fifo_clean(smp_fifo_t *p_fifo);

uint8_t drv_bq796xx_clean_fifo(void){

	drv_bq796xx_clear_fifobuffer();
	//smp_fifo_clean(&uart0_rx_fifo);
	//bq796xx_res_buf_c = 0;
}

/************************ (C) COPYRIGHT Simplo all right reserved *****END OF FILE****/


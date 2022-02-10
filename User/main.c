/**
  ******************************************************************************
  * @file        main.c
  * @author      Golden 
  * @version     v0.0.6
  * @date        2022/02/08
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
#include "define.h"
#include "main.h"
#include "AppProject.h"
#include "LibSwTimer.h"
#include "HalBsp.h"
#include "ApiFu.h"

#include "smp_drv_bq796xx.h"
#include "smp_adc.h"
#include "smp_ADS7946_Driver.h"
#include "ApiIRMonitoring.h"
#include "HalAfeADS7946.h"

#include "ApiModbusTCPIP.h"
#include "AppProjectTest.h"
#include "smp_MX25L_Driver.h"
#include "smp_max7219.h"
#include "smp_TLC6C5912.h"
#include "smp_log_managment.h"
#include "SEGGER_RTT.h"
#include "RTT_Log.h"

#if 1
#define G_TEST_INT_ADC
#endif

#if 0
#define G_TEST_MX25LXX_FLASH 
#endif

#if 1
#define G_TEST_TLC6C5912_FUNC
#endif

#if 1
#define G_TEST_LED_DISPLAY_BUTTON
#endif 

#if 0
#define G_TEST_MAX7219_LCD_MARITEX
#endif

#if 1
#define G_TEST_AD7946_ADC
#define G_TEST_INT_ADC_TEST_CYCLE_NUM           50
#endif

#if 1
#define G_TEST_BQ796XX_SETTING_INIT_WITHOUT_STEP
#endif 

#if 1
#define G_TEST_BQ796XX_SETTING_INIT_WITH_STEP
#define G_TEST_BQ796XX_SETTING_INIT_WITH_STEP_TEST_CYCLE_NUM                   10
#define G_TEST_BQ796XX_SETTING_INIT_WITH_STEP_WAKE_CNT                         10
#endif

#if 1
#define G_TEST_BQ796XX_DIRECTION_CHECK_BMU_WITH_STEP
#define G_TEST_BQ796XX_DIRECTION_CHECK_BMU_WITH_STEP_TEST_CYCLE_NUM            10
#endif

#if 1
#define G_TEST_BQ796XX_OVUVOTUT_FUNC
#define G_TEST_BQ796XX_OVUVOTUT_FUNC_TEST_CYCLE_NUM                            10
#endif

#if 1
#define G_TEST_BQ796XX_CELL_BALANCE_FUNC 
#define G_TEST_BQ796XX_CELL_BALANCE_FUNC_TEST_CYCLE_NUM                        100
#endif

#if 1
#define G_TEST_BQ796XX_GPIO_SELECT_READ_ADC_FUNC
#define G_TEST_BQ796XX_GPIO_SELECT_READ_ADC_FUNC_TEST_CYCLE_NUM                5
#endif

#if 1
#define G_TEST_IRM_FUNCTION
#define G_TEST_IRM_FUNCTION_TEST_CYCLE_NUM                                     50
#define G_TEST_IRM_FUNCTION_TEST_CYCLE_DELAY_TIME                              100
#endif 

#if 1
#define G_TEST_EVENT_LOG_FUNC
#endif

extern uint8_t app_afe_cb(bq796xx_data_t *bq_data, bq796xx_event_cb_type bq_event);

extern void ads7946_callBack(uint8_t *pDat, uint8_t size);

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

uint16_t app_adc_temp[5]={0};

long int test_bmu_statistics_num[2][BMU_TOTAL_BOARDS+1] ={0};
long int test_bmu_ring_total_count = 0;
//long int test_dir_fail[2] ={0};
long int irm_get_vstack_cont = 0;
uint16_t  test_init_rec_bmu_num[2][40]={0};

uint16_t test_get_vatck_cnt = 0;

#ifdef G_TEST_BQ796XX_GPIO_SELECT_READ_ADC_FUNC
uint16_t test_gpio_HL=0;
uint8_t	test_ntcIoIndex =0;
uint8_t	test_ntcChannelStartIndex = 0;
uint8_t	test_ntcChannelOffset = 0;
uint8_t	test_NtcChannelSelect = 0;
#endif 

LED_Display_Indicator_Type led_status_temp;
uint32_t test_led_display_cnt=0; 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LED_TOGGLE_DELAY                       100
																				
/* UNCOMMENT THE FOLLOWING LINE FOR FAST TRANSITION TOWARDS SMPS ACTIVATION */
#define NO_DELAY_TOWARDS_SMPS                  1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;
static __IO uint32_t button_pressed = 0;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config_80MHz(void);
void SystemClock_Config_HSE_80MHz(void);
void SystemClock_Config_24MHz (void);

/* Private functions ---------------------------------------------------------*/
void G_Test_Internal_ADC(void);
void G_Test_BQ796XX_Setting_Init_Without_Step(void);
void G_Test_BQ796XX_Setting_Init_With_Step(void);
void G_Test_BQ796XX_Gpio_Select_Read_ADC_FUNC(void);
void G_Test_BQ796XX_Cell_balance_func(void);
void G_Test_BQ796XX_Direction_Chechk_BMU_with_Step(void);
void G_Test_BQ796XX_Direction_Chechk_BMU_with_Step_Print_MeasureData(void);
void G_Test_BQ796XX_Direction_Chechk_BMU_with_Step_Print_Statistics(long int k_cnt);
uint8_t G_Test_BQ796XX_DIRECTION_SET_STEPS_NORTH(void);
void G_Test_BQ796XX_OVUVOTUT_FUNC(void);
void G_Test_IRM_Function(void);
void G_Test_TLC6C5912_FUNC(void);
void G_Test_Event_Log_FUNC(void);

//IRM GPIO Call Back Function---------------------------------------------------
#define ADS7946_VREF                           4.096f
#define ADS7946_RESOLUTION_B                   14
#define VSTACK_CAL_P1                          696.64f
#define VSTACK_CAL_P2                          -0.2192f
#define VO_CAL_V_P1                            0.002f   //unit:V


apiIRMonitoring_cb_t app_irm_event_cb;

//  Test use IRM calllback function to rx IRM data.
//--------------------------------------------------
uint8_t app_irm_rxdata_cb(IRMonitoring_Resistor_t *irm_res_data, IRMonitoring_event_cb_type irm_event){

  static IRMonitoring_Resistor_t temp_irm_data;
  static float temp_irm_vstack;
	
 	switch(irm_event)
 	{
 	case IRM_EVENT_BALANCE:
    temp_irm_data = *irm_res_data;
		break;
 	case IRM_EVENT_UNBALANCE:
    temp_irm_data = *irm_res_data;
		break;
 	case IRM_EVENT_GET_VSTACK:
    temp_irm_vstack = irm_res_data->V_stack;
		break;	
 	case IRM_EVENT_OTHER_ERR:

		break;
  }
	
	#if 1
	SEGGER_RTT_printf(0,"IRM EVENT=%d, Vstack=%d, Rn=%d, Rp=%d\r\n", irm_event, (uint16_t)(irm_res_data->V_stack), irm_res_data->Rn_kohm, irm_res_data->Rp_kohm);
	#endif
	
	return 0;
}
//--------------------------------------------------

void app_irm_sw_gpio_init_cb(void){
    GPIO_InitTypeDef GPIO_InitStructure;
	
	  // Control SW1~SW3 GPIO setting
	  //-------------------------------------------------
	  GPIO_InitStructure.Pin   = BSP_IRM_SW1_PIN;
	  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStructure.Pull  = GPIO_PULLUP;
	  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(BSP_IRM_SW1_PORT, &GPIO_InitStructure); 	

 	  GPIO_InitStructure.Pin   = BSP_IRM_SW2_PIN;
	  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStructure.Pull  = GPIO_PULLUP;
	  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(BSP_IRM_SW2_PORT, &GPIO_InitStructure); 	
	
 	  GPIO_InitStructure.Pin   = BSP_IRM_SW3_PIN;
	  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStructure.Pull  = GPIO_PULLUP;
	  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(BSP_IRM_SW3_PORT, &GPIO_InitStructure); 	
	  //-------------------------------------------------
}

void app_irm_sw1_gpio(IRMonitoring_SW_enum on_off){
    if(on_off == IRM_SW_ON){
	      BSP_IRM_SW1_ON();
	  }else{
		    BSP_IRM_SW1_OFF();
		}
}

void app_irm_sw2_gpio(IRMonitoring_SW_enum on_off){
    if(on_off == IRM_SW_ON){
	      BSP_IRM_SW2_ON();
	  }else{
		    BSP_IRM_SW2_OFF();
		}
}

void app_irm_sw3_gpio(IRMonitoring_SW_enum on_off){
    if(on_off == IRM_SW_ON){
	      BSP_IRM_SW3_ON();
	  }else{
		    BSP_IRM_SW3_OFF();
		}
}

IRM_Recv_CB_t irm_fun_ptr = 0;
static void app_imr_ads7946_callBack(uint8_t *pDat, uint8_t size)
{
	tIbyte	AdcValue;
	static float volt_data;
	static float volt_data2;
	static float adc_bits;
  adc_bits = (float)(1<<ADS7946_RESOLUTION_B); 	
	
	AdcValue.b[1] = pDat[2];
	AdcValue.b[0] = pDat[3];
	AdcValue.i >>= 2;
	volt_data = (float)(AdcValue.i)/adc_bits * ADS7946_VREF;
	//volt_data = (volt_data - VO_CAL_V_P1);
	volt_data = volt_data/2;
	
	if(volt_data< 0){
	    volt_data = 0;
	}
	
	if((BSP_IRM_SW1_PORT->ODR & BSP_IRM_SW1_PIN)== BSP_IRM_SW1_PIN){
	    if((BSP_IRM_SW2_PORT->ODR & BSP_IRM_SW2_PIN)== 0x0000){
			    if((BSP_IRM_SW3_PORT->ODR & BSP_IRM_SW3_PIN)== 0x0000){
						  volt_data2 = ((volt_data-VO_CAL_V_P1)*VSTACK_CAL_P1) +VSTACK_CAL_P2;       //Compensate for actual voltage(unit: V) 
	            LOG_GREEN("Vstack=%d mV\r\n",(uint16_t)(volt_data2*1000));
	        }
			}
	}
 
	LOG_GREEN("Vo=%d mV\r\n",(uint16_t)(volt_data*1000));
	
	if(irm_fun_ptr !=NULL){
		  irm_fun_ptr(&volt_data);
	}
	
}

void irm_data_ready_cb(IRM_Recv_CB_t rcv_ptr){
	irm_fun_ptr = rcv_ptr;
}

IRMonitoring_event_read_cb_type app_irm_trigger_voltage_data_cb(void){
	  static int8_t res;
		
	  //Test 2022.01.10
		GPIOD->ODR ^= GPIO_PIN_15;
	
    res = smp_ADS7946_get_data(channel_0,CS_0,app_imr_ads7946_callBack);
	  
	  if(res == SMP_SUCCESS){
		    return(IRM_OK);	
		}else{
		    return(IRM_BUSY);
		}
	
}

void app_irm_get_device_init_cb(void){
    smp_ADS7946_init();
}
//--------------------------------------------------

static void smp_DMA_Init(void)
{
	/* DMA controller clock enable */
	BSP_SPI1_DMAx_CLK_ENABLE();
	BSP_SPI2_DMAx_CLK_ENABLE();
	BSP_SPI3_DMAx_CLK_ENABLE();
	/* DMA interrupt init */
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BSP_SPI1_DMA_RX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(BSP_SPI1_DMA_RX_IRQn);
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BSP_SPI1_DMA_TX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(BSP_SPI1_DMA_TX_IRQn);
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BSP_SPI2_DMA_RX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(BSP_SPI2_DMA_RX_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BSP_SPI2_DMA_TX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(BSP_SPI2_DMA_TX_IRQn);
	/* DMA2_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BSP_SPI3_DMA_RX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(BSP_SPI3_DMA_RX_IRQn);
	/* DMA2_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BSP_SPI3_DMA_TX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(BSP_SPI3_DMA_TX_IRQn);

}

#ifdef G_TEST_EVENT_LOG_FUNC
extern uint16_t Davinci_uart_rx_cnt;
extern smp_uart_t mDavinci_uart;
smp_sector_header_package	header_package;
uint8_t page_data_buffer[256];

//John test event log appcalition, but this test code no use smp_uart, so that Golden modify to "test_uart_rx_process" for test event log.
//---------------------------------------------------------------------------------------
#if 0
uint16_t kk = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {
		if(usart_buf.aRxBuff == '\n'){
			usart_buf.Rx_end_flag = 1;
			usart_buf.RxBuff[usart_buf.RxSize] = usart_buf.aRxBuff;
			printf("UART %s",usart_buf.RxBuff);
			if(!strcmp((char *)usart_buf.RxBuff, "clean all\r\n")){       
				app_flash_log_managment_clean_all_memory();
			} 
			if(!strcmp((char *)usart_buf.RxBuff, "clean reflash\r\n")){       
				app_flash_log_managment_clean_reflash_memory();
			} 
			if(!strcmp((char *)usart_buf.RxBuff, "clean fix\r\n")){       
				app_flash_log_managment_clean_fix_memory();
			} 
			if(!strcmp((char *)usart_buf.RxBuff, "head clean\r\n")){ 
				app_flash_log_managment_clean_head();
			}
			if(!strcmp((char *)usart_buf.RxBuff, "head save\r\n")){  
				header_package.header[0] = 'S';
				header_package.header[1] = 'M';
				header_package.header[2] = 'P';
				header_package.header[3] = 'S';//S : sector
				header_package.reflash_memory_head_page = REFLASH_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
				header_package.reflash_memory_current_page = REFLASH_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
				header_package.reflash_total_log_cnt = kk;
				header_package.fix_memory_current_page = FIX_MEMORY_START_SECTOR * PAGE_NUM_IN_SECTOR;
				header_package.fix_total_log_cnt = kk;
				app_flash_sector_header_save(&header_package);
				kk +=1;
			} 
			if(!strcmp((char *)usart_buf.RxBuff, "head load\r\n")){  
				app_flash_sector_header_load(&header_package);		
			} 
			if(!strcmp((char *)usart_buf.RxBuff, "data save reflash\r\n")){  
				app_flash_page_data_save(SMP_REFLASH_MEMORY);
			} 
			if(!strcmp((char *)usart_buf.RxBuff, "data save fix\r\n")){  
				app_flash_page_data_save(SMP_FIX_MEMORY);
			} 
			uint8_t temp[18];
			uint8_t temp2[4];
			uint8_t temp4[4];
			uint16_t start_package;
			uint16_t length_data;
			memcpy(temp,&usart_buf.RxBuff[0],18);
			if(!strcmp((char *)temp, "data load reflash ")){ 	//data load reflash xxxx xxxx
				memcpy(temp2,&usart_buf.RxBuff[18],4);
				memcpy(temp4,&usart_buf.RxBuff[23],4);
				start_package =  a2i((char *)temp2);
				length_data =  a2i((char *)temp4);
				memset(&page_data_buffer[0], 0, 256);
				app_flash_page_data_load(page_data_buffer,start_package,length_data,SMP_REFLASH_MEMORY);
			} 
			if(!strcmp((char *)temp, "data push reflash ")){  
				smp_log_package log_package;
				memcpy(temp2,&usart_buf.RxBuff[18],4);
				length_data =  a2i((char *)temp2);
				for(int i = 0; i < length_data;i++){
					log_package.ID = 0xaa;
					log_package.SMP_RTC[0] = i;
					log_package.SMP_RTC[1] = 0;
					log_package.SMP_RTC[2] = 0;
					log_package.SMP_RTC[3] = 0;
					log_package.data[0] = 0x12;
					log_package.data[1] = 0x34;
					log_package.sum = 0xa5;
					app_flash_page_data_push(log_package,SMP_REFLASH_MEMORY);
				}
			} 
			uint8_t temp3[14];
			memcpy(temp3,&usart_buf.RxBuff[0],14);
			if(!strcmp((char *)temp3, "data load fix ")){ 	//data load fix xxxx xxxx
				memcpy(temp2,&usart_buf.RxBuff[14],4);
				memcpy(temp4,&usart_buf.RxBuff[19],4);
				start_package =  a2i((char *)temp2);
				length_data =  a2i((char *)temp4);
				memset(&page_data_buffer[0], 0, 256);
				app_flash_page_data_load(page_data_buffer,start_package,length_data,SMP_FIX_MEMORY);
			} 
			if(!strcmp((char *)temp3, "data push fix ")){  
				smp_log_package log_package;
				memcpy(temp2,&usart_buf.RxBuff[14],4);
				length_data =  a2i((char *)temp2);
				for(int i = 0; i < length_data;i++){
					log_package.ID = 0xaa;
					log_package.SMP_RTC[0] = i;
					log_package.SMP_RTC[1] = 0;
					log_package.SMP_RTC[2] = 0;
					log_package.SMP_RTC[3] = 0;
					log_package.data[0] = 0x12;
					log_package.data[1] = 0x34;
					log_package.sum = 0xa5;
					app_flash_page_data_push(log_package,SMP_FIX_MEMORY);
				}
			} 
			memset(usart_buf.RxBuff,0,256);
			usart_buf.RxSize = 0;
			usart_buf.Rx_end_flag = 0;
			
		}else{
			usart_buf.RxBuff[usart_buf.RxSize] = usart_buf.aRxBuff;
			usart_buf.RxSize++;
		}
		HAL_UART_Receive_IT(huart, &usart_buf.aRxBuff, 1);
	}
}
#endif
//---------------------------------------------------------------------------------------

// MCU UART3 Input char 'A'~'D' to test event log function api.
uint16_t kk = 0;
void test_uart_rx_process(void)
{
static uint8_t rx_data = 0;
static int8_t fifo_res;
	
	if(Davinci_uart_rx_cnt>0){
		fifo_res = smp_uart_get(&mDavinci_uart, &rx_data);
		--Davinci_uart_rx_cnt;
		if(fifo_res == SMP_SUCCESS)
		{	
      switch(rx_data){
			case 'A':	
				LOG_YELLOW("EVENT_LOG Clean ALL Memory\r\n");
				app_flash_log_managment_clean_all_memory();
			  break; 
			case 'B':
				LOG_YELLOW("EVENT_LOG Clean REFLASH Memory\r\n");
				app_flash_log_managment_clean_reflash_memory();
			  break;
			case 'C':		
				LOG_YELLOW("EVENT_LOG Clean FIX Memory\r\n");
				app_flash_log_managment_clean_fix_memory();
			  break;
			case 'D':
				LOG_YELLOW("EVENT_LOG Clean HEAD\r\n");
				app_flash_log_managment_clean_head();
			  break;
		  }
		}
	}
}

void app_flash_log_event_handler(smp_log_evt_type p_evt)
{
	switch(p_evt){
		case SMP_LOG_EVENT_SECTOR_HEADER_LOAD_DONE:
				LOG_CYAN("load head\r\n");
				LOG_CYAN("header %x\r\n",header_package.header[0]);
				LOG_CYAN("header %x\r\n",header_package.header[1]);
				LOG_CYAN("header %x\r\n",header_package.header[2]);
				LOG_CYAN("header %x\r\n",header_package.header[3]);
				LOG_CYAN("reflash head page%x\r\n",header_package.reflash_memory_head_page);
				LOG_CYAN("reflash current page%x\r\n",header_package.reflash_memory_current_page);
				LOG_CYAN("reflash cnt%d\r\n",header_package.reflash_total_log_cnt);
				LOG_CYAN("fix curent page%x\r\n",header_package.fix_memory_current_page);
				LOG_CYAN("fix cnt%d\r\n",header_package.fix_total_log_cnt);
		break;
		case SMP_LOG_EVENT_SECTOR_HEADER_SAVE_DONE:
				LOG_CYAN("save head\r\n");
				LOG_CYAN("header %x\r\n",header_package.header[0]);
				LOG_CYAN("header %x\r\n",header_package.header[1]);
				LOG_CYAN("header %x\r\n",header_package.header[2]);
				LOG_CYAN("header %x\r\n",header_package.header[3]);
				LOG_CYAN("reflash head page%x\r\n",header_package.reflash_memory_head_page);
				LOG_CYAN("reflash cnt%x\r\n",header_package.reflash_memory_current_page);
				LOG_CYAN("reflash cnt%d\r\n",header_package.reflash_total_log_cnt);
				LOG_CYAN("fix curent page%x\r\n",header_package.fix_memory_current_page);
				LOG_CYAN("fix cnt%d\r\n",header_package.fix_total_log_cnt);
		break;
		case SMP_LOG_EVENT_PAGE_LOAD_DONE:
				LOG_CYAN("page load\r\n");
				for(int i = 0; i < 256;i++){				
					LOG_CYAN("%d,%x\r\n",i,page_data_buffer[i]);
				}
		break;
		case SMP_LOG_EVENT_PAGE_SAVE_DONE:
				LOG_CYAN("page save\r\n");
		break;
		case SMP_LOG_EVENT_MEMORY_FULL:
				LOG_CYAN("fix memory full\r\n");
		break;
		case SMP_LOG_EVENT_ERROR:
				LOG_CYAN("Error\r\n");
		break;
		default:
		break;
	}
}
#endif

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
#ifdef NO_DELAY_TOWARDS_SMPS
  static uint32_t counter = 0; 
#endif
  
	uint32_t		del;
  GPIO_InitTypeDef GPIO_InitStructure;

	uint32_t SMPS_status = 0;
	uint32_t resumed_from_standby = 0;
	
	/* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
  */
		
	NVIC->ICER[0]=0xFFFFFFFF;
	NVIC->ICER[1]=0xFFFFFFFF;
	NVIC->ICER[2]=0xFFFFFFFF;
	
  HalBspInit();
	#if 0
  HAL_Init();
  #endif

  /* Configure the system clock to 80 MHz(HSI / HSE) */
	#if 0
  //SystemClock_Config_80MHz();
	#endif
  SystemClock_Config_HSE_80MHz();
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();	
	__HAL_RCC_RTCAPB_CLK_ENABLE();

  #if 0
  HAL_RCC_MCOConfig( RCC_MCO1 ,RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_16);          //Output SYSCLK to GPIOA PIN8
  #endif
	
	GPIO_InitStructure.Pin = 0x1F;                                               //GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	#if 0
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
	#endif
	
	GPIO_InitStructure.Pin = GPIO_PIN_13 | GPIO_PIN_14 |GPIO_PIN_15;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure); 

	GPIO_InitStructure.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	#if 0
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure); 
  #endif
	
	GPIO_InitStructure.Pin = GPIO_PIN_6;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	GPIO_InitStructure.Pin = GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  #if 0
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure); 
  #endif
	
	GPIO_InitStructure.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure); 
  HAL_Delay(200);
	
	SEGGER_RTT_Init();
	SEGGER_RTT_printf(0,"  start\r\n" );
	
	//LED Display Indicator Test
	//-------------------------------------------
	#ifdef G_TEST_TLC6C5912_FUNC
	G_Test_TLC6C5912_FUNC();
	#endif
	
  // MAX7219 LCD Maritex Test
	//-------------------------------------------
	#ifdef G_TEST_MAX7219_LCD_MARITEX
  maxInit(4,0);
	HAL_Delay(500);
	MAX7219_All_Display(0xFF);
	HAL_Delay(500);
	MAX7219_All_Display(0x00);
	HAL_Delay(500);
	MAX7219_Display_1();
	HAL_Delay(1000);
	#endif
  //-------------------------------------------
  
	appProjectOpen();
	LibSwTimerClearCount();
	
	// MCU UART3 Test
	//-------------------------------------------
  appSerialUartSendMessage("12345678");
	//-------------------------------------------
	
	// MX25LXX Flash Test
	//------------------------------------------
	#ifdef G_TEST_MX25LXX_FLASH
	res = smp_mx25l_flash_init();
	#endif
	//------------------------------------------
	
	// AD7946 SPI ADC Test
	//------------------------------------------
	#ifdef G_TEST_AD7946_ADC 
	smp_ADS7946_init();
	#endif
	//------------------------------------------
	
	//Test IRM SW1 mesaure Vstack(Total Battreary Voltage)
	#if 0
	app_irm_sw_gpio_init_cb();
	BSP_IRM_SW1_ON();
	HAL_Delay(500);
	BSP_IRM_SW1_OFF();
	HAL_Delay(500);
	BSP_IRM_SW1_ON();
	HAL_Delay(500);	
	BSP_IRM_SW2_ON();
	HAL_Delay(500);	
	BSP_IRM_SW2_OFF();	
	HAL_Delay(500);
	BSP_IRM_SW3_ON();
	HAL_Delay(500);	
	BSP_IRM_SW3_OFF();	
	HAL_Delay(500);	
	while(1){
		 BSP_IRM_SW1_OFF();
     BSP_IRM_SW2_ON();
     BSP_IRM_SW3_OFF();			
	   app_irm_trigger_voltage_data_cb();
	   HAL_Delay(500);
	}
	#endif
	
	//ADC Test
	//------------------------------------------
	#ifdef G_TEST_INT_ADC
  G_Test_Internal_ADC();
  #endif 
  //------------------------------------------
	
  // BQ96XX Setting Init without step Test
  //----------------------------------------------------------------------------------
	#ifdef G_TEST_BQ796XX_SETTING_INIT_WITHOUT_STEP
  G_Test_BQ796XX_Setting_Init_Without_Step();
	#endif
	//----------------------------------------------------------------------------------
	
	// BQ96XX Setting Init with step Test
	//----------------------------------------------------------------------------------
	#ifdef G_TEST_BQ796XX_SETTING_INIT_WITH_STEP
  G_Test_BQ796XX_Setting_Init_With_Step(); 
  #endif
	//----------------------------------------------------------------------------------
	
	// BQ796XX Cell Balance function Test
	//----------------------------------------------------------------------------------
	#ifdef G_TEST_BQ796XX_CELL_BALANCE_FUNC
  G_Test_BQ796XX_Cell_balance_func();
	#endif
	//----------------------------------------------------------------------------------

	// BQ796XX GPIO select read ADC function Test
	//----------------------------------------------------------------------------------	
  #ifdef G_TEST_BQ796XX_GPIO_SELECT_READ_ADC_FUNC
  G_Test_BQ796XX_Gpio_Select_Read_ADC_FUNC();
	#endif
	//----------------------------------------------------------------------------------

	// BQ796XX  Direction check North/South BMU number Test with step.
	//----------------------------------------------------------------------------------
	#ifdef G_TEST_BQ796XX_DIRECTION_CHECK_BMU_WITH_STEP
  G_Test_BQ796XX_Direction_Chechk_BMU_with_Step();
	#endif
  //----------------------------------------------------------------------------------

	// BQ796XX TEST OV/UV/OT/UT nFault and nFault setting at this data struct "bq796xx_init_default_t".
	//----------------------------------------------------------------------------------
	#ifdef G_TEST_BQ796XX_OVUVOTUT_FUNC
  G_Test_BQ796XX_OVUVOTUT_FUNC();
	#endif
  //----------------------------------------------------------------------------------	
	
  apiFuCheckMagicCode();
	
  for(int k=0 ;k<64; k++){
	    drv_bq796xx_Read_Stack_FaultSummary(0);                                    //Stack read Fault summary status.
	    drv_bq796xx_delay_ms(2*bq796xx_default.bmu_total_num);                     //Please set  delay > (n X BMU conut) ms
  }

	drv_bq796xx_clear_fifobuffer();
	 
	//IRM Test(2022/01/12)
	//----------------------------------------------------------------------------------------- 
	#ifdef G_TEST_IRM_FUNCTION
  G_Test_IRM_Function();
	#endif
	//-------------------------------------------------------------------------------------- 
	
	//Event log Test (2022/01/04)
	//--------------------------------------------------------------------------------------
	#ifdef G_TEST_EVENT_LOG_FUNC
  G_Test_Event_Log_FUNC();
	#endif
	//--------------------------------------------------------------------------------------
	
	while(1)
	{
		#ifdef G_TEST_LED_DISPLAY_BUTTON
	  if(BSP_BUTTON_READ() == 0){
			  if(test_led_display_cnt==0){
			      led_status_temp = smp_TLC6C5912_Get_Ledstauts(); 
        }
				smp_TLC6C5912_All_LED_On();
			  test_led_display_cnt++;
    }else{
			  test_led_display_cnt = 0;
			  smp_TLC6C5912_Set_Ledstauts(led_status_temp);
			  smp_TLC6C5912_Display_Ledstauts();
 	  }
		#endif	
			
		#if 0
		GPIOD->ODR |= GPIO_PIN_13;
		#endif
		
		LibSwTimerHandle();
		
		
		
		
		#if 0
		GPIOD->ODR &= ~GPIO_PIN_13;
		GPIOD->ODR ^= GPIO_PIN_14;
    GPIOB->ODR ^= GPIO_PIN_6;
	
    GPIOA->ODR ^= GPIO_PIN_4;//(GPIO_PIN_0 |1 );//| GPIO_PIN_1 | GPIO_PIN_2);
    for(del=0; del<2000000; del++);
		GPIOD->ODR ^= GPIO_PIN_13;
    GPIOC->ODR ^= GPIO_PIN_6;

    GPIOB->ODR ^= (GPIO_PIN_10 | GPIO_PIN_11);// |  GPIO_PIN_11);
    GPIOB->ODR ^= (GPIO_PIN_11);// |  GPIO_PIN_11);
		#endif
	}

#if	0	
  /* Configure LED1, and LED2 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);

  /* Check if the system was resumed from Standby mode */ 
  if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
  {
    /* Clear Standby flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

    resumed_from_standby = 1;
      
    /* Wait that user release the User push-button */
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
    while(BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_SET){}       

#ifndef NO_DELAY_TOWARDS_SMPS 
    HAL_Delay(2500);
#endif
  } 
  else
  {
    /* User push-button (External line 13) will be used to wakeup the system from Standby mode */
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

    /* Configure the system clock to 80 MHz */
    SystemClock_Config_80MHz();
 
    /* Insert 5 seconds delay */
    /* Delay at wake-up       */
    HAL_Delay(5000);
    
    /* ------------------------------------------ */

    SMPS_status = BSP_SMPS_DeInit();
    if (SMPS_status != SMPS_OK)
    {
      Error_Handler();
    }

  }
  
  SMPS_status = BSP_SMPS_Init(PWR_REGULATOR_VOLTAGE_SCALE2);
  if (SMPS_status != SMPS_OK)
  {
    Error_Handler();
  } 
			  
  if (resumed_from_standby == 0)
  {
    /* ------------------------------------------ */
    SystemClock_Config_24MHz(); 

#ifndef NO_DELAY_TOWARDS_SMPS     
    /* Insert 3 seconds delay */
    HAL_Delay(3000);
#endif
    
    /********************************/
    /* After reset                  */
    /* 24 MHZ                       */		
    /* PWR_REGULATOR_VOLTAGE_SCALE1 */
    /* SMPS IS OFF                  */
    /********************************/
  }
  
  /* ------------------------------------------ */
  
  /* PWR_REGULATOR_VOLTAGE_SCALE2 only with AD SMPS */		
  SMPS_status = HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
  if (SMPS_status != SMPS_OK)  
  {
    Error_Handler();
  } 

#ifndef NO_DELAY_TOWARDS_SMPS 
  /* Insert 2 seconds delay */
  HAL_Delay(2000);
#endif
     
  /********************************/
  /* After reset                  */
  /* 24 MHZ                       */		
  /* PWR_REGULATOR_VOLTAGE_SCALE2 */
  /* SMPS IS OFF                  */
  /********************************/
  /*             OR               */
  /********************************/
  /* Wake up from standby         */
  /* 4 MHZ                        */		
  /* PWR_REGULATOR_VOLTAGE_SCALE2 */
  /* SMPS IS OFF                  */
  /********************************/
  
  /* ------------------------------------------ */
  
  /* Enable SMPS */
  /* Check of PG but not blocking */
  SMPS_status = BSP_SMPS_Enable (0 , 1);  

#ifdef NO_DELAY_TOWARDS_SMPS
  while (SMPS_OK != BSP_SMPS_Supply_Enable (0,1))
  {
    counter++;
  }   
#else
  /* Check of Power Good */
  SMPS_status = BSP_SMPS_Supply_Enable (10 , 1); 
  
  if (SMPS_status != SMPS_OK)
  {
    Error_Handler();
  } 
#endif  
  
#ifndef NO_DELAY_TOWARDS_SMPS 
  /* Insert 1 second delay */
  HAL_Delay(1000);
#endif

  /********************************/
  /* After reset                  */
  /* 24 MHZ                       */		
  /* PWR_REGULATOR_VOLTAGE_SCALE2 */
  /* SMPS IS ON                  */
  /********************************/
  /*             OR               */
  /********************************/
  /* Wake up from standby         */
  /* 4 MHZ                        */		
  /* PWR_REGULATOR_VOLTAGE_SCALE2 */
  /* SMPS IS ON                  */
  /********************************/
  
  /* ------------------------------------------ */
  
  SystemClock_Config_80MHz(); 
   
  /********************************/
  /* 80 MHZ                       */		
  /* PWR_REGULATOR_VOLTAGE_SCALE2 */
  /* SMPS IS ON                  */
  /********************************/
  
  /* ------------------------------------------ */
  
  /* Enable Power Clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  while (1)
  {

  /* ------------------ GPIO ------------------ */
    
  /* Turn OFF LED's */
  BSP_LED_Off(LED1);
  BSP_LED_Off(LED2);

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();        
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();

  /* Set all GPIO in analog state to reduce power consumption,                */
  /*   except GPIOC to keep user button interrupt enabled                     */
  /* Note: Debug using ST-Link is not possible during the execution of this   */
  /*       example because communication between ST-link and the device       */
  /*       under test is done through UART. All GPIO pins are disabled (set   */
  /*       to analog input mode) including  UART I/O pins.                    */
  GPIO_InitStructure.Pin = GPIO_PIN_All;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure); 
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOF, &GPIO_InitStructure); 
  HAL_GPIO_Init(GPIOG, &GPIO_InitStructure); 
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOI, &GPIO_InitStructure);
 
  /* Disable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOE_CLK_DISABLE();
  __HAL_RCC_GPIOF_CLK_DISABLE();  
  __HAL_RCC_GPIOG_CLK_DISABLE();  
  __HAL_RCC_GPIOH_CLK_DISABLE();
  __HAL_RCC_GPIOI_CLK_DISABLE();

  /* Wait that user release the User push-button */
  while(BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_SET){}
    
  /* Disable all used wakeup sources: WKUP pin */
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
  
  /* Clear wake up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
    
  /* Enable wakeup pin WKUP2 */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_HIGH);  
  
  /* Insert 10 seconds delay */
  HAL_Delay(10000);

  SystemClock_Config_24MHz();

  /* wait 100 ms */
  HAL_Delay(100);
  
  BSP_SMPS_Supply_Disable();
  
  /* wait 100 ms */
  HAL_Delay(100);
  
  BSP_SMPS_Disable(); 
   
  /* wait 100 ms */
  HAL_Delay(100);
  
  /* NO NEED OF                                                 */
  /* HAL_PWREx_EnableGPIOPullUp (PWR_GPIO_SMPS, PWR_GPIO_ENABLE */
  /* HAL_PWREx_EnablePullUpPullDownConfig();                    */
  /* AS DONE IN BSP_SMPS_Enable                                 */ 
 
  /* Enter STANDBY mode */
  HAL_PWR_EnterSTANDBYMode();
   
  /* ... STANDBY mode ... */    
  }
  #endif
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 24000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 24000000
  * @param  None
  * @retval None
  */

void SystemClock_Config_24MHz(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Select MSI as system clock source */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI; 
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Update MSI to 24Mhz (RCC_MSIRANGE_9) */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config_80MHz(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

void SystemClock_Config_HSE_80MHz(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}




/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* Suspend tick */
 // HAL_SuspendTick();
  
  /* Turn LED2 */
  //BSP_LED_On(LED2);
  //while (1)
 // {
  ///}
	return;
}

void SysTick_Handler(void)
{
	HAL_IncTick();
}
/**
  * @brief SYSTICK callback
  * @param None
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{
  HAL_IncTick();

  if (TimingDelay != 0)
  {
    TimingDelay--;
  }
  else
  {
    /* Toggle LED1 */
    BSP_LED_Toggle(LED1);
    TimingDelay = LED_TOGGLE_DELAY;
  }
}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == USER_BUTTON_PIN)
  {
    /* Reconfigure LED1 */
    BSP_LED_Init(LED1); 
    /* Switch on LED1 */
    BSP_LED_On(LED1);
    
    /**/
    button_pressed = 1;
    
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

#ifdef G_TEST_INT_ADC
void G_Test_Internal_ADC(void){	
    static	uint16_t	app_adc_temp[10];
    extern bsp_adc_init_io_config bsp_in_adc_ch[5];
	
	  test_cont = 0;
	  smp_adc_adc_para_init(adc1);                            //Initial ADC 
	  while(1){
		    hal_internal_adc_get(&app_adc_temp[0] ,adc1 , bsp_in_adc_ch[0]);
		    hal_internal_adc_get(&app_adc_temp[1] ,adc1 , bsp_in_adc_ch[1]);
		    hal_internal_adc_get(&app_adc_temp[2] ,adc1 , bsp_in_adc_ch[2]);
		    hal_internal_adc_get(&app_adc_temp[3] ,adc1 , bsp_in_adc_ch[3]);
		    hal_internal_adc_get(&app_adc_temp[4] ,adc1 , bsp_in_adc_ch[4]);
		
		    LOG_BLUE("TEST ADC#%04d %04d,%04d,%04d,%04d,%04d\r\n", test_cont,app_adc_temp[0],app_adc_temp[1],app_adc_temp[2],app_adc_temp[3],app_adc_temp[4]);
		
		    HAL_Delay(50);
		
		    test_cont++;
		    if(test_cont>=G_TEST_INT_ADC_TEST_CYCLE_NUM) break;
    }	
}	
#endif 

#ifdef G_TEST_BQ796XX_SETTING_INIT_WITHOUT_STEP
void G_Test_BQ796XX_Setting_Init_Without_Step(void){
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
#endif

#ifdef G_TEST_BQ796XX_SETTING_INIT_WITH_STEP
void G_Test_BQ796XX_Setting_Init_With_Step(void){
	for(int k = 0; k < G_TEST_BQ796XX_SETTING_INIT_WITH_STEP_TEST_CYCLE_NUM; k++){
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
			if(wake_cnt >= G_TEST_BQ796XX_SETTING_INIT_WITH_STEP_WAKE_CNT){
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
#endif

#ifdef G_TEST_BQ796XX_GPIO_SELECT_READ_ADC_FUNC
void G_Test_BQ796XX_Gpio_Select_Read_ADC_FUNC(void){
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
		if(test_cont>G_TEST_BQ796XX_GPIO_SELECT_READ_ADC_FUNC_TEST_CYCLE_NUM) break;
		
		HAL_Delay(500); 
  }
}
#endif


#ifdef G_TEST_BQ796XX_CELL_BALANCE_FUNC
void G_Test_BQ796XX_Cell_balance_func(void){
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
		if(test_cont > G_TEST_BQ796XX_CELL_BALANCE_FUNC_TEST_CYCLE_NUM) break;
	}
}
#endif

#ifdef G_TEST_BQ796XX_DIRECTION_CHECK_BMU_WITH_STEP
// BQ796XX  Direction check North/South BMU number Test with step.
void G_Test_BQ796XX_Direction_Chechk_BMU_with_Step(void){
	north_res = 0;
	south_res = 0;
	bmu_is_ring = 0;

  for(long int k = 0; k < G_TEST_BQ796XX_DIRECTION_CHECK_BMU_WITH_STEP_TEST_CYCLE_NUM; k++){
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
			
		  G_Test_BQ796XX_Direction_Chechk_BMU_with_Step_Print_MeasureData();
			
			if((k%100)==0){
				  HAL_Delay(100);
			    G_Test_BQ796XX_Direction_Chechk_BMU_with_Step_Print_Statistics(k+1);
				  HAL_Delay(100);
			}
			
			dir_step_com_f = 0;
			dir_before_d_ms = 0;
  }
	
	HAL_Delay(100);
	G_Test_BQ796XX_Direction_Chechk_BMU_with_Step_Print_Statistics(G_TEST_BQ796XX_DIRECTION_CHECK_BMU_WITH_STEP_TEST_CYCLE_NUM);
	HAL_Delay(100);
}

void G_Test_BQ796XX_Direction_Chechk_BMU_with_Step_Print_MeasureData(void){
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

void G_Test_BQ796XX_Direction_Chechk_BMU_with_Step_Print_Statistics(long int k_cnt){
	LOG_GREEN("BMU CHK statistics Read BMU Count:\r\n");
	LOG_GREEN("Total Test = %06d, Current#%06d \r\n",G_TEST_BQ796XX_DIRECTION_CHECK_BMU_WITH_STEP_TEST_CYCLE_NUM, k_cnt);
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

#endif

#ifdef G_TEST_BQ796XX_OVUVOTUT_FUNC
void G_Test_BQ796XX_SET_GPIO_Channel(uint8_t gpio_ch){
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

uint8_t G_Test_BQ796XX_DIRECTION_SET_STEPS_NORTH(void){
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

void G_Test_BQ796XX_OVUVOTUT_FUNC(void){
  uint8_t d_payload[4]={0};
	north_res = 0;
	south_res = 0;
	bmu_is_ring = 0;

	
    LOG_GREEN("TEST BMU BQ795XX OVUVOTUT(FXIED_Channel CH=0)\r\n"); 
	  HAL_Delay(1000);
	
	  //Init BQ796XX
	  G_Test_BQ796XX_DIRECTION_SET_STEPS_NORTH();
	
	  //Select NTC Channel = 0 (NTC0 ~ NTC15), BMU with multipxer control NTC to GPIO1 input. 
	  G_Test_BQ796XX_SET_GPIO_Channel(0);

	  //Clear All nFault.
	  fill_data4_payload(d_payload,0xFF,0xFF,0,0); //Clear all=x0FF
	  drv_bq796xx_command_framing(STACK_WRITE, 0x00, BQ79600_FAULT_RST1, 2, d_payload, 2);
	  drv_bq796xx_command_framing(SINGLE_WRITE, 0x00, BQ79600_FAULT_RST, 1, d_payload, 2);
		
	  test_cont = 0;
	  while(1){
				 drv_bq796xx_clear_fifobuffer();
				
	       drv_bq796xx_Read_AFE_ALL_VCELL(STACK, 0, 0);                               //Stack(BMU #1,#2) Read all VCell 1~16(Main ADC).
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
		
				 for(int ki =0; ki<10*BMU_TOTAL_BOARDS; ki++){
				     res = drv_bq796xx_data_frame_parser();
				 }					 
		     
				 if(test_cont >=G_TEST_BQ796XX_OVUVOTUT_FUNC_TEST_CYCLE_NUM) break;
		     test_cont++; 
				 
				 G_Test_BQ796XX_Direction_Chechk_BMU_with_Step_Print_MeasureData();          //Show Read BUM Information
				 
				 HAL_Delay(1000);
	  }
			  
}
#endif

#ifdef G_TEST_IRM_FUNCTION
void G_Test_IRM_Function(void){
	//Register All callbasck function for IRM
	app_irm_event_cb.SW_gpioinit_cb = app_irm_sw_gpio_init_cb;
	app_irm_event_cb.SW_gpio_crtl_cb[0]=app_irm_sw1_gpio; 
	app_irm_event_cb.SW_gpio_crtl_cb[1]=app_irm_sw2_gpio;
	app_irm_event_cb.SW_gpio_crtl_cb[2]=app_irm_sw3_gpio; 
	app_irm_event_cb.GetVoltDeviceInit_cb = app_irm_get_device_init_cb;
	app_irm_event_cb.TriggerData_cb = app_irm_trigger_voltage_data_cb;
	app_irm_event_cb.irm_outdata = app_irm_rxdata_cb;
	app_irm_event_cb.DataReady_cb = irm_data_ready_cb;
	
	//Open IRM function, Setting 5s= IRM detection period, 200ms = IRM switch SW1~Sw3 waitting time. 
	res = apiIRMonitoringOpen(ITRM_TESTPARAM_DET_CYCLE_TIME, ITRM_TESTPARAM_RELAY_WAIT_TIME, app_irm_event_cb);
	
	//Get current Vstack, IRM use callback notification Vstack.
	while(test_get_vatck_cnt < G_TEST_IRM_FUNCTION_TEST_CYCLE_NUM){
		  test_get_vatck_cnt++;
		
	    LOG_BLUE("IRM Get Vstack...\r\n");
	    apiIRMonitoringGetVstack();
		  LibSwTimerHandle();
		  HAL_Delay(G_TEST_IRM_FUNCTION_TEST_CYCLE_DELAY_TIME); 
	}
}
#endif


#ifdef   G_TEST_TLC6C5912_FUNC
void G_Test_TLC6C5912_FUNC(void){
	smp_TLC6C5912_Init();
	for(int i=0;i<10;i++){
	    if((i%2)==1)
	        smp_TLC6C5912_All_LED_Off();
	    else
				  smp_TLC6C5912_All_LED_On();
			HAL_Delay(50);
	}
	
	for(int i=0;i<10;i++){
		  led_status_temp.w = 0x0555;
      smp_TLC6C5912_Set_Ledstauts(led_status_temp);
		  smp_TLC6C5912_Display_Ledstauts(); 
			HAL_Delay(50);
	}	

	for(int i=0;i<10;i++){
		  led_status_temp.w = 0x0AAA;
      smp_TLC6C5912_Set_Ledstauts(led_status_temp);
		  smp_TLC6C5912_Display_Ledstauts(); 
			HAL_Delay(50);
	}	
	
	smp_TLC6C5912_All_LED_Off();
	
	for(int i=0;i<10;i++){
      smp_TLC6C5912_Percent20_LED(i%2);
			HAL_Delay(50);
	}	
	
	for(int i=0;i<10;i++){
      smp_TLC6C5912_Percent40_LED(i%2);
			HAL_Delay(50);
	}
	
	for(int i=0;i<10;i++){
      smp_TLC6C5912_Percent60_LED(i%2);
			HAL_Delay(50);
	}	
	
	for(int i=0;i<10;i++){
      smp_TLC6C5912_Percent80_LED(i%2);
			HAL_Delay(50);
	}	

	for(int i=0;i<10;i++){
      smp_TLC6C5912_Percent100_LED(i%2);
			HAL_Delay(50);
	}	
	
	for(int i=0;i<10;i++){
      smp_TLC6C5912_Discharge_LED(i%2);
			HAL_Delay(50);
	}		
	
	for(int i=0;i<10;i++){
      smp_TLC6C5912_Charge_LED(i%2);
			HAL_Delay(50);
	}
	
	for(int i=0;i<10;i++){
      smp_TLC6C5912_Alarm_LED(i%2);
			HAL_Delay(50);
	}

	for(int i=0;i<10;i++){
      smp_TLC6C5912_Comm_LED(i%2);
			HAL_Delay(50);
	}	
	
	smp_TLC6C5912_All_LED_Off();
	smp_TLC6C5912_Percent100_LED(LED_ON);
	smp_TLC6C5912_Comm_LED(LED_ON);
}
#endif

#ifdef G_TEST_EVENT_LOG_FUNC
void G_Test_Event_Log_FUNC(void){
  smp_mx25l_status mx251_status;
	smp_mx25l_flash_init();
	
	uint16_t test_event_log_cnt = 0;
	
	app_flash_log_managment_init(app_flash_log_event_handler);
	HAL_Delay(1000);
	while(test_event_log_cnt<50)
	{
		
	  smp_mx25l_flash_read_status(&mx251_status);
		if((mx251_status.status1&STATUS_WRITE_IN_PROGRESS)==0){		
			 MX25L_SPI_send_command();
		}
    
		test_uart_rx_process();
		
		test_event_log_cnt++;
		HAL_Delay(100);
		LOG_WHITE("Event log#%d transfer\r\n", test_event_log_cnt);
	}
}
#endif
/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


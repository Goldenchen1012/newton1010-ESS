/**
  ******************************************************************************
  * @file        main.c
  * @author      Golden 
  * @version     v0.0.4
  * @date        2022/01/04
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
#include "smp_log_managment.h"
#include "SEGGER_RTT.h"
#include "RTT_Log.h"

#if 0
#define G_TEST_INT_ADC
#endif

#if 0
#define G_TEST_MX25LXX_FLASH 
#endif

#if 0
#define G_TEST_MAX7219_LCD_MARITEX
#endif

#if 0
#define G_TEST_AD7946_ADC
#endif

#if 0
#define G_TEST_BQ796XX_SETTING_INIT_WITHOUT_STEP
#endif 

#if 0
#define G_TEST_BQ796XX_SETTING_INIT_WITH_STEP
#endif

#if 0
#define G_TEST_BQ796XX_DIRECTION_CHECK_BMU_WITH_STEP
#endif

#if 0
#define G_TEST_BQ796XX_CELL_BALANCE_FUNC 
#endif

#if 0
#define G_TEST_IRM_FUNCTION
#endif 

#if 0
#define G_TEST_EVENT_LOG_FUNC
#endif

extern uint8_t app_afe_cb(bq796xx_data_t *bq_data, bq796xx_event_cb_type bq_event);

extern void ads7946_callBack(uint8_t *pDat, uint8_t size);

#define AFE_PARAM_LOAD_VALUE    {                                                          \
													          .ov_threshold         =OV_4175MV,                      \
	                                  .ov_step_threshold    =OV_STEP_100MV,                  \
	                                  .uv_threshold         =UV_3000MV,                      \
	                                  .ot_threshold         =OT_10_PCT,                      \
	                                  .ut_threshold         =UT_80_PCT,                      \
	                                  .ov_enable            =BQ_DISABLE,                     \
	                                  .uv_enable            =BQ_ENABLE,                      \
		                                .ot_enable            =BQ_DISABLE,                     \
	                                  .ut_enable            =BQ_DISABLE,                     \
		                                .cb_cycle_time        =CB_CYC_T_10S,                   \
	                                  .bmu_total_num        =BMU_TOTAL_BOARDS,               \
                                }	

#if 0
bq796xx_init_default_t afe_load_data = AFE_PARAM_LOAD_VALUE;																
#endif
																
extern bq796xx_init_default_t bq796xx_default;

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

long int test_dir_ok[2] ={0};
long int test_dir_fail[2] ={0};
long int irm_get_vstack_cont = 0;

uint16_t  test_init_rec_bmu_num[2][40]={0};

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

//IRM GPIO Call Back Function---------------------------------------------------
#define ADS7946_VREF                           4.096f
#define ADS7946_RESOLUTION_B                   14

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
	SEGGER_RTT_printf(0,"IRM EVENT=%d, Vstack=%d, Rn=%d, Rp=%d\r\n", irm_event, (int)(temp_irm_data.V_stack), temp_irm_data.Rn_kohm, temp_irm_data.Rp_kohm);
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
	static float adc_bits;
  adc_bits = (float)(1<<ADS7946_RESOLUTION_B); 	
	
	AdcValue.b[1] = pDat[2];
	AdcValue.b[0] = pDat[3];
	AdcValue.i >>= 3;
	volt_data = (float)(AdcValue.i)/adc_bits * ADS7946_VREF;
	
	if(irm_fun_ptr !=NULL){
		  irm_fun_ptr(&volt_data);
	}
	
}

void irm_data_ready_cb(IRM_Recv_CB_t rcv_ptr){
	irm_fun_ptr = rcv_ptr;
}

IRMonitoring_event_read_cb_type app_irm_trigger_voltage_data_cb(void){
	  static int8_t res;
		
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
	
//	smp_DMA_Init();
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

#if	1
	GPIO_InitStructure.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure); 
  HAL_Delay(200);
	
	SEGGER_RTT_Init();
	SEGGER_RTT_printf(0,"  start\r\n" );
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
//  appSerialUartSendMessage("12345678");
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
	BSP_IRM_SW3_OFF();
	BSP_IRM_SW3_OFF();
	
	while(1){
	   app_irm_trigger_voltage_data_cb();
	   HAL_Delay(500);
	}
	#endif
	
	//ADC Test
	//------------------------------------------
	#ifdef G_TEST_INT_ADC
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
		if(test_cont>=50) break;
	}
  #endif 
  //------------------------------------------
	
  // BQ96XX Setting Init without step Test
  //----------------------------------------------------------------------------------
	#ifdef G_TEST_BQ796XX_SETTING_INIT_WITHOUT_STEP
  drv_bq796xx_init_default_load(bq796xx_default);

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
	#endif
	
	#ifdef G_TEST_BQ796XX_SETTING_INIT_WITH_STEP
  //----------------------------------------------------------------------------------
	for(int k = 0; k<10 ; k++){
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
			if(wake_cnt>=3){
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
			    north_res = res & 0x7F;
			}else{
			    south_res = res & 0x7F;
			}
			
			test_init_rec_bmu_num[dir_state][res & 0x7F]++;
			
			bmu_is_ring = ((res & 0x80)>> 7);                                        //Results MSB is  0 = Non Ring, 1= Ring. 

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
  //---------------------------------------------------------------------------------
  #endif
	
	// BQ796XX Cell Balance function Test
	//----------------------------------------------------------------------------------
	#ifdef G_TEST_BQ796XX_CELL_BALANCE_FUNC
  drv_bq796xx_CellBalance_1to8_clear(STACK, 0 , 1);
  drv_bq796xx_CellBalance_9to16_clear(STACK, 0 , 1);
	 
  //Test Cell balance
  //Setting BMU=0x01, Cell 1,5,15,16 enable cell balance func =10s.
  cb_en1 =((1 << 4) | (1 << 0));
  cb_en2 =((1 << 3) | (1 << 1));				
	drv_bq796xx_CellBalance_1to8_set( 0x01, cb_en1,CB_TIME_10S,1);
  drv_bq796xx_CellBalance_9to16_set(0x01, cb_en2,CB_TIME_10S,1);		
	
  //Setting BMU=0x02, Cell 1,5 enable cell balance func=10s.
	drv_bq796xx_CellBalance_1to8_set(0x02, cb_en2,CB_TIME_10S,1);
	
	while(1){
	  //Starting cell balance, cell balance active until CB_CELLXX TIME, then this bit be self-clearing.
	  drv_bq796xx_Stack_CellBalanceStarting(CB_MANUAL, 1);
		drv_bq796xx_delay_ms(10);
		test_cont++;
		if(test_cont>100) break;
	}
	#endif
	//----------------------------------------------------------------------------------
  
	// BQ796XX  Direction check North/South BMU number Test with step.
	//----------------------------------------------------------------------------------
	#ifdef G_TEST_BQ796XX_DIRECTION_CHECK_BMU_WITH_STEP
	north_res = 0;
	south_res = 0;
	bmu_is_ring = 0;

  for(long int k = 0; k<100; k++){
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
			    north_res = dir_res & 0x7F;
			}else{
			    south_res = dir_res & 0x7F;
			}
			
			bmu_is_ring = ((dir_res & 0x80)>> 7);                                       //Results MSB is  0 = Non Ring, 1= Ring. 
			
			LOG_GREEN("BMU CHK#%04d N=%d,S=%d,Ring=%d\r\n", k, north_res, south_res , bmu_is_ring);
			
			if(dir_res>=2){
			   test_dir_ok[dir_state]++;
			}else{
			   test_dir_fail[dir_state]++;
			}
			
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
			
			dir_step_com_f = 0;
			dir_before_d_ms = 0;
  }
  //------------------------------------------------------------------------------------------		

	#endif
  //------------------------------------------------------------------------------------------
#if	0
  apiFuCheckMagicCode();
	
  for(int k=0 ;k<64; k++){
	    drv_bq796xx_Read_Stack_FaultSummary(0);                                    //Stack read Fault summary status.
	    drv_bq796xx_delay_ms(2*bq796xx_default.bmu_total_num);                     //Please set  delay > (n X BMU conut) ms
  }

	drv_bq796xx_clear_fifobuffer();
#endif	 
	//IRM Test(2021/12/21)
	//----------------------------------------------------------------------------------------- 
	#ifdef G_TEST_IRM_FUNCTION
	//Register All callbasck function for IRM
	app_irm_event_cb.SW_gpioinit_cb = app_irm_sw_gpio_init_cb;
	app_irm_event_cb.SW_gpio_crtl_cb[0]=app_irm_sw1_gpio; 
	app_irm_event_cb.SW_gpio_crtl_cb[1]=app_irm_sw2_gpio;
	app_irm_event_cb.SW_gpio_crtl_cb[2]=app_irm_sw3_gpio; 
	app_irm_event_cb.GetVoltDeviceInit_cb = app_irm_get_device_init_cb;
	app_irm_event_cb.TriggerData_cb = app_irm_trigger_voltage_data_cb;
	app_irm_event_cb.irm_outdata = app_irm_rxdata_cb;
	app_irm_event_cb.DataReady_cb = irm_data_ready_cb;
	
	//Open IRM function, Setting 2s= IRM detection period, 100ms = IRM switch SW1~Sw3 waitting time. 
	res = apiIRMonitoringOpen(2, 100, app_irm_event_cb);
	
	//Get current Vstack, IRM use callback notification Vstack.
	apiIRMonitoringGetVstack();
	#endif
	//-------------------------------------------------------------------------------------- 
	
	//Event log Test (2022/01/04)
	//--------------------------------------------------------------------------------------
	#ifdef G_TEST_EVENT_LOG_FUNC
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
	#endif
	//--------------------------------------------------------------------------------------
	
	while(1)
	{
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

#if 0
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
#endif

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

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


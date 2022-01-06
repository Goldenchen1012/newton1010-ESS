#include "AppProjectTest.h"
#include "smp_debug.h"
#include <stdbool.h>



static uint8_t	SystemReadyFlag = 0;
tHalTimer	mHalTimer4={3, 1000};

static void DebugGPIOInit(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	__HAL_RCC_GPIOE_CLK_ENABLE();
	GPIO_InitStructure.Pin = GPIO_PIN_10 | GPIO_PIN_15;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure); 

}

static void ADDS7946_Test_API1_callBack(uint8_t *pDat, uint8_t size){
	
}

static void ADDS7946_Test_API2_callBack(uint8_t *pDat, uint8_t size){
	

}
static void ADDS7946_Test_API1_SwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr){
	if(evt == LIB_SW_TIMER_EVT_SW_10MS_0){	
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);		
		smp_ADS7946_get_data(channel_1,CS_0,ADDS7946_Test_API1_callBack);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
	}

}
static void ADS7946_Test_API1_2_SwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr){
	static uint8_t Cnt_1S = 0;
	if(evt == LIB_SW_TIMER_EVT_SW_10MS_1){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
		smp_ADS7946_get_data(channel_1,CS_1,ADDS7946_Test_API2_callBack);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
	}
	if(evt == LIB_SW_TIMER_EVT_SW_1S){
		Cnt_1S++;
		if(Cnt_1S== 2){
			Cnt_1S = 0;
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
			smp_ADS7946_get_data(channel_1,CS_1,ADDS7946_Test_API2_callBack);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
		}
	}

}

static void ADDS7946_Test_API1_Open(void){
	smp_ADS7946_init();
	LibSwTimerOpen(ADDS7946_Test_API1_SwTimerHandler, 0);
}

static void ADDS7946_Test_API2_Open(void){
	smp_ADS7946_init();
	LibSwTimerOpen(ADS7946_Test_API1_2_SwTimerHandler, 0);
}

static void appProjectTestSwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
	static	uint8_t		SystemReadyCount = 10;
	char	str[100];

    if(evt == LIB_SW_TIMER_EVT_SW_1MS)
	{
			
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
		if(SystemReadyCount)
		{
			SystemReadyCount--;
			if(!SystemReadyCount)
				SystemReadyFlag = 1;
		}
	}
}

static void appProjectTestHwTimerHandler(void *pin, uint16_t evt, void *pData)
{
	LibSwTimerHwHandler(LIB_SW_TIMER_EVT_HW_1MS, 0);
	LibHwTimerHandle();
}

void appTestProjectOpen(void){
	
	DebugGPIOInit();
	HalTimerOpen(&mHalTimer4, appProjectTestHwTimerHandler);
	ADDS7946_Test_API1_Open();
	ADDS7946_Test_API2_Open();
	
}
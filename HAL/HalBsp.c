/**
  ******************************************************************************
  * @file        HalBsp.c
  * @author      Johnny
  * @version     v0.0.0
  * @date        2021/10/29
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"
#include "Bsp.h"
#include "HalBsp.h"

void appSerialCanDavinciSendTextMessage(char *str);
#define	halbspDebugMsg(str)	appSerialCanDavinciSendTextMessage(str);

/* Private macro -------------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/

/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/

void HalBspInit(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	BSP_DO1_OPEN();
	BSP_DO2_OPEN();
	
	#if 0
	BSP_ADC_CH_SEL_OPEN();
	BSP_ADC_CS_MAIN_OPEN();
	BSP_ADC_CS_AUX_OPEN();
  #endif 
	
	BSP_K1_OPEN();
	BSP_K2_OPEN();
	BSP_K3_OPEN();
	BSP_K4_OPEN();
	BSP_RELAY_PS_OPEN();
	BSP_TOWER_LIGHT_RED_OPEN();
	BSP_TOWER_LIGHT_ORANGE_OPEN();
	BSP_TOWER_LIGHT_GREEN_OPEN();
	BSP_OCP_RELEASE_OPEN();
	BSP_OD_OUT_OPEN();
	
	BSP_DI1_OPEN();
	BSP_DI2_OPEN();
	BSP_EPO_OPEN();
	BSP_SP_FB_OPEN();
	BSP_PS1_OK_OPEN();
	BSP_PS2_OK_OPEN();
	BSP_PS3_OK_OPEN();
	BSP_BUTTON_OPEN();
	BSP_NFAULT_OPEN();
	
//	BSP_K1_FB_OPEN();
//	BSP_K1_FB_READ();
//	BSP_K2_FB_OPEN();
//	BSP_K3_FB_OPEN();
//	BSP_K4_FB_OPEN();

	BSP_DOCP_LATCH_OPEN();
	BSP_COCP_LATCH_OPEN();
	BSP_OD_IN_OPEN();
	
//	HalBspRelayPsCtrl(1);
}

void HalBspDO1Ctrl(uint8_t hi)
{
	if(hi)
		BSP_DO1_HI();
	else
		BSP_DO1_LO();
}
void HalBspDO2Ctrl(uint8_t hi)
{
	if(hi)
		BSP_DO2_HI();
	else
		BSP_DO2_LO();
}

#if 0
void HalBspAdcChCtrl(uint8_t hi)
{
	if(hi)
		BSP_ADC_CH_SEL_HI();
	else
		BSP_ADC_CH_SEL_LO();
}

void HalBspAdcCsMainCtrl(uint8_t hi)
{
	if(hi)
		BSP_ADC_CS_MAIN_HI();
	else
		BSP_ADC_CS_MAIN_LO();
}
void HalBspAdcCsAuxCtrl(uint8_t hi)
{
	if(hi)
	 	BSP_ADC_CS_AUX_HI();
	else
		BSP_ADC_CS_AUX_LO();
}
#endif 

void HalBslK1Ctrl(uint8_t hi)
{
	if(hi)
		BSP_K1_HI();
	else
		BSP_K1_LO();
}
void HalBspK2Ctrl(uint8_t hi)
{
	if(hi)
		BSP_K2_HI();
	else
		BSP_K2_LO();
}
void halBspFanRelayOn(void)
{
	BSP_K1_HI();
}
void halBspFanRelayOff(void)
{
	BSP_K1_LO();
}

void halBspPreDischargeRelayOn(void)
{
	BSP_K2_HI();
}
void halBspPreDischargeRelayOff(void)
{
	BSP_K2_LO();
}

void halBspPostiveRelayOn(void)
{
	BSP_K3_HI();
}

void halBspPostiveRelayOff(void)
{
	BSP_K3_LO();
}
void halBspNegtiveRelayOn(void)
{
	BSP_K4_HI();
}
void halBspNegtiveRelayOff(void)
{
	BSP_K4_LO();
}

void HalBspRelayPsCtrl(uint8_t hi)
{
	if(hi)
		BSP_RELAY_PS_HI();
	else
		BSP_RELAY_PS_LO();
}
void HalBspTowerLightRedCtrl(uint8_t hi)
{
	if(hi)
		BSP_TOWER_LIGHT_RED_HI();
	else
		BSP_TOWER_LIGHT_RED_LO();
}

void HalBspTowerLightOrangeCtrl(uint8_t hi)
{
	if(hi)
		BSP_TOWER_LIGHT_ORANGE_HI();
	else
		BSP_TOWER_LIGHT_ORANGE_LO();
}
void HalBspTowerLightGreenCtrl(uint8_t hi)
{
	if(hi)
		BSP_TOWER_LIGHT_GREEN_HI();
	else
		BSP_TOWER_LIGHT_GREEN_LO();
}
void HalBspReleaseCtrl(uint8_t hi)
{
	if(hi)
		BSP_OCP_RELEASE_HI();
	else
		BSP_OCP_RELEASE_LO();
}
void HalBspOdOutCtrl(uint8_t hi)
{
	if(hi)
		BSP_OD_OUT_HI();
	else
		BSP_OD_OUT_LO();
}

//-----------------------------------
//	Input
uint8_t HalBspGetDi1Status(void)
{
	if(BSP_DI1_READ())
		return 1;
	else
		return 0;
}
uint8_t HalBspGetDi2Status(void)
{
	if(BSP_DI2_READ())
		return 1;
	else
		return 0;
}

uint8_t HalBspGetEpoStatus(void)
{
	if(BSP_EPO_READ())
		return 1;
	else
		return 0;
}

uint8_t HalBspGetSpStatus(void)
{
	if(BSP_SP_FB_READ())
		return 1;
	else
		return 0;
}

uint8_t HalBspGetPs1Status(void)
{
	if(BSP_PS1_OK_READ())
		return 1;
	else
		return 0;
}
uint8_t HalBspGetPs2Status(void)
{
	if(BSP_PS2_OK_READ())
		return 1;
	else
		return 0;
}
uint8_t HalBspGetPs3Status(void)
{
	if(BSP_PS3_OK_READ())
		return 1;
	else
		return 0;
}
uint8_t HalBspGetButtonStatus(void)
{
	if(BSP_BUTTON_READ())
		return 1;
	else
		return 0;
}



uint8_t HalBspGetK1Status(void)
{
	if(BSP_K1_FB_READ())
		return 1;
	else
		return 0;
}

uint8_t HalBspGetK2Status(void)
{
	if(BSP_K2_FB_READ())
		return 1;
	else
		return 0;
}

uint8_t HalBspGetK3Status(void)
{
	if(BSP_K3_FB_READ())
		return 1;
	else
		return 0;
}

uint8_t HalBspGetK4Status(void)
{
	if(BSP_K4_FB_READ())
		return 1;
	else
		return 0;
}

uint8_t HalBspGetDocpLatchStatus(void)
{
	if(BSP_DOCP_LATCH_READ())
		return 1;
	else
		return 0;
}

uint8_t HalBspGetCocpLatchStatus(void)
{
	if(BSP_COCP_LATCH_READ())
		return 1;
	else
		return 0;
}

uint8_t HalBspGetOdInStatus(void)
{
	if(BSP_OD_IN_READ())
		return 1;
	else
		return 0;
}
uint8_t halBspGetNfaultStatus(void)
{
	if(BSP_NFAULT_READ())
		return 1;
	else
		return 0;
}


static void K1Hi(void)
{
	BSP_K1_HI();
}

static void K1Lo(void)
{
	BSP_K1_LO();
}

static void K2Hi(void)
{
	BSP_K2_HI();
}

static void K2Lo(void)
{
	BSP_K2_LO();
}

static void K3Hi(void)
{
	BSP_K3_HI();
}

static void K3Lo(void)
{
	BSP_K3_LO();
}
static void K4Hi(void)
{
	BSP_K4_HI();
}

static void K4Lo(void)
{
	BSP_K4_LO();
}

static void Do1Hi(void)
{
	BSP_DO1_HI();
}
static void Do1Lo(void)
{
	BSP_DO1_LO();
}

static void Do2Hi(void)
{
	BSP_DO2_HI();
}
static void Do2Lo(void)
{
	BSP_DO2_LO();
}
static void relayPsHi(void)
{
	BSP_RELAY_PS_HI();
}
static void relayPsLo(void)
{
	BSP_RELAY_PS_LO();
}
static void odOutHi(void)
{
	BSP_OD_OUT_HI();
}
static void odOutLo(void)
{
	BSP_OD_OUT_LO();
}
static void ocpReleaseHi(void)
{
	BSP_OCP_RELEASE_HI();
}
static void ocpReleaseLo(void)
{
	BSP_OCP_RELEASE_LO();
}

static void irmSw1Hi(void)
{	
	BSP_IRM_SW1_ON();
}
static void irmSw1Lo(void)
{	
	BSP_IRM_SW1_OFF();
}
static void irmSw2Hi(void)
{	
	BSP_IRM_SW2_ON();
}
static void irmSw2Lo(void)
{	
	BSP_IRM_SW2_OFF();
}	

static void irmSw3Hi(void)
{	
	BSP_IRM_SW3_ON();
}
static void irmSw3Lo(void)
{	
	BSP_IRM_SW3_OFF();
}	
static void towerRedLedHi(void)
{
	BSP_TOWER_LIGHT_RED_HI();
}
static void towerRedLedLo(void)
{
	BSP_TOWER_LIGHT_RED_LO();
}
static void towerOrangeLedHi(void)
{
	BSP_TOWER_LIGHT_ORANGE_HI();
}
static void towerOrangeLedLo(void)
{
	BSP_TOWER_LIGHT_ORANGE_LO();
}
static void towerGreenLedHi(void)
{
	BSP_TOWER_LIGHT_GREEN_HI();
}
static void towerGreenLedLo(void)
{
	BSP_TOWER_LIGHT_GREEN_LO();
}
static void termlrHi(void)
{
	BSP_TERMLR_HI();
}
static void termlrLo(void)
{
	BSP_TERMLR_LO();
}

typedef void (* tGpioControlFun)(void);


typedef struct{
	uint8_t			group;
	uint32_t		bits;
	tGpioControlFun	HiFun;
	tGpioControlFun	LoFun;
	char			*msg;
}tGpioCtrlGroup;
const tGpioCtrlGroup	GpioCtrlGroup[]={
{0, (1<<0) ,K1Hi, K1Lo,								"K1      "},
{0, (1<<1) ,K2Hi, K2Lo,								"K2      "},
{0, (1<<2) ,K3Hi, K3Lo,								"K3      "},
{0, (1<<3) ,K4Hi, K4Lo,								"K4      "},
{0, (1<<4) ,Do1Hi, Do1Lo,							"DO1     "},
{0, (1<<5) ,Do2Hi, Do2Lo,							"DO2     "},
{0, (1<<6) ,relayPsHi, relayPsLo,					"RelayPS "},
{0, (1<<7) ,odOutHi, odOutLo,						"OD_O    "},
{0, (1<<8) ,ocpReleaseHi, ocpReleaseLo,				"OcpRels."},
{0, (1<<9) ,irmSw1Hi, irmSw1Lo,						"I.R. Sw1"},
{0, (1<<10) ,irmSw2Hi, irmSw2Lo,					"I.R. Sw2"},
{0, (1<<11) ,irmSw3Hi, irmSw3Lo,					"I.R. Sw3"},
{0, (1<<12) ,towerRedLedHi, towerRedLedLo,			"TowerRed"},
{0, (1<<13) ,towerOrangeLedHi, towerOrangeLedLo,	"TowerOrg"},
{0, (1<<14) ,towerGreenLedHi, towerGreenLedLo,		"TowerGre"},
{0, (1<<15) ,termlrHi, termlrLo,					"TermlR  "},

{0,0,0,0}
	
};
	
char *halBspGetGpioControlMsg(uint8_t group, uint32_t mask)
{
	uint8_t		i;
	
	for(i=0; i<250; i++)
	{
		if(GpioCtrlGroup[i].bits == 0)
			break;
		if(GpioCtrlGroup[i].group == group &&
		   GpioCtrlGroup[i].bits == mask)
		{
			return GpioCtrlGroup[i].msg;
		}		   
	}
	return "        ";
}

void halBspGpioControl(uint8_t group, uint32_t mask, uint32_t dat)
{
	uint8_t		i;
	
	for(i=0; i<250; i++)
	{
		if(GpioCtrlGroup[i].bits == 0)
			break;
		if(GpioCtrlGroup[i].group == group &&
		   GpioCtrlGroup[i].bits == mask)
		{
			halbspDebugMsg(GpioCtrlGroup[i].msg);

			if(mask & dat)
			{
				halbspDebugMsg("Hi");
				GpioCtrlGroup[i].HiFun();
			}
			else
			{
				halbspDebugMsg("Lo");
				GpioCtrlGroup[i].LoFun();
			}
			break;
		}		   
	}
}

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    


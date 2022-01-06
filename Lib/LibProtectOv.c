/**
  ******************************************************************************
  * @file        LibProtectOv.c
  * @author      Johnny
  * @version     v0.0
  * @date        2021/10/14
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
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "boards.h"
#include "datatype.h"
#include "fault.h"
#include "LibProtectDef.h"
#include "LibAfe.h"
#include "LibSwTimer.h"


/* Private typedef -----------------------------------------------------------*/
typedef struct {
  tLibProtectEvtHandler evtHandler;
  uint8_t flag[BSP_CELL_NUM_MAX];
  struct{
    tAdcData setVoltageLsb;
    tAdcData releaseVoltageLsb;
	uint8_t	count[BSP_CELL_NUM_MAX];
    uint8_t setTime;
    uint8_t releaseTime;
  }Par[LIB_OVP_LEVEL_MAX];
} tOvpData;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
tOvpData mOvp={.flag = {0}, 
               .Par[0].setVoltageLsb = LIB_OVP_THRESHOLD_L1,
               .Par[1].setVoltageLsb = LIB_OVP_THRESHOLD_L2,
               .Par[2].setVoltageLsb = LIB_OVP_THRESHOLD_L3,
               .Par[0].releaseVoltageLsb = LIB_OVP_THRESHOLD_RELEASE,
               .Par[1].releaseVoltageLsb = LIB_OVP_THRESHOLD_RELEASE,
               .Par[2].releaseVoltageLsb = LIB_OVP_THRESHOLD_RELEASE,
               .Par[0].setTime = LIB_OVP_TIME,
               .Par[1].setTime = LIB_OVP_TIME,
               .Par[2].setTime = LIB_OVP_TIME,
               .Par[0].releaseTime = LIB_OVP_TIME_RELEASE,
               .Par[1].releaseTime = LIB_OVP_TIME_RELEASE,
               .Par[2].releaseTime = LIB_OVP_TIME_RELEASE,
               };

const uint8_t LibProtectLevelMask[LIB_OVP_LEVEL_MAX] = {L1_FLAG_MASK, L2_FLAG_MASK, L3_FLAG_MASK};
const uint8_t LibProtectLevelInvMask[LIB_OVP_LEVEL_MAX] = {~L1_FLAG_MASK, ~L2_FLAG_MASK, ~L3_FLAG_MASK};
const uint8_t LibProtectLevelSetting[LIB_OVP_LEVEL_MAX] = {L1_FLAG_SETING, L2_FLAG_SETING, L3_FLAG_SETING};
const uint8_t LibProtectLevelSetted[LIB_OVP_LEVEL_MAX] = {L1_FLAG_SETTED, L2_FLAG_SETTED, L3_FLAG_SETTED};
const uint8_t LibProtectLevelReling[LIB_OVP_LEVEL_MAX] = {L1_FLAG_RelING, L2_FLAG_RelING, L3_FLAG_RelING};
/* Private function prototypes -----------------------------------------------*/


#ifndef DISABLE_OVP_FUN

#ifndef DISABLE_ALL_MESSAGE	
	#define	SHOW_OVP_DEBUG_MSG
#endif

void LibProtectHandlerOv(WORD startCell)
{
#ifdef SHOW_OVP_DEBUG_MSG
	BYTE	str[200];
#endif
	tAdcData	voltage;
	WORD	cell;
	BYTE level;
    for(level = 0;level<LIB_OVP_LEVEL_MAX;level++){
	  for(cell=startCell;cell<BSP_CELL_NUM_MAX;cell++){
		voltage = libAfeAdcGetCellVoltageLsb(cell);

        //Check Setting
		if(voltage >= mOvp.Par[level].setVoltageLsb)
		{
			if((mOvp.flag[cell]&LibProtectLevelMask[level])==0)	//���e�bRelease���A�A�}�l�o�� OVP Set
			{
				mOvp.flag[cell]&=LibProtectLevelInvMask[level];
				mOvp.flag[cell]|=LibProtectLevelSetting[level];
				mOvp.Par[level].count[cell]=0;
			}
			else if((mOvp.flag[cell]&LibProtectLevelMask[level])==LibProtectLevelSetting[level])	//���b�o��OVP Set
			{
				mOvp.Par[level].count[cell]++;
				if(mOvp.Par[level].count[cell]>=mOvp.Par[level].setTime)
				{
					mOvp.flag[cell]&=LibProtectLevelInvMask[level];
					mOvp.flag[cell]|=LibProtectLevelSetted[level];	//Seted
					mOvp.evtHandler(FAULT_TYPE_OVP_L1_SET+level,&cell);
				}
			}
		}
		else if((mOvp.flag[cell]&LibProtectLevelMask[level])==LibProtectLevelSetting[level])	//���bSetting,�^��Release���A
		{
			mOvp.flag[cell]&=LibProtectLevelInvMask[level];
		}

        //Check release
        if(voltage < mOvp.Par[level].releaseVoltageLsb)
		{
			if((mOvp.flag[cell]&LibProtectLevelMask[level])==LibProtectLevelSetting[level])	//Seted!! �w�o�͹LOVP Set
			{
				mOvp.flag[cell]|=LibProtectLevelReling[level];		//Releaseing
				mOvp.Par[level].count[cell]=0;
			}
			else if((mOvp.flag[cell]&LibProtectLevelMask[level])==LibProtectLevelReling[level])
			{
				mOvp.Par[level].count[cell]++;
				if(mOvp.Par[level].count[cell]>=mOvp.Par[level].releaseTime)
				{
					mOvp.flag[cell]&=LibProtectLevelInvMask[level];	//Released
					mOvp.Par[level].count[cell]=0;				
					mOvp.evtHandler(FAULT_TYPE_OVP_L1_RELEASE+level,&cell);
				}
			}
		}
		else if((mOvp.flag[cell]&LibProtectLevelMask[level])==LibProtectLevelReling[level])	//���brelease,
		{
			mOvp.flag[cell]&=LibProtectLevelInvMask[level];
			mOvp.flag[cell]|=LibProtectLevelSetted[level];
		}
      }
	}	
}

static void LibProtectOvTimerHandler(uint16_t evt, void *data){
  if(evt == LIB_SW_TIMER_EVT_SW_100MS){
    LibProtectHandlerOv(0);
  }
}

void LibProtectOvOpen(tLibProtectEvtHandler handler){
  if(handler != 0){
    mOvp.evtHandler = handler;  
  }
  LibSwTimerOpen(LibProtectOvTimerHandler);
}

void LibProtectOvClose(void){
  LibSwTimerClose(LibProtectOvTimerHandler);
}
	

#endif //DISABLE_OVP_FUN


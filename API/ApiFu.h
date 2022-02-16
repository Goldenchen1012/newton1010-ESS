/**
  ******************************************************************************
  * @file        ApiFu.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/11/25
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _API_FU_H_
#define _API_FU_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/* Public typedef -----------------------------------------------------------*/
typedef void(*tApiFuCallbackFunction)(uint16_t evt, uint8_t *pMsgBuf);

/* Public define ------------------------------------------------------------*/
enum{
	API_FU_FW_CHECK_INI = 0,
	API_FU_FW_CHECKING,
	API_FU_FW_CHECK_SUCCESS,
	API_FU_FW_CHECK_FAIL,
	API_FU_FW_CHECK_END
};

enum{
	API_FU_EVT_PROGRESS = 0,
	API_FU_EVT_START_FW_CHECK,
	API_FU_EVT_CHECK_RESULT,
	API_FU_EVT_FW_CHECKING,
	API_FU_EVT_CHECK_FINISH,
	API_FU_EVT_END
};

enum{
	MAGIC_CODE_FROM_APP = 0,
	MAGIC_CODE_FROM_ISP,
	MAGIC_CODE_FROM_APP_RESET,
	MAGIC_CODE_FROM_ISP_RESET,
	MAGIC_CODE_UPDATE_FW1,
	MAGIC_CODE_UPDATE_FW2,
	MAGIC_CODE_APP_NORMAL	
};

/* Public variables ---------------------------------------------------------*/
extern const uint8_t	SmpFwHeadInfo1[];
extern const uint8_t	SmpFwHeadInfo2[];

/* Public macro -------------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
uint8_t	apiFuGetFwCheckStatus(void);
void apiFuStartUp(tApiFuCallbackFunction CbFunction);
void apiFuSetTotalPackageNum(uint32_t num);
void apiFuRcvSetVersion(uint32_t Version);
void apiFuRcvSetBaseAddr(uint32_t BaseAddr);
void apiFuSetUpgradeData(uint32_t addr, uint8_t *pDatBuf, uint16_t leng);

void apiFuSetMagicCode(uint8_t type);
void apiFuCheckMagicCode(void);

uint8_t apiFuGetMagicModeIndex(void);
void apiFuJumpToBootloader(void);
void apiFuUpdateFw(void);
void apiFuResetAndUpdate(void);
void apiFuResetApp(void);
void apiFuCheckMagicCode(void);

uint32_t apiFuGetFwVersion(void);
uint32_t apiFuGetFwBuildDate(void);
uint32_t apiFuGetFwBuildTime(void);
uint32_t apiFuGetFwChecksum(void);


#ifdef __cplusplus
}
#endif


	

#endif /* _API_FU_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



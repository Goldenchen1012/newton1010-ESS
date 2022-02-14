/**
  ******************************************************************************
  * @file        ApiCanbusPkgFu.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/12/27
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _API_CANBUS_PKG_FU_H_
#define _API_CANBUS_PKG_FU_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/

/* Public macro -------------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
void apiCanbusPkgFuDecodeInfoPackage(smp_can_package_t *pCanPkg, tApiFuCallbackFunction CbFunction);
void apiCanbusPkgFuDecodeDataPackage(smp_can_package_t *pCanPkg);


#ifdef __cplusplus
}
#endif


	

#endif /* _API_CANBUS_PKG_FU_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    






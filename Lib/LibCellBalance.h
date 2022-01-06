/**
  ******************************************************************************
  * @file        LibCellBalance.h
  * @author      Norman
  * @version     v1.0
  * @date        2019/11/14
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Norman</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _LIB_CELL_BALANCE_H
#define _LIB_CELL_BALANCE_H

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

	
/* Public define ------------------------------------------------------------*/
	
/* Public typedef -----------------------------------------------------------*/

/* Public macro -------------------------------------------------------------*/

    
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
void libCellBalanceOpen(void);
void libCellBalanceClose(void);
void libCellBalanceStop(void);
void libCellBalanceStart(void);
bool libCellBalanceIsBalancing(void);


#ifdef __cplusplus
}
#endif

#endif /* _LIB_AVG_WEIGHT_H */

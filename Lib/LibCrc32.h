/**
  ******************************************************************************
  * @file        LibCrc32.h
  * @author      Norman
  * @version     v1.0
  * @date        2021/3/5
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Norman</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _LIB_CRC_32_H
#define _LIB_CRC_32_H

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

	
/* Public define ------------------------------------------------------------*/
	
/* Public typedef -----------------------------------------------------------*/

/* Public macro -------------------------------------------------------------*/

/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
unsigned int libCrc32Cal (const unsigned char *buf, int len, unsigned int init);


#ifdef __cplusplus
}
#endif

#endif /* _LIB_CRC_32_H */

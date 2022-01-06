/********************************************************************************
  * @file        LibDefine.h
  * @author      Norman
  * @version     v1.0
  * @date        2020/2/12
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 Norman</center></h2>
  *
  *
  ******************************************************************************
  */
#ifndef _LIB_DEFINE_H
#define _LIB_DEFINE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Public define ------------------------------------------------------------*/

/* Public typedef -----------------------------------------------------------*/
// - typedef -
#ifndef TRUE
	#define	TRUE	1
#endif

#ifndef FALSE
	#define	FALSE	0
#endif

typedef union{
	uint8_t	b[2];
	uint16_t i;
	int16_t si;
}tLibDefIbyte;

typedef union{
	uint8_t	b[4];
	uint16_t	i[2];
	uint32_t	l;
	int32_t	sl;
}tLibDefLbyte;

/* Public variables ---------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif 

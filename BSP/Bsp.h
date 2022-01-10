/**
  ******************************************************************************
  * @file        Bsp.h
  * @author      Golden Chen
  * @version     v0.0.1
  * @date        2022/01/06
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021</center></h2>
  *
  *
  ******************************************************************************
  */

// Select your board BSP
#define BSP_REV                                2	
	
#ifndef _BSP_H_
#define _BSP_H_

#ifdef __cplusplus
extern "C" {
#endif

#if   (BSP_REV == 1)
   #include "DavinciBsp_Rev1.h"
#elif (BSP_REV == 2)
   #include "DavinciBsp_Rev2.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* _BSP_H_ */

/************************ (C) COPYRIGHT *****END OF FILE****/    

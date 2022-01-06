/**
  ******************************************************************************
  * @file    smp_debug.h
  * @author  Golden
  * @version V0.0.1
  * @date    2021/10/14
  * @brief   Header for SMP debug message and API returned returned error code.  
  ****************************************************************************** 
  */

#ifndef __SMP_DEBUG__
#define __SMP_DEBUG__

/* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Save debug_msg log to SD Card file LOG.TXT */
#define SD_DEBUG    DISABLE

#define debug_msg(fmtstr, args...)             printf(fmtstr, ##args)

#define SMP_ERROR_BASE_NUM                     (0x0)///< STK error base	
	
#define SMP_SUCCESS                            (SMP_ERROR_BASE_NUM - 0)  ///< Successful command
#define SMP_ERROR_NOT_FOUND                    (SMP_ERROR_BASE_NUM - 1)  ///< Not found or unknown error 
#define SMP_ERROR_NOT_SUPPORTED                (SMP_ERROR_BASE_NUM - 2)  ///< Not supported
#define SMP_ERROR_INVALID_PARAM                (SMP_ERROR_BASE_NUM - 3)  ///< Invalid Parameter
#define SMP_ERROR_INVALID_STATE                (SMP_ERROR_BASE_NUM - 4)  ///< Invalid state, operation disallowed in this state
#define SMP_ERROR_INVALID_LENGTH               (SMP_ERROR_BASE_NUM - 5)  ///< Invalid Length
#define SMP_ERROR_INVALID_FLAGS                (SMP_ERROR_BASE_NUM - 6) ///< Invalid Flags
#define SMP_ERROR_INVALID_DATA                 (SMP_ERROR_BASE_NUM - 7) ///< Invalid Data
#define SMP_ERROR_INVALID_ADDR                 (SMP_ERROR_BASE_NUM - 8) ///< Bad Memory Address
#define SMP_ERROR_DATA_SIZE                    (SMP_ERROR_BASE_NUM - 9) ///< Invalid Data size
#define SMP_ERROR_TIMEOUT                      (SMP_ERROR_BASE_NUM - 10) ///< Operation timed out
#define SMP_ERROR_NULL                         (SMP_ERROR_BASE_NUM - 11) ///< Null Pointer
#define SMP_ERROR_FORBIDDEN                    (SMP_ERROR_BASE_NUM - 12) ///< Forbidden Operation
#define SMP_ERROR_BUSY                         (SMP_ERROR_BASE_NUM - 13) ///< Busy
#define SMP_ERROR_RESOURCES                    (SMP_ERROR_BASE_NUM - 14) ///< Not enough resources for operation
#define SMP_ERROR_NOT_OPEN                     (SMP_ERROR_BASE_NUM - 15) ///< Some software equipment was not open.
#define SMP_ERROR_ACCESS                       (SMP_ERROR_BASE_NUM - 16) ///< Accress path error.
#define SMP_ERROR_MALLOC                       (SMP_ERROR_BASE_NUM - 17) ///< Memory allocate error.
#define SMP_ERROR_UNINIT                       (SMP_ERROR_BASE_NUM - 18) ///< UNINIT error.
#define SMP_ERROR_FULL                 		     (SMP_ERROR_BASE_NUM - 19) ///< Buffer full error.

#endif //__DEBUG_MODULE__

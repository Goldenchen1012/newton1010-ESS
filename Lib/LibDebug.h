#ifndef _LIB_DEBUG_H
#define	_LIB_DEBUG_H

#include <stdint.h>
//#include "sdk_config.h"

//#define __USE_FULL_ASSERT
/* Exported macro ------------------------------------------------------------*/
#ifdef  __USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports 
  *         the name of the source file and the source line number of the call 
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
#ifndef assert_param
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
#endif
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

#define tErrCode int8_t
#define RES_ERROR_BASE_NUM (tErrCode)(0x00)

/** @} */

#define RES_SUCCESS                           (RES_ERROR_BASE_NUM - 0)  ///< Successful command
#define RES_ERROR_NOT_FOUND                   (RES_ERROR_BASE_NUM - 1)  ///< Not found or unknown error 
#define RES_ERROR_NOT_SUPPORTED               (RES_ERROR_BASE_NUM - 2)  ///< Not supported
#define RES_ERROR_INVALID_PARAM               (RES_ERROR_BASE_NUM - 3)  ///< Invalid Parameter
#define RES_ERROR_INVALID_STATE               (RES_ERROR_BASE_NUM - 4)  ///< Invalid state, operation disallowed in this state
#define RES_ERROR_INVALID_LENGTH              (RES_ERROR_BASE_NUM - 5)  ///< Invalid Length
#define RES_ERROR_INVALID_FLAGS               (RES_ERROR_BASE_NUM - 6) ///< Invalid Flags
#define RES_ERROR_INVALID_DATA                (RES_ERROR_BASE_NUM - 7) ///< Invalid Data
#define RES_ERROR_INVALID_ADDR                (RES_ERROR_BASE_NUM - 8) ///< Bad Memory Address
#define RES_ERROR_DATA_SIZE                   (RES_ERROR_BASE_NUM - 9) ///< Invalid Data size
#define RES_ERROR_TIMEOUT                     (RES_ERROR_BASE_NUM - 10) ///< Operation timed out
#define RES_ERROR_NULL                        (RES_ERROR_BASE_NUM - 11) ///< Null Pointer
#define RES_ERROR_FORBIDDEN                   (RES_ERROR_BASE_NUM - 12) ///< Forbidden Operation
#define RES_ERROR_BUSY                        (RES_ERROR_BASE_NUM - 13) ///< Busy
#define RES_ERROR_RESOURCES                   (RES_ERROR_BASE_NUM - 14) ///< Not enough resources for operation
#define RES_ERROR_NOT_OPEN                    (RES_ERROR_BASE_NUM - 15) ///< Some software equipment was not open.
#define RES_ERROR_ACCESS                      (RES_ERROR_BASE_NUM - 16) ///< Accress path error.
#define RES_ERROR_MALLOC                      (RES_ERROR_BASE_NUM - 17) ///< Memory allocate error.
#define RES_ERROR_UNINIT                      (RES_ERROR_BASE_NUM - 18) ///< UNINIT error.
#define RES_ERROR_FULL                           (RES_ERROR_BASE_NUM - 19) ///< Buffer full error.
#define RES_ERROR_REINIT                       (RES_ERROR_BASE_NUM - 20) ///< 
#define RES_ERROR_INIT                             (RES_ERROR_BASE_NUM - 21) ///< 
#define RES_ERROR_EMPTY                             (RES_ERROR_BASE_NUM - 22) ///< Buffer empty error
#define RES_ERROR_FAIL				(RES_ERROR_BASE_NUM - 23)
#endif

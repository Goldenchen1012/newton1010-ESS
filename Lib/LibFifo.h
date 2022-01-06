/**
  ******************************************************************************
  * @file        LibFifo.h
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

#ifndef _LIB_FIFO_H_
#define _LIB_FIFO_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "LibDebug.h"
#include "sdk_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/

typedef int (*FIFO_DataInCBTypeDef)(void* Desc); ///< Command handler function prototype

typedef struct 
{
    __far uint8_t *BufferAddr;
    uint32_t BufferSize;
    uint32_t in;
    uint32_t out;
    uint8_t DataInCBFlag;
    FIFO_DataInCBTypeDef DataInCB;
    
} FIFO_HandleTypeDef;

typedef struct 
{
    uint8_t *BufferAddr;
    uint32_t BufferSize;
    uint32_t in;
    uint32_t out;
    uint8_t dataSize;
    uint8_t DataInCBFlag;
    FIFO_DataInCBTypeDef DataInCB;
    
} FIFO_SIZE_Handle_t;

typedef struct 
{
    uint32_t *BufferAddr;
    uint32_t BufferSize;
    uint32_t in;
    uint32_t out;
} FIFO_32_HandleTypeDef;

typedef struct 
{
    float *BufferAddr;
    uint32_t BufferSize;
    uint32_t in;
    uint32_t out;
} FIFO_Float_HandleTypeDef;

typedef struct
{
    uint32_t seq_num;
    uint16_t Lead_III;
    uint16_t Lead_II;
    uint16_t Lead_I;    
}Lead_HandleTypeDef;

typedef struct 
{
    Lead_HandleTypeDef *BufferAddr;
    uint32_t BufferSize;
    uint32_t in;
    uint32_t out;
} FIFO_ADC_HandleTypeDef;

typedef struct 
{
    void **BufferAddr;
    uint32_t BufferSize;
    uint32_t in;
    uint32_t out;
} FIFO_void_HandleTypeDef;

typedef struct
{
   int16_t      X;
   int16_t      Y;
   int16_t      Z;
} AXIS16_TypeDef;


typedef struct 
{
    AXIS16_TypeDef acc;
    uint32_t time;
}FIFO_3AXIS16_TypeDef;

#define FIFO_3AXIS16_TypeDef_SIZE sizeof(FIFO_3AXIS16_TypeDef)
typedef struct 
{
    FIFO_3AXIS16_TypeDef *BufferAddr;
    uint32_t BufferSize;
    uint32_t in;
    uint32_t out;
} FIFO_MOTION_3AXIS16_HandleTypeDef;

/* Public macro -------------------------------------------------------------*/
#define LIB_FIFO_DEF(id, Size, cb)    \
    static uint8_t id##_data[Size+1] = {0};                  \
    FIFO_HandleTypeDef id = {(__far uint8_t *)id##_data, Size+1, 0, 0, 0, cb}
                    
#define LIB_FIFO_STRCH_NOT_FOUND	0xFFFFFFFF

/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/



tErrCode FIFO_Init(__far FIFO_HandleTypeDef *Info);

tErrCode FIFO_WriteIn(__far FIFO_HandleTypeDef *Info, uint8_t value);

tErrCode FIFO_ReadOut(__far FIFO_HandleTypeDef *Info, __far uint8_t *value);

tErrCode FIFO_Check(__far FIFO_HandleTypeDef *Info, __far uint8_t *value, uint32_t index);

uint32_t FIFO_GetDataNum(__far FIFO_HandleTypeDef *Info);
uint32_t FIFO_GetFreeNum(__far FIFO_HandleTypeDef *Info);

__far uint8_t *FIFO_GetBufferAddress(__far FIFO_HandleTypeDef *Info);

void FIFO_Reset(__far FIFO_HandleTypeDef *Info, uint8_t clearFlg);
#if 0
tErrCode LibFifoStrncmp(FIFO_HandleTypeDef *Info, const uint32_t dataOffset, const uint8_t * cmpStr, uint32_t cmpStrSize);
uint32_t LibFifoStrchr(FIFO_HandleTypeDef *Info, const uint8_t tarChar);

//--------------------------FIFO_Float----------------------------------------------
tErrCode FIFO_Float_Init(FIFO_Float_HandleTypeDef *Info);
tErrCode FIFO_Float_WriteIn(FIFO_Float_HandleTypeDef *Info, float value);
tErrCode FIFO_Float_ReadOut(FIFO_Float_HandleTypeDef *Info, float *value);
tErrCode FIFO_Float_Check(FIFO_Float_HandleTypeDef *Info, float *value, uint32_t index);
uint32_t FIFO_Float_GetDataNum(FIFO_Float_HandleTypeDef *Info);
//--------------------------FIFO_32----------------------------------------------
tErrCode FIFO_32_WriteIn(FIFO_32_HandleTypeDef *Info, uint32_t value);
tErrCode FIFO_32_ReadOut(FIFO_32_HandleTypeDef *Info, uint32_t *value);
tErrCode FIFO_32_Check(FIFO_32_HandleTypeDef *Info, uint32_t *value, uint32_t index);
uint32_t FIFO_32_GetDataNum(FIFO_32_HandleTypeDef *Info);
//--------------------------FIFO_ADC----------------------------------------------
tErrCode FIFO_ADC_WriteIn(FIFO_ADC_HandleTypeDef *Info, Lead_HandleTypeDef value);
tErrCode FIFO_ADC_ReadOut(FIFO_ADC_HandleTypeDef *Info, Lead_HandleTypeDef *value);
uint32_t FIFO_ADC_GetDataNum(FIFO_ADC_HandleTypeDef *Info);
//--------------------------FIFO_void----------------------------------------------
tErrCode FIFO_void_WriteIn(FIFO_void_HandleTypeDef *Info, void *value);
itErrCodent FIFO_void_ReadOut(FIFO_void_HandleTypeDef *Info, void *value);
uint32_t FIFO_void_GetDataNum(FIFO_void_HandleTypeDef *Info);
//--------------------------FIFO_motion----------------------------------------------
tErrCode FIFO_3AXIS_WriteIn(FIFO_MOTION_3AXIS16_HandleTypeDef *Info, FIFO_3AXIS16_TypeDef *value);
tErrCode FIFO_3AXIS_ReadOut(FIFO_MOTION_3AXIS16_HandleTypeDef *Info, FIFO_3AXIS16_TypeDef *value);
uint32_t FIFO_3AXIS_GetDataNum(FIFO_MOTION_3AXIS16_HandleTypeDef *Info);
#endif
#ifdef __cplusplus
}
#endif

#endif /* _LIB_FIFO_H_ */


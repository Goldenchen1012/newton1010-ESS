/**
  ******************************************************************************
  * @file    LibFifo.c
  * @author  Johnny
  * @version 0.0
  * @date    2021/10/14
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  * 
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "LibFifo.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



tErrCode FIFO_Init(__far FIFO_HandleTypeDef *Info)
{
 //   Info->BufferAddr=Buf;
//    Info->BufferSize=BufSize;
//    Info->in=0;
//    Info->out=0;
    if(Info!=0){
        FIFO_Reset(Info,1);
        return RES_SUCCESS;
    }else{
        return RES_ERROR_INVALID_PARAM;
    }
}

uint32_t FIFO_GetDataNum(__far FIFO_HandleTypeDef *Info){
    if(Info->in==Info->out){
        Info->DataInCBFlag=0;
        return 0; //Buf empty
    }else if(Info->in>Info->out){
        return Info->in-Info->out; 
    }else{
        return (Info->BufferSize-Info->out)+(Info->in);
    }
}

uint32_t FIFO_GetFreeNum(__far FIFO_HandleTypeDef *Info){
	return (Info->BufferSize - 1) - FIFO_GetDataNum(Info);
}

tErrCode FIFO_WriteIn(__far FIFO_HandleTypeDef *Info, uint8_t value)
{
    if((Info->in+1)%Info->BufferSize==Info->out){
        return RES_ERROR_FULL; //Buf full
    }else{
        Info->BufferAddr[Info->in]=value;
        Info->in=(Info->in+1)%Info->BufferSize;
        if(Info->DataInCBFlag==0){
            Info->DataInCBFlag=1;
            if(Info->DataInCB)
                Info->DataInCB((void*)Info);
        }
        return RES_SUCCESS;
    }
}

tErrCode FIFO_ReadOut(__far FIFO_HandleTypeDef *Info, __far uint8_t *value)
{
    if(Info->in==Info->out){
        Info->DataInCBFlag=0;
        return RES_ERROR_EMPTY; //Buf empty
    }else{
        *value=Info->BufferAddr[Info->out];
        Info->out=(Info->out+1)%Info->BufferSize;
        return RES_SUCCESS;
    }
}

tErrCode FIFO_Check(__far FIFO_HandleTypeDef *Info, __far uint8_t *value, uint32_t index)
{
    if(Info->in==Info->out){
        return RES_ERROR_EMPTY; //Buf empty
    }else if(index>=FIFO_GetDataNum(Info)){
        return RES_ERROR_INVALID_PARAM; 
    }else{
        *value=Info->BufferAddr[(Info->out+index)%Info->BufferSize];
        return RES_SUCCESS;
    }
}

__far uint8_t *FIFO_GetBufferAddress(__far FIFO_HandleTypeDef *Info){
    return Info->BufferAddr;
}

void FIFO_Reset(__far FIFO_HandleTypeDef *Info, uint8_t clearFlg){
    if(clearFlg){
        memset(Info->BufferAddr,0,Info->BufferSize);
    }
    Info->in=0;
    Info->out=0;
    Info->DataInCBFlag=0;
}

#if 0
tErrCode LibFifoStrncmp(FIFO_HandleTypeDef *Info, const uint32_t dataOffset, const uint8_t * cmpStr, uint32_t cmpStrSize){
    uint32_t chkIndex;
	uint8_t data;
	if(FIFO_GetDataNum(Info) < cmpStrSize){
        return RES_ERROR_DATA_SIZE;
    }
	
	for(chkIndex = dataOffset; chkIndex < cmpStrSize; chkIndex++){
		FIFO_Check(Info, &data, chkIndex);
		if(data != cmpStr[chkIndex]){
            return RES_ERROR_INVALID_PARAM;
		}
	}
	
	return RES_SUCCESS;
}

uint32_t LibFifoStrchr(FIFO_HandleTypeDef *Info, const uint8_t tarChar){
    uint32_t chkIndex;
	char data;
	
	for(chkIndex = 0; chkIndex < Info->BufferSize; chkIndex++){
		FIFO_Check(Info, &data, chkIndex);
		if(data == tarChar){
            return chkIndex;
		}
	}
	
	return SMP_LIB_FIFO_STRCH_NOT_FOUND;
}

//---------------------SysLockCB Private API ----------------------------------
//--------------------------FIFO_Float----------------------------------------------
void FIFO_Float_Reset(FIFO_Float_HandleTypeDef *Info, uint8_t clearFlg){
    if(clearFlg){
        memset(Info->BufferAddr,0,Info->BufferSize*sizeof(float));
    }
    Info->in=0;
    Info->out=0;
}

tErrCode FIFO_Float_Init(FIFO_Float_HandleTypeDef *Info)
{
 //   Info->BufferAddr=Buf;
//    Info->BufferSize=BufSize;
//    Info->in=0;
//    Info->out=0;
    if(Info!=0){
        FIFO_Float_Reset(Info,1);
        return RES_SUCCESS;
    }else{
        return RES_ERROR_INVALID_PARAM;
    }
}

tErrCode FIFO_Float_WriteIn(FIFO_Float_HandleTypeDef *Info, float value)
{
    if((Info->in+1)%Info->BufferSize==Info->out){
        return RES_ERROR_FULL; //Buf full
    }else{
        Info->BufferAddr[Info->in]=value;
        Info->in=(Info->in+1)%Info->BufferSize;
        return RES_SUCCESS;
    }
}

tErrCode FIFO_Float_ReadOut(FIFO_Float_HandleTypeDef *Info, float *value)
{
    if(Info->in==Info->out){
        return RES_ERROR_EMPTY; //Buf empty
    }else{
        *value=Info->BufferAddr[Info->out];
        Info->out=(Info->out+1)%Info->BufferSize;
        return RES_SUCCESS;
    }
}

tErrCode FIFO_Float_Check(FIFO_Float_HandleTypeDef *Info, float *value, uint32_t index)
{
    if(Info->in==Info->out){
        return RES_ERROR_EMPTY; //Buf empty
    }else if(index>=Info->BufferSize){
        return RES_ERROR_INVALID_PARAM; 
    }else{
        *value=Info->BufferAddr[(Info->in+index)%Info->BufferSize];
        return RES_SUCCESS;
    }
}

uint32_t FIFO_Float_GetDataNum(FIFO_Float_HandleTypeDef *Info){
    if(Info->in==Info->out){
        return 0; //Buf empty
    }else if(Info->in>Info->out){
        return Info->in-Info->out; 
    }else{
        return (Info->BufferSize-Info->out)+(Info->in);
    }
}

//--------------------------FIFO_32----------------------------------------------
tErrCode FIFO_32_WriteIn(FIFO_32_HandleTypeDef *Info, uint32_t value)
{
    if((Info->in+1)%Info->BufferSize==Info->out){
        return RES_ERROR_FULL; //Buf full
    }else{
        Info->BufferAddr[Info->in]=value;
        Info->in=(Info->in+1)%Info->BufferSize;
        return RES_SUCCESS;
    }
}

tErrCode FIFO_32_ReadOut(FIFO_32_HandleTypeDef *Info, uint32_t *value)
{
    if(Info->in==Info->out){
        return RES_ERROR_EMPTY; //Buf empty
    }else{
        *value=Info->BufferAddr[Info->out];
        Info->out=(Info->out+1)%Info->BufferSize;
        return RES_SUCCESS;
    }
}

tErrCode FIFO_32_Check(FIFO_32_HandleTypeDef *Info, uint32_t *value, uint32_t index)
{
    if(Info->in==Info->out){
        return RES_ERROR_EMPTY; //Buf empty
    }else if(index>=Info->BufferSize){
        return RES_ERROR_INVALID_PARAM; 
    }else{
        *value=Info->BufferAddr[(Info->in+index)%Info->BufferSize];
        return RES_SUCCESS;
    }
}

uint32_t FIFO_32_GetDataNum(FIFO_32_HandleTypeDef *Info){
    if(Info->in==Info->out){
        return RES_ERROR_EMPTY; //Buf empty
    }else if(Info->in>Info->out){
        return Info->in-Info->out; 
    }else{
        return (Info->BufferSize-Info->out)+(Info->in);
    }
}

//--------------------------FIFO_void*----------------------------------------------
tErrCode FIFO_void_WriteIn(FIFO_void_HandleTypeDef *Info, void* value)
{
    if((Info->in+1)%Info->BufferSize==Info->out){
        return RES_ERROR_FULL; //Buf full
    }else{
        Info->BufferAddr[Info->in]=value;
        Info->in=(Info->in+1)%Info->BufferSize;
        return RES_SUCCESS;
    }
}

tErrCode FIFO_void_ReadOut(FIFO_void_HandleTypeDef *Info, void *value)
{
    if(Info->in==Info->out){
        value=value;
        return RES_ERROR_EMPTY; //Buf empty
    }else{
        value=Info->BufferAddr[Info->out];
        Info->out=(Info->out+1)%Info->BufferSize;
        return RES_SUCCESS;
    }
}

uint32_t FIFO_void_GetDataNum(FIFO_void_HandleTypeDef *Info){
    if(Info->in==Info->out){
        return 0; //Buf empty
    }else if(Info->in>Info->out){
        return Info->in-Info->out; 
    }else{
        return (Info->BufferSize-Info->out)+(Info->in);
    }
}

//--------------------------FIFO_3_AXIS----------------------------------------------
tErrCode FIFO_3AXIS_WriteIn(FIFO_MOTION_3AXIS16_HandleTypeDef *Info, FIFO_3AXIS16_TypeDef *value)
{
    if((Info->in+1)%Info->BufferSize==Info->out){
        return RES_ERROR_FULL; //Buf full
    }else{
        Info->BufferAddr[Info->in]=*value;
        Info->in=(Info->in+1)%Info->BufferSize;
        return RES_SUCCESS;
    }
}

tErrCode FIFO_3AXIS_ReadOut(FIFO_MOTION_3AXIS16_HandleTypeDef *Info, FIFO_3AXIS16_TypeDef *value)
{
    if(Info->in==Info->out){
        return RES_ERROR_EMPTY; //Buf empty
    }else{
        *value=Info->BufferAddr[Info->out];
        Info->out=(Info->out+1)%Info->BufferSize;
        return RES_SUCCESS;
    }
}

uint32_t FIFO_3AXIS_GetDataNum(FIFO_MOTION_3AXIS16_HandleTypeDef *Info){
    if(Info->in==Info->out){
        return 0; //Buf empty
    }else if(Info->in>Info->out){
        return Info->in-Info->out; 
    }else{
        return (Info->BufferSize-Info->out)+(Info->in);
    }
}

//--------------------------FIFO_Record----------------------------------------------
uint8_t* FIFO_Record_GetAddr(FIFO_SIZE_Handle_t *Info)
{
    if((Info->in+Info->dataSize)%Info->BufferSize==Info->out){
        return 0; //Buf full
    }else{
        return (uint8_t*)&Info->BufferAddr[Info->in];
//        Info->in=(Info->in+Info->dataSize)%Info->BufferSize;
//        return 0;
    }
}

uint32_t FIFO_Record_GetNum(FIFO_SIZE_Handle_t *Info){
    uint32_t size;
    if(Info->in==Info->out){
        size=0; //Buf empty
    }else if(Info->in>Info->out){
        size=Info->in-Info->out; 
    }else{
        size=(Info->BufferSize-Info->out)+(Info->in);
    }
    return size/Info->dataSize;
}


tErrCode FIFO_Record_Free(FIFO_SIZE_Handle_t *Info,uint16_t freeSize)
{
    uint32_t size=FIFO_Record_GetNum(Info);
    if(size==0){
        return RES_ERROR_EMPTY; //Buf empty
    }else if(size<=freeSize){
        Info->in=Info->out;
    }else{
        Info->out=(Info->out+(freeSize*Info->dataSize))%Info->BufferSize;
    }
    return RES_SUCCESS;
}

#endif

/**
  ******************************************************************************
  * @file    smp_fifo.c
  * @author  Randy Huang
  * @version V0.0.1
  * @date    23-December-2016
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 SMP</center></h2>
  *
  * 
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "smp_can_fifo.h"
#include "smp_debug.h"
#include <string.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int8_t smp_can_fifo_open(smp_can_fifo_t *p_fifo)
{
	if(p_fifo!=NULL){
		smp_can_fifo_clean(p_fifo);
	}else{
		return SMP_ERROR_NULL;
	}	
	
	return SMP_SUCCESS;
}

int8_t smp_can_fifo_push(smp_can_fifo_t *p_fifo, smp_can_package_t *pCanPkg)
{
	if((p_fifo->in+1)%p_fifo->buffer_size == p_fifo->out){
		return SMP_ERROR_FULL; //Buf full
	}else{
		memcpy(&p_fifo->buffer_addr[p_fifo->in] ,pCanPkg, sizeof(smp_can_package_t)); 
		p_fifo->in = (p_fifo->in + 1) % p_fifo->buffer_size;
	}
	return SMP_SUCCESS;
}

int8_t smp_can_fifo_pop(smp_can_fifo_t *p_fifo,smp_can_package_t *pCanPkg)
{
	if(p_fifo->in == p_fifo->out){
		return SMP_ERROR_RESOURCES; //Buf empty
	}else{
		memcpy(pCanPkg, &p_fifo->buffer_addr[p_fifo->out], sizeof(smp_can_package_t)); 
		p_fifo->out=(p_fifo->out+1)%p_fifo->buffer_size;
	}		
	return SMP_SUCCESS;
}
#if	0
int8_t smp_can_fifo_back(smp_can_fifo_t *p_fifo)
{
	if(p_fifo->in==p_fifo->out){
		return SMP_ERROR_RESOURCES; //Buf empty
	}else{
		p_fifo->in=(p_fifo->in-1)%p_fifo->buffer_size;
	}		
	return SMP_SUCCESS;
}

int8_t smp_can_fifo_check(smp_can_fifo_t *p_fifo, char *val, uint32_t index)
{
	if(p_fifo->in==p_fifo->out){
		return SMP_ERROR_RESOURCES; //Buf empty
	}else if(index>=p_fifo->buffer_size){
		return SMP_ERROR_INVALID_PARAM; 
	}else{
		*val=p_fifo->buffer_addr[(p_fifo->out+index)%p_fifo->buffer_size];
	}
	return SMP_SUCCESS;
}
#endif
int8_t smp_can_fifo_get_size(smp_can_fifo_t *p_fifo, uint16_t *size)
{
    if(p_fifo->in==p_fifo->out){
		return SMP_ERROR_RESOURCES; //Buf empty
	}else if(p_fifo->in>p_fifo->out){
		*size = p_fifo->in-p_fifo->out;
	}else{
		*size = (p_fifo->buffer_size-p_fifo->out)+(p_fifo->in);
	}	
	return SMP_SUCCESS;
}

int8_t smp_can_fifo_clean(smp_can_fifo_t *p_fifo)
{
	memset(p_fifo->buffer_addr,0,p_fifo->buffer_size);
	p_fifo->in=0;
	p_fifo->out=0;	
	return SMP_SUCCESS;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Simplo all right reserved *****END OF FILE****/

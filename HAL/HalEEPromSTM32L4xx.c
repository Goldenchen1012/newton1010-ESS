/**
  ******************************************************************************
  * @file        HalEEPromSTM32L4xx.c
  * @author      Johnny
  * @version     v0.0
  * @date        2021/9/7
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"
#include "LibDebug.h"
#include "halEeProm.h"

void appSerialCanDavinciSendTextMessage(uint8_t *str);
#define	HalRtcDebugMsg(str)	//appSerialCanDavinciSendTextMessage(str)


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
static FLASH_EraseInitTypeDef EraseInitStruct;

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;
  
  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }
  
  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;
  
  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
  	/* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
  	/* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }
  
  return bank;
}

#define	ERASE_TRY_TIMES	2
tErrCode HalEePromErase(tHalEeProm *pEeProm)
{
	static uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;
	static uint32_t Address = 0, PAGEError = 0;
	__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;
	uint8_t		trytimes;
  for(trytimes=0; trytimes<ERASE_TRY_TIMES; trytimes++)
  { 
	HAL_FLASH_Unlock();

  	/* Clear OPTVERR bit set on virgin samples */
  	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_BSY | FLASH_FLAG_OPTVERR | 
						FLASH_FLAG_PROGERR | FLASH_FLAG_PGAERR |
						FLASH_FLAG_OPERR 
						); 
  	/* Get the 1st page to erase */
  	FirstPage = GetPage(pEeProm->StartAddress);
  	/* Get the number of pages to erase from 1st page */
  	NbOfPages = GetPage(pEeProm->StartAddress + pEeProm->Length) - FirstPage + 1;
  	/* Get the bank */
  	BankNumber = GetBank(pEeProm->StartAddress);
  	/* Fill EraseInit structure*/
  	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  	EraseInitStruct.Banks       = BankNumber;
  	EraseInitStruct.Page        = FirstPage;
  	EraseInitStruct.NbPages     = NbOfPages;
  	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
  	{
  		if(trytimes == (ERASE_TRY_TIMES-1))
  		{
	 		return RES_ERROR_FAIL;
	 	}
	}
  }
	return RES_SUCCESS;
}
tErrCode HalEePromWrite(tHalEeProm *pEeProm)
{
	uint8_t		u8;
	uint32_t	i,address;
	union{
		uint8_t		b[8];
		uint64_t	dat;
	}U64Dat;

	HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS); 

	address = pEeProm->StartAddress;
	if((address&0x07))
		return RES_ERROR_INVALID_PARAM;
	for(i=0; i<pEeProm->Length; )
	{
		for(u8=0; u8<8; u8++)
			U64Dat.b[u8] = pEeProm->pDataBuffer[i++];
		//dat = *(uint64_t *)&pEeProm->pDataBuffer[i];
	    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, U64Dat.dat) != HAL_OK)
	    	return RES_ERROR_FAIL;
		address += 8;
	}
	return RES_SUCCESS;	
}
tErrCode HalEePromRead(tHalEeProm *pEeProm)
{
	uint8_t		AligmentStatus;
	uint8_t		u8;
	uint32_t	i,address;
	union{
		uint8_t		b[4];
		uint32_t	dat;
	}U32Dat;
	//char	str[100];
	
	address = pEeProm->StartAddress;
	if((address&0x03))
		return RES_ERROR_INVALID_PARAM;
		
	if(((uint32_t)pEeProm->pDataBuffer & 0x07) == 0)
		AligmentStatus = 1;
	else
		AligmentStatus = 0;	
//	sprintf(str,"%.8lX %.8lX %d",address, pEeProm->StartAddress,pEeProm->Length);
//	HalRtcDebugMsg(str);
	
	for(i=0; i<pEeProm->Length; )
	{
		U32Dat.dat = *(__IO uint32_t *)(address);
		address += 4;
		for(u8=0; u8<4; u8++)
			pEeProm->pDataBuffer[i++] = U32Dat.b[u8];
	}	
	return RES_SUCCESS;
}



/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



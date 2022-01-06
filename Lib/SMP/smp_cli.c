/**
  ******************************************************************************
  * @file    smp_cli.c
  * @author  Golden Chen
  * @version 
  * @date    2021/09/14
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 SMP</center></h2>
  *
  * 
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "smp_cli.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CLI_HandleTypeDef  *SMP_CLI_DescReg[SMP_CLI_RERISTER_MAX];

CLI_HandleTypeDef* NowCliDesc;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
int SMP_CLI_Init(CLI_HandleTypeDef *pCliDesc)
{
	if(pCliDesc==NULL) return -1;
	
	uint8_t i;
	for(i=0;i<=SMP_CLI_RERISTER_MAX;i++){
		if(SMP_CLI_DescReg[i]==0){
			if(smp_fifo_open(pCliDesc->pFifoDesc)==0){
				pCliDesc->index=i;
				SMP_CLI_DescReg[i]=pCliDesc;
				return 0;
			}else{
				return -2; //FIFO init fail
			}
		}
	}
	return -1; //CLI register full
}


int SMP_CLI_DataIn(CLI_HandleTypeDef *pCliDesc,char *Buf, uint32_t BufSize){
	uint32_t i;
	for(i=0;i<BufSize;i++){
		if(Buf[i]!=SMP_CLI_SYMBOL_DELET){
			if(smp_fifo_push(pCliDesc->pFifoDesc,Buf[i])!=0){
				smp_fifo_back(pCliDesc->pFifoDesc);
				return -1; //FIFO buffer full
			}
		}else{
			if(smp_fifo_back(pCliDesc->pFifoDesc)!=0){
				return -1; //FIFO empty
			}
		}
		fprintf((FILE*)pCliDesc->printf,"%c",Buf[i]);
	}

	return 0;
}

int SMP_CLI_ErrorHandle(CLI_HandleTypeDef *pCliDesc,int ErrorCode){

	switch(ErrorCode){
		case SMP_CLI_ERROR_LINE_FEED:
			SMP_CLI_PRINT("\r\n");
			break;
		case SMP_CLI_ERROR_COMMAND_NOT_FOUND:
			SMP_CLI_PRINT(pCliDesc->SymbolWrongCommand,pCliDesc->pFifoDesc->buffer_addr);
			break;
		case SMP_CLI_ERROR_DISCRIMINATION:
		case SMP_CLI_ERROR_CALLBACK:
			SMP_CLI_PRINT("Command parameter error '%s'\r\n",pCliDesc->pFifoDesc->buffer_addr);
			break;
		case SMP_CLI_ERROR_INVALID_PARAMETER:
			SMP_CLI_PRINT(pCliDesc->SymbolInvalidCommand,pCliDesc->pFifoDesc->buffer_addr);
			break;
	}
	return 0;
}


int SMP_CLI_handleCommand(CLI_HandleTypeDef *pCliDesc){
	char * data;
	int errorCode;
	uint8_t	TblIndex;
	uint32_t CmdIndex=0;
	uint16_t size = 0;
	
	
	data=pCliDesc->pFifoDesc->buffer_addr;
	for(TblIndex=0;TblIndex<pCliDesc->CmdTblSize;TblIndex++){
		//Check table name
		if(pCliDesc->pCmdTbl[TblIndex]!=0){
//			if(strncmp(data,pCliDesc->pCmdTbl[TblIndex][0].pName,strlen(pCliDesc->pCmdTbl[TblIndex][0].pName))==0){
//				data+=strlen(pCliDesc->pCmdTbl[TblIndex][0].pName);
//				if(strncmp(data,pCliDesc->SymbolDiscrimination,strlen(pCliDesc->SymbolDiscrimination))==0){
//					data+=strlen(pCliDesc->SymbolDiscrimination);
					CmdIndex=1;

					while(pCliDesc->pCmdTbl[TblIndex][CmdIndex].pName!=0){
						if(strncmp(data,pCliDesc->pCmdTbl[TblIndex][CmdIndex].pName,strlen(pCliDesc->pCmdTbl[TblIndex][CmdIndex].pName))==0){
							data+=strlen(pCliDesc->pCmdTbl[TblIndex][CmdIndex].pName);
							//Save command string for ACK in callback.
							pCliDesc->CommandStrSave=pCliDesc->pCmdTbl[TblIndex][CmdIndex].pName;
							
							if(strncmp(data,pCliDesc->SymbolDiscrimination,strlen(pCliDesc->SymbolDiscrimination))==0){
								data+=strlen(pCliDesc->SymbolDiscrimination);
							}
							//Check command help
							if(strncmp(data,pCliDesc->SymbolHelp,strlen(pCliDesc->SymbolHelp))==0){
								SMP_CLI_PRINT("%s\r\n",pCliDesc->pCmdTbl[TblIndex][CmdIndex].pDesc);	
								return SMP_CLI_SUCCESSFUL;								
							}else{
								errorCode = pCliDesc->pCmdTbl[TblIndex][CmdIndex].pfFunc(data);
								switch(errorCode){
									case 0 	: return SMP_CLI_SUCCESSFUL;
									case -1 : return SMP_CLI_ERROR_INVALID_PARAMETER;
									default : return SMP_CLI_ERROR_CALLBACK;
								}						
							}
						}
						CmdIndex++;
					}
//				}else{
//					//Error
//					return SMP_CLI_ERROR_DISCRIMINATION;
//				}
//			} 
			if(strncmp(data,pCliDesc->CmdHelp,strlen(pCliDesc->CmdHelp))==0){
					//Check Tab Help
				CmdIndex=0;
				while(pCliDesc->pCmdTbl[TblIndex][CmdIndex].pName!=0){		
					SMP_CLI_PRINT("%-20s",pCliDesc->pCmdTbl[TblIndex][CmdIndex].pName);				
					SMP_CLI_PRINT("%s\r\n",pCliDesc->pCmdTbl[TblIndex][CmdIndex].pDesc);	
					CmdIndex++;							
				}
				return SMP_CLI_SUCCESSFUL;		
			}
		}
	}
	smp_fifo_get_size(pCliDesc->pFifoDesc, &size);
	if(size==1)
		return SMP_CLI_ERROR_LINE_FEED;
	
	//Error command is not support

	return SMP_CLI_ERROR_COMMAND_NOT_FOUND;

}

int SMP_CLI_Handle(void){
	uint8_t i;
	for(i=0;i<SMP_CLI_RERISTER_MAX;i++){
		if(SMP_CLI_DescReg[i]!=0){
		    if(strchr(SMP_CLI_DescReg[i]->pFifoDesc->buffer_addr,SMP_CLI_DescReg[i]->SymbolEnd)!=0)
		    {
				NowCliDesc=SMP_CLI_DescReg[i];
		        SMP_CLI_ErrorHandle(SMP_CLI_DescReg[i],SMP_CLI_handleCommand(SMP_CLI_DescReg[i]));
				smp_fifo_clean(SMP_CLI_DescReg[i]->pFifoDesc);
				SMP_PRINT(SMP_CLI_SYMBOL_PROMPT);
		    }
		}
	}
    return 0;
}

int SMP_CLI_AddTable(CLI_HandleTypeDef *pCliDesc,SMP_CLI_CMD_ENTRY *pTbl){
	if((pCliDesc==NULL) || (pTbl==NULL)) return -1;
	
	uint16_t i;
	for(i=0;i<pCliDesc->CmdTblSize;i++){
		if(pCliDesc->pCmdTbl[i]==0){
		    pCliDesc->pCmdTbl[i]=pTbl;		    
		    return 0;
		}
	}
	return -1; //Command table buffer full
}


//---------------------SysLockCB Private API ----------------------------------


/************************ (C) COPYRIGHT Simplo all right reserved *****END OF FILE****/

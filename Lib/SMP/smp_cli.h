
#ifndef __SMP_CLI_h
#define __SMP_CLI_h

#include <stdint.h>
#include "smp_fifo.h"

//---------------------Default Macro -----------------------------
#define SMP_CLI_SYMBOL_DISCRIMINATION	" "
#define SMP_CLI_SYMBOL_END				'\r'//0x0A
#define SMP_CLI_SYMBOL_HELP				"?"
#define SMP_CLI_CMD_HELP				"help"
#define SMP_CLI_SYMBOL_WRONG_COMMAND	"Error: Command is not found ! Use help\r\n"
#define SMP_CLI_SYMBOL_INVALID_COMMAND	"Invalid Parameter\r\n"
#define SMP_CLI_SYMBOL_ACK				"CLI=%s:"
#define SMP_CLI_SYMBOL_DELET			'\b'
#define SMP_CLI_SYMBOL_PROMPT			"> "
//---------------------SysLockCB Debug Definition -----------------------------
#define SMP_CLI_RERISTER_MAX 				3

#define SMP_CLI_SUCCESSFUL 					0
#define SMP_CLI_ERROR_LINE_FEED				1
#define SMP_CLI_ERROR_COMMAND_NOT_FOUND		-1
#define SMP_CLI_ERROR_CALLBACK				-2
#define SMP_CLI_ERROR_DISCRIMINATION		-3
#define SMP_CLI_ERROR_INVALID_PARAMETER		-4
//---------------------SysFuncCB Debug Definition -----------------------------
#define SMP_FUNC_SUCCESSFUL 				0
#define SMP_FUNC_ERROR_LINE_FEED			1
#define SMP_FUNC_ERROR_COMMAND_NOT_FOUND	-1
#define SMP_FUNC_ERROR_CALLBACK				-2
#define SMP_FUNC_ERROR_DISCRIMINATION		-3
#define SMP_FUNC_ERROR_INVALID_PARAMETER	-4
//---------------------SysLockCB Global Variables -----------------------------
//---------------------SysLockCB Prototype Declaration  -----------------------
typedef int (*SMP_CLI_CMD_FUN)(char* strCmd); ///< Command handler function prototype

typedef struct 
{
    void *handle;
    void  ( *pPutc)(void *handle,int ch);     /*!<       */

} printf_putc_HandleTypeDef;

typedef struct 
{
    char    *pName; ///< command's module name
    SMP_CLI_CMD_FUN  pfFunc; ///< command table
    char    *pDesc;
}
SMP_CLI_CMD_ENTRY;


typedef struct 
{
	uint8_t	index;
	char *descriptor;
	char *SymbolDiscrimination;
	char *SymbolHelp;
	char *CmdHelp;	
	char *SymbolWrongCommand;
	char *SymbolInvalidCommand;
	char *SymbolACK;
	char *CommandStrSave;
	int	 SymbolEnd;
    SMP_CLI_CMD_ENTRY **pCmdTbl;
    uint16_t	CmdTblSize;
    smp_fifo_t *pFifoDesc;
	printf_putc_HandleTypeDef *printf;
} CLI_HandleTypeDef;


#define SMP_CLI_CMD_BEGIN(tbl,desc) SMP_CLI_CMD_ENTRY (tbl)[]={{#tbl,0,desc}, ///< begin a command table
#define SMP_CLI_CMD_ITEM(cmd,func,desc)  {(cmd), (func), (desc)}, ///< insert a command item in command table
#define SMP_CLI_CMD_END()    {0,0,NULL}}; ///< end a command table

#define SMP_FUNC_BEGIN(tbl,desc) SMP_CLI_CMD_ENTRY (tbl)[]={{#tbl,0,desc}, ///< begin a command table
#define SMP_FUNC_ITEM(cmd,func,desc)  {(cmd), (func), (desc)}, ///< insert a command item in command table
#define SMP_FUNC_END()    {0,0,NULL}}; ///< end a command table

extern CLI_HandleTypeDef* NowCliDesc;
#define	SMP_PRINT(fmtstr, args...) fprintf((FILE*)NowCliDesc->printf,fmtstr, ##args)
#define	SMP_CLI_PRINT(fmtstr, args...) fprintf((FILE*)NowCliDesc->printf,fmtstr, ##args)
#define SMP_CLI_PRINT_ACK(fmtstr, args...)	fprintf((FILE*)NowCliDesc->printf,NowCliDesc->SymbolACK,NowCliDesc->CommandStrSave); \
											fprintf((FILE*)NowCliDesc->printf,fmtstr, ##args)
//---------------------SysLockCB Public API  ----------------------------------
int SMP_CLI_DataIn(CLI_HandleTypeDef *pCliDesc,char *Buf, uint32_t BufSize);
int SMP_CLI_Init(CLI_HandleTypeDef *pCliDesc);
int SMP_CLI_Handle(void);
int SMP_CLI_AddTable(CLI_HandleTypeDef *pCliDesc,SMP_CLI_CMD_ENTRY *pTbl);

#endif /* __SMP_CLI_h */

/************************ (C) COPYRIGHT Simplo all right reserved *****END OF FILE****/

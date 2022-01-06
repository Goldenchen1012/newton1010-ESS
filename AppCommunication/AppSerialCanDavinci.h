/**
  ******************************************************************************
  * @file        AppSerialCanDavinci.h
  * @author      Johnny
  * @version     v0.0.1
  * @date        2021/10/20
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _APP_SERIAL_CAN_DAVINCI_H_
#define _APP_SERIAL_CAN_DAVINCI_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "smp_can_fifo.h"
#include "SmpCanBusProtocol.h"
#include "SmpParameterID.h"
#include "AppProject.h"

#ifdef __cplusplus
extern "C" {
#endif
/* Public typedef -----------------------------------------------------------*/
typedef void (*tAppSerialPacketExe)(smp_can_package_t *pCanPkg);


	
typedef struct {
  uint32_t 	canid;
  uint32_t	mask;
  tAppSerialPacketExe fun;
} tAppSerialSmpCanDecode;

/* Public define ------------------------------------------------------------*/

#define	SCU_ID()				1
//appProjectGetScuId()
#define	CHECK_SMP_CAN_FUN		(0x0fu << 25)
#define	CHECK_SMP_CAN_OBJ		(0xffu << 10)
#define	CHECK_SMP_CAN_SUB		(0x3ff)

#define	CHECK_SMP_CAN_SCU_ID	(0x07fu << 18)
	
#define	SMP_CAN_GET_FUN(id)			((id >> 25) & 0x0f)
#define	SMP_CAN_GET_SCU_ID(id)		((id >> 18) & 0x7f)
#define	SMP_CAN_GET_OBJ_INDEX(id)	((id >> 10) & 0xff)
#define	SMP_CAN_GET_SUB_INDEX(id)	(id & 0x3ff)

	
#define	MAKE_SMP_CAN_ID(fun, bmuid, obj, sub)	(((uint32_t)fun<<25)|((uint32_t)bmuid<<18)|((uint32_t)obj<<10)|(uint32_t)sub)

#define SMP_CAN_DECODE_CMD_START(name) const tAppSerialSmpCanDecode name[] = {
#define SMP_CAN_DECODE_CMD_CONTENT(canid, mask, fun) {canid, mask, (tAppSerialPacketExe)fun},
#define SMP_CAN_DECODE_CMD_END() {0, 0, 0}}


/* Public macro -------------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/
uint8_t appSerialCanDavinciIsCorrectScuId(smp_can_package_t *pCanPkg);
void appSerialCanDavinciOpen(void);
void appSerialCanDavinciClose(void);
void appSerialCanDavinciPutPkgToCanFifo(smp_can_package_t *pCanPkg);
void appSerialCanDavinciSendTextMessage(char *msg);


#ifdef __cplusplus
}
#endif


	

#endif /* _APP_SERIAL_UART_H_ */

/************************ (C) COPYRIGHT Johnny Wang *****END OF FILE****/    



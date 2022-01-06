/**
  ******************************************************************************
  * @file        LibSerial.h
  * @author      Norman
  * @version     v1.0
  * @date        2019/9/12
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Norman</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _HAL_SERIAL_H_
#define _HAL_SERIAL_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "datatype.h"
#include "HalSerial.h"
#include "LibRegister.h"


#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/
enum{
  LIB_SERIAL_EVT_RX_DATE_UPDATED = 0,
  LIB_SERIAL_EVT_RX_BUF_FULL
  LIB_SERIAL_EVT_TX_BUF_EMPTY,
  LIB_SERIAL_EVT_MAX
};

/* Public typedef -----------------------------------------------------------*/
#define tSerialEvtHandler tLibRegisterEvtHandler

typedef struct {
  tHalSerialPar par;
  tHalSerialFun fun;
} tLibSerial;

/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
tErrCode LibSerialOpen(tHalSerial *port, tSerialEvtHandler evtHandler, uint8_t id);
tErrCode LibSerialClose(tHalSerial *port);
tErrCode LibSerialRxDequeue(tHalSerial *port, uint8_t *value);
tErrCode LibSerialTxEnqueue(tHalSerial *port, uint8_t value);
tErrCode LibSerialGetRxData(tHalSerial *port, uint8_t *value, uint32_t index);
uint32_t LibSerialGetRxNum(tHalSerial *port);


#ifdef __cplusplus
}
#endif

#endif /* LIB_AFE_H_ */

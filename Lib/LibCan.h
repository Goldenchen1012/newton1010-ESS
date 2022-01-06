/**
  ******************************************************************************
  * @file        LibCan.h
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

#ifndef _LIB_CAN_H
#define _LIB_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "HalCan.h"
#include "LibDebug.h"
#include "LibRegister.h"

#include <stdint.h>
#include <stdbool.h>

/* Public define ------------------------------------------------------------*/
#define tLibCanEvtHandler tLibRegisterEvtHandler

/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/
enum{
  LIB_CAN_EVT_RX_DATE_UPDATED = 0,
  LIB_CAN_EVT_RX_BUF_FULL,
  LIB_CAN_EVT_TX_BUF_EMPTY,
  LIB_CAN_EVT_MAX
};

/* Public typedef -----------------------------------------------------------*/


typedef struct {
  uint8_t Id;
  tHalCanPort *hal;
} tLibCan;

/* Public macro -------------------------------------------------------------*/
#define LIB_CAN_PORT_CREATE(name, ch) const tLibCan name = {0, &mCanPort_##ch}
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
tErrCode LibCanOpen(tLibCan *port, tLibCanEvtHandler evtHandler);
tErrCode LibCanClose(tLibCan *port);
tErrCode LibCanTxSend(tLibCan *port, uint32_t id, uint8_t dlc, uint8_t *buf);
bool LibCanIsTxBufEmpty(tLibCan *port);
bool LibCanIsRxBufEmpty(tLibCan *port);


#ifdef __cplusplus
}
#endif

#endif /* _LIB_CAN_H */

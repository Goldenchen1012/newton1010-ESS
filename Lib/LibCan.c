/**
  ******************************************************************************
  * @file        LibCan.c
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

/* Includes ------------------------------------------------------------------*/
#include "LibDebug.h"
#include "HalCan.h"
#include "LibCan.h"

#include <stdint.h>
#include <stdbool.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
tErrCode LibCanClose(tLibCan *port){
    port->hal->fun.close();
    return RES_SUCCESS;
}

tErrCode LibCanOpen(tLibCan *port, tLibCanEvtHandler evtHandler){
    evtHandler = evtHandler; 
    port->hal->fun.open();
    return RES_SUCCESS;
}

tErrCode LibCanTxSend(tLibCan *port, uint32_t id, uint8_t dlc, uint8_t *buf){
    port->hal->fun.packetSent(id, dlc, buf);
    return RES_SUCCESS;
}

bool LibCanIsTxBufEmpty(tLibCan *port){
    return port->hal->fun.isTxBufEmpty();
}

bool LibCanIsRxBufEmpty(tLibCan *port){
    return port->hal->fun.isRxBufEmpty(port->hal->rxData);
}


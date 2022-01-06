/**
  ******************************************************************************
  * @file        LibSerialDecoder.h
  * @author      Norman
  * @version     v1.0
  * @date        2019/9/11
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Norman</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _LIB_SERIAL_DECODER_H_
#define _LIB_SERIAL_DECODER_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "datatype.h"
#include "LibSerial.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Public define ------------------------------------------------------------*/

/* Public typedef -----------------------------------------------------------*/
typedef bool (*tSerialJudgePacket)(tLibSerial *port); 
typedef uint8_t (*tSerialJudgeId)(tLibSerial *port); 
typedef void (*tSerialParser)(uint16_t len, uint8_t data);

typedef struct {
  uint8_t id;
  tSerialParser fun;
} tLibSerialDecoderProtocolItem;

typedef struct {
  tSerialJudgePacket judgePacket;
  tSerialJudgeId judgeId;
  tLibSerialDecoderProtocolItem *parserItem;
} tLibSerialDecoderProtocol;

typedef struct {
  tLibSerialDecoderProtocol *protocol;
  tLibSerial *port;
} tLibSerialDecoder;

/* Public macro -------------------------------------------------------------*/
#define SERIAL_PROTOCOL_ITEM_BEGIN(protocolName) const tLibSerialDecoderProtocolItem protocolName##_Item[] = {
#define SERIAL_PROTOCOL_ITEM_ADD(id, fun) {id, fun},
#define SERIAL_PROTOCOL_ITEM_END(protocolName, JPFun, JIFun) }; \
        const tLibSerialDecoderProtocol protocolName = { .judgePacket = JPFun, \
                                                         .judgeId = JIFun, \
                                                         .parserItem = protocolName##_Item};

#define SERIAL_DECODER_DEF(name, protocol, port) tLibSerialDecoder name = {.protocol = protocol, \
                                                                          .port = port};
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* LIB_AFE_H_ */

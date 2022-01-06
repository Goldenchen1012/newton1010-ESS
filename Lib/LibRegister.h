
#ifndef _LIB_REGISTER_H
#define _LIB_REGISTER_H

#include "sdk_config.h"
#include "LibDebug.h"

#include <stdint.h>
#include <stdbool.h>
//---------------------SysLockCB Debug Definition -----------------------------
#define LibRegisterAdd(head, handler, dest) _LibRegisterAdd((__far tLibRegister *)head, handler, (__far void *)dest)
#define LibRegisterRm(head, handler, dest) _LibRegisterRm((__far tLibRegister *)head, handler, (__far void *)dest)
#define LibRegisterTypeHandlerExe(head, evt, data) _LibRegisterTypeHandlerExe((__far tLibRegister *)head, evt, (__far void *)data)
#define LibRegisterGetMemberAddr(head, number) _LibRegisterGetMemberAddr((__far tLibRegister *)head, number)
#define LibRegisterIsMemberNull(head) _LibRegisterIsMemberNull((__far tLibRegister *)head)
//---------------------SysLockCB Global Variables -----------------------------
//---------------------SysLockCB Prototype Declaration  -----------------------    
typedef void (* tLibRegisterEvtHandler)(__far void *dest, uint16_t evt, __far void *data);

typedef struct 
{
	__far void *dest;
    tLibRegisterEvtHandler handler;
    __far void *next;
}tLibRegisterMember;

typedef struct 
{
    __far tLibRegisterMember *next;
	__far tLibRegisterMember *executing;
	bool removeExecutingHandlerFlag;
}tLibRegister;

//---------------------SysLockCB Public API  ----------------------------------
tErrCode _LibRegisterAdd(__far tLibRegister *head, tLibRegisterEvtHandler handler, __far void *dest); //Store member to heap.
tErrCode _LibRegisterRm(__far tLibRegister *head,tLibRegisterEvtHandler handler, __far void *dest); //When "head" is 0,this API will remove all member.
tErrCode _LibRegisterTypeHandlerExe(__far tLibRegister *head, uint16_t evt, __far void * data);
__far tLibRegisterMember *_LibRegisterGetMemberAddr(__far tLibRegister *head, uint16_t number);
bool _LibRegisterIsMemberNull(__far tLibRegister *head);
#endif /* _LIB_REGISTER_H */


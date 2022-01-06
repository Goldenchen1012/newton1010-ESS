#ifndef LIB_STATE_MACHINE_H_
#define LIB_STATE_MACHINE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "LibRegister.h"
#include <stdint.h>
#include <stdbool.h>
/* Public define ------------------------------------------------------------*/

#define STATE_SYS_EVT_INITIAL 0
#define STATE_SYS_EVT_UNINITIAL 1
/* Public typedef -----------------------------------------------------------*/
typedef struct{
  uint8_t id;
  tLibRegisterEvtHandler handler;
}tLibStateMachineMember;

typedef struct{
  __far void **dest;
  const tLibStateMachineMember *memberTab;
  const tLibStateMachineMember **nowMember;
}tLibStateMachine;

typedef void (* tLibStateMachineEvtHandler)(__far void *dest, uint16_t evt, __far void *data);
/* Public macro -------------------------------------------------------------*/ 
#define LIB_STATE_MACHINE_TAB_CREATE_START(name) const tLibStateMachineMember name[] = {
#define LIB_STATE_MACHINE_TAB_CREATE_ADD(id, pHandler) {id, (tLibRegisterEvtHandler)pHandler},
#define LIB_STATE_MACHINE_TAB_CREATE_END {0xFF, 0}};
#define LIB_STATE_MACHINE_TAB_EXTERN(name) extern const tLibStateMachineMember name[]

#define LIB_STATE_MACHINE_CREATE(name, pMemberTab) \
    static __far void *name##_DEST; \
    static tLibStateMachineMember *name##_NOW_MEMBER; \
	static const tLibStateMachine name = {(__far void **)&name##_DEST, pMemberTab, ( const tLibStateMachineMember **)&name##_NOW_MEMBER}
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/

void LibStateMachineEvtExe(const tLibStateMachine *pStateMachine, uint16_t evt, __far void *vDataPtr);
void LibStateMachineSetState(const tLibStateMachine *pStateMachine, uint8_t newState);
void LibStateMachineSetStateInit(const tLibStateMachine *pStateMachine, __far void *dest, uint8_t initState);
uint8_t LibStateMachineGetState(const tLibStateMachine *pStateMachine);
bool LibStateMachineIsInit(const tLibStateMachine *pStateMachine);

#ifdef __cplusplus
}
#endif

#endif 

#ifndef _DMX_OUTPUTS_H_
#define _DMX_OUTPUTS_H_
#include <DMX.h>
#include <rtos.h>
#include <resources.h>

#define SERVO_UPDATE_TIME 	5		//unit 1/120th of a second
#define TRIAC_UPDATE_TIME 	10

void initTriacs();
void initPCA();

void setTriacs(char*, size_t, uint64_t);
void setPCA(char*, size_t, uint64_t);
void triacInterrupt(void);

extern uint8_t ACLockout;
inline void triacLockout(int i){
	ACLockout |= 1 << i;
}
inline void triacUnlock(int i){
	ACLockout &=~ 1 << i;
}
#endif
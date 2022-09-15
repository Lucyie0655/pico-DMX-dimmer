#ifndef _DMX_OUTPUTS_H_
#define _DMX_OUTPUTS_H_
#include <DMX.h>
#include <rtos.h>
#include <resources.h>

#define SERVO_UPDATE_TIME 	10		//unit 1/120th of a second (83ms)
#define TRIAC_UPDATE_TIME 	4		//we don't want the update times to sync up so that we can process stuff faster (175ms)
//the triac update could technically be absurdly long because of the filiment hysteresis (https://en.wikipedia.org/wiki/Hysteresis)

void initTriacs();
void initPCA();

void setTriacs(char*, size_t, uint64_t);
void setPCA(char*, size_t, uint64_t);

#endif
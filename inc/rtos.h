#ifndef _DMX_DIMMER_RTOS_H_
#define _DMX_DIMMER_RTOS_H_
#include <stdint.h>
#include <stdio.h>

#ifdef DEBUG

#define dbg_printf printf
#else
#define dbg_printf(s, ...)	//(nothing here)
#endif

#define TSK_SWITCH	100		//task switch timer (in Hz)
#define __irq __attribute__((optimize(3)))		//any other attributes we need for all interrupts can go here
#define __nvolvar __attribute__((section(".nonvolatile.data")))		//non-volatile variables
#define __volvar  __attribute__((section(".data")))					//volatile variables (default)

#define NO_TSK_SWITCH		//I want this here instead of the command line because this will probably never be implemented
#ifndef NO_TSK_SWITCH
typedef struct {
	char name[32];
	char pid;
	char nice;
	char pending_signal;
	uint32_t last_called;
} *proc_t;

void execProc(char*, char, proc_t);
void signalProc(proc_t, char);
#endif

inline long map(long x, long in_min, long in_max, long out_min, long out_max){		//I don't know where to put this, but we do need it somewhere
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif
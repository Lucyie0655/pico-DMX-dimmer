#ifndef _DMX_DIMMER_INIT_H_
#define _DMX_DIMMER_INIT_H_

//this file is almost compleately preprocessor macros and attribute defines


#define SPI_BAUD 10000000
#define I2C_BAUD 1000000

void entryCore1(void) __attribute__((noreturn, long_call));	//note that both cores do some initiallization before jumping to execution
void entryCore0(void) __attribute__((noreturn, long_call));	//in total, including the 8.3s watchdog timeout we should be operational in just under 8.5s
void entryWdg(void) __attribute__((used, cold));	//we don't care about startup time as long as it is less than the amount of time it takes to walk 50 feet

#endif
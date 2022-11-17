/*
if you are just here for software stuff, you may be confused by all these mentions of "TRIACs"
these are just the 120vAC outputs on the back of the device

some datasheets I've used:
	DMX512-A datasheet: https://tsp.esta.org/tsp/documents/docs/ANSI-ESTA_E1-11_2008R2018.pdf
	RP2040 datasheet: https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf
	RP2040 SDK: https://github.com/raspberrypi/pico-sdk
	PCA9685 datasheet: https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
	MCP23017 datasheet: https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf
*/
#ifndef _DMX_DMX_H_
#define _DMX_DMX_H_

#include "hardware/flash.h"
#include "hardware/dma.h"

#define DMX_BAUD 250000		//PLEASE don't change this, you are not the one in control of this, the computer is 
#define DREQ_DMX_RX DREQ_UART0_RX
//if you have to change random numbers change the bauds in init.h

typedef struct {			//656 bytes
	int16_t baseAddr;		//values 513-(-1) are undifined
	uint8_t noAddrs[16];
	uint64_t lockedChan[4];	//keep a mask of what channels cannot be turned on NOTE: only allows 256 total channels
	uint8_t totalAddrs;		//just all the noAddrs added together
	uint8_t halfWave; 		//half-waves passed since power-on mod 256
	uint8_t updateTime[16];	//only call each function every (1/60)*updateTime seconds [eg. a value of 60 means update every 1 second]
	uint8_t nextOut;		//for records, stores a number from 0-31

	union {		//I just named these for some random use case, just different ways of acessing the DMX data
		uint8_t intens[512];
		uint16_t strobe[256];
		volatile int8_t pan[512];	//this is the only one marked volatile so we can increase efficiancy because it rarely maters that this is volatile
		int16_t tilt[256];	//in any real light this would match the data type of pan
	} __attribute__((aligned(512)));
	void (*routines[16])(char*, size_t, uint64_t);	//XXX: FIXME: NOTE: these are called from an ISR when we are not in TASK_SWITCH mode; therefore these need to be fast
	//^^ this is actually intended not fixme, but it is so important that I want it to be rainbow in your IDE
} DMX_info;

void DMX_init(void);

void irq_DMX_onTXCompleate(void);
void irq_DMX_onZero(uint __unused gpio, uint32_t __unused event);

int DMX_parseCommand(char, char*, int);
void DMX_setOutputs(char*, int);
void DMX_setBaseAddr(int);		//this can't be inline because it sets a static variable in a file
int DMX_getBaseAddr(void);		//same thing, uses the static variables
void DMX_receive(void);

int DMX_registerOutputs(uint8_t base, uint8_t num, uint8_t updateTime, void (*setRoutine)(char*, size_t, uint64_t));

void DMX_lockoutChan(int i);
void DMX_unlockChan(int i);
int DMX_isChanLocked(int i);

#define BUTTON_PLUS1	1
#define BUTTON_PLUS10	2
#define BUTTON_PLUS100	4
#define BUTTON_CONFIRM	8
#define BUTTON_CHAN_OFF 16
#define BUTTON_SET_ADDR 32
#define BUTTON_SET_ILIM 64
#define BUTTON_SET_TLIM 128

int DMX_readButtons();
#endif
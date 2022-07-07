#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/regs/psm.h"
#include <rtos.h>
#include <resources.h>
#include <monitoring.h>
#include <outputs.h>

char current[8];
char temp[9];		//temp[8] is for the rp2040 temp
char nextRead;		//first 4 bits indicate channel; highest 3 bits indicate ADC channel
static uint8_t triacBase;		//the first DMX addr that is a triac
static uint8_t triacVals[8];	//the actual values that the triacs are set to (more accurate than DMX_data)
static uint8_t isLockedOut;		//keep our own record of what is locked out

extern volatile char DMX_data[512];		//from DMX.c
extern uint8_t systemError;				//from ctrl/display.c

void irq_MON_ADC(void){
#ifdef ISRDEBUG
	dbg_printf("A");
#endif
	//TODO: current should be RMS over about 10 seconds and temperature should be mean averaged over about a second
	//to prevent the error mentioned on page 589 of the RP2040 datasheet
	//(error peaks every 2^9 samples)
	uint16_t adc_data;
	for(int i=0; adc_fifo_get_level() != 0; i++){		//if we have more than one thing, do all of them
		adc_data = adc_fifo_get();		//get all of the data into memory
		
		//convert data into actual measures (amps and degrees C) (0.8mV/step)
		switch(nextRead >> 5){
			case 0:				//FIXME: what channel is this on?
				//convert the current values (40mV/A == 50steps/A)
				current[nextRead & 0xf] = (adc_data - 2048)/50;		//current *should* now have a value from -50 to +50
				break;

			case 1:				//FIXME: what channel is this on
				//convert the temp sensors (1.3mV/A? == 1.6steps/A?)
				temp[nextRead & 0xf] = 0;							//FIXME: we need numbers
				break;

			case 5:
				//convert the internal temp value (-1.72mV/°C == 2.15steps/°C) (27°C @ 882 read)
				temp[8] = ((adc_data - 882)/2)+27;					//temp[8] should stay below 85°C; other temps should be good to 105
				break;

			default:		//something must have gone wrong in memory to be here
#if defined(DEBUG) && (DEBUG == 2)			//we really shouldn't be spamming the console even in a debug build
				dbg_printf("unused ADC read\n");
#endif
				;			//null statement so it compiles non-debug
		}
		nextRead += 0x20;				//add one to the channel bit field
		//the actual build will have optimizeations so this is just as good, if not better than a bit field
		if(!((nextRead ^ 0xE0) >> 5)){	//when we are reading from the internal temperature sensor
			nextRead++;
			nextRead &= ~0x10;			//if our bit field overflows we have a buffer bit we can clear
		}
	}
}

void pre_lim_temp(int i){		//do I even need this?
	if(i <= 8){
		if(90 <= DMX_data[triacBase+i] < 100)	//full conducion is better than partial conduction
			DMX_data[triacBase+i] = 100;
		else if(0 < DMX_data[triacBase+i] <= 50)	//otherwise lower-voltage switching is better
			DMX_data[triacBase+i] -= DMX_data[triacBase+i] % 10;
		else if(DMX_data[triacBase+i] > 50)		//round to ten, do not turn off
			DMX_data[triacBase+i] += DMX_data[triacBase+i] % 10;
	}
}
void pre_lim_current(int i){}
void err_lim_temp(int i){
	if(i < 8){
		if(DMX_data[triacBase+i] >= triacVals[i]){
			if(!(isLockedOut && 1 << i)){
				printf("****WARNING: tried to increase channel while at temp limit\n");
				setTriacs(triacVals, 8, 0);
				triacLockout(i);			//don't let the DMX increase the output while at our limit
				isLockedOut |= (1 << i);
				systemError = 0xE8;			//I'm making these up as I go, so they are going to seem pretty random
			}
		}
		else{
			triacUnlock(i);				//if we try to go down it is all good again
			isLockedOut &= ~(1 << i);
			systemError = 0;			//clear error
		}
	}
	else{						//the RP2040 is overheating
		printf("****WARNING: chip got too hot, reducing voltage and speed\n");
		//drop the supply voltage; XXX: unknown stabillity; small risk of BOD reset
		*(uint32_t*)VREG_AND_CHIP_RESET_BASE = 0x81;		//decrease DVDD by about 14% (950mV)
		*(uint32_t*)(CLOCKS_BASE+0x40) = 0x122;				//decreases clock to 100.1MHz from 125 (procs, bus fabric, memory, DMA, UART)
	}
}
void err_lim_current(int i){
	if(i < 8){				//actual channel
		if(DMX_data[triacBase+i] > triacVals[i]){		//if we try to go up
			if(!(isLockedOut && 1 << i)){
				printf("****WARNING: tried to increase channel while at channel current limit\n");
				setTriacs(triacVals, 8, 0);					//this call makes it practically impossible to false bump this
				//and impossible if you concider the USB poll of the desk keyboard
				triacLockout(1 << i);						//and take control of this
				isLockedOut |= (1 << i);
				systemError = 0xEA;				//every error will be even
			}
		}
		else if(DMX_data[triacBase+i] < triacVals[i]){
			triacUnlock(1 << i);						//we want to turn down the current, that's fine
			isLockedOut &= ~(1 << i);
			systemError = 0;
		}
	}
	else{					//lock all of them
		for(int j=0; j<8; j++){
			if(DMX_data[triacBase+j] > triacVals[j]){
				if(!(isLockedOut && 1 << i)){
					printf("****WARNING: tried to increase channel while at device current limit\n");
					setTriacs(triacVals, 8, 0);
					triacLockout(0xFF);						//mask all of them locked
					isLockedOut |= (1 << i);
					systemError = 0xEC;			//I am going to put a comment on every one of these
				}
			}
			else if(DMX_data[triacBase+j] < triacVals[j]){
				//non-critical function just let us change all of them again
				triacUnlock(0xFF);
				isLockedOut &= ~(1 << i);
				systemError = 0;
			}
		}
	}
}
void crit_lim_temp(int i){
	printf("****ERROR: reached temperature limit!\n");
	if(i < 8){
		if(!(isLockedOut && 1 << i)){
			triacVals[i] = 0;			//shut down the channel completely
			setTriacs(triacVals, 8, 0);
			triacLockout(1 << i);
			isLockedOut |= (1 << i);
			systemError = 0xFA;					//0xFx errors are now critical
		}
	}
	//XXX: this won't oscillate like the current, however we would like to turn off the output before it restarts
	else{
		gpio_put(GPIO_LED, false);			//goodbye
		__asm__ volatile ("cpsid if");		//disable interrupts

		//tinyUSB will not printf unless interrupts are enabled (or so my testing concluded)
		//disabling interrupts absolutely has to be the first thing, so we can't print

		//disable everything including the clocks
		//corruption can happen if we disable the running core so leave core1 and the XIP running
		*(io_rw_32*)(PSM_BASE+PSM_FRCE_OFF_OFFSET) = 0x67FF;
		__asm__ volatile ("wfi");			//wait for an interrupt (which can never happen)

		//here we only have one processor sleeping whith irq's off so we need to reset
		//but it will definetly be able to cool down doing this
	}
}
void crit_lim_current(int i){
	printf("****ERROR: reached current limit on channel!\n");
	if(i < 8){
		if(!(isLockedOut && 1 << i)){
			triacVals[i] = 0;			//turn off the overloaded channel
			setTriacs(triacVals, 8, 0);
			triacLockout(1 << i);
			isLockedOut |= (1 << i);
			systemError = 0xFC;					//last one
		}
	}
	else{
		if(!(isLockedOut && 1 << i)){
			*(uint64_t*)triacVals = 0;		//turn off all of the triacs in one line (8 chars)
			setTriacs(triacVals, 8, 0);
			triacLockout(0xFF);				//shut it all down
			isLockedOut = 0xFF;
		}
	}
}

void triacSetBase(uint16_t baseAddr){
	triacBase = baseAddr;
}

void read_sensors(void){
	int current_sum;
	current_sum = 0;			//reset it every time for now

	for(int i=0; i<8; i++){
		//check the current from highest to lowest
		//and don't call the lower handlers
		if(current[i] > I_CHAN_CRIT){
			crit_lim_current(i);
			goto checkTempLim;
		}
		if(current[i] > I_CHAN_ERR){
			err_lim_current(i);
			goto checkTempLim;
		}
		if(current[i] > I_CHAN_PRE){
			pre_lim_current(i);
			goto checkTempLim;
		}

checkTempLim:
		//check the temprature and only call the highest handler
		if(temp[i] > T_CHAN_CRIT){
			crit_lim_temp(i);
			goto finishReadLim;
		}
		if(temp[i] > T_CHAN_ERR){
			err_lim_temp(i);
			goto finishReadLim;
		}
		if(temp[i] > T_CHAN_PRE){
			pre_lim_temp(i);
			goto finishReadLim;
		}

finishReadLim:
//		dbg_printf("current %i: %i\n", i, current[i]);
//		dbg_printf("temp %i: %i\n", i, temp[i]);
		current_sum += current[i];		//keep a running tally
		triacVals[i] = DMX_data[triacBase+i];		//update the real values; note that the get is after the limit calls
	}

//	dbg_printf("total current: %i\n", current_sum);
//	dbg_printf("RP2040 temp: %i\n", temp[8]);

	if(current_sum > I_LIM_TOTAL+5)
		crit_lim_current(8);			//a little buffer of 5 amps above the max before we shut down compleately
	if(current_sum > I_LIM_TOTAL)
		err_lim_current(8);

	if(temp[8] > 85)
		err_lim_temp(8);
	if(temp[8] > 75)					//the RP2040 is not able to handle as high a temp as everything else
		err_lim_temp(8);



	//if we reach a critical point we need to reset manually by turning it off and turning it on again
	if((isLockedOut != 0) && (isLockedOut != 0xFF)){		//if the device is not locked out
		for(int i=0; i<8; i++){
			//if the channel is locked out AND the channel is set to 0 AND the channel is not overheating (err level)
			if((isLockedOut && (1 << i)) && 
						(DMX_data[triacBase+i] == 0) &&
						(temp[i] <= 85)){
				isLockedOut &= ~(1 << i);
				triacUnlock(1 << i);
				systemError = 0;
			}
		}
	}
	else if(isLockedOut){		//if the entire device is locked out, we can assume it a current issue
		for(int i=0; i<8; i++){
			if(DMX_data[triacBase+i] != 0)
				return;			//this is the last part of the function, so just return early
		}
		triacUnlock(0xFF);
		systemError = 0;
	}

}
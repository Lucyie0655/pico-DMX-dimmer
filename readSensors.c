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

unsigned char current[8];
unsigned char temp[9];			//temp[8] is for the rp2040 temp
char nextRead;					//LSB 4 bits indicate channel


static uint8_t triacBase;		//the first DMX addr that is a triac
static uint8_t triacVals[8];	//the actual values that the triacs are set to (more accurate than DMX_data)
static uint8_t isLockedOut;		//keep our own record of what is locked out

extern DMX_info DMX_data;		//from DMX.c
extern uint8_t systemError;		//from ctrl/display.c

void irq_MON_ADC(void){
#ifdef ISRDEBUG
	dbg_printf("A");
#endif
	//TODO: current should be RMS over about 1 second and temperature should be mean averaged over about a second
	//to prevent the error mentioned on page 589 of the RP2040 datasheet
	//(error peaks every 2^9 samples)
	int16_t adc_data;
	int8_t nextADCInput;

	while(adc_fifo_get_level() != 0){		//if we have more than one thing, do all of them
		adc_data = adc_fifo_get();			//get all of the data into memory
		if(adc_data & 0x8000)				//indicates error
			continue;						//we don't really care if we miss only one sample

		nextADCInput = (adc_hw->cs & ADC_CS_AINSEL_BITS) >> ADC_CS_AINSEL_LSB;
		if(nextADCInput == 4)					//if the ADC is taking input from the 4th MUX in
			nextADCInput = 3;					//that is the third type of input for us
		nextADCInput -= adc_fifo_get_level();
		while(nextADCInput <= 0)
			nextADCInput += 3;
		
		//convert data into actual measures (amps and degrees C) (0.8mV/step)
		//who knows what these case constants mean, I figured them out through trial and error
		switch(nextADCInput){
			case 3:		//this is current
				//convert the current values (40mV/A == 50steps/A)
				if(adc_data > 2048)
					current[nextRead & 0xf] = (adc_data-2048)/50;		//current *should* now have a value from -50 to +50
				else
					current[nextRead & 0xf] = (2048-adc_data)/50;		//for an absolute current rating
				break;

			case 2:		//TRIAC temperature
				//convert the temp sensors (1.3mV/C? == 1.6steps/C?)
				temp[nextRead & 0xf] = (adc_data - 882)/2+27;			//FIXME: we need numbers for the actual diode we used
				break;

			case 1:		//chip temperature
				//convert the internal temp value (-1.72mV/°C == 2.15steps/°C) (27°C @ 882 read)
				temp[8] = ((adc_data - 882)/2)+27;					//temp[8] should stay below 85°C; other temps should be good to 105
				break;

			default:		//something must have gone wrong in memory to be here
#if defined(DEBUG)
				dbg_printf("unused ADC read: %i\n", nextRead >> 5);
#endif
				;			//null statement so it compiles non-debug
		}
		//the actual build will have optimizeations so this is just as good, if not better than a bit field
		if(nextADCInput == 4){
			nextRead++;
			nextRead &= ~0xF0;			//if our bit field overflows we have a buffer bit we can clear
			gpio_put_masked(1 << GPIO_ADC_SEL_A | 1 << GPIO_ADC_SEL_B | 1 << GPIO_ADC_SEL_C, 
							(nextRead+2 & 0x7) << GPIO_ADC_SEL_C);		//set the MUX GPIOs to whatever the var says they should be
		}
	}
}

void pre_lim_temp(int i){		//do I even need this?
	if(i <= 8){		//numbers are in 256ths
		if(240 <= DMX_data.intens[triacBase+i])			//full conducion is better than partial conduction
			DMX_data.intens[triacBase+i] = 255;
		else if(0 < DMX_data.intens[triacBase+i] <= 128)	//otherwise lower-voltage switching is better
			DMX_data.intens[triacBase+i] -= DMX_data.intens[triacBase+i] % 10;
		else if(DMX_data.intens[triacBase+i] > 128)		//round to ten, do not turn off
			DMX_data.intens[triacBase+i] += DMX_data.intens[triacBase+i] % 10;
	}
}
void pre_lim_current(int i){}
void err_lim_temp(int i){
	if(i < 8){
		if(DMX_data.intens[triacBase+i] > triacVals[i]){
			if(!DMX_isChanLocked(triacBase+i)){
				printf("****WARNING: tried to increase channel while at temp limit\n");
				setTriacs(triacVals, 8, 0);
				DMX_lockoutChan(triacBase+i);		//don't let the DMX increase the output while at our limit
				systemError = 0xE8;			//I'm making these up as I go, so they are going to seem pretty random
			}
		}
		else{
			DMX_unlockChan(triacBase+i);	//if we try to go down it is all good again
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
		if(DMX_data.intens[triacBase+i] > triacVals[i]){		//if we try to go up
			if(!DMX_isChanLocked(triacBase+i)){
				printf("****WARNING: tried to increase channel while at channel current limit\n");
				setTriacs(triacVals, 8, 0);					//this call makes it practically impossible to false bump this
				//and impossible if you concider the USB poll of the desk keyboard
				DMX_lockoutChan(triacBase+i);				//and take control of this
				systemError = 0xEA;				//every error will be even
			}
		}
		else if(DMX_data.intens[triacBase+i] < triacVals[i]){
			DMX_unlockChan(triacBase+i);					//we want to turn down the current, that's fine
			systemError = 0;
		}
	}
	else{					//lock all of them
		for(int j=0; j<8; j++){
			if(DMX_data.intens[triacBase+j] > triacVals[j]){
				if(!DMX_isChanLocked(triacBase+j)){
					printf("****WARNING: tried to increase channel while at device current limit\n");
					setTriacs(triacVals, 8, 0);
					for(int i=0; i<8; i++)
						DMX_lockoutChan(triacBase+i);			//mask all of them locked
					systemError = 0xEC;			//I am going to put a comment on every one of these
				}
			}
			//this lets us choose a lower current without having to turn everything off
			else if(DMX_data.intens[triacBase+j] < triacVals[j]){
				//non-critical function just let us change all of them again
				for(int i=0; i<8; i++)
					DMX_unlockChan(triacBase+i);
				systemError = 0;
			}
		}
	}
}
void crit_lim_temp(int i){
	if(i < 8){
		if(!DMX_isChanLocked(triacBase+i)){
			printf("****ERROR: reached temperature limit!\n");
			triacVals[i] = 0;			//shut down the channel completely
			setTriacs(triacVals, 8, 0);
			DMX_lockoutChan(triacBase+i);
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
		//we are also in a dorment state which I think will use even less power
	}
}
void crit_lim_current(int i){
	if(i < 8){
		if(!DMX_isChanLocked(triacBase+i)){
			printf("****ERROR: reached current limit on channel!\n");
			triacVals[i] = 0;			//turn off the overloaded channel
			setTriacs(triacVals, 8, 0);
			DMX_lockoutChan(triacBase+i);
			systemError = 0xFC;					//last one
		}
	}
	else{
			*(uint64_t*)triacVals = 0;		//turn off all of the triacs in one line (8 chars)
			setTriacs(triacVals, 8, 0);
			for(int i=0; i<8; i++)
				DMX_lockoutChan(triacBase+i);				//shut it all down
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
#ifdef ENFORCE_TEMP_LIMITS
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
#endif

finishReadLim:
		current_sum += current[i];		//keep a running tally
		triacVals[i] = DMX_data.intens[triacBase+i];		//update the real values; note that the get is after the limit calls
	}

#ifdef LOG_ADC
	if((time_us_32() / 1000) % 500 == 0){ 	//print telemetry to serial every 500ms
		for(int i=0; i<8; i++){
			dbg_printf("current %i: %i\n", i, (int)current[i]);
			dbg_printf("temp %i: %i\n", i, (int)temp[i]);
		}
		dbg_printf("total current: %i\n", current_sum);
		dbg_printf("RP2040 temp: %i\n", temp[8]);
	}
#endif

	if(current_sum > I_LIM_TOTAL+5)
		crit_lim_current(8);			//a little buffer of 5 amps above the max before we shut down compleately
	if(current_sum > I_LIM_TOTAL)
		err_lim_current(8);

//	if(temp[8] > 85)
//		crit_lim_temp(8);
//	if(temp[8] > 75)					//the RP2040 is not able to handle as high a temp as everything else
//		err_lim_temp(8);


	//if we reach a critical point we need to reset manually by turning it off and turning it on again
	bool isILim = true;
	for(int i=0; i<8; i++){				//check if the whole device is locked out
		if(!DMX_isChanLocked(triacBase+i)){
			isILim = false;
			break;
		}
	}

	if(isILim){							//if the entire device is locked out, we can assume it a current issue
		for(int i=0; i<8; i++){
			if(DMX_data.intens[triacBase+i] != 0){
				return;
			}
		}
		for(int i=0; i<8; i++)
			DMX_unlockChan(triacBase+i);
		systemError = 0;
	}
	else{
		for(int i=0; i<8; i++){

			//if the channel is locked out AND the channel is set to 0 AND the channel is not overheating (err level)
			if(!DMX_isChanLocked(triacBase+i) 
					&& DMX_data.intens[triacBase+i]
#ifdef ENFORCE_TEMP_LIMITS
					&& temp[i] <= 85
#endif
					)
			{

				DMX_unlockChan(triacBase+i);
				systemError = 0;
			}
		}
	}
}
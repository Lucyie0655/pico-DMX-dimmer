#include <hardware/pio.h>
#include <hardware/i2c.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <pico/multicore.h>

#include <rtos.h>
#include <outputs.h>
#include <resources.h>
#include <DMX.h>
#include <string.h>

static uint16_t triacTiming[256];	//save every possible timing in order as a bitmap of what to turn on
uint16_t nextPIOOut;

void setTriacs(char* data, size_t len, uint64_t lockout){
	nextPIOOut = 0;
	memset(triacTiming, 0, 512);	//clear out all of the old timing info

	for(int i=0; i<len; i++){		//order the stuff into our 256 different output timings
#if 1
//		if(!(lockout && 1 << i)){	//if we are not locked out
			triacTiming[256-data[i]] |= (1 << i);		//earlier turn on means higher power, so we need to reverse-map this
//		}
#else
		triacTiming[0] = 0xFF;
#endif
	}
}


/*
called every 65.5-ish us
because of that this needs to stay really short to work
the only blocking is the phase correct, and keep conditional logic to a minimum
*/
void __irq triacInterrupt(void){
#ifdef ISRDEBUG
	dbg_printf("T");
#endif
	while((volatile uint16_t)nextPIOOut >= 256);		//steps are 0-255, reset by irq_DMX_onZero()

	pio_sm_put(PIO_SHIFTS, 1, (char)triacTiming[nextPIOOut]);
	pio_sm_put(PIO_SHIFTS, 2, (char)(triacTiming[nextPIOOut] >> 8));

	nextPIOOut++;
	PIO_SHIFTS->irq |= 1;								//let the PIO continue
}

void __irq setPCA(char* data, size_t len, uint64_t lockout){	//XXX: if you ever need another PCA a lot of this function should be rewritten
#if defined(DEBUG) && DEBUG == 2
	//probably nothing here?
#else	/*DEBUG == 2*/
	uint16_t values[16] = {0};		//where we store the 16-bit mapped values, twice the number of servos we have, 16 bits for on time, 16 bits for off time

	for(int i=0; i<len; i++){		//copy to the bigger value, every other value needs to be 0
		if(!(lockout && 1 << i))
			values[i] = map(data[i], 0, 255, 512, 256);	//turn off time so reverse-map; 512 - 256 is between 1-2ms pulse length at 62.5Hz
		else values[i] = 384;		//set to +/-0 degrees
	}

	//send the data
	//fifo doesn't need the address we are writing to
	for(int i=0; i<16; i+=2){
		//USUALLY we dont block in a isr
		//this is the exception because:
		//a) we need to send all the data into a limited fifo
		//b) the "periferal" is just as fast as the processor and takes interrupts
		multicore_fifo_push_blocking((values[i+1] << 16) | values[i]);
	}
#endif 	/*DEBUG == 2*/
}
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/irq.h>
#include <hardware/spi.h>
#include <hardware/i2c.h>
#include <hardware/timer.h>
#include <hardware/flash.h>
#include <busCtrl.h>
#include <resources.h>
#include <rtos.h>

//I've never heard of a little-endian microcontroller, but this is important
#ifndef BIG_ENDIAN
#error BIG_ENDIAN not defined
#endif
static uint8_t servoData[32];		//not volatile anymore because it is only accessed in the ISR that sets it; accessed as a uint8_t or a uint32_t; logically a uint16_t

static char   i2c_nextTransmitAddr[8];
static void*  i2c_nextTransmitDat[8];
static size_t i2c_nextTransmitLen[8];
static long   i2c_nextTransmitTim[8];

//XXX: if we want literly anything but our servos, this function and probably "DMX/outputs.c:setPCA()" will need to be entirely rewritten
void isr1_fifo(void){
	if(sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS){		//only if we received data
		for(int i=0; i<32; i+=4){
			if(multicore_fifo_pop_timeout_us(10, (uint32_t*)servoData+i) == false){			//this casts servoData to a uint32, and then we skip one uint32 worth of servoData
				printf("ERROR: fifo rx err\n");
				break;				//something probably went horribly wrong but we will leave it because I don't want to deal with that now
			}
		}
	}
	add_i2c_transmit(PCA0_ADDR, servoData, sizeof(servoData));
}

void add_i2c_transmit(uint8_t addr, const uint8_t* buffer, size_t len){
	for(int i=0; i<8; i++){		//store the new transfer in the first available spot
		if(i2c_nextTransmitLen[i] == 0){			//search for a free spot
			i2c_nextTransmitLen[i] = len;			//set the len
			i2c_nextTransmitTim[i] = time_us_32();	//keep track of when the request was made
			i2c_nextTransmitAddr[i] = addr;
			i2c_nextTransmitDat[i] = malloc(len);	//XXX: unchecked for now, if malloc fails we probably are crashing anyway
			memcpy(i2c_nextTransmitDat[i], buffer, len);		//copy over data
			return;				//and quit
		}
	}
}

void do_i2c_transmit(void){
	uint32_t oldestTX = 0;		//we want a circular buffer like behaviour from the exec transmit

	for(int i=0; i<8; i++){		//loop through until we know the oldest request (fails when one transfer is after the timer 32-bit rollover), but thats OK
		if(i2c_nextTransmitTim[i] < i2c_nextTransmitTim[oldestTX])
			oldestTX = i;
	}

	if(oldestTX == 0)			//transmit buffer empty
		return;

	i2c_write_blocking(i2c1, i2c_nextTransmitAddr[oldestTX], i2c_nextTransmitDat[oldestTX], i2c_nextTransmitLen[oldestTX], false);
	free(i2c_nextTransmitDat[oldestTX]);			//free the allocated memory
	i2c_nextTransmitTim[oldestTX] = 0;				//and clear the time so that we can put stuff here again
}

/*Put these here because I'm not sure where they go*/
void write_to_flash(uint32_t addr, const void* buf, size_t len){
	char blockCopy[FLASH_SECTOR_SIZE];		//should be small enough to be on the stack like this?
	for(uint32_t i=addr-(addr%FLASH_SECTOR_SIZE); i<=addr+len; i+=FLASH_SECTOR_SIZE){
		read_from_flash(i, blockCopy, FLASH_SECTOR_SIZE);
		memcpy(blockCopy+(addr%FLASH_SECTOR_SIZE), buf, len);
		flash_range_erase(i, FLASH_SECTOR_SIZE);
//		flash_range_program(i, blockCopy, FLASH_SECTOR_SIZE);
	}
}

void read_from_flash(uint32_t addr, void* buf, size_t len){
	for(int i=0; i<len; i++){
		((char*)buf)[i] = XIP_BASE+addr+i;
	}
}
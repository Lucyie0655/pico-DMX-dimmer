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
#include <hardware/resets.h>
#include <hardware/regs/psm.h>
#include <busCtrl.h>
#include <resources.h>
#include <rtos.h>

//NOTE: the RP2040 is little endian dispite what the GCC macros would lead you to believe
static uint32_t servoData[17] = {0x06000000};		//set the first u32/ third u8 to 0x06 the register address to send before our data

static char   i2c_nextTransmitAddr[8];
static void*  i2c_nextTransmitDat[8];
static size_t i2c_nextTransmitLen[8];
static long   i2c_nextTransmitTim[8];

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
	dbg_printf("buffer full\n");
}

void do_i2c_transmit(void){
	uint32_t oldestTX = 0x10;	//we want a circular buffer like behaviour from the exec transmit
	//there is nothing special about this init value except it is above the highest buffer slot

	for(int i=0; i<8; i++){		//loop through until we know the oldest request (fails when one transfer is after the timer 32-bit rollover), but thats OK
		//cast to unsigned to ignore the empty entries because -1 is the highest unsigned number
		if((unsigned)i2c_nextTransmitTim[i] < i2c_nextTransmitTim[oldestTX])
			oldestTX = i;
	}

	if(oldestTX == 0x10)		//transmit buffer empty
		return;

	if(i2c_nextTransmitLen[oldestTX] > 0){
		i2c_write_blocking(i2c1, i2c_nextTransmitAddr[oldestTX], i2c_nextTransmitDat[oldestTX], i2c_nextTransmitLen[oldestTX], false);
		free(i2c_nextTransmitDat[oldestTX]);			//free the allocated memory
		i2c_nextTransmitTim[oldestTX] = -1;				//and clear the time so that we can put stuff here again
	}
}

/*Put these here because I'm not sure where they go*/

void __no_inline_not_in_flash_func(isr1_flash_prison)(void){
	__asm__ volatile ("cpsid if");		//clear interrupts

	while(sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS)
		sio_hw->fifo_rd;				//clear the fifo

	while(!sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS);		//wait untill something else is in the fifo
	sio_hw->fifo_rd;					//clear the fifo again

	__asm__ volatile ("cpsie if");		//re-enable interrupts
}

void __not_in_flash_func(write_to_flash)(uint32_t addr, const void* buf, size_t len){
	char* blockCopy;
	blockCopy = malloc(FLASH_SECTOR_SIZE);

	for(uint32_t i=addr&(~0xFFF); i<=addr+len; i+=FLASH_SECTOR_SIZE){
		read_from_flash(i, blockCopy, FLASH_SECTOR_SIZE);
		memcpy(blockCopy+(addr%FLASH_SECTOR_SIZE), buf, len);

		*(uint32_t*)(PSM_BASE+PSM_FRCE_OFF_OFFSET) = 0x10000;
//		multicore_fifo_push_blocking(0x200);	//send core one to prision (cut off flash access)
		reset_block(0x1DBCC9D);					//and kill everything else

		__asm__ volatile ("cpsid if");			//clear interrupts while we cannot access the vecotrs

		flash_range_erase(i, FLASH_SECTOR_SIZE);
		flash_range_program(i, blockCopy, FLASH_SECTOR_SIZE);
//		__asm__ volatile ("cpsie if");

		sio_hw->gpio_clr = 1 << GPIO_LED;

//		multicore_fifo_push_blocking(1);		//and realease core 1 to continue its important duties
	}

	free(blockCopy);
}

void __not_in_flash_func(read_from_flash)(uint32_t addr, void* buf, size_t len){
	memcpy((void*)(XIP_BASE+addr), buf, len);
}
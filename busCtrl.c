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

//NOTE: the RP2040 is little endian dispite what the GCC macros would lead you to believe
static uint32_t servoData[17] = {0x06000000};		//set the first u32/ third u8 to 0x06 the register address to send before our data

static char   i2c_nextTransmitAddr[8];
static void*  i2c_nextTransmitDat[8];
static size_t i2c_nextTransmitLen[8];
static long   i2c_nextTransmitTim[8];

void isr1_fifo(void){
#ifdef ISRDEBUG
	dbg_printf("F");
#endif
	uint32_t sio_command;
	uint32_t nextValue;
	uint8_t x = 1;

	__asm__ volatile ("cpsid if");

	if(!sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS)
		goto irq_exit;
	
	sio_command = sio_hw->fifo_rd;
	if((sio_command & 0xFF00) != 0x100){
		printf("ERROR: unsupported fifo command: 0x%2X\n", sio_command);
		goto irq_exit;
	}

	for(int i=0; i<(sio_command & 0xFF); i+=4){
		while((!sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS) && x)
			x++;				//same timeout method as the send data has

#ifdef DEBUG					//save processining time on the relese build
		if(!x)
			dbg_printf("****ERROR: timeout on fifo pop\n");
#endif

		nextValue = sio_hw->fifo_rd;

		//extract the first data byte from the uint32; set the 9th bit; this implicitly sets the on time (LEDn_ON_[L/H]) to 0
		//this is so complicated because one line uses two different uint32's as uint8's
		servoData[i+1] = (((nextValue >> 24) & 0xFF) << 16) + 0x1000000;

		//do the same thing for the next 3 values pakaged with that
		servoData[i+2] = (((nextValue >> 16) & 0xFF) << 16) + 0x1000000;
		servoData[i+3] = (((nextValue >> 8) & 0xFF) << 16) + 0x1000000;
		servoData[i+4] = (((nextValue >> 0) & 0xFF) << 16) + 0x1000000;
	}

//	add_i2c_transmit(PCA0_ADDR, (uint8_t*)servoData+3, sizeof(servoData)-3);
	i2c_write_blocking(I2C_MAIN, PCA0_ADDR, (uint8_t*)servoData+3, sizeof(servoData)-3, false);

irq_exit:
	__asm__ volatile ("cpsie if");
	while(sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS)		//clear the FIFO
		sio_hw->fifo_rd;
	multicore_fifo_clear_irq();							//clear the IRQ
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
void __no_inline_not_in_flash_func(write_to_flash)(uint32_t addr, const void* buf, size_t len){
	char blockCopy[FLASH_SECTOR_SIZE];		//should be small enough to be on the stack like this?
	int interrupts;

	for(uint32_t i=addr-(addr%FLASH_SECTOR_SIZE); i<=addr+len; i+=FLASH_SECTOR_SIZE){
		read_from_flash(i, blockCopy, FLASH_SECTOR_SIZE);
		memcpy(blockCopy+(addr%FLASH_SECTOR_SIZE), buf, len);
		flash_range_erase(i, FLASH_SECTOR_SIZE);
		flash_range_program(i, blockCopy, FLASH_PAGE_SIZE);
	}
}

void __no_inline_not_in_flash_func(read_from_flash)(uint32_t addr, void* buf, size_t len){
	for(int i=0; i<len; i++){
		((char*)buf)[i] = XIP_BASE+addr+i;
	}
}
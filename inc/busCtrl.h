/*
all of the bus logic is run through core 1 (I2C, SPI, etc.)
if we want to do stuff from core 0 we need an interrupt, and some bus arbitration logic
to keep track of what is supposed to be running a transfer

all arguments to theese functions will be kept in static, volatile variables

since the file name is confusing I thought I would leave a discription
*/
#ifndef _DMX_BUSCTRL_H_
#define _DMX_BUSCTRL_H_
#include <pico/multicore.h>
#include <pico/platform.h>
#include <hardware/i2c.h>
#include <hardware/spi.h>

void isr1_fifo(void);

void add_i2c_transmit(uint8_t, const uint8_t*, size_t);		//allows us to add anything to the static arbitration variables
void do_i2c_transmit(void);		//this is the only one that we use right now

//void do_spi_transmit(void);		//we could use this for wireless DMX? TODO:?

#define FLASH_DMX_ADDR (1024*1024)
void write_to_flash(uint32_t, const void*, size_t);				//I don't know where we should put these functions
void read_from_flash(uint32_t, void*, size_t);

#endif
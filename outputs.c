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
#include "shiftOuts.pio.h"

uint8_t triacTiming[512];	//save every possible timing in order as a bitmap of what to turn on

void setTriacs(char* restrict data, const size_t len, const uint64_t lockout){
	if(len == 8)
		memset(triacTiming, 0, 256);	//if we aren't using the GPB, we can save time by not clearing it
	else
		memset(triacTiming, 0, 512);	//clear out all of the old timing info

	/*
	I would like the easier processing of just a pulse trigger 
	but the triacs have a latching current
	so if the intensity is to high or to low (within about 7 value points of either extreme) then the triac would not latch

	we get around this by holding the gate high the whole time so the current path is manually held open

	FIXME: this may present a problem if the power grid runs to fast
	hopefully the abort in irq_onZero is called before the latch and the testing seems to agree with that assumption
	
	we could as a lazy fix just calibrate clocks for a powergrid of closer to 62 Hz and just lose out on a few points of max power under normal conditions
	*/
	for(int i=0; i<256; i++){
		for(int j=0; j<len; j++){
			data[j] >= 256-i ? triacTiming[i] |= (1 << j) : triacTiming[i];
		}
	}
#if 0
	for(int i=0; i<8; i++){		//order the stuff into our 256 different output timings
//		if(!(lockout && 1 << i)){	//if we are not locked out
			triacTiming[256-data[i]] |= (1 << i);		//earlier turn on means higher power, so we need to reverse-map this
//		}
		if(len == 16)
			triacTiming[512-data[i+8]] |= (1 << i);		//just loop through once
	}
#endif
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
		//[it doesn't really block unless something goes catastrophically wrong]
		multicore_fifo_push_blocking((values[i+1] << 16) | values[i]);
	}
#endif 	/*DEBUG == 2*/
}

void initTriacs(){
	/*INIT: setup PIO_SHIFTS with our shift register driver*/
	pio_claim_sm_mask(PIO_SHIFTS, 0xf);							//we use all of the state machines in this block
	pio_set_sm_mask_enabled(PIO_SHIFTS, 0xF, false);			//disable all of the state machines

	//start all of the PIO SMs on their origin
	pio_add_program_at_offset(PIO_SHIFTS, &SftClkCtrl_program, 0);				//load the clock program
	pio_sm_config pio_config = SftClkCtrl_program_get_default_config(0);
	sm_config_set_out_pins(&pio_config, GPIO_SFT_RCK, 1);
	sm_config_set_set_pins(&pio_config, GPIO_SFT_RCK, 1);
	sm_config_set_sideset_pins(&pio_config, GPIO_SFT_CLK);
	pio_sm_set_consecutive_pindirs(PIO_SHIFTS, 0, 0, 32, true);
	sm_config_set_clkdiv(&pio_config, 68.96f);									//a divider of 135.63@125MHz = 921.62501KHz clock (output 0.003% fast)
	pio_sm_init(PIO_SHIFTS, 0, 0, &pio_config);
	
	pio_add_program_at_offset(PIO_SHIFTS, &SftOutsCtrl_program, 12);			//load the output program block @ addr 4
	pio_config = SftOutsCtrl_program_get_default_config(12);
	sm_config_set_out_pins(&pio_config, GPIO_DAT_GPA, 1);
	sm_config_set_clkdiv(&pio_config, 68.96f);									//a divider of 5.0@125MHz = 25MHz clock
	sm_config_set_fifo_join(&pio_config, PIO_FIFO_JOIN_TX);						//we don't read anything from here, so we can do this and write all of them at the same time
	sm_config_set_out_shift(&pio_config, true, true, 32);						//configure autopull so that we always take exactly one char in
	sm_config_set_out_special(&pio_config, true, false, 0);						//re-assert the out signal while holding for the clock
	pio_sm_init(PIO_SHIFTS, 1, 12, &pio_config);

	sm_config_set_out_pins(&pio_config, GPIO_DAT_GPB, 1);
	pio_sm_init(PIO_SHIFTS, 2, 12, &pio_config);
	
	pio_add_program_at_offset(PIO_SHIFTS, &SftInsCtrl_program, 16);			//load the input program block
	pio_config = SftInsCtrl_program_get_default_config(16);
	sm_config_set_in_pins(&pio_config, GPIO_DAT_BTN);
//	sm_config_set_in_shift(&pio_config, false, true, 8);
	sm_config_set_clkdiv(&pio_config, 5.0f);
	pio_sm_init(PIO_SHIFTS, 3, 16, &pio_config);

	//init the GPIOs for the pio module
	pio_gpio_init(PIO_SHIFTS, GPIO_SFT_CLK);
	pio_gpio_init(PIO_SHIFTS, GPIO_SFT_RCK);
	pio_gpio_init(PIO_SHIFTS, GPIO_DAT_BTN);
	pio_gpio_init(PIO_SHIFTS, GPIO_DAT_GPA);
	pio_gpio_init(PIO_SHIFTS, GPIO_DAT_GPB);

	pio_sm_set_pindirs_with_mask(PIO_SHIFTS, 0, 0x1F, 0x1F);
	pio_sm_set_pindirs_with_mask(PIO_SHIFTS, 1, 0x1F, 0x1F);
	pio_sm_set_pindirs_with_mask(PIO_SHIFTS, 2, 0x1F, 0x1F);
	pio_sm_set_pindirs_with_mask(PIO_SHIFTS, 3, 0, 0x1F);		//this one takes inputs instead of outputs

	pio_enable_sm_mask_in_sync(PIO_SHIFTS, 0xf);				//start the state machine clocks

	/*INIT: setup DMA to take data from triacTiming[] and move it to the fifos for SM1&2*/
	dma_channel_claim(DMA_TRIAC_1);								//claim both channels
	dma_channel_claim(DMA_TRIAC_2);

	dma_channel_config c;
	channel_config_set_read_increment(&c, true);
	channel_config_set_write_increment(&c, false);
	channel_config_set_dreq(&c, DREQ_PIO0_TX1);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
	channel_config_set_enable(&c, true);
	
	//start one channel to write data to the lowest byte of SM1's tx fifo
	dma_channel_configure(DMA_TRIAC_1, &c, &pio0->txf[1], triacTiming, 64, false);

	channel_config_set_dreq(&c, DREQ_PIO0_TX2);
	//start the other channel to write to the lowest byte of SM2's tx fifo
	dma_channel_configure(DMA_TRIAC_2, &c, &pio0->txf[2], triacTiming+256, 64, false);
}
void initPCA(){
	//assume i2c has been init'd because there doesn't seem to be an easy check
	/*I2C INIT: setup the PCA*/
	//read this document for the registers and what each bit means: https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
	//NOTE: we don't maintain the bus because I don't know how that effects register auto-incriment
	char i2c_data_buf[2];									//we only use two bytes at a time
	{
		i2c_data_buf[0] = 0x00;
		i2c_data_buf[1] = 0b00000100;
	}
	i2c_write_blocking(I2C_MAIN, 0x41, i2c_data_buf, 2, false);		//enable auto increment
	{
		i2c_data_buf[0] = 0x01;
		i2c_data_buf[1] = 0x0c;
	}
	i2c_write_blocking(I2C_MAIN, 0x41, i2c_data_buf, 2, false);		//change outputs on ack; and outputs open-drain
	{
		i2c_data_buf[0] = 0xFE;
		i2c_data_buf[1] = 99;
	}
	i2c_write_blocking(I2C_MAIN, 0x41, i2c_data_buf, 2, false);		//set the prescaler to about 62.5 Hz (easiest prescaler for control)
}
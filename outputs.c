#include <hardware/pio.h>
#include <hardware/i2c.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <pico/multicore.h>

#include <rtos.h>
#include <outputs.h>
#include <resources.h>
#include <DMX.h>
#include <busCtrl.h>
#include <string.h>
#include "shiftOuts.pio.h"

uint8_t triacTiming[512];	//save every possible timing in order as a bitmap of what to turn on

void setTriacs(char* restrict data, const size_t len, const uint64_t lockout){
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
			if(!(lockout & (1 << j)))
				data[j] >= 256-i ? triacTiming[i] |= (1 << j) : (triacTiming[i] &= ~(1 << j));
		}
	}/*
	if(len > 8){
		for(int i=0; i<256; i++){
			for(int j=8; j<len; j++){
				if(!(lockout & j))
					data[j] >= 256-i ? triacTiming[i+256] |= (1 << j) : (triacTiming[i+256] &= ~(1 << j));
			}
		}
	}*/
}

struct {
	uint8_t command;
	uint32_t intensity[16];
} __attribute__((packed)) servoData;

void __irq setPCA(char* data, size_t len, uint64_t __unused lockout){
#if defined(DEBUG) && DEBUG == 3
	//probably nothing here?
#else	/*DEBUG == 2*/
	uint8_t x = 1;
	servoData.command = 0x06;

	if(sio_hw->fifo_st & SIO_FIFO_ST_RDY_BITS){
		for(int i=0; i<(len & 0xFF); i++){
			servoData.intensity[i] = ((uint32_t)data[i] << 16) + 0x1000000;
		}

		sio_hw->fifo_wr = 0x100 | (len & 0xFF);		//this will work well for if the total number of PCA outputs is < 256, which is currently the most we can control
	}
#endif 	/*DEBUG == 2*/
}

void isr1_fifo(void){
#ifdef ISRDEBUG
	dbg_printf("F");
#endif
	uint32_t sio_command;
	uint32_t nextValue;
	uint8_t x = 1;

	__asm__ volatile ("cpsid if");			//clear interrupts because this irq will interrupt itself

	if(!sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS)
		goto irq_exit;
	
	sio_command = sio_hw->fifo_rd;
	switch(sio_command & 0xFF00){
		case 0x100:
			goto read_servos;
		case 0x200:
			isr1_flash_prison();
			goto irq_exit;
		default:
			printf("ERROR: unsupported fifo command: 0x%02X\n", sio_command);
			goto irq_exit;
	}

read_servos:
//	add_i2c_transmit(PCA0_ADDR, (uint8_t*)servoData+3, sizeof(servoData)-3);
	i2c_write_blocking(I2C_MAIN, PCA0_ADDR, (uint8_t*)&(servoData.command), sizeof(servoData), false);

irq_exit:
	__asm__ volatile ("cpsie if");
	while(sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS)		//clear the FIFO
		sio_hw->fifo_rd;
	multicore_fifo_clear_irq();							//clear the IRQ
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
	sm_config_set_clkdiv(&pio_config, 65.56f);									//a divider of 68.96@125MHz = 1.813MHz clock (output 0.001% fast @ exactly 60Hz)
	pio_sm_init(PIO_SHIFTS, 0, 0, &pio_config);
	
	pio_add_program_at_offset(PIO_SHIFTS, &SftOutsCtrl_program, 12);			//load the output program block @ addr 4
	pio_config = SftOutsCtrl_program_get_default_config(12);
	sm_config_set_out_pins(&pio_config, GPIO_DAT_GPA, 1);
	sm_config_set_clkdiv(&pio_config, 65.96f);
	sm_config_set_fifo_join(&pio_config, PIO_FIFO_JOIN_TX);						//we don't read anything from here, so we can do this and write all of them at the same time
	sm_config_set_out_shift(&pio_config, true, true, 32);						//configure autopull so that we always take exactly one char in
	sm_config_set_out_special(&pio_config, true, false, 0);						//re-assert the out signal while holding for the clock
	pio_sm_init(PIO_SHIFTS, 1, 12, &pio_config);

	sm_config_set_out_pins(&pio_config, GPIO_DAT_GPB, 1);
	pio_sm_init(PIO_SHIFTS, 2, 12, &pio_config);
	
	pio_add_program_at_offset(PIO_SHIFTS, &SftInsCtrl_program, 16);				//load the input program block
	pio_config = SftInsCtrl_program_get_default_config(16);
	sm_config_set_in_pins(&pio_config, GPIO_DAT_BTN);
//	sm_config_set_in_shift(&pio_config, false, true, 8);
	sm_config_set_clkdiv(&pio_config, 68.96f);
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
	char i2c_data_buf[6];									//we only use a few bytes at a time
	{
		i2c_data_buf[0] = 0x00;
		i2c_data_buf[1] = 0b00110000;
	}
	i2c_write_blocking(I2C_MAIN, PCA0_ADDR, i2c_data_buf, 2, false);		//enable auto increment and put to sleep mode
	{
		i2c_data_buf[0] = 0x01;
		i2c_data_buf[1] = 0x04;
	}
	i2c_write_blocking(I2C_MAIN, PCA0_ADDR, i2c_data_buf, 2, false);		//change outputs on ack; and outputs totem pole output
	{
		i2c_data_buf[0] = 0xFE;
		i2c_data_buf[1] = 99;
	}
	i2c_write_blocking(I2C_MAIN, PCA0_ADDR, i2c_data_buf, 2, false);		//set the prescaler to about 62.5 Hz (easiest prescaler for control)
/*	{
		i2c_data_buf[0] = 0xFA;
		i2c_data_buf[1] = 0;
		i2c_data_buf[2] = 0;
		i2c_data_buf[3] = 128;
		i2c_data_buf[4] = 1;
	}
	i2c_write_blocking(I2C_MAIN, PCA0_ADDR, i2c_data_buf, 5, false);*/
	{
		i2c_data_buf[0] = 0x00;
		i2c_data_buf[1] = 0b00100000;
	}
	i2c_write_blocking(I2C_MAIN, PCA0_ADDR, i2c_data_buf, 2, false);

	servoData.command = 0x06;
}
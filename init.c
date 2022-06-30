/* init.c - main entry point and system initialization

NOTE: some code may look weird because
I first assumed we would need a lot more processing we actually do

to search for a specific initializeation step that you suspect is not working
just Ctrl-F for "INIT:"
both cores do some initializeation in paralell

(this is my checklist assume TODO: if not DONE)
initializes the following (not in order):
	-setup a stack of 4096 bytes/core		DONE
	-watchdog
	-system agent timer						TODO?
	-basic interupts						DONE
	-DMX interface							DONE
	-peripherals (MCP, PCA, NRF, etc.)		DONE; expandable
	-some DMA channels						DONE; expandable
	-PIO triacs								DONE
	-probably more I can't think of right now
*/
#include <string.h>
#include <stdlib.h>
#include <hardware/structs/systick.h>
#include <hardware/structs/scb.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <pico/stdio.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/timer.h>
#include <hardware/i2c.h>
#include <hardware/spi.h>
#include <hardware/pwm.h>
#include <hardware/pio.h>
#include <hardware/uart.h>
#include <hardware/watchdog.h>
#include <hardware/adc.h>
#include <hardware/dma.h>
#include <hardware/clocks.h>
#include "shiftOuts.pio.h"

#include <init.h>
#include <rtos.h>
#include <DMX.h>
#include <resources.h>
#include <monitoring.h>
#include <outputs.h>
#include <ssd1306.h>
#include <busCtrl.h>
#ifdef DEBUG
#include <math.h>				//used for converting string to number
#endif

#ifdef DEBUG
#warning Debug build, disable debug before final build
#endif

ssd1306_t __nvolvar display;		//FIXME: this should move to ctrl/display.c; however the init is in core0 init for now

void entryCore1(void){
	uint16_t buttonInfo;

	/*INIT: setup SPI*/
	spi_init(spi0, SPI_BAUD);		//TODO: check again if this is spi0 or spi1;; overclock spi to 14MHz
	spi_set_slave(spi0, false); 	//this makes me think we could do some cool stuff with spi1 as slave?
	spi_set_format(spi0, 8, 0, 0, SPI_MSB_FIRST);		//check some of this stuff [data bits, CPOL, CPHA]
	gpio_set_function(9, GPIO_FUNC_SPI);		//set the output pins
	gpio_set_function(10, GPIO_FUNC_SPI);
	gpio_set_function(11, GPIO_FUNC_SPI);
	gpio_set_function(12, GPIO_FUNC_SPI);		//and CS_NRF
	dbg_printf("spi setup!\n");
	/*INIT: TODO: this is where we should init the NRF and other SPI stuff*/

	/*INIT: setup ADC*/
	adc_init();
	adc_set_temp_sensor_enabled(true);
	adc_gpio_init(26);
	adc_gpio_init(27);
	adc_set_round_robin(0b11111);					//enable the core temp sensor, the TRIAC temp. sensor, and the bus current sensor
	adc_fifo_setup(true, false, 1, false, false);	//irq when fifo reaches 1 entry, do not record errors; we use a fifo because an ADC conversion only takes 250 ARM cycles
	dbg_printf("ADC ready!\n");

	/*INIT: we could claim a DMA channel here, but I think we have enough processing time to use IRQs*/

	/*INIT: use the builtin LED to show panic info without the display*/
	gpio_init(GPIO_LED);
	gpio_set_dir(GPIO_LED, 1);
	gpio_put(GPIO_LED, true);			//turn this on for now, we can flash on errors maybe


#ifndef NO_TSK_SWITCH
	/*INIT: setup some timing for task overlap*/
	systick_hw->csr = 0;			//disable the sysTick timer
	systick_hw->rvr = TSK_SWITCH/clock_get_hz(clk_sys);		//enable the sysTick to interrupt at TSK_SWITCH Hz
	//FIXME: enable the timer when we are ready enough to use it
#endif
	/*INIT: syncronize with core 0 so we know that intrs are valid*/
	//I was previously using a blocking FIFO exchange instead of this but it seemed unreliable for some reason
	__asm__ volatile ("sev; wfe");			//see RP2040 datasheet section 2.3.3 - both processors wake up at the same time +/- a few clocks
	
	/*INIT: interrupt time!*/
	dbg_printf("setting core 1 IRQ handlers\n");
//	sleep_ms(500);
	//attach handlers XXX: something might be wrong with irq_set_exclusive_handler() it seems to hard-assert if you are setting a handler for the first time
//	((uint32_t*)scb_hw->vtor)[16+ADC_IRQ_FIFO] = &irq_MON_ADC;
//	((uint32_t*)scb_hw->vtor)[16+SIO_IRQ_PROC1] = &isr1_fifo;
	irq_set_exclusive_handler(SIO_IRQ_PROC1, &isr1_fifo);
	irq_set_exclusive_handler(ADC_IRQ_FIFO, &irq_MON_ADC);

	//and enable them all
//	irq_set_enabled(PWM_IRQ_WRAP, true);
	irq_set_enabled(ADC_IRQ_FIFO, true);
	irq_set_enabled(SIO_IRQ_PROC0, true);
	sleep_ms(500);
	while(1);

#ifndef NO_TSK_SWITCH
	//[add the handler here]
#endif
	while(1){		//mainloop
		read_sensors();					//check ADC values XXX: EXIT POINT
		do_i2c_transmit();				//send any data we need on the I2C bus; XXX: BLOCKING


#if !defined(DEBUG) || DEBUG != 3
		//read all of the buttons NOTE: buttons are not debounced
		buttonInfo = DMX_readButtons();
		if((time_us_32() / 1000) % 42 == 0){	//update the display every 42 ms (24 times/s)
			updateDisplay(buttonInfo);
		}
#endif
	}
}

void entryCore0(void){
	DMX_init();
	dbg_printf("DMX interface setup!\n");

	/*INIT: add output handlers for the DMX*/
	DMX_registerOutputs(0, 8, 3, &setTriacs);				//XXX: if you would like to use the GPB change 8 to 16 and the 8 on the next line to 16 NOTE: base addr needs to be 0
	DMX_registerOutputs(8, 16, 5, &setPCA); 				//16 outputs, starting at address 8, updating (1/60s*5)=12Hz for the servos
	dbg_printf("DMX outputs are primed!\n");

	/*INIT: setup i2c*/
#if (DEBUG != 3)
#ifndef NO_I2C
	i2c_init(I2C_MAIN, I2C_BAUD);							//XXX: fast mode plus may not be supported on other devices
	gpio_set_function(4, GPIO_FUNC_I2C);					//if I were smart I would put the pindefs BEFORE we use the pins
	gpio_set_function(5, GPIO_FUNC_I2C);
	gpio_set_pulls(4, true, false);							//pull up resistors are not on the board
	gpio_set_pulls(5, true, false);

		/*I2C INIT: setup the PCA*/
	//just pretend this is magic because I can't be bothered to comment it properly
	//NOTE: we don't maintain the bus because I don't know how that effects register auto-incriment
	char i2c_data_buf[2];									//we only use two bytes at a time
	{
		i2c_data_buf[0] = 0x00;
		i2c_data_buf[1] = 0b00000100;
	}
	i2c_write_blocking(I2C_MAIN, 0x41, i2c_data_buf, 2, false);		//enable auto incriment
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

		/*I2C INIT: setup the display*/
	ssd1306_init(&display, 128, 32, 0x3C, I2C_MAIN);				//init the display
	ssd1306_contrast(&display, 30);									//XXX: I don't know the units for the contrast value FIXME:
	ssd1306_clear(&display);
	ssd1306_poweron(&display);										//display is now running
#else
	dbg_printf("****WARNING: I2C disabled!\n");
#endif  /*defined(NO_I2C)*/
#endif	/*DEBUG != 3*/
	dbg_printf("I2C ready!\n");

	/*INIT: setup PIO_SHIFTS with our shift register driver*/
	pio_claim_sm_mask(PIO_SHIFTS, 0xf);							//we use all of the state machines in this block
	pio_set_sm_mask_enabled(PIO_SHIFTS, 0xF, false);			//disable all of the state machines

	//start all of the PIO SMs on their origin
	pio_add_program_at_offset(PIO_SHIFTS, &SftClkCtrl_program, 0);			//load the clock program
	pio_sm_config pio_config = SftClkCtrl_program_get_default_config(0);
	sm_config_set_out_pins(&pio_config, GPIO_SFT_RCK, 1);
	sm_config_set_set_pins(&pio_config, GPIO_SFT_RCK, 1);
	sm_config_set_sideset_pins(&pio_config, GPIO_SFT_CLK);
	pio_sm_set_consecutive_pindirs(PIO_SHIFTS, 0, 0, 32, true);
	sm_config_set_clkdiv(&pio_config, 5.0);									//a divider of 5.0@125MHz = 25MHz clock
	pio_sm_init(PIO_SHIFTS, 0, 0, &pio_config);
	
	pio_add_program_at_offset(PIO_SHIFTS, &SftOutsCtrl_program, 9);			//load the output program block @ addr 4
	pio_config = SftOutsCtrl_program_get_default_config(9);
	sm_config_set_out_pins(&pio_config, GPIO_DAT_GPA, 1);
	sm_config_set_clkdiv(&pio_config, 5.0);
	sm_config_set_fifo_join(&pio_config, PIO_FIFO_JOIN_TX);					//we don't read anything from here, so we can do this and write all of them at the same time
	pio_sm_init(PIO_SHIFTS, 1, 9, &pio_config);								//this immediatly stops the clocks but whatever

	sm_config_set_out_pins(&pio_config, GPIO_DAT_GPB, 1);
	pio_sm_init(PIO_SHIFTS, 2, 9, &pio_config);
	
	pio_add_program_at_offset(PIO_SHIFTS, &SftInsCtrl_program, 13);			//load the input program block
	pio_config = SftInsCtrl_program_get_default_config(13);
	sm_config_set_in_pins(&pio_config, GPIO_DAT_BTN);
	sm_config_set_in_shift(&pio_config, false, true, 8);
	sm_config_set_clkdiv(&pio_config, 5.0);
	pio_sm_init(PIO_SHIFTS, 3, 10, &pio_config);

	//init the GPIOs for the pio module
	pio_gpio_init(PIO_SHIFTS, GPIO_SFT_CLK);
	pio_gpio_init(PIO_SHIFTS, GPIO_SFT_RCK);
	pio_gpio_init(PIO_SHIFTS, GPIO_DAT_BTN);
	pio_gpio_init(PIO_SHIFTS, GPIO_DAT_GPA);
	pio_gpio_init(PIO_SHIFTS, GPIO_DAT_GPB);

	pio_set_irq0_source_enabled(PIO_SHIFTS, pis_interrupt0, true);
	pio_enable_sm_mask_in_sync(PIO_SHIFTS, 0xf);				//start the state machine clocks
	dbg_printf("TRIAC output PIO ready!\n");


	/*INIT: syncronize with core 0 so we know that irq's are valid*/
	__asm__ volatile ("sev; wfe");			//see RP2040 datasheet section 2.3.3 - both processors wake up at the same time +/- a few clocks
	printf("Initiallizeation done! enabling inerrupts!\n");

	/*INIT: interrupt time!*/
	pio_sm_put(PIO_SHIFTS, 1, 0);							//send a pointless value at the last possible second
	pio_sm_put(PIO_SHIFTS, 2, 0);							//just to get it all started
	irq_set_exclusive_handler(PIO_SFT_IRQ, &triacInterrupt);//when PIO_SHIFTS raises irq0 (more data needed)
	gpio_set_irq_enabled(GPIO_ZC, GPIO_IRQ_EDGE_FALL, true);	//enable the zero cross IRQ
//	irq_set_enabled(IO_IRQ_BANK0, true);
	/*NOTE: this only allows us to have one GPIO interrupt [I think???]
	comment the above line and uncomment the next one aswell as change the DMX/outputs.c/h*/
	gpio_set_irq_enabled_with_callback(GPIO_ZC, GPIO_IRQ_EDGE_FALL, true, &irq_DMX_onZero);

	irq_set_exclusive_handler(UART0_IRQ, &irq_DMX_onTXCompleate);
	irq_set_enabled(UART0_IRQ, true);

	while(1){
#ifndef NOCTRL
		static char debugString[32];						//we need some way of debugging since I don't have my own DMX controller
		char nextFeild;										//note that I left you no comments when writting this so just leave this alone; it works :)
		int debugAddr, debugValue;
		extern volatile char DMX_data[512];

		printf("enter a command or h for help> ");
		for(int i=0; i<32; i++){
			debugString[i] = getchar();
//			while((debugString[i] = getchar()) != EOF);
			if(debugString[i] == '\r'){
				printf("\r\n");
				break;
			}
			printf("%c", debugString[i]);
		}
		switch(debugString[0]){				//we are basically making a basic EOS
			case 'h':
				printf("Only the first character of each command is checked:\n");
				printf("\t[i]ntensity <addr> <percent>\n");
				printf("\t[m]ax <addr>\n");
				printf("\t[k]ill <addr>\n");
				printf("\t[f]ault <0-current | 1-temp> <0-pre | 1-err | 2-crit> <channel (RELITIVE)>\n");
				printf("\t[a]ddress <new device base addr>\n");
				printf("\t[g]etBaseAddr\n");
				break;
			case 'i':
				nextFeild = 0;
				for(int i=1; i<=strlen(debugString); i++){
					if(debugString[i-1] == ' '){
						if(nextFeild == 0)
							debugAddr = strtol(debugString+i, NULL, 10);
						else
							debugValue = strtol(debugString+i, NULL, 10);
						nextFeild++;
					}
				}
				if(debugAddr >= 0 && debugAddr <= 512)
					DMX_data[debugAddr] = (char)debugValue;
				printf("executed set %i at %i\n", debugAddr, debugValue);
				break;
			case 'm':
				for(int i=1; i<=strlen(debugString); i++){
					if(debugString[i-1] == ' '){
						debugAddr = strtol(debugString+i, NULL, 10);
						DMX_data[debugAddr] = 255;
						printf("executed full intensity on address %i\n", debugAddr);
						break;
					}
				}
				break;
			case 'k':
				for(int i=1; i<=strlen(debugString); i++){
					if(debugString[i-1] == ' '){
						debugAddr = strtol(debugString+i, NULL, 10);
						DMX_data[debugAddr] = 0;
						printf("executed out on address %i\n", debugAddr);
						break;
					}
				}
				break;
			case 'f':
				nextFeild = 0;
				for(int i=1; i<=strlen(debugString); i++){
					if(debugString[i-1] == ' '){
						if(nextFeild == 0){
							debugAddr = strtol(debugString+i, NULL, 10);
							nextFeild++;
						}
						else if(nextFeild == 1){
							debugValue = strtol(debugString+i, NULL, 10);
							nextFeild++;
						}
						else if(nextFeild == 2){
							//big if-else mess
							if(debugAddr == 0){
								if(debugValue == 0)
									pre_lim_current(strtol(debugString+i, NULL, 10));
								if(debugValue == 1)
									err_lim_current(strtol(debugString+i, NULL, 10));
								if(debugValue == 2)
									crit_lim_current(strtol(debugString+i, NULL, 10));
							}
							else if(debugValue == 1){
								if(debugValue == 0)
									pre_lim_temp(strtol(debugString+i, NULL, 10));
								if(debugValue == 1)
									err_lim_temp(strtol(debugString+i, NULL, 10));
								if(debugValue == 2)
									crit_lim_temp(strtol(debugString+i, NULL, 10));
							}
						}
					}
				}
				printf("executed handler\n");
				break;
			case 'a':
				for(int i=1; i<=strlen(debugString); i++){
					if(debugString[i-1] == ' '){
						debugAddr = strtol(debugString+i, NULL, 10);
						DMX_setBaseAddr(debugAddr);
						printf("set base address to: %i\n", debugAddr);
						break;
					}
				}
				break;
			case 'g':
				printf("the current device base address is: %i\n", DMX_getBaseAddr());
		}
#endif
	}
}

//NOTE: the main() stack frame is not garenteed safe until the "dbg_printf( starting init )"
void main(void){
	stdio_init_all();
	if(sio_hw->cpuid == 0){		//split execution for the two cpu cores
//		watchdog_update();										//immediately reset it since the reset value is undefined

#if 0
		/*we are using the watchdog to set the stack pointer from C, this is also safer then forcing the stack pointer

		signaling through scratch[0] lets us use the watchdog reset when flashing
		*/
		if(watchdog_hw->scratch[0] == 0xDEADBEEF){
			watchdog_enable(0x7fffff, true);
			watchdog_reboot((int)&entryWdg, SRAM5_BASE, 0x7fffff);	//enable the watchdog
			watchdog_hw->scratch[0] = 0xDEADBEEF; 				//put something into watchdog memory to say we are coming from POR
			__asm__ volatile ("nop");							//force GCC to complete everything before we write to the reset register
			watchdog_hw->ctrl |= WATCHDOG_CTRL_TRIGGER_BITS;	//force reboot now
		}
		watchdog_hw->scratch[0] = 0;							//clear this now
#endif
		//TODO: find some way to copy the stack frame
/*
		__asm__ volatile ( "movs r0, #0x20;"
						   "lsls r0, r0, #12;"
						   "movs r1, #0x41;"
						   "orrs r0, r0, r1;"
						   "lsls r0, r0, #12;"
						   "mov  sp, r0;"
						   : : : "r0","r1");
*/

#ifdef DEBUG
		sleep_ms(2000);			//wait so you can latch your terminal if you don't have a JTAG debugger
		//(you can set it to anything under 8 seconds)
#endif

		dbg_printf("starting initializeation...\n");
		watchdog_update();		//if you have a really long debug wait, we may overflow here
		multicore_launch_core1(&main);
		entryCore0();
	}
	else{
		entryCore1();
	}

	while(1);		//impossible (both entries are noreturn)
}

void __not_in_flash_func(entryWdg)(void){
	stdio_init_all();

	if(watchdog_hw->scratch[0] == 0xDEADBEEF){
		gpio_init(GPIO_LED);
		gpio_set_dir(GPIO_LED, GPIO_OUT);
		gpio_put(GPIO_LED, 1);
		main();				//skip everything if we match our check
	}

	//something should be here, but I don't know what I want here
	//maybe an integrity check?
	busy_wait_ms(1000);
	dbg_printf("Watchdog reboot\r\n");

	main();					//I feel powerful calling main()
							//I've never called main() from C before :)
}

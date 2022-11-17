/* init.c - main entry point and system initialization

NOTE: some code may look weird because
I first assumed we would need a lot more processing we actually do

to search for a specific initializeation step that you suspect is not working
just Ctrl-F for "INIT:"
both cores do some initializeation in paralel

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
#include <hardware/structs/psm.h>
#include "shiftOuts.pio.h"

#include <init.h>
#include <rtos.h>
#include <DMX.h>
#include <resources.h>
#include <monitoring.h>
#include <outputs.h>
#include <ssd1306.h>
#include <busCtrl.h>
#include <outputs.h>
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
	spi_init(spi0, SPI_BAUD);		//TODO: check again if this is spi0 or spi1
	spi_set_slave(spi0, false); 	//this makes me think we could do some cool stuff with spi1 as slave?
	spi_set_format(spi0, 8, 0, 0, SPI_MSB_FIRST);		//check some of this stuff [data bits, CPOL, CPHA]
	gpio_set_function(9, GPIO_FUNC_SPI);		//set the output pins
	gpio_set_function(10, GPIO_FUNC_SPI);
	gpio_set_function(11, GPIO_FUNC_SPI);
	gpio_set_function(12, GPIO_FUNC_SPI);		//and CS_NRF
	dbg_printf("spi setup!\n");
	/*INIT: TODO: this is where we should init the NRF and other SPI stuff*/


	/*INIT: setup the MUX outputs for the ADC reads*/
	gpio_init(GPIO_ADC_SEL_A);
	gpio_init(GPIO_ADC_SEL_B);
	gpio_init(GPIO_ADC_SEL_C);
	gpio_set_dir(GPIO_ADC_SEL_A, true);
	gpio_set_dir(GPIO_ADC_SEL_B, true);
	gpio_set_dir(GPIO_ADC_SEL_C, true);
	//turn off all the ADC MUX GPIOs
	gpio_put_masked(GPIO_ADC_SEL_A | GPIO_ADC_SEL_B | GPIO_ADC_SEL_C, 0);
	/*INIT: setup ADC*/
	//slow the ADC down to increase intigration time, 100KHz input clock is a REALLY long intigration time for this ADC
//	clock_configure(clk_adc, 0, CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12*1000*1000, 10*1000);
	adc_init();
	adc_set_temp_sensor_enabled(true);
	adc_gpio_init(GPIO_ADC_ISENSE);
	adc_gpio_init(GPIO_ADC_THERM);
//	adc_set_round_robin(0b10110);					//enable the core temp sensor, the TRIAC temp. sensor, and the bus current sensor
	adc_set_round_robin(4);
	adc_fifo_setup(true, false, 1, true, false);	//don't use the FIFO
	adc_irq_set_enabled(true);						//hey, this is a thing that has to be done :)
	adc_run(false);									//we want single shot mode
	dbg_printf("ADC ready!\n");

	/*INIT: we could claim a DMA channel here, but I think we have enough processing time to use IRQs*/

	/*INIT: use the builtin LED as a status indicator, and pull up the zero crossing a bit better*/
	gpio_init(GPIO_ZC);
	gpio_set_pulls(GPIO_ZC, true, false);			//this is purely a hardware fault, the on-board pullup is not strong enough
	gpio_init(GPIO_LED);
	gpio_set_dir(GPIO_LED, 1);
	gpio_put(GPIO_LED, true);						//turn this on for now, we can flash on errors maybe


#ifndef NO_TSK_SWITCH
	/*INIT: setup some timing for task overlap*/
	systick_hw->csr = 0;			//disable the sysTick timer
	systick_hw->rvr = TSK_SWITCH/clock_get_hz(clk_sys);		//enable the sysTick to interrupt at TSK_SWITCH Hz
	//FIXME: enable the timer when we are ready enough to use it
#endif
	/*INIT: syncronize with core 0 so we know that intrs are valid*/
	//I was previously using a blocking FIFO exchange instead of this but it seemed unreliable for some reason
	__asm__ volatile ("wfe");			//see RP2040 datasheet section 2.3.3 - both processors wake up at the same time +/- a few clocks
	
	/*INIT: interrupt time!*/
	dbg_printf("setting core 1 IRQ handlers\n");
	irq_set_exclusive_handler(SIO_IRQ_PROC1, &isr1_fifo);
	irq_set_exclusive_handler(ADC_IRQ_FIFO, &irq_MON_ADC);

	//and enable them all
	irq_set_enabled(ADC_IRQ_FIFO, false);			//TODO: I'm just ignoring the measurements, they mostly work just change the enable here
	irq_set_enabled(SIO_IRQ_PROC1, true);
	sleep_ms(100);									//TBH I don't remember why we wait here but it shouldn't matter to anything
	adc_hw->cs |= ADC_CS_START_ONCE_BITS;

#ifndef NO_TSK_SWITCH
	//[add the handler here]
#endif

	while(1){		//mainloop
		//FIXME: the sensor values are unstable, unstable is bad, I don't want to fix it so no protection for you
		read_sensors();					//check ADC values XXX: EXIT POINT

#if DEBUG != 3
#ifndef NO_I2C
		//FIXME: the i2c buffered functions don't seem to work
		//it doesn't matter right now but someone needs to fix it before changing anything on the I2C bus
//		do_i2c_transmit();				//send any data we need on the I2C bus; XXX: BLOCKING
#endif /*!def NO_I2C*/
#endif /*DEBUG != 3*/

#if !defined(DEBUG) || DEBUG != 3
		//read all of the buttons NOTE: buttons are not debounced
	//	buttonInfo = DMX_readButtons();
//		if((time_us_32() / 1000) % 100 == 0){	//update the display every 100 ms (10 times/s)
//			updateDisplay(buttonInfo);
//		}
#endif
	}
}

void entryCore0(void){
	DMX_init();
	dbg_printf("DMX interface setup!\n");

	/*INIT: add output handlers for the DMX*/
	DMX_registerOutputs(0, 8, TRIAC_UPDATE_TIME, &setTriacs);	//XXX: if you would like to use the GPB change 8 to 16 and the 8 on the next line to 16 NOTE: base addr needs to be 0
	DMX_registerOutputs(8, 16, SERVO_UPDATE_TIME, &setPCA); 				//16 outputs, starting at address 8, updating (1/120s*10)=12Hz for the servos
	dbg_printf("DMX outputs are primed!\n");

	/*INIT: setup i2c*/
#if (DEBUG != 3)
#ifndef NO_I2C
	i2c_init(I2C_MAIN, I2C_BAUD);							//XXX: fast mode plus may not be supported on other devices
	gpio_set_function(2, GPIO_FUNC_I2C);
	gpio_set_function(3, GPIO_FUNC_I2C);
	gpio_set_pulls(2, true, false);							//pull up resistors are not on the board
	gpio_set_pulls(3, true, false);

	initPCA();

		/*I2C INIT: setup the display*/
/*	display.external_vcc = false;
	if(!ssd1306_init(&display, 128, 32, 0x3C, I2C_MAIN))
		dbg_printf("failed to init display!\n");					//init the display
	ssd1306_poweron(&display);										//display is now running
	ssd1306_clear(&display);
	ssd1306_contrast(&display, 0x80);
	ssd1306_invert(&display, true);
	ssd1306_draw_char(&display, 64, 16, 1, 'x');
	ssd1306_show(&display);*/
	watchdog_update();
	sleep_ms(5);							//add a short delay because the I2C devices may take some time to init
#else
	dbg_printf("****WARNING: I2C disabled!\n");
#endif  /*defined(NO_I2C)*/
#endif	/*DEBUG != 3*/
	dbg_printf("I2C ready!\n");

	initTriacs();
	dbg_printf("TRIAC output PIO ready!\n");


	/*INIT: syncronize with core 0 so we know that irq's are valid*/
	watchdog_update();
	__asm__ volatile ("sev; wfe");			//see RP2040 datasheet section 2.3.3 - both processors wake up at the same time +/- a few clocks
	watchdog_update();
	printf("Initiallizeation done! enabling inerrupts!\n");

	/*INIT: interrupt time!*/
	gpio_set_irq_enabled_with_callback(GPIO_ZC, GPIO_IRQ_EDGE_RISE, true, &irq_DMX_onZero);
	irq_set_priority(IO_IRQ_BANK0, 0);						//the zero cross is the main timing method so it needs to take priority over everything
	irq_set_exclusive_handler(UART0_IRQ, &irq_DMX_onTXCompleate);
#ifndef NOCTRL
	irq_set_enabled(UART0_IRQ, true);
#endif

#ifdef NOCTRL
		irq_set_enabled(IRQ_DMX_RX, false);
		uart_deinit(DMX_UART);								//if we have no control we should disable the hardware DMX because it would be a headache if this wrote data

		static char debugString[32];						//we need some way of debugging since I don't have my own DMX controller
		char nextFeild;										//note that I left you no comments when writting this so just leave this alone; it works :)
		int debugAddr, debugValue;
		extern DMX_info DMX_data;
#endif
	while(1){
		watchdog_update();

#ifdef NOCTRL
		printf("enter a command or h for help> ");
		for(int i=0; i<32; i++){
			debugString[i] = getchar();
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
				printf("\t[f]ault <[c]urrent | [t]emp> <[p]re | [e]rr | [c]rit> <channel (RELITIVE)>\n");
				printf("\t[a]ddress <new device base addr>\n");
				printf("\t[g]etBaseAddr\n");
				printf("\ti[n]vertAllChannels\n");
				printf("\t[s]hutdown\n");
				printf("\t\thalt both processors (useful if debug info is scrolling to fast to read)\n");
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
					DMX_data.intens[debugAddr] = (char)debugValue;
				printf("executed set %i at %i\n", debugAddr, debugValue);
				break;
			case 'm':
				for(int i=1; i<=strlen(debugString); i++){
					if(debugString[i-1] == ' '){
						debugAddr = strtol(debugString+i, NULL, 10);
						DMX_data.intens[debugAddr] = 255;
						printf("executed full intensity on address %i\n", debugAddr);
						break;
					}
				}
				break;
			case 'k':
				for(int i=1; i<=strlen(debugString); i++){
					if(debugString[i-1] == ' '){
						debugAddr = strtol(debugString+i, NULL, 10);
						DMX_data.intens[debugAddr] = 0;
						printf("executed out on address %i\n", debugAddr);
						break;
					}
				}
				break;
			case 'f':
				nextFeild = 0;
				for(int i=1; i<=strlen(debugString); i++){
					if(debugString[i-1] == ' '){
						if(debugString[i] == 'c'){
							if(debugString[i+2] == 'p'){
								debugAddr = strtol(debugString+i+4, NULL, 10);
								pre_lim_current(debugAddr);
								printf("executed handler for pre_current on %i\n", debugAddr);
							}
							if(debugString[i+2] == 'e'){
								debugAddr = strtol(debugString+i+4, NULL, 10);
								err_lim_current(debugAddr);
								printf("executed handler for err_current on %i\n", debugAddr);
							}
							if(debugString[i+2] == 'c'){
								debugAddr = strtol(debugString+i+4, NULL, 10);
								crit_lim_current(debugAddr);
								printf("executed handler for crit_current on %i\n", debugAddr);
							}
						}
						else if(debugString[i] == 't'){
							if(debugString[i+2] == 'p'){
								debugAddr = strtol(debugString+i+4, NULL, 10);
								pre_lim_temp(debugAddr);
								printf("executed handler for pre_temp on %i\n", debugAddr);
							}
							if(debugString[i+2] == 'e'){
								debugAddr = strtol(debugString+i+4, NULL, 10);
								err_lim_temp(debugAddr);
								printf("executed handler for err_temp on %i\n", debugAddr);
							}
							if(debugString[i+2] == 'c'){
								debugAddr = strtol(debugString+i+4, NULL, 10);
								crit_lim_temp(debugAddr);
								printf("executed handler for crit_temp on %i\n", debugAddr);
							}
						}
					}
				}
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
				break;
			case 'n':
				for(int i=0; i<512; i++){
					DMX_data.intens[i] ^= 0xFF;
				}
				printf("all outputs at opposite extreme\n");
				break;
			case 's':
				__asm__ volatile ("cpsid if");
				multicore_reset_core1();
				__asm__ volatile ("wfi");
				break;
			default:
				printf("invalid command!\n");
		}
#endif
	}
}

//NOTE: the main() stack frame is not garenteed safe until the "dbg_printf( starting init )"
void main(void){
	stdio_init_all();
	if(sio_hw->cpuid == 0){		//split execution for the two cpu cores
#if 0
		/*there is an alternate entry point for a watchdog reset

		we use scratch[0] to tag that this was an intentional watchdog reset
		*/
		if(watchdog_hw->scratch[0] != 0xDEADBEEF){				//returned by entryWdg
			watchdog_enable(2000, true);
			watchdog_update();
			watchdog_reboot((int)&entryWdg, 0, 2000);			//enable the watchdog
			watchdog_hw->scratch[0] = 0xBEEFDEAD; 				//tell the watchdog that we don't need an integrity check
			__asm__ volatile ("nop");							//force GCC to complete everything before we write to the reset register
			gpio_init(GPIO_LED);
			gpio_set_dir(GPIO_LED, GPIO_OUT);
			gpio_put(GPIO_LED, 0);
			while(1);
//			watchdog_hw->ctrl |= WATCHDOG_CTRL_TRIGGER_BITS;	//force reboot now
		}
		watchdog_hw->scratch[0] = 0;							//clear this now
#endif
#ifdef DEBUG
		sleep_ms(1100);			//wait so you can latch your terminal if you don't have a JTAG debugger
		//(you can set it to anything under 2 seconds)
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

void entryWdg(void){
	if(watchdog_hw->scratch[0] == 0xBEEFDEAD){
		gpio_init(GPIO_LED);
		gpio_set_dir(GPIO_LED, GPIO_OUT);
		gpio_put(GPIO_LED, 1);
		watchdog_hw->scratch[0] = 0xDEADBEEF;
		main();				//skip everything if we match our check
	}

	stdio_init_all();
	//something should be here, but I don't know what I want here
	//maybe an integrity check?
//	busy_wait_ms(1000);
	dbg_printf("Watchdog reboot\r\n");

	watchdog_hw->scratch[0] = 0xDEADBEEF;
	main();					//I feel powerful calling main()
							//I've never called main() from C before :)
}

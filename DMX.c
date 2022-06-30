/*
DMX.c - handles the interface to the DMX master and sets the outputs

functions:
	DMX_parseCommand()
	DMX_setOutputs()
	DMX_setBaseAddr()
	DMX_receive() __interrupt
*/

#include <stdio.h>
#include <string.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <hardware/timer.h>
#include <hardware/irq.h>
#include <hardware/i2c.h>
#include <DMX.h>
#include <rtos.h>
#include <resources.h>
#include <init.h>
#include <busCtrl.h>
#include <monitoring.h>


//this is where all of the data goes, stores the DMX command and 511 channels
volatile char __volvar DMX_data[512] __attribute__((aligned(512)));
static DMX_info __nvolvar info;
static char __volvar halfWave;

extern uint16_t nextPIOOut;		//from ./outputs.c; used to sync to AC cycle

static inline bool isChanLocked(uint8_t channel){
	return !!(info.lockedChan[channel/64] && channel);
}
static inline void unlockChan(uint8_t channel){
	info.lockedChan[channel/64] &=~ (channel%64);
}
static inline void lockChan(uint8_t channel){
	info.lockedChan[channel/64] |= (channel%64);
}

void __irq irq_DMX_onZero(uint __unused gpio, uint32_t __unused event){
#ifdef ISRDEBUG
	dbg_printf("z");
#endif /*ISRDEBUG*/
	uint16_t dataOff;				//points to the first DMX addr used by this handler
	dataOff = info.baseAddr;		//the first address assigned to us

	//NOTE: if you are leanring programming, pre incriment/decriment is the absolute worst operator
	//this is the only time I have used it and I try to avoid it if I possibly can.
	//It is usually just as efficient to decriment again if you have to as long as you have compiler optimizeations on.
	++halfWave < 0 ? halfWave = 0 : halfWave;		//if we will overflow, reset the count just in time

#ifdef NO_TSK_SWITCH
	for(int i=0; i<info.nextOut; i++){				//call all of the handlers to fire here
		if(halfWave % info.updateTime[i] == 0){
			info.routines[i](
							info.intens+dataOff,
							info.noAddrs[i], 
							info.lockedChan[(dataOff-info.baseAddr)/64] >> (dataOff-info.baseAddr)%64
							);	//call your handler with data[0] being your first address and the number of addresses you have to set
		}
		dataOff += info.noAddrs[i];					//'almost' a linked list
	}
#else /*NO_TSK_SWITCH*/
	//TODO: set some flag so we can call the handlers outside of an interrupt
#endif /*NO_TSK_SWITCH*/

	//FIXME: every time this is executed ther is a roughly 1/10,000 chance that we desync the shift register
	if(nextPIOOut < 256){
#ifdef TIMING_DEBUG
#warning "TIMING_DEBUG causes prints from an ISR, be careful when using"
		dbg_printf("%hi\n",nextPIOOut);
#endif /*TIMING_DEBUG*/
		pio_sm_exec(PIO_SHIFTS, 1, 9);		//jmp 9
		pio_sm_exec(PIO_SHIFTS, 2, 9);		//jmp 9
		pio_sm_exec(PIO_SHIFTS, 3, 13);		//jmp 13
	}

/*	while(nextPIOOut < 256){		//send the PIO the rest of the data it needs
		pio_sm_put(PIO_SHIFTS, 0, (char)0);
		pio_sm_put(PIO_SHIFTS, 1, (char)0);
		nextPIOOut++;
	}*/
	nextPIOOut = 0;					//reset the count for the PIO irq
}

void __irq irq_DMX_onTXCompleate(void){			//triggers on line break (DMX packet begin)
	dma_channel_abort(DMA_DMX_RX);				//kill the transfer if the controller did not send a full packet (this method blocks)
	dma_channel_start(DMA_DMX_RX);				//restart this for the next transfer
}

/*
DMX_init() - initialize the DMX subsystem
*/
void DMX_init(void){
	/*INIT: setup the DMX interface on DMX_UART*/
	//exactly 250Kbit should is possible, but may as well
	if(uart_init(DMX_UART, DMX_BAUD) != DMX_BAUD){//set the baud rate to 250Kbit
		printf("****WARNING: uart baud rate is not exactly %ikbps; potental RX errors\r\n", DMX_BAUD/1000);
	}
	uart_set_format(DMX_UART, 8, 2, UART_PARITY_NONE);		//DMX (RS-485) has 8 data, 2 stop bits and no parity
	uart_get_hw(DMX_UART)->imsc = 0x100;		//only interrupt we need is the break error (/BE)
	uart_get_hw(DMX_UART)->dmacr |= 0x01;		//allow for dma from the RX buffer, unless an error is received (break) FIXME: TODO: The documentation is unclear as to exactly what this does, if this doesn't work try a value of 0x05
	gpio_set_function(0, GPIO_FUNC_UART);		//set pin 0 (physical pin no. 1) to be used by the uart
	gpio_set_function(1, GPIO_FUNC_UART);		//set pin 1 (physical pin no. 2) to be used by the uart

	/*INIT: Claim the Rx DMA channel*/
	dma_channel_config dma_config;
	dma_channel_claim(DMA_DMX_RX);				//claim our channel

	dma_config = dma_get_channel_config(DMA_DMX_RX);
	channel_config_set_enable(&dma_config, true);						//we are using the channel
	channel_config_set_dreq(&dma_config, DREQ_UART0_RX);				//triggered off of the UART DREQ
	channel_config_set_read_increment(&dma_config, false);				//always read from the same memory address (uart0 rx)
	channel_config_set_write_increment(&dma_config, true);				//update the write address each time
	channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_8);		//only 8 data bits
	channel_config_set_ring(&dma_config, true, 9); 						//wrap the write adddress every 512 bytes

	dma_channel_configure(DMA_DMX_RX, 
						&dma_config, 
						DMX_data, 
						&uart_get_hw(DMX_UART)->dr, 
						512, 
						false
						);		//set channel 0 to constantly read from uart0 into DMX_values
	
	memset(&info, 0, sizeof(info));			//clear out dmx info first

	//read the stored DMX address
	read_from_flash(FLASH_DMX_ADDR, (uint8_t*)&info.baseAddr, sizeof(info.baseAddr));

	irq_add_shared_handler(IRQ_DMX_RX, &irq_DMX_onTXCompleate, 0);		//this should be one of the most important handlers
}

/*
DMX_setBaseAddr/DMX_getBaseAddr() - allows access to the first DMX address
just a macro but we can't inline this or pre-process this becuse info is static to DMX.c
*/
void DMX_setBaseAddr(int addr){
	info.baseAddr = addr;
/*TODO: write_to_flash is broken so someone needs to fix it*/
//	write_to_flash(FLASH_DMX_ADDR, &info.baseAddr, sizeof(info.baseAddr));
}
int DMX_getBaseAddr(void){
	return info.baseAddr;
}

/*
DMX_registerOutputs() - called when you have a new type of output to register
				eg. TRIAC outputs, servo outputs

ARGUMENTS: 
	base - the first address to be used by this driver (relative to our starting address)
	num - the number of addresses you need
	updateTime - how often your output gets updated (in incriments of 1/60 of a second)
	setRoutine - your routine to call every time we update your outputs
	info - common between all calls
RETURN:
	number of channels you actually got or -1 on error
*/
int DMX_registerOutputs(uint8_t base, uint8_t num, uint8_t updateTime, void (*setRoutine)(char*, size_t, uint64_t)){
	//start off with a bunch of ways we can return
	if(base+num > 128){
		printf("****ERROR: cannot allocate more than 128 addresses\r\n");
		return -1;
	}

	//XXX: does base even do anything anymore?
	//TODO/XXX: find if we are overlapping anything

	if(info.nextOut > 31){
		printf("****ERROR: Cannot allocate any more handlers\r\n");
		return -1;
	}

	//load into the struct; NOTE: the base doesn't actually do anything but a sanity check
	info.updateTime[info.nextOut] = updateTime;		//in half waves
	info.routines[info.nextOut] = setRoutine;			//routine to set value
	info.noAddrs[info.nextOut] = num;					//amount of addrs we have used
	info.totalAddrs += num;

	info.nextOut++;
	return info.noAddrs[info.nextOut-1];
}


/*
DMX_readButtons() - reads the front pannel buttons and acts accordingly
*/
int DMX_readButtons(){
	static struct setInfo {
		uint16_t selChan : 5;
		uint16_t LOMask  : 5;
		uint16_t setMode : 4;
	} currentSetInfo;
	static char lastButtonMask;
	static char currentButtonMask;
	char buttonWasPressed;

	//figure out what buttons are pushed
	lastButtonMask = currentButtonMask;
	currentButtonMask = pio_sm_get(PIO_SHIFTS, 3);		//grab the currently pressed buttons
	buttonWasPressed = ~(currentButtonMask & lastButtonMask);		//all of the buttons that wern't pushed last loop

	if(!buttonWasPressed)						//if we havn't pressed any buttons, just return
		return *(uint16_t*)&currentSetInfo;		//we need to have a pointer to be able to cast a struct to an int

	//figure out what mode we are in
	if(buttonWasPressed && BUTTON_SET_ADDR)
		currentSetInfo.setMode = 1;
	else if(buttonWasPressed && BUTTON_CHAN_OFF)
		currentSetInfo.setMode = 2;
	else if(buttonWasPressed && BUTTON_SET_ILIM)
		currentSetInfo.setMode = 3;
	else if(buttonWasPressed && BUTTON_SET_TLIM)
		currentSetInfo.setMode = 4;

	switch(currentSetInfo.setMode){
		case 1:			//set address
			if(buttonWasPressed & BUTTON_PLUS100){
				if(info.baseAddr >= 500)		//if the address is already over 500, wrap around
					info.baseAddr -= 500;
				else
					info.baseAddr += 100;
			}
			if(buttonWasPressed & BUTTON_PLUS10){
				if((info.baseAddr % 100) >= 90)	//don't change the 100's digit
					info.baseAddr -= 90;
				else
					info.baseAddr += 10;

				if(info.baseAddr > 512-info.totalAddrs)		//if we are running past the end of the universe
					info.baseAddr -= 10;
			}
			if(buttonWasPressed & BUTTON_PLUS1){
				if((info.baseAddr % 10) >= 9)		//don't change the 100's digit
					info.baseAddr -= 9;
				else
					info.baseAddr += 1;

				if(info.baseAddr > 512-info.totalAddrs)		//if we are running past the end of the universe
					info.baseAddr -= 1;
			}

			if(buttonWasPressed & BUTTON_CONFIRM){
				currentSetInfo.setMode = 0;
				triacSetBase(info.baseAddr);
				write_to_flash(0x100FFFF0, &(info.baseAddr), sizeof(info.baseAddr));
			}
			break;
		case 2:			//channel off
			if(buttonWasPressed & BUTTON_PLUS1){			//move right
				currentSetInfo.selChan += 1;				//the logic of where we are will be available to the display, so I'll check this there
			}
			if(buttonWasPressed & BUTTON_PLUS10){			//move left
				currentSetInfo.selChan -= 1;
			}
			if(buttonWasPressed & BUTTON_PLUS100){			//page right
				currentSetInfo.selChan += 8;
			}
			if(buttonWasPressed & BUTTON_CONFIRM){
				if(isChanLocked(currentSetInfo.LOMask)){
					lockChan(currentSetInfo.LOMask);
					currentSetInfo.LOMask |= currentSetInfo.selChan;
				}
				else{
					unlockChan(currentSetInfo.LOMask);
					currentSetInfo.LOMask &=~ currentSetInfo.selChan;
				}
			}
			break;
		case 3:			//set current limit
			break;
		case 4: 		//set temp limit
			break;
		default:		//set us back to a mode we understand
			printf("****ERROR: invalid set mode!\n");
			currentSetInfo.setMode = 0;
	}
}
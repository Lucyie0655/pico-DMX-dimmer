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
DMX_info __nvolvar DMX_data;
static char __volvar halfWave;

static inline bool isChanLocked(uint8_t channel){
	return !!(DMX_data.lockedChan[channel/64] && channel);
}
static inline void unlockChan(uint8_t channel){
	DMX_data.lockedChan[channel/64] &=~ (channel%64);
}
static inline void lockChan(uint8_t channel){
	DMX_data.lockedChan[channel/64] |= (channel%64);
}

void __irq irq_DMX_onZero(uint __unused gpio, uint32_t __unused event){
#ifdef ISRDEBUG
	dbg_printf("z");
#endif /*ISRDEBUG*/
	extern char triacTiming[512];
	uint16_t dataOff;				//points to the first DMX addr used by this handler
	dataOff = DMX_data.baseAddr;		//the first address assigned to us

	//dispatch DMA as soon as possible
	//NOTE: since we start DMA before channel updates there may be artifacts around the time of switching but it should be unoticable with 500+ watt incandecent lights
	dma_channel_abort(DMA_TRIAC_1);		//kill the current transfer and restart
	dma_channel_abort(DMA_TRIAC_2);
	dma_channel_set_read_addr(DMA_TRIAC_1, triacTiming, true);	//reset the read address every time
	dma_channel_set_read_addr(DMA_TRIAC_2, triacTiming+256, true);


	//NOTE: if you are leanring programming, pre incriment/decriment is the absolute worst operator
	//this is the only time I have used it and I try to avoid it if I possibly can.
	//It is usually just as efficient to decriment again if you have to as long as you have compiler optimizeations on.
	++halfWave < 0 ? halfWave = 0 : halfWave;		//if we will overflow, reset the count just in time

#ifdef NO_TSK_SWITCH
	for(int i=0; i<DMX_data.nextOut; i++){				//call all of the handlers to fire here
		//by doing halfWave+i will make this irq faster by
		//staggering the half waves that handlers are called on
		if((halfWave+i) % DMX_data.updateTime[i] == 0){		//but only if they are ready to right now
			DMX_data.routines[i](
							DMX_data.intens+dataOff,
							DMX_data.noAddrs[i], 
							DMX_data.lockedChan[(dataOff-DMX_data.baseAddr)/64] >> (dataOff-DMX_data.baseAddr)%64
							);	//call your handler with data[0] being your first address and the number of addresses you have to set
		}
		dataOff += DMX_data.noAddrs[i];					//'almost' a linked list
	}
#else /*NO_TSK_SWITCH*/
	//TODO: set some flag so we can call the handlers outside of an interrupt
#endif /*NO_TSK_SWITCH*/
}

void __irq irq_DMX_onTXCompleate(void){			//triggers on line break (DMX packet begin)
#ifdef ISRDEBUG
	dbg_printf("D");
#endif
	dma_channel_abort(DMA_DMX_RX);				//kill the transfer if the controller did not send a full packet (this method blocks)
	while(uart_is_readable(DMX_UART))
		uart_getc(DMX_UART);					//drain the uart FIFO (likely just one errored 0 for a break)

	if(uart_getc(DMX_UART) == 0)				//FIXME: we need to check for a null start code before dispatching the DMA channel but if possible this should not block
		dma_channel_start(DMA_DMX_RX);			//restart this for the next transfer
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
	uart_get_hw(DMX_UART)->dmacr |= 0x05;		//allow for dma from the RX buffer, unless an error is received (break)
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
						DMX_data.pan, 
						&uart_get_hw(DMX_UART)->dr, 
						512, 
						false
						);		//set channel 0 to constantly read from uart0 into DMX_values
	
	memset(&DMX_data, 0, sizeof(DMX_data));			//clear out dmx DMX_data first
	assert(DMX_data == DMX_data.intens);

	//read the stored DMX address
	DMX_data.baseAddr = *(uint16_t*)(XIP_BASE+FLASH_DMX_ADDR);		//read_from_flash() seems upset for some reason, but this works just fine
	triacSetBase(DMX_data.baseAddr);
}

/*
DMX_setBaseAddr/DMX_getBaseAddr() - allows access to the first DMX address
just a macro but we can't inline this or pre-process this becuse DMX_data is static to DMX.c
*/
void DMX_setBaseAddr(int addr){
	DMX_data.baseAddr = addr;
	triacSetBase(addr);

	write_to_flash(FLASH_DMX_ADDR, &(DMX_data.baseAddr), sizeof(DMX_data.baseAddr));
}
int DMX_getBaseAddr(void){
	return DMX_data.baseAddr;
}

/*
DMX_registerOutputs() - called when you have a new type of output to register
				eg. TRIAC outputs, servo outputs

ARGUMENTS: 
	base - the first address to be used by this driver (relative to our starting address)
	num - the number of addresses you need
	updateTime - how often your output gets updated (in incriments of 1/60 of a second)
	setRoutine - your routine to call every time we update your outputs
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

	if(DMX_data.nextOut > 31){
		printf("****ERROR: Cannot allocate any more handlers\r\n");
		return -1;
	}

	//load into the struct; NOTE: the base doesn't actually do anything but a sanity check
	DMX_data.updateTime[DMX_data.nextOut] = updateTime;			//in half waves
	DMX_data.routines[DMX_data.nextOut] = setRoutine;			//routine to set value
	DMX_data.noAddrs[DMX_data.nextOut] = num;					//amount of addrs we have used
	DMX_data.totalAddrs += num;

	DMX_data.nextOut++;
	return DMX_data.noAddrs[DMX_data.nextOut-1];
}

void DMX_lockoutChan(int i){
	DMX_data.lockedChan[i/64] |= 1 << (i%64);
}
void DMX_unlockChan(int i){
	DMX_data.lockedChan[i/64] &=~ 1 << (i%64);
}
int DMX_isChanLocked(int i){
	return !!(DMX_data.lockedChan[i/64] & (1 << i%64));
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
				if(DMX_data.baseAddr >= 500)		//if the address is already over 500, wrap around
					DMX_data.baseAddr -= 500;
				else
					DMX_data.baseAddr += 100;
			}
			if(buttonWasPressed & BUTTON_PLUS10){
				if((DMX_data.baseAddr % 100) >= 90)	//don't change the 100's digit
					DMX_data.baseAddr -= 90;
				else
					DMX_data.baseAddr += 10;

				if(DMX_data.baseAddr > 512-DMX_data.totalAddrs)		//if we are running past the end of the universe
					DMX_data.baseAddr -= 10;
			}
			if(buttonWasPressed & BUTTON_PLUS1){
				if((DMX_data.baseAddr % 10) >= 9)		//don't change the 100's digit
					DMX_data.baseAddr -= 9;
				else
					DMX_data.baseAddr += 1;

				if(DMX_data.baseAddr > 512-DMX_data.totalAddrs)		//if we are running past the end of the universe
					DMX_data.baseAddr -= 1;
			}

			if(buttonWasPressed & BUTTON_CONFIRM){
				currentSetInfo.setMode = 0;
				triacSetBase(DMX_data.baseAddr);
//				write_to_flash(0x100FFFF0, &(DMX_data.baseAddr), sizeof(DMX_data.baseAddr));
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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include <monitoring.h>
#include <ssd1306.h>
#include <rtos.h>
#include <outputs.h>
#include <DMX.h>

extern ssd1306_t display;					//FIXME: from boot/init.c for now
uint8_t systemError;						//we are going to cheat the error code and use the MSB to indicate error == true


void updateDisplay(uint16_t info){
	struct {
		uint16_t selChan : 5;
		uint16_t LOMask  : 5;
		uint16_t setMode : 4;
	} *currentSetInfo;
	static char frameCount;
	char nextString[25] = {0};				//we can't fit more than 25 chars on a line

	//cast info to the currentSetInfo struct
	currentSetInfo = (void*)&info;
	
	ssd1306_clear(&display);

	switch(currentSetInfo->setMode){
		case 0:				//no set mode
			break;
		case 1: 			//set address
			snprintf(nextString, 6, "Addr: ");
			ssd1306_draw_string(&display, 32, 11, 2, nextString);

			if(frameCount % 24 <= 12){			//blink at 1Hz with a 50% duty cycle??? XXX: brain no work today
				snprintf(nextString, 3, "%03d", DMX_getBaseAddr());	//turn the addr into a string that always contains three digits and a NUL
				ssd1306_draw_string(&display, 44, 11, 2, nextString);	//put the address on screen
			}	//otherwise no address
			break;
		case 2:				//set lockout channel
			if(currentSetInfo->selChan & 24){				//'page' 4 (channels 25-32) [currently unused]
				snprintf(nextString, 32, "25 26 27 28 29 30 31 32");
				ssd1306_draw_string(&display, 6, 11, 1, nextString);
				for(int i=0; i<=7; i++){		//if the channel is masked off print an 'x', otherwise print a space
					nextString[i*3] = currentSetInfo->LOMask & (i+24) ? 'x' : ' ';	//add the base to the counter value and shift to the masked channel section
					nextString[i*3+1] = ' ';	//add some space to align with the channel number
					nextString[i*3+2] = ' ';
					nextString[i*3+3] = '\0';	//clear out the byte after, only important for the last one, but won't add time because optimizations
				}
				ssd1306_draw_string(&display, 6, 18, 1, nextString);		//lined up and drawn 2 pixels lower
			}
			else if(currentSetInfo->selChan & 16){		//'page' 3 (channels 17-24)
				snprintf(nextString, 32, "17 18 19 20 21 22 23 24");
				ssd1306_draw_string(&display, 6, 11, 1, nextString);
				for(int i=0; i<=7; i++){		//if the channel is masked off print an 'x', otherwise print a space
					nextString[i*3] = currentSetInfo->LOMask & (i+16) ? 'x' : ' ';	//add the base to the counter value and shift to the masked channel section
					nextString[i*3+1] = ' ';	//add some space to align with the channel number
					nextString[i*3+2] = ' ';
					nextString[i*3+3] = '\0';	//clear out the byte after, only important for the last one, but won't add time because optimizations
				}
				ssd1306_draw_string(&display, 6, 18, 1, nextString);		//lined up and drawn 2 pixels lower
			}
			else if(currentSetInfo->selChan & 8){		//'page' 2 (channels 9-16)
				snprintf(nextString, 32, "9  10 11 12 13 14 15 16");
				ssd1306_draw_string(&display, 6, 11, 1, nextString);
				for(int i=0; i<=7; i++){		//if the channel is masked off print an 'x', otherwise print a space
					nextString[i*3] = currentSetInfo->LOMask & (i+8) ? 'x' : ' ';	//add the base to the counter value and shift to the masked channel section
					nextString[i*3+1] = ' ';	//add some space to align with the channel number
					nextString[i*3+2] = ' ';
					nextString[i*3+3] = '\0';	//clear out the byte after, only important for the last one, but won't add time because optimizations
				}
				ssd1306_draw_string(&display, 6, 18, 1, nextString);		//lined up and drawn 2 pixels lower
			}
			else{										//'page' 1 (channels 1-8)
				snprintf(nextString, 32, "1  2  3  4  5  6  7  8 ");
				ssd1306_draw_string(&display, 6, 11, 1, nextString);
				for(int i=0; i<=7; i++){		//if the channel is masked off print an 'x', otherwise print a space
					nextString[i*3] = currentSetInfo->LOMask & (i+0) ? 'x' : ' ';	//add the base to the counter value and shift to the masked channel section
					nextString[i*3+1] = ' ';	//add some space to align with the channel number
					nextString[i*3+2] = ' ';
					nextString[i*3+3] = '\0';	//clear out the byte after, only important for the last one, but won't add time because optimizations
				}
				ssd1306_draw_string(&display, 6, 18, 1, nextString);		//lined up and drawn 2 pixels lower
			}
			break;
		case 3: 			//set I-limit
			break;
		case 4: 			//set T-limit
			break;
	}

	//TODO: indicate errors
	if(systemError && 0x80){
		ssd1306_clear(&display);			//change of plans, don't draw the address
		snprintf(nextString, 4, "0x%2X", systemError);
		ssd1306_draw_string(&display, 32, 8, 3, nextString);		//put this on the screen
	}

	//TODO: output view?
	frameCount++;							//keep our frame counter
	ssd1306_show(&display);					//execute draw
}
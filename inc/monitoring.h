#ifndef _DMX_MONITORING_H_
#define _DMX_MONITORING_H_
#include <stdint.h>

void irq_MON_ADC(void);

/*XXX: 
pre_ functions are for preemptive limiting and not required
err_ functions are for when limits are reached, turn down outputs to keep within limits; prints a debug message
crit_ functions are for when limits are exceeded, immediately turn off outputs; prints an error message
*/

#define I_LIM_TOTAL 30			//current limit for the entire system
#define I_CHAN_CRIT 20
#define I_CHAN_ERR  15
#define I_CHAN_PRE  12
#define T_CHAN_CRIT 105
#define T_CHAN_ERR  85
#define T_CHAN_PRE  75

void pre_lim_temp(int);
void pre_lim_current(int);
void err_lim_temp(int);
void err_lim_current(int);
void crit_lim_temp(int);
void crit_lim_current(int);

void read_sensors(void);

void triacSetBase(uint16_t);

/*ctrl/display.c*/
void updateDisplay(uint16_t);
#endif
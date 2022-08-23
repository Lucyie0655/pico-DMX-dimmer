#ifndef _DMX_RESOURCES_H_
#define _DMX_RESOURCES_H_
/*any commented out lines are reserved and cannot be allocated for anything else*/
#include <hardware/adc.h>
#include <hardware/dma.h>
#include <hardware/i2c.h>
#include <hardware/spi.h>
#include <hardware/pio.h>
#include <hardware/timer.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>

/*DMA mappings*/
#define DMA_DMX_RX 0				//keep these in sync
#define IRQ_DMX_RX DMA_IRQ_0		//otherwise you will break it terribly

#define DMA_TRIAC_1 1				//triacs need 2 DMA channels
#define DMA_TRIAC_2 2
//#define DMA_DMX_TX 1
//#define DMA_I2C_RX 1
//#define DMA_I2C_TX 2

/*I2C mappings*/
#define I2C_MAIN i2c1
#define PCA0_ADDR 0x60

/*SPI mappings*/
#define SPI_MAIN spi0

/*PIO mappings*/
#define PIO_SHIFTS pio0
#define PIO_SFT_IRQ PIO0_IRQ_0

/*TIMER mappings*/
#define TIME_ALM_SLEEP 0
#define PWM_FRAME_INT 0

/*GPIO mappings*/
#define GPIO_LED 25
#define GPIO_SFT_CLK 16
#define GPIO_SFT_RCK 17
#define GPIO_DAT_GPA 18			//TODO: figure out what pin this actually is
#define GPIO_DAT_GPB 19
#define GPIO_DAT_BTN 20
#define GPIO_ZC 15
#define GPIO_ADC_SEL_A 13
#define GPIO_ADC_SEL_B 12
#define GPIO_ADC_SEL_C 11
#define GPIO_ADC_THERM 27
#define GPIO_ADC_ISENSE 28

/*UART mappings*/
#define DMX_UART uart0
#define DBG_UART uart1

#endif
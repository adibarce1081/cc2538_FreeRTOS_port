/*
 * main.c
 *
 *  Created on: 28-Nov-2019
 *      Author: tnstark
 */


#include <stdint.h>
#include "reg.h"
#include "cpu.h"
#include "gptimer.h"
#include "gpio.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_ioc.h"
#include "sys_ctrl.h"


typedef uint32_t clock_time_t;
#define CLOCK_SECOND (clock_time_t)32
#define SYSTICK_PERIOD          (SYS_CTRL_SYS_CLOCK / CLOCK_SECOND)
#define PRESCALER_VALUE         (SYS_CTRL_SYS_CLOCK / SYS_CTRL_1MHZ - 1)
#define GPIO_C_DIR                0x400DB400
#define GPIO_C_DATA                0x400DB000
#define HWREG(x)                                                              \
    (*((volatile uint32_t *)(x)))

#if SYS_CTRL_OSC32K_USE_XTAL
#define SYS_CTRL_OSCS   0
#else
#define SYS_CTRL_OSCS   SYS_CTRL_CLOCK_CTRL_OSC32K
#endif





int main(void)
{
	//clock_init();
    volatile uint32_t i;
   // volatile uint32_t ui32LoopXV=0;
    // Set direction output and initial value for PC2 and PC0
    // Greed LED on PC2
    // Red LED on PC0
    HWREG(GPIO_C_DIR) |= 0xE0;
    HWREG(GPIO_C_DATA + (0xE0 << 2)) = 0xE0;
    //HWREG(GPIO_C_DATA) |= 0xE0;

    //GPIO_SET_OUTPUT(GPIO_C_BASE, 0x06);
    // Loop forever.
    while(1)
    {
        // Turn on both LED's.
       // HWREG(GPIO_C_DATA) |=  0xE0;
    	HWREG(GPIO_C_DATA + (0xE0 << 2)) ^= 0xE0;
    	//GPIO_SET_PIN(GPIO_C_BASE, 0x06);
    	// Delay for a bit
        for(i = 2000000; i > 0; i--)
        {
           // ui32LoopXV++;
        }


    }
}


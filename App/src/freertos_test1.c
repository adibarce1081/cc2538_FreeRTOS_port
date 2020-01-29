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


#include "FreeRTOS.h"
#include "task.h"

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

void vTask1( void *pvParameters );
void sys_ctrl_init(void);

//void clock_init(void);
//void sys_ctrl_init(void);


int main(void)
{
	//clock_init();
    //volatile uint32_t i;
   // volatile uint32_t ui32LoopXV=0;
    // Set direction output and initial value for PC2 and PC0
    // Greed LED on PC2
    // Red LED on PC0
    SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_32MHZ);

	HWREG(GPIO_C_DIR) |= 0xE0;
    HWREG(GPIO_C_DATA + (0xE0 << 2)) = 0xE0;
    //HWREG(GPIO_C_DATA) |= 0xE0;
    uint32_t ret;
    ret = xTaskCreate( vTask1, "Task 2", 240, NULL, 1, NULL );
    configASSERT(ret);
    //GPIO_SET_OUTPUT(GPIO_C_BASE, 0x06);
    // Loop forever.
   /* while(1)
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


    }*/
    vTaskStartScheduler();

      while(1);
      return 0;
}
void vTask1( void *pvParameters )
{

//volatile unsigned long ul;

        /* As per most tasks, this task is implemented in an infinite loop. */
        for( ;; )
        {

        		//BSP_LED_Toggle(LED4);
        	    HWREG(GPIO_C_DATA + (0xE0 << 2)) ^= 0xE0;
                //lecture - a dummy loop - just for demo ??
                /* Delay for a period. */
               // for( ul = 0; ul < 200000; ul++ )
                //{
                        /* This loop is just a very crude delay implementation.  There is
                        nothing to do in here.  Later exercises will replace this crude
                        loop with a proper delay/sleep function. */
                //}
                //these variables can be used to check the execution of the task
        	    vTaskDelay(1000);
        }
}
#if 1
void sys_ctrl_init(void)
{
  uint32_t val;

#if SYS_CTRL_OSC32K_USE_XTAL
  /* Set the XOSC32K_Q pads to analog for crystal */
  GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_D_NUM), GPIO_PIN_MASK(6));
  GPIO_SET_INPUT(GPIO_PORT_TO_BASE(GPIO_D_NUM), GPIO_PIN_MASK(6));
  ioc_set_over(GPIO_D_NUM, 6, IOC_OVERRIDE_ANA);
  GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_D_NUM), GPIO_PIN_MASK(7));
  GPIO_SET_INPUT(GPIO_PORT_TO_BASE(GPIO_D_NUM), GPIO_PIN_MASK(7));
  ioc_set_over(GPIO_D_NUM, 7, IOC_OVERRIDE_ANA);
#endif

  /*
   * Desired Clock Ctrl configuration:
   * 32KHz source: RC or crystal, according to SYS_CTRL_OSC32K_USE_XTAL
   * System Clock: 32 MHz
   * Power Down Unused
   * I/O Div: according to SYS_CTRL_IO_DIV
   * Sys Div: according to SYS_CTRL_SYS_DIV
   * Rest: Don't care
   */

  val = SYS_CTRL_OSCS | SYS_CTRL_CLOCK_CTRL_OSC_PD
    | SYS_CTRL_IO_DIV | SYS_CTRL_SYS_DIV;
  REG(SYS_CTRL_CLOCK_CTRL) = val;

  while((REG(SYS_CTRL_CLOCK_STA)
        & (SYS_CTRL_CLOCK_STA_OSC32K | SYS_CTRL_CLOCK_STA_OSC))
        != SYS_CTRL_OSCS);

#if SYS_CTRL_OSC32K_USE_XTAL
  /* Wait for the 32-kHz crystal oscillator to stabilize */
  while(REG(SYS_CTRL_CLOCK_STA) & SYS_CTRL_CLOCK_STA_SYNC_32K);
  while(!(REG(SYS_CTRL_CLOCK_STA) & SYS_CTRL_CLOCK_STA_SYNC_32K));
#endif

}
#endif
void vApplicationMallocFailedHook( void )
{
        /* This function will only be called if an API call to create a task, queue
        or semaphore fails because there is too little heap RAM remaining. */
        for( ;; );
}


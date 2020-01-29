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

#include "ubimotesez_config.h"
#include "bsp.h"
//#include "bsp_key.h"
#include "hal_int.h"
#include "basic_rf.h"
#include "hal_rf.h"
#include "hal_timer_32k.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "bsp_led.h"
#include "uartstdio.h"
#include "hw_ioc.h"
#include "ioc.h"
#include "string.h"




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
//void vTask2( void *pvParameters );
void vRF_TX_Task( void *pvParameters );
void vRF_RX_Task( void *pvParameters );

//void sys_ctrl_init(void);

//void clock_init(void);
//void sys_ctrl_init(void);

moteCfg_t moteConfig = {0};
basicRfCfg_t basicRfConfig;
volatile unsigned char timer_experied = 0;
//uint8_t data[]="Hi..... I am Ubimote-SEZ";
uint8_t data = 0xAB;
uint8_t payload[MAX_PAYLOAD];
//static void transmit(void);
static void UARTlibinit(void);
//static void delay(uint16_t mSec);
//static void timerIsr(void);


int main(void)
{
	//clock_init();
    //volatile uint32_t i;
   // volatile uint32_t ui32LoopXV=0;
    // Set direction output and initial value for PC2 and PC0
    // Greed LED on PC2
    // Red LED on PC0
//    SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_32MHZ);
    bspInit(BSP_SYS_CLK_SPD);


	//HWREG(GPIO_C_DIR) |= 0xE0;
    //HWREG(GPIO_C_DATA + (0xE0 << 2)) = 0xE0;
    //HWREG(GPIO_C_DATA) |= 0xE0;

    UARTlibinit();

    	moteConfig.mode = MOTE_MODE_RX;
        	moteConfig.channel = CHANNEL;
        	moteConfig.txPower = 7;          		// Index 0. Max output
        	moteConfig.gainMode = MOTE_GAIN_MODE_HI; 	// No PA/LNA

    	// Enable interrupts
        	halIntOn();

    	// Config basicRF
        	basicRfConfig.panId = PAN_ID;
        	basicRfConfig.ackRequest = false;

    uint32_t ret;
    ret = xTaskCreate( vRF_TX_Task, "Task 2", 240, NULL, 1, NULL );
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

void vRF_TX_Task( void *pvParameters )
{
	//rf_handle *rf_p = (rf_handle *)pvParameters;
	basicRfConfig.myAddr = TX_ADDR;
    basicRfConfig.channel = moteConfig.channel;

	if(basicRfInit(&basicRfConfig) == FAILED){
		while(1);
    	}

	// Turn receiver off
    	basicRfReceiveOff();

	halRfSetTxPower(moteConfig.txPower);

	payload[0] = sizeof(data);

	memcpy(&(payload[1]),&data, payload[0]);

	while(1){

		bspLedClear(BSP_LED_ALL);
		basicRfSendPacket(RX_ADDR, (unsigned char *)payload, (payload[0])+1);

		bspLedSet(BSP_LED_ALL);

		vTaskDelay(1000);
	}
}

void vRF_RX_Task( void *pvParameters )
{
	signed short ssRssi;
		int i;

		basicRfConfig.myAddr = RX_ADDR;
	    	basicRfConfig.channel = moteConfig.channel;
	    	if(basicRfInit(&basicRfConfig)==FAILED){
			UARTprintf("rf init failed\n\r");
			while(1);
	    	}

		if(moteConfig.gainMode != MOTE_GAIN_MODE_NONE){

	        	// Set gain mode
	        	halRfSetGain(moteConfig.gainMode);
	    	}

		// Start RX
		basicRfReceiveOn();

		while(1) {

			while(!basicRfPacketIsReady());
	        	if(basicRfReceive((unsigned char*)payload, 103, &ssRssi) > 0) {

				bspLedToggle(BSP_LED_ALL);
				for(i=1;i<payload[0];i++){
						UARTprintf("%c", payload[i]);
				}
				UARTprintf("\n\r");
	        	}
	    	}
}


#if 0
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

#if 0
static void transmit(void){

    	basicRfConfig.myAddr = TX_ADDR;
    	basicRfConfig.channel = moteConfig.channel;

	if(basicRfInit(&basicRfConfig) == FAILED){
		while(1);
    	}

	// Turn receiver off
    	basicRfReceiveOff();

	halRfSetTxPower(moteConfig.txPower);

	payload[0] = sizeof(data);

	memcpy(&(payload[1]),&data, payload[0]);

	while(1){

		bspLedClear(BSP_LED_ALL);
		basicRfSendPacket(RX_ADDR, (unsigned char *)payload, (payload[0])+1);

		bspLedSet(BSP_LED_ALL);

		delay(1000);
	}
}
#endif
static void UARTlibinit(void){

    	// Map UART signals to the correct GPIO pins and configure them as
    	// hardware controlled.
    	IOCPinConfigPeriphOutput(GPIO_A_BASE, GPIO_PIN_1,
                             IOC_MUX_OUT_SEL_UART0_TXD);
    	GPIOPinTypeUARTOutput(GPIO_A_BASE, GPIO_PIN_1);

    	IOCPinConfigPeriphInput(GPIO_A_BASE, GPIO_PIN_0,
                            IOC_UARTRXD_UART0);
    	GPIOPinTypeUARTInput(GPIO_A_BASE, GPIO_PIN_0);

    	// Initialize the UART (UART0) for console I/O.
    	UARTStdioInit(0);
}
#if 0
static void delay(uint16_t mSec) {

	timer_experied = 0;
	IntPrioritySet(INT_SMTIM, 0x80);        // Reduce timer interrupt priority
	halTimer32kInit((uint16_t)(((float)(32768/1000.0))*mSec));
    halTimer32kIntConnect(&timerIsr);    // Connect ISR
   	halTimer32kIntEnable();                 // Enable interrupts
	while(!(timer_experied == 1));
}

static void timerIsr(void){

	timer_experied = 1;
	halTimer32kIntDisable();
}
#endif

void vApplicationMallocFailedHook( void )
{
        /* This function will only be called if an API call to create a task, queue
        or semaphore fails because there is too little heap RAM remaining. */
        for( ;; );
}



#include "ubimotesez_config.h"
#include "bsp.h"
#include "bsp_key.h"
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
#include "sys_ctrl.h"

moteCfg_t moteConfig = {0};
basicRfCfg_t basicRfConfig;
volatile unsigned char timer_experied = 0;
uint8_t payload[48];

static void receive(void);
static void UARTlibinit(void);

int main(void){

	// Initialize board
	bspInit(BSP_SYS_CLK_SPD);

	// Initialize keys and key interrupts
	bspKeyInit(BSP_KEY_MODE_ISR);
	bspKeyIntEnable(BSP_USER_KEY);

	UARTlibinit();

	moteConfig.mode = MOTE_MODE_RX;
    	moteConfig.channel = CHANNEL;
    	moteConfig.txPower = 5;          		// Index 0. Max output
    	moteConfig.gainMode = MOTE_GAIN_MODE_NONE; 	// No PA/LNA

	// Enable interrupts
    	halIntOn();

	// Config basicRF
    	basicRfConfig.panId = PAN_ID;
    	basicRfConfig.ackRequest = false;
	
	receive();

	
	return 0;
}

static void receive(void){

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

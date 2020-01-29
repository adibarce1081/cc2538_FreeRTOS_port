
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
#include "sys_ctrl.h"
     

moteCfg_t moteConfig = {0};
basicRfCfg_t basicRfConfig;
volatile unsigned char timer_experied = 0;
uint8_t data[]="Hi..... I am Ubimote-SEZ";
uint8_t payload[MAX_PAYLOAD];
static void transmit(void);
static void UARTlibinit(void);
static void delay(uint16_t mSec);
static void timerIsr(void);

int main(void){

	// Initialize board
	bspInit(BSP_SYS_CLK_SPD);

	// Initialize keys and key interrupts
	//bspKeyInit(BSP_KEY_MODE_ISR);
	//bspKeyIntEnable(BSP_USER_KEY);

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

	transmit();
}

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
	
	memcpy(&(payload[1]),data, payload[0]);
	
	while(1){
		
		bspLedClear(BSP_LED_ALL);
		basicRfSendPacket(RX_ADDR, (unsigned char *)payload, (payload[0])+1);
		
		bspLedSet(BSP_LED_ALL);
		
		delay(1000);
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


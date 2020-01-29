#include "systick.h"
#include "interrupt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"
#include "semphr.h"
#include "hw_memmap.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_ioc.h"
#include "sys_ctrl.h"
#include "gpio.h"
#include "ioc.h"
#include "bsp_led.h"

#define GPIO_C_DIR                      0x400DB400
#define GPIO_C_DATA                     0x400DB000
#define EXAMPLE_PIN_UART_RXD            GPIO_PIN_0
#define EXAMPLE_PIN_UART_TXD            GPIO_PIN_1
#define EXAMPLE_INT_UART0               INT_UART0
#define EXAMPLE_GPIO_BASE               GPIO_A_BASE

TaskHandle_t RxTaskHandle = NULL,task1_H=NULL,task2_H=NULL;
SemaphoreHandle_t task1,task2;

void Task1( void *pvParameters );
void Task2( void *pvParameters );
void RxTx( void *pvParameters );
void rxIT(void);

int main()
{
/******************************************************************INITIALIZATION*******************************************************************************/
      BaseType_t ret = 0;
      SysCtrlClockGet();
      SysTickEnable();
      UARTIntRegister(BSP_UART_BASE, &rxIT);
      IntPrioritySet(INT_UART0, 160);
      bspLedInit();
          //
          // Set the clocking to run directly from the external crystal/oscillator.
          // (no ext 32k osc, no internal osc)
          //
          SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_32MHZ);

          //
          // Set IO clock to the same as system clock
          //
          SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);

          //
          // Enable UART peripheral module
          //
          SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_UART0);

          //
          // Disable UART function
          //
          UARTDisable(UART0_BASE);

          //
          // Disable all UART module interrupts
          //
          UARTIntDisable(UART0_BASE, 0x1FFF);

          //
          // Set IO clock as UART clock source
          //
          UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

          //
          // Map UART signals to the correct GPIO pins and configure them as
          // hardware controlled.
          //
          IOCPinConfigPeriphOutput(EXAMPLE_GPIO_BASE, EXAMPLE_PIN_UART_TXD, IOC_MUX_OUT_SEL_UART0_TXD);
          GPIOPinTypeUARTOutput(EXAMPLE_GPIO_BASE, EXAMPLE_PIN_UART_TXD);
          IOCPinConfigPeriphInput(EXAMPLE_GPIO_BASE, EXAMPLE_PIN_UART_RXD, IOC_UARTRXD_UART0);
          GPIOPinTypeUARTInput(EXAMPLE_GPIO_BASE, EXAMPLE_PIN_UART_RXD);

          //
          // Configure the UART for 115,200, 8-N-1 operation.
          // This function uses SysCtrlClockGet() to get the system clock
          // frequency.  This could be also be a variable or hard coded value
          // instead of a function call.
          //

          UARTConfigSetExpClk(UART0_BASE, SysCtrlClockGet(), 115200,
                              (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                               UART_CONFIG_PAR_NONE));
          //UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

          UARTIntEnable(UART0_BASE, UART_INT_RX);/* setting interrupt register of UART0 */
          IntMasterEnable();                     /* Enabling Master Interrupt           */
          IntEnable(EXAMPLE_INT_UART0);          /* Enabling uart peripheral interrupt  */
          //UARTFIFODisable(UART0_BASE);           /* Disabling FIFO                      */
          //UARTFIFOEnable(UART0_BASE);
          UARTEnable(UART0_BASE);                /* enabling UART                       */
          //
          // Put a character to show start of example.  This will display on the
          // terminal.
          //
          task2= xSemaphoreCreateBinary();
          task1= xSemaphoreCreateBinary();

/***************************************************************************************************************************************************************/


       /********************** MAIN APPLICATION CODE ****************************/
        UARTCharPut(UART0_BASE, '!');


      //TASK creation
      ret = xTaskCreate(Task1,"task1",240,NULL,tskIDLE_PRIORITY + 1UL,&task1_H );
      configASSERT(ret);
      ret = xTaskCreate(Task2,"task2",240,NULL,tskIDLE_PRIORITY + 1UL,&task2_H );
      configASSERT(ret);
      ret = xTaskCreate(RxTx,"transmit",240,NULL,tskIDLE_PRIORITY + 1UL,&RxTaskHandle );
      configASSERT(ret);


      vTaskStartScheduler();

/**  IDLE TASK  **/
      while(1);
/**  *********  **/
}
// crude TOGGLE
void Toggle1()
{
    HWREG(GPIO_C_DATA + (0x30 << 2)) ^= 0x30;
}
void Toggle2()
{
    HWREG(GPIO_C_DATA + (0xc0 << 2)) ^= 0xc0;
}

/*****TASK's****************************************************************************************************************************************************/
void Task2( void *pvParameters )
{
        for( ;; )
        {
           xSemaphoreTake(task2,portMAX_DELAY);
           Toggle2();
        }
}

void Task1( void *pvParameters )
{

        for( ;; )
        {
           xSemaphoreTake(task1,portMAX_DELAY);
           Toggle1();
        }
}

void RxTx( void *pvParameters )
{
    char cThisChar;
    for(;;)
    {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      cThisChar = UARTCharGet(UART0_BASE);
      UARTCharPut(UART0_BASE,cThisChar);
      if(cThisChar == '1')
      {
          xSemaphoreGive(task1);
      }
      if(cThisChar == '2')
      {
         xSemaphoreGive(task2);
      }
    }
}
/**************************************************************************************************************************************************TASK*********/

/*-----------------------------------------------------------------  RTOS AWARE ISR's -------------------------------------------------------------------*/

void rxIT()
{
    UARTIntClear(BSP_UART_BASE, UART_INT_RX);
    BaseType_t xHigherPriorityTaskWoken;
    uint32_t ulIntBm = UARTIntStatus(BSP_UART_BASE, 1);
    xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(RxTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*-----------------------------------------------------------------  ERROR HANDLER HOOK  ------------------------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
        /* This function will only be called if an API call to create a task, queue
        or semaphore fails because there is too little heap RAM remaining. */
        for( ;; );
}
void vApplicationTickHook( void )
{
    //for(;;);   /* This example does not use the tick hook to perform any processing. */
}


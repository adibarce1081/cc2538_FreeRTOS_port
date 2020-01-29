/******************************************************************************
*  Filename:       uart_polled.c
*  Revised:        $Date: 2013-04-10 11:21:26 +0200 (Wed, 10 Apr 2013) $
*  Revision:       $Revision: 9703 $
*
*  Description:    Example demonstrating UART polled I/O.
*
*  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_ioc.h"
#include "sys_ctrl.h"
#include "uart.h"
#include "gpio.h"
#include "ioc.h"


//
// NOTE:
// This example has been run on a SmartRF06 board.
// A terminal program on a PC was connected to the usb serial port.
// The example will echo characters typed on the terminal until you press the
// enter key - then the program ends.
//
//*****************************************************************************
//
//! \addtogroup uart_examples_list
//! <h1>UART Polled I/O (uart_polled)</h1>
//!
//! This example shows how to set up the UART and use polled I/O methods
//! for transmitting and receiving UART data.  The example receives characters
//! from UART0 and retransmits the same character using UART0.  It can be
//! tested by using a serial terminal program on a host computer.  This
//! example will echo every character that is type until the return/enter key
//! is pressed.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//
//*****************************************************************************

#define EXAMPLE_PIN_UART_RXD            GPIO_PIN_0 
#define EXAMPLE_PIN_UART_TXD            GPIO_PIN_1 
#define EXAMPLE_GPIO_BASE               GPIO_A_BASE

//*****************************************************************************
//
// Configure the UART and perform reads and writes using polled I/O.
//
//*****************************************************************************
int
main(void)
{
    char cThisChar;
    char *str = "Hello Word!\n";
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
    UARTEnable(UART0_BASE);
    
    //
    // Put a character to show start of example.  This will display on the
    // terminal.
    //
    while(*str != '\n')
    {
    	UARTCharPut(UART0_BASE, *str);

    	str++;
    }
    UARTCharPut(UART0_BASE, '!');

    //
    // Enter a loop to read characters from the UART, and write them back
    // (echo).  When a line end is received, the loop terminates.
    //
    do
    {
        //
        // Read a character using the blocking read function.  This function
        // will not return until a character is available.
        //
        cThisChar = UARTCharGet(UART0_BASE);

        //
        // Write the same character using the blocking write function.  This
        // function will not return until there was space in the FIFO and
        // the character is written.
        //
        UARTCharPut(UART0_BASE, cThisChar);

    //
    // Stay in the loop until either a CR or LF is received.
    //
    } while((cThisChar != '\n') && (cThisChar != '\r'));

    //
    // Put a character to show the end of the example.  This will display on
    // the terminal.
    //
    UARTCharPut(UART0_BASE, '@');

    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }    
}


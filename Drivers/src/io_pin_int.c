//*****************************************************************************
//! @file       io_pin_int.c
//! @brief      I/O pin interrupt handler for CC2538. This is an extension to
//!             the driverlib GPIO module, allowing GPIO pins on the same GPIO
//!             port to have different interrupt handlers.
//!
//!             \b Use:
//!
//!             1) Register a custom interrupt handler using ioPinIntRegister.
//!
//!             2) Unregister a custom interrupt handler using ioPinIntRegister
//!
//!             When registering a custom interrupt handler to GPIO a pin, a
//!             generic interrupt handler is assigned to the GPIO port. The
//!             generic interrupt handler calls the custom interrupt handler.
//!
//!             If an interrupt is triggered on a pin without a custom interrupt
//!             handler, the generic interrupt handler simply clears the pin's
//!             interrupt flag.
//!
//!             Example using driverlib and the extension: Assuming interrupts
//!             are disabled:
//!             \code
//!             // Register interrupt handler to GPIO port C, pin 0
//!             ioPinIntRegister(GPIO_C_BASE, GPIO_PIN_0, &myIsr);
//!
//!             // Set interrupt type to rising edge
//!             GPIOIntTypeSet(GPIO_C_BASE, GPIO_PIN_0, GPIO_RISING_EDGE);
//!
//!             // Enable pin interrupt (driverlib function)
//!             GPIOPinIntEnable(GPIO_C_BASE, GPIO_PIN_0);
//!
//!             // Enable master interrupt (driverlib function)
//!             IntMasterEnable();
//!             \endcode
//!
//! Revised     $Date: 2013-04-11 19:41:57 +0200 (Thu, 11 Apr 2013) $
//! Revision    $Revision: 9707 $
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/
#ifndef IO_PIN_INT_EXCLUDE


/**************************************************************************//**
* @addtogroup io_pin_int_api
* @{
******************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "io_pin_int.h"
#include "gpio.h"


/******************************************************************************
* LOCAL VARIABLES AND PROTOTYPES
*/
//
// Lookup variables (a set bit in these variables means the corresponding GPIO
// pin has a custom ISR)
static uint_fast8_t ui8PortAPinHasIsr;
static uint_fast8_t ui8PortBPinHasIsr;
static uint_fast8_t ui8PortCPinHasIsr;
static uint_fast8_t ui8PortDPinHasIsr;

//
// Function pointer arrays
//
static void (*pfnPortAIsr[8])(void);
static void (*pfnPortBIsr[8])(void);
static void (*pfnPortCIsr[8])(void);
static void (*pfnPortDIsr[8])(void);

//
// Generic GPIO port ISRs
//
static void ioPortAIsr(void);
static void ioPortBIsr(void);
static void ioPortCIsr(void);
static void ioPortDIsr(void);


/**************************************************************************//**
* @brief    Register an interrupt handler to the GPIO pin (or pins) specified
*           by bitmask \e ui8Pins on GPIO port given by \e ui32Base.
* 			This function registers a general ISR to the GPIO port and then
*           assigns the ISR specified by \e pfnIntHandler to the given pins.
*
* @param    ui32Base        is the base address of the GPIO port.
* @param    ui8Pins         is the bit-packed representation of the pin (or
*                           pins).
* @param    pfnIntHandler   is a pointer to the interrupt handler function.
*
* @return   None
******************************************************************************/
void
ioPinIntRegister(uint32_t ui32Base, uint8_t ui8Pins,
                 void (*pfnIntHandler)(void))
{
    volatile uint_fast8_t ui8Cnt;

    //
    // Disable global interrupts
    //
    bool bIntDisabled = IntMasterDisable();

    //
    // If function pointer is not null, register that pin has a custom
    // handler to the fast access 'ui8PortXPinHasIsr' variables.
    //
    if(pfnIntHandler)
    {
        switch(ui32Base)
        {
        case GPIO_A_BASE:
            if(!ui8PortAPinHasIsr)
            {
                GPIOPortIntRegister(GPIO_A_BASE, &ioPortAIsr);
            }
            ui8PortAPinHasIsr |=  ui8Pins;
            break;
        case GPIO_B_BASE:
            if(!ui8PortBPinHasIsr)
            {
                GPIOPortIntRegister(GPIO_B_BASE, &ioPortBIsr);
            }
            ui8PortBPinHasIsr |=  ui8Pins;
            break;
        case GPIO_C_BASE:
            if(!ui8PortCPinHasIsr)
            {
                GPIOPortIntRegister(GPIO_C_BASE, &ioPortCIsr);
            }
            ui8PortCPinHasIsr |=  ui8Pins;
            break;
        case GPIO_D_BASE:
            if(!ui8PortDPinHasIsr)
            {
                GPIOPortIntRegister(GPIO_D_BASE, &ioPortDIsr);
            }
            ui8PortDPinHasIsr |=  ui8Pins;
            break;
        }
    }

    //
    // Iterate over port pins and store handler into the correct lookup table.
    //
    for(ui8Cnt = 0; ui8Cnt < 8; ui8Cnt++)
    {
        if(ui8Pins & (1 << ui8Cnt))
        {
            switch(ui32Base)
            {
            case GPIO_A_BASE:
                pfnPortAIsr[ui8Cnt] = pfnIntHandler;
                break;
            case GPIO_B_BASE:
                pfnPortBIsr[ui8Cnt] = pfnIntHandler;
                break;
            case GPIO_C_BASE:
                pfnPortCIsr[ui8Cnt] = pfnIntHandler;
                break;
            case GPIO_D_BASE:
                pfnPortDIsr[ui8Cnt] = pfnIntHandler;
                break;
            default:
                break;
            }
        }
    }

    //
    // Clear interrupts on specified pins
    //
    GPIOPinIntClear(ui32Base, ui8Pins);

    //
    // If interrupt was enabled, re-enable
    //
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }
}


/**************************************************************************//**
* @brief    Unregister the interrupt handler to GPIO pin (or pins) specified by
*           bitmask \e ui8Pins on GPIO port \e ui32Base.
*
* @param    ui32Base    is the base address of the GPIO port.
* @param    ui8Pins     is the bit-packed representation of the pin (or pins).
*
* @return   None
******************************************************************************/
void
ioPinIntUnregister(uint32_t ui32Base, uint8_t ui8Pins)
{
    //
    // Disable global interrupts
    //
    bool bIntDisabled = IntMasterDisable();

    //
    // Register null function to pins
    // Doing this is not necessary, but is less confusing during debug. If not
    // cleared, the pins' interrupt vector tables may be non-null even when
    // no custom ISR is actually registered. Note that it is the
    // ui8PortXPinHasIsr variables that are used to decide whether a custom
    // interrupt is registered or not.
    //
    ioPinIntRegister(ui32Base, ui8Pins, 0);

    //
    // Clear "pin has ISR" variables
    //
    switch(ui32Base)
    {
    case GPIO_A_BASE:
        ui8PortAPinHasIsr &= ~ui8Pins;
        break;
    case GPIO_B_BASE:
        ui8PortBPinHasIsr &= ~ui8Pins;
        break;
    case GPIO_C_BASE:
        ui8PortCPinHasIsr &= ~ui8Pins;
        break;
    case GPIO_D_BASE:
        ui8PortDPinHasIsr &= ~ui8Pins;
        break;
    }

    //
    // If interrupt was enabled, re-enable
    //
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }
}


/******************************************************************************
* LOCAL FUNCTIONS
*/

/**************************************************************************//**
* @internal
*
* @brief    General purpose IO interrupt function for GPIO port A.
*           If an interrupt function handler is defined for the pins with
*           its interrupt flag set, this function is called. The interrupt
*           flag for this pin is then cleared. If no custom ISR is registered,
*           function simply clears the interrupt pin flag(s) and returns.
*
* @return   None
******************************************************************************/
static void
ioPortAIsr(void)
{
    register uint_fast8_t ui8IntBm, ui8IsrBm;

    //
    // Get bitmask of pins with interrupt triggered
    //
    ui8IntBm = GPIOPinIntStatus(GPIO_A_BASE, true);

    //
    // Create bitmask of pins with interrupt _and_ custom isr
    //
    ui8IsrBm = (ui8IntBm & ui8PortAPinHasIsr);

    //
    // Run custom isr if any and clear the interrupt
    //
    if((ui8IsrBm & GPIO_PIN_0))
    {
        (*pfnPortAIsr[0])();
        GPIOPinIntClear(GPIO_A_BASE, GPIO_PIN_0);
    }
    if((ui8IsrBm & GPIO_PIN_1))
    {
        (*pfnPortAIsr[1])();
        GPIOPinIntClear(GPIO_A_BASE, GPIO_PIN_1);
    }
    if((ui8IsrBm & GPIO_PIN_2))
    {
        (*pfnPortAIsr[2])();
        GPIOPinIntClear(GPIO_A_BASE, GPIO_PIN_2);
    }
    if((ui8IsrBm & GPIO_PIN_3))
    {
        (*pfnPortAIsr[3])();
        GPIOPinIntClear(GPIO_A_BASE, GPIO_PIN_3);
    }
    if((ui8IsrBm & GPIO_PIN_4))
    {
        (*pfnPortAIsr[4])();
        GPIOPinIntClear(GPIO_A_BASE, GPIO_PIN_4);
    }
    if((ui8IsrBm & GPIO_PIN_5))
    {
        (*pfnPortAIsr[5])();
        GPIOPinIntClear(GPIO_A_BASE, GPIO_PIN_5);
    }
    if((ui8IsrBm & GPIO_PIN_6))
    {
        (*pfnPortAIsr[6])();
        GPIOPinIntClear(GPIO_A_BASE, GPIO_PIN_6);
    }
    if((ui8IsrBm & GPIO_PIN_7))
    {
        (*pfnPortAIsr[7])();
        GPIOPinIntClear(GPIO_A_BASE, GPIO_PIN_7);
    }

    //
    // Clear flag for pins with no custom isr
    //
    GPIOPinIntClear(GPIO_A_BASE, (ui8IntBm & (~ui8IsrBm)));
}


/**************************************************************************//**
* @internal
*
* @brief    General purpose IO interrupt function for GPIO port B.
*
* @see      ioPortAIsr
*
* @return   None
******************************************************************************/
static void
ioPortBIsr(void)
{
    register uint_fast8_t ui8IntBm, ui8IsrBm;

    //
    // Get bitmask of pins with interrupt triggered
    //
    ui8IntBm = GPIOPinIntStatus(GPIO_B_BASE, true);

    //
    // Create bitmask of pins with interrupt _and_ custom isr
    //
    ui8IsrBm = (ui8IntBm & ui8PortBPinHasIsr);

    //
    // Run custom isr for the pins in question and clear the interrupt
    //
    if((ui8IsrBm & GPIO_PIN_0))
    {
        (*pfnPortBIsr[0])();
        GPIOPinIntClear(GPIO_B_BASE, GPIO_PIN_0);
    }
    if((ui8IsrBm & GPIO_PIN_1))
    {
        (*pfnPortBIsr[1])();
        GPIOPinIntClear(GPIO_B_BASE, GPIO_PIN_1);
    }
    if((ui8IsrBm & GPIO_PIN_2))
    {
        (*pfnPortBIsr[2])();
        GPIOPinIntClear(GPIO_B_BASE, GPIO_PIN_2);
    }
    if((ui8IsrBm & GPIO_PIN_3))
    {
        (*pfnPortBIsr[3])();
        GPIOPinIntClear(GPIO_B_BASE, GPIO_PIN_3);
    }
    if((ui8IsrBm & GPIO_PIN_4))
    {
        (*pfnPortBIsr[4])();
        GPIOPinIntClear(GPIO_B_BASE, GPIO_PIN_4);
    }
    if((ui8IsrBm & GPIO_PIN_5))
    {
        (*pfnPortBIsr[5])();
        GPIOPinIntClear(GPIO_B_BASE, GPIO_PIN_5);
    }
    if((ui8IsrBm & GPIO_PIN_6))
    {
        (*pfnPortBIsr[6])();
        GPIOPinIntClear(GPIO_B_BASE, GPIO_PIN_6);
    }
    if((ui8IsrBm & GPIO_PIN_7))
    {
        (*pfnPortBIsr[7])();
        GPIOPinIntClear(GPIO_B_BASE, GPIO_PIN_7);
    }

    //
    // Clear flag for pins with no custom isr
    //
    GPIOPinIntClear(GPIO_B_BASE, (ui8IntBm & (~ui8IsrBm)));
}


/**************************************************************************//**
* @internal
*
* @brief    General purpose IO interrupt function for GPIO port C.
*
* @see      ioPortAIsr
*
* @return   None
******************************************************************************/
static void
ioPortCIsr(void)
{
    register uint_fast8_t ui8IntBm, ui8IsrBm;

    //
    // Get bitmask of pins with interrupt triggered
    //
    ui8IntBm = GPIOPinIntStatus(GPIO_C_BASE, true);

    //
    // Create bitmask of pins with interrupt _and_ custom isr
    //
    ui8IsrBm = (ui8IntBm & ui8PortCPinHasIsr);

    //
    // Run custom isr for the pins in question and clear the interrupt
    //
    if((ui8IsrBm & GPIO_PIN_0))
    {
        (*pfnPortCIsr[0])();
        GPIOPinIntClear(GPIO_C_BASE, GPIO_PIN_0);
    }
    if((ui8IsrBm & GPIO_PIN_1))
    {
        (*pfnPortCIsr[1])();
        GPIOPinIntClear(GPIO_C_BASE, GPIO_PIN_1);
    }
    if((ui8IsrBm & GPIO_PIN_2))
    {
        (*pfnPortCIsr[2])();
        GPIOPinIntClear(GPIO_C_BASE, GPIO_PIN_2);
    }
    if((ui8IsrBm & GPIO_PIN_3))
    {
        (*pfnPortCIsr[3])();
        GPIOPinIntClear(GPIO_C_BASE, GPIO_PIN_3);
    }
    if((ui8IsrBm & GPIO_PIN_4))
    {
        (*pfnPortCIsr[4])();
        GPIOPinIntClear(GPIO_C_BASE, GPIO_PIN_4);
    }
    if((ui8IsrBm & GPIO_PIN_5))
    {
        (*pfnPortCIsr[5])();
        GPIOPinIntClear(GPIO_C_BASE, GPIO_PIN_5);
    }
    if((ui8IsrBm & GPIO_PIN_6))
    {
        (*pfnPortCIsr[6])();
        GPIOPinIntClear(GPIO_C_BASE, GPIO_PIN_6);
    }
    if((ui8IsrBm & GPIO_PIN_7))
    {
        (*pfnPortCIsr[7])();
        GPIOPinIntClear(GPIO_C_BASE, GPIO_PIN_7);
    }

    //
    // Clear flag for pins with no custom isr
    //
    GPIOPinIntClear(GPIO_C_BASE, (ui8IntBm & (~ui8IsrBm)));
}

/**************************************************************************//**
* @internal
*
* @brief    General purpose IO interrupt function for GPIO port D.
*
* @see      ioPortAIsr
*
* @return   None
******************************************************************************/
static void
ioPortDIsr(void)
{
    register uint_fast8_t ui8IntBm, ui8IsrBm;

    //
    // Get bitmask of pins with interrupt triggered
    //
    ui8IntBm = GPIOPinIntStatus(GPIO_D_BASE, true);

    //
    // Create bitmask of pins with interrupt _and_ custom isr
    //
    ui8IsrBm = (ui8IntBm & ui8PortDPinHasIsr);

    //
    // Run custom isr for the pins in question and clear the interrupt
    //
    if((ui8IsrBm & GPIO_PIN_0))
    {
        (*pfnPortDIsr[0])();
        GPIOPinIntClear(GPIO_D_BASE, GPIO_PIN_0);
    }
    if((ui8IsrBm & GPIO_PIN_1))
    {
        (*pfnPortDIsr[1])();
        GPIOPinIntClear(GPIO_D_BASE, GPIO_PIN_1);
    }
    if((ui8IsrBm & GPIO_PIN_2))
    {
        (*pfnPortDIsr[2])();
        GPIOPinIntClear(GPIO_D_BASE, GPIO_PIN_2);
    }
    if((ui8IsrBm & GPIO_PIN_3))
    {
        (*pfnPortDIsr[3])();
        GPIOPinIntClear(GPIO_D_BASE, GPIO_PIN_3);
    }
    if((ui8IsrBm & GPIO_PIN_4))
    {
        (*pfnPortDIsr[4])();
        GPIOPinIntClear(GPIO_D_BASE, GPIO_PIN_4);
    }
    if((ui8IsrBm & GPIO_PIN_5))
    {
        (*pfnPortDIsr[5])();
        GPIOPinIntClear(GPIO_D_BASE, GPIO_PIN_5);
    }
    if((ui8IsrBm & GPIO_PIN_6))
    {
        (*pfnPortDIsr[6])();
        GPIOPinIntClear(GPIO_D_BASE, GPIO_PIN_6);
    }
    if((ui8IsrBm & GPIO_PIN_7))
    {
        (*pfnPortDIsr[7])();
        GPIOPinIntClear(GPIO_D_BASE, GPIO_PIN_7);
    }

    //
    // Clear flag for pins with no custom isr
    //
    GPIOPinIntClear(GPIO_D_BASE, (ui8IntBm & (~ui8IsrBm)));
}


/**************************************************************************//**
* Close the Doxygen group.
* @}
******************************************************************************/
#endif // #ifndef IO_PIN_INT_EXCLUDE

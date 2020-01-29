//*****************************************************************************
//! @file       bsp_led.c
//! @brief      LED board support package for CCxxxx Cortex devices on
//!             SmartRF06 Battery/Evaluation Board.
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
#ifndef BSP_LED_EXCLUDE


/**************************************************************************//**
* @addtogroup bsp_led_api
* @{
******************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "bsp_led.h"


/******************************************************************************
* FUNCTIONS
*/
/**************************************************************************//**
* @brief    This function initializes GPIO pins connected to LEDs. LEDs are
*           initialized to be off. The function bspInit() does the same LED
*           initialization as this function.
*
* @return   None
******************************************************************************/
void
bspLedInit(void)
{
    //
    // Set GPIO pins as output low
    //
    GPIOPinTypeGPIOOutput(BSP_LED_BASE, BSP_LED_ALL);
    GPIOPinWrite(BSP_LED_BASE, BSP_LED_ALL, 0);
    bspLedSet(BSP_LED_ALL);
}


/**************************************************************************//**
* @brief    This function sets LED(s) specified by \e ui8Leds. The function
*           assumes that LED pins have been initialized by, for example,
*           bspLedInit().
*
* @param    ui8Leds      is an ORed bitmask of LEDs (for example \b BSP_LED_1).
*
* @return   None
******************************************************************************/
void
bspLedSet(uint8_t ui8Leds)
{
    //
    // Turn on specified LEDs
    //
    GPIOPinWrite(BSP_LED_BASE, ui8Leds, ui8Leds);
}


/**************************************************************************//**
* @brief    This function clears LED(s) specified by \e ui8Leds.
*           This function assumes that LED pins have been initialized by,
*           for example, bspLedInit().
*
* @param    ui8Leds      is an ORed bitmask of LEDs (for example \b BSP_LED_1).
*
* @return   None
******************************************************************************/
void
bspLedClear(uint8_t ui8Leds)
{
    //
    // Turn off specified LEDs
    //
    GPIOPinWrite(BSP_LED_BASE, ui8Leds, 0);
}

/**************************************************************************//**
* @brief    This function toggles LED(s) specified by \e ui8Leds. The function
*           assumes that LED pins have been initialized by, for example,
*           bspLedInit().
*
* @param    ui8Leds      ORed bitmask of LEDs (for example \b BSP_LED_1).
*
* @return   None
******************************************************************************/
void
bspLedToggle(uint8_t ui8Leds)
{
    //
    // Get current pin values of selected bits
    //
    uint32_t ui32Toggle = GPIOPinRead(BSP_LED_BASE, ui8Leds);

    //
    // Invert selected bits
    //
    ui32Toggle = (~ui32Toggle) & ui8Leds;

    //
    // Set GPIO
    //
    GPIOPinWrite(BSP_LED_BASE, ui8Leds, ui32Toggle);
}


/**************************************************************************//**
* Close the Doxygen group.
* @}
******************************************************************************/
#endif // #ifndef BSP_LED_EXCLUDE

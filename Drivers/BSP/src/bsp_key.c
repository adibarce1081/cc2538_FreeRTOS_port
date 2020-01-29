//*****************************************************************************
//! @file       bsp_key.c
//! @brief      Key board support package for CC2538 on SmartRF06EB/BB.
//!             Key debounce is by default implemented using a timer.
//!             The user may register custom ISRs using the bspKeyIntRegister()
//!             function.
//!
//!             If a custom ISR is registered, it will be called prior to
//!             starting the debounce timer.
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
#ifndef BSP_KEY_EXCLUDE


/**************************************************************************//**
* @addtogroup   bsp_key_api
* @{
******************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "bsp_key.h"
#include "ioc.h"                // Access to driverlib IOC_xx defines
#include "gpio.h"               // Access to driverlib IOC_xx defines
#include "io_pin_int.h"         // Access to GPIO pin specific ISRs
#include "interrupt.h"          // Access to driverlib interrupt fns
#include "hw_smwdthrosc.h"


/******************************************************************************
* DEFINES
*/
// Number of keys on board.
#define BSP_KEY_COUNT           5
// Active wait debounce macro
#define BSP_KEY_DEBOUNCE(expr)  { do {uint16_t i; for(i = 0; i < 500; i++) {  \
                                  if (!(expr)) i = 0; } } while(0);}


/******************************************************************************
* LOCAL VARIABLES AND FUNCTION PROTOTYPES
*/
static volatile uint_fast8_t bspKeysPressed;
static volatile uint_fast8_t bspKeyIntDisabledMask;
static void (*bspKeysIsrTable[BSP_KEY_COUNT])(void);
static uint_fast8_t ui8BspKeyMode;

static void bspKeyDirPushedISR(void);
static void bspKeySelPushedISR(void);

static void bspKeyTimerISR(void);
static void bspKeyTimerEnable(void);
static void bspKeyTimerDisable(void);
static void bspKeyTimerIntRegister(void (*pfnHandler)(void));


/******************************************************************************
* FUNCTIONS
*/
/**************************************************************************//**
* @brief    This function initializes key GPIO as input pullup and disables
*           interrupts. If \e ui8Mode is \b BSP_KEY_MODE_POLL, key presses are
*           handled using polling and active state debounce. Functions starting
*           with \b bspKeyInt then do nothing.
*
*           If \e ui8Mode is \b BSP_KEY_MODE_ISR, key presses are handled by
*           interrupts, and debounce is implemented using a timer.
*
* @param    ui8Mode is the operation mode; must be one of the following:
*                   \li \b BSP_KEY_MODE_POLL for polling-based handling
*                   \li \b BSP_KEY_MODE_ISR for interrupt-based handling
* @return   None
******************************************************************************/
void
bspKeyInit(uint8_t ui8Mode)
{
    //
    // Store mode
    //
    ui8BspKeyMode = ui8Mode;

    //
    // Initialize keys on GPIO port C (input pullup)
    //
    //GPIOPinTypeGPIOInput(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL);
    //IOCPadConfigSet(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL, IOC_OVERRIDE_PUE);

    //
    // Initialize SELECT key on GPIO port A (input pullup)
    //
    GPIOPinTypeGPIOInput(BSP_USER_KEY_BASE, BSP_USER_KEY);
    IOCPadConfigSet(BSP_USER_KEY_BASE, BSP_USER_KEY, IOC_OVERRIDE_PUE);

    //
    // Disable interrupts
    //
    //GPIOPinIntDisable(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL);
    GPIOPinIntDisable(BSP_USER_KEY_BASE, BSP_USER_KEY);

    if(ui8BspKeyMode == BSP_KEY_MODE_ISR)
    {
        //
        // Connect bspKeyPushedISR() to key pins
        //
        //ioPinIntRegister(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL, &bspKeyDirPushedISR);
        ioPinIntRegister(BSP_USER_KEY_BASE, BSP_USER_KEY, &bspKeySelPushedISR);

        //
        // Set trigger type
        //
        //GPIOIntTypeSet(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL, GPIO_RISING_EDGE);
        GPIOIntTypeSet(BSP_USER_KEY_BASE, BSP_USER_KEY, GPIO_RISING_EDGE);

        //
        // Disable timer
        //
        HWREG(SMWDTHROSC_WDCTL) &= ~SMWDTHROSC_WDCTL_EN;
    }
}


/**************************************************************************//**
* @brief    This function returns a bitmask of keys pushed.
*
* @note     If keys are handled using polling (\b BSP_KEY_MODE_POLL), the
*           returned bitmask will never contain a combination of multiple key
*           bitmasks, for example, (\b BSP_KEY_LEFT |\b BSP_KEY_UP).
*           Furthermore, in this case argument \e ui8ReadMask is ignored.
*
* @param    ui8ReadMask     is a bitmask of keys to read. Read keys are cleared
*                           and new key presses can be registered. Use
*                           \b BSP_KEY_ALL to read status of all keys.
*
* @return   Returns bitmask of pushed keys
******************************************************************************/
uint8_t
bspKeyPushed(uint8_t ui8ReadMask)
{
    if(ui8BspKeyMode == BSP_KEY_MODE_ISR)
    {
        uint8_t ui8Bm = 0;

        //
        // Disable global interrupts
        //
        bool bIntDisabled = IntMasterDisable();

        //
        // Critical section
        //
        ui8Bm = bspKeysPressed;
        bspKeysPressed &= ~ui8ReadMask;

        //
        // Re-enable interrupt if initially enabled.
        //
        if(!bIntDisabled)
        {
            IntMasterEnable();
        }

        //
        // Return bitmask of pushed keys
        //
        return ui8Bm;
    }
    else
    {
        //
        // Read pin states
        //
        //uint32_t ui32DirPins = ~GPIOPinRead(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL);
        uint32_t ui32SelPin  = ~GPIOPinRead(BSP_KEY_SEL_BASE, BSP_KEY_SELECT);

        //
        // Check LEFT key
        //
        /*if(ui32DirPins & BSP_KEY_LEFT)
        {
            BSP_KEY_DEBOUNCE(GPIOPinRead(BSP_KEY_DIR_BASE, BSP_KEY_LEFT));
            return BSP_KEY_LEFT;
        }

        //
        // Check RIGHT key
        //
        if(ui32DirPins & BSP_KEY_RIGHT)
        {
            BSP_KEY_DEBOUNCE(GPIOPinRead(BSP_KEY_DIR_BASE, BSP_KEY_RIGHT));
            return BSP_KEY_RIGHT;
        }

        //
        // Check UP key
        //
        if(ui32DirPins & BSP_KEY_UP)
        {
            BSP_KEY_DEBOUNCE(GPIOPinRead(BSP_KEY_DIR_BASE, BSP_KEY_UP));
            return BSP_KEY_UP;
        }

        //
        // Check DOWN key
        //
        if(ui32DirPins & BSP_KEY_DOWN)
        {
            BSP_KEY_DEBOUNCE(GPIOPinRead(BSP_KEY_DIR_BASE, BSP_KEY_DOWN));
            return BSP_KEY_DOWN;
        }*/

        //
        // Check SELECT key
        //
        if(ui32SelPin & BSP_KEY_SELECT)
        {
            BSP_KEY_DEBOUNCE(GPIOPinRead(BSP_KEY_SEL_BASE, BSP_KEY_SELECT));
            return BSP_KEY_SELECT;
        }

        //
        // No keys pressed
        //
        return 0;
    }
}


/**************************************************************************//**
* @brief    This function reads the directional event. If multiple keys are
*           registered as "pressed", this function will only return the
*           directional event of the first key. Remaining key events will
*           be ignored. \sa bspKeyPushed()
*
* @return   Returns \b BSP_KEY_EVT_LEFT if LEFT key has been pressed.
* @return   Returns \b BSP_KEY_EVT_RIGHT if RIGHT key has been pressed.
* @return   Returns \b BSP_KEY_EVT_UP if UP key has been pressed.
* @return   Returns \b BSP_KEY_EVT_DOWN if DOWN key has been pressed.
* @return   Returns \b BSP_KEY_EVT_NONE if no key has been pressed.
******************************************************************************/
uint8_t
bspKeyGetDir(void)
{
    //
    // Get bitmask of pressed keys
    //
    uint8_t bitmask = bspKeyPushed(BSP_KEY_ALL);

    //
    // Return directional event based on bitmask of pressed keys
    //
    if(bitmask & BSP_KEY_LEFT)
    {
        return BSP_KEY_EVT_LEFT;
    }
    if(bitmask & BSP_KEY_RIGHT)
    {
        return BSP_KEY_EVT_RIGHT;
    }
    if(bitmask & BSP_KEY_UP)
    {
        return BSP_KEY_EVT_UP;
    }
    if(bitmask & BSP_KEY_DOWN)
    {
        return BSP_KEY_EVT_DOWN;
    }
    if(bitmask & BSP_KEY_SELECT)
    {
        return BSP_KEY_EVT_SELECT;
    }

    return BSP_KEY_EVT_NONE;
}


/**************************************************************************//**
* @brief    This function registers a custom ISR to keys specified by
*           \e ui8Keys.
*
* @note     If bspKeyInit() was initialized with argument \b BSP_KEY_MODE_POLL,
*           this function does nothing.
*
* @param    ui8Keys     is an ORed bitmask of keys (for example BSP_KEY_1).
* @param    pfnHandler  is a void function pointer to ISR.
*
* @return   None
******************************************************************************/
void
bspKeyIntRegister(uint8_t ui8Keys, void (*pfnHandler)(void))
{
    if(ui8BspKeyMode == BSP_KEY_MODE_ISR)
    {
        //
        // Assign handler to the specified key(s)
        //
        if(ui8Keys & BSP_KEY_SELECT)
        {
            bspKeysIsrTable[0] = pfnHandler;
        }
        if(ui8Keys & BSP_KEY_LEFT)
        {
            bspKeysIsrTable[1] = pfnHandler;
        }
        if(ui8Keys & BSP_KEY_RIGHT)
        {
            bspKeysIsrTable[2] = pfnHandler;
        }
        if(ui8Keys & BSP_KEY_UP)
        {
            bspKeysIsrTable[3] = pfnHandler;
        }
        if(ui8Keys & BSP_KEY_DOWN)
        {
            bspKeysIsrTable[4] = pfnHandler;
        }
    }
}


/**************************************************************************//**
* @brief    This function clears the custom ISR from keys specified by
*           \e ui8Keys.
*
* @note     If bspKeyInit() was initialized with argument \b BSP_KEY_MODE_POLL,
*           this function does nothing.
*
* @param    ui8Keys     is an ORed bitmask of keys (for example BSP_KEY_1).
*
* @return   None
******************************************************************************/
void
bspKeyIntUnregister(uint8_t ui8Keys)
{
    if(ui8BspKeyMode == BSP_KEY_MODE_ISR)
    {
        //
        // Unregister handler for the specified keys
        //
        bspKeyIntRegister(ui8Keys, 0);
    }
}


/**************************************************************************//**
* @brief    This function enables interrupts on specified key GPIO pins.
*
* @note     If bspKeyInit() was initialized with argument \b BSP_KEY_MODE_POLL,
*           this function does nothing.
*
* @param    ui8Keys     is an ORed bitmask of keys (for example BSP_KEY_1).
*
* @return   None
******************************************************************************/
void
bspKeyIntEnable(uint8_t ui8Keys)
{
    if(ui8BspKeyMode == BSP_KEY_MODE_ISR)
    {
        //
        // Enable interrupt for pins on "dir" port:
        //
        //GPIOPinIntEnable(BSP_KEY_DIR_BASE, (ui8Keys & BSP_KEY_DIR_ALL));

        //
        // Enable interrupt for pins on "select" port:
        //
        GPIOPinIntEnable(BSP_KEY_SEL_BASE, (ui8Keys & BSP_KEY_SELECT));
    }
}


/**************************************************************************//**
* @brief    This function disables interrupts on specified key GPIOs.
*
* @note     If bspKeyInit() was initialized with argument \b BSP_KEY_MODE_POLL,
*           this function does nothing.
*
* @param    ui8Keys     is an ORed bitmask of keys (for example BSP_KEY_1).
*
* @return   None
******************************************************************************/
void
bspKeyIntDisable(uint8_t ui8Keys)
{
    if(ui8BspKeyMode == BSP_KEY_MODE_ISR)
    {
        //
        // Disable interrupt for pins on "dir" port:
        //
        //GPIOPinIntDisable(BSP_KEY_DIR_BASE, (ui8Keys & BSP_KEY_DIR_ALL));

        //
        // Disable interrupt for pins on "select" port:
        //
        GPIOPinIntDisable(BSP_KEY_SEL_BASE, (ui8Keys & BSP_KEY_SELECT));
    }
}


/**************************************************************************//**
* @brief    This function clears interrupt flags on selected key GPIOs.
*
* @note     If bspKeyInit() was initialized with argument \b BSP_KEY_MODE_POLL,
*           this function does nothing.
*
* @param    ui8Keys     is an ORed bitmask of keys (for example BSP_KEY_1).
*
* @return   None
******************************************************************************/
void
bspKeyIntClear(uint8_t ui8Keys)
{
    if(ui8BspKeyMode == BSP_KEY_MODE_ISR)
    {
        //
        // Clear interrupt flag for selected pins on "dir" port:
        //
        //GPIOPinIntClear(BSP_KEY_DIR_BASE, (ui8Keys & BSP_KEY_DIR_ALL));

        //
        // Clear interrupt flag for selected pins on "select" port:
        //
        GPIOPinIntClear(BSP_KEY_SEL_BASE, (ui8Keys & BSP_KEY_SELECT));
    }
}


/******************************************************************************
* LOCAL FUNCTIONS
*/
/**************************************************************************//**
* @brief    Interrupt Service Routine for an activated directional key.
*           Stores the pin where the interrupt occured, disables the interrupt
*           on that pin and starts the debouncing by use of WDT.
*
* @return   None
******************************************************************************/
static void
bspKeyDirPushedISR(void)
{
    register uint_fast8_t ui8IrqBm;
    bool bIntDisabled;

    //
    // Disable WDT.
    //
    bspKeyTimerDisable();

    //
    // Get mask of directional keys from interrupt flag status bitmask.
    //
    ui8IrqBm = (GPIOPinIntStatus(BSP_KEY_DIR_BASE, true) & BSP_KEY_DIR_ALL);

    //
    // Critical section: Disable global interrupts, update volatile
    // variables and re-enable global interrupts.
    //
    bIntDisabled = IntMasterDisable();
    bspKeysPressed |= ui8IrqBm;
    bspKeyIntDisabledMask |= ui8IrqBm;
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }

    //
    // Disable interrupts on keys where interrupt flag was set.
    //
    GPIOPinIntDisable(BSP_KEY_DIR_BASE,
                      (bspKeyIntDisabledMask & BSP_KEY_DIR_ALL));

    //
    // Run custom ISR if any (unrolled for speed)
    //
    if((ui8IrqBm & BSP_KEY_LEFT)  && (bspKeysIsrTable[1] != 0))
    {
        (*bspKeysIsrTable[1])();
    }
    if((ui8IrqBm & BSP_KEY_RIGHT) && (bspKeysIsrTable[2] != 0))
    {
        (*bspKeysIsrTable[2])();
    }
    if((ui8IrqBm & BSP_KEY_UP)    && (bspKeysIsrTable[3] != 0))
    {
        (*bspKeysIsrTable[3])();
    }
    if((ui8IrqBm & BSP_KEY_DOWN)  && (bspKeysIsrTable[4] != 0))
    {
        (*bspKeysIsrTable[4])();
    }

    //
    // Start the debounce timer
    //
    bspKeyTimerIntRegister(&bspKeyTimerISR);
    bspKeyTimerEnable();
}


/**************************************************************************//**
* @brief    Interrupt Service Routine for the select key. The select key is
*           separated from the directional keys because it's located on a
*           different GPIO port.
*
* @see      bspKeyDirPushedISR
*
* @return   None
******************************************************************************/
static void
bspKeySelPushedISR(void)
{
    uint_fast8_t ui8IrqBm;
    bool bIntDisabled;

    //
    // Disable debounce timer
    //
    bspKeyTimerDisable();

    //
    // Get interrupt flag status from select port
    //
    ui8IrqBm = (GPIOPinIntStatus(BSP_USER_KEY_BASE, true) & BSP_USER_KEY);


    //
    // Critical section: Disable global interrupts, update volatile
    // variables and re-enable global interrupts.
    //
    bIntDisabled = IntMasterDisable();
    bspKeysPressed |= ui8IrqBm;
    bspKeyIntDisabledMask |= ui8IrqBm;
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }

    //
    // Disable interrupts on keys where interrupt flag was set.
    //
    GPIOPinIntDisable(BSP_USER_KEY_BASE, (bspKeyIntDisabledMask & BSP_USER_KEY));

    //
    // Run custom ISR if any
    //
    if((ui8IrqBm & BSP_USER_KEY) && (bspKeysIsrTable[0] != 0))
    {
        (*bspKeysIsrTable[0])();
    }

    //
    // Start debounce timer
    //
    bspKeyTimerIntRegister(&bspKeyTimerISR);
    bspKeyTimerEnable();
}


/**************************************************************************//**
* @brief    Interrupt Service Routine for an activated key.
*           Stores the pin where the interrupt occured, disables the interrupt
*           on that pin and starts the debouncing timer.
*
* @return   None
******************************************************************************/
static void
bspKeyTimerISR(void)
{
    //
    // Disable timer
    //
    bspKeyTimerDisable();

    //
    // Clear pending interrupts
    //
    //GPIOPinIntClear(BSP_KEY_DIR_BASE, (bspKeyIntDisabledMask & BSP_KEY_DIR_ALL));
    GPIOPinIntClear(BSP_USER_KEY_BASE, (bspKeyIntDisabledMask & BSP_USER_KEY));

    //
    // Re-enable the pin interrupts
    //
    //GPIOPinIntEnable(BSP_KEY_DIR_BASE, (bspKeyIntDisabledMask & BSP_KEY_DIR_ALL));
    GPIOPinIntEnable(BSP_USER_KEY_BASE, (bspKeyIntDisabledMask & BSP_USER_KEY));

    //
    // Clear disabled bitmask
    //
    bspKeyIntDisabledMask = 0;
}


/**************************************************************************//**
* @brief    Configure and start debounce timer.
*
* @return   None
******************************************************************************/
static void
bspKeyTimerEnable(void)
{
    uint32_t ui32Regval;

    //
    // Disable Timer and set interval
    //
    bspKeyTimerDisable();
    ui32Regval = HWREG(SMWDTHROSC_WDCTL);
    ui32Regval &= ~SMWDTHROSC_WDCTL_INT_M;
    ui32Regval |= 0x00000001; // Debounce timer x 8192
    HWREG(SMWDTHROSC_WDCTL) = ui32Regval;

    //
    // Set mode and Enable the debounce timer.
    //
    ui32Regval = HWREG(SMWDTHROSC_WDCTL);
    ui32Regval |= (4 | 8);
    HWREG(SMWDTHROSC_WDCTL) = ui32Regval;
}


/**************************************************************************//**
* @brief    Disable debounce timer.
*
* @return   None
******************************************************************************/
static void
bspKeyTimerDisable(void)
{
    HWREG(SMWDTHROSC_WDCTL) &= ~SMWDTHROSC_WDCTL_EN;
}


/**************************************************************************//**
* @brief    Register interrupt handler to debounce timer interrupt vector.
*
* @param    pfnHandler  is a pointer to the interrupt handler.
*
* @return   None
******************************************************************************/
static void
bspKeyTimerIntRegister(void (*pfnHandler)(void))
{
    //
    // Register the interrupt handler and enable interrupt
    //
    IntRegister(34, pfnHandler);
    IntEnable(34);
}

/**************************************************************************//**
* Close the Doxygen group.
* @}
******************************************************************************/
#endif // #ifndef BSP_KEY_EXCLUDE

//*****************************************************************************
//! @file       bsp.c
//! @brief      Board support package for CC2538 Cortex devices on SmartRF06EB.
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


/**************************************************************************//**
* @addtogroup bsp_api
* @{
******************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "bsp.h"
#include "hw_ioc.h"         // Access to IOC register defines
#include "hw_ssi.h"         // Access to SSI register defines
#include "ioc.h"            // Access to driverlib ioc fns
#include "gpio.h"           // Access to driverlib gpio fns
#include "sys_ctrl.h"       // Access to driverlib SysCtrl fns
#include "interrupt.h"      // Access to driverlib interrupt fns
#include "ssi.h"            // Access to driverlib ssi fns


/******************************************************************************
* LOCAL VARIABLES
*/
//
// 3.3-V domain enable call counter
//
static int8_t i8Bsp3V3DomainEnableCount = 0;


/******************************************************************************
* FUNCTIONS
*/

/**************************************************************************//**
* @brief    This function initializes the CC2538 clocks and I/O for use on
*           SmartRF06EB.
*
*           The function assumes that an external crystal oscillator is
*           available to the CC2538. The CC2538 system clock is set to the
*           frequency given by input argument \e ui32SysClockSpeed. The I/O
*           system clock is set configured to the same value as the system
*           clock.
*
*           If the value of \e ui32SysClockSpeed is invalid, the system clock
*           is set to the highest allowed value.
*
* @param    ui32SysClockSpeed   is the system clock speed in Hz; it must be one
*                               of the following:
*           \li \b SYS_CTRL_32MHZ
*           \li \b SYS_CTRL_16MHZ
*           \li \b SYS_CTRL_8MHZ
*           \li \b SYS_CTRL_4MHZ
*           \li \b SYS_CTRL_2MHZ
*           \li \b SYS_CTRL_1MHZ
*           \li \b SYS_CTRL_500KHZ
*           \li \b SYS_CTRL_250KHZ
*
* @return   None
******************************************************************************/
void
bspInit(uint32_t ui32SysClockSpeed)
{
    uint32_t ui32SysDiv;

    //
    // Disable global interrupts
    //
    bool bIntDisabled = IntMasterDisable();

    //
    // Determine sys clock divider and realtime clock
    //
    switch(ui32SysClockSpeed)
    {
    case SYS_CTRL_250KHZ:
        ui32SysDiv = SYS_CTRL_SYSDIV_250KHZ;
        break;
    case SYS_CTRL_500KHZ:
        ui32SysDiv = SYS_CTRL_SYSDIV_500KHZ;
        break;
    case SYS_CTRL_1MHZ:
        ui32SysDiv = SYS_CTRL_SYSDIV_1MHZ;
        break;
    case SYS_CTRL_2MHZ:
        ui32SysDiv = SYS_CTRL_SYSDIV_2MHZ;
        break;
    case SYS_CTRL_4MHZ:
        ui32SysDiv = SYS_CTRL_SYSDIV_4MHZ;
        break;
    case SYS_CTRL_8MHZ:
        ui32SysDiv = SYS_CTRL_SYSDIV_8MHZ;
        break;
    case SYS_CTRL_16MHZ:
        ui32SysDiv = SYS_CTRL_SYSDIV_16MHZ;
        break;
    case SYS_CTRL_32MHZ:
    default:
        ui32SysDiv = SYS_CTRL_SYSDIV_32MHZ;
        break;
    }

    //
    // Set system clock (no ext 32k osc, no internal osc)
    //
    SysCtrlClockSet(false, false, ui32SysDiv);

    //
    // Set IO clock to the same as system clock
    //
    SysCtrlIOClockSet(ui32SysDiv);

    //
    // LEDs (off, output low)
    //
    GPIOPinTypeGPIOOutput(BSP_LED_BASE, BSP_LED_ALL);
    GPIOPinWrite(BSP_LED_BASE, BSP_LED_ALL, 0);

    //
    // Keys (input pullup)
    //
#if 0
    GPIOPinTypeGPIOInput(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL);
    IOCPadConfigSet(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL, IOC_OVERRIDE_PUE);
    GPIOPinTypeGPIOInput(BSP_KEY_SEL_BASE, BSP_KEY_SELECT);
    IOCPadConfigSet(BSP_KEY_SEL_BASE, BSP_KEY_SELECT, IOC_OVERRIDE_PUE);
#endif
    //
    // Turn off 3.3-V domain (lcd/sdcard power, output low)
    //
    GPIOPinTypeGPIOOutput(BSP_3V3_EN_BASE, BSP_3V3_EN);
    GPIOPinWrite(BSP_3V3_EN_BASE, BSP_3V3_EN, 0);

    //
    // LCD CSn (output high)
    //
#if 0
    GPIOPinTypeGPIOOutput(BSP_LCD_CS_BASE, BSP_LCD_CS);
    GPIOPinWrite(BSP_LCD_CS_BASE, BSP_LCD_CS, BSP_LCD_CS);

    //
    // SD Card reader CSn (output high)
    //
    GPIOPinTypeGPIOOutput(BSP_SDCARD_CS_BASE, BSP_SDCARD_CS);
    GPIOPinWrite(BSP_SDCARD_CS_BASE, BSP_SDCARD_CS, BSP_SDCARD_CS);

    //
    // Accelerometer (PWR output low, CSn output high)
    //
    GPIOPinTypeGPIOOutput(BSP_ACC_PWR_BASE, BSP_ACC_PWR);
    GPIOPinWrite(BSP_ACC_PWR_BASE, BSP_ACC_PWR, 0);
    GPIOPinTypeGPIOOutput(BSP_ACC_CS_BASE, BSP_ACC_CS);
    GPIOPinWrite(BSP_ACC_CS_BASE, BSP_ACC_CS, BSP_ACC_CS);

    //
    // Ambient light sensor (off, output low)
    //
    GPIOPinTypeGPIOOutput(BSP_ALS_PWR_BASE, BSP_ALS_PWR);
    GPIOPinWrite(BSP_ALS_PWR_BASE, BSP_ALS_PWR, 0);
#endif
    //
    // UART Backchannel (TXD/RXD/CTS/RTS input pullup)
    //
    GPIOPinTypeGPIOInput(BSP_UART_BUS_BASE, (BSP_UART_TXD | BSP_UART_RXD));
    IOCPadConfigSet(BSP_UART_BUS_BASE, (BSP_UART_TXD | BSP_UART_RXD),
                    IOC_OVERRIDE_PUE);
    GPIOPinTypeGPIOInput(BSP_UART_CTS_BASE, BSP_UART_CTS);
    IOCPadConfigSet(BSP_UART_CTS_BASE, BSP_UART_CTS, IOC_OVERRIDE_PUE);
    GPIOPinTypeGPIOInput(BSP_UART_RTS_BASE, BSP_UART_RTS);
    IOCPadConfigSet(BSP_UART_RTS_BASE, BSP_UART_RTS, IOC_OVERRIDE_PUE);

    //
    // Re-enable interrupt if initially enabled.
    //
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }
}


/**************************************************************************//**
* @brief    This function initializes SPI interface. The SPI is configured to
*           Motorola mode with clock idle high and data valid on the second
*           (rising) edge. The SSI module uses the I/O clock as clock source
*           (I/O clock frequency set in bspInit()).
*
*           Input argument \e ui32SpiClockSpeed must obey the following
*           criteria:
*           \li \e ui32SpiClockSpeed = srcClk / 2
*           where n is integer, n >= 2, and srcClk is the clock frequency set
*           by bspInit().
*
* @param    ui32SpiClockSpeed   is the SPI clock speed in Hz
*
* @return   None
******************************************************************************/
void
bspSpiInit(uint32_t ui32SpiClockSpeed)
{
    uint32_t ui32Dummy;

    //
    // Enable SSI peripheral module
    //
    SysCtrlPeripheralEnable(BSP_SPI_SSI_ENABLE_BM);

    //
    // Disable SSI function before configuring module
    //
    SSIDisable(BSP_SPI_SSI_BASE);

    //
    // Set IO clock as SSI clock source
    //
    SSIClockSourceSet(BSP_SPI_SSI_BASE, SSI_CLOCK_PIOSC);

    //
    // Map SSI signals to the correct GPIO pins and configure them as HW ctrl'd
    //
    IOCPinConfigPeriphOutput(BSP_SPI_BUS_BASE, BSP_SPI_SCK,
                             IOC_MUX_OUT_SEL_SSI0_CLKOUT);
    IOCPinConfigPeriphOutput(BSP_SPI_BUS_BASE, BSP_SPI_MOSI,
                             IOC_MUX_OUT_SEL_SSI0_TXD);
    IOCPinConfigPeriphInput(BSP_SPI_BUS_BASE, BSP_SPI_MISO,
                            IOC_SSIRXD_SSI0);
    GPIOPinTypeSSI(BSP_SPI_BUS_BASE, (BSP_SPI_MOSI | BSP_SPI_MISO | BSP_SPI_SCK));

    //
    // Set SPI mode and speed
    //
    bspSpiClockSpeedSet(ui32SpiClockSpeed);

    //
    // Enable the SSI function
    //
    SSIEnable(BSP_SPI_SSI_BASE);

    //
    // Flush the RX FIFO
    //
    while(SSIDataGetNonBlocking(BSP_SPI_SSI_BASE, &ui32Dummy))
    {
    }
}


/**************************************************************************//**
* @brief    This function returns the clock speed of the BSP SPI interface.
*           It is assumed that the BSP SPI SSI module runs off the I/O clock.
*
* @return   Returns the SPI clock speed in Hz
******************************************************************************/
uint32_t
bspSpiClockSpeedGet(void)
{
    uint32_t ui32PreDiv, ui32Scr;

    //
    // Get current SSI settings
    //
    ui32PreDiv = HWREG(BSP_SPI_SSI_BASE + SSI_O_CPSR);
    ui32Scr = ((HWREG(BSP_SPI_SSI_BASE + SSI_O_CR0) & 0x0000FF00) >> 8);

    //
    // Return clock speed
    //
    return (SysCtrlIOClockGet() / (ui32PreDiv * (1 + ui32Scr)));
}


/**************************************************************************//**
* @brief    This function configures the SPI interface to the given clock
*           speed, Motorola mode with clock idle high and data valid on the
*           second (rising) edge. For proper SPI function, the SPI interface
*           must first be initialized using bspSpiInit().
*
* @warning  Limitations apply to the allowed values of \e ui32ClockSpeed.
*           Please refer to device's driverlib documentation.
*
* @param    ui32ClockSpeed  is the SPI clock speed in Hz
*
* @return   None
******************************************************************************/
void
bspSpiClockSpeedSet(uint32_t ui32ClockSpeed)
{
    //
    // Disable SSI function
    //
    SSIDisable(BSP_SPI_SSI_BASE);

    //
    // Configure SSI module to Motorola/Freescale SPI mode 3:
    // Polarity  = 1, SCK steady state is high
    // Phase     = 1, Data changed on first and captured on second clock edge
    // Word size = 8 bits
    //
    SSIConfigSetExpClk(BSP_SPI_SSI_BASE, SysCtrlIOClockGet(),
                       SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, ui32ClockSpeed, 8);

    //
    // Enable the SSI function
    //
    SSIEnable(BSP_SPI_SSI_BASE);
}


/**************************************************************************//**
* @brief    This function enables the 3.3-V domain on SmartRF06EB. The LCD and
*           SD card reader are powered by the 3.3-V domain. This functon
*           increments a counter variable each time it is called. The function
*           assumes the 3.3-V domain enable pin is configured as output by,
*           for example, bspInit().
*           The 3.3-V domain needs up to approximately 400 us to settle when
*           enabled.
*
* @see      bsp3V3DomainDisable(), bsp3V3DomainDisableForced()
*
* @return   None
******************************************************************************/
void
bsp3V3DomainEnable(void)
{
    //
    // Keep score of the enable/disable relation
    //
    i8Bsp3V3DomainEnableCount++;

    //
    // Enable 3.3-V domain
    //
    GPIOPinWrite(BSP_3V3_EN_BASE, BSP_3V3_EN, BSP_3V3_EN);  // high
}


/**************************************************************************//**
* @brief    This function disables the 3.3-V domain on SmartRF06EB. This
*           function is "soft" and only disables the 3.3-V domain if counter
*           variable \e i8Bsp3V3DomainEnableCount is 1 or 0. The function
*           assumes the 3.3-V domain enable pin is configured as output by,
*           for example, bspInit().

*           This function decrements \e i8Bsp3V3DomainEnableCount and disables
*           the 3.3-V domain if \e i8Bsp3V3DomainEnableCount is less than or
*           equal to 0. If \e i8Bsp3V3DomainEnableCount is greater than 0
*           after decrement, function bsp3V3DomainEnable() has been called more
*           times than this function, and the 3.3-V domain will not be disabled.
*           To disable the 3.3-V domain irrespective of the value of
*           \e i8Bsp3V3DomainEnableCount, use bsp3V3DomainDisableForced().
*
* @see      bsp3V3DomainEnable(), bsp3V3DomainDisableForced()
*
* @return   None
******************************************************************************/
void
bsp3V3DomainDisable(void)
{
    //
    // Only disable 3.3-V domain if disable requests >= enable requests.
    //
    if((--i8Bsp3V3DomainEnableCount) <= 0)
    {
        //
        // Disable 3.3-V domain
        //
        GPIOPinWrite(BSP_3V3_EN_BASE, BSP_3V3_EN, 0);
        i8Bsp3V3DomainEnableCount = 0;
    }
}


/**************************************************************************//**
* @brief    This function disables the 3.3-V domain on SmartRF06EB. The function
*           assumes the 3.3-V domain enable pin is configured as output by,
*           for example, bspInit().
*           The 3.3-V domain needs approximately 400 us to fall below 0.5 V.
*
* @see      bsp3V3DomainEnable(), bsp3V3DomainDisable()
*
* @return   None
******************************************************************************/
void
bsp3V3DomainDisableForced(void)
{
    //
    // Disable 3.3-V domain and reset score variable
    //
    GPIOPinWrite(BSP_3V3_EN_BASE, BSP_3V3_EN, 0);
    i8Bsp3V3DomainEnableCount = 0;
}


/**************************************************************************//**
* @brief    This function returns the current state of the 3.3-V domain.
*
* @return   Returns 1 if 3.3-V domain is enabled
* @return   Returns 0 if 3.3-V domain is disabled
******************************************************************************/
uint8_t
bsp3V3DomainEnabled(void)
{
    //
    // Get pin state
    //
    return (!!(GPIOPinRead(BSP_3V3_EN_BASE, BSP_3V3_EN) & 0xFF));
}


/**************************************************************************//**
* @brief    Assert function. Eternal loop that blinks all LEDs quickly.
*           Function assumes LEDs to be initialized by, for example, bspInit().
*
* @return   None
******************************************************************************/
void
bspAssert(void)
{
    uint32_t ui32Delay = 1000000UL;
    uint8_t ui8SetLeds = 1;

    while(1)
    {
        if(ui8SetLeds)
        {
            //
            // Turn on all LEDs
            //
            GPIOPinWrite(BSP_LED_BASE, BSP_LED_ALL, BSP_LED_ALL);
        }
        else
        {
            //
            // Turn off all LEDs
            //
            GPIOPinWrite(BSP_LED_BASE, BSP_LED_ALL, 0);
        }

        //
        // Wait and update LEDs' "on/off" variable
        //
        SysCtrlDelay(ui32Delay);
        ui8SetLeds ^= 1;
    }
}


/**************************************************************************//**
* Close the Doxygen group.
* @}
******************************************************************************/

//*****************************************************************************
//! @file       bsp_uart.c
//! @brief      UART implementation to SmartRF06EB's UART backchannel.
//!             Implementation does not support flow control.
//!
//!             Configuration options:
//!             If \b BSP_UART_ISR is defined, UART communication is CPU ISR
//!             driven.
//!
//!             If \b BSP_UART_ALL_OR_NOTHING is defined, functions
//!             bspUartDataGet() and bspUartDataPut() will only read(write)
//!             from(to) the UART buffer if the specified number of bytes
//!             are available (can fit) in the UART buffer. Eor example, if 3
//!             bytes are present in the UART RX buffer bspUartDataGet(myBuf, 4)
//!             will not read any bytes whereas bspUartDataGet(myBuf, 3) will
//!             read 3 bytes.
//!
//!             If \b BSP_UART_ALLOCATE_ISR is defined, the BSP UART module
//!             will allocate the necessary ISRs.
//!
//! Revised     $Date: 2013-04-12 13:57:26 +0200 (Fri, 12 Apr 2013) $
//! Revision    $Revision: 9725 $
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
#ifndef BSP_UART_EXCLUDE


/**************************************************************************//**
* @addtogroup   bsp_uart_api
* @{
******************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "bsp.h"
#include "bsp_uart.h"

#include <uart.h>               // Access to driverlib uart fns
#include <interrupt.h>          // Access to driverlib interrupt fns
#include <sys_ctrl.h>           // Access to driverlib sys_ctrl fns
#include <ioc.h>                // Access to driverlib IO control fns
#include <gpio.h>               // Access to driverlib gpio fns

#include <hw_ioc.h>             // Access to IOC defines
#include <hw_uart.h>            // Access to UART defines


/******************************************************************************
* DEFINES
*/
//
// Configuration handling:
// If not specified, use CPU ISR
//
#if (!defined(BSP_UART_ISR))
#define BSP_UART_ISR
#endif

//
// This macro gets the byte size of the specified BSP UART ring buffer.
//
#define BU_GET_BUF_SIZE(pRbc)       ((uint32_t)((pRbc)->pui8End) -            \
                                     (uint32_t)((pRbc)->pui8Start))
//
// This macro gets the number of used bytes in the specified BSP UART
// ring buffer.
//
#define BU_GET_USED_SPACE(pRbc)     ((pRbc)->ui16NumBytes)

//
// This macro gets the number of unused bytes in the specified BSP UART
// ring buffer.
//
#define BU_GET_FREE_SPACE(pRbc)     (BU_GET_BUF_SIZE(pRbc) -                  \
                                     ((pRbc)->ui16NumBytes))

//
// This macro checks if the specified BSP UART ring buffer is empty.
//
#define BU_IS_BUF_EMPTY(pRbc)       ((pRbc)->ui16NumBytes == 0)

//
// This macro checks if the specified BSP UART ring buffer is full.
//
#define BU_IS_BUF_FULL(pRbc)        (BU_GET_USED_SPACE(pRbc) ==               \
                                     BU_GET_BUF_SIZE(pRbc))
//
// This macro gets the byte count to tail wrap for the specified BSP UART
// ring buffer.
//
#define BU_BYTES_TO_TAIL_WRAP(pRbc) ((uint32_t)((pRbc)->pui8End) -            \
                                     (uint32_t)((pRbc)->pui8Tail))


/******************************************************************************
* TYPEDEFS
*/
//
//! Ringbuffer implementation control structure
//
typedef struct
{
    //
    //! Lowest address in buffer
    //
    uint8_t *pui8Start;

    //
    //! Highest address in buffer + 1
    //
    uint8_t *pui8End;

    //
    //! Buffer data input pointer (push)
    //
    volatile uint8_t *pui8Head;

    //
    //! Buffer data output pointer (pop)
    //
    volatile uint8_t *pui8Tail;

    //
    //! Number of stored bytes
    //
    volatile uint16_t ui16NumBytes;
}
tBuBuf;

typedef struct
{
	uint8_t UART_mode;
	uint32_t UART_BaudRate;
	uint8_t UART_StopBits;
	uint8_t UART_DataLength;
	uint8_t UART_ParityCtrl;
	uint8_t UART_HwFlowCtrl;
	uint8_t UART_FIFOEn;
	uint8_t UART_FIFOTxLvl;
	uint8_t UART_FIFORxLvl;
}UART_Config_t;
typedef struct
{
	tBuBuf sBuBufTx;
	tBuBuf sBuBufRx;
	uint32_t ui32Base;
	UART_Config_t uart_config;

}UARTHandle_t;



/******************************************************************************
* MACROS
*/


/******************************************************************************
* LOCAL FUNCTIONS AND VARIABLES
*/

//
// Holds currently configured baud rate
//
static uint32_t ui32BuBaudRate;

//
// RX buffer control structure
//
static tBuBuf sBuBufRx;

//
// TX buffer control structure
//
static tBuBuf sBuBufTx;

//
// Functions
//
static void buBufFlush(tBuBuf *psBuf);
static void buBufPopByte(void);
#if defined(BSP_UART_ISR)
static void buBufPushByte(void);
#endif


/******************************************************************************
* FUNCTIONS
*/
/**************************************************************************//**
* @brief    This function initializes buffers used by BSP UART module.
*
* @param    pui8TxBuf       is a pointer to the TX buffer.
* @param    ui16TxByteAlloc is the size of the TX buffer in bytes.
* @param    pui8RxBuf       is a pointer to the RX buffer.
* @param    ui16RxByteAlloc is the size of the RX buffer in bytes.
*
* @return   Returns BSP_UART_SUCCESS on success.
* @return   Returns BSP_UART_ERROR on configuration error.
******************************************************************************/
uint8_t
bspUartBufInit(uint8_t *pui8TxBuf, uint16_t ui16TxByteAlloc,
               uint8_t *pui8RxBuf, uint16_t ui16RxByteAlloc)
{
    //
    // Check input arguments
    //
    if((pui8RxBuf == 0) || (ui16RxByteAlloc == 0) ||                          \
            (pui8TxBuf == 0) || (ui16TxByteAlloc == 0))
    {
        return BSP_UART_ERROR;
    }

    //
    // Store pointers
    //
    sBuBufTx.pui8Start = pui8TxBuf;
    sBuBufTx.pui8End = pui8TxBuf + ui16TxByteAlloc;
    sBuBufRx.pui8Start = pui8RxBuf;
    sBuBufRx.pui8End = pui8RxBuf + ui16RxByteAlloc;

    //
    // Reset control structure
    //
    buBufFlush(&sBuBufTx);
    buBufFlush(&sBuBufRx);

    //
    // Return status
    //
    return BSP_UART_SUCCESS;
}


/**************************************************************************//**
* @brief    This function initializes UART communication at the baud rate
*           specified by \e ui32BaudRate. This function must be called after
*           initializing the UART buffers using bspUartBufInit().
*           The UART format between the BSP MCU is 8-N-1 (that is, 8 data bits,
*           no parity bit, and 1 stop bit). The implementation does not support
*           flow control.
*
*           The UART module is configured to use IO clock as clock source.
*
* @param    ui32BaudRate    is the UART baud rate. The baud rate must be one of
*                           the following enumerated values:
*           \li \b eBaudRate9600
*           \li \b eBaudRate38400
*           \li \b eBaudRate57600
*           \li \b eBaudRate115200
*           \li \b eBaudRate230400
*           \li \b eBaudRate460800
*
* @return   Returns BSP_UART_SUCCESS on success.
* @return   Returns BSP_UART_UNCONFIGURED if buffers are not configured.
* @return   Returns BSP_UART_BAUDRATE_ERROR if the baud rate is not supported.
******************************************************************************/
uint8_t
bspUartOpen(uint32_t ui32BaudRate)
{
    if((BU_GET_BUF_SIZE(&sBuBufTx) == 0) || (BU_GET_BUF_SIZE(&sBuBufRx) == 0))
    {
        //
        // One or more buffer size is zero -> no buffers configured
        //
        return BSP_UART_UNCONFIGURED;
    }

    //
    // Store baud rate
    //
    switch(ui32BaudRate)
    {
    case eBaudRate9600:
        ui32BuBaudRate = 9600;
        break;
    case eBaudRate38400:
        ui32BuBaudRate = 38400;
        break;
    case eBaudRate57600:
        ui32BuBaudRate = 57600;
        break;
    case eBaudRate115200:
        ui32BuBaudRate = 115200;
        break;
    case eBaudRate230400:
        ui32BuBaudRate = 230400;
        break;
    case eBaudRate460800:
        ui32BuBaudRate = 460800;
        break;
    default:
        return BSP_UART_BAUDRATE_ERROR;
    }

    //
    // Enable UART peripheral module
    //
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_UART0);

    //
    // Disable UART function
    //
    UARTDisable(BSP_UART_BASE);

    //
    // Disable all UART module interrupts
    //
    UARTIntDisable(BSP_UART_BASE, 0x17FF);

    //
    // Set IO clock as UART clock source
    //
    UARTClockSourceSet(BSP_UART_BASE, UART_CLOCK_PIOSC);

    //
    // Map UART signals to the correct GPIO pins and configure them as
    // hardware controlled.
    //
    IOCPinConfigPeriphOutput(BSP_UART_BUS_BASE, BSP_UART_TXD,
                             IOC_MUX_OUT_SEL_UART0_TXD);
    GPIOPinTypeUARTOutput(BSP_UART_BUS_BASE, BSP_UART_TXD);
    IOCPinConfigPeriphInput(BSP_UART_BUS_BASE, BSP_UART_RXD,
                            IOC_UARTRXD_UART0);
    GPIOPinTypeUARTInput(BSP_UART_BUS_BASE, BSP_UART_RXD);

#if (defined(BSP_UART_ALLOCATE_ISR) && defined(BSP_UART_ISR))
    //
    // Register UART interrupt handler
    //
    UARTIntRegister(BSP_UART_BASE, &bspUartIsrHandler);
#endif

    //
    // Configure and enable UART module
    //
    UARTConfigSetExpClk(BSP_UART_BASE, SysCtrlClockGet(), ui32BuBaudRate,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |          \
                         UART_CONFIG_PAR_NONE));

#ifdef BSP_UART_ISR
    //
    // TX interrupts occur on FIFO threshold
    //
    UARTTxIntModeSet(BSP_UART_BASE, UART_TXINT_MODE_FIFO);

    //
    // Configure FIFO interrupt threshold (half full)
    //
    UARTFIFOLevelSet(BSP_UART_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

    //
    // Clear all UART module interrupt flags
    //
    UARTIntClear(BSP_UART_BASE, 0x17FF);

    //
    // Enable UART module interrupts (RX, TX , RX timeout)
    //
    UARTIntEnable(BSP_UART_BASE, (UART_INT_RX | UART_INT_TX | UART_INT_RT));
#endif

    //
    // Enable UART function
    //
    UARTEnable(BSP_UART_BASE);

    //
    // Report success
    //
    return BSP_UART_SUCCESS;
}


/**************************************************************************//**
* @brief    This function stops UART communication.
*
* @return   Returns BSP_UART_SUCCESS on success.
******************************************************************************/
uint8_t
bspUartClose(void)
{
    //
    // Disable UART module
    //
    UARTDisable(BSP_UART_BASE);

    //
    // Set RXD/TXD pins back as GPIO input pullup
    //
    GPIOPinTypeGPIOInput(BSP_UART_BUS_BASE, (BSP_UART_TXD | BSP_UART_RXD));
    IOCPadConfigSet(BSP_UART_BUS_BASE, (BSP_UART_TXD | BSP_UART_RXD),
                    IOC_OVERRIDE_PUE);

    //
    // Clear baud rate variable
    //
    ui32BuBaudRate = 0;

    return BSP_UART_SUCCESS;
}


/**************************************************************************//**
* @brief    This function reads up to \e ui16Length bytes from the BSP UART RX
*           buffer into the buffer specified by \e pui8Data.
*
*           If \b BSP_UART_ALL_OR_NOTHING is defined, data is read only if
*           \e ui16Length or more bytes are available in the BSP UART RX
*           buffer.
*
* @param    pui8Data        is a pointer to the destination buffer.
* @param    ui16Length      is the number of bytes to transfer.
*
* @return   Returns the number of bytes read from the BSP UART RX buffer.
******************************************************************************/
uint16_t
bspUartDataGet(uint8_t *pui8Data, uint16_t ui16Length)
{
    bool bIntDisabled;
    register uint_fast16_t ui1N;
    uint_fast16_t ui1Bytes;

    //
    // Data available in RX buffer?
    //
    ui1Bytes = bspUartRxCharsAvail();

    //
    // Return early if no data available/requested
    //
    if(!ui1Bytes || !ui16Length)
    {
        return 0;
    }

    if(ui16Length <= ui1Bytes)
    {
        //
        // Limit by input argument
        //
        ui1Bytes = ui16Length;
    }
#ifdef BSP_UART_ALL_OR_NOTHING
    else
    {
        //
        // All or nothing. Not enough data in buffer, fetching nothing.
        //
        return 0;
    }
#endif

    //
    // Copy data to application buffer
    //
    for(ui1N = 0; ui1N < ui1Bytes; ui1N++)
    {
        pui8Data[ui1N] = *sBuBufRx.pui8Tail++;

        //
        // Handle ringbuffer wrap-around
        //
        if(sBuBufRx.pui8Tail == sBuBufRx.pui8End)
        {
            sBuBufRx.pui8Tail = sBuBufRx.pui8Start;
        }
    }

    //
    // Critical section. Update volatile count variable
    //
    bIntDisabled = IntMasterDisable();
    sBuBufRx.ui16NumBytes -= ui1N;
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }

    //
    // End of critical section
    //

    //
    // Return number of bytes read
    //
    return ui1N;
}


/**************************************************************************//**
* @brief    This function puts up to \e ui16Length bytes into the BSP UART TX
*           buffer and starts to transfer data over UART.
*
*           If \b BSP_UART_ALL_OR_NOTHING is defined, data is put into the
*           TX buffer only if there is room for all \e ui16Length bytes.
*
* @param    pui8Data        is a pointer to the source buffer.
* @param    ui16Length      is the number of bytes to transfer.
*
* @return   Returns the number of bytes actually copied to the TX buffer.
******************************************************************************/
uint16_t
bspUartDataPut(uint8_t *pui8Data, uint16_t ui16Length)
{
    uint16_t ui16BufCount;
    bool bIntDisabled, bDoTrigger;
    register uint16_t ui16Idx = 0;
    uint16_t ui16FifoCount = 0;

    //
    // Get number of bytes available in buffer
    //
    ui16BufCount = bspUartTxSpaceAvail();

    //
    // Return early if no bytes are to be put, or no space is available
    //
    if(!ui16Length || !ui16BufCount)
    {
        return 0;
    }

#ifdef BSP_UART_ALL_OR_NOTHING
    if(ui16Length > ui16BufCount)
    {
        return 0;
    }
#endif

    //
    // Critical section start
    //
    bIntDisabled = IntMasterDisable();

    //
    // If buffer is empty, we may have to send a trigger byte to fifo
    //
    bDoTrigger = (BU_IS_BUF_EMPTY(&sBuBufTx)) ? 1 : 0;

    //
    // Critical section end
    //
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }

    if(bDoTrigger)
    {
        //
        // TX buffer is initially empty, transferring data directly to UART FIFO
        // Copy directly to FIFO.
        //
        while(UARTSpaceAvail(BSP_UART_BASE) && (ui16Idx < ui16Length))
        {
            UARTCharPut(BSP_UART_BASE, pui8Data[ui16Idx++]);
        }

        //
        // Store number of bytes transferred directly to TX FIFO
        //
        ui16FifoCount = ui16Idx;

        //
        // Bytes sent to fifo, no need for a trigger byte
        //
        if(ui16Idx)
        {
            bDoTrigger = 0;
        }
    }

    //
    // Limit transfer size to buffer limit
    //
    if(ui16Length > (ui16BufCount + ui16FifoCount))
    {
        ui16Length = ui16BufCount + ui16FifoCount;
    }

    //
    // Copy data to buffer
    //
    while(ui16Idx < ui16Length)
    {
        *sBuBufTx.pui8Head++ = pui8Data[ui16Idx++];

        //
        // Manage wrap-around
        //
        if(sBuBufTx.pui8Head == sBuBufTx.pui8End)
        {
            sBuBufTx.pui8Head = sBuBufTx.pui8Start;
        }
    }

    //
    // Bytes were stored in buffer
    //
    if(ui16Length > ui16FifoCount)
    {
        //
        // Critical section start. Update byte count.
        //
        bIntDisabled = IntMasterDisable();
        sBuBufTx.ui16NumBytes += (ui16Length - ui16FifoCount);

        //
        // Send byte to UART fifo (to trigger TX interrupt)
        //
        if(bDoTrigger)
        {
            buBufPopByte();
        }

        //
        // Critical section end
        //
        if(!bIntDisabled)
        {
            IntMasterEnable();
        }
    }

    //
    // Return number of bytes actually transferred
    //
    return ui16Length;
}


/**************************************************************************//**
* @brief    This function returns the number of bytes available in the BSP UART
*           TX buffer.
*
* @return   Returns the free space in bytes of the BSP UART TX buffer.
******************************************************************************/
uint16_t
bspUartTxSpaceAvail(void)
{
    uint16_t ui16Space;
    bool bIntDisabled = IntMasterDisable();
    ui16Space = BU_GET_FREE_SPACE(&sBuBufTx);
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }
    return ui16Space;
}


/**************************************************************************//**
* @brief    This function flushes the BSP UART RX buffer by resetting the
*           buffer control structure.
*
* @return   None
******************************************************************************/
void bspUartFlushRx(void)
{
    //
    // Update buffer pointers
    //
    buBufFlush(&sBuBufRx);
}


/**************************************************************************//**
* @brief    This function flushes the BSP UART TX buffer by resetting the
*           buffer control structure.
*
* @return   None
******************************************************************************/
void
bspUartFlushTx(void)
{
    //
    // Update buffer pointers
    //
    buBufFlush(&sBuBufTx);
}


/**************************************************************************//**
* @brief    This function returns the number of data bytes available in the BSP
*           UART RX buffer.
*
* @return   Returns the number data bytes available in the BSP UART RX buffer.
******************************************************************************/
uint16_t
bspUartRxCharsAvail(void)
{
    uint_fast16_t ui16Space;

    //
    // Critical section. Read volatile variable from RX control structure.
    //
    bool bIntDisabled = IntMasterDisable();
    ui16Space = BU_GET_USED_SPACE(&sBuBufRx);

    //
    // Critical section end.
    //
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }

    return ui16Space;
}


/**************************************************************************//**
* @brief    This function returns the current BSP UART baud rate in baud. The
*           function returns 0 if the BSP UART module is not configured.
*
* @return   Returns the current BSP UART baud rate in baud.
******************************************************************************/
uint32_t
bspUartBaudRateGet(void)
{
    return ui32BuBaudRate;
}


/******************************************************************************
* LOCAL FUNCTIONS
*/
/**************************************************************************//**
* @brief    This function pops a byte from the BSP UART TX buffer to the UART
*           TX FIFO. The function handles TX buffer tail wrap-around, but does
*           not handle buffer underrun. The function modifies volatile
*           variables and should only be called when interrupts are disabled.
*
* @return   None
******************************************************************************/
static void
buBufPopByte(void)
{
    //
    // Send byte to TX FIFO (wait for space to become available)
    //
    UARTCharPut(BSP_UART_BASE, (*sBuBufTx.pui8Tail++));

    //
    // Update byte count
    //
    sBuBufTx.ui16NumBytes--;

    //
    // Manage ringbuffer wrap-around
    //
    if(sBuBufTx.pui8Tail == sBuBufTx.pui8End)
    {
        sBuBufTx.pui8Tail = sBuBufTx.pui8Start;
    }
}


#ifdef BSP_UART_ISR
/**************************************************************************//**
* @brief    This function pushes a byte from the UART RX FIFO to the BSP UART
*           RX buffer. The function handles RX buffer wrap-around, but does not
*           handle RX buffer overflow. The function should only be called if
*           there is data in the UART RX FIFO. It modifies volatile variables
*           and should only be called when interrupts are disabled.
*
* @brief    None
******************************************************************************/
static void
buBufPushByte(void)
{
    //
    // Push byte from RX FIFO to buffer
    //
    *sBuBufRx.pui8Head++ = UARTCharGetNonBlocking(BSP_UART_BASE);

    //
    // Update byte count
    //
    sBuBufRx.ui16NumBytes++;

    //
    // Manage wrap-around
    //
    if(sBuBufRx.pui8Head == sBuBufRx.pui8End)
    {
        sBuBufRx.pui8Head = sBuBufRx.pui8Start;
    }
}
#endif // BSP_UART_ISR


/**************************************************************************//**
* @brief    This function flushes the ringbuffer control structure specified
*           by \e psBuf.
*
* @param    psBuf       is a pointer to a \e tBuBuf ringbuffer structure.
*
* @return   None
******************************************************************************/
static void
buBufFlush(tBuBuf *psBuf)
{
    //
    // Start of critical section
    //
    bool bIntDisabled = IntMasterDisable();

    psBuf->pui8Head = psBuf->pui8Start;
    psBuf->pui8Tail = psBuf->pui8Start;
    psBuf->ui16NumBytes = 0;

    //
    // Return to previous interrupt state
    //
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }
}


#ifdef BSP_UART_ISR
/**************************************************************************//**
* @brief    This function handles BSP UART interrupts. This function clears all
*           handled interrupt flags.
*
* @return   None
******************************************************************************/
void
bspUartIsrHandler(void)
{
    //
    // Get status of enabled interrupts
    //
    uint32_t ui32IntBm = UARTIntStatus(BSP_UART_BASE, 1);

    //
    // Clear flags handled by this handler
    //
    UARTIntClear(BSP_UART_BASE, (ui32IntBm & BSP_UART_INT_BM));

    //
    // RX or RX timeout
    //
    if(ui32IntBm & (UART_INT_RX | UART_INT_RT))
    {
        //
        // Put received bytes into buffer
        //
        while(UARTCharsAvail(BSP_UART_BASE) && !BU_IS_BUF_FULL(&sBuBufRx))
        {
            buBufPushByte();
        }
    }

    //
    // TX
    //
    if(ui32IntBm & UART_INT_TX)
    {
        //
        // Fill fifo with bytes from buffer
        //
        while(UARTSpaceAvail(BSP_UART_BASE) && (!BU_IS_BUF_EMPTY(&sBuBufTx)))
        {
            buBufPopByte();
        }
    }
}
#endif // BSP_UART_ISR


/**************************************************************************//**
* Close the Doxygen group.
* @}
******************************************************************************/
#endif // #ifndef BSP_UART_EXCLUDE

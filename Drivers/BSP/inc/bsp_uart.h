//*****************************************************************************
//! @file       bsp_uart.h
//! @brief      UART Board support package header file.
//!
//! Revised     $Date: 2013-04-11 19:57:23 +0200 (Thu, 11 Apr 2013) $
//! Revision    $Revision: 9711 $
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
#ifndef __BSP_UART_H__
#define __BSP_UART_H__


/******************************************************************************
* If building with a C++ compiler, make all of the definitions in this header
* have a C binding.
******************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif


/******************************************************************************
* INCLUDES
*/
#include "bsp.h"


/******************************************************************************
* DEFINES
*/

// Baud rates


// UART return codes
#define BSP_UART_SUCCESS        0x00
#define BSP_UART_UNCONFIGURED   0x01
#define BSP_UART_NOT_SUPPORTED  0x02
#define BSP_UART_BAUDRATE_ERROR 0x04
#define BSP_UART_ERROR          0x08


/******************************************************************************
* TYPEDEFS
*/
//
//! UART baud rates.
//
typedef enum
{
    eBaudRate9600 = 0,
    eBaudRate38400,
    eBaudRate57600,
    eBaudRate115200,
    eBaudRate230400,
    eBaudRate460800
} tUartBaudrate;


/******************************************************************************
* FUNCTION PROTOTYPES
*/
extern uint8_t bspUartBufInit(uint8_t *pui8TxBuf, uint16_t ui1TxByteAlloc,
                              uint8_t *pui8RxBuf, uint16_t ui1RxByteAlloc);
extern uint8_t bspUartOpen(uint32_t ui32BaudRate);
extern uint8_t bspUartClose(void);
extern uint32_t bspUartBaudRateGet(void);
extern uint16_t bspUartDataGet(uint8_t *pui8Data, uint16_t ui1Length);
extern uint16_t bspUartDataPut(uint8_t *pui8Data, uint16_t ui1Length);
extern uint16_t bspUartTxSpaceAvail(void);
extern uint16_t bspUartRxCharsAvail(void);
extern void bspUartFlushTx(void);
extern void bspUartFlushRx(void);

// Functions only implemented if bsp.lib is built without BSP_UART_ALLOCATE_ISR
extern void bspUartIsrHandler(void);


/******************************************************************************
* Mark the end of the C bindings section for C++ compilers.
******************************************************************************/
#ifdef __cplusplus
}
#endif
#endif /* #ifndef __BSP_UART_H__ */

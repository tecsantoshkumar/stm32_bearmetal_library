/*****************************************************************************
 * Copyright (c) 2020-2021, WiSig Networks Pvt Ltd. All rights reserved.     *
 * www.wisig.com                                                             *
 *                                                                           *
 * All information contained herein is property of WiSig Networks Pvt Ltd.   *
 * unless otherwise explicitly mentioned.                                    *
 *                                                                           *
 * The intellectual and technical concepts in this file are proprietary      *
 * to WiSig Networks and may be covered by granted or in process national    *
 * and international patents and are protect by trade secrets and            *
 * copyright law.                                                            *
 *                                                                           *
 * Redistribution and use in source and binary forms of the content in       *
 * this file, with or without modification are not permitted unless          *
 * permission is explicitly granted by WiSig Networks.                       *
 * If WiSig Networks permits this source code to be used as a part of        *
 * open source project, the terms and conditions of CC-By-ND (No Derivative) *
 * license (https://creativecommons.org/licenses/by-nd/4.0/) shall apply.    *
 *****************************************************************************/
 
#ifndef __UART_H__
#define __UART_H__

#include "Driver_USART.h"
#include "wisig.h"
#include "wn5gNrPsDataTypes.h"
#include "wn5gNrPsErrTypes.h"

#define WISIG_UART	  WISIG_UART1
//#define WISIG_UART	  WISIG_UART0
#define DEFAULT_UART_BAUDRATE 115200
//#define DEFAULT_UART_BAUDRATE 230400

#define UART0		0x00
#define UART1		0x01
/**
 * UART state definitions 
 */
#define wnUSART_ERROR_NOT_INITIALIZED 	(wnDRIVER_ERROR_SPECIFIC - 9)
#define wnUART_INITIALIZED  			(1ul << 0)

#ifndef ARG_UNUSED
#define ARG_UNUSED(arg)  (wnVoid)arg
#endif

//wnDRIVER_VERSION     version;
/**
 * Driver version 
 */
#define wnUSART_DRV_VERSION  wnDRIVER_VERSION_MAJOR_MINOR(1, 0)

typedef struct __UARTx_Resources{
	wnUInt32 base;                    /* UART base address             */
	wnUInt32 state;                   /* Indicates if the uart driver is initialized and enabled */
    wnUInt32 system_clk;              /* System clock                 */
    wnUInt32 baudrate;                /* Baudrate                     */	

    wnUInt32 tx_nbr_bytes;            /* Number of bytes transfered   */
    wnUInt32 rx_nbr_bytes;            /* Number of bytes recevied     */
    wnUSART_SignalEvent_t cb_event; /* Callback function for events */
} UARTx_Resources;

extern wnDRIVER_USART WISIG_UART0;
extern wnDRIVER_USART WISIG_UART1;

#endif /* __UART_H__ */

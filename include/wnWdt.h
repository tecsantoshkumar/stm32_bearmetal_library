/*****************************************************************************
 * Copyright (c) 2016-2018, WiSig Networks Pvt Ltd. All rights reserved.     *
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

/**
 * @file wnWdtDriver.h
 * @author Sneha Eingoli
 *
 * @brief Typedef'ed datatypes and Watchdog timer(WDT) Driver Api's
 *
 * 
 */

#ifndef _WN_WDT_DRIVER_H
#define _WN_WDT_DRIVER_H

#include "wisig.h"
#include "wn5gNrPsDataTypes.h"
#include "wn5gNrPsErrTypes.h"

#define WDT_UNLOCK_VALUE            0x1ACCE551
#define WDT_DISABLE_VALUE           0x00

#define WDT_INTCLR                  0xFFFF
#define WDT_TESTMODE_INT_VALUE      0x02

#define WDT_SUCCESS                 WN_SUCCESS
#define WDT_FAILURE                 WN_FAILURE

typedef enum
{
    WDT_TYPE_NO_ACTION = 0,
    WDT_TYPE_INTR_ENABLE,
    WDT_TYPE_RESET_ENABLE
}WDT_TYPES;

/*wnWdtLock - Lock the Watchdog timer */
wnVoid wnWdtLock(wnWatchdogT *WDT);

/*wnWdtUnlock - Unlock the Watchdog */
wnVoid wnWdtUnlock(wnWatchdogT *WDT);

/*wnWdtInit - Initialise the Watchdog*/
wnVoid wnWdtInit(wnWatchdogT *WDT, wnUInt32 CounterValue, WDT_TYPES type);

/*wnWdtCounterValue - Show the current value of watchdog counter*/
wnInt32 wnWdtGetCounterValue(wnWatchdogT *WDT);

/*wnWdtLoadCounter - Load the counter */
wnVoid wnWdtLoadCounter(wnWatchdogT *WDT, wnUInt32 CounterValue);

/*wnWdtClearInterrupt -Clear the Watchdog Interrupt Status*/
wnVoid wnWdtClearInterrupt(wnWatchdogT *WDT);

/*wnWdtEnableIntegrationTestmode - Enable Watchdog in Test Integration Mode*/
wnUInt32 wnWdtEnableIntegrationTestmode(wnWatchdogT *WDT);

/*wnWdtIntegrationOutputSet - Enable RESET and Interrupt in TEST Integration Mode*/
wnVoid wnWdtIntegrationOutputSet(wnWatchdogT *WDT);

#endif


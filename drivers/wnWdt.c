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
 * @file wnWdtDriver.c
 * @author Sneha Eingoli
 * @brief Typedef'ed datatypes and Watchdog timer(WDT) Driver Api's
*/

#include "wnWdt.h"
#include "wn5gNrPsErrTypes.h"
#include "wn5gNrPsDataTypes.h"
#include "stddef.h"


/* wnWdtLock - Lock watchdog access 
WDT - Pointer to structure
*/
wnVoid wnWdtLock(wnWatchdogT *WDT)
{
    WDT->LOCK = 0x0;
}


/*
wnWdtUnlock - Unlock the Watchdog 
WDT - Pointer to structure
Return - Void
*/
wnVoid wnWdtUnlock(wnWatchdogT *WDT)
{
    WDT->LOCK = WDT_UNLOCK_VALUE;
}


/*
wnWdtInit - Initialise the Watchdog Timer
WDT - Pointer to structure
CounterValue - Value to be loaded to timer
type - Sets the action to be done by Watchdog
*/
wnVoid wnWdtInit(wnWatchdogT *WDT, wnUInt32 CounterValue, WDT_TYPES type)
{
    /*Unlocking watchdog */
    wnWdtUnlock(WDT);
    WDT->CTRL = WDT->CTRL & 0x0;
    WDT->LOAD = CounterValue;
    if(type == 0x0)
    {
        WDT->CTRL = 0x0;/*Set to no action */
    }
    else if(type == 0x01) 
    {
        WDT->CTRL = WDT_CTRL_INTEN_Msk; /*Set to NMI generation */
    } 
    else  
    {
        WDT->CTRL = WDT_CTRL_RESEN_Msk | WDT_CTRL_INTEN_Msk;/*Set to reset generation */
    }
    
    wnWdtLock(WDT);/* Locking watchdog */
} 



/* 
wnWdtGetCounterValue- Show the current value of watchdog Timer
Return : Current value in Timer
*/ 
wnInt32 wnWdtGetCounterValue(wnWatchdogT *WDT)
{
    if(WDT != WN_NULL)
        return WDT->VALUE;
    else
        return WN_FAILURE;  
}


/*
 wnWdtLoadCounter - Load the counter 
 WDT - Pointer to structure
 CounterValue - Value to be loaded to timer
 Return -   Void
*/
wnVoid wnWdtLoadCounter(wnWatchdogT *WDT, wnUInt32 CounterValue)
{
    wnWdtUnlock(WDT);
    WDT->LOAD = CounterValue;
    wnWdtLock(WDT);
}


/*
 wnWdtClearInterrupt -Clear the Watchdog Interrupt 
 WDT - Pointer to structure
 Return - WATCHDOG_SUCCESS    - On Success 
          WATCHDOG_FAILURE    - On Failure
*/
wnVoid wnWdtClearInterrupt(wnWatchdogT *WDT) 
{	
    /*we need to call the unlock function because
     *after calling wnWdtInit(), the registers are locked*/
    wnWdtUnlock(WDT);
//    WDT->INTCLR = WDT_INTCLR;
    WDT->INTCLR = 0xFFFFFFFF;

    wnWdtLock(WDT);
}

/*
 wnWdtEnableIntegrationTestmode  - Enable Watchdog in Test Integration Mode
 WDT - Pointer to structure
 Return - WATCHDOG_SUCCESS - On Success
          WATCHDOG_FAILURE - On Failure
*/
wnUInt32 wnWdtEnableIntegrationTestmode(wnWatchdogT *WDT)
{
    WDT->ITCR = WDT_INTEGTESTEN_Msk;
    if(WDT->ITCR == WDT_INTEGTESTEN_Msk)	
	    return WDT_SUCCESS;
    else
	    return WDT_FAILURE;
}


/* 
wnWdtIntegrationOutputSet - Enable Interrupt in TEST Integration Mode
WDT - Pointer to structure
Return - Void
*/
wnVoid wnWdtIntegrationOutputSet(wnWatchdogT *WDT)
{
    WDT->ITOP = WDT_TESTMODE_INT_VALUE;
}




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
 
#include <stdio.h>
#include "wnTimer.h"

/*************** Single timer API***************************/
/*------------------------------------------------------------------
Func    wnTimerGetBaseAdrs()
@Desc   This function gets the baseaddress of selected timer.
@param  timer(0 / 1 / 2)
@return
on success returns Base address
on Failure returns WN_RETNULL

---------------------------------------------------------------*/
wnTimerT *wnTimerGetBaseAdrs (wnUInt32 timer)
{
	if (timer == 0)
		return WISIG_TIMER0;
	else if (timer == 1)
		return WISIG_TIMER1;
	else if (timer == 2)
		return WISIG_TIMER2;
	else 
		return WN_RETNULL;
}

/*-------------------------------------------------------------------
Func	wnTimerInitIntrnlClk()
@Desc	This function is used to initialise the timer(internal clock)
@param	timer(0 / 1 / 2)
	irq_en:interrupt enable
@return
on success returns WN_SUCCESS
on Failure returns  WN_FAILURE

---------------------------------------------------------------*/

wnInt32 wnTimerInitIntrnlClk (wnUInt32 timer, wnUInt32 irq_en)
{
	wnTimerT *WISIG_timer;
	WISIG_timer = wnTimerGetBaseAdrs (timer);
	if (WISIG_timer != WN_RETNULL){
		WISIG_timer->CTRL = 0;	
		if (irq_en!=0)                                                                          
			wnTimerEnableIRQ (timer);
		return WN_SUCCESS;
	}
	else
		return WN_FAILURE;
}

/*------------------------------------------------------------------
Func    wnTimerStatusCheck()
@Desc   This function is used to check the counter free or not.
@param	timer(0 / 1 / 2)
@return
on success returns WN_SUCCESS
on Failure returns  WN_FAILURE

---------------------------------------------------------------*/
wnInt32 wnTimerStatusCheck (wnUInt32 timer)
{
	wnTimerT *WISIG_timer;
	WISIG_timer = wnTimerGetBaseAdrs (timer);
	if (WISIG_timer != WN_RETNULL){
		if (!(WISIG_timer->CTRL & WISIG_TIMER_CTRL_EN_Msk))
			return WN_SUCCESS;
	}
	else
		return WN_FAILURE;
}

/*------------------------------------------------------------------
Func   	wnTimerInitExtrnlClk()
@Desc   This function is used to initialise the timer(External clock)
@param	timer(0 / 1 / 2)
		irq_en:interrupt enable
@return
on success returns WN_SUCCESS
on Failure returns  WN_FAILURE

---------------------------------------------------------------*/
wnInt32 wnTimerInitExtrnlClk (wnUInt32 timer, wnUInt32 irq_en)
{
	wnTimerT *WISIG_timer;
	WISIG_timer = wnTimerGetBaseAdrs (timer);
	if (WISIG_timer != WN_RETNULL){
		WISIG_timer->CTRL = 0;	
		WISIG_timer->CTRL |= WISIG_TIMER_CTRL_SELEXTEN_Msk;
		if (irq_en!=0)                                                                          
			wnTimerEnableIRQ (timer);
		return WN_SUCCESS;
	}
	else
		return WN_FAILURE;

}

/*------------------------------------------------------------------
Func    wnTimerStartExtrnlTimer()
@Desc   This function is used to start the external timer
@param	timer(0 / 1 / 2)
		reload : Reload value
@return
on success returns WN_SUCCESS
on Failure returns  WN_FAILURE

---------------------------------------------------------------*/
wnInt32 wnTimerStartExtrnlTimer (wnUInt32 timer, wnUInt32 reload)
{
	wnTimerT *WISIG_timer;
	WISIG_timer = wnTimerGetBaseAdrs (timer);
	if (WISIG_timer != WN_RETNULL){
		wnTimerSetReload (timer, reload);
		WISIG_timer->CTRL |= WISIG_TIMER_CTRL_SELEXTCLK_Msk;
		return WN_SUCCESS;
	}
	else
		return WN_FAILURE;
}

/*------------------------------------------------------------------
Func   wnTimerEnableIRQ()
@Desc   This function is to enable interrupt
@param	timer(0 / 1 / 2)
@return
on success returns  WN_SUCCESSS
on Failure returns  WN_FAILURE

---------------------------------------------------------------*/
wnInt32 wnTimerEnableIRQ (wnUInt32 timer)
{
	wnTimerT *WISIG_timer;
	WISIG_timer = wnTimerGetBaseAdrs (timer);
	if (WISIG_timer != WN_RETNULL){
		WISIG_timer->CTRL |= WISIG_TIMER_CTRL_IRQEN_Msk;
		return WN_SUCCESS;
	}
	else
		return WN_FAILURE;
}

/*------------------------------------------------------------------
Func    wnTimerDisableIRQ()
@Desc   This function is to Disable the interrupt
@param	timer(0 / 1 / 2)
@return
on success returns  WN_SUCCESSS
on Failure returns  WN_FAILURE

---------------------------------------------------------------*/
wnInt32 wnTimerDisableIRQ (wnUInt32 timer)
{
	wnTimerT *WISIG_timer;
	WISIG_timer = wnTimerGetBaseAdrs (timer);
	if (WISIG_timer != WN_RETNULL){
		WISIG_timer->CTRL &= ~WISIG_TIMER_CTRL_IRQEN_Msk;
		return WN_SUCCESS;
	}
	else
		return WN_FAILURE;
	
}

/*------------------------------------------------------------------
Func   	wnTimerStart()
@Desc   This function is to start the timer.
@param	timer(0 / 1 / 2)
		reload : Reload value
@return
on success returns WN_SUCCESS
on Failure returns WN_FAILURE

---------------------------------------------------------------*/
wnInt32 wnTimerStart (wnUInt32 timer, wnUInt32 reload)
{
	wnTimerT *WISIG_timer;
	WISIG_timer = wnTimerGetBaseAdrs (timer);
	if (WISIG_timer != WN_RETNULL){
		wnTimerSetReload (timer, reload);
		WISIG_timer->CTRL |= WISIG_TIMER_CTRL_EN_Msk;	
		return WN_SUCCESS;
	}
	else
		return WN_FAILURE;
}

/*------------------------------------------------------------------
Func   	wnTimerStop()
@Desc   This function is to stop the timer
@param	timer(0/1/2)
@return
on success returns  WN_SUCCESSS
on Failure returns  WN_FAILURE

---------------------------------------------------------------*/
wnInt32 wnTimerStop (wnUInt32 timer)
{
	wnTimerT *WISIG_timer;
	WISIG_timer = wnTimerGetBaseAdrs (timer);
	if (WISIG_timer != WN_RETNULL){
		WISIG_timer->CTRL &= ~WISIG_TIMER_CTRL_EN_Msk;
		return WN_SUCCESS;
	}
	else
		return WN_FAILURE;
	
}

/*------------------------------------------------------------------
Func   	wnTimerGetValue()
@Desc   This function is to get the current counter value
@param	timer(0/1/2)
@return
on success returns current count value
on Failure returns  WN_FAILURE

---------------------------------------------------------------*/
wnInt32 wnTimerGetValue (wnUInt32 timer)
{
	wnTimerT *WISIG_timer;
	WISIG_timer = wnTimerGetBaseAdrs (timer);
	if (WISIG_timer != WN_RETNULL)
		return WISIG_timer->VALUE;
	else
		return WN_FAILURE;	
}

/*------------------------------------------------------------------
Func   	wnTimerGetReload()
@Desc   This function is to get the reloaded value 
@param	timer(0/1/2)
@return
on success returns reload value
on Failure returns  WN_FAILURE

---------------------------------------------------------------*/
wnInt32 wnTimerGetReload (wnUInt32 timer)
{
	wnTimerT *WISIG_timer;
	WISIG_timer = wnTimerGetBaseAdrs (timer);
	if (WISIG_timer != WN_RETNULL)
		return WISIG_timer->RELOAD;
	else
		return WN_FAILURE;	
}

/*------------------------------------------------------------------
Func    wnTimerSetReload()
@Desc   This function writes the reload value
@param	timer(0/1/2),
		relaod : Reload value
@return
on success returns  WN_SUCCESSS
on Failure returns  WN_FAILURE

---------------------------------------------------------------*/
wnInt32 wnTimerSetReload (wnUInt32 timer, wnUInt32 reload)
{
	wnTimerT *WISIG_timer;
	WISIG_timer = wnTimerGetBaseAdrs (timer);
	if (WISIG_timer != WN_RETNULL){
		WISIG_timer->RELOAD = reload;
		return WN_SUCCESS;
	}
	else
		return WN_FAILURE;
}

/*------------------------------------------------------------------
Func   	wnTimerClearIRQ()
@Desc   This function clears the Interrupt.
@param	timer(0/1/2)
@return
on success returns  WN_SUCCESSS
on Failure returns  WN_FAILURE

---------------------------------------------------------------*/
wnInt32 wnTimerClearIRQ (wnUInt32 timer)
{
	wnTimerT *WISIG_timer;
	WISIG_timer = wnTimerGetBaseAdrs (timer);
	if (WISIG_timer != WN_RETNULL){
		WISIG_timer->INTCLEAR = WISIG_TIMER_INTCLEAR_Msk;
		return WN_SUCCESS;
	}
	else
		return WN_FAILURE;	
}

/*------------------------------------------------------------------
Func    wnTimerStatusIRQ()
@Desc   This function gets the status of the interrupt.
@param	timer(0 / 1 / 2)
@return
on success returns  WN_SUCCESSS
on Failure returns  WN_FAILURE

---------------------------------------------------------------*/
wnInt32 wnTimerStatusIRQ (wnUInt32 timer)
{
	wnTimerT *WISIG_timer;
	WISIG_timer = wnTimerGetBaseAdrs (timer);
	if (WISIG_timer != WN_RETNULL)
		return WISIG_timer->INTSTATUS;
	else
		return WN_FAILURE;	
}


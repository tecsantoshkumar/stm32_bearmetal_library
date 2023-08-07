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

#ifndef _WISIG_TIMER_H_
#define _WISIG_TIMER_H_

#include "wisig.h"
#include "wn5gNrPsDataTypes.h"
#include "wn5gNrPsErrTypes.h"

#define IRQ_ENABLE				0x01
#define IRQ_DISABLE 			0x00
#define INTERNAL_CLK  			0x01
#define EXTERNAL_CLK  			0x02
#define ISR_ON					0x01
#define ISR_OFF					0x00

#define  wnTimerT CMSDK_TIMER_TypeDef 

/* WISIG_TIMER Peripheral address Declarations */

#define WISIG_TIMER0	CMSDK_TIMER0
#define WISIG_TIMER1	CMSDK_TIMER1
#define WISIG_TIMER2	CMSDK_TIMER2

/* WISIG_TIMER  Register Definitions */

#define WISIG_TIMER_CTRL_IRQEN_Pos          3                                              
#define WISIG_TIMER_CTRL_IRQEN_Msk          (0x01ul << WISIG_TIMER_CTRL_IRQEN_Pos)         

#define WISIG_TIMER_CTRL_SELEXTCLK_Pos      2                                              
#define WISIG_TIMER_CTRL_SELEXTCLK_Msk      (0x01ul << WISIG_TIMER_CTRL_SELEXTCLK_Pos)
    
#define WISIG_TIMER_CTRL_SELEXTEN_Pos       1                                             
#define WISIG_TIMER_CTRL_SELEXTEN_Msk       (0x01ul << WISIG_TIMER_CTRL_SELEXTEN_Pos)     

#define WISIG_TIMER_CTRL_EN_Pos             0                                              
#define WISIG_TIMER_CTRL_EN_Msk             (0x01ul << WISIG_TIMER_CTRL_EN_Pos)            

#define WISIG_TIMER_VAL_CURRENT_Pos         0                                             
#define WISIG_TIMER_VAL_CURRENT_Msk         (0xFFFFFFFFul << WISIG_TIMER_VAL_CURRENT_Pos)  

#define WISIG_TIMER_RELOAD_VAL_Pos          0                                             
#define WISIG_TIMER_RELOAD_VAL_Msk          (0xFFFFFFFFul << WISIG_TIMER_RELOAD_VAL_Pos)   

#define WISIG_TIMER_INTSTATUS_Pos           0                                              
#define WISIG_TIMER_INTSTATUS_Msk           (0x01ul << WISIG_TIMER_INTSTATUS_Pos)          

#define WISIG_TIMER_INTCLEAR_Pos            0                                              
#define WISIG_TIMER_INTCLEAR_Msk            (0x01ul << WISIG_TIMER_INTCLEAR_Pos)           

/* WISIG_TIMER Test application functions */

/* @breif Test the specified Timer with internal clock */
wnInt32 wnInternalClkTimer (wnUInt32 timer);

/* @breif Test the specified Timer with External clock */
wnInt32 wnExternalClkTimer (wnUInt32 timer);

/* WISIG_TIMER Driver APIs */

/* @brief checks the WISIG_TIMER status whether busy or free */
wnInt32 wnTimerStatusCheck (wnUInt32 timer);

/* @brief Initialise the WISIG_TIMER with Internal clock */
wnInt32 wnTimerInitIntrnlClk (wnUInt32 timer, wnUInt32 irq_en);

/* @brief Initialise the WISIG_TIMER with external clock */
wnInt32 wnTimerInitExtrnlClk (wnUInt32 timer, wnUInt32 irq_en);

/* @brief start the WISIG_TIMER with external clock  */
wnInt32 wnTimerStartExtrnlTimer (wnUInt32 timer, wnUInt32 reload);

/* @brief Enable the WISIG_TIMER Interrupt requests. */
wnInt32 wnTimerEnableIRQ (wnUInt32 timer);

/* @brief Disable the WISIG_TIMER Interrupt requests. */
wnInt32 wnTimerDisableIRQ (wnUInt32 timer);

/* @brief Starts the WISIG_TIMER */
wnInt32 wnTimerStart (wnUInt32 timer, wnUInt32 reload);

/* @brief  Stops the WISIG_TIMER  */
wnInt32 wnTimerStop (wnUInt32 timer);

/* @brief Returns the current count value */
wnInt32 wnTimerGetValue (wnUInt32 timer);

/* @brief Return the reload value  */
wnInt32 wnTimerGetReload (wnUInt32 timer);

/* @brief Set the WISIG_TIMER load value */
wnInt32 wnTimerSetReload (wnUInt32 timer, wnUInt32 value);

/* @brief WISIG_TIMER Interrupt clear */
wnInt32 wnTimerClearIRQ (wnUInt32 timer);

/* @brief Returns  the WISIG_TIMER IRQ status */
wnInt32 wnTimerStatusIRQ (wnUInt32 timer);

/* @brief Returns the WISIG_TIMER Baseaddress */
wnTimerT *wnTimerGetBaseAdrs (wnUInt32 timer);

#endif

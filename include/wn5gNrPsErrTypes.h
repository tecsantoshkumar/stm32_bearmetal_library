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
 * @file wn5gNrPsErrTypes.h
 * @author Naresh Vattikuti, Anurag Asokan
 * @brief Error numbers and related.
 *
 * @see http://git.wisig.com/root/5gNrBsPs
 */

#ifndef __WN_5G_NR_PS_ERR_TYPES_H__
#define __WN_5G_NR_PS_ERR_TYPES_H__

#include "wn5gNrPsDataTypes.h"
/*
 * TODO: Error numbers are always indicated using 32-bit integers
 */


/* Success */
#define WN_SUCCESS               0

/*
 * Generic error code to indicate failure.
 * TODO: A mapped error number shall be used.
 */
#define WN_FAILURE              -1      /* 0xFFFFFFFF */
#define WN_RETNULL              WN_NULL /* NULL return */
#define WN_RETINVD              1       /* Invalid value */

#endif /* __WN_5G_NR_PS_ERR_TYPES_H__ */

/*  EOF  */

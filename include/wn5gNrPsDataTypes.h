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
 * @file wn5gNrPsDataTypes.h
 * @author Naresh Vattikuti, Anurag Asokan
 *
 * @brief Typedef'ed datatypes and certain macro's.
 *
 * @see
 */


#ifndef __WN_5G_NR_PS_DATA_TYPES_H__
#define __WN_5G_NR_PS_DATA_TYPES_H__

#include <stdbool.h>
#include <stdint.h>

#define WN_FALSE                0
#define WN_TRUE                 1

#define WN_NULL                 NULL

typedef void                    wnVoid;
typedef char                    wnChar;
typedef unsigned char           wnUChar;
typedef short                   wnInt16;
typedef unsigned short          wnUInt16;
typedef int                     wnInt32;
typedef unsigned int            wnUInt32;
typedef long int                wnLong;
typedef long long int           wnInt64;
typedef unsigned long long int  wnUInt64;
typedef float                   wnFloat32;
typedef double                  wnDouble64;
typedef long double             wnLDouble128;
typedef bool                    wnBool;
typedef void                    wnVoid;
typedef int                     wnSocket_t;

typedef uint8_t                 wnUInt8;
typedef int8_t                  wnInt8;

#endif /** __WN_5G_NR_PS_DATA_TYPES_H__ */

/** EOF */

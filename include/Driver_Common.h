/*
 * Copyright (c) 2013-2017 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * $Date:        2. Feb 2017
 * $Revision:    V2.0
 *
 * Project:      Common Driver definitions
 */

/* History:
 *  Version 2.0
 *    Changed prefix wnDRV -> wnDRIVER
 *    Added General return codes definitions
 *  Version 1.10
 *    Namespace prefix wn added
 *  Version 1.00
 *    Initial release
 */

#ifndef DRIVER_COMMON_H_
#define DRIVER_COMMON_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define wnDRIVER_VERSION_MAJOR_MINOR(major,minor) (((major) << 8) | (minor))

/**
\brief Driver Version
*/
typedef struct _wnDRIVER_VERSION {
  uint16_t api;                         ///< API version
  uint16_t drv;                         ///< Driver version
} wnDRIVER_VERSION;

/* General return codes */
#define wnDRIVER_OK                 0 ///< Operation succeeded 
#define wnDRIVER_ERROR             -1 ///< Unspecified error
#define wnDRIVER_ERROR_BUSY        -2 ///< Driver is busy
#define wnDRIVER_ERROR_TIMEOUT     -3 ///< Timeout occurred
#define wnDRIVER_ERROR_UNSUPPORTED -4 ///< Operation not supported
#define wnDRIVER_ERROR_PARAMETER   -5 ///< Parameter error
#define wnDRIVER_ERROR_SPECIFIC    -6 ///< Start of driver specific errors 

/**
\brief General power states
*/ 
typedef enum _wnPOWER_STATE {
  wnPOWER_OFF,                        ///< Power off: no operation possible
  wnPOWER_LOW,                        ///< Low Power mode: retain state, detect and signal wake-up events
  wnPOWER_FULL                        ///< Power on: full operation at maximum performance
} wnPOWER_STATE;

#endif /* DRIVER_COMMON_H_ */

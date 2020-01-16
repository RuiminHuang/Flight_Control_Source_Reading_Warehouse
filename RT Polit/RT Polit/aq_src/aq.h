/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright ?2011-2014  Bill Nesbitt
*/

// NOTE: These parameters must be passed to GCC:
//
//  -fsingle-precision-constant
//

#ifndef _aq_h
#define _aq_h

#include <rtthread.h>


#define USE_MAVLINK
//#define USE_PRES_ALT                    // uncomment to use pressure altitude instead of GPS 
//#define HAS_QUATOS                    // build including Quatos library
//#define HAS_AQ_TELEMETRY              // uncomment to include AQ native binary telemetry and command interface
//#define DIMU_VERSION		11      // uncomment to build for AQ6 hardware with DIMU add-on
//#define HAS_TELEM_SMARTPORT             // enable FrSky SmartPort telemetry option
//#define CAN_CALIB	                // transmit IMU data over CAN


#define BOARD_VERSION       9
#define BOARD_REVISION      0

#ifndef BOARD_VERSION
    #define BOARD_VERSION	6
#endif
#ifndef BOARD_REVISION
    #define BOARD_REVISION	0
#endif

#include "board.h"

#ifndef BOARD_HEADER_FILE
    #if BOARD_VERSION == 6
	#if BOARD_REVISION == 0
	    #define BOARD_HEADER_FILE "board_6_r0.h"
	#elif BOARD_REVISION == 1
	    #define BOARD_HEADER_FILE "board_6_r1.h"
	#endif

    #elif BOARD_VERSION == 7
	#if BOARD_REVISION == 0
	    #define BOARD_HEADER_FILE "board_7_0.h"
	#endif

    #elif BOARD_VERSION == 8
	#if BOARD_REVISION == 1 || BOARD_REVISION == 2
	    #define BOARD_HEADER_FILE "board_m4.h"
	#elif BOARD_REVISION == 3
	    #define BOARD_HEADER_FILE "board_m4_r3.h"
	#elif BOARD_REVISION == 4 || BOARD_REVISION == 5
	    #define BOARD_HEADER_FILE "board_m4_r5.h"
	#elif BOARD_REVISION == 6
	    #define BOARD_HEADER_FILE "board_m4_r6.h"
	#endif

    #elif BOARD_VERSION == 9
	
	    #if BOARD_REVISION == 0
	        #define BOARD_HEADER_FILE "board_m7_r0.h"
	    #endif
		
    #endif	
#endif


#include BOARD_HEADER_FILE

#ifndef M_PI
#define M_PI			3.14159265f
#define M_2PI	        (2.0*M_PI)
#define M_PI_2			(M_PI / 2.0f)
#endif

#define RAD_TO_DEG		(180.0f / M_PI)
#define DEG_TO_RAD		(M_PI / 180.0f)

#define GRAVITY			9.80665f	// m/s^2

#define AQ_US_PER_SEC		1000000

#ifndef __float32_nan
 // __float32_nan is not defined with gcc
 #define __float32_nan 0.0f
 #endif

 #ifndef __CROSSWORKS_ARM
 // add missing defs for gcc
 //#define __sqrtf sqrtf
#endif

#ifndef NAN
#define NAN	__float32_nan
#endif

#ifdef bool
#undef bool
#endif

#ifndef true
#define true 1
#define false 0
#endif

typedef uint8_t bool;

enum dataTypesEnum {
    AQ_TYPE_DBL = 0,
    AQ_TYPE_FLT,
    AQ_TYPE_U32,
    AQ_TYPE_S32,
    AQ_TYPE_U16,
    AQ_TYPE_S16,
    AQ_TYPE_U8,
    AQ_TYPE_S8
};

extern volatile unsigned long counter;
extern volatile unsigned long minCycles;

extern rt_uint32_t rt_hw_tick_get_microsecond(void);


#define CC_ALIGNED  __attribute__ ((aligned (8))) 

#endif

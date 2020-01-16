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

    Copyright © 2011-2014  Bill Nesbitt

    Change Logs:
    Date           Author          Notes
    2018-06-22     jiezhi320       for imx-rt1052
*/

#ifndef _digital_imu_h
#define _digital_imu_h

#include "board.h"
#include "fsl_common.h" 
#include "fsl_iomuxc.h" 
#include "fsl_gpio.h" 
#include "fsl_pit.h"

#include "aq.h"
#include "config.h"
#include <rtthread.h>

#ifdef DIMU_HAVE_MPU6000
    #include "mpu6000.h"
#endif

#ifdef DIMU_HAVE_MAX21100
    #include "max21100.h"
#endif 

#ifdef  DIMU_HAVE_EEPROM
    #include "eeprom.h"
#endif

#ifdef DIMU_HAVE_HMC5983
    #include "hmc5983.h"
#endif

#ifdef DIMU_HAVE_MS5611
#include "ms5611.h"
#endif

#include "spl06.h"
#include "icm20602.h"

#define DIMU_STACK_SIZE	    248     // must be evenly divisible by 8
#define DIMU_PRIORITY	      11

#define DIMU_OUTER_PERIOD   5000    // [us] (200 Hz)
#define DIMU_INNER_PERIOD   2500    // [us] (400 Hz)
#define DIMU_OUTER_DT	    ((float)DIMU_OUTER_PERIOD / 1e6f)
#define DIMU_INNER_DT	    ((float)DIMU_INNER_PERIOD / 1e6f)
#define DIMU_OUTER_HZ	    (1e6f/((float)DIMU_OUTER_PERIOD))
#define DIMU_INNER_HZ	    (1e6f/((float)DIMU_INNER_PERIOD))	
#define DIMU_TEMP_TAU	    5.0f

#define DIMU_TIM	      PIT
#define DIMU_IRQ_CH	    PIT_IRQn
#define DIMU_ISR	      PIT_IRQHandler

//#ifndef __rev16
//    #define __rev16 __REV16
//#endif


typedef struct {
    float temp;
    float dTemp, dTemp2, dTemp3;
    volatile uint32_t lastUpdate;
    //int alarm1Parameter;
    uint16_t nextPeriod;
    uint8_t calibReadWriteFlag;		// 0=no request, 1=read request, 2=write request
    bool sensorsEnabled;

    rt_event_t flag;

} dImuStruct_t __attribute__((aligned));

extern dImuStruct_t dImuData;

extern void dIMUInit(void);
extern void dIMUTare(void);
extern void dIMUSetSensorsEnabled(bool enable);
extern void dIMURequestCalibWrite(void);
extern void dIMURequestCalibRead(void);

#endif

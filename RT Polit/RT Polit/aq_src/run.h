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
*/

#ifndef _run_h
#define _run_h

#include "aq.h"


#define RUN_TASK_SIZE		280     // must be evenly divisible by 8
#define RUN_PRIORITY		10//30

#define RUN_SENSOR_HIST		10				// number of timesteps to average observation sensors' data

#define ALTITUDE                 (*runData.altPos)
#define VELOCITYD                (*runData.altVel)

typedef struct {
    float *altPos;
    float *altVel;
    float bestHacc;
    float accMask;
    float accHist[3][RUN_SENSOR_HIST];
    float magHist[3][RUN_SENSOR_HIST];
    float presHist[RUN_SENSOR_HIST];
    float sumAcc[3];
    float sumMag[3];
    float sumPres;
#ifdef _ALT_MULTI
    float alt_a;
    float alt_b;
#endif
    int sensorHistIndex;

    //OS_TID runTask;
    struct rt_semaphore  runFlag;

    bool ukfInitFlag;
} CC_ALIGNED runStruct_t; 
//__attribute__((aligned));

extern runStruct_t runData;

extern void runInit(void);
extern void runInitHistory(void);
extern void runInitUkf(void);

#endif

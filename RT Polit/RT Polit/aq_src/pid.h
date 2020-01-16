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
    Copyright 2016 Maxim Paperno
*/

#ifndef pid_h
#define pid_h

#include "aq.h"

typedef struct {
    float setPoint;		// Last setpoint
    float dState;		// Last position input
    float iState;		// Integrator state
    float p;		// proportional gain
    float i;		// integral gain
    float d;		// derivative gain
    float f;		// low pass filter factor (1 - pole) for derivative gain
    float pMax, iMax, dMax, oMax, vMax;
    float pv_1, pv_2;
    float co_1;
    float pTerm_1;
    float iTerm_1;
    float dTerm_1;
    float sp_1;
	
	float error_last,error_sum, error_delta;//fuzzypidº∆À„ π”√
} pidStruct_t;

extern pidStruct_t *pidInit(int pParam, int iParam, int dParam, int fParam, int pMaxParam, int iMaxParam, int dMaxParam, int oMaxParam);
extern float pidUpdate(pidStruct_t *pid, float setpoint, float position);
//extern float pidUpdateTest(pidStruct_t *pid, float setpoint, float position);
//extern float pidUpdate2(pidStruct_t *pid, float setpoint, float position);
//extern float pidUpdateB(pidStruct_t *pid, float setpoint, float position);
//extern float pidUpdateC(pidStruct_t *pid, float setpoint, float position);
//extern float pidUpdate3(pidStruct_t *pid, float setpoint, float position);
//extern void pidStartDump(void);
//extern void pidStopDump(void);
//extern void pidDump(pidStruct_t *pid);
extern void pidZeroIntegral(pidStruct_t *pid, float pv, float iState);

#endif

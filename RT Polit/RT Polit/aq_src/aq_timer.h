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
    Date           Author        Notes
    2018-06-21     jiezhi320     for imx_rt1052 
*/

#ifndef _timer_h
#define _timer_h

#define TIMER_TIM		  	GPT1	// can only use 32bit timers 
#define TIMER_IRQ_CH		GPT1_IRQn
#define TIMER_ISR			  GPT1_IRQHandler


#define timerMicros()		TIMER_TIM->CNT  //rt_hw_tick_get_microsecond()//
#define timerStart()		timerData.timerStart = timerMicros()
#define timerStop()			(timerMicros() - timerData.timerStart)


typedef struct {
    uint32_t timerStart;
} timerStruct_t;

extern timerStruct_t timerData;


extern void timerInit(void);


#endif

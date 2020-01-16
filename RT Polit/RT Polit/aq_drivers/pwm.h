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

#ifndef _pwm_h
#define _pwm_h

#include <drivers/pin.h>
#include "aq.h"

#define pwmDigitalHi(Port_id)	rt_pin_write(Port_id, PIN_HIGH)
#define pwmDigitalLo(Port_id)	rt_pin_write(Port_id, PIN_LOW)
#define pwmDigitalGet(Port_id)	(rt_pin_read(Port_id))


typedef struct {
    volatile uint32_t *ccr;
    volatile uint32_t *cnt;
    uint32_t period;
    int8_t direction;
    uint16_t pin;
} pwmPortStruct_t;

void pwm_init(void);
void pwm_out(uint16_t pwm1,uint16_t pwm2,uint16_t pwm3,uint16_t pwm4);


#endif

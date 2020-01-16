/*
 * File      : spl06.c
 * This file is part of RT-Polit
 * COPYRIGHT (C) 2018, RT-Polit Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://openlab.rt-thread.com/license/LICENSE.
 *
 * Change Logs:
 * Date           Author          Notes
 * 2018-06-15     jiezhi320       the first version.
 */

#include "digital.h"
#include "util.h"

uint32_t digitalInit(const uint16_t pin, uint8_t initial) 
{
    rt_pin_mode(pin, PIN_MODE_OUTPUT);
    
	if (initial)
	{	
	    rt_pin_write(pin, PIN_LOW);
	}
    else
    {
	    rt_pin_write(pin, PIN_HIGH);	
	}		
    return pin;
}

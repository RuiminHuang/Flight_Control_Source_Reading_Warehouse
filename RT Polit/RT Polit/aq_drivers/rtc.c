/*
 * File      : rtc.c
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

#include "aq.h"
#include "drv_rtc.h" 
#include "rtc.h"
#include "comm.h"
#include <stdio.h>
#include <string.h>
#include <time.h>


int rtcSetDataTime(int year, int month, int day, int hour, int minute, int second) {
    
    struct tm tm_new = {0}; 
	time_t timestamp;
	rt_device_t rtc;

    tm_new.tm_sec  = second; 
    tm_new.tm_min  = minute; 
    tm_new.tm_hour = hour;
    
    tm_new.tm_mday = day; 
    tm_new.tm_mon  = month; 
    tm_new.tm_year = year; 

    timestamp = mktime(&tm_new);
	
	rtc = rt_device_find("rtc");
	
	if (rtc != RT_NULL)
    {		
		rtc->control(rtc, RT_DEVICE_CTRL_RTC_SET_TIME, &timestamp);

		AQ_PRINTF("RTC set: %d-%02d-%02d %02d:%02d:%02d UTC\n", year, month, day, hour, minute, second);

		// return success
		return 1;
	}
	else
	{
	    return 0;	
	}	
}

void rtcSetDateTimeLong(unsigned long dateTime) {
	
	rt_device_t rtc;
	
	rtc = rt_device_find("rtc");
	
	if (rtc != RT_NULL)
    {		
		rtc->control(rtc, RT_DEVICE_CTRL_RTC_SET_TIME, &dateTime);

		AQ_PRINTF("RTC set: %8x\n", dateTime);
	}
}

unsigned long rtcGetDateTime(void) {
	rt_device_t rtc;
	time_t timestamp;
	
	rtc = rt_device_find("rtc");
	
	if (rtc != RT_NULL)
    {		
		rtc->control(rtc, RT_DEVICE_CTRL_RTC_GET_TIME, &timestamp);

		return timestamp;
	}
}

void rtcInit(void)
{

}


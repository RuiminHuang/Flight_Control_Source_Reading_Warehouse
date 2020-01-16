/*
 * File      : rtc.h
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

#ifndef _rtc_h
#define _rtc_h



extern void rtcInit(void);
extern unsigned long rtcGetDateTime(void);
extern int rtcSetDataTime(int year, int month, int day, int hour, int minute, int second);
extern void rtcSetDateTimeLong(unsigned long dateTime);

#endif

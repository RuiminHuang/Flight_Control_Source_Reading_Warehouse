/*
 * File      : main.c
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

#include <stdint.h>
#include <rthw.h>
#include <rtthread.h>

#ifdef RT_USING_DFS
#include <dfs_file.h>
#endif

#ifdef RT_USING_DEVICE
#include <rtdevice.h>
#endif

#include <board.h>
#include <drivers/pin.h>
#include "platform.h"
#include <rtdevice.h>
#include "aq_init.h"

#include "alt_ukf.h"
#include "analog.h"
#include "aq_mavlink.h"
#include "aq_timer.h"
//#include "can.h"
#include "comm.h"
#include "config.h"
#include "control.h"
#include "filer.h"
//#include "gimbal.h"
#include "gps.h"
#include "hilSim.h"
#include "imu.h"
#include "logger.h"
#include "motors.h"
#include "nav_ukf.h"
#include "nav.h"
#include "radio.h"
#include "rtc.h"
#include "run.h"
//#include "sdio.h"
#include "aq_serial.h"
#include "signaling.h"
#include "supervisor.h"
#include "util.h"
#ifdef HAS_AQ_TELEMETRY
#include "command.h"
#include "telemetry.h"
#endif
#ifdef CAN_CALIB
#include "canCalib.h"
#endif
#ifdef HAS_TELEM_SMARTPORT
#include "telem_sPort.h"
#endif
#include <cpuusage.h>
//#include "app_easyflash.h"
//#include "easyflash.h"

volatile unsigned long counter;
volatile unsigned long minCycles = 0xFFFFFFFF;

static void led1_thread_entry(void* parameter);
static int led_init(void);

void dump_clock(void)
{
    rt_kprintf("OSC clock : %d\n",                  CLOCK_GetFreq(kCLOCK_OscClk));
    rt_kprintf("RTC clock : %d\n",                  CLOCK_GetFreq(kCLOCK_RtcClk));
    rt_kprintf("CPU clock: %d\n",                   CLOCK_GetFreq(kCLOCK_CpuClk));
    rt_kprintf("AHB clock : %d\n",                  CLOCK_GetFreq(kCLOCK_AhbClk));
    rt_kprintf("SEMC clock : %d\n",                 CLOCK_GetFreq(kCLOCK_SemcClk));
    rt_kprintf("IPG clock : %d\n",                  CLOCK_GetFreq(kCLOCK_IpgClk));
    rt_kprintf("ARMPLLCLK(PLL1) : %d\n",            CLOCK_GetFreq(kCLOCK_ArmPllClk));
    rt_kprintf("SYSPLLCLK(PLL2/528_PLL) : %d\n",    CLOCK_GetFreq(kCLOCK_SysPllClk));
    rt_kprintf("SYSPLLPDF0CLK : %d\n",              CLOCK_GetFreq(kCLOCK_SysPllPfd0Clk));
    rt_kprintf("SYSPLLPFD1CLK : %d\n",              CLOCK_GetFreq(kCLOCK_SysPllPfd1Clk));
    rt_kprintf("SYSPLLPFD2CLK : %d\n",              CLOCK_GetFreq(kCLOCK_SysPllPfd2Clk));
    rt_kprintf("SYSPLLPFD3CLK : %d\n",              CLOCK_GetFreq(kCLOCK_SysPllPfd3Clk));
    rt_kprintf("USB1PLLCLK(PLL3) : %d\n",           CLOCK_GetFreq(kCLOCK_Usb1PllClk));
    rt_kprintf("USB1PLLPDF0CLK : %d\n",             CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk));
    rt_kprintf("USB1PLLPFD1CLK : %d\n",             CLOCK_GetFreq(kCLOCK_Usb1PllPfd1Clk));
    rt_kprintf("USB1PLLPFD2CLK : %d\n",             CLOCK_GetFreq(kCLOCK_Usb1PllPfd2Clk));
    rt_kprintf("USB1PLLPFD3CLK : %d\n",             CLOCK_GetFreq(kCLOCK_Usb1PllPfd3Clk));
    rt_kprintf("Audio PLLCLK(PLL4) : %d\n",         CLOCK_GetFreq(kCLOCK_AudioPllClk));
    rt_kprintf("Video PLLCLK(PLL5) : %d\n",         CLOCK_GetFreq(kCLOCK_VideoPllClk));
    rt_kprintf("Enet PLLCLK ref_enetpll0 : %d\n",   CLOCK_GetFreq(kCLOCK_EnetPll0Clk));
    rt_kprintf("Enet PLLCLK ref_enetpll1 : %d\n",   CLOCK_GetFreq(kCLOCK_EnetPll1Clk));
    rt_kprintf("Enet PLLCLK ref_enetpll2 : %d\n",   CLOCK_GetFreq(kCLOCK_EnetPll2Clk));
    rt_kprintf("USB2PLLCLK(PLL7) : %d\n",           CLOCK_GetFreq(kCLOCK_Usb2PllClk));
}

void dump_cc_info(void)
{
#if defined(__CC_ARM)
    rt_kprintf("using armcc, version: %d\n", __ARMCC_VERSION);
#elif defined(__ICCARM__)
    rt_kprintf("using iccarm, version: %d\n", __VER__);
#elif defined(__GNUC__)
    rt_kprintf("using gcc, version: %d.%d\n", __GNUC__, __GNUC_MINOR__);
#endif
}

void dump_link_info(void)
{
#if defined(__CC_ARM)

#elif defined(__ICCARM__)

#elif defined(__GNUC__)
#define DUMP_SYMBOL(__SYM)                  \
        extern int __SYM;                       \
        rt_kprintf("%s: %p\n", #__SYM, &__SYM)

    DUMP_SYMBOL(__fsymtab_start);
    DUMP_SYMBOL(__fsymtab_end);
    DUMP_SYMBOL(__vsymtab_start);
    DUMP_SYMBOL(__vsymtab_end);
    DUMP_SYMBOL(__rt_init_start);
    DUMP_SYMBOL(__rt_init_end);

    DUMP_SYMBOL(__exidx_start);
    DUMP_SYMBOL(__exidx_end);

    DUMP_SYMBOL(__etext);

    DUMP_SYMBOL(__data_start__);
    DUMP_SYMBOL(__data_end__);

    DUMP_SYMBOL(__noncachedata_start__);
    DUMP_SYMBOL(__noncachedata_init_end__);

    DUMP_SYMBOL(__noncachedata_end__);

    DUMP_SYMBOL(__bss_start__);
    DUMP_SYMBOL(__bss_end__);

    DUMP_SYMBOL(stack_start);
    DUMP_SYMBOL(stack_end);

    DUMP_SYMBOL(heap_start);
#endif
}

//static char test[90];
static void rtt_user_assert_hook(const char* ex, const char* func, rt_size_t line)
{
	rt_kprintf("rtt_user_assert_hook\r\n");
    motorsOff();
    //sprintf(test,"(%s)  :%s, :%s\n",  func, line,rt_thread_self()->name);
    //rt_kprintf("(%s) assertion failed at function:%s, line number:%d \n", ex_string, func, line);

    while(1);
}

static rt_err_t exception_hook(void *context)
{
	rt_kprintf("exception_hook\r\n");
    motorsOff();
    while(1);
 
    return RT_EOK;
}

int main(void)
{
    rt_uint32_t result;
	rt_uint32_t priority;

	rt_kprintf("build time: %s %s\n", __DATE__, __TIME__);
    rt_kprintf("\r\n");
    rt_kprintf("********************************");
    rt_kprintf("\r\n");
    rt_kprintf("\r\n");
    rt_kprintf("RT Polit Project\r\n");
    rt_kprintf("\r\n");
    rt_kprintf("********************************");
    rt_kprintf("\r\n");
	
    //dump_clock();
    dump_cc_info();
    dump_link_info();

   
    /* set hardware exception hook */
    rt_hw_exception_install(exception_hook);
    /* set RT-Thread assert hook */
    rt_assert_set_hook(rtt_user_assert_hook);
		
    cpu_usage_init();

    rt_platform_init();		

#if defined(RT_USING_DFS) && defined(RT_USING_SDIO)
    result = mmcsd_wait_cd_changed(RT_TICK_PER_SECOND);
    if (result == MMCSD_HOST_PLUGED)
    {
        /* mount sd card fat partition 1 as root directory */
        if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
            rt_kprintf("File System initialized!\n");
        else
            rt_kprintf("File System init failed!\n");
    }
    else
    {
        rt_kprintf("sdcard init fail or timeout: %d!\n", result);
    }
#endif

    led_init();

    rtcInit();	    // have to do this first as it requires our microsecond timer to calibrate
    timerInit();    // now setup the microsecond timer before everything else

    commNoticesInit();  // set up notice queue
//    filerInit();//jiezhi320

    //easyflash_init();
	
//	NorFlashInit();
  configInit();

    //获取comm1 bitrate并保存 用于boot 初始化串口com1
    //set_env_by_name("comm1_bitrate", p[COMM_BAUD1]);

    signalingInit();
#if 1	
    supervisorInit();
    commInit();
#ifdef USE_MAVLINK
    mavlinkInit();
#endif
#ifdef HAS_AQ_TELEMETRY
    telemetryInit();
#endif
#ifdef HAS_TELEM_SMARTPORT
    if (((uint32_t)p[TELEMETRY_RX_CFG] & 0xF) == 1)
        sPortInit();
#endif
    imuInit();
    analogInit();
    navUkfInit();
    altUkfInit();
    radioInit();
    gpsInit();
    navInit();
    //loggerInit();//jiezhi320
    hilSimInit();
    motorsInit();
    controlInit();
  
    runInit();

    info();

    supervisorInitComplete();

    // allow tasks to startup
    yield(10);

    AQ_NOTICE("Initialization complete, READY.\n");

    // startup complete, reduce comm task priority

    priority = COMM_PRIORITY;
    rt_thread_control(&commTask,RT_THREAD_CTRL_CHANGE_PRIORITY,&priority);//CoSetPriority(commData.commTask, COMM_PRIORITY);

#ifdef HAS_AQ_TELEMETRY
    // start telemetry
    telemetryEnable();
#endif

    // reset idle loop calibration now that everything is running
    minCycles = 999999999;
#endif
    while (1)
    {
        rt_thread_delay(RT_TICK_PER_SECOND);
    }
}



int led_init(void)
{
    rt_thread_t led1_thread = RT_NULL;
    /*
       * 开发板硬件初始化，RTT系统初始化已经在main函数之前完成，
       * 即在component.c文件中的rtthread_startup()函数中完成了。
       * 所以在main函数中，只需要创建线程和启动线程即可。
       */

    led1_thread = rt_thread_create("led1",                     /* 线程名字，字符串形式 */
                                   led1_thread_entry,          /* 线程入口函数 */
                                   RT_NULL,                    /* 线程入口函数参数 */
                                   512,     /* 线程栈大小，单位为字节 */
                                   20,      /* 线程优先级，数值越大，优先级越小 */
                                   5);     /* 线程时间片 */

    if (led1_thread != RT_NULL)
    {
        rt_thread_startup(led1_thread);
        return 0;
    }
    else
        return -1;
}


/*
*************************************************************************
*                             线程定义
*************************************************************************
*/

static void led1_thread_entry(void* parameter)
{

    rt_device_t pin = rt_device_find("pin");


    rt_pin_mode(67, PIN_MODE_OUTPUT);

    while (1)
    {
        rt_pin_write(67, PIN_HIGH);
        rt_thread_delay(500);   /* 延时，单位为tick */
        rt_pin_write(67, PIN_LOW);
        rt_thread_delay(500);
    }
}




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

#include "gps.h"

#include "aq_mavlink.h"
#include "aq_timer.h"
#include "comm.h"
#include "config.h"
#include "digital.h"
//#include "ext_irq.h"
#include "filer.h"
#include "imu.h"
#include "nav.h"
#include "aq_serial.h"
#include "signaling.h"
#include "telemetry.h"
#include "util.h"


#include <string.h>

gpsStruct_t gpsData;
gpsTask_t gpsTaskData;

rt_uint32_t *gpsTaskStack;

struct rt_thread gpsTask;

#ifdef GPS_LOG_BUF
char gpsLog[GPS_LOG_BUF];
#endif

void gpsSendSetup(void) {
    ubloxSendSetup();
}

void gpsCheckBaud(serialPort_t *s) {
    if ((IMU_LASTUPD - gpsData.lastMessage) > 5000000)//10000000 
    {
	if (!gpsTaskData.baudCycle[++gpsTaskData.baudSlot])
	    gpsTaskData.baudSlot = 0;
	AQ_NOTICE("GPS: trying new baud rate\n");
	serialChangeBaud(s, gpsTaskData.baudCycle[gpsTaskData.baudSlot]);
	ubloxInitGps();
	serialChangeBaud(s, GPS_BAUD_RATE);
	gpsSendSetup();
	gpsData.lastMessage = IMU_LASTUPD;
    }
}

void gpsTaskCode(void *p) {
    serialPort_t *s = gpsTaskData.gpsPort;
    char c;
#ifdef GPS_LOG_BUF
    int logPointer = 0;
#endif
    unsigned int ret = 0;

    AQ_NOTICE("GPS task task started\n");

    ubloxInit();

    while (1) {
	yield(1);
	gpsCheckBaud(s);
	signalingGpsDataStream();

	while (serialAvailable(s)) {
	    c = serialRead(s);
	    if (gpsTaskData.ubloxEnabled)
		ret = ubloxCharIn(c);
	    else
		ret = 0;

	    // position update
	    if (ret == 1) {
		// notify world of new data
	
		  rt_sem_release(gpsTaskData.gpsPosFlag);	
	    }
	    // velocity update
	    else if (ret == 2) {
		// notify world of new data
	
		  rt_sem_release(gpsTaskData.gpsVelFlag);	
	    }
	    // lost sync
	    else if (ret == 3) {
		gpsCheckBaud(s);
	    }

#ifdef GPS_LOG_BUF
	    gpsLog[logPointer] = c;
	    logPointer = (logPointer + 1) % GPS_LOG_BUF;
#endif
	}

#ifdef GPS_LOG_BUF
	filerSetHead(gpsTaskData.logHandle, logPointer);
#endif


	signalingGpsDataStream();
    }
}

void gpsPassThrough(commRcvrStruct_t *r) {
    while (commAvailable(r))
	serialWrite(gpsTaskData.gpsPort, commReadChar(r));
}

void gpsTpHandler() {
    unsigned long tp = timerMicros();
    unsigned long diff = (tp - gpsTaskData.lastTimepulse);

    if (diff > 950000 && diff < 1050000)
	gpsTaskData.microsPerSecond -= (gpsTaskData.microsPerSecond - (signed long)((tp - gpsTaskData.lastTimepulse)<<11))>>5;
    gpsTaskData.lastTimepulse = tp;
    gpsData.TPtowMS = gpsData.lastReceivedTPtowMS;
}

void gpsInit(void) {
    AQ_NOTICE("GPS init\n");
	
    //TODO 
    memset((void *)&gpsData, 0, sizeof(gpsData));
    memset((void *)&gpsTaskData, 0, sizeof(gpsTaskData));

    gpsTaskData.ubloxEnabled = true;
    gpsTaskData.baudCycle[0] = 9600;
    gpsTaskData.baudCycle[1] = GPS_BAUD_RATE;
    gpsTaskData.baudCycle[2] = 19200;
    gpsTaskData.baudCycle[3] = 38400;
    gpsTaskData.baudCycle[4] = 57600;
    gpsTaskData.baudCycle[5] = 230400;
    gpsTaskData.baudCycle[6] = 0;

    gpsTaskData.baudSlot = 0;

    gpsTaskData.gpsPort = serialOpen(GPS_USART, GPS_BAUD_RATE, 0, 512, 512);

    // manual reset flags
	gpsTaskData.gpsVelFlag = rt_sem_create("gpsv", 0, RT_IPC_FLAG_FIFO);
	gpsTaskData.gpsPosFlag = rt_sem_create("gpsp", 0, RT_IPC_FLAG_FIFO);	

    gpsTaskStack = aqStackInit(GPS_STACK_SIZE, "GPS");


     rt_thread_init(&gpsTask,
                            "radio",
                            gpsTaskCode,
                            RT_NULL,
                            (rt_uint8_t*)gpsTaskStack,
                            GPS_STACK_SIZE*4,
                            GPS_PRIORITY,
                            5);
 
    rt_thread_startup(&gpsTask);
#ifdef GPS_TP_PORT
    extRegisterCallback(GPS_TP_PORT, GPS_TP_PIN, EXTI_Trigger_Rising, 1, GPIO_PuPd_NOPULL, gpsTpHandler);
#endif

    gpsTaskData.microsPerSecond = AQ_US_PER_SEC<<11;

    gpsData.hAcc = gpsData.vAcc = gpsData.sAcc = 100.0f;

#ifdef GPS_LOG_BUF
    gpsTaskData.logHandle = filerGetHandle(GPS_FNAME);
    filerStream(gpsTaskData.logHandle, gpsLog, GPS_LOG_BUF);
#endif

    commRegisterRcvrFunc(COMM_STREAM_TYPE_GPS, gpsPassThrough);
}

void gpsSendPacket(unsigned char len, char *buf) {
    unsigned int i;

    for (i = 0; i < len; i++)
	serialWrite(gpsTaskData.gpsPort, buf[i]);
}

void gpsSetEnabled(bool enable) {
    gpsTaskData.ubloxEnabled = enable;
	int priority = 0;
    // TODO: Better way to disable GPS task?
    if (enable)
	{
		priority=GPS_PRIORITY;
		rt_thread_control(&gpsTask,RT_THREAD_CTRL_CHANGE_PRIORITY,&priority);

	}	

    else
	{
		priority=RT_THREAD_PRIORITY_MAX - 2;
		rt_thread_control(&gpsTask,RT_THREAD_CTRL_CHANGE_PRIORITY,&priority);		

	}	

}

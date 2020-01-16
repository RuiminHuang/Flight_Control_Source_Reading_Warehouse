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

    Change Logs:
    Date           Author          Notes
    2018-06-22     jiezhi320       for imx-rt1052
*/
#include "fsl_common.h" 
#include "fsl_iomuxc.h" 
#include "fsl_gpio.h" 
#include "fsl_pit.h"

#include "board.h"
#include "config.h"
#ifdef HAS_DIGITAL_IMU
#include "aq_init.h"
#include "aq_timer.h"
#include "comm.h"
#include "d_imu.h"
#include "hilSim.h"
#include "imu.h"
#include "nav_ukf.h"
#include "supervisor.h"
#include "util.h"


rt_uint32_t *dIMUTaskStack;
static struct rt_thread task;
dImuStruct_t dImuData;

static void DIMU_Init(void);

const uint16_t dImuCalibParameters[] = {
    IMU_ACC_BIAS_X,
    IMU_ACC_BIAS_Y,
    IMU_ACC_BIAS_Z,
    IMU_ACC_BIAS1_X,
    IMU_ACC_BIAS1_Y,
    IMU_ACC_BIAS1_Z,
    IMU_ACC_BIAS2_X,
    IMU_ACC_BIAS2_Y,
    IMU_ACC_BIAS2_Z,
    IMU_ACC_BIAS3_X,
    IMU_ACC_BIAS3_Y,
    IMU_ACC_BIAS3_Z,
    IMU_ACC_SCAL_X,
    IMU_ACC_SCAL_Y,
    IMU_ACC_SCAL_Z,
    IMU_ACC_SCAL1_X,
    IMU_ACC_SCAL1_Y,
    IMU_ACC_SCAL1_Z,
    IMU_ACC_SCAL2_X,
    IMU_ACC_SCAL2_Y,
    IMU_ACC_SCAL2_Z,
    IMU_ACC_SCAL3_X,
    IMU_ACC_SCAL3_Y,
    IMU_ACC_SCAL3_Z,
    IMU_ACC_ALGN_XY,
    IMU_ACC_ALGN_XZ,
    IMU_ACC_ALGN_YX,
    IMU_ACC_ALGN_YZ,
    IMU_ACC_ALGN_ZX,
    IMU_ACC_ALGN_ZY,
    IMU_GYO_BIAS_X,
    IMU_GYO_BIAS_Y,
    IMU_GYO_BIAS_Z,
    IMU_GYO_BIAS1_X,
    IMU_GYO_BIAS1_Y,
    IMU_GYO_BIAS1_Z,
    IMU_GYO_BIAS2_X,
    IMU_GYO_BIAS2_Y,
    IMU_GYO_BIAS2_Z,
    IMU_GYO_BIAS3_X,
    IMU_GYO_BIAS3_Y,
    IMU_GYO_BIAS3_Z,
    IMU_GYO_SCAL_X,
    IMU_GYO_SCAL_Y,
    IMU_GYO_SCAL_Z,
    IMU_GYO_ALGN_XY,
    IMU_GYO_ALGN_XZ,
    IMU_GYO_ALGN_YX,
    IMU_GYO_ALGN_YZ,
    IMU_GYO_ALGN_ZX,
    IMU_GYO_ALGN_ZY,
    IMU_MAG_BIAS_X,
    IMU_MAG_BIAS_Y,
    IMU_MAG_BIAS_Z,
    IMU_MAG_BIAS1_X,
    IMU_MAG_BIAS1_Y,
    IMU_MAG_BIAS1_Z,
    IMU_MAG_BIAS2_X,
    IMU_MAG_BIAS2_Y,
    IMU_MAG_BIAS2_Z,
    IMU_MAG_BIAS3_X,
    IMU_MAG_BIAS3_Y,
    IMU_MAG_BIAS3_Z,
    IMU_MAG_SCAL_X,
    IMU_MAG_SCAL_Y,
    IMU_MAG_SCAL_Z,
    IMU_MAG_SCAL1_X,
    IMU_MAG_SCAL1_Y,
    IMU_MAG_SCAL1_Z,
    IMU_MAG_SCAL2_X,
    IMU_MAG_SCAL2_Y,
    IMU_MAG_SCAL2_Z,
    IMU_MAG_SCAL3_X,
    IMU_MAG_SCAL3_Y,
    IMU_MAG_SCAL3_Z,
    IMU_MAG_ALGN_XY,
    IMU_MAG_ALGN_XZ,
    IMU_MAG_ALGN_YX,
    IMU_MAG_ALGN_YZ,
    IMU_MAG_ALGN_ZX,
    IMU_MAG_ALGN_ZY
};

void dIMUTare(void) {
    float acc[3], gyo[3];
    uint32_t lastUpdate;
    float samples = 0.5f / DIMU_OUTER_DT; // 0.5 second
    int i;

    // reset all parameters
    for (i = 0; dImuCalibParameters[i] != IMU_MAG_BIAS_X; i++)
	p[dImuCalibParameters[i]] = 0.0f;

    p[IMU_ACC_SCAL_X] = 1.0f;
    p[IMU_ACC_SCAL_Y] = 1.0f;
    p[IMU_ACC_SCAL_Z] = 1.0f;

    p[IMU_GYO_SCAL_X] = 1.0f;
    p[IMU_GYO_SCAL_Y] = 1.0f;
    p[IMU_GYO_SCAL_Z] = 1.0f;

    lastUpdate = IMU_LASTUPD;

    // let new averages settle
    for (i = 0; i < (int)samples; i++) {
	while (lastUpdate == IMU_LASTUPD)
	    ;
	lastUpdate = IMU_LASTUPD;
    }

    for (i = 0; i < 3; i++) {
	acc[i] = 0.0f;
	gyo[i] = 0.0f;
    }

    for (i = 0; i < (int)samples; i++) {
	while (lastUpdate == IMU_LASTUPD)
	    ;
	lastUpdate = IMU_LASTUPD;

	acc[0] += IMU_RAW_ACCX;
	acc[1] += IMU_RAW_ACCY;
	acc[2] += IMU_RAW_ACCZ;

	gyo[0] += IMU_RAW_RATEX;
	gyo[1] += IMU_RAW_RATEY;
	gyo[2] += IMU_RAW_RATEZ;
    }

    p[IMU_ACC_BIAS_X] = -(acc[0] / samples);
    p[IMU_ACC_BIAS_Y] = -(acc[1] / samples);
    p[IMU_ACC_BIAS_Z] = (GRAVITY - (acc[2] / samples));

    p[IMU_GYO_BIAS_X] = -(gyo[0] / samples);
    p[IMU_GYO_BIAS_Y] = -(gyo[1] / samples);
    p[IMU_GYO_BIAS_Z] = -(gyo[2] / samples);

    navUkfResetBias();
    navUkfResetVels();
}

static void dIMUCalcTempDiff(void) {
    float temp = 0.0f;
    int i = 0;

#ifdef DIMU_HAVE_MPU6000
    if (mpu6000Data.enabled) {
	temp += mpu6000Data.temp;
	i++;
    }
#endif
#ifdef DIMU_HAVE_MAX21100
    if (max21100Data.enabled) {
	temp += max21100Data.temp;
	i++;
    }
#endif
#ifdef DIMU_HAVE_MS5611
    if (ms5611Data.enabled) {
	temp += ms5611Data.temp;
	i++;
    }
#endif

    if (!i)
	return;

    dImuData.temp = temp / (float)i;
    dImuData.dTemp = dImuData.temp - IMU_ROOM_TEMP;
    dImuData.dTemp2 = dImuData.dTemp * dImuData.dTemp;
    dImuData.dTemp3 = dImuData.dTemp2 * dImuData.dTemp;
}

static void dIMUReadCalib(void) {
#ifdef DIMU_HAVE_EEPROM
    uint8_t *buf;
    int size;
    int p1 = 0;

    buf = eepromOpenRead();

    if (buf == 0) {
	    AQ_NOTICE("DIMU: cannot read EEPROM parameters!\n");
    }
    else {
	while ((size = eepromRead(DIMU_EEPROM_BLOCK_SIZE)) != 0)
	    p1 = configParseParams((char *)buf, size, p1);

	AQ_NOTICE("DIMU: read calibration parameters from EEPROM\n");
    }
#endif
}

static void dIMUWriteCalib(void) {
#ifdef DIMU_HAVE_EEPROM
    char *lineBuf;
    uint8_t *buf;
    int n;
    int i, j, k;

    if (!(lineBuf = (char *)aqCalloc(128, sizeof(char)))) {
	AQ_NOTICE("DIMU: Error writing to EEPROM, cannot allocate memory.\n");
	return;
    }

    buf = eepromOpenWrite();

    k = 0;
    for (i = 0; i < sizeof(dImuCalibParameters) / sizeof(uint16_t); i++) {
	n = configFormatParam(lineBuf, dImuCalibParameters[i]);

	for (j = 0; j < n; j++) {
	    buf[k++] = lineBuf[j];
	    if (k == DIMU_EEPROM_BLOCK_SIZE) {
		eepromWrite();
		k = 0;
	    }
	}
    }
    if (k != 0)
	eepromWrite();
    if (lineBuf)
	aqFree(lineBuf, 128, sizeof(char));

    AQ_NOTICE("DIMU: wrote calibration parameters to EEPROM\n");

    eepromClose();
#endif
}

void dIMUSetSensorsEnabled(bool enable) {
    dImuData.sensorsEnabled = enable;

#ifdef DIMU_HAVE_MPU6000
    if (enable)
	mpu6000Enable();
    else
	mpu6000Disable();
#endif
#ifdef DIMU_HAVE_MAX21100
    if (enable)
	max21100Enable();
    else
	max21100Disable();
#endif
#ifdef DIMU_HAVE_ICM20602
    if (enable)
	icm20602Enable();
    else
	icm20602Disable();
#endif
		
#ifdef DIMU_HAVE_HMC5983
    if (enable)
	hmc5983Enable();
    else
	hmc5983Disable();
#endif
#ifdef DIMU_HAVE_MS5611
    if (enable)
	ms5611Enable();
    else
	ms5611Disable();
#endif
		
#ifdef DIMU_HAVE_SPL06
    if (enable)
	spl06Enable();
    else
	spl06Disable();
#endif		
}

static void dIMUReadWriteCalib(void) {
    uint8_t rw = dImuData.calibReadWriteFlag;
    dImuData.calibReadWriteFlag = 0;
#ifdef DIMU_HAVE_EEPROM

    dIMUSetSensorsEnabled(0);

    if (rw == 1)
	dIMUReadCalib();
    else if (rw == 2)
	dIMUWriteCalib();

    dIMUSetSensorsEnabled(1);

#endif  //DIMU_HAVE_EEPROM
}

void dIMURequestCalibWrite(void) {
    if (!dImuData.calibReadWriteFlag)
	dImuData.calibReadWriteFlag = 2;
}

void dIMURequestCalibRead(void) {
    if (!dImuData.calibReadWriteFlag)
	dImuData.calibReadWriteFlag = 1;
}

static void dIMUTaskCode(void *unused) {
    uint32_t loops = 0;

    while (1) {
	// wait for work  2.5ms

     rt_event_recv(dImuData.flag, (1 << 0), \
                          RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,\
                          RT_WAITING_FOREVER, 0);        
		
	if (dImuData.calibReadWriteFlag)
	    dIMUReadWriteCalib();

	if (dImuData.sensorsEnabled) {
	    // double rate gyo loop
#ifdef DIMU_HAVE_MPU6000
	    mpu6000DrateDecode();
#endif
#ifdef DIMU_HAVE_MAX21100
	    max21100DrateDecode();
#endif
#ifdef DIMU_HAVE_ICM20602
	    icm20602DrateDecode();
#endif		

	    imuSensorReady(IMU_TYPE_DIMU, IMU_UPDT_DRATE);

	    // full sensor loop
	    if (!(loops % (DIMU_OUTER_PERIOD/DIMU_INNER_PERIOD))) {
#ifdef DIMU_HAVE_MPU6000
		mpu6000Decode();
#endif
    #ifdef DIMU_HAVE_MAX21100
		max21100Decode();
#endif
#ifdef DIMU_HAVE_ICM20602
	    icm20602Decode();
#endif					
#ifdef DIMU_HAVE_HMC5983
		hmc5983Decode();
#endif
#ifdef DIMU_HAVE_MS5611
		ms5611Decode();
#endif
#ifdef DIMU_HAVE_SPL06		
		spl06Decode();
#endif				
		dImuData.lastUpdate = timerMicros();
		imuSensorReady(IMU_TYPE_DIMU, IMU_UPDT_FULL);
		dIMUCalcTempDiff();
	    }
	}
	else if ((supervisorData.state & STATE_SIM_ENABLED)) {
	    hilSimTick(loops);
	}

	loops++;
    }
}

void dIMUInit(void) {
    rt_memset((void *)&dImuData, 0, sizeof(dImuData));

#ifdef DIMU_HAVE_MPU6000
    mpu6000PreInit();
#endif
#ifdef DIMU_HAVE_MAX21100
    max21100PreInit();
#endif
#ifdef DIMU_HAVE_ICM20602
    icm20602PreInit();
#endif
	
#ifdef DIMU_HAVE_EEPROM
    eepromPreInit();
#endif
#ifdef DIMU_HAVE_HMC5983
    hmc5983PreInit();
#endif
#ifdef DIMU_HAVE_MS5611
    ms5611PreInit();
#endif
#ifdef DIMU_HAVE_SPL06
    spl06PreInit();
#endif

#ifdef DIMU_HAVE_MPU6000
    mpu6000Init();
#endif
#ifdef DIMU_HAVE_MAX21100
    max21100Init();
#endif

#ifdef DIMU_HAVE_ICM20602
    icm20602_init("spi3","spi32");
#endif

#ifdef DIMU_HAVE_EEPROM
    eepromInit();
    dIMUReadCalib();
#endif
#ifdef DIMU_HAVE_HMC5983
    if (hmc5983_init("spi3","spi31") == 0)
      AQ_NOTICE("DIMU: MAG sensor init failed!\n");
#endif
#ifdef DIMU_HAVE_MS5611
    if (ms5611Init() == 0)
      AQ_NOTICE("DIMU: PRES sensor init failed!\n");
#endif
		
#ifdef DIMU_HAVE_SPL06
    if (spl06_init("spi3","spi30") == 0)
      AQ_NOTICE("DIMU: PRES sensor init failed!\n");
#endif
		
		

    dImuData.sensorsEnabled = true;

    dIMUTaskStack = aqStackInit(DIMU_STACK_SIZE, "DIMU");

	dImuData.flag = rt_event_create("dflag", RT_IPC_FLAG_FIFO);
	
  
     rt_thread_init(&task,
                            "dimu",
                            dIMUTaskCode,
                            RT_NULL,
                            (rt_uint8_t*)dIMUTaskStack,
                            DIMU_STACK_SIZE*4,
                            DIMU_PRIORITY,
                            5);
 
    rt_thread_startup(&task);
    // setup digital IMU timer
    DIMU_Init();

#ifdef DIMU_HAVE_MPU6000
    mpu6000Enable();
#endif
#ifdef DIMU_HAVE_MAX21100
    max21100Enable();
#endif
#ifdef DIMU_HAVE_HMC5983
    hmc5983Enable();
#endif
#ifdef DIMU_HAVE_MS5611
    ms5611Enable();
#endif

//    // setup IMU timestep alarm
//    dImuData.nextPeriod = DIMU_TIM->CCR2 + DIMU_INNER_PERIOD;
//    DIMU_TIM->CCR2 = dImuData.nextPeriod;
//    DIMU_TIM->DIER |= TIM_IT_CC2;

#ifdef DIMU_HAVE_MPU6000
    mpu6600InitialBias();
#endif
#ifdef DIMU_HAVE_MAX21100
    max21100InitialBias();
#endif
#ifdef DIMU_HAVE_MS5611
    ms5611InitialBias();
#endif
}

//2.5ms定时中断
static void DIMU_Init(void)
{
	uint32_t ldval =75000000/400;//2.5ms
    pit_config_t pit_config;
	
    PIT_GetDefaultConfig(&pit_config);  
    pit_config.enableRunInDebug=true;   
    PIT_Init(DIMU_TIM,&pit_config);          
    
    PIT_SetTimerPeriod(DIMU_TIM,kPIT_Chnl_0,ldval);
    PIT_EnableInterrupts(DIMU_TIM,kPIT_Chnl_0,kPIT_TimerInterruptEnable);
    RT1052_NVIC_SetPriority(DIMU_IRQ_CH,0,0);	//设为最高优先级
  	EnableIRQ(DIMU_IRQ_CH);	                
    PIT_StartTimer(DIMU_TIM,kPIT_Chnl_0);        
}

//PIT中断服务函数
void DIMU_ISR(void)
{
	rt_interrupt_enter();
	
    if((PIT_GetStatusFlags(PIT,kPIT_Chnl_0)&kPIT_TimerFlag)==kPIT_TimerFlag)
    {
       
        PIT_ClearStatusFlags(PIT,kPIT_Chnl_0,kPIT_TimerFlag);
	    rt_event_send(dImuData.flag, (1 << 0));
    }
    __DSB();				//数据同步屏蔽指令
		
	rt_interrupt_leave();
}

#endif  // HAS_DIGITAL_IMU

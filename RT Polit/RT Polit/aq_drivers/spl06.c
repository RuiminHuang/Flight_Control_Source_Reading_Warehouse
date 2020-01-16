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
#include "drv_spi_bus.h"
#include "config.h"
#ifdef HAS_DIGITAL_IMU
#include "spl06.h"
#include "imu.h"
#include "aq_timer.h"
#include <math.h>

#define BARO_DEBUG

#ifdef BARO_DEBUG
#define BARO_TRACE         rt_kprintf
#else
#define BARO_TRACE(...)

#endif

spl06Struct_t spl06Data;

static struct spi_baro_device*  spi_baro_device;

static void baro_lock(struct spi_baro_device * baro_device)
{
    rt_mutex_take(baro_device->lock, RT_WAITING_FOREVER);
}

static void baro_unlock(struct spi_baro_device * baro_device)
{
    rt_mutex_release(baro_device->lock);
}

static void spl06_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t send_buffer[2];

    send_buffer[0] = reg&0x7f;
    send_buffer[1] = value;
    rt_spi_send(spi_baro_device->rt_spi_device, send_buffer, 2);
}


static uint8_t spl06_read_reg(uint8_t reg)
{
    uint8_t rxBuf[2];
    uint8_t txBuf[2];

    txBuf[0] = reg|0x80;

    rt_spi_send_then_recv(spi_baro_device->rt_spi_device, &txBuf, 1, rxBuf, 2);

    return rxBuf[0];
}

static uint8_t spl06_read_buffer(uint8_t reg, uint8_t *buffer, uint16_t len)
{
    uint8_t txBuf[2];
    uint8_t rxBuf[30];

    baro_lock(spi_baro_device);

    txBuf[0] = reg|0x80;

    rt_spi_send_then_recv(spi_baro_device->rt_spi_device, txBuf, 1, rxBuf, len);
    rt_memcpy(buffer, rxBuf, len);

    baro_unlock(spi_baro_device);

    return 0;
}

static void spl06_config_temperature(uint8_t rate,uint8_t oversampling)
{
	switch(oversampling)
	{
		case TMP_PRC_1:
			spl06Data._kT = 524288;
			break;
		case TMP_PRC_2:
			spl06Data._kT = 1572864;
			break;
		case TMP_PRC_4:
			spl06Data._kT = 3670016;
			break;
		case TMP_PRC_8:
			spl06Data._kT = 7864320;
			break;
		case TMP_PRC_16:
			spl06Data._kT = 253952;
			break;
		case TMP_PRC_32:
			spl06Data._kT = 516096;
			break;
		case TMP_PRC_64:
			spl06Data._kT = 1040384;
			break;
		case TMP_PRC_128:
			spl06Data._kT = 2088960;
			break;
	}
	spl06Data._kT = 1.0f/spl06Data._kT;
	baro_lock(spi_baro_device);
	
	spl06_write_reg(SP06_TMP_CFG,rate|oversampling|0x80);	//温度每秒128次测量一次
	if(oversampling > TMP_PRC_8)
	{
		uint8_t temp = spl06_read_reg(SP06_CFG_REG);
		spl06_write_reg(SP06_CFG_REG,temp|SPL06_CFG_T_SHIFT);
	}
	baro_unlock(spi_baro_device);
}

static void spl06_config_pressure(uint8_t rate,uint8_t oversampling)
{
	switch(oversampling)
	{
		case PM_PRC_1:
			spl06Data._kP = 524288;
			break;
		case PM_PRC_2:
			spl06Data._kP = 1572864;
			break;
		case PM_PRC_4:
			spl06Data._kP = 3670016;
			break;
		case PM_PRC_8:
			spl06Data._kP = 7864320;
			break;
		case PM_PRC_16:
			spl06Data._kP = 253952;
			break;
		case PM_PRC_32:
			spl06Data._kP = 516096;
			break;
		case PM_PRC_64:
			spl06Data._kP = 1040384;
			break;
		case PM_PRC_128:
			spl06Data._kP = 2088960;
			break;
	}
	spl06Data._kP = 1.0f/spl06Data._kP;
	
	baro_lock(spi_baro_device);
	spl06_write_reg(SP06_PSR_CFG,rate|oversampling);
	if(oversampling > PM_PRC_8)
	{
		uint8_t temp = spl06_read_reg(SP06_CFG_REG);
		spl06_write_reg(SP06_CFG_REG,temp|SPL06_CFG_P_SHIFT);
	}
	baro_unlock(spi_baro_device);
}

static void spl06_start(uint8_t mode)
{
	baro_lock(spi_baro_device);
	spl06_write_reg(SP06_MEAS_CFG, mode);
	baro_unlock(spi_baro_device);
}

static int32_t spl06_get_pressure(void)
{
	uint8_t buf[3];
	int32_t adc;
	spl06_read_buffer(SP06_PSR_B2,buf,3);
	adc = (int32_t)(buf[0]<<16) + (buf[1]<<8) + buf[2];
	adc = (adc&0x800000)?(0xFF000000|adc):adc;
	return adc;
}

static int32_t spl06_get_temperature(void)
{
	uint8_t buf[3];
	int32_t adc;
	spl06_read_buffer(SP06_TMP_B2,buf,3);
	adc = (int32_t)(buf[0]<<16) + (buf[1]<<8) + buf[2];
	adc = (adc&0x800000)?(0xFF000000|adc):adc;
	return adc;
}

//50hz调用一次
void spl06Decode(void) 
{
	static uint32_t loop=0;
	float Traw_src, Praw_src;
	float qua2, qua3;

	loop++;
	if (loop%4!=0)
		return;
	
    if (spl06Data.enabled) {
	    spl06Data._raw_temp = spl06_get_temperature();
	    spl06Data._raw_press = spl06_get_pressure();
		
	    Traw_src = spl06Data._kT * spl06Data._raw_temp;
	    Praw_src = spl06Data._kP * spl06Data._raw_press;

	    //计算温度
	    spl06Data.temp_orgi = 0.5f*spl06Data._C0 + Traw_src * spl06Data._C1;
        spl06Data.temp = utilFilter(&spl06Data.tempFilter, spl06Data.temp_orgi);
	    
		//计算气压
	    qua2 = spl06Data._C10 + Praw_src * (spl06Data._C20 + Praw_src* spl06Data._C30);
	    qua3 = Traw_src * Praw_src * (spl06Data._C11 + Praw_src * spl06Data._C21);
	    spl06Data.pres = spl06Data._C00 + Praw_src * qua2 + Traw_src * spl06Data._C01 + qua3;			
        spl06Data.lastUpdate = timerMicros();
    }
}

void spl06Enable(void) {
    if (spl06Data.initialized && !spl06Data.enabled) {
        spl06Data.enabled = 1;
        spl06Data.step = 0;
    }
}

void spl06Disable(void) {
    spl06Data.enabled = 0;
}

void spl06PreInit(void) {
    // spl06Data.spi = spiClientInit(DIMU_MS5611_SPI, MS5611_SPI_BAUD, 0, DIMU_MS5611_CS_PORT, DIMU_MS5611_CS_PIN, &ms5611Data.spiFlag, 0);
}

rt_err_t spl06_init(const char *bus_name, const char * spi_device_name)
{
    struct rt_spi_device * rt_spi_device;
    rt_err_t res;
    uint8_t id;
    int i, j;
    uint8_t coef[18];


    spi_baro_device = rt_malloc(sizeof(struct spi_baro_device));

    /* initialize mutex */
    spi_baro_device->lock = rt_mutex_create(spi_device_name, RT_IPC_FLAG_FIFO);
    if (spi_baro_device->lock == RT_NULL)
    {
        rt_kprintf("init lock mutex failed\r\n");
        return -RT_ENOSYS;
    }

    rt_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        BARO_TRACE("spi device %s not found!\r\n", spi_device_name);
        return -RT_ENOSYS;
    }
    spi_baro_device->rt_spi_device = rt_spi_device;

    /* config spi */
    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
    cfg.max_hz = SPL06_SPI_BAUD; /* 2M */
    rt_spi_configure(spi_baro_device->rt_spi_device, &cfg);


    /* init baro*/
    utilFilterInit(&spl06Data.tempFilter, (1.0f / 13.0f), DIMU_TEMP_TAU, IMU_ROOM_TEMP);

    baro_lock(spi_baro_device);

    spl06_write_reg(SP06_RESET,0x89);

    id = spl06_read_reg(SP06_ID);

    baro_unlock(spi_baro_device);
    BARO_TRACE("spl06 id=0x%x\r\n",id);//读取ID

    if(id != 0x10)
    {
        BARO_TRACE("spl06 id error !!!\r\n");
        spl06Data.initialized = 0;
    }
    else
    {
        BARO_TRACE("spl06 pass\r\n",id);

        spl06Data.initialized = 1;

        rt_thread_delay(150);

        spl06_read_buffer(SP06_COEF,coef,18);

        spl06Data._C0 	= ((int16_t)coef[0]<<4 ) + ((coef[1]&0xF0)>>4);
        spl06Data._C0 = (spl06Data._C0&0x0800)?(0xF000|spl06Data._C0):spl06Data._C0;
        spl06Data._C1 	= ((int16_t)(coef[1]&0x0F)<<8 ) + coef[2];
        spl06Data._C1 = (spl06Data._C1&0x0800)?(0xF000|spl06Data._C1):spl06Data._C1;
        spl06Data._C00 = ((int32_t)coef[3]<<12 ) + ((uint32_t)coef[4]<<4 ) + (coef[5]>>4);
        spl06Data._C10 = ((int32_t)(coef[5]&0x0F)<<16 ) + ((uint32_t)coef[6]<<8 ) + coef[7];
        spl06Data._C00 = (spl06Data._C00&0x080000)?(0xFFF00000|spl06Data._C00):spl06Data._C00;
        spl06Data._C10 = (spl06Data._C10&0x080000)?(0xFFF00000|spl06Data._C10):spl06Data._C10;
        spl06Data._C01   	= ((int16_t)coef[8]<<8 ) + coef[9];
        spl06Data._C11   	= ((int16_t)coef[10]<<8 ) + coef[11];
        spl06Data._C11 = (spl06Data._C11&0x0800)?(0xF000|spl06Data._C11):spl06Data._C11;
        spl06Data._C20   	= ((int16_t)coef[12]<<8 ) + coef[13];
        spl06Data._C20 = (spl06Data._C20&0x0800)?(0xF000|spl06Data._C20):spl06Data._C20;
        spl06Data._C21   	= ((int16_t)coef[14]<<8 ) + coef[15];
        spl06Data._C21 = (spl06Data._C21&0x0800)?(0xF000|spl06Data._C21):spl06Data._C21;
        spl06Data._C30   	= ((int16_t)coef[16]<<8 ) + coef[17];
        spl06Data._C30 = (spl06Data._C30&0x0800)?(0xF000|spl06Data._C30):spl06Data._C30;

		BARO_TRACE("_C0=%d\r\n",spl06Data._C0);
		BARO_TRACE("_C1=%d\r\n",spl06Data._C1);
		BARO_TRACE("_C00=%d\r\n",spl06Data._C00);
		BARO_TRACE("_C10=%d\r\n",spl06Data._C10);
		BARO_TRACE("_C01=%d\r\n",spl06Data._C01);
		BARO_TRACE("_C11=%d\r\n",spl06Data._C11);
		BARO_TRACE("_C20=%d\r\n",spl06Data._C20);
		BARO_TRACE("_C21=%d\r\n",spl06Data._C21);
		BARO_TRACE("_C30=%d\r\n",spl06Data._C30);
		
	    spl06_config_pressure(PM_RATE_128,PM_PRC_64);
	    spl06_config_temperature(PM_RATE_8,TMP_PRC_8);
	    spl06_start(MEAS_CTRL_ContinuousPressTemp);	//启动连续的气压温度测量
	    rt_thread_delay(20);
    }

    return spl06Data.initialized;
}

#endif

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
    2018-06-21     jiezhi320     for imx_rt1052 spi bus
*/
#include "drv_spi_bus.h"
#include "config.h"
#ifdef HAS_DIGITAL_IMU
#include "imu.h"
#include "hmc5983.h"
#include "aq_timer.h"
#include "util.h"
#include "config.h"
//#include "ext_irq.h"
#ifndef __CC_ARM
#include <intrinsics.h>
#endif

hmc5983Struct_t hmc5983Data;


#define MAG_DEBUG

#ifdef MAG_DEBUG
#define MAG_TRACE         rt_kprintf
#else
#define MAG_TRACE(...)

#endif

static struct spi_mag_device*  spi_mag_device;


static void mag_lock(struct spi_mag_device * mag_device)
{
    rt_mutex_take(mag_device->lock, RT_WAITING_FOREVER);
}

static void mag_unlock(struct spi_mag_device * mag_device)
{
    rt_mutex_release(mag_device->lock);
}

static uint8_t hmc5983GetReg(uint8_t reg)
{
    uint8_t rxBuf[2];
    uint8_t txBuf[2];

    mag_lock(spi_mag_device);

    txBuf[0] = HMC5983_READ_BIT | reg;

    rt_spi_send_then_recv(spi_mag_device->rt_spi_device, &txBuf, 1, rxBuf, 2);

    mag_unlock(spi_mag_device);

    return rxBuf[0];
}

static void hmc5983SetReg(uint8_t reg, uint8_t val) {
    uint8_t rxBuf[2];
    uint8_t txBuf[2];

    mag_lock(spi_mag_device);

    txBuf[0] = HMC5983_WRITE_BIT | reg;
    txBuf[1] = val;

    rt_spi_send(spi_mag_device->rt_spi_device, txBuf, 2);

    mag_unlock(spi_mag_device);
}

static void hmc5983ReliablySetReg(uint8_t reg, uint8_t val) {
    uint8_t ret;

    mag_lock(spi_mag_device);

    do {
        rt_thread_delay(10);
        hmc5983SetReg(reg, val);
        rt_thread_delay(10);
        ret = hmc5983GetReg(reg);
    } while (ret != val);

    mag_unlock(spi_mag_device);
}



static uint8_t hmc5983_read_buffer(uint8_t reg, uint8_t *buffer, uint16_t len)
{
    uint8_t txBuf[2];
    uint8_t rxBuf[20];

    mag_lock(spi_mag_device);

    txBuf[0] = reg;

    rt_spi_send_then_recv(spi_mag_device->rt_spi_device, txBuf, 1, rxBuf, len);
    rt_memcpy(buffer, rxBuf, len);

    mag_unlock(spi_mag_device);

    return 0;
}


static uint8_t mag_get_data(int16_t *mag)
{
    //int16_t mag[3];
    uint8_t buf[6];

    hmc5983_read_buffer(HMC5983_READ_MULT_BIT | 0x03,buf,6);

    mag[0] = ((int16_t)buf[0]<<8) + buf[1];
    mag[2] = ((int16_t)buf[2]<<8) + buf[3];
    mag[1] = ((int16_t)buf[4]<<8) + buf[5];

    //MAG_TRACE("mag0=%d, mag1=%d, mag2=%d\r\n,",mag[0],mag[1],mag[2]);

    return 0;
}



static void hmc5983ScaleMag(int32_t *in, float *out, float divisor) {
    float scale;

    scale = divisor * (1.0f / 187.88f);

    out[0] = hmc5983Data.magSign[0] * DIMU_ORIENT_MAG_X * scale;
    out[1] = hmc5983Data.magSign[1] * DIMU_ORIENT_MAG_Y * scale;
    out[2] = hmc5983Data.magSign[2] * DIMU_ORIENT_MAG_Z * scale;
}

static void hmc5983CalibMag(float *in, volatile float *out) {
    float a, b, c;
    float x, y, z;

    // bias
    a = +(in[0] + p[IMU_MAG_BIAS_X] + p[IMU_MAG_BIAS1_X]*dImuData.dTemp + p[IMU_MAG_BIAS2_X]*dImuData.dTemp2 + p[IMU_MAG_BIAS3_X]*dImuData.dTemp3);
    b = +(in[1] + p[IMU_MAG_BIAS_Y] + p[IMU_MAG_BIAS1_Y]*dImuData.dTemp + p[IMU_MAG_BIAS2_Y]*dImuData.dTemp2 + p[IMU_MAG_BIAS3_Y]*dImuData.dTemp3);
    c = -(in[2] + p[IMU_MAG_BIAS_Z] + p[IMU_MAG_BIAS1_Z]*dImuData.dTemp + p[IMU_MAG_BIAS2_Z]*dImuData.dTemp2 + p[IMU_MAG_BIAS3_Z]*dImuData.dTemp3);

    // misalignment
    x = a + b*p[IMU_MAG_ALGN_XY] + c*p[IMU_MAG_ALGN_XZ];
    y = a*p[IMU_MAG_ALGN_YX] + b + c*p[IMU_MAG_ALGN_YZ];
    z = a*p[IMU_MAG_ALGN_ZX] + b*p[IMU_MAG_ALGN_ZY] + c;

    // scale
    x /= (p[IMU_MAG_SCAL_X] + p[IMU_MAG_SCAL1_X]*dImuData.dTemp + p[IMU_MAG_SCAL2_X]*dImuData.dTemp2 + p[IMU_MAG_SCAL3_X]*dImuData.dTemp3);
    y /= (p[IMU_MAG_SCAL_Y] + p[IMU_MAG_SCAL1_Y]*dImuData.dTemp + p[IMU_MAG_SCAL2_Y]*dImuData.dTemp2 + p[IMU_MAG_SCAL3_Y]*dImuData.dTemp3);
    z /= (p[IMU_MAG_SCAL_Z] + p[IMU_MAG_SCAL1_Z]*dImuData.dTemp + p[IMU_MAG_SCAL2_Z]*dImuData.dTemp2 + p[IMU_MAG_SCAL3_Z]*dImuData.dTemp3);

    out[0] = x * imuData.cosRot - y * imuData.sinRot;
    out[1] = y * imuData.cosRot + x * imuData.sinRot;
    out[2] = z;
}

void hmc5983Decode(void) {
    int32_t mag[3];
    float divisor;
    int i;

    if (hmc5983Data.enabled) {

        mag_get_data(mag);

        divisor=1;

        hmc5983ScaleMag(mag, hmc5983Data.rawMag, divisor);
        hmc5983CalibMag(hmc5983Data.rawMag, hmc5983Data.mag);

        hmc5983Data.lastUpdate = timerMicros();
    }
}

inline void hmc5983Enable(void) {
    if (hmc5983Data.initialized)
        hmc5983Data.enabled = 1;
}

inline void hmc5983Disable(void) {
    hmc5983Data.enabled = 0;
}

void hmc5983PreInit(void) {
    //hmc5983Data.spi = spiClientInit(DIMU_HMC5983_SPI, HMC5983_SPI_BAUD, 0, DIMU_HMC5983_CS_PORT, DIMU_HMC5983_CS_PIN, &hmc5983Data.spiFlag, 0);
}

rt_err_t hmc5983_init(const char *bus_name, const char * spi_device_name)
{
    struct rt_spi_device * rt_spi_device;
    rt_err_t res;
    uint8_t id;
    int i = HMC5983_RETRIES;

    switch ((int)p[IMU_FLIP]) {
    case 1:
        hmc5983Data.magSign[0] =  1.0f;
        hmc5983Data.magSign[1] = -1.0f;
        hmc5983Data.magSign[2] = -1.0f;
        break;

    case 2:
        hmc5983Data.magSign[0] = -1.0f;
        hmc5983Data.magSign[1] =  1.0f;
        hmc5983Data.magSign[2] = -1.0f;
        break;

    case 0:
    default:
        hmc5983Data.magSign[0] = 1.0f;
        hmc5983Data.magSign[1] = 1.0f;
        hmc5983Data.magSign[2] = 1.0f;
        break;
    }


    spi_mag_device = rt_malloc(sizeof(struct spi_mag_device));

    /* initialize mutex */
    spi_mag_device->lock = rt_mutex_create(spi_device_name, RT_IPC_FLAG_FIFO);
    if (spi_mag_device->lock == RT_NULL)
    {
        MAG_TRACE("init lock mutex failed\r\n");
        return -RT_ENOSYS;
    }

    rt_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        MAG_TRACE("spi device %s not found!\r\n", spi_device_name);
        return -RT_ENOSYS;
    }
    spi_mag_device->rt_spi_device = rt_spi_device;

    /* config spi */
    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
    cfg.max_hz = HMC5983_SPI_BAUD;
    rt_spi_configure(spi_mag_device->rt_spi_device, &cfg);

    /* init mag*/

    // wait for a valid response
    while (--i && hmc5983GetReg(0x0a) != 'H')
    {
        rt_thread_delay(100);
    }
    if (i > 0)
    {
        MAG_TRACE("hmc5983 pass \r\n",id);

        rt_thread_delay(5);

        // 75Hz, 8x oversample
        hmc5983ReliablySetReg(0x00, 0xF8);
        rt_thread_delay(10);

        //    highest gain (+-0.88 Ga)
        //    hmc5983ReliablySetReg(0x01, 0b00000000);
        //    gain (+-2.5 Ga)
        hmc5983ReliablySetReg(0x01, 0x60);
        rt_thread_delay(10);

        hmc5983ReliablySetReg(0x02, 0x00);
        rt_thread_delay(10);

        hmc5983Data.initialized = 1;
    }
    else
    {
        MAG_TRACE("hmc5983 failed \r\n",id);
        hmc5983Data.initialized = 0;
    }
//    while(1)
//		{
//    mag_get_data(mag);
//		rt_thread_delay(500);
//
//		}
    return hmc5983Data.initialized;

}
#endif

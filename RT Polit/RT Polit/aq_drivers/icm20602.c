/*
 * File      : icm20602.c
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
#if defined(HAS_DIGITAL_IMU) && defined(DIMU_HAVE_ICM20602)
#include "imu.h"
#include "icm20602.h"

#include "aq_timer.h"
#include "util.h"
#include "config.h"
//#include "ext_irq.h"
#ifndef __CC_ARM
#include <intrinsics.h>
#endif

#define ICM_DEBUG

#ifdef ICM_DEBUG
#define ICM_TRACE         rt_kprintf
#else
#define ICM_TRACE(...)

#endif

icm20602Struct_t icm20602Data;

static struct spi_acc_gyro_device*  spi_acc_gyro_device;


static void sensor_lock(struct spi_acc_gyro_device * sensor_device)
{
    rt_mutex_take(sensor_device->lock, RT_WAITING_FOREVER);
}

static void sensor_unlock(struct spi_acc_gyro_device * sensor_device)
{
    rt_mutex_release(sensor_device->lock);
}


static void icm20602_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t send_buffer[2];

    send_buffer[0] = reg&0x7f;
    send_buffer[1] = value;
    rt_spi_send(spi_acc_gyro_device->rt_spi_device, send_buffer, 2);
}


static uint8_t icm20602_read_reg(uint8_t reg)
{
    uint8_t rxBuf[2];
    uint8_t txBuf[2];

    txBuf[0] = reg|0x80;

    rt_spi_send_then_recv(spi_acc_gyro_device->rt_spi_device, txBuf, 1, rxBuf, 2);

    return rxBuf[0];
}


static uint8_t icm20602_read_buffer(uint8_t reg, uint8_t *buffer, uint16_t len)
{
    uint8_t txBuf[2];
    uint8_t rxBuf[20];
    sensor_lock(spi_acc_gyro_device);

    txBuf[0] = reg|0x80;

    rt_spi_send_then_recv(spi_acc_gyro_device->rt_spi_device, txBuf, 1, rxBuf, len);
    rt_memcpy(buffer, rxBuf, len);

    sensor_unlock(spi_acc_gyro_device);

    return 0;
}


static uint8_t icm20602_get_accel(int16_t *accel, int16_t *temp)
{
    uint8_t buf[10];
    icm20602_read_buffer(ICM20_ACCEL_XOUT_H,buf,8);

    accel[0] = ((int16_t)buf[0]<<8) + buf[1];
    accel[1] = ((int16_t)buf[2]<<8) + buf[3];
    accel[2] = ((int16_t)buf[4]<<8) + buf[5];
    *temp     = ((int16_t)buf[6]<<8) + buf[7];

    //ICM_TRACE("acc0=%d, acc1=%d, acc2=%d\r\n",accel[0],accel[1],accel[2]);
    return 0;
}


static uint8_t icm20602_get_gyro(int16_t *gyro)
{
    uint8_t buf[8];
    icm20602_read_buffer(ICM20_GYRO_XOUT_H,buf,8);

    gyro[0] = (buf[0]<<8) + buf[1];
    gyro[1] = (buf[2]<<8) + buf[3];
    gyro[2] = (buf[4]<<8) + buf[5];

    // ICM_TRACE("gyro0=%d, gyro1=%d, gyro2=%d\r\n",gyro[0],gyro[1],gyro[2]);

    return 0;
}


void icm20602InitialBias(void) {
    uint32_t lastUpdate = icm20602Data.lastUpdate;
    float tempSum;
    int i;

    tempSum = 0.0f;

    for (i = 0; i < 50; i++) {
        while (lastUpdate == icm20602Data.lastUpdate)
            delay(1);
        lastUpdate = icm20602Data.lastUpdate;

        tempSum += icm20602Data.rawTemp;
    }

    icm20602Data.temp = tempSum / 50.0f;
    utilFilterReset(&icm20602Data.tempFilter, icm20602Data.temp);

    for (i=0; i<3; i++)
    {
        LowPassFilterFloat_init(&icm20602Data.lpf_dRateGyo[i], DIMU_INNER_HZ, 200.0f);
    }

    for (i=0; i<3; i++)
    {
        LowPassFilterFloat_init(&icm20602Data.lpf_acc[i],  DIMU_OUTER_HZ, 50.0f);
        LowPassFilterFloat_init(&icm20602Data.lpf_gyro[i], DIMU_OUTER_HZ, 200.0f);
    }
}

static void icm20602CalibAcc(float *in, volatile float *out) {
    float a, b, c;
    float x, y, z;

    // bias
    a = -(in[0] + p[IMU_ACC_BIAS_X] + p[IMU_ACC_BIAS1_X]*dImuData.dTemp + p[IMU_ACC_BIAS2_X]*dImuData.dTemp2 + p[IMU_ACC_BIAS3_X]*dImuData.dTemp3);
    b = +(in[1] + p[IMU_ACC_BIAS_Y] + p[IMU_ACC_BIAS1_Y]*dImuData.dTemp + p[IMU_ACC_BIAS2_Y]*dImuData.dTemp2 + p[IMU_ACC_BIAS3_X]*dImuData.dTemp3);
    c = -(in[2] + p[IMU_ACC_BIAS_Z] + p[IMU_ACC_BIAS1_Z]*dImuData.dTemp + p[IMU_ACC_BIAS2_Z]*dImuData.dTemp2 + p[IMU_ACC_BIAS3_X]*dImuData.dTemp3);

    // misalignment
    x = a + b*p[IMU_ACC_ALGN_XY] + c*p[IMU_ACC_ALGN_XZ];
    y = a*p[IMU_ACC_ALGN_YX] + b + c*p[IMU_ACC_ALGN_YZ];
    z = a*p[IMU_ACC_ALGN_ZX] + b*p[IMU_ACC_ALGN_ZY] + c;

    // scale
    x /= (p[IMU_ACC_SCAL_X] + p[IMU_ACC_SCAL1_X]*dImuData.dTemp + p[IMU_ACC_SCAL2_X]*dImuData.dTemp2 + p[IMU_ACC_SCAL3_X]*dImuData.dTemp3);
    y /= (p[IMU_ACC_SCAL_Y] + p[IMU_ACC_SCAL1_Y]*dImuData.dTemp + p[IMU_ACC_SCAL2_Y]*dImuData.dTemp2 + p[IMU_ACC_SCAL3_Y]*dImuData.dTemp3);
    z /= (p[IMU_ACC_SCAL_Z] + p[IMU_ACC_SCAL1_Z]*dImuData.dTemp + p[IMU_ACC_SCAL2_Z]*dImuData.dTemp2 + p[IMU_ACC_SCAL3_Z]*dImuData.dTemp3);

    // IMU rotation
    out[0] = x * imuData.cosRot - y * imuData.sinRot;
    out[1] = y * imuData.cosRot + x * imuData.sinRot;
    out[2] = z;
}

static void icm20602ScaleGyo(int32_t *in, float *out, float divisor) {
    float scale;

    scale = 1.0f / ((1<<16) / (MPU6000_GYO_SCALE * 2.0f)) * divisor * DEG_TO_RAD;

    out[0] = icm20602Data.gyoSign[0] * DIMU_ORIENT_GYO_X * scale;
    out[1] = icm20602Data.gyoSign[1] * DIMU_ORIENT_GYO_Y * scale;
    out[2] = icm20602Data.gyoSign[2] * DIMU_ORIENT_GYO_Z * scale;
}

static void icm20602ScaleAcc(int32_t *in, float *out, float divisor) {
    float scale;

    scale = 1.0f / ((1<<16) / (MPU6000_ACC_SCALE * 2.0f)) * divisor * GRAVITY;

    out[0] = icm20602Data.accSign[0] * DIMU_ORIENT_ACC_X * scale;
    out[1] = icm20602Data.accSign[1] * DIMU_ORIENT_ACC_Y * scale;
    out[2] = icm20602Data.accSign[2] * DIMU_ORIENT_ACC_Z * scale;
}

static void icm20602CalibGyo(float *in, volatile float *out) {
    float a, b, c;
    float x, y, z;

    // bias
    a = +(in[0] + icm20602Data.gyoOffset[0] + p[IMU_GYO_BIAS_X] + p[IMU_GYO_BIAS1_X]*dImuData.dTemp + p[IMU_GYO_BIAS2_X]*dImuData.dTemp2 + p[IMU_GYO_BIAS3_X]*dImuData.dTemp3);
    b = -(in[1] + icm20602Data.gyoOffset[1] + p[IMU_GYO_BIAS_Y] + p[IMU_GYO_BIAS1_Y]*dImuData.dTemp + p[IMU_GYO_BIAS2_Y]*dImuData.dTemp2 + p[IMU_GYO_BIAS3_Y]*dImuData.dTemp3);
    c = -(in[2] + icm20602Data.gyoOffset[2] + p[IMU_GYO_BIAS_Z] + p[IMU_GYO_BIAS1_Z]*dImuData.dTemp + p[IMU_GYO_BIAS2_Z]*dImuData.dTemp2 + p[IMU_GYO_BIAS3_Z]*dImuData.dTemp3);

    // misalignment
    x = a + b*p[IMU_GYO_ALGN_XY] + c*p[IMU_GYO_ALGN_XZ];
    y = a*p[IMU_GYO_ALGN_YX] + b + c*p[IMU_GYO_ALGN_YZ];
    z = a*p[IMU_GYO_ALGN_ZX] + b*p[IMU_GYO_ALGN_ZY] + c;

    // scale
    x /= p[IMU_GYO_SCAL_X];
    y /= p[IMU_GYO_SCAL_Y];
    z /= p[IMU_GYO_SCAL_Z];

    // IMU rotation
    out[0] = x * imuData.cosRot - y * imuData.sinRot;
    out[1] = y * imuData.cosRot + x * imuData.sinRot;
    out[2] = z;
}

void icm20602DrateDecode(void) {
    int32_t gyo[3];
    float divisor;
    int s, i;
    float tmp[3];

    if (icm20602Data.enabled)
    {
        icm20602_get_gyro(gyo);
        divisor = 1.0f;

        icm20602ScaleGyo(gyo, icm20602Data.dRateRawGyo, divisor);
        icm20602CalibGyo(icm20602Data.dRateRawGyo, icm20602Data.dRateGyo);
    }
}

void icm20602Decode(void) {
    int32_t acc[3], temp, gyo[3];
    float divisor;
    int i;
    float tmp[3];

    if (icm20602Data.enabled)
    {
        icm20602_get_gyro(gyo);
        icm20602_get_accel(acc, &temp);
        divisor = 1.0f ;

        icm20602Data.rawTemp = temp * divisor * (1.0f / 340.0f) + 36.53f;
        icm20602Data.temp = utilFilter(&icm20602Data.tempFilter, icm20602Data.rawTemp);

        icm20602ScaleAcc(acc, icm20602Data.rawAcc, divisor);
        for (i=0; i<3; i++)
        {
            tmp[i] = LowPassFilterFloat_apply(&icm20602Data.lpf_acc[i], icm20602Data.rawAcc[i]);
        }
        icm20602CalibAcc(icm20602Data.rawAcc, icm20602Data.acc);


        icm20602ScaleGyo(gyo, icm20602Data.rawGyo, divisor);
        icm20602CalibGyo(icm20602Data.rawGyo, icm20602Data.gyo);

        icm20602Data.lastUpdate = timerMicros();
    }
}

void icm20602Enable(void) {
    icm20602Data.enabled = 1;
}

void icm20602Disable(void) {
    icm20602Data.enabled = 0;
}

void icm20602PreInit(void) {
    //icm20602Data.spi = spiClientInit(DIMU_MPU6000_SPI, MPU6000_SPI_REG_BAUD, 0, DIMU_MPU6000_CS_PORT, DIMU_MPU6000_CS_PIN, &icm20602Data.spiFlag, 0);
}

rt_err_t icm20602_init(const char *bus_name, const char * spi_device_name)
{
    struct rt_spi_device * rt_spi_device;
    rt_err_t res;
    uint8_t id;

    switch ((int)p[IMU_FLIP]) {
    case 1:
        icm20602Data.accSign[0] =  1.0f;
        icm20602Data.accSign[1] = -1.0f;
        icm20602Data.accSign[2] = -1.0f;
        icm20602Data.gyoSign[0] =  1.0f;
        icm20602Data.gyoSign[1] = -1.0f;
        icm20602Data.gyoSign[2] = -1.0f;
        break;

    case 2:
        icm20602Data.accSign[0] = -1.0f;
        icm20602Data.accSign[1] =  1.0f;
        icm20602Data.accSign[2] = -1.0f;
        icm20602Data.gyoSign[0] = -1.0f;
        icm20602Data.gyoSign[1] =  1.0f;
        icm20602Data.gyoSign[2] = -1.0f;
        break;

    case 0:
    default:
        icm20602Data.accSign[0] = 1.0f;
        icm20602Data.accSign[1] = 1.0f;
        icm20602Data.accSign[2] = 1.0f;
        icm20602Data.gyoSign[0] = 1.0f;
        icm20602Data.gyoSign[1] = 1.0f;
        icm20602Data.gyoSign[2] = 1.0f;
        break;
    }

    utilFilterInit(&icm20602Data.tempFilter, DIMU_OUTER_DT, DIMU_TEMP_TAU, IMU_ROOM_TEMP);


    spi_acc_gyro_device = rt_malloc(sizeof(struct spi_acc_gyro_device));

    /* initialize mutex */
    spi_acc_gyro_device->lock = rt_mutex_create(spi_device_name, RT_IPC_FLAG_FIFO);
    if (spi_acc_gyro_device->lock == RT_NULL)
    {
        rt_kprintf("init lock mutex failed\r\n");
        return -RT_ENOSYS;
    }

    rt_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        ICM_TRACE("spi device %s not found!\r\n", spi_device_name);
        return -RT_ENOSYS;
    }
    spi_acc_gyro_device->rt_spi_device = rt_spi_device;

    /* config spi */
    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
    cfg.max_hz = MPU6000_SPI_BAUD; /* 1M */
    rt_spi_configure(spi_acc_gyro_device->rt_spi_device, &cfg);

    /* init sensor*/

    sensor_lock(spi_acc_gyro_device);

    icm20602_write_reg(ICM20_PWR_MGMT_1,0x80);	//��λ����λ��λ0x41,˯��ģʽ��
    rt_thread_delay(50);
    icm20602_write_reg(ICM20_PWR_MGMT_1,0x01);		//�ر�˯�ߣ��Զ�ѡ��ʱ��
    rt_thread_delay(50);

    id = icm20602_read_reg(ICM20_WHO_AM_I);//��ȡID
    ICM_TRACE("icm_20602 id=0x%x\r\n",id);

    if(id != 0x12)
    {
        ICM_TRACE("icm_20602 id error !!!\r\n");
        return -RT_ENOSYS;

    }

    ICM_TRACE("icm20602 init pass\r\n");

    icm20602_write_reg(ICM20_PWR_MGMT_2, 0x00);
    rt_thread_delay(10);

    icm20602_write_reg(ICM20_SMPLRT_DIV,0);			//��Ƶ��=Ϊ0+1�������������Ϊ�ڲ���������
    rt_thread_delay(10);
    icm20602_write_reg(ICM20_CONFIG,DLPF_BW_250);	//GYRO��ͨ�˲�����
    rt_thread_delay(10);
    icm20602_write_reg(ICM20_ACCEL_CONFIG2,ACCEL_AVER_4|ACCEL_DLPF_BW_218);	//ACCEL��ͨ�˲�����
    rt_thread_delay(10);

    //��������
    // ACC scale
#if MPU6000_ACC_SCALE == 2
    icm20602_write_reg(ICM20_ACCEL_CONFIG,ICM20_ACCEL_FS_2G);
#endif
#if MPU6000_ACC_SCALE == 4
    icm20602_write_reg(ICM20_ACCEL_CONFIG,ICM20_ACCEL_FS_4G);
#endif
#if MPU6000_ACC_SCALE == 8
    icm20602_write_reg(ICM20_ACCEL_CONFIG,ICM20_ACCEL_FS_8G);
#endif
#if MPU6000_ACC_SCALE == 16
    icm20602_write_reg(ICM20_ACCEL_CONFIG,ICM20_ACCEL_FS_16G);
#endif
    rt_thread_delay(10);
    // GYO scale
#if MPU6000_GYO_SCALE == 250
    icm20602_write_reg(ICM20_GYRO_CONFIG,ICM20_GYRO_FS_250);
#endif
#if MPU6000_GYO_SCALE == 500
    icm20602_write_reg(ICM20_GYRO_CONFIG,ICM20_GYRO_FS_500);
#endif
#if MPU6000_GYO_SCALE == 1000
    icm20602_write_reg(ICM20_GYRO_CONFIG,ICM20_GYRO_FS_1000);
#endif
#if MPU6000_GYO_SCALE == 2000
    icm20602_write_reg(ICM20_GYRO_CONFIG,ICM20_GYRO_FS_2000);
#endif

    rt_thread_delay(10);

    icm20602_write_reg(ICM20_LP_MODE_CFG, 0x00);	//�رյ͹���
    rt_thread_delay(10);
    icm20602_write_reg(ICM20_FIFO_EN, 0x00);		//�ر�FIFO

    sensor_unlock(spi_acc_gyro_device);

    rt_thread_delay(100);

//		while(1)
//		{
//		icm20602_get_accel(accel);
//		icm20602_get_gyro(gyro);
//		rt_thread_delay(500);
//		}

    return RT_EOK;
}
#endif
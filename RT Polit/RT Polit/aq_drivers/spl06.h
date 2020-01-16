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

#ifndef __SPL06_H_
#define __SPL06_H_

#include <rtthread.h>
#include <drivers/spi.h>
#include "util.h"

//��ѹ��������(sample/sec),Background ģʽʹ��
#define  PM_RATE_1			(0<<4)		//1 measurements pr. sec.
#define  PM_RATE_2			(1<<4)		//2 measurements pr. sec.
#define  PM_RATE_4			(2<<4)		//4 measurements pr. sec.			
#define  PM_RATE_8			(3<<4)		//8 measurements pr. sec.
#define  PM_RATE_16			(4<<4)		//16 measurements pr. sec.
#define  PM_RATE_32			(5<<4)		//32 measurements pr. sec.
#define  PM_RATE_64			(6<<4)		//64 measurements pr. sec.
#define  PM_RATE_128		(7<<4)		//128 measurements pr. sec.

//��ѹ�ز�������(times),Background ģʽʹ��
#define PM_PRC_1			0		//Sigle			kP=524288	,3.6ms
#define PM_PRC_2			1		//2 times		kP=1572864	,5.2ms
#define PM_PRC_4			2		//4 times		kP=3670016	,8.4ms
#define PM_PRC_8			3		//8 times		kP=7864320	,14.8ms
#define PM_PRC_16			4		//16 times		kP=253952	,27.6ms
#define PM_PRC_32			5		//32 times		kP=516096	,53.2ms
#define PM_PRC_64			6		//64 times		kP=1040384	,104.4ms
#define PM_PRC_128			7		//128 times		kP=2088960	,206.8ms

//�¶Ȳ�������(sample/sec),Background ģʽʹ��
#define  TMP_RATE_1			(0<<4)		//1 measurements pr. sec.
#define  TMP_RATE_2			(1<<4)		//2 measurements pr. sec.
#define  TMP_RATE_4			(2<<4)		//4 measurements pr. sec.			
#define  TMP_RATE_8			(3<<4)		//8 measurements pr. sec.
#define  TMP_RATE_16		(4<<4)		//16 measurements pr. sec.
#define  TMP_RATE_32		(5<<4)		//32 measurements pr. sec.
#define  TMP_RATE_64		(6<<4)		//64 measurements pr. sec.
#define  TMP_RATE_128		(7<<4)		//128 measurements pr. sec.

//�¶��ز�������(times),Background ģʽʹ��
#define TMP_PRC_1			0		//Sigle
#define TMP_PRC_2			1		//2 times
#define TMP_PRC_4			2		//4 times
#define TMP_PRC_8			3		//8 times
#define TMP_PRC_16			4		//16 times
#define TMP_PRC_32			5		//32 times
#define TMP_PRC_64			6		//64 times
#define TMP_PRC_128			7		//128 times


//SPL06_MEAS_CFG
#define MEAS_COEF_RDY		0x80
#define MEAS_SENSOR_RDY		0x40		//��������ʼ�����
#define MEAS_TMP_RDY		0x20		//���µ��¶�����
#define MEAS_PRS_RDY		0x10		//���µ���ѹ����

#define MEAS_CTRL_Standby				0x00		//����ģʽ
#define MEAS_CTRL_PressMeasure			0x01	//������ѹ����
#define MEAS_CTRL_TempMeasure			0x02	//�����¶Ȳ���
#define MEAS_CTRL_ContinuousPress		0x05	//������ѹ����
#define MEAS_CTRL_ContinuousTemp		0x06	//�����¶Ȳ���
#define MEAS_CTRL_ContinuousPressTemp	0x07	//������ѹ�¶Ȳ���



//FIFO_STS
#define SPL06_FIFO_FULL		0x02
#define SPL06_FIFO_EMPTY	0x01


//INT_STS
#define SPL06_INT_FIFO_FULL		0x04
#define SPL06_INT_TMP			0x02
#define SPL06_INT_PRS			0x01


//CFG_REG
#define SPL06_CFG_T_SHIFT	0x08	//oversampling times>8ʱ����ʹ��
#define SPL06_CFG_P_SHIFT	0x04

#define SP06_PSR_B2		0x00		//��ѹֵ
#define SP06_PSR_B1		0x01
#define SP06_PSR_B0		0x02
#define SP06_TMP_B2		0x03		//�¶�ֵ
#define SP06_TMP_B1		0x04
#define SP06_TMP_B0		0x05

#define SP06_PSR_CFG	0x06		//��ѹ��������
#define SP06_TMP_CFG	0x07		//�¶Ȳ�������
#define SP06_MEAS_CFG	0x08		//����ģʽ����

#define SP06_CFG_REG	0x09
#define SP06_INT_STS	0x0A
#define SP06_FIFO_STS	0x0B

#define SP06_RESET		0x0C
#define SP06_ID			0x0D

#define SP06_COEF		0x10		//-0x21,12���ֽ�
#define SP06_COEF_SRCE	0x28


#define SPL06_ADDRESS	0xEE

struct spi_baro_device
{
    struct rt_spi_device *          rt_spi_device;
    rt_mutex_t                      lock;
};

#define SPL06_SPI_BAUD		    (2*1000*1000)	// 2Mhz



typedef struct {
    utilFilter_t tempFilter;

    int16_t _C0,_C1,_C01,_C11,_C20,_C21,_C30;
    int32_t _C00,_C10;
	
    uint8_t step;
    uint8_t enabled;
//    uint8_t startTempConv;
//    uint8_t startPresConv;
    uint8_t adcRead;
    uint8_t initialized;
	
    float _kT,_kP;	
	int32_t _raw_temp,_raw_press;	
	volatile float temp_orgi;
    volatile float temp;
    volatile float pres;
    volatile uint32_t lastUpdate;
} spl06Struct_t;

extern spl06Struct_t spl06Data;

extern void spl06PreInit(void);
extern rt_err_t spl06_init(const char *bus_name, const char * spi_device_name);
extern void spl06InitialBias(void);
extern void spl06Result(int unused);
extern void spl06Conversion(int unused);
extern void spl06Decode(void);
extern void spl06Enable(void);
extern void spl06Disable(void);
extern void spl06Enable(void);
extern void spl06Disable(void);




#endif // __SPL06_H_

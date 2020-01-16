/*
 * File      : hmc5983.h
 * This file is part of RT-Polit
 * COPYRIGHT (C) 2018, RT-Polit Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://openlab.rt-thread.com/license/LICENSE.
 *
 * Change Logs:
 * Date           Author          Notes
 * 2018-06-08     jiezhi320       the first version.
 */

#ifndef __HMC5983_H_
#define __HMC5983_H_

#include "aq.h"
#include <rtthread.h>
#include <drivers/spi.h>

#define HMC5983_SPI_BAUD	    (1*1000*1000)// 1 Mhz

#define HMC5983_RETRIES       5

#define HMC5983_READ_BIT	    (0x80)
#define HMC5983_READ_MULT_BIT	(0xc0)
#define HMC5983_WRITE_BIT	    (0x00)


struct spi_mag_device
{
    struct rt_spi_device *          rt_spi_device;
    rt_mutex_t                      lock;
};

typedef struct {
   // spiClient_t *spi;
    float rawMag[3];
    float mag[3];
    float magSign[3];
    //volatile uint32_t spiFlag;
    volatile uint32_t lastUpdate;
    //volatile uint8_t rxBuf[HMC5983_SLOT_SIZE*HMC5983_SLOTS];
    //volatile uint8_t slot;
    //uint8_t readCmd;
    bool enabled;
    bool initialized;
} hmc5983Struct_t;

extern hmc5983Struct_t hmc5983Data;

extern rt_err_t hmc5983_init(const char *bus_name, const char * spi_device_name);
extern void hmc5983PreInit(void);
extern void hmc5983Decode(void);
extern void hmc5983Enable(void);
extern void hmc5983Disable(void);



#endif // __HMC5983_H_

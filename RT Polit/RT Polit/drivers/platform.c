/*
 * File      : platform.c
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
 
#include "platform.h"  
#include "drv_spi_bus.h" 


#ifdef RT_USING_SPI
/*
CLK GPIO_AD_B1_15
SDO GPIO_AD_B1_14
SDI GPIO_AD_B0_02

CS0 GPIO_AD_B1_12

CS1 GPIO_AD_B1_05//GPIO_AD_B0_04
CS2 GPIO_AD_B1_06//GPIO_AD_B0_05
CS3 GPIO_AD_B1_07

*/

/*
GPIO_AD_B0_00  FLEXPWM2_PWMA3
GPIO_AD_B0_02  FLEXPWM1_PWMX0
GPIO_AD_B0_03  FLEXPWM1_PWMX1
GPIO_AD_B1_09  FLEXPWM4_PWMA1


*/
static int rt_hw_spi3_init(void)
{
    rt_err_t res;
	
	  // baro spl06
    rt1050_spi_bus_attach_device("spi3", "spi30", 65);
		//mag hmc5983 
    rt1050_spi_bus_attach_device("spi3", "spi31", 66);//71
		
	  // acc+gyro icm20602
    rt1050_spi_bus_attach_device("spi3", "spi32", 64);  
    return res;
}


#endif /* RT_USING_SPI */


void rt_platform_init(void)
{

#ifdef RT_USING_SPI
    rt_hw_spi3_init();
#ifdef RT_USING_DFS
    //w25qxx_init("flash0", "spi10");
#endif /* RT_USING_DFS */
#endif
	
    rt_thread_delay(2);
}


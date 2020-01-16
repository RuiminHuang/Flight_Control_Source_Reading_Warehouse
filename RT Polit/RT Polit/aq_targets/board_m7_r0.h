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

#ifndef _board_h
#define _board_h

//#define _ALT_MULTI 
#define _INVERT_DISARM
   #define _INVERT_DISARM_TIMS 600//1.5s

#define _DISARM_AFTER_3S//3s没起飞自动加锁
   #define DISARM_TIMES  2400//2.5ms

#define _ADD_PRES_OFT
#define _AUTO_TAKEOFF_LAND
   #define AUTO_SWT_TIMES  5
   #define AUTO_SWT_TIME_OVER  80//50ms*20 * 4 

//#define _VSPD_SLOW  
//#define _HSPD_SLOW



#define _DISABLE_MANUAL//可定高起飞
   #define _ONLAND_THRO   530

#ifdef  _DISABLE_MANUAL
    #define _DISARM_ON_LAND  
        #define _DISARM_ON_LAND_CNT   400//2.5ms*400=1.25s
#endif



//#define HAS_USB

enum pwmPorts {
    PWM_1 = 0,
    PWM_2,
    PWM_3,
    PWM_4,
    PWM_5,
    PWM_6,
    PWM_7,
    PWM_8,
    PWM_9,
    PWM_NUM_PORTS
};

#define PPM_PWM_CHANNEL		8	// which PWM channel to use for PPM capture


#define RC1_DELTANG_BAUD	134078
#define RC1_UART		"uart2"
//#define RC2_UART		"uart3"


//#define SERIAL_UART1
#define SERIAL_UART2
#define SERIAL_UART3
//#define SERIAL_UART4
#define SERIAL_UART5
//#define SERIAL_UART6



#define SUPERVISOR_READY_PIN	66
#define SUPERVISOR_DEBUG_PIN	68

//KEVIN

#define SUPERVISOR_USER_LED1_PIN	71
#define SUPERVISOR_USER_LED2_PIN	71

#define GPS_USART		"uart5"
#define GPS_LED_PIN		71


#define ADC_DMA_STREAM		0
#define ADC_DMA_CHANNEL		0
#define ADC_DMA_FLAGS		0
#define ADC_DMA_TC_FLAG		0
#define ADC_DMA_ISR		0
#define ADC_DMA_CR		0
#define ADC_DMA_IRQ		0
#define ADC_DMA_HANDLER		0


//#define CANx			    CAN2
////#define CAN_CLK			    (RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2)
////#define CAN_GPIO_PORT		    GPIOB
////#define CAN_RX_PIN		    GPIO_Pin_12
////#define CAN_TX_PIN		    GPIO_Pin_13
////#define CAN_AF_PORT		    GPIO_AF_CAN2
////#define CAN_RX_SOURCE		    GPIO_PinSource12
////#define CAN_TX_SOURCE		    GPIO_PinSource13
////#define CAN_RX0_IRQ		    CAN2_RX0_IRQn
////#define CAN_RX0_HANDLER		    CAN2_RX0_IRQHandler
////#define CAN_TX_IRQ		    CAN2_TX_IRQn
////#define CAN_TX_HANDLER		    CAN2_TX_IRQHandler


#define HAS_DIGITAL_IMU
#define USE_DIGITAL_IMU

//#define DIMU_HAVE_EEPROM
//#define DIMU_HAVE_MPU6000
#define DIMU_HAVE_HMC5983
//#define DIMU_HAVE_MS5611
#define DIMU_HAVE_ICM20602
#define DIMU_HAVE_SPL06

#define MPU6000_ACC_SCALE           8//16     // g      (2, 4, 8, 16)
#define MPU6000_GYO_SCALE           2000  // deg/s  (250, 500, 1000, 2000)

#define ICM20602_ACC_SCALE           8//16     // g      (2, 4, 8, 16)
#define ICM20602_GYO_SCALE           2000  // deg/s  (250, 500, 1000, 2000)

#define DIMU_ORIENT_ACC_X	    (+in[0])
#define DIMU_ORIENT_ACC_Y	    (-in[1])
#define DIMU_ORIENT_ACC_Z	    (-in[2])

#define DIMU_ORIENT_GYO_X	    (-in[0])
#define DIMU_ORIENT_GYO_Y	    (+in[1])
#define DIMU_ORIENT_GYO_Z	    (-in[2])

#define DIMU_ORIENT_MAG_X	    (-in[0])
#define DIMU_ORIENT_MAG_Y	    (+in[1])
#define DIMU_ORIENT_MAG_Z	    (-in[2])



#define COMM_PORT1		    "uart2"    
#define COMM_PORT2		    "uart3"      
//#define COMM_PORT3		    USART5




#define ANALOG_REF_VOLTAGE	3.3f
#define ANALOG_VIN_RTOP         10.0f
#define ANALOG_VIN_RBOT         10.0f
#define ANALOG_EXT_VOLT_RTOP	10.0f
#define ANALOG_EXT_VOLT_RBOT	1.2f

//#define ANALOG_EXT_AMP_RTOP	1.0f
//#define ANALOG_EXT_AMP_RBOT	1.2f

#define	ANALOG_CHANNEL_VIN	       ADC_Channel_10
#define	ANALOG_CHANNEL_EXT_VOLT	    ADC_Channel_11
#define	ANALOG_CHANNEL_EXT_AMP	    ADC_Channel_10
#define ANALOG_CHANNEL_SPARE4	    ADC_Channel_4
#define ANALOG_CHANNEL_SPARE5	    ADC_Channel_5
#define ANALOG_CHANNEL_SPARE6	    ADC_Channel_6
#define ANALOG_CHANNEL_SPARE7	    ADC_Channel_7


#define UKF_VEL_Q               +3.2545e-02     // +0.032544903471       0.000000350530 +0.000037342305
#define UKF_VEL_ALT_Q           +1.4483e-01     // +0.144827254833       0.000000347510 -0.000055111229
#define UKF_POS_Q               +7.1562e+03     // +7156.240473309331    0.000000352142 +2.727925965284749
#define UKF_POS_ALT_Q           +5.3884e+03     // +5388.369673129109    0.000000351319 -6.187843541372100
#define UKF_ACC_BIAS_Q          +1.3317e-03     // +0.001331748045       0.000000359470 +0.000000039113
#define UKF_GYO_BIAS_Q          +4.5256e-02     // +0.045255679186       0.000000349060 +0.000045999290
#define UKF_QUAT_Q              +5.4005e-04     // +0.000540045060       0.000000353882 +0.000000029711
#define UKF_PRES_ALT_Q          +6.3105e+01     // +63.104671424320      0.000000353790 +0.0166164673283
#define UKF_ACC_BIAS_V          +7.8673e-07     // +0.000000786725       0.000000345847 -0.000000000977
#define UKF_GYO_BIAS_V          +4.0297e-09     // +0.000000004030       0.000000359017 +0.000000000000
#define UKF_RATE_V              +1.7538e-05     // +0.000017538388       0.000000358096 +0.000000000397
#define UKF_VEL_V               +2.8605e-07     // +0.000000286054       0.000000351709 +0.000000000183
#define UKF_ALT_VEL_V           +6.8304e-08     // +0.000000068304       0.000000362348 -0.000000000050
#define UKF_GPS_POS_N           +8.0703e-06     // +0.000008070349       0.000000353490 +0.000000005602
#define UKF_GPS_POS_M_N         +3.0245e-05     // +0.000030245341       0.000000345021 -0.000000008396
#define UKF_GPS_ALT_N           +1.1796e-05     // +0.000011795879       0.000000356036 -0.000000010027
#define UKF_GPS_ALT_M_N         +3.8329e-05     // +0.000038328879       0.000000346581 +0.000000027268
#define UKF_GPS_VEL_N           +1.7640e-01     // +0.176404763511       0.000000355574 -0.000094105688
#define UKF_GPS_VEL_M_N         +3.0138e-02     // +0.030138272888       0.000000343584 -0.000002668997
#define UKF_GPS_VD_N            +4.6379e+00     // +4.637855992835       0.000000358079 +0.000310962082
#define UKF_GPS_VD_M_N          +1.3127e-02     // +0.013127146795       0.000000347978 -0.000001550944
#define UKF_ALT_N               +9.5913e-02     // +0.095913477777       0.000000356359 -0.000049781087
#define UKF_ACC_N               +6.3287e-05     // +0.000063286884       0.000000342761 -0.000000022717
#define UKF_DIST_N              +9.7373e-03     // +0.009737270392       0.000000356147 +0.000009059372
#define UKF_MAG_N               +5.2355e-01     // +0.523549973965       0.000000500000 +0.000000000000
#define UKF_POS_DELAY           +2.1923e+03     // +2192.300048828125    0.000000500000 +0.000000000000125
#define UKF_VEL_DELAY           -1.0182e+05     // -101820.000000000000  0.000000500000 +0.00000000000000000


//#define RCC_SYSOFF_PORT		GPIOB
//#define RCC_SYSOFF_PIN		GPIO_Pin_14
#define RCC_EN1_PORT		GPIOC
#define RCC_EN1_PIN		    GPIO_Pin_5
#define RCC_EN2_PORT		GPIOC
#define RCC_EN2_PIN		   GPIO_Pin_4
//#define RCC_STEPUP_EN_PORT      GPIOC
//#define RCC_STEPUP_EN_PIN       GPIO_Pin_2
#define RCC_SYNC_PORT           GPIOB
#define RCC_SYNC_PIN            GPIO_Pin_10

#define MOTORS_PWM_FREQ		400	// Hz
#define PWM_RESOLUTION		1000000

#define HAS_ONBOARD_ESC
#define MOTORS_ONBOARD_PWM_FREQ		80000	// Hz
#define ONBOARD_ESC_PWM_RESOLUTION	84000000
#define ONBOARD_ESC_PWM_MIN		0
#define ONBOARD_ESC_PWM_ARM		0
#define ONBOARD_ESC_PWM_START		0
#define ONBOARD_ESC_PWM_MAX		1050

#define DEFAULT_IMU_ACC_BIAS_X      0.0
#define DEFAULT_IMU_ACC_BIAS_Y      0.0
#define DEFAULT_IMU_ACC_BIAS_Z      0.0
#define DEFAULT_IMU_ACC_BIAS1_X     0.0
#define DEFAULT_IMU_ACC_BIAS1_Y     0.0
#define DEFAULT_IMU_ACC_BIAS1_Z     0.0
#define DEFAULT_IMU_ACC_BIAS2_X     0.0
#define DEFAULT_IMU_ACC_BIAS2_Y     0.0
#define DEFAULT_IMU_ACC_BIAS2_Z     0.0
#define DEFAULT_IMU_ACC_BIAS3_X     0.0
#define DEFAULT_IMU_ACC_BIAS3_Y     0.0
#define DEFAULT_IMU_ACC_BIAS3_Z     0.0

#define DEFAULT_IMU_ACC_SCAL_X      1.0
#define DEFAULT_IMU_ACC_SCAL_Y      1.0
#define DEFAULT_IMU_ACC_SCAL_Z      1.0
#define DEFAULT_IMU_ACC_SCAL1_X     0.0
#define DEFAULT_IMU_ACC_SCAL1_Y     0.0
#define DEFAULT_IMU_ACC_SCAL1_Z     0.0
#define DEFAULT_IMU_ACC_SCAL2_X     0.0
#define DEFAULT_IMU_ACC_SCAL2_Y     0.0
#define DEFAULT_IMU_ACC_SCAL2_Z     0.0
#define DEFAULT_IMU_ACC_SCAL3_X     0.0
#define DEFAULT_IMU_ACC_SCAL3_Y     0.0
#define DEFAULT_IMU_ACC_SCAL3_Z     0.0

#define DEFAULT_IMU_ACC_ALGN_XY     0.0
#define DEFAULT_IMU_ACC_ALGN_XZ     0.0
#define DEFAULT_IMU_ACC_ALGN_YX     0.0
#define DEFAULT_IMU_ACC_ALGN_YZ     0.0
#define DEFAULT_IMU_ACC_ALGN_ZX     0.0
#define DEFAULT_IMU_ACC_ALGN_ZY     0.0

#define DEFAULT_IMU_MAG_BIAS_X      0.0
#define DEFAULT_IMU_MAG_BIAS_Y      0.0
#define DEFAULT_IMU_MAG_BIAS_Z      0.0
#define DEFAULT_IMU_MAG_BIAS1_X     0.0
#define DEFAULT_IMU_MAG_BIAS1_Y     0.0
#define DEFAULT_IMU_MAG_BIAS1_Z     0.0
#define DEFAULT_IMU_MAG_BIAS2_X     0.0
#define DEFAULT_IMU_MAG_BIAS2_Y     0.0
#define DEFAULT_IMU_MAG_BIAS2_Z     0.0
#define DEFAULT_IMU_MAG_BIAS3_X     0.0
#define DEFAULT_IMU_MAG_BIAS3_Y     0.0
#define DEFAULT_IMU_MAG_BIAS3_Z     0.0

#define DEFAULT_IMU_MAG_SCAL_X      1.0
#define DEFAULT_IMU_MAG_SCAL_Y      1.0
#define DEFAULT_IMU_MAG_SCAL_Z      1.0
#define DEFAULT_IMU_MAG_SCAL1_X     0.0
#define DEFAULT_IMU_MAG_SCAL1_Y     0.0
#define DEFAULT_IMU_MAG_SCAL1_Z     0.0
#define DEFAULT_IMU_MAG_SCAL2_X     0.0
#define DEFAULT_IMU_MAG_SCAL2_Y     0.0
#define DEFAULT_IMU_MAG_SCAL2_Z     0.0
#define DEFAULT_IMU_MAG_SCAL3_X     0.0
#define DEFAULT_IMU_MAG_SCAL3_Y     0.0
#define DEFAULT_IMU_MAG_SCAL3_Z     0.0

#define DEFAULT_IMU_MAG_ALGN_XY     0.0
#define DEFAULT_IMU_MAG_ALGN_XZ     0.0
#define DEFAULT_IMU_MAG_ALGN_YX     0.0
#define DEFAULT_IMU_MAG_ALGN_YZ     0.0
#define DEFAULT_IMU_MAG_ALGN_ZX     0.0
#define DEFAULT_IMU_MAG_ALGN_ZY     0.0

#define DEFAULT_IMU_GYO_BIAS_X      0.0
#define DEFAULT_IMU_GYO_BIAS_Y      0.0
#define DEFAULT_IMU_GYO_BIAS_Z      0.0
#define DEFAULT_IMU_GYO_BIAS1_X     0.0
#define DEFAULT_IMU_GYO_BIAS1_Y     0.0
#define DEFAULT_IMU_GYO_BIAS1_Z     0.0
#define DEFAULT_IMU_GYO_BIAS2_X     0.0
#define DEFAULT_IMU_GYO_BIAS2_Y     0.0
#define DEFAULT_IMU_GYO_BIAS2_Z     0.0
#define DEFAULT_IMU_GYO_BIAS3_X     0.0
#define DEFAULT_IMU_GYO_BIAS3_Y     0.0
#define DEFAULT_IMU_GYO_BIAS3_Z     0.0

#define DEFAULT_IMU_GYO_SCAL_X      1.0
#define DEFAULT_IMU_GYO_SCAL_Y      1.0
#define DEFAULT_IMU_GYO_SCAL_Z      1.0

#define DEFAULT_IMU_GYO_ALGN_XY     0.0
#define DEFAULT_IMU_GYO_ALGN_XZ     0.0
#define DEFAULT_IMU_GYO_ALGN_YX     0.0
#define DEFAULT_IMU_GYO_ALGN_YZ     0.0
#define DEFAULT_IMU_GYO_ALGN_ZX     0.0
#define DEFAULT_IMU_GYO_ALGN_ZY     0.0

#endif

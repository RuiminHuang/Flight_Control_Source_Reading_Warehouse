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
    Copyright 2015-2016 Maxim Paperno
*/

#include "aq.h"
#include "telemetry.h"
#include "pid.h"
#include "util.h"
#include "comm.h"
#include "motors.h"
#include "control.h"
#include "config.h"
#include "nav.h"
#include "compass.h"
#include "aq_timer.h"
#include "imu.h"
#include "radio.h"
#include "nav_ukf.h"
#include "aq_mavlink.h"
#include "supervisor.h"
#include "gps.h"
#include "run.h"
#include "rc.h"
#ifdef HAS_QUATOS
#include "quatos.h"
#endif

#include <string.h>
#include <math.h>
#ifndef __CC_ARM
#include <intrinsics.h>
#endif
#include "fuzzyPID.h"

controlStruct_t controlData;

FUZZY_PID rate_fuzzy_pid[2];

#define C_ETHR  0.35f   //�����ֵ
#define C_ECTHR 0.004f    //���仯����ֵ

#define C_KPTHR 10.0f   //KP�仯����ֵ
#define C_KITHR 0.01f
#define C_KDTHR 5.0f


rt_uint32_t *controlTaskStack;
static struct rt_thread controlTask;

void controlSetMode(void) {
    // Set and announce input control mode

    // user control
    if (navData.mode < NAV_STATUS_POSHOLD) {
        // what is the user controlling, rotation rate or rotation angle?
        // check for rate modes
        if (rcIsSwitchActive(NAV_CTRL_LR_MODE)) {
            if (controlData.controlMode != CTRL_MODE_LTD_RATE)
                AQ_NOTICE("Manual Limited Rate Control Mode active.");
            controlData.controlMode = CTRL_MODE_LTD_RATE;
        }
        else if (rcIsSwitchActive(NAV_CTRL_RT_MODE)) {
            if (controlData.controlMode != CTRL_MODE_RATE)
                AQ_NOTICE("Manual Rate Control Mode active.");
            controlData.controlMode = CTRL_MODE_RATE;
        }
        // default is angle control mode
        else {
            if (controlData.controlMode != CTRL_MODE_ANGLE)
                AQ_NOTICE("Manual Angle Control Mode active.");
            controlData.controlMode = CTRL_MODE_ANGLE;
        }
    }
    // nav has control
    else {
        if (controlData.controlMode != CTRL_MODE_NAV)
            AQ_NOTICE("Navigation Controller active.");
        controlData.controlMode = CTRL_MODE_NAV;
    }

}


/*
void test_io_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;


    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void led_onn(void)
{
   GPIO_SetBits(GPIOC,GPIO_Pin_5 | GPIO_Pin_6);
}

void led_offf(void)
{
    GPIO_ResetBits(GPIOC,GPIO_Pin_5 | GPIO_Pin_6);
}

void RC_LEDToggle(void)
{
   GPIOC->ODR ^= (GPIO_Pin_5 | GPIO_Pin_6);
}
*/


int16_t applay_deadzone(int16_t in ,int16_t deadzone)
{
    if (abs(in)<=deadzone)
    {
        return 0;
    }
    else if (in>deadzone)
    {
        return in-deadzone;
    }
    else
    {
        return in+deadzone;
    }
}

void controlTaskCode(void *unused) {
    float outputThrot;          // final outputs for motor commands
    float outputs[3];
    float ratesActual[3];       // current gyo d-rates
    float tmp_f[4];             // temp storage
    float flipBias;
    int16_t rc[3];              // RC command channels
    uint8_t axis;               // index for loops
    bool rateTiltLimit;
#ifdef _DISABLE_MANUAL
    //bool take_off;
    //float vertStick;
#endif

#ifdef _DISARM_ON_LAND
    uint32_t mt_thro_low;
    uint32_t beep_cnt=0;
#endif

#ifdef _DISARM_AFTER_3S
    uint32_t disarmCount;
#endif

#ifdef _INVERT_DISARM
    uint32_t invertDisarmTimes;
#endif

    // init per-axis data //

    // rate override timer
    uint16_t overrides[3] = {0};
    // timeout limits
    const uint16_t ovrdTimeouts[3] = { CONTROL_MIN_PR_OVERRIDE, CONTROL_MIN_PR_OVERRIDE, CONTROL_MIN_YAW_OVERRIDE };
    // map params to axes
    const int pCtrlFact[2] = { CTRL_FACT_ROLL, CTRL_FACT_PITC };  // no yaw angle control factor
    const int pManRate[3]  = { CTRL_MAN_TLT_RT, CTRL_MAN_TLT_RT, CTRL_MAN_YAW_RT };
    // point to UKF and nav data
    const float *navHoldTilt[2]  = { &navData.holdTiltE, &navData.holdTiltN };
    const float *anglesActual[3] = { &AQ_ROLL, &AQ_PITCH, &AQ_YAW };

    AQ_NOTICE("Control task started\n");

#ifdef _DISABLE_MANUAL
    navData.takeOff = false;
#endif

#ifdef _DISARM_AFTER_3S
    disarmCount = 0;
#endif

#ifdef _INVERT_DISARM
    invertDisarmTimes = 0;
#endif

    while (1) {
        // wait for work
        rt_sem_take(imuData.dRateFlag, RT_WAITING_FOREVER);
#ifdef _DISARM_ON_LAND
        if (beep_cnt)
        {
            beep_cnt--;
            //led_onn();
        }
        else
        {
            //led_offf();
        }
#endif

#ifdef _DISARM_AFTER_3S
        if ((supervisorData.state & STATE_ARMED) && (navData.takeOff == false) && ((navData.mode >= NAV_STATUS_ALTHOLD)))
        {

            if (disarmCount == 150000)
                disarmCount = DISARM_TIMES;

            if ((disarmCount) && (disarmCount<=DISARM_TIMES))
            {
                disarmCount--;
                if (!disarmCount)
                {
                    // shut everything down (sure hope we are really on the ground :)
                    //spvrModeOverride ==misson  �ٸɣ�
                    navData.spvrModeOverride = 0;
                    supervisorDisarm();   
                }
            }
        }
//       else
//       {
//           disarmCount = 0;
//       }
        if (supervisorData.state & STATE_DISARMED)
        {
            disarmCount = 150000;
        }
#endif
        // this needs to be done ASAP with the freshest of data
        if (supervisorData.state & STATE_ARMED) {

#ifdef _DISABLE_MANUAL
            //vertStick = RADIO_THROT - RADIO_MID_THROTTLE;

            /*
            //��������в����ʵĵط�
            if ((navData.mode >= NAV_STATUS_ALTHOLD)&&(navData.mode < NAV_STATUS_MISSION))
            {
                 if (vertStick > p[CTRL_DBAND_THRO])
                    take_off = true;
            }
            else if (navData.mode == NAV_STATUS_MISSION)
            {
                if (navData.targetHoldSpeedAlt>0)
                {
                   take_off = true;
                }
            }
            else
            {
                 take_off = false;
            }
            */
            //��������в����ʵĵط�
            if (navData.mode >= NAV_STATUS_ALTHOLD)
            {
                if (navData.holdSpeedAlt/*targetHoldSpeedAlt*/>0.2f)
                {
                    navData.takeOff = true;
                }
            }
            else
            {
                navData.takeOff = false;
            }
#endif

#ifdef _DISARM_ON_LAND

            if ((supervisorData.state & STATE_ARMED) && (navData.mode >= NAV_STATUS_ALTHOLD)&&(navData.takeOff))
            {
                if (navData.holdSpeedAlt < 0.1f)//if (navData.targetHoldSpeedAlt<0)
                {
                    if (motorsData.throttle <= 1)
                    {
                        if (++mt_thro_low >= _DISARM_ON_LAND_CNT)
                        {
                            // shut everything down (sure hope we are really on the ground :)
                            //spvrModeOverride ==misson  �ٸɣ�
                            navData.spvrModeOverride = 0;
                            supervisorDisarm();   
                            beep_cnt = 1000;
                            mt_thro_low = 0;
                        }
                    }
                }
                else
                {
                    mt_thro_low = 0;
                }
            }
            else
            {
                mt_thro_low = 0;
            }

#endif
            // flying?
            if (RADIO_THROT > p[CTRL_MIN_THROT] || navData.mode >= NAV_STATUS_ALTHOLD) {

                supervisorThrottleUp(1);

                rateTiltLimit = false;
                flipBias = (UKF_Q1 * UKF_Q1 + UKF_Q4 * UKF_Q4);  // < 0.5 == inverted attitude

#ifdef _INVERT_DISARM
                if ((flipBias < 0.5f) && (navData.mode >= NAV_STATUS_ALTHOLD))
                {
                    if (++invertDisarmTimes >= _INVERT_DISARM_TIMS)
                    {   //�ɻ�����
                        //spvrModeOverride ==misson  �ٸɣ�
                        navData.spvrModeOverride = 0;
                        supervisorDisarm();          
                        invertDisarmTimes = 0;
                    }
                }
                else
                {
                    invertDisarmTimes = 0;
                }
#endif


                // TODO: test why gyo bias is left out for PID
                ratesActual[RPY_R] = IMU_DRATEX ;//+ UKF_GYO_BIAS_X;
                ratesActual[RPY_P] = IMU_DRATEY ;//+ UKF_GYO_BIAS_Y;
                ratesActual[RPY_Y] = IMU_DRATEZ ;//+ UKF_GYO_BIAS_Z;
                if (controlData.controllerType == CONTROLLER_TYPE_QUATOS) {
                    //ratesActual[RPY_R] += UKF_GYO_BIAS_X;
                    //ratesActual[RPY_P] += UKF_GYO_BIAS_Y;
                    //ratesActual[RPY_Y] += UKF_GYO_BIAS_Z;
                    ratesActual[RPY_R] = IMU_DRATEX + UKF_GYO_BIAS_X;
                    ratesActual[RPY_P] = IMU_DRATEY + UKF_GYO_BIAS_Y;
                    ratesActual[RPY_Y] = IMU_DRATEZ + UKF_GYO_BIAS_Z;
                }

                // motors not running yet,
#ifdef _DISABLE_MANUAL
                if ((motorsData.throttle == 0)||((navData.mode >= NAV_STATUS_ALTHOLD)&&(navData.takeOff == false)))
#else
                if (motorsData.throttle == 0)
#endif
                {
                    // Reset all PIDs and filters
                    for (axis = 0; axis < 3; ++axis) {
                        overrides[axis] = 0;
                        utilFilterReset3(controlData.rateFilter[axis], 0.0f);
                        if (controlData.controllerType == CONTROLLER_TYPE_QUATOS)
                            continue;
                        if (axis != RPY_Y)
                            utilFilterReset3(controlData.angleFilter[axis], 0.0f);
                        pidZeroIntegral(controlData.ratePID[axis], 0.0f, 0.0f);
                        pidZeroIntegral(controlData.anglePID[axis], 0.0f, 0.0f);
                        pidZeroIntegral(controlData.rateModePID[axis], 0.0f, 0.0f);
                    }

#ifdef HAS_QUATOS
                    // reset quatos controller on startup
                    if (controlData.controllerType == CONTROLLER_TYPE_QUATOS) {
                        quatosReset(&UKF_Q1);
                        quatos(&UKF_Q1, &UKF_Q1, ratesActual, ratesActual, overrides);
                    }
#endif

                    // use this heading as hold heading
                    controlData.anglesDesired[RPY_Y] = navData.holdHeading = AQ_YAW;

                    // also set this position as hold position
                    if (navData.mode == NAV_STATUS_POSHOLD)
                        navUkfSetHereAsPositionTarget();
                }

                // Calculate throttle output first. //
                // are we in altitude hold mode?
                if (navData.mode >= NAV_STATUS_ALTHOLD) {
                    // override throttle with nav's request
                    {
                        outputThrot = pidUpdate(navData.altSpeedPID, navData.holdSpeedAlt, -VELOCITYD) * CONTROL_RADIO_THROT_SCALE;

                        // don't allow negative throttle to be built up
                        if (navData.altSpeedPID->iState < 0.0f)
                            navData.altSpeedPID->iState = 0.0f;
#ifdef _DISABLE_MANUAL
                        if (navData.takeOff == false)
                        {
                            pidZeroIntegral(navData.altSpeedPID, 0.0f, 0.0f);
                            outputThrot=_ONLAND_THRO;
                        }
#endif
                    }
                }
                // manual throttle
                else
                    outputThrot = ((uint32_t)RADIO_THROT - p[CTRL_MIN_THROT]) * CONTROL_RADIO_THROT_SCALE * configGetParamValue(CTRL_FACT_THRO);

                // progressively cut throttle if upside-down
                if (flipBias < 0.5 && ((navData.mode >= NAV_STATUS_ALTHOLD && configCheckFlag(CONFIG_FLAG_INVRT_TCUT_AUTO)) || (navData.mode < NAV_STATUS_ALTHOLD && configCheckFlag(CONFIG_FLAG_INVRT_TCUT_MAN))))
                    // curve graph: https://www.desmos.com/calculator/fi6z9dnb4m
                    outputThrot *= 1.0f - (0.99f - powf(flipBias, 4) * 16.0f);

                // constrain throttle
                outputThrot = constrainInt(outputThrot, 1, MOTORS_SCALE);

                // Now figure out the desired pitch/roll/yaw controls. //

//                rc[RPY_R] = RADIO_ROLL;
//                rc[RPY_P] = RADIO_PITCH;
//                rc[RPY_Y] = RADIO_RUDD;

                rc[RPY_R] = applay_deadzone(RADIO_ROLL, p[CTRL_DEAD_BAND]);
                rc[RPY_P] = applay_deadzone(RADIO_PITCH, p[CTRL_DEAD_BAND]);
                rc[RPY_Y] = applay_deadzone(RADIO_RUDD, p[CTRL_DBAND_YAW]);

                // adjust controls for heading-free mode
                if (controlData.controlMode == CTRL_MODE_ANGLE && navData.headFreeMode > NAV_HEADFREE_OFF && configCheckFlag(CONFIG_FLAG_ALWAYS_ALLOW_HF)) {
                    // rotate controls to stored frame
                    tmp_f[RPY_R] = rc[RPY_R] * navData.hfReferenceCos - rc[RPY_P] * navData.hfReferenceSin;
                    tmp_f[RPY_P] = rc[RPY_P] * navData.hfReferenceCos + rc[RPY_R] * navData.hfReferenceSin;
                    // then compensate for current rotation
                    rc[RPY_R] = tmp_f[RPY_R] * navUkfData.yawCos + tmp_f[RPY_P] * navUkfData.yawSin;
                    rc[RPY_P] = tmp_f[RPY_P] * navUkfData.yawCos - tmp_f[RPY_R] * navUkfData.yawSin;
                }

                // limit tilt angle in limited-rate mode
                if (controlData.controlMode == CTRL_MODE_LTD_RATE) {
                    // calculate max tilt
                    // try to predict final angle based on roll rate and remaining timeout steps

                    // method A: normalize total tilt angle and current rates
                    tmp_f[0] = RADIO_STICK_SCALE  * ((configGetParamValue(pCtrlFact[RPY_R]) + configGetParamValue(pCtrlFact[RPY_P])) * 0.5f);
                    tmp_f[1] = __sqrtf(ratesActual[RPY_R] * ratesActual[RPY_R] + ratesActual[RPY_P] * ratesActual[RPY_P]);
                    tmp_f[0] -= tmp_f[1] * AQ_INNER_TIMESTEP * MAX(overrides[RPY_R], overrides[RPY_P]);
                    tmp_f[2] = __sqrtf(*anglesActual[RPY_R] * *anglesActual[RPY_R] + *anglesActual[RPY_P] * *anglesActual[RPY_P]);
                    if (tmp_f[2] >= tmp_f[0]) {
                        rateTiltLimit = true;
                        overrides[RPY_R] /= 4;
                        overrides[RPY_P] /= 4;
                    }

                    // method B: check each axis separately
//		    tmp_f[RPY_R] = RADIO_STICK_SCALE * configGetParamValue(pCtrlFact[RPY_R]) + fabsf(ratesActual[RPY_R]) * AQ_INNER_TIMESTEP * overrides[RPY_R];
//		    tmp_f[RPY_P] = RADIO_STICK_SCALE * configGetParamValue(pCtrlFact[RPY_P]) + fabsf(ratesActual[RPY_P]) * AQ_INNER_TIMESTEP * overrides[RPY_P];
//		    if (fabsf(*anglesActual[RPY_R]) >= tmp_f[RPY_R]) {
//			rateTiltLimit = true;
//			overrides[RPY_R] /= 4;
//		    }
//		    if (fabsf(*anglesActual[RPY_P]) >= tmp_f[RPY_P]) {
//			rateTiltLimit = true;
//			overrides[RPY_P] /= 4;
//		    }
                }

                for (axis = 0; axis < 3; ++axis)
                {
                    // manual rotation-rate control mode or yaw rate override in any mode
                    //if ((controlData.controlMode > CTRL_MODE_ANGLE && !rateTiltLimit) || (axis == RPY_Y && abs(rc[axis]) > (int)p[CTRL_DBAND_YAW]))
                    if ((controlData.controlMode > CTRL_MODE_ANGLE && !rateTiltLimit) || (axis == RPY_Y && abs(rc[axis]) > 0))
                    {

                        controlData.ratesDesired[axis] = (float)rc[axis];
                        if (controlData.controlMode == CTRL_MODE_ANGLE)
                        {
//                            if (rc[axis] > p[CTRL_DEAD_YAW])
//                                controlData.ratesDesired[axis] -= p[CTRL_DEAD_YAW];
//                            else
//                                controlData.ratesDesired[axis] += p[CTRL_DEAD_YAW];

#ifdef _DISABLE_MANUAL
                            if (((navData.mode >= NAV_STATUS_ALTHOLD)&&(navData.takeOff == false)))
                            {
                                controlData.ratesDesired[RPY_Y]=0;

                            }
#endif

                        }

                        // calculate desired rate based on full stick scale
                        controlData.ratesDesired[axis] *= configGetParamValue(pManRate[axis]) * CONTROL_RADIO_VALUE_SCALE;

                        // if first entering override mode, reset rate mode PID and filter
                        if (!overrides[axis]) {
                            if (controlData.controllerType != CONTROLLER_TYPE_QUATOS)
                                pidZeroIntegral(controlData.rateModePID[axis], 0.0f, 0.0f);

                            utilFilterReset3(controlData.rateFilter[axis], 0.0f); // controlData.ratesDesired[axis] // ratesActual[axis]
                        }

                        // smooth
                        controlData.ratesDesired[axis] = utilFilter3(controlData.rateFilter[axis], controlData.ratesDesired[axis]);

                        overrides[axis] = ovrdTimeouts[axis];

                        // keep up with actual craft heading
                        if (axis == RPY_Y)
                            controlData.anglesDesired[RPY_Y] = navData.holdHeading = AQ_YAW;
                        else
                            controlData.anglesDesired[axis] = 0.0f;
                    }
                    // angle control mode
                    else
                    {
                        // continue axis rate override until timeout
                        if (!overrides[axis]) {
                            if (axis == RPY_Y) {
                                // constrain nav yaw angle rates
                                tmp_f[RPY_Y] = configGetParamValue(CTRL_NAV_YAW_RT) * AQ_INNER_TIMESTEP;
                                tmp_f[RPY_Y] = constrainFloat(compassDifference(controlData.anglesDesired[RPY_Y], navData.holdHeading), -tmp_f[RPY_Y], +tmp_f[RPY_Y]);
                                controlData.anglesDesired[RPY_Y] = compassNormalize(controlData.anglesDesired[RPY_Y] + tmp_f[RPY_Y]);
                            }
                            else {
                                // navigation requests
                                if (controlData.controlMode == CTRL_MODE_NAV)
                                {
                                    controlData.anglesDesired[axis] = *navHoldTilt[axis];
                                }
                                // user pitch / roll requests
                                else
                                {
                                    controlData.anglesDesired[axis] = (float)rc[axis] * configGetParamValue(pCtrlFact[axis]);
                                }
                            }
                        }
                        // still overriding
                        else {
                            --overrides[axis];
                            controlData.ratesDesired[axis] = utilFilter3(controlData.rateFilter[axis], 0.0f); 
                            // keep up with actual craft heading
                            if (axis == RPY_Y)
                                controlData.anglesDesired[RPY_Y] = navData.holdHeading = AQ_YAW;
                            else
                                controlData.anglesDesired[axis] = 0.0f;

                            // if override timeout then reset angle mode PIDs and filter
                            if (!overrides[axis] && controlData.controllerType != CONTROLLER_TYPE_QUATOS) {
                                pidZeroIntegral(controlData.anglePID[axis], 0.0f, 0.0f);
                                pidZeroIntegral(controlData.ratePID[axis], 0.0f, 0.0f);
                                if (axis != RPY_Y)
                                    utilFilterReset3(controlData.angleFilter[axis], controlData.anglesDesired[axis]);
                            }
                        }
                    } // control type

                }  // axes loop

                // Final output calculation and method depends on attitude controller type, PID or Quatos.

                // PID
                if (controlData.controllerType != CONTROLLER_TYPE_QUATOS) {

                    if (controlData.controlMode == CTRL_MODE_NAV) {
                        // rotate nav's NE frame of reference to our craft's local frame of reference
                        tmp_f[RPY_R] = controlData.anglesDesired[RPY_R];
                        controlData.anglesDesired[RPY_R] = controlData.anglesDesired[RPY_R] * navUkfData.yawCos + controlData.anglesDesired[RPY_P] * navUkfData.yawSin;
                        controlData.anglesDesired[RPY_P] = controlData.anglesDesired[RPY_P] * navUkfData.yawCos - tmp_f[RPY_R] * navUkfData.yawSin;
                    }

                    tmp_f[3] = configGetParamValue(CTRL_MAX);  // to constrain output
#ifdef _DISABLE_MANUAL
                    if((navData.takeOff == false) && (navData.mode >= NAV_STATUS_ALTHOLD))
                    {
                        tmp_f[3] = tmp_f[3]/7;
                    }
#endif
                    // calculate final outputs per RPY axis
                    for (axis = 0; axis < 3; ++axis) {
                        outputs[axis] = 0.0f;

                        if (overrides[axis])  // rotation rate override
                            // get just the rate
                            outputs[axis] = pidUpdate(controlData.rateModePID[axis], controlData.ratesDesired[axis], ratesActual[axis]);

                        else {  // angle control

                            if (axis == RPY_Y) {
                                // seek a 0 deg difference between hold heading and actual yaw
                                outputs[RPY_Y] = pidUpdate(controlData.anglePID[RPY_Y], 0.0f, compassDifference(controlData.anglesDesired[RPY_Y], *anglesActual[RPY_Y]));
                            } else {
                                // smooth
                                controlData.anglesDesired[axis] = utilFilter3(controlData.angleFilter[axis], controlData.anglesDesired[axis]);
                                // seek 0 diff between desired and actual angles
                                outputs[axis] = pidUpdate(controlData.anglePID[axis], controlData.anglesDesired[axis], *anglesActual[axis]);

                            }

                            if (axis == RPY_Y || controlData.controllerType == CONTROLLER_TYPE_PID_C)
                            {
                                // get rate from angle
                                outputs[axis] = pidUpdate(controlData.ratePID[axis], outputs[axis], ratesActual[axis]);
								
								//����pidʹ��ģ��pid
								if (axis<RPY_Y)
								{	
									//����p i d
								    //rate_fuzzy_pid[axis].kp += FUZZY_Calc_detKp(controlData.ratePID[axis]->error_sum, controlData.ratePID[axis]->error_delta, rate_fuzzy_pid[axis].ethr, rate_fuzzy_pid[axis].ecthr, rate_fuzzy_pid[axis].kpthr);  //����detKp
                                    //rate_fuzzy_pid[axis].ki += FUZZY_Calc_detKi(controlData.ratePID[axis]->error_sum, controlData.ratePID[axis]->error_delta, rate_fuzzy_pid[axis].ethr, rate_fuzzy_pid[axis].ecthr, rate_fuzzy_pid[axis].kithr);  //����detKi
                                    //rate_fuzzy_pid[axis].kd += FUZZY_Calc_detKd(controlData.ratePID[axis]->error_sum, controlData.ratePID[axis]->error_delta, rate_fuzzy_pid[axis].ethr, rate_fuzzy_pid[axis].ecthr, rate_fuzzy_pid[axis].kdthr);  //����detKd								
								    
									//ʹ���µ�pid
									//controlData.ratePID[axis]->p = rate_fuzzy_pid[axis].kp;
									//controlData.ratePID[axis]->i = rate_fuzzy_pid[axis].ki;
									//controlData.ratePID[axis]->d = rate_fuzzy_pid[axis].kd;
								
								}
								
                            }
                            else
                            {
                                // seek zero rate change and add it to output
                                outputs[axis] += pidUpdate(controlData.ratePID[axis], 0.0f, ratesActual[axis]);
                            }
                        }

                        // constrain output
                        outputs[axis] = constrainFloat(outputs[axis], -tmp_f[3], tmp_f[3]);
                    }

                    motorsCommands(outputThrot, outputs[RPY_P], outputs[RPY_R], outputs[RPY_Y]);

                }
#ifdef HAS_QUATOS
                // Quatos
                else {
                    // rates mode
                    if (controlData.controlMode > CTRL_MODE_ANGLE && !rateTiltLimit) {
                        // keep up with current orientation
                        quatos(&UKF_Q1, &UKF_Q1, ratesActual, controlData.ratesDesired, overrides);
                    }
                    // angle mode
                    else {
                        if (controlData.controlMode == CTRL_MODE_NAV)
                            // control from nav reference frame
                            eulerToQuatRPY(tmp_f, controlData.anglesDesired[RPY_R], controlData.anglesDesired[RPY_P], controlData.anglesDesired[RPY_Y]);
                        else
                            // local reference
                            eulerToQuatYPR(tmp_f, controlData.anglesDesired[RPY_Y], controlData.anglesDesired[RPY_P], controlData.anglesDesired[RPY_R]);
                        quatos(&UKF_Q1, tmp_f, ratesActual, controlData.ratesDesired, overrides);
                    }

                    quatosPowerDistribution(outputThrot);
                    motorsSendThrust();
                    motorsData.throttle = outputThrot;
                }
#endif
            }
            // no throttle input
            else
            {
                supervisorThrottleUp(0);

                motorsOff();
#ifdef _DISARM_ON_LAND

                mt_thro_low = 0;
#endif

            }
        }
        // not armed
        else
        {
            motorsOff();

#ifdef _DISABLE_MANUAL
            navData.takeOff = false;
#endif
#ifdef _DISARM_ON_LAND
            mt_thro_low = 0;
#endif

#ifdef _INVERT_DISARM
            invertDisarmTimes = 0;
#endif
        }

        controlSetMode();

        controlData.lastUpdate = IMU_LASTUPD;
        controlData.loops++;

    }  // perpetual loop
}


void controlInit(void) {
    AQ_NOTICE("Control init\n");

    memset((void *)&controlData, 0, sizeof(controlData));

    // default control input mode
    controlData.controlMode = CTRL_MODE_ANGLE;
    // parallel or cascading PID controller type
    controlData.controllerType = (configCheckFlag(CONFIG_FLAG_PID_CTRL_TYPE_C) ? CONTROLLER_TYPE_PID_C : CONTROLLER_TYPE_PID_P);

#ifdef HAS_QUATOS
    if ((uint8_t)p[QUATOS_ENABLE]) {
        quatosInit(AQ_INNER_TIMESTEP, AQ_OUTER_TIMESTEP);
        controlData.controllerType = CONTROLLER_TYPE_QUATOS;
    }
#endif

    // init tilt/roll filters and PIDs
    for (int i = 0; i < 2; ++i) {
        utilFilterInit3(controlData.rateFilter[i], AQ_INNER_TIMESTEP, p[CTRL_TLT_RTE_TAU], 0.0f);
        if (controlData.controllerType != CONTROLLER_TYPE_QUATOS) {
            utilFilterInit3(controlData.angleFilter[i], AQ_INNER_TIMESTEP, p[CTRL_TLT_ANG_TAU], 0.0f);
            controlData.ratePID[i]     = pidInit(CTRL_TLT_RTE_P,   CTRL_TLT_RTE_I,   CTRL_TLT_RTE_D,   CTRL_TLT_RTE_F,   CTRL_TLT_RTE_PM, CTRL_TLT_RTE_IM, CTRL_TLT_RTE_DM, CTRL_TLT_RTE_OM);
            controlData.rateModePID[i] = pidInit(CTRL_TLT_RTE_R_P, CTRL_TLT_RTE_R_I, CTRL_TLT_RTE_R_D, CTRL_TLT_RTE_R_F, CTRL_TLT_RTE_PM, CTRL_TLT_RTE_IM, CTRL_TLT_RTE_DM, CTRL_TLT_RTE_OM);
            controlData.anglePID[i]    = pidInit(CTRL_TLT_ANG_P,   CTRL_TLT_ANG_I,   CTRL_TLT_ANG_D,   CTRL_TLT_ANG_F,   CTRL_TLT_ANG_PM, CTRL_TLT_ANG_IM, CTRL_TLT_ANG_DM, CTRL_TLT_ANG_OM);
        }
    }

    // init yaw filters and PIDs
    utilFilterInit3(controlData.rateFilter[RPY_Y], AQ_INNER_TIMESTEP, p[CTRL_YAW_RTE_TAU], 0.0f);
    if (controlData.controllerType != CONTROLLER_TYPE_QUATOS) {
        controlData.ratePID[RPY_Y]  = pidInit(CTRL_YAW_RTE_P, CTRL_YAW_RTE_I, CTRL_YAW_RTE_D, CTRL_YAW_RTE_F, CTRL_YAW_RTE_PM, CTRL_YAW_RTE_IM, CTRL_YAW_RTE_DM, CTRL_YAW_RTE_OM);
        controlData.anglePID[RPY_Y] = pidInit(CTRL_YAW_ANG_P, CTRL_YAW_ANG_I, CTRL_YAW_ANG_D, CTRL_YAW_ANG_F, CTRL_YAW_ANG_PM, CTRL_YAW_ANG_IM, CTRL_YAW_ANG_DM, CTRL_YAW_ANG_OM);
        // yaw is always in rate mode, this assignment just simplifies code the taskCode processing loops
        controlData.rateModePID[RPY_Y] = controlData.ratePID[RPY_Y];
    }
    
	//roll pitch �Ƕȶȿ���ʹ��fuzzy pid
	for (int i = 0; i < 2; ++i)
	{
	   rate_fuzzy_pid[i].ethr = C_ETHR;//�����ֵ
	   rate_fuzzy_pid[i].ecthr = C_ECTHR;//���仯����ֵ
	   rate_fuzzy_pid[i].kpthr = C_KPTHR;//kp�仯����ֵ
	   rate_fuzzy_pid[i].kithr = C_KITHR;//kp�仯����ֵ
	   rate_fuzzy_pid[i].kdthr = C_KDTHR;//kp�仯����ֵ

	   rate_fuzzy_pid[i].kp = controlData.ratePID[i]->p;
	   rate_fuzzy_pid[i].ki = controlData.ratePID[i]->i;
	   rate_fuzzy_pid[i].kd	= controlData.ratePID[i]->d;
	}	
	//yaw ���Ʋ�ʹ��ģ��pid
//    rate_fuzzy_pid[2].ethr
//    rate_fuzzy_pid[2].ecthr
//    rate_fuzzy_pid[2].kpthr
//    rate_fuzzy_pid[2].kithr
//    rate_fuzzy_pid[2].kdthr

//    rate_fuzzy_pid[2].kp
//    rate_fuzzy_pid[2].ki
//    rate_fuzzy_pid[2].kd	

	
    controlTaskStack = aqStackInit(CONTROL_STACK_SIZE, "CONTROL");

   
    rt_thread_init(&controlTask,
                   "contrl",
                   controlTaskCode,
                   RT_NULL,
                   (rt_uint8_t*)controlTaskStack,
                   CONTROL_STACK_SIZE*4,
                   CONTROL_PRIORITY,
                   5);

    rt_thread_startup(&controlTask);
}

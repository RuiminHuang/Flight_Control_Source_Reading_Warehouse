/*
 * File      : pwm.c
 * This file is part of RT-Polit
 * COPYRIGHT (C) 2018, RT-Polit Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://openlab.rt-thread.com/license/LICENSE.
 *
 * Change Logs:
 * Date           Author          Notes
 * 2018-06-18     jiezhi320       the first version.
 */
 
#include "pwm.h"  
#include "fsl_pwm.h"  
#include "fsl_iomuxc.h" 
#include "fsl_gpio.h" 
#include "fsl_xbara.h"
/*


GPIO_AD_B0_03  
GPIO_AD_B1_09  


GPIO_SD_B1_00  FLEXPWM1_PWMA3  PWM1
GPIO_SD_B1_01  FLEXPWM1_PWMB3  PWM2
GPIO_SD_B1_02  FLEXPWM2_PWMA3  PWM3
GPIO_SD_B1_03  FLEXPWM2_PWMB3  PWM4
*/

static pwm_config_t pwm1sm3_config;   
static pwm_config_t pwm2sm3_config;    //PWM2模块3配置结构体

//设置XBARA1的信号连接关系
//input: XBARA_INn选择,低8位有小,范围:0~87,参见xbar_input_signal_t的枚举值.
//output: XBARA_OUTx编号,低8位有效,范围:0~130, 参见xbar_output_signal_t的枚举值.
//该函数设置XBARA1_OUTx(output)对应的输入信号是哪个XBARA1_INn(input).
//详细的对应表,见:<<RT1052英文参考手册>>Table 3-5 和 Table 3-6
static void XBARA1_Signal_Set(xbar_input_signal_t input, xbar_output_signal_t output)
{ 
	uint8_t outx,inx;
  
	outx=output&0XFF;									//得到真正的XBARA_OUT编号.
	inx=input&0XFF;										//得到真正的XBARA_IN编号. 
	if(outx>3&&outx<20)IOMUXC_GPR->GPR6|=1<<(12+outx);	//GPIO做输出,则设置对应的I0为输出模式
	if(inx>3&&inx<20)IOMUXC_GPR->GPR6&=~(1<<(12+inx));	//GPIO做输入,则设置对应的IO位输入模式 
    
    XBARA_Init(XBARA1);                                 //初始化XBARA1
    XBARA_SetSignalsConnection(XBARA1,input,output);    //设置输入和输出连接
}

//初始化FLEXPWM,PWM2的模块3的通道A和B
//通过PWM2的模块3在通道A和B上面产生2路PWM输出.
//即在GPIO3_IO02和GPIO3_IO03上面输出PWM.
//psc : 预分频器,0~7,表示2^psc分频.
//fre : 频率
//duty:占空比
static void PWM1_SM3_PWMAB_Init(uint16_t psc,uint32_t fre,uint8_t duty)
{
    uint32_t sourceclock;
    pwm_signal_param_t pwm_ignal[2];
    pwm_clock_prescale_t pwm_prescale=(pwm_clock_prescale_t)psc; //分频
	
    XBARA1_Signal_Set(kXBARA1_InputLogicHigh,kXBARA1_OutputFlexpwm1Fault0);
    XBARA1_Signal_Set(kXBARA1_InputLogicHigh,kXBARA1_OutputFlexpwm1Fault1);
    XBARA1_Signal_Set(kXBARA1_InputLogicHigh,kXBARA1_OutputFlexpwm1234Fault2);
    XBARA1_Signal_Set(kXBARA1_InputLogicHigh,kXBARA1_OutputFlexpwm1234Fault3); 		
    
	  //IO功能设置
	  IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_00_FLEXPWM1_PWMA03,0);	 
	  IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_01_FLEXPWM1_PWMB03,0);   
    
	  //配置IO引脚GPIO_SD_B1_02和GPIO_SD_B1_03的功能
	  //低转换速度,驱动能力为R0/6,速度为100Mhz，关闭开路功能，使能pull/keepr
	  //选择keeper功能，下拉100K Ohm，关闭Hyst
	  IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_00_FLEXPWM1_PWMA03,0x10B0);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_01_FLEXPWM1_PWMB03,0x10B0);
    
    //初始化PWM2 模块3的通道A和B
    PWM_GetDefaultConfig(&pwm1sm3_config);              //先初始化为默认配置
    pwm1sm3_config.clockSource=kPWM_BusClock;           //时钟源为IP BUS=IPG_CLK_ROOT=150MHz
    pwm1sm3_config.prescale=pwm_prescale;               //设置分频
    pwm1sm3_config.reloadLogic=kPWM_ReloadPwmFullCycle; //全周期更新
    pwm1sm3_config.pairOperation=kPWM_Independent;      //PMWA PWMB独立模式
	  PWM_Init(PWM1,kPWM_Module_3,&pwm1sm3_config);       //初始化PWM2模块3

    //设置PWM的两个通道
    sourceclock=CLOCK_GetFreq(kCLOCK_IpgClk);
  
    //PWMA
    pwm_ignal[0].pwmChannel=kPWM_PwmA;                  //PWM通道A
    pwm_ignal[0].level=kPWM_HighTrue;                   //高电平有效
    pwm_ignal[0].dutyCyclePercent=duty;                 //占空比

    //PWMB
    pwm_ignal[1].pwmChannel=kPWM_PwmB;                  //PWM通道B
    pwm_ignal[1].level=kPWM_HighTrue;                   //高电平有效
    pwm_ignal[1].dutyCyclePercent=duty;                 //占空比

    //设置PWM2，中央对齐模式
    PWM_SetupPwm(PWM1,kPWM_Module_3,pwm_ignal,2,kPWM_SignedEdgeAligned,fre,sourceclock);
    
    PWM_SetPwmLdok(PWM1,kPWM_Control_Module_3,true);    //设置PWM的load ok位
    PWM_StartTimer(PWM1,kPWM_Control_Module_3);         //开启定时器
}

//更新PWM2占空比
//duty:占空比
//为了精度问题 修改了PWM_UpdatePwmDutycycle  库函数原型 jiezhi320
static void PWM1_SM3_DutySet(float duty1, float duty2 ) 
{
    PWM_UpdatePwmDutycycle(PWM1,kPWM_Module_3,kPWM_PwmA,kPWM_SignedEdgeAligned,duty1); //更新PWMA占空比
    PWM_UpdatePwmDutycycle(PWM1,kPWM_Module_3,kPWM_PwmB,kPWM_SignedEdgeAligned,duty2); //更新PWMB占空比
    PWM_SetPwmLdok(PWM1,kPWM_Control_Module_3,true);    //设置PWM的load ok位
}

//初始化FLEXPWM,PWM2的模块3的通道A和B
//通过PWM2的模块3在通道A和B上面产生2路PWM输出.
//即在GPIO3_IO02和GPIO3_IO03上面输出PWM.
//psc : 预分频器,0~7,表示2^psc分频.
//fre : 频率
//duty:占空比
static void PWM2_SM3_PWMAB_Init(uint16_t psc,uint32_t fre,uint8_t duty)
{
    uint32_t sourceclock;
    pwm_signal_param_t pwm_ignal[2];
    pwm_clock_prescale_t pwm_prescale=(pwm_clock_prescale_t)psc; //分频
	
    XBARA1_Signal_Set(kXBARA1_InputLogicHigh,kXBARA1_OutputFlexpwm2Fault0);
    XBARA1_Signal_Set(kXBARA1_InputLogicHigh,kXBARA1_OutputFlexpwm2Fault1);
    XBARA1_Signal_Set(kXBARA1_InputLogicHigh,kXBARA1_OutputFlexpwm1234Fault2);
    XBARA1_Signal_Set(kXBARA1_InputLogicHigh,kXBARA1_OutputFlexpwm1234Fault3); 		
    
	  //IO功能设置
	  IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_02_FLEXPWM2_PWMA03,0);	 
	  IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_03_FLEXPWM2_PWMB03,0);   
    
	  //配置IO引脚GPIO_SD_B1_02和GPIO_SD_B1_03的功能
	  //低转换速度,驱动能力为R0/6,速度为100Mhz，关闭开路功能，使能pull/keepr
	  //选择keeper功能，下拉100K Ohm，关闭Hyst
	  IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_02_FLEXPWM2_PWMA03,0x10B0);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_03_FLEXPWM2_PWMB03,0x10B0);
    
    //初始化PWM2 模块3的通道A和B
    PWM_GetDefaultConfig(&pwm2sm3_config);              //先初始化为默认配置
    pwm2sm3_config.clockSource=kPWM_BusClock;           //时钟源为IP BUS=IPG_CLK_ROOT=150MHz
    pwm2sm3_config.prescale=pwm_prescale;               //设置分频
    pwm2sm3_config.reloadLogic=kPWM_ReloadPwmFullCycle; //全周期更新
    pwm2sm3_config.pairOperation=kPWM_Independent;      //PMWA PWMB独立模式
	  PWM_Init(PWM2,kPWM_Module_3,&pwm2sm3_config);       //初始化PWM2模块3

    //设置PWM的两个通道
    sourceclock=CLOCK_GetFreq(kCLOCK_IpgClk);
  
    //PWMA
    pwm_ignal[0].pwmChannel=kPWM_PwmA;                  //PWM通道A
    pwm_ignal[0].level=kPWM_HighTrue;                   //高电平有效
    pwm_ignal[0].dutyCyclePercent=duty;                 //占空比

    //PWMB
    pwm_ignal[1].pwmChannel=kPWM_PwmB;                  //PWM通道B
    pwm_ignal[1].level=kPWM_HighTrue;                   //高电平有效
    pwm_ignal[1].dutyCyclePercent=duty;                 //占空比

    //设置PWM2，中央对齐模式
    PWM_SetupPwm(PWM2,kPWM_Module_3,pwm_ignal,2,kPWM_SignedEdgeAligned,fre,sourceclock);
    
    PWM_SetPwmLdok(PWM2,kPWM_Control_Module_3,true);    //设置PWM的load ok位
    PWM_StartTimer(PWM2,kPWM_Control_Module_3);         //开启定时器

}

//更新PWM2占空比
//duty:占空比
//为了精度问题 修改了PWM_UpdatePwmDutycycle  库函数原型 jiezhi320
static void PWM2_SM3_DutySet(float duty1, float duty2 ) 
{
    PWM_UpdatePwmDutycycle(PWM2,kPWM_Module_3,kPWM_PwmA,kPWM_SignedEdgeAligned,duty1); //更新PWMA占空比
    PWM_UpdatePwmDutycycle(PWM2,kPWM_Module_3,kPWM_PwmB,kPWM_SignedEdgeAligned,duty2); //更新PWMB占空比
    PWM_SetPwmLdok(PWM2,kPWM_Control_Module_3,true);    //设置PWM的load ok位
}

//400hz
void pwm_init(void)
{
		PWM1_SM3_PWMAB_Init(7,MOTORS_PWM_FREQ,0);  //pwm_init();
		PWM2_SM3_PWMAB_Init(7,MOTORS_PWM_FREQ,0);
}

void pwm_out(uint16_t pwm1,uint16_t pwm2,uint16_t pwm3,uint16_t pwm4)
{
	  float v1 = (float)pwm1/25.0f;//  /2500*100
	  float v2 = (float)pwm2/25.0f;
	  float v3 = (float)pwm3/25.0f;
	  float v4 = (float)pwm4/25.0f;	
	
      PWM1_SM3_DutySet(v1,v2);
	  PWM2_SM3_DutySet(v3,v4);
}	

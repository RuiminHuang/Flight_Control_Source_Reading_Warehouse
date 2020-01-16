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
static pwm_config_t pwm2sm3_config;    //PWM2ģ��3���ýṹ��

//����XBARA1���ź����ӹ�ϵ
//input: XBARA_INnѡ��,��8λ��С,��Χ:0~87,�μ�xbar_input_signal_t��ö��ֵ.
//output: XBARA_OUTx���,��8λ��Ч,��Χ:0~130, �μ�xbar_output_signal_t��ö��ֵ.
//�ú�������XBARA1_OUTx(output)��Ӧ�������ź����ĸ�XBARA1_INn(input).
//��ϸ�Ķ�Ӧ��,��:<<RT1052Ӣ�Ĳο��ֲ�>>Table 3-5 �� Table 3-6
static void XBARA1_Signal_Set(xbar_input_signal_t input, xbar_output_signal_t output)
{ 
	uint8_t outx,inx;
  
	outx=output&0XFF;									//�õ�������XBARA_OUT���.
	inx=input&0XFF;										//�õ�������XBARA_IN���. 
	if(outx>3&&outx<20)IOMUXC_GPR->GPR6|=1<<(12+outx);	//GPIO�����,�����ö�Ӧ��I0Ϊ���ģʽ
	if(inx>3&&inx<20)IOMUXC_GPR->GPR6&=~(1<<(12+inx));	//GPIO������,�����ö�Ӧ��IOλ����ģʽ 
    
    XBARA_Init(XBARA1);                                 //��ʼ��XBARA1
    XBARA_SetSignalsConnection(XBARA1,input,output);    //����������������
}

//��ʼ��FLEXPWM,PWM2��ģ��3��ͨ��A��B
//ͨ��PWM2��ģ��3��ͨ��A��B�������2·PWM���.
//����GPIO3_IO02��GPIO3_IO03�������PWM.
//psc : Ԥ��Ƶ��,0~7,��ʾ2^psc��Ƶ.
//fre : Ƶ��
//duty:ռ�ձ�
static void PWM1_SM3_PWMAB_Init(uint16_t psc,uint32_t fre,uint8_t duty)
{
    uint32_t sourceclock;
    pwm_signal_param_t pwm_ignal[2];
    pwm_clock_prescale_t pwm_prescale=(pwm_clock_prescale_t)psc; //��Ƶ
	
    XBARA1_Signal_Set(kXBARA1_InputLogicHigh,kXBARA1_OutputFlexpwm1Fault0);
    XBARA1_Signal_Set(kXBARA1_InputLogicHigh,kXBARA1_OutputFlexpwm1Fault1);
    XBARA1_Signal_Set(kXBARA1_InputLogicHigh,kXBARA1_OutputFlexpwm1234Fault2);
    XBARA1_Signal_Set(kXBARA1_InputLogicHigh,kXBARA1_OutputFlexpwm1234Fault3); 		
    
	  //IO��������
	  IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_00_FLEXPWM1_PWMA03,0);	 
	  IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_01_FLEXPWM1_PWMB03,0);   
    
	  //����IO����GPIO_SD_B1_02��GPIO_SD_B1_03�Ĺ���
	  //��ת���ٶ�,��������ΪR0/6,�ٶ�Ϊ100Mhz���رտ�·���ܣ�ʹ��pull/keepr
	  //ѡ��keeper���ܣ�����100K Ohm���ر�Hyst
	  IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_00_FLEXPWM1_PWMA03,0x10B0);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_01_FLEXPWM1_PWMB03,0x10B0);
    
    //��ʼ��PWM2 ģ��3��ͨ��A��B
    PWM_GetDefaultConfig(&pwm1sm3_config);              //�ȳ�ʼ��ΪĬ������
    pwm1sm3_config.clockSource=kPWM_BusClock;           //ʱ��ԴΪIP BUS=IPG_CLK_ROOT=150MHz
    pwm1sm3_config.prescale=pwm_prescale;               //���÷�Ƶ
    pwm1sm3_config.reloadLogic=kPWM_ReloadPwmFullCycle; //ȫ���ڸ���
    pwm1sm3_config.pairOperation=kPWM_Independent;      //PMWA PWMB����ģʽ
	  PWM_Init(PWM1,kPWM_Module_3,&pwm1sm3_config);       //��ʼ��PWM2ģ��3

    //����PWM������ͨ��
    sourceclock=CLOCK_GetFreq(kCLOCK_IpgClk);
  
    //PWMA
    pwm_ignal[0].pwmChannel=kPWM_PwmA;                  //PWMͨ��A
    pwm_ignal[0].level=kPWM_HighTrue;                   //�ߵ�ƽ��Ч
    pwm_ignal[0].dutyCyclePercent=duty;                 //ռ�ձ�

    //PWMB
    pwm_ignal[1].pwmChannel=kPWM_PwmB;                  //PWMͨ��B
    pwm_ignal[1].level=kPWM_HighTrue;                   //�ߵ�ƽ��Ч
    pwm_ignal[1].dutyCyclePercent=duty;                 //ռ�ձ�

    //����PWM2���������ģʽ
    PWM_SetupPwm(PWM1,kPWM_Module_3,pwm_ignal,2,kPWM_SignedEdgeAligned,fre,sourceclock);
    
    PWM_SetPwmLdok(PWM1,kPWM_Control_Module_3,true);    //����PWM��load okλ
    PWM_StartTimer(PWM1,kPWM_Control_Module_3);         //������ʱ��
}

//����PWM2ռ�ձ�
//duty:ռ�ձ�
//Ϊ�˾������� �޸���PWM_UpdatePwmDutycycle  �⺯��ԭ�� jiezhi320
static void PWM1_SM3_DutySet(float duty1, float duty2 ) 
{
    PWM_UpdatePwmDutycycle(PWM1,kPWM_Module_3,kPWM_PwmA,kPWM_SignedEdgeAligned,duty1); //����PWMAռ�ձ�
    PWM_UpdatePwmDutycycle(PWM1,kPWM_Module_3,kPWM_PwmB,kPWM_SignedEdgeAligned,duty2); //����PWMBռ�ձ�
    PWM_SetPwmLdok(PWM1,kPWM_Control_Module_3,true);    //����PWM��load okλ
}

//��ʼ��FLEXPWM,PWM2��ģ��3��ͨ��A��B
//ͨ��PWM2��ģ��3��ͨ��A��B�������2·PWM���.
//����GPIO3_IO02��GPIO3_IO03�������PWM.
//psc : Ԥ��Ƶ��,0~7,��ʾ2^psc��Ƶ.
//fre : Ƶ��
//duty:ռ�ձ�
static void PWM2_SM3_PWMAB_Init(uint16_t psc,uint32_t fre,uint8_t duty)
{
    uint32_t sourceclock;
    pwm_signal_param_t pwm_ignal[2];
    pwm_clock_prescale_t pwm_prescale=(pwm_clock_prescale_t)psc; //��Ƶ
	
    XBARA1_Signal_Set(kXBARA1_InputLogicHigh,kXBARA1_OutputFlexpwm2Fault0);
    XBARA1_Signal_Set(kXBARA1_InputLogicHigh,kXBARA1_OutputFlexpwm2Fault1);
    XBARA1_Signal_Set(kXBARA1_InputLogicHigh,kXBARA1_OutputFlexpwm1234Fault2);
    XBARA1_Signal_Set(kXBARA1_InputLogicHigh,kXBARA1_OutputFlexpwm1234Fault3); 		
    
	  //IO��������
	  IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_02_FLEXPWM2_PWMA03,0);	 
	  IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_03_FLEXPWM2_PWMB03,0);   
    
	  //����IO����GPIO_SD_B1_02��GPIO_SD_B1_03�Ĺ���
	  //��ת���ٶ�,��������ΪR0/6,�ٶ�Ϊ100Mhz���رտ�·���ܣ�ʹ��pull/keepr
	  //ѡ��keeper���ܣ�����100K Ohm���ر�Hyst
	  IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_02_FLEXPWM2_PWMA03,0x10B0);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_03_FLEXPWM2_PWMB03,0x10B0);
    
    //��ʼ��PWM2 ģ��3��ͨ��A��B
    PWM_GetDefaultConfig(&pwm2sm3_config);              //�ȳ�ʼ��ΪĬ������
    pwm2sm3_config.clockSource=kPWM_BusClock;           //ʱ��ԴΪIP BUS=IPG_CLK_ROOT=150MHz
    pwm2sm3_config.prescale=pwm_prescale;               //���÷�Ƶ
    pwm2sm3_config.reloadLogic=kPWM_ReloadPwmFullCycle; //ȫ���ڸ���
    pwm2sm3_config.pairOperation=kPWM_Independent;      //PMWA PWMB����ģʽ
	  PWM_Init(PWM2,kPWM_Module_3,&pwm2sm3_config);       //��ʼ��PWM2ģ��3

    //����PWM������ͨ��
    sourceclock=CLOCK_GetFreq(kCLOCK_IpgClk);
  
    //PWMA
    pwm_ignal[0].pwmChannel=kPWM_PwmA;                  //PWMͨ��A
    pwm_ignal[0].level=kPWM_HighTrue;                   //�ߵ�ƽ��Ч
    pwm_ignal[0].dutyCyclePercent=duty;                 //ռ�ձ�

    //PWMB
    pwm_ignal[1].pwmChannel=kPWM_PwmB;                  //PWMͨ��B
    pwm_ignal[1].level=kPWM_HighTrue;                   //�ߵ�ƽ��Ч
    pwm_ignal[1].dutyCyclePercent=duty;                 //ռ�ձ�

    //����PWM2���������ģʽ
    PWM_SetupPwm(PWM2,kPWM_Module_3,pwm_ignal,2,kPWM_SignedEdgeAligned,fre,sourceclock);
    
    PWM_SetPwmLdok(PWM2,kPWM_Control_Module_3,true);    //����PWM��load okλ
    PWM_StartTimer(PWM2,kPWM_Control_Module_3);         //������ʱ��

}

//����PWM2ռ�ձ�
//duty:ռ�ձ�
//Ϊ�˾������� �޸���PWM_UpdatePwmDutycycle  �⺯��ԭ�� jiezhi320
static void PWM2_SM3_DutySet(float duty1, float duty2 ) 
{
    PWM_UpdatePwmDutycycle(PWM2,kPWM_Module_3,kPWM_PwmA,kPWM_SignedEdgeAligned,duty1); //����PWMAռ�ձ�
    PWM_UpdatePwmDutycycle(PWM2,kPWM_Module_3,kPWM_PwmB,kPWM_SignedEdgeAligned,duty2); //����PWMBռ�ձ�
    PWM_SetPwmLdok(PWM2,kPWM_Control_Module_3,true);    //����PWM��load okλ
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

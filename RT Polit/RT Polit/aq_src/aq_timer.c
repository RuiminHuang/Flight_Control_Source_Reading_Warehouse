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
    2018-06-21     jiezhi320     for imx_rt1052 
		
*/
#include "board.h"
#include "fsl_common.h" 
#include "fsl_iomuxc.h" 
#include "fsl_gpio.h" 
#include "fsl_gpt.h"
#include <drivers/pin.h>
#include "aq.h"
#include "aq_timer.h"


timerStruct_t timerData;

static gpt_config_t gpt1_config;


void timerInit(void)
{
	///1875-1,10000   =1hz
  //∂® ± ±º‰=ocrx*(psc+1)/PERCLK_CLK_ROOT	=18750 000
	uint16_t psc = 13-1;//1us
	uint32_t ocrx = 0xffffffff;
	
	GPT_GetDefaultConfig(&gpt1_config);	//œ»≥ı ºªØGPT1Œ™ƒ¨»œ÷µ
	gpt1_config.clockSource=kGPT_ClockSource_Osc;	
	gpt1_config.divider=psc;	        //…Ë÷√∑÷∆µ÷µ
	GPT_Init(TIMER_TIM,&gpt1_config);
	
	//GPT_SetOutputCompareValue(TIMER_TIM,kGPT_OutputCompare_Channel1,ocrx);	    //…Ë÷√±»Ωœº∆ ˝÷µ
	//GPT_EnableInterrupts(TIMER_TIM,kGPT_OutputCompare1InterruptEnable);			// πƒ‹GPT±»ΩœÕ®µ¿1÷–∂œ
	//RT1052_NVIC_SetPriority(TIMER_IRQ_CH,4,0);									//«¿’º”≈œ»º∂4£¨◊””≈œ»º∂0
	//EnableIRQ(GPT1_IRQn);	// πƒ‹GPT1÷–∂œ
	GPT_StartTimer(TIMER_TIM);	//ø™ º∂® ±∆˜	      
}



//GPT1÷–∂œ∑˛ŒÒ∫Ø ˝
void TIMER_ISR(void) 
{
    //OCR1÷–∂œ
    if(GPT_GetStatusFlags(TIMER_TIM,kGPT_OutputCompare1Flag))
    {
        //led_toggle();
        GPT_ClearStatusFlags(TIMER_TIM,kGPT_OutputCompare1Flag);//«Â≥˝÷–∂œ±Í÷æŒª
    }
	  __DSB();	// ˝æ›Õ¨≤Ω∆¡±Œ÷∏¡Ó
}


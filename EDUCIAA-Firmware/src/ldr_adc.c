/* Copyright 2020, UTN-FRH
 * All rights reserved.
 *
 * This file is part of Workspace.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */


/*==================[inclusions]=============================================*/

#include "board.h"
#include "ldr_adc.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

static uint16_t adc_value[4];


/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/


void ldr_adc_init(void)
{
	ADC_CLOCK_SETUP_T adc;

	Chip_ADC_Init(LPC_ADC0, &adc);

	NVIC_ClearPendingIRQ(ADC0_IRQn);
	NVIC_EnableIRQ(ADC0_IRQn);
}

void ADC0_IRQHandler(void)
{
	if(Chip_ADC_ReadStatus(LPC_ADC0, ADC_CH0, ADC_DR_DONE_STAT) == SET){
		Chip_ADC_ReadValue(LPC_ADC0, ADC_CH0, &adc_value[LDR1]);
	}

	else if(Chip_ADC_ReadStatus(LPC_ADC0, ADC_CH1, ADC_DR_DONE_STAT) == SET){
		Chip_ADC_ReadValue(LPC_ADC0, ADC_CH1, &adc_value[LDR2]);
	}

	else if(Chip_ADC_ReadStatus(LPC_ADC0, ADC_CH2, ADC_DR_DONE_STAT) == SET){
		Chip_ADC_ReadValue(LPC_ADC0, ADC_CH2, &adc_value[LDR3]);
	}

	else if(Chip_ADC_ReadStatus(LPC_ADC0, ADC_CH3, ADC_DR_DONE_STAT) == SET){
		Chip_ADC_ReadValue(LPC_ADC0, ADC_CH3, &adc_value[LDR4]);
	}
}


int16_t ldr_meas(uint8_t id){

	switch(id){
		case LDR1:
			Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH0, ENABLE);
			Chip_ADC_Int_SetChannelCmd(LPC_ADC0, ADC_CH0, ENABLE);

			break;
		case LDR2:
			Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH1, ENABLE);
			Chip_ADC_Int_SetChannelCmd(LPC_ADC0, ADC_CH1, ENABLE);

			break;
		case LDR3:
			Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH2, ENABLE);
			Chip_ADC_Int_SetChannelCmd(LPC_ADC0, ADC_CH2, ENABLE);

			break;
		case LDR4:
			Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH3, ENABLE);
			Chip_ADC_Int_SetChannelCmd(LPC_ADC0, ADC_CH3, ENABLE);

			break;
		default:
			break;
	}

	//Conversion is enabled constantly
	Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, 0);

	Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH0, DISABLE);
	Chip_ADC_Int_SetChannelCmd(LPC_ADC0, ADC_CH0, DISABLE);

	Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH1, DISABLE);
	Chip_ADC_Int_SetChannelCmd(LPC_ADC0, ADC_CH1, DISABLE);

	Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH2, DISABLE);
	Chip_ADC_Int_SetChannelCmd(LPC_ADC0, ADC_CH2, DISABLE);

	Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH0, DISABLE);
	Chip_ADC_Int_SetChannelCmd(LPC_ADC0, ADC_CH2, DISABLE);

	return adc_value[id];

}


/*==================[internal functions definition]==========================*/

/* ==================[external functions definition]========================== */

/*==================[end of file]============================================*/

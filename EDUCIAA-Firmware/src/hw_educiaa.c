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
#include "hw_educiaa.h"
#include "board.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/



/*==================[external data definition]===============================*/


void hw_educiaa_init(void)
{

	/* EDU-CIAA TEC configuration */
	// Set the pin mode and function [UM:17.3] [UM:17.4.1]
	Chip_SCU_PinMux(1, 0, SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN, SCU_MODE_FUNC0);
	Chip_SCU_PinMux(1, 1, SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN, SCU_MODE_FUNC0);
	Chip_SCU_PinMux(1, 2, SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN, SCU_MODE_FUNC0);

	// Set the pin as input [UM:19.5.3.3]
	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, TEC1_PORT, TEC1_PIN);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, TEC2_PORT, TEC2_PIN);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, TEC3_PORT, TEC3_PIN);

	// Set it as the source for pin interrupt 0 (8 available) [UM:17.4.11]
	Chip_SCU_GPIOIntPinSel(0, TEC1_PORT, TEC1_PIN);
	Chip_SCU_GPIOIntPinSel(1, TEC2_PORT, TEC2_PIN);
	Chip_SCU_GPIOIntPinSel(2, TEC3_PORT, TEC3_PIN);

	// Configure the interrupt as falling edge [UM:19.5.1.1] [UM:19.5.1.6]
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH0);
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH1);
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH2);

	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH0);
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH1);
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH2);

	// Clear pending interrupts [UM:19.5.1.10]
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH0);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH1);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH2);

	/* EDU-CIAA LEDs configuration */
	Chip_SCU_PinMux(2, 0, SCU_MODE_PULLUP, SCU_MODE_FUNC4);
	Chip_SCU_PinMux(2, 1, SCU_MODE_PULLUP, SCU_MODE_FUNC4);
	Chip_SCU_PinMux(2, 2, SCU_MODE_PULLUP, SCU_MODE_FUNC4);
	Chip_SCU_PinMux(2, 10, SCU_MODE_PULLUP, SCU_MODE_FUNC0);
	Chip_SCU_PinMux(2, 11, SCU_MODE_PULLUP, SCU_MODE_FUNC0);

	// Set the pin as output
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, LED_R_PORT, LED_R_PIN);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, LED_G_PORT, LED_G_PIN);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, LED_B_PORT, LED_B_PIN);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, LED1_PORT, LED1_PIN);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, LED2_PORT, LED2_PIN);

	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, LED_R_PORT, LED_R_PIN);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, LED_G_PORT, LED_G_PIN);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, LED_B_PORT, LED_B_PIN);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, LED1_PORT, LED1_PIN);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, LED2_PORT, LED2_PIN);

	/* EDU-CIAA GPIO0 (P6_1) servo_VERTICAL */
	Chip_SCU_PinMux(6, 1, SCU_MODE_PULLUP, SCU_MODE_FUNC0);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, SERVO_V_PORT, SERVO_V_PIN);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, SERVO_V_PORT, SERVO_V_PIN);


	/* EDU-CIAA GPIO0 (P6_5) servo_HORIZONTAL */
	Chip_SCU_PinMux(6, 5, SCU_MODE_PULLUP, SCU_MODE_FUNC0);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, SERVO_H_PORT, SERVO_H_PIN);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, SERVO_H_PORT, SERVO_H_PIN);

}


/*==================[internal functions definition]==========================*/

/* ==================[external functions definition]========================== */

/*==================[end of file]============================================*/

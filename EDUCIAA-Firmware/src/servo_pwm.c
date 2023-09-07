/* Copyright 2020, TD2-UTNFRH -Team1
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

/** @brief PWM for servo motors.
 *  @details PWM signal on Board LED 0, period 1ms, duty between 10% and 90%.
 */

 /** \addtogroup PWM
 ** @{ */

/*==================[inclusions]=============================================*/

#include "board.h"
#include "hw_educiaa.h"

#include "servo_pwm.h"

/*==================[macros and definitions]=================================*/

/** @brief Macros PWM Signals
 *  @details flags to indicate start (I) and end (F) of PWM for both servos
 */
#define MATCH_I_SERVO_V 0	/**<  Start of PWM signal for Vertical servo */
#define MATCH_F_SERVO_V	1	/**<  End of PWM signal for Vertical servo */
#define MATCH_I_SERVO_H	2	/**<  Start of PWM signal for Horizontal servo */
#define MATCH_F_SERVO_H	3	/**<  End of PWM signal for Horizontal servo */

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/**
 * @brief Interpretation of degrees to duration of pulse width
 * @param[in] ang angle in degrees
 * @return nothing
 */
static uint16_t DegreeToTime(uint16_t ang);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

void servo_pwm_init(void)
{
	// Timer 1
	Chip_TIMER_Init(LPC_TIMER1);
	Chip_TIMER_PrescaleSet(LPC_TIMER1, Chip_Clock_GetRate(CLK_MX_TIMER1) / 1000000 - 1);

	// Match 0 (period)
	Chip_TIMER_MatchEnableInt(LPC_TIMER1, MATCH_I_SERVO_V);
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1, MATCH_I_SERVO_V);
	Chip_TIMER_StopOnMatchDisable(LPC_TIMER1, MATCH_I_SERVO_V);
	Chip_TIMER_SetMatch(LPC_TIMER1, MATCH_I_SERVO_V, PERIOD_SERVO_MS);

	// Match 1 (duty)
	Chip_TIMER_MatchEnableInt(LPC_TIMER1, MATCH_F_SERVO_V);
	Chip_TIMER_ResetOnMatchDisable(LPC_TIMER1, MATCH_F_SERVO_V);
	Chip_TIMER_StopOnMatchDisable(LPC_TIMER1, MATCH_F_SERVO_V);
	Chip_TIMER_SetMatch(LPC_TIMER1, MATCH_F_SERVO_V, MIN_TIME_PULSE_WIDTH_MS);

	Chip_TIMER_Reset(LPC_TIMER1);
	Chip_TIMER_Enable(LPC_TIMER1);

	NVIC_EnableIRQ(TIMER1_IRQn);


	// Timer 2
	Chip_TIMER_Init(LPC_TIMER2);
	Chip_TIMER_PrescaleSet(LPC_TIMER2, Chip_Clock_GetRate(CLK_MX_TIMER2) / 1000000 - 1);

	// Match 2 (period)
	Chip_TIMER_MatchEnableInt(LPC_TIMER2, MATCH_I_SERVO_H);
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER2, MATCH_I_SERVO_H);
	Chip_TIMER_StopOnMatchDisable(LPC_TIMER2, MATCH_I_SERVO_H);
	Chip_TIMER_SetMatch(LPC_TIMER2, MATCH_I_SERVO_H, PERIOD_SERVO_MS);

	// Match 3 (duty)
	Chip_TIMER_MatchEnableInt(LPC_TIMER2, MATCH_F_SERVO_H);
	Chip_TIMER_ResetOnMatchDisable(LPC_TIMER2, MATCH_F_SERVO_H);
	Chip_TIMER_StopOnMatchDisable(LPC_TIMER2, MATCH_F_SERVO_H);
	Chip_TIMER_SetMatch(LPC_TIMER2, MATCH_F_SERVO_H, MIN_TIME_PULSE_WIDTH_MS);

	Chip_TIMER_Reset(LPC_TIMER2);
	Chip_TIMER_Enable(LPC_TIMER2);

	NVIC_EnableIRQ(TIMER2_IRQn);
}

static uint16_t DegreeToTime(uint16_t ang)
{
	uint16_t duty;
	uint16_t value_per_div; // Value per division
	int div = MAX_DIV;

	value_per_div = ( MAX_TIME_PULSE_WIDTH_MS - MIN_TIME_PULSE_WIDTH_MS)/div;
	duty = MIN_TIME_PULSE_WIDTH_MS + ang * value_per_div;

	return duty;
}

/*==================[external functions definition]==========================*/

void TIMER1_IRQHandler(void)
{
   if (Chip_TIMER_MatchPending(LPC_TIMER1, MATCH_I_SERVO_V)) {
      Chip_TIMER_ClearMatch(LPC_TIMER1, MATCH_I_SERVO_V);
      Chip_GPIO_SetPinState(LPC_GPIO_PORT, SERVO_V_PORT, SERVO_V_PIN, 1);
    }

   if (Chip_TIMER_MatchPending(LPC_TIMER1, MATCH_F_SERVO_V)) {
      Chip_TIMER_ClearMatch(LPC_TIMER1, MATCH_F_SERVO_V);
      Chip_GPIO_SetPinState(LPC_GPIO_PORT, SERVO_V_PORT, SERVO_V_PIN, 0);
   }
}

void TIMER2_IRQHandler(void)
{
   if (Chip_TIMER_MatchPending(LPC_TIMER2, MATCH_I_SERVO_H)) {
      Chip_TIMER_ClearMatch(LPC_TIMER2, MATCH_I_SERVO_H);
      Chip_GPIO_SetPinState(LPC_GPIO_PORT, SERVO_H_PORT, SERVO_H_PIN, 1);
    }

   if (Chip_TIMER_MatchPending(LPC_TIMER2, MATCH_F_SERVO_H)) {
      Chip_TIMER_ClearMatch(LPC_TIMER2, MATCH_F_SERVO_H);
      Chip_GPIO_SetPinState(LPC_GPIO_PORT, SERVO_H_PORT, SERVO_H_PIN, 0);
   }
}

void move_servo(ID_SERVO id, uint16_t ang)
{
	uint16_t duty = MIN_TIME_PULSE_WIDTH_MS;

	if (ang >= MAX_ANG){
		ang = MAX_ANG;
	}
	else if (MIN_ANG >= ang){
		ang = MIN_ANG;
	}

	duty = DegreeToTime(ang);

	if (id  == SERVO_VERTICAL){
		Chip_TIMER_SetMatch(LPC_TIMER1, MATCH_F_SERVO_V, duty);
		Chip_TIMER_Reset(LPC_TIMER1);
		Chip_TIMER_ClearMatch(LPC_TIMER1, MATCH_F_SERVO_V);
		Chip_TIMER_ClearMatch(LPC_TIMER1, MATCH_I_SERVO_V);
	}

	else if (id == SERVO_HORIZONTAL){
		Chip_TIMER_SetMatch(LPC_TIMER2, MATCH_F_SERVO_H, duty);
		Chip_TIMER_Reset(LPC_TIMER2);
		Chip_TIMER_ClearMatch(LPC_TIMER2, MATCH_F_SERVO_H);
		Chip_TIMER_ClearMatch(LPC_TIMER2, MATCH_I_SERVO_H);
	}
}

/** @} doxygen end group definition */

/*==================[end of file]============================================*/

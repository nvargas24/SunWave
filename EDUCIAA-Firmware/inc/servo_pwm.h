/* Copyright 2020, UTN - FRH
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

/**
 * @author Team 1
 * @authors Kyanka, Santiago
 * @authors	Lopez, Ignacio
 * @authors	Vargas, Nahuel
 */

#ifndef PROJECTS_PROYECTO_FINAL_INC_SERVO_PWM_H_
#define PROJECTS_PROYECTO_FINAL_INC_SERVO_PWM_H_

/** \addtogroup PWM
 ** @{ */

/*==================[inclusions]=============================================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
/**
 * @name MACROS Timers
 */
///@{
#define MAX_TIME_PULSE_WIDTH_MS 2300
#define MIN_TIME_PULSE_WIDTH_MS 700
#define PERIOD_SERVO_MS 20000
///@}

/**
 * @name MACROS Max and min angles
 */
///@{
#define MAX_DIV 180
#define MAX_ANG 180
#define MIN_ANG 0
///@}

/**
 *@name MACRO N of servos
 */
#define MAX_SERVO 2

/*==================[typedef]================================================*/

/**
 *@brief Servo's working mode
 */
typedef enum{
	MODE_NORMAL = 0,		/**< MODE_NORMAL: only assign angle value with a division of 180 */
	MODE_CALIBRATION = 1,	/**< MODE_CALIBRATION: assign angle value and write value by UART */
	MODE_PRECISION = 2		/**< MODE_PRECISION: only assign angle value with a division of 360 */
}MODE_SERVO;

/**
 *@brief Servo's ID
 */
typedef enum{
	SERVO_VERTICAL = 0,
	SERVO_HORIZONTAL = 1
}ID_SERVO;


/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/**
 * @brief Configuration Timers
 * @return nothing
 */
void servo_pwm_init(void);

/**
 * @brief Assign angle value
 * @param[in] id handle struct FSM_SERVO
 * @param[in] ang
 * @note It must receive an servo struct
 * @return nothing
 */
void move_servo(ID_SERVO id, uint16_t ang);


/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef PROJECTS_PROYECTO_FINAL_INC_SERVO_PWM_H_ */

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

/**
 * @author Team 1
 * @authors Kyanka, Santiago
 * @authors	Lopez, Ignacio
 * @authors	Vargas, Nahuel
 */

#ifndef PROJECTS_PROYECTO_FINAL_INC_HW_EDUCIAA_H_
#define PROJECTS_PROYECTO_FINAL_INC_HW_EDUCIAA_H_

/** @brief SetUp of EDU-CIAA
 *  @details Set Hardware interruptions
 */

/** \addtogroup Hardware_EDUCIAA
 ** @{ */

/*==================[inclusions]=============================================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
/**
 * @name MACROS Push Buttons
 */
///@{
#define TEC1_PORT	0   /* TEC1: P1_0 pin -> GPIO0[4] @ FUNC0 [UM:Table 190] */
#define TEC1_PIN	4

#define TEC2_PORT	0	/* TEC2: P1_1 pin -> GPIO0[8] @ FUNC0 [UM:Table 190] */
#define TEC2_PIN	8

#define TEC3_PORT	0	/* TEC3: P1_2 pin -> GPIO0[9] @ FUNC0 [UM:Table 190] */
#define TEC3_PIN	9

#define TEC4_PORT	1	/* TEC4: P1_6 pin -> GPIO1[9] @ FUNC0 [UM:Table 190] */
#define TEC4_PIN	9
///@}

/**
 * @name MACROS LEDs
 */
///@{
#define LED_R_PORT	5   /* LED_R: P2_0 pin -> GPIO5[0] @ FUNC4 */
#define LED_R_PIN	0

#define LED_G_PORT	5	/* LED_G: P2_1 pin -> GPIO5[1] @ FUNC4 */
#define LED_G_PIN	1

#define LED_B_PORT	5   /* LED_B: P2_2 pin -> GPIO5[2] @ FUNC4 */
#define LED_B_PIN	2

#define LED1_PORT	0	/* LED1: P2_10 pin -> GPIO0[14] @ FUNC0 */
#define LED1_PIN	14

#define LED2_PORT	1	/* LED2: P2_11 pin -> GPIO1[11] @ FUNC0 */
#define LED2_PIN	11

#define LED3_PORT	1	/* LED3: P2_12 pin -> GPIO1[12] @ FUNC0 */
#define LED3_PIN	12

/**
 * @name MACROS Servos
 */
///@{
#define SERVO_V_PORT	2	/* SERVO_VERTICAL: P6_12 pin -> GPIO2[8] @ FUNC0 */ // EDUCIAA-> GPIO8
#define SERVO_V_PIN		8

#define SERVO_H_PORT	3	/* SERVO_HORIZONTAL: P6_5 pin -> GPIO3[4] @ FUNC0 */ // EDUCIAA-> GPIO2
#define SERVO_H_PIN		4

///@}
/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/**
 * @brief Interruptions
 * @details Initialization hardware with interruptions
 * @return nothing
 */
void hw_educiaa_init(void);


/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/** @} doxygen end group definition */
/*==================[end of file]============================================*/

#endif /* PROJECTS_PROYECTO_FINAL_INC_HW_EDUCIAA_H_ */

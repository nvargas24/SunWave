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


#ifndef PROJECTS_PROYECTO_FINAL_INC_LDR_ADC_H_
#define PROJECTS_PROYECTO_FINAL_INC_LDR_ADC_H_

/** @brief LDR measure to ADC
 *  @details The light measured by the LDR is sent to ADC
 */
/** \addtogroup LDR-ADC
 ** @{ */

/*==================[inclusions]=============================================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
/**
 * @brief LDR's ID
 *
 */
typedef enum{
	LDR1 = 0,
	LDR2 = 1,
	LDR3 = 2,
	LDR4 = 3,
	MAX_LDR = 4
}ID_LDR;

/**
 * @brief LDR's measurement structure
 */
typedef struct{
	float avTop;			/**<  Top LDR's average */
	float avDown;			/**<  Down LDR's average */
	float avLeft;			/**<  Left LDR's average */
	float avRight;			/**<  Right LDR's average */
	float difVertical;		/**<  Difference between top and down */
	float difHorizontal;	/**<  Difference between right and left */
}MEAS_LDR_T;

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/**
 * @brief Initialization ADC for LDRs
 * @return Nothing
 */
void ldr_adc_init(void);

/**
 * @brief Reading port ADC where ldr is connected
 * @return Measurement of ADC from ldr 1,2,3 or 4
 * 		------------------
 *		|| 	1	||	3	||
 *		------------------
 * 		||	2	||	4	||
 *		------------------
 */
int16_t ldr_meas(uint8_t n_ldr);

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/** @} doxygen end group definition */
/*==================[end of file]============================================*/

#endif /* PROJECTS_PROYECTO_FINAL_INC_LDR_ADC_H_ */

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

#ifndef PROJECTS_PROYECTO_FINAL_INC_LCD_I2C_H_
#define PROJECTS_PROYECTO_FINAL_INC_LCD_I2C_H_

/** @brief LCD Setup and control (I2C)
 */

/** \addtogroup LCD_I2C
 ** @{ */

/*==================[inclusions]=============================================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
/**
 * @name MACROS Filas y columnas
 */
///@{

#define FILA0		 		  0
#define FILA1		  		  1
#define COLUMNAS		      16
///@}
/*==================[typedef]================================================*/

/**
 * @brief LCD states
 */
typedef enum
{
	MODE_4BIT			= 0,	/**<  Set ON LCD, it is the default state*/
	INIT_LCD			= 1,	/**<  Init LCD screen modes  */
}I2C_STATES;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
/**
 * @brief pause_useg
 * @return nothing
 */
void delay_useg(uint32_t );

/**
 * @brief lcd_tick
 * @details wait send data
 * @return nothing
 */
void lcd_i2c_tick(void);

/**
 * @brief Initialization I2C
 * @details use timer and port I2C0
 * @return nothing
 */
void lcd_i2c_init(void);

/**
 * @brief Update print LCD
 * @return nothing
 */
void lcd_i2c_loop(void);

/**
 * @brief Write instruction register
 * @attention P3 bit logical 1 to ensure security PCF
 * @param[in] cmd Instruction LCD (hex)
 * @return nothing
 */
void lcd_send_inst(unsigned char );

/**
 * @brief Write data register
 * @attention P3 bit logical 1 to ensure security PCF
 * @param[in] data Character to print
 * @return nothing
 */
void lcd_send_data(unsigned char);

/**
 * @brief Write data register and position cursor
 * @param[in] str Character string
 * @param[in] row Cursor Y position
 * @param[in] col Cursor X position
 * @return nothing
 */
void lcd_send_string(char *, int, int);

/**
 * @brief Print state
 * @param[in] buf Character string
 * @param[in] row Select row
 * @param[in] col Select column
 * @return nothing
 */
void print_lcd(char *buf , uint8_t row, uint16_t col );

/**
 * @brief Configuration LCD
 * @attention Mode 4bit
 * @return nothing
 */
void init_lcd(void);

/**
 * @brief Clear displays
 * @return nothing
 */
void lcd_clear(void);

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/** @} doxygen end group definition */
/*==================[end of file]============================================*/

#endif /* PROJECTS_PROYECTO_FINAL_INC_LCD_I2C_H_ */



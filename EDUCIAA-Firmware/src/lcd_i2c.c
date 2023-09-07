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

#include "lcd_i2c.h"

#include <stdint.h>
#include "string.h"
#include "strings.h"
#include <stdlib.h>
#include <stdio.h>

/*==================[macros and definitions]=================================*/

/*
 * @brief PCF address of reading and writing to I2C
 */
#define ADDR_PCF_I2C_W		  0x27 //  (default) PCF8574  -> 0x27     PCF8574A -> 0x3F
#define ADDR_PCF_I2C_R		  0x2F

/*	Instruction LCD */
#define LCD_FUNCTION_SET_4BIT 0x28 // 0b00101000 -> 0010 N F * *   DL = 0    N = 1    F = 0

/*
 * @brief Masks to LCD instructions
 */
#define LCD_DISPLAY_OFF       0x08 // 0b00001000 -> 0000 1 D C B   D = 0     C = 0    B = 0
#define LCD_DISPLAY_ON        0x0C // 0b00001100 -> 0000 1 D C B   D = 1     C = 0    B = 0
#define LCD_DISPLAY_CLEAR     0x01 // 0b00000001
#define LCD_ENTRY_MODE_SET    0x06 // 0b00000110 -> 0000 01 I/D S  I/D = 1   S = 0

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

uint8_t lcd_state;
uint32_t usTicks;
char p_lcd[16];
int row;

/*==================[external data definition]===============================*/



void delay_useg(uint32_t useg)
{
	Chip_TIMER_SetMatch(LPC_TIMER3, 0, useg);
	Chip_TIMER_Reset(LPC_TIMER3);
	Chip_TIMER_Enable(LPC_TIMER3);

	while (!usTicks){
		__WFI();
	}
	usTicks = 0;

	Chip_TIMER_Disable(LPC_TIMER3);
}


void TIMER3_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER3, 0)) {
		usTicks ++;
	}
	Chip_TIMER_ClearMatch(LPC_TIMER3, 0);
}


void lcd_i2c_init(void)
{
	Board_I2C_Init(I2C0);
	Chip_I2C_SetClockRate(I2C0, 100000);
	Chip_I2C_SetMasterEventHandler(I2C0, Chip_I2C_EventHandlerPolling);

	Chip_TIMER_Init(LPC_TIMER3);
	Chip_TIMER_PrescaleSet(LPC_TIMER3, Chip_Clock_GetRate(CLK_MX_TIMER1) / 1000000 - 1);
	Chip_TIMER_MatchEnableInt(LPC_TIMER3, 0);
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER3, 0);
	Chip_TIMER_StopOnMatchDisable(LPC_TIMER3, 0);

	NVIC_EnableIRQ(TIMER3_IRQn);

	init_lcd();
}



void init_lcd(void)
{
	switch (lcd_state)
	{
		case MODE_4BIT:

			delay_useg(16000);     // VCC rises to 5V
			lcd_send_inst(0x30);

			delay_useg(5000);
			lcd_send_inst(0x30);

			delay_useg(100);
			lcd_send_inst(0x30);
			lcd_send_inst(0x20);

			lcd_state = INIT_LCD;

		case INIT_LCD:
			lcd_send_inst(LCD_FUNCTION_SET_4BIT);
			lcd_send_inst(LCD_DISPLAY_OFF);
			lcd_send_inst(LCD_DISPLAY_CLEAR);
			lcd_send_inst(LCD_ENTRY_MODE_SET);

		break;

		default:
			lcd_state = MODE_4BIT;
		break;
	}
}

//		P7   P6   P5   P4   P3   P2  P1   P0   => PCF8574
//		D7   D6   D5   D4   --   E   R/W  RS   => LCD

void lcd_send_inst(unsigned char cmd)
{
	char data_u, data_l;
	uint8_t data_t[4];

	data_u = (cmd&0xF0);
	data_l = ((cmd<<4)&0xF0);

	data_t[0] = data_u|0x0C;  // E = 1   R/W = 0   RS = 0
	data_t[1] = data_u|0x08;  // E = 0   R/W = 0   RS = 0
	data_t[2] = data_l|0x0C;  // E = 1   R/W = 0   RS = 0
	data_t[3] = data_l|0x08;  // E = 0   R/W = 0   RS = 0

	Chip_I2C_MasterSend(I2C0, ADDR_PCF_I2C_W, (uint8_t *)data_t, 4);
	delay_useg(100);
}

void lcd_send_data(unsigned char data)
{
	char data_u, data_l;
	uint8_t data_t[4];

	data_u = (data&0xF0);
	data_l = ((data<<4)&0xF0);

	data_t[0] = data_u|0x0D;  // E = 1   R/W = 0   RS = 1
	data_t[1] = data_u|0x09;  // E = 0   R/W = 0   RS = 1
	data_t[2] = data_l|0x0D;  // E = 1   R/W = 0   RS = 1
	data_t[3] = data_l|0x09;  // E = 0   R/W = 0   RS = 1

	Chip_I2C_MasterSend(I2C0, ADDR_PCF_I2C_W, (uint8_t *)data_t, 4);
	delay_useg(100);
}

void lcd_send_string (char *str, int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_inst(col);

	while (*str){
		lcd_send_data (*str);
		str++;
	}

}

void lcd_clear(void)
{   int j = COLUMNAS + 1;

	for(int i=0; i<j; i++) {
		lcd_send_string(" ", FILA0, i);
		lcd_send_string(" ", FILA1, i);
	}
}

void print_lcd(char *buf, uint8_t row, uint16_t col)
{
	lcd_send_inst(0x02);
	delay_useg(10000);

	lcd_send_string(buf, row, col);
	delay_useg(100000);

}


/*==================[internal functions definition]==========================*/

/* ==================[external functions definition]========================== */

/*==================[end of file]============================================*/


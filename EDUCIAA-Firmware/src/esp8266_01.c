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
#define _GNU_SOURCE
#include <stdint.h>
#include "string.h"
#include "strings.h"
#include <stdlib.h>
#include <stdio.h>

#include <ciaaUART.h>
#include <board.h>
#include <ciaaIO.h>

#include "newlib_stubs.h"
#include "esp8266_01.h"
#include "hw_educiaa.h"

/*==================[macros and definitions]=================================*/

#define BUF_L  			700
#define BUF_L_SMALL 	50

#define SSID "Digitales"
#define PSWD "td2frhg1"

#define IP_ADDRESS "192.168.43.240"  //Changes for each PC

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

static uint8_t buffer[BUF_L];
static uint8_t tx[BUF_L];
static uint8_t cmd[BUF_L];
static char cipsend[BUF_L];


static uint8_t msg_buf[BUF_L_SMALL];
int * pointer = NULL;

static uint32_t tx_len;

static uint32_t pos;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

void reset_buffer(void)
{
	pos=0;
	bzero(buffer,BUF_L);
}

/* ==================[external functions definition]========================== */


void espConnect(void)
{
	int i;

	i = uartRecv(CIAA_UART_232, buffer + pos, BUF_L - pos);
	pos += i;

	switch(esp_state){
	case RESET_ESP:

		uartSend(CIAA_UART_232,"AT+RST\r\n",8);
		esp_state = SETMODE;

		break;
	case SETMODE:
		if (memmem(buffer,BUF_L,"ready",5)) {
			reset_buffer();
			uartSend(CIAA_UART_232,"AT+CWMODE=1\r\n",13);
			esp_state = READ_BUFFER_1;
		}
		break;

	case READ_BUFFER_1:

		if (memmem(buffer, BUF_L, "OK", 2)) {
			esp_state = CONNECT_AP;
		}
		if (memmem(buffer, BUF_L, "ERROR", 5)) {
			esp_state = SETMODE;
		}
		if (memmem(buffer, BUF_L, "no change", 9)) {
			esp_state = SETMODE;
		}
		break;

	case CONNECT_AP:

		reset_buffer();
		snprintf((char*)cmd,BUF_L,"AT+CWJAP=\""SSID"\",\""PSWD"\"\r\n");  //Join to access point
		uartSend(CIAA_UART_232, cmd, strlen((char *)cmd));
		esp_state = READ_BUFFER_2;

		break;

	case READ_BUFFER_2:

		if (memmem(buffer, BUF_L, "WIFI CONNECTED", 14) &&
			memmem(buffer, BUF_L, "WIFI GOT IP", 11) &&
			memmem(buffer, BUF_L, "OK", 2)) {

			esp_state = CIPSTART;
		}
		break;

	case CIPSTART:
		reset_buffer();
		snprintf((char*)cmd, BUF_L, "AT+CIPSTART=\"UDP\",\""IP_ADDRESS"\",8000,1112,0\r\n");
		uartSend(CIAA_UART_232, cmd, strlen((char *)cmd));
		esp_state = READ_BUFFER_3;

		break;

	case READ_BUFFER_3:

		if (memmem(buffer, BUF_L, "CONNECT", 7)) {

			reset_buffer();
			esp_state = SEND_DATA_LENGTH;

		}
		break;

	case SEND_DATA_LENGTH:

		if (memmem(buffer,BUF_L,"SEND OK",7)) {
			reset_buffer();
		}
		if (tx_len) {
			reset_buffer();
			snprintf(cipsend, BUF_L, "AT+CIPSEND=%lu\r\n", tx_len);
			uartSend(CIAA_UART_232, (uint8_t *)cipsend, strlen((char*)cmd));
			esp_state = SEND_DATA;
		}
		break;

	case SEND_DATA:

		if (memchr(buffer, '>', BUF_L)) {
			if (uartSend(CIAA_UART_232, tx, tx_len)){
				reset_buffer();
				tx_len = 0;
				esp_state = FINISH_MSJ;
			}
		}
		break;

	default:
		break;
	}

}

int32_t esp_uart_init(void)
{
	ciaaUARTInit();
	reset_buffer();

	bzero(cipsend, BUF_L);
	bzero(cmd, BUF_L);
	bzero(tx, BUF_L);
	bzero(msg_buf,BUF_L_SMALL);
	esp_state = RESET_ESP;

	return 0;
}

int32_t esp_command(char * buf, size_t l)
{
	int32_t rv= -1;

	if (tx_len == 0) {
		memcpy(tx, buf, l);
		tx_len = l;
		rv = l;
	}

	return rv;
}

int32_t esp_receive(void*  buf, size_t l)
{
	int32_t rv = -1;

	if (pointer != NULL) {
		memcpy(buf,msg_buf,l);
		rv = strlen((char*) pointer);
		pointer = NULL;
		bzero(msg_buf,BUF_L_SMALL);
	}

	return rv;
}
/*==================[end of file]============================================*/


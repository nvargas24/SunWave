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

#ifndef PROJECTS_PROYECTO_FINAL_INC_ESP8266_01_H_
#define PROJECTS_PROYECTO_FINAL_INC_ESP8266_01_H_

/** @brief ESP commands and SetUp
 */

/** \addtogroup ESP8266_01
 ** @{ */

/*==================[inclusions]=============================================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

#define TIMEOUT  500	/**<  MACRO: timeout to send data*/


/*==================[typedef]================================================*/

/**
 * @brief ESP AT status enumeration
 */
typedef enum
{
	ESP_BUF_FULL    = -3,	/**<  Indicates full buffer */
	ESP_TIMEOUT     = -2,	/**<  Reach timeout */
	ESP_ERROR       = -1,	/**<  Indicates some error occurs */
	ESP_OK          =  0,	/**<  Indicates all is right */
	ESP_NO_CHANGE   =  1,	/**<  No change has occurred */
	ESP_READY       =  2	/**<  Indicates ESP is ready */
}espStatus_e;

/**
 * @brief ESP mode enumeration
 */
typedef enum
{
	ESP_STATION     = 1,	/**<  Station mode (client) */
	ESP_AP          = 2,	/**<  AP mode (host) */
	ESP_AP_STATION  = 3		/**<  AP + Station mode (dual) */
}espMode_e;

/**
 * @brief ESP states
 */
typedef enum
{
    RESET_ESP           = 0,	/**<  Reset ESP module */
    SETMODE				= 1,	/**<  Send AT command to set mode */
    READ_BUFFER_1		= 2,	/**<  Read buffer until OK is received */
    CONNECT_AP			= 3,	/**<  Use password and id to join access point */
    READ_BUFFER_2		= 4,	/**<  Read buffer until get correct strings */
	CIPSTART			= 5,	/**<  Set UDP connection */
	READ_BUFFER_3		= 6,	/**<  Read buffer until get correct string */
	SEND_DATA_LENGTH	= 7,	/**<  Send command lenght */
	SEND_DATA 			= 8,	/**<  Send command or message */
	FINISH_MSJ			= 9

}ESP_FSM_STATES;
/*==================[external data declaration]==============================*/

uint8_t esp_state;		/**<  ESP actual state flag */
uint32_t esp_mstimeout;	/**<  timeout on milliseconds */

/*==================[external functions declaration]=========================*/

/**
 * @brief connection of esp_8266
 * @details send command AT
 *
 * @bug Join to access point
 * @attention SSID: Digitales
 * @attention PSWD: td2frhg1
 *
 * @bug Connection
 * @attention Type:	UDP
 * @attention IP: 	192.168.43.82
 *
 * @return nothing
 */
void espConnect(void);

/**
 * @brief esp_8266 tick
 * @details wait for connection
 * @attention TIMEOUT 10seg
 * @return nothing
 */
void esp_tick(void);

/**
 * @brief initialization ESP
 * @details clean and initialization UART232
 * @return 0
 */
int32_t esp_uart_init(void);

/**
 * @brief Key pressed
 * @param[in] buf Character string
 * @param[in] l Long string
 * @return rv
 */
int32_t esp_command(char * , size_t );

/**
 * @brief Reception message
 * @attention  Only one message can be stored in msg_buf
 * @param[in] buf Stores character string
 * @param[in] l Long string
 * @return rv
 */
int32_t esp_receive(void * , size_t );

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* PROJECTS_PROYECTO_FINAL_INC_ESP8266_01_H_ */

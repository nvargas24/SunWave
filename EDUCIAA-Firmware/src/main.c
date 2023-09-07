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

/** @brief Solar Tracker - TD2's project
 * @details Proyecto desarrollado como trabajo final para la materia de
 * Tecnicas Digitales II en la Universidad Tecnologica Nacional Facultad Regional Haedo.
 * El mismo consta de un seguidor solar con el fin de aprovechar la máxima energía solar
 * y almacenar la misma en un sistema de alimentación. Para esto utiliza distintos módulos
 * electrónicos, mecánicos y una estructura fabricada en PLA.
 */

/** \addtogroup Main
 ** @{ */
/*==================[inclusions]=============================================*/

#include "board.h"

#include <stdint.h>
#include "string.h"
#include "strings.h"
#include <stdlib.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "ciaaUART.h"
#include "ciaaIO.h"

#include "main.h"
#include "hw_educiaa.h"
#include "ldr_adc.h"
#include "servo_pwm.h"
#include "lcd_i2c.h"
#include "esp8266_01.h"
#include "sd_spi.h"

/*==================[macros and definitions]=================================*/

#define TOL_DIF 200 /**<  MACRO: tolerance for LDR*/

/*==================[internal data declaration]==============================*/

/**
 * @brief LDR measurement data structure definitions
 */
typedef struct{
	uint16_t lux[MAX_LDR]; 			/**<  light measured value from [LDR]*/
	MEAS_LDR_T prom;				/**<  current LDR values structure*/
	float luxIntensity[MAX_LDR];	/**<  auxiliary variable for light intensity from [LDR]*/
}queue_data_t;

/*==================[internal functions declaration]=========================*/

static xQueueHandle queue_data;
static xQueueHandle queue_servo;
TaskHandle_t xHandle;
TaskHandle_t xLcdHandle;
TaskHandle_t xEspHandle;

/**
 * @brief	Send message or command to ESP
 * @param	msj			: char pointer as string
 * @return	Nothing
 */
static void send_message_esp(char *msj);

/**
 * @brief	Initializes hardware.
 * @return	Nothing
 */
static void init_hw(void);

/**
 * @brief	Servo stabilization
 * @param	id			: Servo ID (0 for azimuth, 1 for zenit)
 * @param	newAngle	: Value of the new angle for servo
 * @param	id			: Measure from LDR
 * @return	new angle to move the servo
 */
static uint32_t estabilize_servo(ID_SERVO id, uint16_t newAngle, MEAS_LDR_T ldrMeasure);

/**
 * @brief	Convert LDR data to percentage
 * @param	data		: value to convert
 * @return	light intensity in percentage
 */
static float DataToPercentLight(uint16_t data);

/*==================[internal functions definition]===============================*/

static void send_message_esp(char *msj)
{
	while ( (esp_mstimeout < TIMEOUT) && (esp_state != FINISH_MSJ)  ){
		esp_command(msj, strlen(msj));
		espConnect();
		vTaskDelay(1 / portTICK_RATE_MS);
		esp_mstimeout ++;
	}
	esp_state = CIPSTART;
	esp_mstimeout = 0;
}

static uint32_t estabilize_servo(ID_SERVO id, uint16_t newAngle, MEAS_LDR_T ldrMeasure)
{
	if (id == SERVO_HORIZONTAL){
		if (ldrMeasure.difHorizontal >= TOL_DIF || ldrMeasure.difHorizontal <=-TOL_DIF){
			if (ldrMeasure.avLeft > ldrMeasure.avRight){
				newAngle --;
			}
			else if (ldrMeasure.avLeft < ldrMeasure.avRight){
				newAngle ++;
			}
		}
	}

	else if (id == SERVO_VERTICAL){
		if (ldrMeasure.difVertical >= TOL_DIF || ldrMeasure.difVertical <=-TOL_DIF){
			if (ldrMeasure.avTop > ldrMeasure.avDown){
				newAngle ++;
			}
			else if (ldrMeasure.avTop < ldrMeasure.avDown){
				newAngle --;
			}
		}
	}

	if (newAngle > MAX_ANG){
		newAngle = MAX_ANG;
	}
	else if (newAngle < MIN_ANG){
		newAngle = MIN_ANG;
	}

	return newAngle;
}

static float DataToPercentLight(uint16_t data)
{
	float light_percent;
	light_percent = (1-(data/ 1024.0)) * 100.0;
	return light_percent;
}


static void init_hw(void)
{
	Board_Init();
	SystemCoreClockUpdate();

	hw_educiaa_init();
	ldr_adc_init();
	servo_pwm_init();
	esp_uart_init();
	lcd_i2c_init();
	sd_spi_init();
}
/*==================[external data definition]===============================*/

/* ==================[external functions definition]========================== */

/** @brief Task that wait for data queuefor move servo after stabilize them
 *  based on LDR's measures
 * @param pointer without use
 */
void taskServo(void *pointer)
{
	queue_data_t meas;
	uint16_t ang[MAX_SERVO]= {0, 0};

	while (1){
		xQueueReceive(queue_data, &meas, 100 / portTICK_RATE_MS);

		ang[SERVO_HORIZONTAL] = estabilize_servo(SERVO_HORIZONTAL, ang[SERVO_HORIZONTAL], meas.prom);
		ang[SERVO_VERTICAL] = estabilize_servo(SERVO_VERTICAL, ang[SERVO_VERTICAL], meas.prom);

		xQueueSend(queue_servo, ang, 10 / portTICK_RATE_MS);

		move_servo(SERVO_VERTICAL, ang[SERVO_VERTICAL]);
		vTaskDelay(22 / portTICK_RATE_MS);
		move_servo(SERVO_HORIZONTAL, ang[SERVO_HORIZONTAL]);
		vTaskDelay(22 / portTICK_RATE_MS);
	}
}

/** @brief Task that wait for data queue and send information
 * of LDR's values to ESP
 *  @param pointer without use
 */
void taskEsp(void *pointer)
{
	queue_data_t meas;
	uint16_t ang[MAX_SERVO] = {0, 0};
	char msj[80];

	while (1){
		xQueueReceive(queue_data, &meas, 1000 / portTICK_RATE_MS);
		xQueueReceive(queue_servo, ang, 1000 / portTICK_RATE_MS);

		snprintf(msj, sizeof(msj), "\n SERVO VERTICAL:%u\r\n SERVO HORIZONTAL:%u\r\n\n",
					ang[SERVO_VERTICAL], ang[SERVO_HORIZONTAL]);
			send_message_esp(msj);

		bzero(msj, sizeof(msj));
		vTaskDelay(1 / portTICK_RATE_MS);

		snprintf(msj, sizeof(msj), "\nINTENSIDAD LUMINICA EN %%:\n ldr1:%4.2f%%\n ldr2:%4.2f%%\n ldr3:%4.2f%%\n ldr4:%4.2f%%\n\n",
				meas.luxIntensity[LDR1], meas.luxIntensity[LDR2], meas.luxIntensity[LDR3], meas.luxIntensity[LDR4]);
		send_message_esp(msj);

		bzero(msj, sizeof(msj));
		vTaskDelay(1 / portTICK_RATE_MS);

		snprintf(msj, sizeof(msj), "\nEN BITS:\n ldr1:%u\n ldr2:%u\n ldr3:%u\n ldr4:%u\n\n",
				meas.lux[LDR1], meas.lux[LDR2], meas.lux[LDR3], meas.lux[LDR4]);
		send_message_esp(msj);

		vTaskDelay(100 / portTICK_RATE_MS);
	}
}

/** @brief Task that wait for data queue for send information
 * of LDR's values to LCD screen
 *  @param pointer without use
 */
void taskLcd(void *pointer)
{
	queue_data_t meas;
	uint16_t ang[MAX_SERVO]={0, 0};
	char msj[2][16];

	while (1){
		xQueueReceive(queue_data, &meas, 1000 / portTICK_RATE_MS);
		xQueueReceive(queue_servo, ang, 1000 / portTICK_RATE_MS);

		snprintf(msj[0], 16, "Vertical  :%u",ang[SERVO_VERTICAL]);
		snprintf(msj[1], 16, "Horizontal:%u",ang[SERVO_HORIZONTAL]);

		print_lcd(msj[0], FILA0, 0);
		print_lcd(msj[1], FILA1, 0);

		vTaskDelay(500 / portTICK_RATE_MS);
		lcd_clear();

		bzero(msj[0], 16);
		bzero(msj[1], 16);

		snprintf(msj[0], 16, "LDR1:%4.2f%%",meas.luxIntensity[LDR1]);
		snprintf(msj[1], 16, "LDR2:%4.2f%%",meas.luxIntensity[LDR2]);

		print_lcd(msj[0], FILA0, 2);
		print_lcd(msj[1], FILA1, 2);

		vTaskDelay(500 / portTICK_RATE_MS);
		lcd_clear();

		snprintf(msj[0], 16, "LDR3:%4.2f%%",meas.luxIntensity[LDR3]);
		snprintf(msj[1], 16, "LDR4:%4.2f%%",meas.luxIntensity[LDR4]);

		print_lcd(msj[0], FILA0, 2);
		print_lcd(msj[1], FILA1, 2);

		vTaskDelay(500 / portTICK_RATE_MS);
		lcd_clear();
	}
}

/** @brief Task that wait for data queue to write information on a SD card
 *  @param pointer without use
 */
void taskSd(void *pointer)
{
	queue_data_t meas;
	uint16_t ang[MAX_SERVO]={0, 0};
	char msj[50];

	send_sd("\nLDR1[%]\t LDR2[%]\t LDR3[%]\t LDR4[%]\t LDR1\t LDR2\t LDR3\t LDR4\t S.Vertical\t S.Horizontal\n", 88);
	vTaskDelay(10 / portTICK_RATE_MS);

	while (1){
		xQueueReceive(queue_data, &meas, portMAX_DELAY);
		xQueueReceive(queue_servo, ang, portMAX_DELAY);

		snprintf(msj, sizeof(msj), "%3.2f\t %3.2f\t %3.2f\t %3.2f\t",
					meas.luxIntensity[LDR1], meas.luxIntensity[LDR2], meas.luxIntensity[LDR3], meas.luxIntensity[LDR4]);
		vTaskDelay(10 / portTICK_RATE_MS);
		send_sd(msj,27);

		snprintf(msj, sizeof(msj), "%4u\t %4u\t %4u\t %4u\t",
					meas.lux[LDR1], meas.lux[LDR2], meas.lux[LDR3], meas.lux[LDR4]);
		send_sd(msj,23);

		snprintf(msj, 15, "%5u\t %5u\r\n", ang[SERVO_VERTICAL], ang[SERVO_HORIZONTAL]);
		send_sd(msj,15);

		Board_LED_Set(LEDB, 1);
		vTaskDelay(100 / portTICK_RATE_MS);
		Board_LED_Set(LEDB, 0);
	}
}

/** @brief Task for CIAA buttons control to extract SD with security
 *  @param pointer without use
 */
void taskTecSd(void *pointer)
{
	uint16_t cont_ms_push = 0;
	bool est = 1;

	while(1){

		while( Buttons_GetStatus() & (1 << TEC1) ){
			vTaskDelay(1 / portTICK_RATE_MS);
			cont_ms_push ++;
		}

		if(cont_ms_push > 1000){
			est = !est;
			if(est == 0){
				Board_LED_Set(LEDB, 0);
				vTaskSuspend(xHandle);
				Board_LED_Set(LEDR, 1);
				vTaskDelay(1000 / portTICK_RATE_MS);
				Board_LED_Set(LEDR, 0);
			}
			else{
				vTaskResume(xHandle);
			}
			cont_ms_push = 0;
		}
	}
}

/** @brief Task for CIAA buttons and LEDs control to select modules
 *  @param pointer without use
 */
void taskTecSelec(void *pointer)
{
	uint16_t cont_push = 1;

	while(1){

		if( Buttons_GetStatus() & (1 << TEC2) ){
			vTaskDelay(1 / portTICK_RATE_MS);
			cont_push ++;
			if(cont_push >= 3){
				cont_push = 0;
			}
		}

		switch (cont_push) {
			case 0:
				vTaskSuspend(xLcdHandle);
				vTaskResume(xEspHandle);
				Board_LED_Set(LED1, 1);
				break;
			case 1:
				vTaskSuspend(xEspHandle);
				vTaskResume(xLcdHandle);
				Board_LED_Set(LED2, 1);
				break;
			case 2:
				vTaskResume(xLcdHandle);
				vTaskResume(xEspHandle);
				Board_LED_Set(LED3, 1);
				break;

			default:
				break;
		}

		vTaskDelay(100 / portTICK_RATE_MS);
		Board_LED_Set(LED1, 0);
		Board_LED_Set(LED2, 0);
		Board_LED_Set(LED3, 0);
	}
}

/** @brief Task for read LDR measurements
 *  @param pointer without use
 */
void taskLdr(void *pointer)
{
	queue_data_t meas;

	while (1){

		for (uint8_t i = LDR1; i < MAX_LDR; i++){
			meas.lux[i] = ldr_meas(i);
			vTaskDelay(1 / portTICK_RATE_MS);
		}

		meas.prom.avTop = (meas.lux[LDR1] + meas.lux[LDR3]) / 2.0;
		meas.prom.avDown = ( meas.lux[LDR2] + meas.lux[LDR4] ) /2.0;
		meas.prom.avRight = ( meas.lux[LDR3] + meas.lux[LDR4] ) /2.0;
		meas.prom.avLeft = ( meas.lux[LDR1] + meas.lux[LDR2] ) /2.0;

		/* tolerancia antirrebote */
		meas.prom.difVertical = meas.prom.avTop - meas.prom.avDown;
		meas.prom.difHorizontal = meas.prom.avLeft -meas.prom.avRight;

		for (uint8_t i = LDR1; i < MAX_LDR; i++){
			meas.luxIntensity[i] = DataToPercentLight(meas.lux[i]);
			vTaskDelay(1 / portTICK_RATE_MS);
		}

		xQueueSend(queue_data, &meas, 0);

		vTaskDelay(10 / portTICK_RATE_MS);
	}
}


int main(void)
{
	init_hw();

	queue_data = xQueueCreate(5, sizeof(queue_data_t) );
	queue_servo = xQueueCreate(5, 16);

	xTaskCreate(taskServo, (const char *)"servo",
			configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);

	xTaskCreate(taskEsp, (const char *)"esp_uart",
			configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, &xEspHandle);

	xTaskCreate(taskLdr, (const char *)"ldr",
			configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);

	xTaskCreate(taskLcd, (const char *)"lcd",
			configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, &xLcdHandle);

	xTaskCreate(taskSd, (const char *)"sd",
			configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, &xHandle);

	xTaskCreate(taskTecSd, (const char *)"tecSd",
			configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);

	xTaskCreate(taskTecSelec, (const char *)"tecSelect",
			configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);


	vTaskStartScheduler();

	while (1) {

	}
	return 0;
}
/** @} doxygen end group definition */

/*==================[end of file]============================================*/


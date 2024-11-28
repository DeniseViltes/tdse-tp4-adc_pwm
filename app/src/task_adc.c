/*
 * Copyright (c) 2023 Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file   : task_adc.c
 * @date   : Set 26, 2023
 * @author : Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>
 * @version	v1.0.0
 */

/********************** inclusions *******************************************/
/* Project includes. */
#include "main.h"

/* Demo includes. */
#include "logger.h"
#include "dwt.h"

/* Application & Tasks includes. */
#include "board.h"
#include "app.h"
#include "task_interface.h"

/********************** macros and definitions *******************************/

#define SAMPLES_COUNTER (10)

/********************** internal data declaration ****************************/
uint32_t tickstart;
uint16_t sample_idx;

uint16_t sample_array[SAMPLES_COUNTER];
bool b_trig_new_conversion;
uint16_t promedio(uint16_t sample_array[SAMPLES_COUNTER]);

/********************** internal functions definitions ***********************/

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef ADC_Poll_Read(uint16_t *value);
bool test3_tick();
uint16_t obtener_lecturas(uint16_t *lecturas);
/********************** internal data definition *****************************/
const char *p_task_adc 		= "Task ADC";

/********************** external data declaration *****************************/
extern ADC_HandleTypeDef hadc1;


/********************** external functions definition ************************/

void task_adc_init(void *parameters)
{
	/* Print out: Task Initialized */
	LOGGER_LOG("  %s is running - %s\r\n", GET_NAME(task_adc_init), p_task_adc);
	HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
	sample_idx = 0;
	tickstart = HAL_GetTick();
}

void task_adc_update(void *parameters)
{
	static bool b_test_done = false;
	if (!b_test_done){
		b_test_done = test3_tick();
	}
}


bool test3_tick() {

	bool b_done = false;

	if (sample_idx>=SAMPLES_COUNTER) {
		b_done = true;
		goto test3_tick_end;
	}

	/* start of first conversion */
	if (0==sample_idx) {
		b_trig_new_conversion = true;
	}


	if (b_trig_new_conversion) {
		b_trig_new_conversion = false;
		HAL_ADC_Start_IT(&hadc1);
	}

test3_tick_end:
	if (b_done) {
		/*for (sample_idx=0;sample_idx<SAMPLES_COUNTER;sample_idx++) {
			LOGGER_LOG("%u\n",sample_array[sample_idx] );
		}*/
		uint16_t prom = promedio(sample_array);
		cargar_valor_pote(prom);
	}
	return b_done;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

	sample_array[sample_idx++] = HAL_ADC_GetValue(&hadc1);
	if (sample_idx<SAMPLES_COUNTER) {
		b_trig_new_conversion = true;
	}
}


HAL_StatusTypeDef ADC_Poll_Read(uint16_t *value) {
	HAL_StatusTypeDef res;

	res=HAL_ADC_Start(&hadc1);
	if ( HAL_OK==res ) {
		res=HAL_ADC_PollForConversion(&hadc1, 0);
		if ( HAL_OK==res ) {
			*value = HAL_ADC_GetValue(&hadc1);
		}
	}
	return res;
}

uint16_t promedio(uint16_t sample_array[SAMPLES_COUNTER]){
	uint16_t averaged = 0;
	for(uint16_t averager=0 ; averager<SAMPLES_COUNTER ; averager++){
		averaged += sample_array[averager];
	}
	averaged = averaged / SAMPLES_COUNTER;
	return averaged;
}
/********************** end of file ******************************************/

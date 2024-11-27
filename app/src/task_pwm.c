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
 * @file   : task_pwm.c
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

/********************** macros and definitions *******************************/

/********************** internal data declaration ****************************/

/********************** internal functions definition ***********************/
void test2_tick();
void setPWM(TIM_HandleTypeDef timer,
            uint32_t channel,
            uint16_t period,
            uint16_t pulse);

/********************** internal data definition *****************************/
const char *p_task_pwm 		= "Task PWM";

/********************** external data declaration *****************************/


/********************** external functions definition ************************/
void task_pwm_init(void *parameters)
{
	/* Print out: Task Initialized */
	LOGGER_LOG("  %s is running - %s\r\n", GET_NAME(task_pwm_init), p_task_pwm);
}

void task_pwm_update(void *parameters)
{
	test2_tick();
}


void test2_tick() {

	static uint16_t period=PERIOD/4;
	static bool first = true;
	static uint32_t tick;
	static int16_t step = STEP;

	if (first) {
		first = false;
		tick = HAL_GetTick() + DELAY_TICKS;
	}

	if (HAL_GetTick()>=tick) {
		setPWM(htim3, TIM_CHANNEL_1, period, period/2);
		if (step>0) {
			if (period-step>=PERIOD/2) {
				step = step * -1;
			}
		}
		else {
			if (period-step<=PERIOD/4) {
				step = step * -1;
			}
		}
		period = period + step;
		tick = HAL_GetTick() + DELAY_TICKS;
	}
}


void setPWM(TIM_HandleTypeDef timer,
            uint32_t channel,
            uint16_t period,
            uint16_t pulse) {
  HAL_TIM_PWM_Stop(&timer, channel);
  TIM_OC_InitTypeDef sConfigOC;
  timer.Init.Period = period;
  HAL_TIM_PWM_Init(&timer);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);

  HAL_TIM_PWM_Start(&timer,channel);
}



/********************** end of file ******************************************/

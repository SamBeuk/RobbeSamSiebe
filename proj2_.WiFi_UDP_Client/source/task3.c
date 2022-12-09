
/*
 * task1.c
 *
 *  Created on: 27 okt. 2022
 *      Author: 20002890
 */

/* Header file includes. */
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <inttypes.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "cy_pdl.h"
#include "mtb_thermistor_ntc_gpio.h"


/* FreeRTOS header file. */
#include "http_client.h"
#include "task3.h"




/* globals */
mtb_thermistor_ntc_gpio_t mtb_thermistor_obj;
QueueHandle_t temp_command_q;

cyhal_adc_t adc_obj;

int temperature(void);

mtb_thermistor_ntc_gpio_cfg_t therm_gpio_cfg =
	{
		.b_const = (float)(3380), // refers to beta constant
		/* Resistance of the thermistor is 10K at 25 degrees C (from datasheet)
		 * Therefore R0 = 10000 Ohm, and T0 = 298.15 Kelvin, which gives
		 * R_INFINITY = R0 e^(-B_CONSTANT / T0) = 0.1192855
		 */
		.r_infinity = (float)(0.1192855), // refers to resistance at infinity, ideally 0
		.r_ref = (float)(10000)			  // refres to reference resistance
};

void task3(void *arg)
{
	uint32_t thermistorData;
	BaseType_t rtos_api_result;

	cy_rslt_t result;

	result = cyhal_adc_init(&adc_obj, THERM_OUT, NULL);

	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}
	result = mtb_thermistor_ntc_gpio_init(&mtb_thermistor_obj, &adc_obj, THERM_GND, THERM_VDD, THERM_OUT, &therm_gpio_cfg, MTB_THERMISTOR_NTC_WIRING_VIN_R_NTC_GND);

	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

	for (;;)
	{
		rtos_api_result = xQueueReceive(temp_command_q, &thermistorData,
										portMAX_DELAY);
		if (rtos_api_result == pdTRUE)
		{
			thermistorData = temperature();
			xQueueSendToBack(http_message_q, &thermistorData, 0u);
			vTaskDelay(1000);
		}
	}
}

int temperature(void)
{
	float value;
	float sum = 0;
	int sumInt;
	uint8_t loop;
	for (loop = 0; loop < 10; loop++)
	{
		value = mtb_thermistor_ntc_gpio_get_temp(&mtb_thermistor_obj);
		sum = sum + value;
	}
	sumInt = (int)(sum / 10);
	return sumInt;
}

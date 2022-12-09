/*
 * pdm_mic.c
 *
 *  Created on: 17 Nov 2022
 *      Author: beuks
 */


#include "cybsp.h"
#include "cyhal_clock.h"
#include "cyhal.h"
#include "cy_retarget_io.h"
#include "stdlib.h"
#include "mic.h"
#include "queue.h"
#include <inttypes.h>
/* FreeRTOS header file. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "http_client.h"


/*******************************************************************************
 * Macros
 ********************************************************************************/
/* Define how many samples in a frame */
#define FRAME_SIZE (1024)
/* Noise threshold hysteresis */
#define THRESHOLD_HYSTERESIS 3u
/* Volume ratio for noise and print purposes */
#define VOLUME_RATIO (4 * FRAME_SIZE)
/* Desired sample rate. Typical values: 8/16/22.05/32/44.1/48kHz */
#define SAMPLE_RATE_HZ 8000u
/* Decimation Rate of the PDM/PCM block. Typical value is 64 */
#define DECIMATION_RATE 64u
/* Audio Subsystem Clock. Typical values depends on the desire sample rate:
- 8/16/48kHz    : 24.576 MHz
- 22.05/44.1kHz : 22.579 MHz */
#define AUDIO_SYS_CLOCK_HZ 24576000u
/* PDM/PCM Pins */
#define PDM_DATA P10_5
#define PDM_CLK P10_4

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
void button_isr_handler(void *arg, cyhal_gpio_event_t event);
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event);
void clock_init(void);
int volumeCalc(void);
/*******************************************************************************
 * Global Variables
 ********************************************************************************/
/* Interrupt flags */
volatile bool button_flag = false;
volatile bool pdm_pcm_flag = true;

QueueHandle_t mic_command_q;

/* Volume variables */
uint32_t volume = 0;
uint32_t noise_threshold = THRESHOLD_HYSTERESIS;

/* HAL Object */
cyhal_pdm_pcm_t pdm_pcm;
cyhal_clock_t audio_clock;
cyhal_clock_t pll_clock;

/* HAL Config */
const cyhal_pdm_pcm_cfg_t pdm_pcm_cfg =
	{
		.sample_rate = SAMPLE_RATE_HZ,
		.decimation_rate = DECIMATION_RATE,
		.mode = CYHAL_PDM_PCM_MODE_STEREO,
		.word_length = 16, /* bits */
		.left_gain = 0,	   /* dB */
		.right_gain = 0,   /* dB */
};

/*This structure is used to initialize callback*/
cyhal_gpio_callback_data_t cb_data =
	{
		.callback = button_isr_handler,
		.callback_arg = NULL};




/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * The main function for Cortex-M4 CPU does the following:
 *  Initialization:
 *  - Initializes all the hardware blocks
 *  Do forever loop:
 *  - Check if PDM/PCM flag is set. If yes, report the current volume
 *  - Update the LED status based on the volume and the noise threshold
 *  - Check if the User Button was pressed. If yes, reset the noise threshold
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 *******************************************************************************/
void pdm_mic_task(void *arg)
{
	cy_rslt_t result;
	int16_t audio_frame[FRAME_SIZE] = {0};
	BaseType_t rtos_api_result;
	uint32_t micData;

	/* Enable global interrupts */
	__enable_irq();

	/* Init the clocks */
	clock_init();

	/* Initialize the PDM/PCM block */
	cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
	cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_pcm_isr_handler, NULL);
	cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
	cyhal_pdm_pcm_start(&pdm_pcm);

	for (;;)
	{
		/* Check if any microphone has data to process */
		if (pdm_pcm_flag)
		{
			/* Clear the PDM/PCM flag */
			pdm_pcm_flag = 0;

			/* Reset the volume */
			volume = 0;

			/* Calculate the volume by summing the absolute value of all the
			 * audio data from a frame */
			for (uint32_t index = 0; index < FRAME_SIZE; index++)
			{
				volume += abs(audio_frame[index]);
			}

			rtos_api_result = xQueueReceive(mic_command_q, &micData,
											portMAX_DELAY);
			if (rtos_api_result == pdTRUE)
			{
				micData = volume / VOLUME_RATIO;
				xQueueSendToBack(http_message_q, &micData, 0u);
			}

			/* Setup to read the next frame */
			cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame, FRAME_SIZE);
		}

		/* Reset the noise threshold if User Button is pressed */
		if (button_flag)
		{
			/* Reset button flag */
			button_flag = false;

			/* Get the current volume and add a hysteresis as the new threshold */
			noise_threshold = (volume / VOLUME_RATIO) + THRESHOLD_HYSTERESIS;

			/* Report the new noise threshold over UART */
			printf("\n\rNoise threshold: %lu\n\r", (unsigned long)noise_threshold);
		}

		// cyhal_system_sleep();
	}

	//        cyhal_syspm_sleep();
}

/*******************************************************************************
 * Function Name: pdm_pcm_isr_handler
 ********************************************************************************
 * Summary:
 *  PDM/PCM ISR handler. Set a flag to be processed in the main loop.
 *
 * Parameters:
 *  arg: not used
 *  event: event that occurred
 *
 *******************************************************************************/
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event)
{
	(void)arg;
	(void)event;

	pdm_pcm_flag = true;
}

/*******************************************************************************
 * Function Name: clock_init
 ********************************************************************************
 * Summary:
 *  Initialize the clocks in the system.
 *
 *******************************************************************************/
void clock_init(void)
{
	/* Initialize the PLL */
	cyhal_clock_reserve(&pll_clock, &CYHAL_CLOCK_PLL[0]);
	cyhal_clock_set_frequency(&pll_clock, AUDIO_SYS_CLOCK_HZ, NULL);
	cyhal_clock_set_enabled(&pll_clock, true, true);

	/* Initialize the audio subsystem clock (CLK_HF[1])
	 * The CLK_HF[1] is the root clock for the I2S and PDM/PCM blocks */

	cyhal_clock_reserve(&audio_clock, &CYHAL_CLOCK_HF[1]);

	/* Source the audio subsystem clock from PLL */
	cyhal_clock_set_source(&audio_clock, &pll_clock);
	cyhal_clock_set_enabled(&audio_clock, true, true);
}

/* [] END OF FILE */

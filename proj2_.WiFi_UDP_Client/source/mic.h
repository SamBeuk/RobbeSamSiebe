/*
 * pdm_mic.h
 *
 *  Created on: 17 Nov 2022
 *      Author: beuks
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#ifndef SOURCE_PDM_MIC_H_
#define SOURCE_PDM_MIC_H_

void pdm_mic_task(void *arg);

/*******************************************************************************
 * Global variable
 ******************************************************************************/
extern QueueHandle_t mic_command_q;

#endif /* SOURCE_PDM_MIC_H_ */

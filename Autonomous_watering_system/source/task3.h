/*
 * task3.h
 *
 *  Created on: 27 okt. 2022
 *      Author: 20002890
 */



#ifndef TASK3_H_
#define TASK3_H_

void task3(void *arg);

#define THERM_GND P10_3
#define THERM_VDD P10_0
#define THERM_OUT P10_2

/*******************************************************************************
 * Global variable
 ******************************************************************************/
extern QueueHandle_t temp_command_q;

#endif /* TASK3_H_ */

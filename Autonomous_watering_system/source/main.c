/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC6 Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

/* Header file includes. */
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/* FreeRTOS header file. */
#include <FreeRTOS.h>
#include <task.h>

/* TCP client task header file. */
#include "http_client.h"


#include <stdio.h>
#include "queue.h"
#include "task3.h"
#include "mic.h"


/* Include serial flash library and QSPI memory configurations only for the
 * kits that require the Wi-Fi firmware to be loaded in external QSPI NOR flash.
 * VAN OPDR1

#if defined(TARGET_CY8CPROTO_062S3_4343W)
#include "cy_serial_flash_qspi.h"
#include "cycfg_qspi_memslot.h"
#endif
*/

/*******************************************************************************
* Macros
********************************************************************************/
/* RTOS related macros. */
#define HTTP_CLIENT_TASK_STACK_SIZE        (5 * 1024)
#define HTTP_CLIENT_TASK_PRIORITY          (1)

/*******************************************************************************
* Global Variables
********************************************************************************/
/* This enables RTOS aware debugging. */
volatile int uxTopUsedPriority;

/* HTTP Client task handle. */
TaskHandle_t client_task_handle;

/* UDP server task handle. */
TaskHandle_t thermistor_task_handle;

/* Queue lengths of message queues used in this project */
#define SINGLE_ELEMENT_QUEUE (1u)

#define TASK_CAPSENSE_PRIORITY (0)

/* Stack sizes of user tasks in this project */
#define TASK_CAPSENSE_STACK_SIZE (256u)

#define TASK_MIC_STACK_SIZE (1024)

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function sets up user tasks and then starts
*  the RTOS scheduler.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* This enables RTOS aware debugging in OpenOCD. */
	uxTopUsedPriority = configMAX_PRIORITIES - 1;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* To avoid compiler warnings. */
	(void) result;

	/* Enable global interrupts. */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port. */
	cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);


	/* Init QSPI and enable XIP to get the Wi-Fi firmware from the QSPI NOR flash
	 * KOMT VAN OPDR1
	 * */
	/* Init QSPI and enable XIP to get the Wi-Fi firmware from the QSPI NOR flash */
	#if defined(TARGET_CY8CPROTO_062S3_4343W)
	    const uint32_t bus_frequency = 50000000lu;
	    cy_serial_flash_qspi_init(smifMemConfigs[0], CYBSP_QSPI_D0, CYBSP_QSPI_D1,
	                              CYBSP_QSPI_D2, CYBSP_QSPI_D3, NC, NC, NC, NC,
	                              CYBSP_QSPI_SCK, CYBSP_QSPI_SS, bus_frequency);

	    cy_serial_flash_qspi_enable_xip(true);
	#endif


	/* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen. */
	printf("\x1b[2J\x1b[;H");
	printf("============================================================\r\n");
	printf("POST to Edge Impulse\r\n");
	printf("============================================================\r\n");

	/* Create the queues. See the respective data-types for details of queue
	* contents
	*/
	mic_command_q = xQueueCreate(SINGLE_ELEMENT_QUEUE,
	                                 sizeof(uint32_t));
	temp_command_q = xQueueCreate(SINGLE_ELEMENT_QUEUE,
	                                 sizeof(uint32_t));
	http_message_q = xQueueCreate(SINGLE_ELEMENT_QUEUE,
									sizeof(uint32_t));
	/* Create the client task. */
	xTaskCreate(pdm_mic_task, "mic Task", TASK_MIC_STACK_SIZE,
	                NULL, TASK_CAPSENSE_PRIORITY, NULL);
	xTaskCreate(task3, "task_3", TASK_CAPSENSE_STACK_SIZE,
	                NULL, TASK_CAPSENSE_PRIORITY, &thermistor_task_handle);
	xTaskCreate(http_client_task, "Network task", HTTP_CLIENT_TASK_STACK_SIZE, NULL, HTTP_CLIENT_TASK_PRIORITY, &client_task_handle);

	/* Start the FreeRTOS scheduler. */
	vTaskStartScheduler();

	/* Should never get here. */
	CY_ASSERT(0);
}

/* [] END OF FILE */

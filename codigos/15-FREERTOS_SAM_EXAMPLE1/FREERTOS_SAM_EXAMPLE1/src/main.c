/**
 * \file
 *
 * \brief FreeRTOS Real Time Kernel example.
 *
 * Copyright (c) 2012-2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
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
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage FreeRTOS Real Time Kernel example
 *
 * \section Purpose
 *
 * The FreeRTOS example will help users how to use FreeRTOS in SAM boards.
 * This basic application shows how to create task and get information of
 * created task.
 *
 * \section Requirements
 *
 * This package can be used with SAM boards.
 *
 * \section Description
 *
 * The demonstration program create two task, one is make LED on the board
 * blink at a fixed rate, and another is monitor status of task.
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board. Please
 *    refer to the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a>
 *    application note or to the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>,
 *    depending on your chosen solution.
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# LED should start blinking on the board. In the terminal window, the
 *    following text should appear (values depend on the board and chip used):
 *    \code
	-- Freertos Example xxx --
	-- xxxxxx-xx
	-- Compiled: xxx xx xxxx xx:xx:xx --
\endcode
 *
 */

#include <asf.h>
#include "conf_board.h"

#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define	BUT3_PIN_MASK		(1<<BUT3_PIN)

/**
 * Botão
 */ 
#define BUT_PIO_ID		ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN			11			
#define BUT_PIN_MASK	(1<<BUT_PIN)


/**
 * LEDs - placa OLED
 */

/*led 1*/
#define OLED1_PIO_ID		ID_PIOA
#define OLED1_PIO			PIOA
#define OLED1_PIN			0
#define OLED1_PIN_MASK		(1<<OLED1_PIN)
	
/*led 2*/
#define OLED2_PIO_ID		ID_PIOC
#define OLED2_PIO			PIOC
#define OLED2_PIN			30
#define OLED2_PIN_MASK		(1<<OLED2_PIN)
	
/*led 3*/
#define OLED3_PIO_ID		ID_PIOB
#define OLED3_PIO			PIOB
#define OLED3_PIN			2
#define OLED3_PIN_MASK		(1<<OLED3_PIN)

/*Inicialização semáforos*/

SemaphoreHandle_t xSemaphore1 = NULL;
SemaphoreHandle_t xSemaphore2 = NULL;
SemaphoreHandle_t xSemaphore3 = NULL;

/*Flags*/
volatile uint32_t led = 1;
volatile uint32_t but = 0;


/***********************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);
void led_init();

#if !(defined(SAMV71) || defined(SAME70))
/**
 * \brief Handler for Sytem Tick interrupt.
 */
void SysTick_Handler(void)
{
	xPortSysTickHandler();
}
#endif

/*função que realiza a mudança de status de um led*/
void toggle(Pio *p,uint32_t lmask){

	if(pio_get_output_data_status(p, lmask)){
		pio_clear(p, lmask);
	}
	else{
		pio_set(p,lmask);
	}
	
}

/*Inicialização dos 3 led da placa auxiliar OLED*/
void led_init(int estado){
	
	pmc_enable_periph_clk(OLED1_PIO_ID);
	pio_set_output(OLED1_PIO, OLED1_PIN_MASK, 1, 0, 0 );
	
	pmc_enable_periph_clk(OLED2_PIO_ID);
	pio_set_output(OLED2_PIO, OLED2_PIN_MASK, 1, 0, 0 );
	
	pmc_enable_periph_clk(OLED3_PIO_ID);
	pio_set_output(OLED3_PIO, OLED3_PIN_MASK, 1, 0, 0 );
};

void but_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT_PIO_ID);
	pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
};

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	UNUSED(pvParameters);

	for (;;) {
		printf("--- task ## %u", (unsigned int)uxTaskGetNumberOfTasks());
		vTaskList((signed portCHAR *)szList);
		printf(szList);
		vTaskDelay(1000);
	}
}

/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */
static void task_led(void *pvParameters)
{
	UNUSED(pvParameters);
	for (;;) {
	#if SAM4CM
		LED_Toggle(LED4);
	#else
		LED_Toggle(LED0);
	#endif
		vTaskDelay(1000);
	}
}

//task para o led 1 do OLED
static void task_oled1(void *pvParameters)
{
	UNUSED(pvParameters);
	xSemaphore1 = xSemaphoreCreateMutex();
	for (;;) {
		
		if(xSemaphore1 != NULL){
			if( xSemaphoreTake( xSemaphore1, ( TickType_t ) 1 ) == pdTRUE ){
				//muda o status do led
				toggle(OLED1_PIO,OLED1_PIN_MASK);
				
				/* We have finished accessing the shared resource.  Release the
				semaphore. */
				xSemaphoreGive(xSemaphore1);		
			}
			else{
				pio_set(OLED1_PIO,OLED1_PIN_MASK);
			}
			vTaskDelay(300);
		}
		
	}
}

//task para o led 2 do OLED
static void task_oled2(void *pvParameters)
{
	UNUSED(pvParameters);
	xSemaphore2 = xSemaphoreCreateMutex();
	for (;;) {
		
		if(xSemaphore2 != NULL){
			if( xSemaphoreTake( xSemaphore2, ( TickType_t ) 1 ) == pdTRUE ){
				//muda o status do led
				toggle(OLED2_PIO,OLED2_PIN_MASK);
				
				/* We have finished accessing the shared resource.  Release the
				semaphore. */
				xSemaphoreGive(xSemaphore2);		
			}
			else{
				pio_set(OLED2_PIO,OLED2_PIN_MASK);
			}
			vTaskDelay(300);
		}
		
	}
}

//task para o led 3 do OLED
static void task_oled3(void *pvParameters)
{
	UNUSED(pvParameters);
	xSemaphore3 = xSemaphoreCreateMutex();
	for (;;) {
		
		if(xSemaphore3 != NULL){
			if( xSemaphoreTake( xSemaphore3, ( TickType_t ) 1 ) == pdTRUE ){
				//muda o status do led
				toggle(OLED3_PIO,OLED3_PIN_MASK);
				
				/* We have finished accessing the shared resource.  Release the
				semaphore. */
				xSemaphoreGive(xSemaphore3);		
			}
			else{
				pio_set(OLED3_PIO,OLED3_PIN_MASK);
			}
			vTaskDelay(300);
		}
		
	}
}

static void task_but(void *pvParameters){
	
	UNUSED(pvParameters);
	for(;;){
		if((BUT_PIO->PIO_PDSR & BUT_PIN_MASK)==0){
			 if(led){
				 xSemaphoreTake(xSemaphore1, ( TickType_t ) 10);
				 xSemaphoreTake(xSemaphore2, ( TickType_t ) 10);
				 xSemaphoreTake(xSemaphore3, ( TickType_t ) 10);
				 led = 0;
			 }
			 else{
				 xSemaphoreGive(xSemaphore1);
				 xSemaphoreGive(xSemaphore2);
				 xSemaphoreGive(xSemaphore3);
				 led = 1;
			 }
			 but = 1;
		}
		else{
			but = 0;
		}
	}
}

/**
 * \brief Configure the console UART.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}

/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	led_init(1);
	but_init();

	/* Initialize the console uart */
	configure_console();

	/* Output demo infomation. */
	printf("-- Freertos Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);


	/* Create task to monitor processor activity */
	if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL,
			TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Monitor task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
			TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to make led blink */
	if (xTaskCreate(task_oled1, "Led", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to make led blink */
	if (xTaskCreate(task_oled2, "Led", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to make led blink */
	if (xTaskCreate(task_oled3, "Led", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to button */
	if (xTaskCreate(task_but, "Botão", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}

/*******************************************************************************
*                                                                              *
*    main.c                                                                    *
*                                                                              *
*******************************************************************************/

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"
#include "semphr.h"

/* Priorities at which the tasks are created. */
#define tskTEST_PRIORITY            ( tskIDLE_PRIORITY + 1 )

/* The tasks and other required resources. */
static void prvTestTask( void );

/*----------------------------------------------------------------------------*/

#if (defined(GCC))
int main( void )
#endif
{
	/* Initialize the system. */
	SystemInit();
	
	/* Create tasks and set priority for each. */
	TaskHandle_t xHandle = NULL;
	
	xTaskCreate( (TaskFunction_t)prvTestTask, /** Function that implements the task. */
	             "TestTask", /** Text name of the task. */
	             configMINIMAL_STACK_SIZE, /** Stack size in words, not bytes. */
	             NULL, /** No parameters passed into the task. */
	             tskTEST_PRIORITY, /** Priority at which the task is created. */
	             &xHandle ); /** Used to pass out the created task's handle. */
	
	/* Start the tasks and timer running. */
	vTaskStartScheduler();
	
	/* If all is well, the scheduler will now be running, and the following line will
	never be reached.  If the following line does execute, then there was insufficient
	FreeRTOS heap memory available for the idle and/or timer tasks to be created. */
	while(1);
}

/*----------------------------------------------------------------------------*/

static void prvTestTask( void )
{
	LPC_GPIO1->FIODIR |= (1 << 18);
	
	do{
		
		LPC_GPIO1->FIOSET = (1 << 18);
		
		vTaskDelay( 1000 * portTICK_PERIOD_MS ); /** Sleep 1s. */
		
		LPC_GPIO1->FIOCLR = (1 << 18);
		
		vTaskDelay( 1000 * portTICK_PERIOD_MS ); /** Sleep 1s. */
		
	}while(1);
}
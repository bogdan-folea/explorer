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
#define tskWORK_PRIORITY            ( tskIDLE_PRIORITY + 2 )

/* The tasks and other required resources. */
static void prvTestTask( void );
static void prvWorkTask( void );

/*----------------------------------------------------------------------------*/

#if (defined(GCC))
int main( void )
#endif
{
	/* Initialize the system. */
	SystemInit();
	
	/* Set up hardware. */
	SetupHardware();
	
	/* Create tasks and set priority for each. */
	TaskHandle_t xTestHandle = NULL;
	TaskHandle_t xWorkHandle = NULL;
	
	xTaskCreate( (TaskFunction_t)prvTestTask, /** Function that implements the task. */
	             "TestTask", /** Text name of the task. */
	             configMINIMAL_STACK_SIZE, /** Stack size in words, not bytes. */
	             NULL, /** No parameters passed into the task. */
	             tskTEST_PRIORITY, /** Priority at which the task is created. */
	             &xTestHandle ); /** Used to pass out the created task's handle. */
	
	xTaskCreate( (TaskFunction_t)prvWorkTask, /** Function that implements the task. */
	             "WorkTask", /** Text name of the task. */
	             configMINIMAL_STACK_SIZE, /** Stack size in words, not bytes. */
	             NULL, /** No parameters passed into the task. */
	             tskWORK_PRIORITY, /** Priority at which the task is created. */
	             &xWorkHandle ); /** Used to pass out the created task's handle. */
	
	/* Assign a tag to each task. */
	vTaskSetApplicationTaskTag( xTestHandle, ( void * ) 1 );
	vTaskSetApplicationTaskTag( xWorkHandle, ( void * ) 2 );
	
	/* Start the tasks and set the timer running. */
	vTaskStartScheduler();
	
	/* If all is well, the scheduler will now be running, and the following line will
	never be reached. If the following line does execute, then there was insufficient
	FreeRTOS heap memory available for the idle and/or timer tasks to be created. */
	while(1);
}

/*----------------------------------------------------------------------------*/

static void prvTestTask( void )
{
	while(1);
}

/*----------------------------------------------------------------------------*/

static void prvWorkTask( void )
{
	do{
		
		uint32_t i = 10000000;
		while(i--);
		
		vTaskDelay( 1000 * portTICK_PERIOD_MS ); /** Sleep 1s. */
		
	}while(1);
}
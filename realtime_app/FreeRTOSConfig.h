/*******************************************************************************
*                                                                              *
*  FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.               *
*  All rights reserved.                                                        *
*                                                                              *
*  This file is part of the FreeRTOS distribution.                             *
*                                                                              *
*  FreeRTOS is free software; you can redistribute it and/or modify it under   *
*  the terms of the GNU General Public License (version 2) as published by     *
*  the Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.  *
*                                                                              *
*  FreeRTOS is distributed in the hope that it will be useful, but WITHOUT     *
*  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
*  FITNESS FOR A PARTICULAR PURPOSE.  Full license text is available on the    *
*  following link: http://www.freertos.org/a00114.html                         *
*                                                                              *
*******************************************************************************/


#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*=============================================================================/
/ ------------- Application specific definitions, adjusted for --------------- /
/ ------------ particular hardware and application requirements. ------------- /
/=============================================================================*/
 
#include "LPC17xx.h"

#define configUSE_PREEMPTION                 1
#define configUSE_IDLE_HOOK                  0
#define configUSE_TICK_HOOK                  0
#define configCPU_CLOCK_HZ                   ( 96000000UL ) /* running at 96 MHz */
#define configTICK_RATE_HZ                   ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                 ( 5 )
#define configMINIMAL_STACK_SIZE             ( ( unsigned short ) 90 )
#define configTOTAL_HEAP_SIZE                ( ( size_t ) ( 8 * 1024 ) )
#define configMAX_TASK_NAME_LEN              ( 10 )
#define configUSE_TRACE_FACILITY             1
#define configUSE_16_BIT_TICKS               0
#define configIDLE_SHOULD_YIELD              1
#define configUSE_MUTEXES                    1
#define configQUEUE_REGISTRY_SIZE            8
#define configCHECK_FOR_STACK_OVERFLOW       0
#define configUSE_RECURSIVE_MUTEXES          1
#define configUSE_MALLOC_FAILED_HOOK         0
#define configUSE_APPLICATION_TASK_TAG       1
#define configUSE_COUNTING_SEMAPHORES        1
#define configSUPPORT_DYNAMIC_ALLOCATION     1
#define configUSE_STATS_FORMATTING_FUNCTIONS 0

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES            0
#define configMAX_CO_ROUTINE_PRIORITIES  ( 2 )

/* Software timer definitions. */
#define configUSE_TIMERS                 1
#define configTIMER_TASK_PRIORITY        ( 2 )
#define configTIMER_QUEUE_LENGTH         10
#define configTIMER_TASK_STACK_DEPTH     ( configMINIMAL_STACK_SIZE * 2 )

/* Included/excluded API functions. */
#define INCLUDE_vTaskPrioritySet           1
#define INCLUDE_uxTaskPriorityGet          1
#define INCLUDE_vTaskDelete                1
#define INCLUDE_vTaskCleanUpResources      1
#define INCLUDE_vTaskSuspend               1
#define INCLUDE_vTaskDelayUntil            1
#define INCLUDE_vTaskDelay                 1
#define INCLUDE_eTaskGetState              1
#define INCLUDE_xTimerPendFunctionCall     1
#define INCLUDE_vTaskSetApplicationTaskTag 1


/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
    #define configPRIO_BITS             __NVIC_PRIO_BITS
#else
    #define configPRIO_BITS             4  /* 15 priority levels */
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority" function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         0xf

/* The highest interrupt priority that can be used by any interrupt service routine that makes
calls to interrupt safe FreeRTOS API functions. Higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY         ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Normal assert() semantics without relying on the provision of an assert.h header file. */
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); while(1); }

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS standard names. */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
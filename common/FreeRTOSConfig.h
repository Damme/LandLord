/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.
	

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!
    
    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?                                      *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    
    http://www.FreeRTOS.org - Documentation, training, latest information, 
    license and contact details.
    
    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool.

    Real Time Engineers ltd license FreeRTOS to High Integrity Systems, who sell 
    the code with commercial support, indemnification, and middleware, under 
    the OpenRTOS brand: http://www.OpenRTOS.com.  High Integrity Systems also
    provide a safety engineered and independently SIL3 certified version under 
    the SafeRTOS brand: http://www.SafeRTOS.com.
*/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "common.h"

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE. 
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

/* The SWI is used by the scheduler. */
#define vPortYieldProcessor swi_handler


#define configUSE_PREEMPTION		1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_IDLE_HOOK			0
#define configUSE_TICK_HOOK			0
#define configCPU_CLOCK_HZ              ( ( unsigned long ) 120000000 )
#define configTICK_RATE_HZ			( ( portTickType ) 1000 )

#define configMINIMAL_STACK_SIZE		( ( unsigned short ) 80 )
#define configTOTAL_HEAP_SIZE			( ( size_t ) ( 4096 ) )
#define configMAX_TASK_NAME_LEN			( 12 )
#define configUSE_TRACE_FACILITY		1 /* This is set to one so each task is given a unique number, which is then used to generate the logic analyzer output. */
#define configUSE_TIMERS                1
#define configTIMER_TASK_PRIORITY       3
#define configTIMER_QUEUE_LENGTH        10
#define configTIMER_TASK_STACK_DEPTH    configMINIMAL_STACK_SIZE
#define configUSE_16_BIT_TICKS			0
#define configIDLE_SHOULD_YIELD			0
#define configUSE_CO_ROUTINES 			0
#define configUSE_MUTEXES				0
#define configUSE_RECURSIVE_MUTEXES		0
#define configCHECK_FOR_STACK_OVERFLOW	0

#define configMAX_PRIORITIES			( 8 )
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet			1
#define INCLUDE_uxTaskPriorityGet			1
#define INCLUDE_vTaskDelete					1
#define INCLUDE_vTaskCleanUpResources		0
#define INCLUDE_vTaskSuspend				1
#define INCLUDE_vTaskDelayUntil				1
#define INCLUDE_vTaskDelay					1
#define INCLUDE_xTaskGetCurrentTaskHandle	1
#define INCLUDE_uxTaskGetStackHighWaterMark     1

/* ******************************************
if( 0x0000001e  >= 0x00000010 )
    configASSERT( 0x00000000  >= 0x00000018 );
configASSERT( ( 0xfa050000 & 0x00000700 ) <= 0x00000000 );
*/
#ifdef __NVIC_PRIO_BITS
    #define configPRIO_BITS             __NVIC_PRIO_BITS
#else
    #define configPRIO_BITS             5        /* 32 priority levels */
#endif

#define configKERNEL_INTERRUPT_PRIORITY     0xff
/* Priority 5, or 160 as only the top three bits are implemented. */
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ( 3 << (8 - configPRIO_BITS) )


/* **********************************************
if( 0x0000001e  >= 0x00000010 )
    configASSERT( 0x00000000  >= 0x000000a0 );
configASSERT( ( 0xfa050000 & 0x00000700 ) <= 0x00000000 );*/
//#define configKERNEL_INTERRUPT_PRIORITY 		( 7 << 5 )	/* Priority 7, or 255 as only the top three bits are implemented.  This is the lowest priority. */
//#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( 5 << 5 )  /* Priority 5, or 160 as only the top three bits are implemented. */



#define configASSERT( x )           if( ( x ) == 0 ) vAssertCalled( __FILE__, __LINE__ )

#define vPortSVCHandler      SVC_Handler
#define xPortPendSVHandler   PendSV_Handler
#define xPortSysTickHandler  SysTick_Handler

// For runtime stats!
// https://www.freertos.org/rtos-run-time-stats.html
/*
#define configUSE_TRACE_FACILITY        1
#define configGENERATE_RUN_TIME_STATS   1
//configUSE_STATS_FORMATTING_FUNCTIONS 
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS 1
*/
/*extern void vConfigureTimerForRunTimeStats( void );
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() vConfigureTimerForRunTimeStats()
#define portGET_RUN_TIME_COUNTER_VALUE() LPC_TIM0->TC*/

/*
 * Use the Cortex-M3 optimisations, rather than the generic C implementation.
 */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#endif /* FREERTOS_CONFIG_H */ 
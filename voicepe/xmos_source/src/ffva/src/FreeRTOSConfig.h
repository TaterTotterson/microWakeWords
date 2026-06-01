// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Here is a good place to include header files that are required across
your application. */
#include "platform.h"

#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      100000000

#if ON_TILE(0)
#define configNUM_CORES                         8
#endif
#if ON_TILE(1)
#define configNUM_CORES                         8
#endif

#define configTICK_RATE_HZ                      1000
#define configMAX_PRIORITIES                    32
#define configRUN_MULTIPLE_PRIORITIES           1
#define configUSE_TASK_PREEMPTION_DISABLE       1
#define configUSE_CORE_AFFINITY                 1
#define configMINIMAL_STACK_SIZE                ( configSTACK_DEPTH_TYPE ) 256
#define configMAX_TASK_NAME_LEN                 32
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   2
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               10
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIME_SLICING                  0
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 0
#define configSTACK_DEPTH_TYPE                  uint32_t
#define configMESSAGE_BUFFER_LENGTH_TYPE        size_t

/* Memory allocation related definitions. */
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#if ON_TILE(0)
#define configTOTAL_HEAP_SIZE                   180*1024
#endif
#if ON_TILE(1)
#define configTOTAL_HEAP_SIZE                   128*1024
#endif
#define configAPPLICATION_ALLOCATED_HEAP        0

/* Hook function related definitions. */
#define configUSE_IDLE_HOOK                     0
#define configUSE_MINIMAL_IDLE_HOOK             1
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          0
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0
#define configUSE_CORE_INIT_HOOK                0

/* Run time and task stats gathering related definitions. */
#define configGENERATE_RUN_TIME_STATS           0
#if ENABLE_RTOS_XSCOPE_TRACE
#define configUSE_TRACE_FACILITY                1
#else
#define configUSE_TRACE_FACILITY                0
#endif
#define configUSE_STATS_FORMATTING_FUNCTIONS    2 /* Setting to 2 does not include <stdio.h> in tasks.c */

/* Co-routine related definitions. */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         1

/* Software timer related definitions. */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE << 2 )

/* Define to trap errors during development. */
#define configASSERT(x) xassert(x)

/* Define to enable debug_printf() */
#define configENABLE_DEBUG_PRINTF 1

/* Define to map sprintf and snprintf to the
 * lite versions in lib_rtos_support */
 #include <stdio.h>
#define configUSE_DEBUG_SPRINTF 1

/* Define to enable debug prints from tasks.c */
#if ON_TILE(0)
#define configTASKS_DEBUG 0
#endif
#if ON_TILE(1)
#define configTASKS_DEBUG 0
#endif

/* FreeRTOS MPU specific definitions. */
#define configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS 0

/* Optional functions - most linkers will remove unused functions anyway. */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xResumeFromISR                  1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xEventGroupSetBitFromISR        1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_xTaskResumeFromISR              1
#define INCLUDE_xQueueGetMutexHolder            1

/* A header file that defines trace macro can be included here. */
#if ENABLE_RTOS_XSCOPE_TRACE
#include "xcore_trace.h"
#endif

#endif /* FREERTOS_CONFIG_H */

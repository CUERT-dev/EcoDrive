/* ---- Required hooks for the selected FreeRTOS POSIX configuration ---- */
#include "platform.h"
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

void vAssertCalled(const char *pcFile, unsigned long ulLine)
{
    fprintf(stderr, "FreeRTOS assert: %s:%lu\n", pcFile, ulLine);
    fflush(stderr);
    abort();
}

void vApplicationMallocFailedHook(void)
{
    fprintf(stderr, "FreeRTOS malloc failed\n");
    fflush(stderr);
    abort();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    fprintf(stderr, "FreeRTOS stack overflow in task: %s\n", pcTaskName ? pcTaskName : "(unknown)");
    fflush(stderr);
    abort();
}

void vApplicationIdleHook(void)
{
}

void vApplicationTickHook(void)
{
}

void vApplicationDaemonTaskStartupHook(void)
{
}

#if (configSUPPORT_STATIC_ALLOCATION == 1)
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   StackType_t *pulIdleTaskStackSize)
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    StackType_t *pulTimerTaskStackSize)
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif
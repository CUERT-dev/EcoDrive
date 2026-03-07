#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

static volatile unsigned long g_fast_count = 0;
static volatile unsigned long g_slow_count = 0;

static void vFastTask(void *arg)
{
    (void)arg;
    for (;;)
    {
        g_fast_count++;
        printf("[fast] tick=%lu count=%lu\n", (unsigned long)xTaskGetTickCount(), g_fast_count);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void vSlowTask(void *arg)
{
    (void)arg;
    for (;;)
    {
        g_slow_count++;
        printf("[slow] tick=%lu count=%lu\n", (unsigned long)xTaskGetTickCount(), g_slow_count);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

static void vMonitorTask(void *arg)
{
    (void)arg;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        printf("[monitor] fast=%lu slow=%lu\n", g_fast_count, g_slow_count);

        if ((g_fast_count >= 100UL) && (g_slow_count >= 50UL))
        {
            printf("[monitor] smoke test passed, exiting.\n");
            fflush(stdout);
            exit(0);
        }
    }
}

int main(void)
{
    printf("FreeRTOS POSIX smoke test start\n");

    (void)xTaskCreate(vFastTask, "fast", configMINIMAL_STACK_SIZE * 2U, NULL, tskIDLE_PRIORITY + 2U, NULL);
    (void)xTaskCreate(vSlowTask, "slow", configMINIMAL_STACK_SIZE * 2U, NULL, tskIDLE_PRIORITY + 1U, NULL);
    (void)xTaskCreate(vMonitorTask, "mon", configMINIMAL_STACK_SIZE * 2U, NULL, tskIDLE_PRIORITY + 3U, NULL);

    vTaskStartScheduler();

    printf("Scheduler returned unexpectedly.\n");
    return 1;
}

/* ---- Required hooks for the selected FreeRTOS POSIX configuration ---- */

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
